/* ------------------------------------------------ *
 * Title       : Simple I2C interface v1.1          *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : i2c.v                              *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 28/01/2021                         *
 * ------------------------------------------------ *
 * Description : I2C slave and master modules       *
 * ------------------------------------------------ *
 * Revisions                                        *
 *     v1      : Inital version for working master  *
 *     v1.1    : Master samples ack while SCL high, *
 *               Master end transaction when slave  *
 *               gave NACK to write acknowledgment  * 
 * ------------------------------------------------ */

module i2c_master(
  input clk,
  input rst,
  //Config & Control
  input [1:0] freqSLCT, //3.125MHz,781.25kHz,390.625kHz,97.656kHz
  output busy,
  output newData, //New data avaible 
  output dataReq, //when high enter new data
  input data_valid, //Data in valid
  input start, //begin transaction
  input [1:0] data_byte_size,
  input read_nwrite, //1: read; 0: write
  //Data interface
  input [6:0] addr, //I2C address for slave
  input [7:0] data_i, //Data in
  output reg [7:0] data_o, //Data Out
  //I2C pins
  output SCL/* synthesis keep = 1 */,
  inout SDA/* synthesis keep = 1 */);
  localparam READY = 3'b000,
             START = 3'b001,
             ADDRS = 3'b011,
             WRITE = 3'b110,
         WRITE_ACK = 3'b010,
              READ = 3'b111,
          READ_ACK = 3'b101,
              STOP = 3'b100;
  reg [7:0] data_i_buff, data_o_buff;
  wire i2c_clk; //Used to shifting and sampling
  reg i2c_clk_half; //Low: Shift High: Sample
  wire SDA_Write;
  wire SDA_Claim;
  reg SDA_d;
  wire counterDONE;
  reg en;
  reg [2:0] counter;
  reg [1:0] last_byte;
  reg [2:0] state;
  wire moreBytes;
  reg givingADDRS;
  wire in_READY, in_START, in_ADDRS, in_WRITE, in_WRITE_ACK, in_READ, in_READ_ACK, in_STOP;
  wire in_ACK;

  //Decode states
  assign in_READY = (state == READY);
  assign in_START = (state == START);
  assign in_ADDRS = (state == ADDRS);
  assign in_WRITE = (state == WRITE);
  assign in_WRITE_ACK = (state == WRITE_ACK);
  assign in_READ = (state == READ);
  assign in_READ_ACK = (state == READ_ACK);
  assign in_STOP = (state == STOP);
  assign busy = ~in_READY;
  assign newData = in_READ_ACK;
  assign in_ACK = in_READ_ACK | in_WRITE_ACK;

  assign SCL = (in_READY) ? 1'b1 : i2c_clk_half;

  assign dataReq = ~data_valid & ((moreBytes & in_WRITE) | in_ADDRS);
  
  //Data line handling
  assign SDA = (SDA_Claim) ? SDA_Write : 1'bZ;
  assign SDA_Claim = in_START | in_ADDRS | in_WRITE | in_READ_ACK | in_STOP;
  assign SDA_Write = (in_READ_ACK | in_START | in_STOP) ? (in_READ_ACK & (~moreBytes)) : data_i_buff[7];
  always@(negedge i2c_clk)
    begin
      SDA_d <= SDA;
    end
  

  //Count bytes
  assign moreBytes = (data_byte_size != last_byte);
  always@(negedge in_ACK or posedge in_START)
    begin
      if(in_START)
        begin
          last_byte <= 2'b11;
        end
      else
        begin
          last_byte <= last_byte + 2'b1; 
        end
    end
  always@(posedge clk)
    begin
      case(state)
        START:
          begin
            givingADDRS <= 1'b1;
          end
        WRITE:
          begin
            givingADDRS <= 1'b0;
          end
        READ:
          begin
            givingADDRS <= 1'b0;
          end
        default:
          begin
            givingADDRS <= givingADDRS;
          end
      endcase
    end
  
  //Internal enable
  always@(posedge clk or posedge rst)
    begin
      if(rst)
        begin
          en <= 0;
        end
      else
        begin
          case(en)
            0:
              begin
                en <= in_READY & start;
              end
            1:
              begin
                en <= in_READY;
              end
          endcase
          
        end
    end

  //Counter
  assign counterDONE = ~|counter;
  always@(negedge i2c_clk_half) 
    begin
      case(state)
        ADDRS:
          begin
            counter <= counter + 3'd1;
          end
        WRITE:
          begin
            counter <= counter + 3'd1;
          end
        READ:
          begin
            counter <= counter + 3'd1;
          end
        default:
          begin
            counter <= 3'd0;
          end
      endcase
    end

  //State transactions
  always@(negedge i2c_clk or posedge rst)
    begin
      if(rst)
        begin
          state <= READY;
        end
      else
        begin
          case(state)
            READY:
              begin
                state <= (en & i2c_clk_half) ? START : state;
              end
            START:
              begin
                state <= (~SCL) ? ADDRS : state;
              end
            ADDRS:
              begin
                state <= (~SCL & counterDONE) ? WRITE_ACK : state;
              end
            WRITE_ACK:
              begin
                state <= (~SCL) ? ((~SDA_d & (moreBytes | givingADDRS)) ? ((~read_nwrite) ? ((data_valid) ? WRITE : state) : READ): STOP) : state;
              end
            WRITE:
              begin
                state <= (~SCL & counterDONE) ? WRITE_ACK : state;
              end
            READ:
              begin
                state <= (~SCL & counterDONE) ? READ_ACK : state;
              end
            READ_ACK:
              begin
                state <= (~SCL) ? ((moreBytes) ? READ : STOP) : state;
              end
            STOP:
              begin
                state <= (SCL) ? READY : state;
              end
          endcase
        end
    end

  //Handle data in buffer
  always@(negedge i2c_clk)
    begin
      case(state)
        START: //At start load address and op
          begin
            data_i_buff <= {addr, read_nwrite};
          end
        ADDRS: //During address shift
          begin
            data_i_buff <= (SCL) ? data_i_buff : (data_i_buff << 1);
          end
        WRITE_ACK: //Load new data during ack
          begin
            data_i_buff <= data_i;
          end
        WRITE: //During write shift
          begin
            data_i_buff <= (SCL) ? data_i_buff : (data_i_buff << 1);
          end
        default:
          begin
            data_i_buff <= data_i_buff;
          end
      endcase
    end

  //Store output during read ack
  always@(posedge i2c_clk)
    begin
      data_o <= (in_READ_ACK) ? data_o_buff : data_o;
    end
  
  //Handle data out buffer
  always@(posedge i2c_clk)
    begin
      data_o_buff <= (SCL & in_READ) ? {data_o_buff[7:0], SDA} : data_o_buff;
    end
  
  //Divide i2c_clk
  always@(posedge i2c_clk or posedge rst)
    begin
      if(rst)
        begin
          i2c_clk_half <= 1;
        end
      else
        begin
          i2c_clk_half <= ~i2c_clk_half;
        end
    end
  clockGen_i2c sdaGEN(clk, rst, freqSLCT, i2c_clk);
endmodule//i2c_master

module i2c_slave( //TODO: Fix
  //output [5:0] debug,
  input clk,
  input rst,
  //Config & Control
  output busy,
  output newData, //New data avaible 
  output dataReq,
  //Data interface
  input [6:0] addr, //I2C address for slave
  input [7:0] data_i, //Data in
  output reg [7:0] data_o, //Data Out
  //I2C pins
  (* clock_buffer_type="none" *) input SCL,
  inout SDA/* synthesis keep = 1 */);
  localparam  IDLE = 3'b000,
             ADDRS = 3'b001,
         ADDRS_ACK = 3'b011,
             WRITE = 3'b110,
         WRITE_ACK = 3'b010,
              READ = 3'b111,
          READ_ACK = 3'b101,
         WAIT_STOP = 3'b100;
  wire in_IDLE, in_ADDRS_ACK, in_ADDRS, in_WRITE, in_WRITE_ACK, in_READ, in_READ_ACK, in_STOP;
  wire SDA_Write;
  wire SDA_Claim;
  reg [7:0] data_i_buff, data_o_buff;
  reg SDA_d, SCL_d, in_READ_d;
  reg [2:0] state;
  wire SDA_negedge, SDA_posedge, SCL_negedge/*, SCL_posedge*/;
  wire in_READ_pulse;
  reg [2:0] counter;
  wire counterDONE;
  wire addrsed;
  wire startCond, stopCond;

  //assign debug = {counter,state};

  //I2C signal edges into system clock domain
  always@(posedge clk)
    begin
      SDA_d <= SDA;
      SCL_d <= SCL;
      in_READ_d <= in_READ;
    end
  assign SDA_negedge = SDA_d & ~SDA;
  assign SDA_posedge = ~SDA_d & SDA;
  assign SCL_negedge = SCL_d & ~SCL;
  //assign SCL_posedge = ~SCL_d & SCL;
  assign in_READ_pulse = ~in_READ_d & in_READ;

  //Conditions
  assign startCond = SCL & SDA_negedge;
  assign stopCond = SCL & SDA_posedge;

  //Decode states
  assign in_IDLE = (state == IDLE);
  assign in_ADDRS_ACK = (state == ADDRS_ACK);
  assign in_ADDRS = (state == ADDRS);
  assign in_WRITE = (state == WRITE);
  assign in_WRITE_ACK = (state == WRITE_ACK);
  assign in_READ = (state == READ);
  assign in_READ_ACK = (state == READ_ACK);
  assign newData = in_WRITE_ACK;
  assign addrsed = (addr == data_i_buff[7:1]);
  assign dataReq = in_ADDRS_ACK | (in_READ_ACK & ~SDA);

  //Data line handling
  assign SDA = (SDA_Claim) ? SDA_Write : 1'bZ;
  assign SDA_Claim = in_READ | (in_ADDRS_ACK & addrsed) | in_WRITE_ACK;
  assign SDA_Write = (in_READ) ? data_i_buff[7] : 1'b0;

  //State transactions
  always@(posedge clk)
    begin
      if(rst)
        begin
          state <= IDLE;
        end
      else
        begin
          case(state)
            IDLE:
              begin
                state <= (startCond) ? ADDRS : state;
              end
            ADDRS:
              begin
                state <= (SCL_negedge & counterDONE) ? ADDRS_ACK : state;
              end
            ADDRS_ACK:
              begin
                state <= (addrsed) ? ((data_i_buff[0]) ? READ : WRITE): IDLE;
              end
            WRITE:
              begin
                state <= (stopCond) ? IDLE : ((SCL_negedge & counterDONE) ? WRITE_ACK : state);
              end
            WRITE_ACK:
              begin
                state <= (SCL_negedge) ? WRITE : state;
              end
            READ:
              begin
                state <= (SCL_negedge & counterDONE) ? READ_ACK : state;
              end
            READ_ACK:
              begin
                state <= (SDA_posedge) ? ((SDA) ? WAIT_STOP : READ ) : state;
              end
            WAIT_STOP: //redundant
              begin
                state <= (stopCond) ? IDLE : state;
              end
          endcase     
        end
    end
  
  //Counter
  assign counterDONE = ~|counter;
  always@(negedge SCL) 
    begin
      case(state)
        ADDRS:
          begin
            counter <= counter + 3'd1;
          end
        WRITE:
          begin
            counter <= counter + 3'd1;
          end
        READ:
          begin
            counter <= counter + 3'd1;
          end
        default:
          begin
            counter <= 3'd0;
          end
      endcase
      
    end

  //data in buffer
  always@(negedge SCL_d or posedge in_READ_pulse)
    begin
      if(in_READ_pulse)
        begin
          data_i_buff <= data_i;
        end
      else
        begin
          data_i_buff <= (in_READ) ? (data_i_buff << 1) : data_i_buff;
        end
    end
  
  //Store buffer data during ack
  always@(posedge in_WRITE_ACK or posedge rst)
    begin
      if(rst)
        begin
          data_o <= 7'd0;
        end
      else
        begin
          data_o <= data_o_buff;
        end
    end
  
  //data out buffer
  always@(negedge SCL)
    begin
      data_o_buff <= (in_ADDRS | in_WRITE) ? {data_o_buff[6:0], SDA} : data_o_buff;
    end 
endmodule//i2c_slave

//freqSLCT:2x(3.125MHz,781.25kHz,390.625kHz,97.656kHz)
//Following module will generate correct frequencies only for 100 MHz clk_i
module clockGen_i2c(
  input clk_i,
  input rst,
  input [1:0] freqSLCT,
  output clk_o);

  wire [3:0] clk_array; //2x(3.125MHz,781.25kHz,390.625kHz,97.656kHz)
  reg [8:0] clk_d;

  assign clk_o = clk_array[freqSLCT];
  assign clk_array = {clk_d[3],clk_d[5],clk_d[6],clk_d[8]};

  //50MHz
  always@(posedge clk_i or posedge rst)
    begin
      if(rst)
        begin
          clk_d[0] <= 0;
        end
      else
        begin
          clk_d[0] <= ~clk_d[0];
        end
    end
  //25MHz
  always@(posedge clk_d[0] or posedge rst)
    begin
      if(rst)
        begin
          clk_d[1] <= 0;
        end
      else
        begin
          clk_d[1] <= ~clk_d[1];
        end
    end
  //12.5MHz
  always@(posedge clk_d[1] or posedge rst)
    begin
      if(rst)
        begin
          clk_d[2] <= 0;
        end
      else
        begin
          clk_d[2] <= ~clk_d[2];
        end
    end
  //6.25MHz
  always@(posedge clk_d[2] or posedge rst)
    begin
      if(rst)
        begin
          clk_d[3] <= 0;
        end
      else
        begin
          clk_d[3] <= ~clk_d[3];
        end
    end
  //3.125MHz
  always@(posedge clk_d[3] or posedge rst)
    begin
      if(rst)
        begin
          clk_d[4] <= 0;
        end
      else
        begin
          clk_d[4] <= ~clk_d[4];
        end
    end
  //1.562MHz
  always@(posedge clk_d[4] or posedge rst)
    begin
      if(rst)
        begin
          clk_d[5] <= 0;
        end
      else
        begin
          clk_d[5] <= ~clk_d[5];
        end
    end
  //781.25kHz
  always@(posedge clk_d[5] or posedge rst)
    begin
      if(rst)
        begin
          clk_d[6] <= 0;
        end
      else
        begin
          clk_d[6] <= ~clk_d[6];
        end
    end
  //390.625kHz
  always@(posedge clk_d[6] or posedge rst)
    begin
      if(rst)
        begin
          clk_d[7] <= 0;
        end
      else
        begin
          clk_d[7] <= ~clk_d[7];
        end
    end
  //195.312kHz
  always@(posedge clk_d[7] or posedge rst)
    begin
      if(rst)
        begin
          clk_d[8] <= 0;
        end
      else
        begin
          clk_d[8] <= ~clk_d[8];
        end
    end
endmodule
