/* ------------------------------------------------ *
 * Title       : Simple I2C interface v1.3          *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : i2c_masterv1.v                     *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 19/09/2022                         *
 * Licence     : CERN-OHL-W                         *
 * ------------------------------------------------ *
 * Description : I2C master modules v1.3            *
 * ------------------------------------------------ *
 * Revisions                                        *
 *     v1      : Inital version for working master  *
 *     v1.1    : Master samples ack while SCL high, *
 *               Master end transaction when slave  *
 *               gave NACK to write acknowledgment  * 
 *     v1.2    : Multi master support               *
 *     v1.3    : Change coding style                *
 * ------------------------------------------------ */

module i2c_masterv1(
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
  inout SCL/* synthesis keep = 1 */,
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
  reg SDA_d;
  reg en;
  reg [2:0] counter;
  reg [1:0] last_byte;
  reg [2:0] state;
  wire moreBytes;
  reg givingADDRS;
  reg SDA_dSys;
  reg I2C_busy;

  //I2C signal edges into system clock domain
  always@(posedge clk)
    begin
      SDA_dSys <= SDA;
    end
  wire SDA_negedge = SDA_dSys & ~SDA;
  wire SDA_posedge = ~SDA_dSys & SDA;

  //Decode states
  wire in_READY = (state == READY);
  wire in_START = (state == START);
  wire in_ADDRS = (state == ADDRS);
  wire in_WRITE = (state == WRITE);
  wire in_WRITE_ACK = (state == WRITE_ACK);
  wire in_READ = (state == READ);
  wire in_READ_ACK = (state == READ_ACK);
  wire in_STOP = (state == STOP);
  wire in_ACK = in_READ_ACK | in_WRITE_ACK;
  wire SCL_Claim = ~in_READY;
  assign busy = ~in_READY;
  assign newData = in_READ_ACK;

  assign SCL = (SCL_Claim) ? i2c_clk_half : 1'bZ;

  assign dataReq = ~data_valid & ((moreBytes & in_WRITE) | in_ADDRS);

  //Conditions
  wire startCond = SCL & SDA_negedge;
  wire stopCond = SCL & SDA_posedge;

  always@(posedge clk or posedge rst) begin
    if(rst)
      I2C_busy <= 1'b0;
    else case(I2C_busy)
      1'b0: I2C_busy <= (in_READY) ? startCond : I2C_busy;
      1'b1: I2C_busy <= (in_READY) ? ~stopCond : I2C_busy;
    endcase      
  end
  
  //Data line handling
  wire SDA_Write = (in_READ_ACK | in_START | in_STOP) ? (in_READ_ACK & (~moreBytes)) : data_i_buff[7];
  wire SDA_Claim = in_START | in_ADDRS | in_WRITE | in_READ_ACK | in_STOP;
  assign SDA = (SDA_Claim) ? SDA_Write : 1'bZ;
  always@(negedge i2c_clk) begin
    SDA_d <= SDA;
  end
  

  //Count bytes
  assign moreBytes = (data_byte_size != last_byte);
  always@(negedge in_ACK or posedge in_START) begin
    if(in_START) begin
      last_byte <= 2'b11;
    end else begin
      last_byte <= last_byte + 2'b1; 
    end
  end
  always@(posedge clk) begin
    case(state)
        START: givingADDRS <= 1'b1;
        WRITE: givingADDRS <= 1'b0;
         READ: givingADDRS <= 1'b0;
      default: givingADDRS <= givingADDRS;
    endcase
  end
  
  //Internal enable
  always@(posedge clk or posedge rst) begin
    if(rst) begin
      en <= 0;
    end else case(en)
      0: en <= in_READY & start;
      1: en <= in_READY;
    endcase
  end

  //Counter
  wire counterDONE = ~|counter;
  always@(negedge i2c_clk_half)  begin
    case(state)
        ADDRS: counter <= counter + 3'd1;
        WRITE: counter <= counter + 3'd1;
         READ: counter <= counter + 3'd1;
      default: counter <= 3'd0;
    endcase
  end

  //State transactions
  always@(negedge i2c_clk or posedge rst) begin
    if(rst) begin
      state <= READY;
    end else case(state)
      READY: begin
        state <= (~I2C_busy & en & i2c_clk_half) ? START : state;
      end 
      START: begin
        state <= (~SCL) ? ADDRS : state;
      end
      ADDRS: begin
        state <= (~SCL & counterDONE) ? WRITE_ACK : state;
      end
      WRITE_ACK: begin
        state <= (~SCL) ? ((~SDA_d & (moreBytes | givingADDRS)) ? ((~read_nwrite) ? ((data_valid) ? WRITE : state) : READ): STOP) : state;
      end
      WRITE: begin
        state <= (~SCL & counterDONE) ? WRITE_ACK : state;
      end
      READ: begin
        state <= (~SCL & counterDONE) ? READ_ACK : state;
      end
      READ_ACK: begin
        state <= (~SCL) ? ((moreBytes) ? READ : STOP) : state;
      end
      STOP: begin
        state <= (SCL) ? READY : state;
      end
    endcase
  end

  //Handle data in buffer
  always@(negedge i2c_clk) begin
    case(state)
      START: begin //At start load address and op
        data_i_buff <= {addr, read_nwrite};
      end
      ADDRS: begin //During address shift
        data_i_buff <= (SCL) ? data_i_buff : (data_i_buff << 1);
      end
      WRITE_ACK: begin //Load new data during ack
        data_i_buff <= data_i;
      end
      WRITE: begin //During write shift
        data_i_buff <= (SCL) ? data_i_buff : (data_i_buff << 1);
      end
      default: begin
        data_i_buff <= data_i_buff;
      end
    endcase
  end

  //Store output during read ack
  always@(posedge i2c_clk) begin
    data_o <= (in_READ_ACK) ? data_o_buff : data_o;
  end
  
  //Handle data out buffer
  always@(posedge i2c_clk) begin
    data_o_buff <= (SCL & in_READ) ? {data_o_buff[7:0], SDA} : data_o_buff;
  end
  
  //Divide i2c_clk
  always@(posedge i2c_clk or posedge rst) begin
  if(rst) begin
        i2c_clk_half <= 1;
      end else begin
        i2c_clk_half <= ~i2c_clk_half;
      end
  end

  clockGen_i2cv1 sdaGEN(clk, rst, freqSLCT, i2c_clk);
endmodule//i2c_master

//freqSLCT:2x(3.125MHz,781.25kHz,390.625kHz,97.656kHz)
//Following module will generate correct frequencies only for 100 MHz clk_i
module clockGen_i2cv1(
  input clk_i,
  input rst,
  input [1:0] freqSLCT,
  output clk_o);

  wire [3:0] clk_array; //2x(3.125MHz,781.25kHz,390.625kHz,97.656kHz)
  reg [8:0] clk_d;

  assign clk_o = clk_array[freqSLCT];
  assign clk_array = {clk_d[3],clk_d[5],clk_d[6],clk_d[8]};

  //50MHz
  always@(posedge clk_i or posedge rst) begin
    if(rst) begin
        clk_d[0] <= 0;
      end else begin
        clk_d[0] <= ~clk_d[0];
      end
    end
  //25MHz
  always@(posedge clk_d[0] or posedge rst) begin
    if(rst) begin
      clk_d[1] <= 0;
    end else begin
      clk_d[1] <= ~clk_d[1];
    end
  end
  //12.5MHz
  always@(posedge clk_d[1] or posedge rst) begin
    if(rst) begin
      clk_d[2] <= 0;
    end else begin
      clk_d[2] <= ~clk_d[2];
    end
  end
  //6.25MHz
  always@(posedge clk_d[2] or posedge rst) begin
    if(rst) begin
      clk_d[3] <= 0;
    end else begin
      clk_d[3] <= ~clk_d[3];
    end
  end
  //3.125MHz
  always@(posedge clk_d[3] or posedge rst) begin
    if(rst) begin
      clk_d[4] <= 0;
    end else begin
      clk_d[4] <= ~clk_d[4];
    end
  end
  //1.562MHz
  always@(posedge clk_d[4] or posedge rst) begin
    if(rst) begin
      clk_d[5] <= 0;
    end else begin
      clk_d[5] <= ~clk_d[5];
    end
  end
  //781.25kHz
  always@(posedge clk_d[5] or posedge rst) begin
    if(rst) begin
      clk_d[6] <= 0;
    end else begin
      clk_d[6] <= ~clk_d[6];
    end
  end
  //390.625kHz
  always@(posedge clk_d[6] or posedge rst) begin
    if(rst) begin
      clk_d[7] <= 0;
    end else begin
      clk_d[7] <= ~clk_d[7];
    end
  end
  //195.312kHz
  always@(posedge clk_d[7] or posedge rst) begin
    if(rst) begin
      clk_d[8] <= 0;
    end else begin
      clk_d[8] <= ~clk_d[8];
    end
  end
endmodule
