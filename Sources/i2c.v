/* ------------------------------------------------ *
 * Title       : Simple I2C interface v2.1          *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : i2c.v                              *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 02/05/2021                         *
 * ------------------------------------------------ *
 * Description : I2C slave and master modules       *
 * ------------------------------------------------ *
 * Revisions                                        *
 *     v2      : Changed master algorithm           *
 *     v2.1    : Async reset for master bit counter *
 *               and working slave                  *
 * ------------------------------------------------ */

module i2c_master(
  input clk,
  input rst,
  input clkI2Cx2,
  //Control
  input start, //begin transaction
  output ready,
  output reg i2cBusy,
  output data_available, //New data avaible 
  output data_request, //when high enter new data
  input data_valid, //Data in valid
  //Addressing and read/write
  input read_nwrite, //1: read; 0: write
  input [6:0] addr, //I2C address for slave
  //Data interface
  input [7:0] data_i, //Data in
  output [7:0] data_o, //Data Out
  input [2:0] data_size, //!Max 7 Bytes
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
  //state & state control
  reg [2:0] state;
  wire inReady, inStart, inAddrs, inWrite, inWriteAck, inRead, inReadAck, inStop, inAck;
  //data control
  wire [7:0] addressByte;
  wire SDAupdate, SDAshift;
  //Transmisson counters
  reg [2:0] byteCounter;
  reg [2:0] bitCounter;
  wire byteCountDone;
  wire bitCountDone;
  wire byteCountUp;
  wire bitCounterReset;
  //Generate I2C signals with tri-state
  reg SCLK; //Internal I2C clock, always thicks
  wire SCL_claim;
  wire SDA_claim;
  wire SDA_write;
  //delayed signals
  reg inAck_d, SDA_d;
  reg SDA_d_i2c;
  //Buffers
  reg [7:0] send_buffer, receive_buffer;
  //Internal control signals
  wire startCondition, stopCondition; //I2C conditions
  wire SDA_negedge, SDA_posedge;

  assign addressByte = {addr,read_nwrite}; //Get address byte

  //Data flags
  assign data_available = inReadAck;
  assign in_last_byte = (data_size - 3'd1) == byteCounter;
  assign data_request = (~read_nwrite & inAddrs) | (~in_last_byte & inWrite);

  //decode states
  assign      ready = inReady;
  assign     inRead = (state == READ);
  assign     inStop = (state == STOP);
  assign    inReady = (state == READY);
  assign    inStart = (state == START);
  assign    inAddrs = (state == ADDRS);
  assign    inWrite = (state == WRITE);
  assign  inReadAck = (state == READ_ACK);
  assign inWriteAck = (state == WRITE_ACK);
  assign      inAck = inWriteAck | inReadAck;

  //Tri-state control for I2C lines
  assign SCL = (SCL_claim) ?    SCLK   : 1'bZ;
  assign SDA = (SDA_claim) ? SDA_write : 1'bZ;
  assign SCL_claim = ~inReady;
  assign SDA_claim = inStart | inAddrs | inWrite | inReadAck | inStop;
  assign SDA_write = (inStart | inReadAck | inStop) ? (inReadAck & byteCountDone) : send_buffer[7];

  //Listen I2C Bus & cond. gen.
  assign    SDA_negedge  = ~SDA &  SDA_d;
  assign    SDA_posedge  =  SDA & ~SDA_d;
  assign startCondition  =  SCL & SDA_negedge;
  assign  stopCondition  =  SCL & SDA_posedge;

  //delay signals
  always@(posedge clk)
    begin
      inAck_d <= inAck;
      SDA_d <= SDA;
    end
  always@(negedge clkI2Cx2)
    begin
      SDA_d_i2c <= SDA;
    end
  
  //Determine if an other master is using the bus
  always@(posedge clk or posedge rst)
    begin
      if(rst)
        begin
          i2cBusy <= 1'b0;
        end
      else
        begin
          case(i2cBusy)
            1'b0:
              begin
                i2cBusy <= startCondition & inReady;
              end
            1'b1:
              begin
                i2cBusy <= ~stopCondition & inReady;
              end
          endcase
        end
    end
  
  //receive buffer
  assign data_o = receive_buffer;
  always@(posedge clkI2Cx2)
    begin
      if(inRead & SCL)
        begin
          receive_buffer <= {receive_buffer[6:0],SDA};
        end
    end

  //send buffer
  always@(negedge clkI2Cx2)
    begin
      if(SDAupdate)
        send_buffer <= (inStart) ? addressByte : data_i;
      else if(SDAshift & ~SCL & |bitCounter)
        send_buffer <= {send_buffer << 1};
    end
  
  //SDA control states for sending data
  assign SDAupdate = inStart | inWriteAck;
  assign SDAshift  = inAddrs | inWrite;

  //state transactions
  always@(negedge clkI2Cx2 or posedge rst)
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
                state <= (start & SCLK & ~i2cBusy) ? START : state;
              end
            START:
              begin
                state <= (~SCL) ? ADDRS : state;
              end
            ADDRS:
              begin
                state <= (~SCL & bitCountDone) ? WRITE_ACK : state;
              end
            WRITE_ACK:
              begin
                state <= (~SCL) ? ((~SDA_d_i2c & ~byteCountDone) ? ((~read_nwrite) ? ((data_valid) ? WRITE : state) : READ): STOP) : state;
              end
            WRITE:
              begin
                state <= (~SCL & bitCountDone) ? WRITE_ACK : state;
              end
            READ:
              begin
                state <= (~SCL & bitCountDone) ? READ_ACK : state;
              end
            READ_ACK:
              begin
                state <= (~SCL) ? ((byteCountDone) ? STOP : READ) : state;
              end
            STOP:
              begin
                state <= (SCL) ? READY : state;
              end
          endcase
        end
    end

  //bit counter
  assign bitCountDone = ~|bitCounter;
  assign bitCounterReset = inAck|inStart;
  always@(posedge SCL or posedge bitCounterReset) 
    begin
      if(bitCounterReset)
        begin
          bitCounter <= 3'd0;
        end
      else
        begin
          bitCounter <= bitCounter + {2'd0,(inAddrs|inWrite|inRead)};
        end
    end

  //byte counter
  assign byteCountUp = ~inAck_d & inAck;
  assign byteCountDone = (byteCounter == data_size);
  always@(posedge clk) 
    begin//another alternative would be using inAck as clock
      if(inStart)
        begin
          byteCounter <= 3'b111;
        end
      else
        begin
          byteCounter <= byteCounter + {2'd0,byteCountUp};
        end
    end

  //Divide clkI2Cx2 to get I2C clk
  always@(posedge clkI2Cx2 or posedge rst)
    begin
      if(rst)
        begin
          SCLK <= 1'b1;
        end
      else
        begin
          SCLK <= ~SCLK;
        end
    end
endmodule//i2c_master

module i2c_slave(
  input clk,
  input rst,
  //Config & Control
  output busy,
  output data_available, //New data avaible 
  output data_request,
  //Data interface
  input [6:0] addr, //I2C address for slave
  input [7:0] data_i, //Data in
  output reg [7:0] data_o, //Data Out
  //I2C pins
  input SCL,
  inout SDA/* synthesis keep = 1 */);
  localparam  IDLE = 3'b000,
             ADDRS = 3'b001,
         ADDRS_ACK = 3'b011,
             WRITE = 3'b110,
         WRITE_ACK = 3'b010,
              READ = 3'b111,
          READ_ACK = 3'b101,
         WAIT_STOP = 3'b100;
  wire in_IDLE, in_ADDRS_ACK, in_ADDRS, in_WRITE, in_WRITE_ACK, in_READ, in_READ_ACK, in_STOP, in_Get_Data;
  wire SDA_Write;
  wire SDA_Claim;
  reg [7:0] send_buffer, receive_buffer;
  reg SDA_d, SCL_d, in_READ_d;
  reg [2:0] state;
  wire SDA_negedge, SDA_posedge, SCL_negedge, SCL_posedge;
  wire in_READ_pulse;
  reg [2:0] counter;
  wire counterDONE;
  wire addressed, read_nwrite;
  wire start_condition, stop_condition;
  reg start_condition_reg;

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
  assign SCL_posedge = ~SCL_d & SCL;
  assign in_READ_pulse = ~in_READ_d & in_READ;

  //Conditions
  assign start_condition = SCL & SDA_negedge;
  assign stop_condition = SCL & SDA_posedge;
  always@(posedge clk or posedge rst)
    begin
      if(rst)
        begin
          start_condition_reg <= 1'b0;
        end
      else
        begin
          case(start_condition_reg)
            1'b0: start_condition_reg <= start_condition;
            1'b1: start_condition_reg <= in_IDLE;
          endcase
        end
    end

  //Decode states
  assign in_IDLE = (state == IDLE);
  assign in_ADDRS_ACK = (state == ADDRS_ACK);
  assign in_ADDRS = (state == ADDRS);
  assign in_WRITE = (state == WRITE);
  assign in_WRITE_ACK = (state == WRITE_ACK);
  assign in_READ = (state == READ);
  assign in_READ_ACK = (state == READ_ACK);
  assign in_Get_Data = in_ADDRS | in_WRITE;
  assign busy = ~in_IDLE;

  //Flags
  assign data_available = in_WRITE_ACK;
  assign data_request = (in_ADDRS_ACK & addressed) | (in_READ_ACK & ~SDA);
  assign addressed = (addr == receive_buffer[7:1]);
  assign read_nwrite = receive_buffer[0];
  
  //State transactions
  always@(posedge clk or posedge rst)
    begin
      if(rst)
        begin
          state <= IDLE;
        end
      else
        begin
          case(state)
            WAIT_STOP:
              begin
                state <= (stop_condition) ? IDLE : state;
              end
            IDLE:
              begin
                state <= (start_condition_reg & SCL_negedge) ? ADDRS : state;
              end
            ADDRS:
              begin
                state <= (counterDONE & SCL_negedge) ? ADDRS_ACK : state;
              end
            ADDRS_ACK:
              begin
                state <= (SCL_negedge) ? ((addressed) ? ((read_nwrite) ? READ : WRITE) : WAIT_STOP) : state;
              end
            READ:
              begin
                state <= (counterDONE & SCL_negedge) ? READ_ACK : state;
              end
            READ_ACK:
              begin
                state <= (SCL_negedge) ? READ : (((SDA_posedge & SDA)? WAIT_STOP : state));
              end
            WRITE:
              begin
                state <= (stop_condition) ? IDLE : ((counterDONE & SCL_negedge) ? WRITE_ACK : state);
              end
            WRITE_ACK:
              begin
                state <= (SCL_negedge) ? WRITE : state;
              end
            default:
              begin
                state <= IDLE;
              end
          endcase
        end
    end

  //Data line handling
  assign SDA = (SDA_Claim) ? SDA_Write : 1'bZ;
  assign SDA_Claim = in_READ | (in_ADDRS_ACK & addressed) | in_WRITE_ACK;
  assign SDA_Write = (in_READ) ? send_buffer[7] : 1'b0;

  //sample at posedge of SCL
  always@(posedge SCL)
    begin
      receive_buffer <= (in_Get_Data) ? {receive_buffer[6:0], SDA} : receive_buffer;
    end
  
  //Data out buffer
  always@(posedge clk)
    begin
      data_o <= (in_WRITE_ACK) ? receive_buffer : data_o;
    end
  
  //send_buffer
  always@(negedge SCL or posedge in_READ_pulse)
    begin
      if(in_READ_pulse)
        begin
          send_buffer <= data_i;
        end
      else
        begin
          send_buffer <= (in_READ) ? {send_buffer[6:0],1'b1} : send_buffer;
        end
    end

  //Count posedges
  assign counterDONE = &counter;
  always@(posedge SCL or posedge start_condition)
    begin
      if(start_condition)
        begin
          counter <= 3'b111;
        end
      else
        begin
          counter <= counter + {2'd0, (in_Get_Data | in_READ)};
        end
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
