/* ------------------------------------------------ *
 * Title       : Simple I2C interface v2.1          *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : i2c.v                              *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 04/12/2021                         *
 * ------------------------------------------------ *
 * Description : I2C master module                  *
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
  input SCL_i,
  output SCL_o,
  output SCL_t,
  input SDA_i,
  output SDA_o,
  output SDA_t);
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
  reg inAck_d, SDA_d, SCL_d;
  reg SDA_d_i2c;
  reg clkI2Cx2_d;
  //Buffers
  reg [7:0] send_buffer, receive_buffer;

  wire clkI2Cx2_negedge = ~clkI2Cx2 &  clkI2Cx2_d;
  wire clkI2Cx2_posedge =  clkI2Cx2 & ~clkI2Cx2_d;

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
  wire   SCL = (SCL_claim) ?    SCLK   : SCL_i;
  wire   SDA = (SDA_claim) ? SDA_write : SDA_i;
  assign SCL_claim = ~inReady;
  assign SDA_claim = inStart | inAddrs | inWrite | inReadAck | inStop;
  assign SDA_write = (inStart | inReadAck | inStop) ? (inReadAck & byteCountDone) : send_buffer[7];
  assign SCL_o = SCLK;
  assign SCL_t = ~SCL_claim;
  assign SDA_o = SDA_write;
  assign SDA_t = ~SDA_claim;

  //Listen I2C Bus & cond. gen.
  wire    SCL_posedge  =  SCL & ~SCL_d;
  wire    SDA_negedge  = ~SDA &  SDA_d;
  wire    SDA_posedge  =  SDA & ~SDA_d;
  wire  stopCondition  =  SCL & SDA_posedge;
  wire startCondition  =  SCL & SDA_negedge;

  //delay signals
  always@(posedge clk) begin
    inAck_d <= inAck;
    SDA_d <= SDA;
    SCL_d <= SCL;
    clkI2Cx2_d <= clkI2Cx2;
    SDA_d_i2c <= (clkI2Cx2_negedge) ? SDA : SDA_d_i2c;
  end
  
  //Determine if an other master is using the bus
  always@(posedge clk or posedge rst) begin
    if(rst) begin
      i2cBusy <= 1'b0;
    end else case(i2cBusy)
      1'b0: i2cBusy <= startCondition & inReady;
      1'b1: i2cBusy <= ~stopCondition & inReady;
    endcase
  end
  
  //receive buffer
  assign data_o = receive_buffer;
  always@(posedge clk) begin
    if(inRead & SCL & clkI2Cx2_posedge)begin
      receive_buffer <= {receive_buffer[6:0],SDA};
    end
  end

  //send buffer
  always@(posedge clk) begin
    if(clkI2Cx2_negedge) begin
      if(SDAupdate) begin
        send_buffer <= (inStart) ? addressByte : data_i;
      end else if(SDAshift & ~SCL & |bitCounter) begin
        send_buffer <= {send_buffer << 1};
      end
    end
  end
  
  //SDA control states for sending data
  assign SDAupdate = inStart | inWriteAck;
  assign SDAshift  = inAddrs | inWrite;

  //state transactions
  always@(posedge clk or posedge rst) begin
    if(rst) begin
      state <= READY;
    end else if(clkI2Cx2_negedge) begin
      case(state)
        READY: begin
          state <= (start & SCLK & ~i2cBusy) ? START : state;
        end
        START: begin
          state <= (~SCL) ? ADDRS : state;
        end
        ADDRS: begin
          state <= (~SCL & bitCountDone) ? WRITE_ACK : state;
        end
        WRITE_ACK: begin
          state <= (~SCL) ? ((~SDA_d_i2c & ~byteCountDone) ? ((~read_nwrite) ? ((data_valid) ? WRITE : state) : READ): STOP) : state;
        end
        WRITE: begin
          state <= (~SCL & bitCountDone) ? WRITE_ACK : state;
        end
        READ: begin
          state <= (~SCL & bitCountDone) ? READ_ACK : state;
        end
        READ_ACK: begin
          state <= (~SCL) ? ((byteCountDone) ? STOP : READ) : state;
        end
        STOP: begin
          state <= (SCL) ? READY : state;
        end
      endcase
    end
  end

  //bit counter
  assign bitCountDone = ~|bitCounter;
  assign bitCounterReset = inAck|inStart;
  always@(posedge clk or posedge bitCounterReset) begin
    if(bitCounterReset) begin
      bitCounter <= 3'd0;
    end else begin
      bitCounter <= bitCounter + {2'd0,((inAddrs|inWrite|inRead) & SCL_posedge)};
    end
  end

  //byte counter
  assign byteCountUp = ~inAck_d & inAck;
  assign byteCountDone = (byteCounter == data_size);
  always@(posedge clk) begin
    if(inStart) begin
      byteCounter <= 3'b111;
    end else begin
      byteCounter <= byteCounter + {2'd0,byteCountUp};
    end
  end

  //Divide clkI2Cx2 to get I2C clk
  always@(posedge clk or posedge rst) begin
    if(rst) begin
      SCLK <= 1'b1;
    end else begin
      SCLK <= SCLK ^ clkI2Cx2_posedge;
    end
  end
endmodule//i2c_master

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
