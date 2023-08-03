/* ------------------------------------------------ *
 * Title       : Simple I2C Slave v2.2              *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : i2c_slave.v                        *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 19/09/2022                         *
 * Licence     : CERN-OHL-W                         *
 * ------------------------------------------------ *
 * Description : I2C slave                          *
 * ------------------------------------------------ *
 * Revisions                                        *
 *     v2      : Changed master algorithm           *
 *     v2.1    : Async reset for master bit counter *
 *               and working slave                  *
 *     v2.2    : Change coding style                *
 * ------------------------------------------------ */

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
  reg [7:0] send_buffer, receive_buffer;
  reg SDA_d, SCL_d, in_READ_d;
  reg [2:0] state;
  reg [2:0] counter;
  wire counterDONE;
  reg start_condition_reg;

  //I2C signal edges into system clock domain
  always@(posedge clk) begin
    SDA_d <= SDA;
    SCL_d <= SCL;
    in_READ_d <= in_READ;
  end
  wire SDA_negedge = SDA_d & ~SDA;
  wire SDA_posedge = ~SDA_d & SDA;
  wire SCL_negedge = SCL_d & ~SCL;
  wire SCL_posedge = ~SCL_d & SCL;
  wire in_READ_pulse = ~in_READ_d & in_READ;

  //Conditions
  wire start_condition = SCL & SDA_negedge;
  wire stop_condition = SCL & SDA_posedge;
  always@(posedge clk or posedge rst) begin
    if(rst)begin
      start_condition_reg <= 1'b0;
    end else begin
      case(start_condition_reg)
        1'b0: start_condition_reg <= start_condition;
        1'b1: start_condition_reg <= in_IDLE;
      endcase
    end
  end

  //Decode states
  wire in_IDLE = (state == IDLE);
  wire in_ADDRS_ACK = (state == ADDRS_ACK);
  wire in_ADDRS = (state == ADDRS);
  wire in_WRITE = (state == WRITE);
  wire in_WRITE_ACK = (state == WRITE_ACK);
  wire in_READ = (state == READ);
  wire in_READ_ACK = (state == READ_ACK);
  wire in_Get_Data = in_ADDRS | in_WRITE;
  assign busy = ~in_IDLE;

  //Flags
  wire addressed = (addr == receive_buffer[7:1]);
  wire read_nwrite = receive_buffer[0];
  assign data_available = in_WRITE_ACK;
  assign data_request = (in_ADDRS_ACK & addressed) | (in_READ_ACK & ~SDA);
  
  //State transactions
  always@(posedge clk or posedge rst) begin
    if(rst) begin
      state <= IDLE;
    end else begin
      case(state)
        WAIT_STOP: begin
          state <= (stop_condition) ? IDLE : state;
        end
        IDLE: begin
          state <= (start_condition_reg & SCL_negedge) ? ADDRS : state;
        end
        ADDRS: begin
          state <= (counterDONE & SCL_negedge) ? ADDRS_ACK : state;
        end
        ADDRS_ACK: begin
          state <= (SCL_negedge) ? ((addressed) ? ((read_nwrite) ? READ : WRITE) : WAIT_STOP) : state;
        end
        READ: begin
          state <= (counterDONE & SCL_negedge) ? READ_ACK : state;
        end
        READ_ACK: begin
          state <= (SCL_negedge) ? READ : (((SDA_posedge & SDA)? WAIT_STOP : state));
        end
        WRITE: begin
          state <= (stop_condition) ? IDLE : ((counterDONE & SCL_negedge) ? WRITE_ACK : state);
        end
        WRITE_ACK: begin
          state <= (SCL_negedge) ? WRITE : state;
        end
        default: begin
          state <= IDLE;
        end
      endcase
    end
  end

  //Data line handling
  wire SDA_Claim = in_READ | (in_ADDRS_ACK & addressed) | in_WRITE_ACK;
  wire SDA_Write = (in_READ) ? send_buffer[7] : 1'b0;
  assign SDA = (SDA_Claim) ? SDA_Write : 1'bZ;

  //sample at posedge of SCL
  always@(posedge SCL) begin
    receive_buffer <= (in_Get_Data) ? {receive_buffer[6:0], SDA} : receive_buffer;
  end
  
  //Data out buffer
  always@(posedge clk) begin
    data_o <= (in_WRITE_ACK) ? receive_buffer : data_o;
  end
  
  //send_buffer
  always@(negedge SCL or posedge in_READ_pulse) begin
    if(in_READ_pulse) begin
      send_buffer <= data_i;
    end else begin
      send_buffer <= (in_READ) ? {send_buffer[6:0],1'b1} : send_buffer;
    end
  end

  //Count posedges
  assign counterDONE = &counter;
  always@(posedge SCL or posedge start_condition) begin
    if(start_condition) begin
      counter <= 3'b111;
    end else begin
      counter <= counter + {2'd0, (in_Get_Data | in_READ)};
    end
  end
endmodule//i2c_slave
