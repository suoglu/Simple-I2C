/* ------------------------------------------------ *
 * Title       : Simple I2C Slave v2.1              *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : i2c_slave.v                        *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 02/05/2021                         *
 * ------------------------------------------------ *
 * Description : I2C slave                          *
 * ------------------------------------------------ *
 * Revisions                                        *
 *     v2      : Changed master algorithm           *
 *     v2.1    : Async reset for master bit counter *
 *               and working slave                  *
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
