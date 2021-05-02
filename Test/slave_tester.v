/* ------------------------------------------------ *
 * Title       : I2C Slave Test v2.0                *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : slave_tester.v                     *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 02/05/2021                         *
 * ------------------------------------------------ *
 * Description : This module is tests I2C slave     *
 *               module.                            *
 * ------------------------------------------------ */
// `include "Test/ssd_util.v"

module i2c_slave_tester(
  input clk,
  input clk_100MHz,
  input rst,
  //Slave ports
  input busy,
  input data_available,
  input data_request,
  output [7:0] data_i, //Data in
  input [7:0] data_o, //Data Out
  input SCL,
  //input [6:0] debug,
  //Board pins
  //output [7:0] JC,
  //output [1:0] JB,
  input [15:0] sw,
  output [15:0] led,
  output [3:0] an,
  output [6:0] seg);
  wire [3:0] digit3, digit2, digit1, digit0;
  reg [31:0] data_buffer;
  reg data_mode;
  reg [1:0] tx_counter;

  // assign JC = {busy, debug};
  // assign JB = {1'b0,SCL};

  always@(negedge data_request or posedge rst)
    begin
      if(rst)
        begin
          data_mode <= 1'b0;
        end
      else
        begin
          data_mode <= ~data_mode;
        end
    end

  //tx_counter
  always@(negedge data_available or posedge rst)
    begin
      if(rst)
        begin
          tx_counter <= 2'b0;
        end
      else
        begin
          tx_counter <= tx_counter + 2'b1;
        end
    end
  
  always@(posedge clk or posedge rst)
    begin
      if(rst)
        begin
          data_buffer <= 32'b0;
        end
      else
        if(data_available)
          case(tx_counter)
            2'd0:
              begin
                data_buffer[15:8] <= data_o;
              end
            2'd1:
              begin
                data_buffer[7:0] <= data_o;
              end
            2'd2:
              begin
                data_buffer[23:16] <= data_o;
              end
            2'd3:
              begin
                data_buffer[31:24] <= data_o;
              end
          endcase
    end

  //IO control
  assign data_i = (data_mode) ? sw[15:8] : sw[7:0];
  assign led = data_buffer[31:16];
  assign {digit3, digit2, digit1, digit0} = data_buffer[15:0];
  ssdController4 ssdCntr(clk_100MHz, rst, 4'b1111, digit3, digit2, digit1, digit0, seg, an);
endmodule