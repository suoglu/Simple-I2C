/* ------------------------------------------------ *
 * Title       : I2C Slave Test                     *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : slave_board.v                      *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 19/12/2020                         *
 * ------------------------------------------------ *
 * Description : Test board for I2C Slave           *
 * ------------------------------------------------ */
// `include "Sources/i2c.v"
// `include "Test/ssd_util.v"

module slave_board(
  input clk,
  input rst,
  input [15:0] sw,
  output [15:0] led,
  output [6:0] seg,
  output [3:0] an, 
  inout SDA,//JB4
  input SCL); //JB3

  reg [31:0] storage;
  wire busy, newData, dataReq;
  wire [7:0] data_o, data_i;
  reg read_bits;

  assign data_i = (read_bits) ? sw[15:8] : sw[7:0];

  always@(posedge dataReq or posedge rst)
    begin
      if(rst)
        begin
          read_bits <= 1'b1;
        end
      else
        begin
          read_bits <= ~read_bits;
        end
    end
  

  always@(negedge newData or posedge rst)
    begin
      if(rst)
        begin
          storage <= 32'd0;
        end
      else
        begin
          storage <= {storage[23:0], data_o};
        end
    end
  
  assign led = storage[31:16];

  ssdController4 ssd_ctrl(clk, rst, 4'b1111, storage[15:12], storage[11:8], storage[7:4], storage[3:0], seg, an);
  
  i2c_slave uut_slave(clk, rst, busy, newData,  dataReq, 7'b1010101, data_i, data_o, SCL, SDA);
endmodule//slave_board
