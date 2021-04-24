/* ------------------------------------------------ *
 * Title       : I2C Master Test                    *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : master_board.v                     *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 19/12/2020                         *
 * ------------------------------------------------ *
 * Description : Test board for I2C Master          *
 * ------------------------------------------------ */
//`include "Sources/i2c_masterv1.v"
//`include "Test/ssd_util.v"
//`include "Test/btn_debouncer.v"
module master_board(
  input clk,
  input rst,
  input [7:0] sw,
  input btnR,//Send
  output busy,//JB7
  output newData,//JB8
  input read_nwrite, //sw13
  input [1:0] data_byte_size, //sw15:14
  input dig_slct, //sw12
  output [6:0] seg,
  output [3:0] an, 
  inout SDA,//JB4
  output SCL); //JB3

  wire dataReq;
  wire start;
  wire [7:0] data_o;
  reg [23:0] data_buff;
  wire [15:0] selected_dig;

  assign selected_dig = (dig_slct) ? data_buff[23:8] : {data_buff[7:0], data_o};

  always@(posedge newData or posedge rst)
    begin
      if(rst)
        begin
          data_buff <= 24'd0;
        end
      else
        begin
          data_buff <= {data_buff[15:0], data_o};
        end
    end
  
  ssdController4 ssd_ctrl(clk, rst, 4'b1111, selected_dig[15:12], selected_dig[11:8], selected_dig[7:4], selected_dig[3:0], seg, an);
  debouncer snd_db(clk, rst, btnR, start);
  //Hardcoded values: address 0x5A, freq 390.625kHz, data always valid
  i2c_masterv1 uut_m(clk, rst, 2'b01, busy, newData, dataReq, 1'b1,  start,  data_byte_size, read_nwrite, 7'b1000000,  sw,  data_o,  SCL, SDA); 
endmodule//master_board