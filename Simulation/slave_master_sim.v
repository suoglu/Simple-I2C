/* ------------------------------------------------ *
 * Title       : I2C Simulation                     *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : slave_master_sim.v                 *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 02/05/2021                         *
 * ------------------------------------------------ *
 * Description : I2C slave and master simulation    *
 * ------------------------------------------------ */
`timescale 1ns / 1ps
// `include "Sources/i2c.v"

module testbench();
  reg clk, rst, clkI2Cx2, start, read_nwrite;
  wire ready_master, data_available_master, data_request_master, busy_slave, data_available_slave, data_request_slave;
  wire [6:0] addr;
  reg [2:0] data_size;
  wire SCL, SDA;
  wire [7:0] data_o_master, data_o_slave;
  reg [7:0] data_i_master, data_i_slave;

  always #5 clk = ~clk;
  always #50 clkI2Cx2 = ~clkI2Cx2;

  assign addr = 7'b1010101;

  pullup(SCL);
  pullup(SDA);

  i2c_slave slave(clk,rst,busy_slave,data_available_slave,data_request_slave,addr, data_i_slave, data_o_slave,SCL,SDA);
  
  i2c_master master(clk,rst,clkI2Cx2,start, ready_master,,data_available_master, data_request_master, 1'b1,read_nwrite,addr, data_i_master,data_o_master,data_size, SCL, SDA);

  initial
    begin
      clk = 0;
      rst = 0;
      clkI2Cx2 = 0;
      start = 0;
      read_nwrite = 1;
      data_size = 3'd2;
      data_i_master = 8'h13;
      data_i_slave = 8'hCE;
      #2
      rst = 1;
      #10
      rst = 0;
      #10
      start = 1;
      #300
      start = 0;
      #6000
      read_nwrite = 0;
      #10
      start = 1;
      #300
      start = 0;
    end
endmodule