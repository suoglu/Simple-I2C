/* ------------------------------------------------ *
 * Title       : Multi Master Support Add-on v1.1   *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : i2c_Multimaster.v                  *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 19/09/2022                         *
 * ------------------------------------------------ *
 * Description : An add-on module to use single     *
 *               single master I2C masters in multi *
 *               master configuration, requires a   *
 *               master module with enable pin.     *
 * ------------------------------------------------ *
 * Revisions                                        *
 *     v1      : Inital version                     *
 *     v1.1    : Change coding style                *
 * ------------------------------------------------ */

module i2cMMsup(
  input clk,
  input rst,
  input enable,
  input SCL,
  input SDA,
  output I2C_Busy,
  output I2C_Free);

  reg SDA_d; //Delay SDA
  reg I2CinUse; //State

  wire module_rst = ~enable | rst; //Disableing system also resets

  assign I2C_Busy = I2CinUse;
  assign I2C_Free = ~I2C_Busy;

  wire SDA_negedge = SDA_d & ~SDA;
  wire SDA_posedge = ~SDA_d & SDA;

  wire startCond = SCL & SDA_negedge;
  wire  stopCond = SCL & SDA_posedge;

  always@(posedge clk or posedge module_rst) begin
    if(module_rst) begin
      I2CinUse <= 1'd0;
    end else begin
      if(stopCond) begin
        I2CinUse <= 1'd0;
      end else if(startCond) begin
        I2CinUse <= 1'd1;
      end
    end
  end

  always@(posedge clk) begin
    SDA_d <= SDA;
  end
endmodule

module i2cSCLcomb#(parameter MASTERCOUNT = 2)(
  input [(MASTERCOUNT-1):0] SCL_i,
  output SCL_o);
  
  assign SCL_o = &SCL_i;
endmodule
