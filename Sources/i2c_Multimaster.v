/* ------------------------------------------------ *
 * Title       : Multi Master Support Add-on v1     *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : i2c_Multimaster.v                  *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 31/01/2021                         *
 * ------------------------------------------------ *
 * Description : An add-on module to use single     *
 *               single master I2C masters in multi *
 *               master configuration, requires a   *
 *               master module with enable pin.     *
 * ------------------------------------------------ *
 * Revisions                                        *
 *     v1      : Inital version                     *
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
  wire startCond, stopCond; //I2C conditions
  wire SDA_negedge, SDA_posedge; //Edges of SDA
  wire module_rst; //Disableing system also resets

  assign module_rst = ~enable | rst;

  assign I2C_Busy = I2CinUse;
  assign I2C_Free = ~I2C_Busy;

  assign startCond = SCL & SDA_negedge;
  assign  stopCond = SCL & SDA_posedge;

  assign SDA_negedge = SDA_d & ~SDA;
  assign SDA_posedge = ~SDA_d & SDA;

  always@(posedge clk or posedge module_rst)
    begin
      if(module_rst)
        begin
          I2CinUse <= 1'd0;
        end
      else
        begin
          if(stopCond)
            begin
              I2CinUse <= 1'd0;
            end
          else if(startCond)
            begin
              I2CinUse <= 1'd1;
            end
        end
    end

  always@(posedge clk)
    begin
      SDA_d <= SDA;
    end
endmodule

module i2cSCLcomb#(parameter MASTERCOUNT = 2)(
  input [(MASTERCOUNT-1):0] SCL_i,
  output SCL_o);
  
  assign SCL_o = &SCL_i;
endmodule
