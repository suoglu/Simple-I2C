/* ------------------------------------------------ *
 * Title       : I2C Master Test v2.0               *
 * Project     : Simple I2C                         *
 * ------------------------------------------------ *
 * File        : test_module.v                      *
 * Author      : Yigit Suoglu                       *
 * Last Edit   : 23/04/2021                         *
 * ------------------------------------------------ *
 * Description : This module is tests I2C master    *
 *               module.                            *
 * ------------------------------------------------ */
// `include "Sources/i2c.v" //clock gen
// `include "Test/ssd_util.v"
// `include "Test/btn_debouncer.v"

module i2c_master_tester(
  input clk,
  input rst,
  //Board GPIO
  input [15:0] sw,//sw[7:0]: first byte to write; sw[15:13]: resolution bits of config register
  output [15:0] led, //{data_i, data_o}
  input btnU, //READ2
  input btnR, //READ4
  input btnD, //WRITE1
  input btnL, //WRITE3
  output [3:0] an,
  output [6:0] seg,
  //I2C master connections
  output clkI2Cx2,
  output reg start,
  input ready,
  input i2cBusy,
  input data_available, 
  input data_request,
  output data_valid,
  output read_nwrite, //1: read; 0: write
  output [6:0] addr,
  output reg [7:0] data_i, //Data in
  input [7:0] data_o, //Data Out
  output reg [2:0] data_size);
  localparam configByte2 = 8'h00,
         configByte1ms5b = 5'b00010;
  wire [7:0] configByte1;
  reg [15:0] read_buffer;
  localparam WAIT = 3'd0,
            READ2 = 3'd1,
            READ4 = 3'd2,
           WRITE1 = 3'd3,
           WRITE3 = 3'd4;
  reg [2:0] state;
  reg [3:0] dataCounter;
  reg data_request_d;
  wire data_request_posedge;
  wire inWAIT, inREAD2, inREAD4, inWRITE1, inWRITE3, inWRITE, inREAD;
  wire r2, r4, w1, w3;
  assign   inWAIT = (state == WAIT);
  assign  inREAD2 = (state == READ2);
  assign  inREAD4 = (state == READ4);
  assign inWRITE1 = (state == WRITE1);
  assign inWRITE3 = (state == WRITE3);
  assign  inWRITE = inWRITE1 | inWRITE3;
  assign   inREAD =  inREAD2 | inREAD4;

  assign configByte1 = {configByte1ms5b, sw[15:13]};
  assign addr = 7'h40; //Pmod HYGRO addrs
  assign data_valid = 1'b1;

  assign read_nwrite = inREAD | r2 | r4;
  assign led = {data_i, data_o};

  always@(posedge clk or posedge rst)
    begin
      if(rst)
        begin
          data_request_d <= 1'd0;
        end
      else
        begin
          data_request_d <= data_request;
        end
    end
  
  assign data_request_posedge = ~data_request_d & data_request;

  always@(posedge clk)
    begin
      if(inWAIT)
        begin
          dataCounter <= 3'd0;
        end
      else
        begin
          dataCounter <= dataCounter + {2'd0, data_request_posedge};
        end
    end

  always@*
    begin
      if(inWRITE3)
        begin
          case(dataCounter)
            3'd2:
              begin
                data_i = configByte1;
              end
            3'd3:
              begin
                data_i = configByte2;
              end
            default:
              begin
                data_i = sw[7:0];
              end
          endcase
        end
      else
        begin
          data_i = sw[7:0];
        end
    end

  always@*
    begin
      if(inREAD2 | r2)
        begin
          data_size = 3'd2;
        end
      else if(inREAD4 | r4)
        begin
          data_size = 3'd4;
        end
      else if(inWRITE1 | w1)
        begin
          data_size = 3'd1;
        end
      else if(inWRITE3 | w3)
        begin
          data_size = 3'd3;
        end
      else
        begin
          data_size = 3'd0;
        end
    end

  always@(posedge data_available or posedge rst)
    begin
      if(rst)
        begin
          read_buffer <= 16'h0000;
        end
      else
        begin
          read_buffer <= {read_buffer[7:0],data_o};
        end
    end
  
  always@(posedge clk or posedge rst)
    begin
      if(rst)
        begin
          state <= WAIT;
        end
      else
        begin
          case(state)
            WAIT:
              begin
                if(r2)
                  begin
                    state <= READ2;
                  end
                else if(r4)
                  begin
                    state <= READ4;
                  end
                else if(w1)
                  begin
                    state <= WRITE1;
                  end
                else if(w3)
                  begin
                    state <= WRITE3;
                  end
              end
            default:
              begin
                state <= (~start & ready) ? WAIT : state;
              end
          endcase
        end
    end
  
  always@(posedge clk or posedge rst) //start signal
    begin
      if(rst)
        begin
          start <= 1'b0;
        end
      else
        case(start)
          1'b0:
            begin
              start <= r2 | r4 | w1 | w3;
            end
          1'b1:
            begin
              start <= ready;
            end
        endcase
    end

  debouncer dbU(clk, rst, btnU, r2);
  debouncer dbL(clk, rst, btnL, w3);
  debouncer dbR(clk, rst, btnR, r4);
  debouncer dbD(clk, rst, btnD, w1);
  ssdController4 ssdCntrol(clk, rst, 4'b1111, read_buffer[15:12], read_buffer[11:8], read_buffer[7:4], read_buffer[3:0], seg, an);
  clockGen_i2c i2cClkGen(clk,rst,2'd1,clkI2Cx2); //Fixed on ~2x400MHz
endmodule