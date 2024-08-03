`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    22:29:32 11/30/2012 
// Design Name: 
// Module Name:    multiple_ram_cs 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//	This module used for Microblaze and DPRAM/Register interface
//
//////////////////////////////////////////////////////////////////////////////////
module multiple_ram_cs(
	input   clk,
	input   reset,
	input   [3:0] RAM_sel,  // ufix4
	input   signed [31:0] Data_in,  // int32
	input   Write_in,
	input   [12:0] Addr_in,  // int32
			
	output [12:0] Ram_Addr,  // int32	
	output [15:0] ram_cs,
	output signed [31:0] Data,
	output wr
);
	
reg [31:0] data;
reg [12:0] addr;
always @(posedge clk) begin
	data <= Data_in;
	addr <= Addr_in;	
end


reg [15:0] cs=0;
always @(*) case( RAM_sel )
	4'd0:  cs = 16'b0000000000000001;
	4'd1:  cs = 16'b0000000000000010;
	4'd2:  cs = 16'b0000000000000100;
	4'd3:  cs = 16'b0000000000001000;
	4'd4:  cs = 16'b0000000000010000;
	4'd5:  cs = 16'b0000000000100000;
	4'd6:  cs = 16'b0000000001000000;
	4'd7:  cs = 16'b0000000010000000;
	4'd8:  cs = 16'b0000000100000000;
	4'd9:  cs = 16'b0000001000000000;
	4'd10: cs = 16'b0000010000000000;
	4'd11: cs = 16'b0000100000000000;				
	4'd12: cs = 16'b0001000000000000;
	4'd13: cs = 16'b0010000000000000;
	4'd14: cs = 16'b0100000000000000;
	4'd15: cs = 16'b1000000000000000;
endcase


// pos edge
  wire Logical_Operator_out1;
  reg  Integer_Delay_out1;
  wire Logical_Operator2_out1;
  
  assign Logical_Operator_out1 = !Write_in;

  always @ (posedge clk)
    begin: Integer_Delay_process
      if (reset == 1'b1) begin
        Integer_Delay_out1 <= 1'b0;
      end
      else begin
        //if (enb == 1'b1) begin
          Integer_Delay_out1 <= Logical_Operator_out1;
        //end
      end
    end 

    assign Logical_Operator2_out1 =  Write_in & Integer_Delay_out1;

    assign Data = Logical_Operator2_out1 ? data : 32'd0;   
    assign Ram_Addr = addr;
	assign wr = Logical_Operator2_out1;
	assign ram_cs = cs;  

	
endmodule