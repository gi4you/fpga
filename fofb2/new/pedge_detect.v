`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2016/07/28 22:14:06
// Design Name: 
// Module Name: pedge_detect
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module pedge_detect(
    input clk,
    input Reset,
    input trig,
    output edge_out
    );
    
	reg dac_trigReg, dac_trigReg_reg;
	reg trig_out_reg; 
	
    always @ (posedge clk) 
    begin 
        if(Reset) begin
            trig_out_reg <= 1'b0;
        end
        else begin
            dac_trigReg     <= trig;
            dac_trigReg_reg <= dac_trigReg;
            trig_out_reg    <= (dac_trigReg & !dac_trigReg_reg);
        end    
    end
    
    assign edge_out = trig_out_reg;
        
endmodule
