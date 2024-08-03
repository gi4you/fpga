`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/24/2024 07:04:32 PM
// Design Name: 
// Module Name: event_sync
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

module event_sync(
	input  sysClk,
	input  reset,
	input  valid,
	input  evr_trig,
	output reg evr_trig_valid
    );
	
	
    localparam IDLE_      = 2'b01;
    localparam READY_     = 2'b10;
    localparam TRANSFER   = 2'b11;
    
    reg [1:0] state = IDLE_;
    //reg [6:0] count;
	
	
	always@(posedge sysClk) begin		
		if(reset == 1'b1) begin
			evr_trig_valid <= 1'b0;
			state             <= IDLE_;
		end
		else begin
			case (state)
				IDLE_  : begin
					if (evr_trig == 1'b1 && valid) begin	
					    evr_trig_valid <= 1'b1;
						state <= READY_;	
					end
					else begin	
					    if (evr_trig == 1'b1) begin
					        evr_trig_valid <= 1'b0;
					    end
						state <= IDLE_;	
					end									
				end
				READY_ : begin                      
                    state <= IDLE_;                     
				end                    
			endcase	
		end	
	end	
	

endmodule

