`timescale 1ns / 1ps


module DMA_TransferSync #(
    parameter DEBUG  = "true"
    )(
	input sysClk,
	input reset,
	input xfer_trigger,
	input P0_trigger,
	input dma_xfer_req,
	output reg dma_xfer_enable,
	output reg [31:0] turns_cnt	
    );
	
	
	localparam IDLE_      = 2'b01;
	localparam READY_     = 2'b10;
	localparam TRANSFER   = 2'b11;

	reg [1:0] state = IDLE_;
	
	always@(posedge sysClk) begin		
		if(reset == 1'b1) begin
			dma_xfer_enable <= 1'b0;
			state           <= IDLE_;
			turns_cnt       <= 32'd0;
		end
		else begin
			case (state)
				IDLE_  : begin
					if (xfer_trigger) begin	
						state <= READY_;	//start trigger go to READY
					end
					else state <= IDLE_;				
				end
				READY_ : begin
					if (P0_trigger) begin	
						if(dma_xfer_req == 1'b1) begin
							dma_xfer_enable <= 1'b1;
							turns_cnt       <= 32'd0;	
							state           <= TRANSFER;	
						end
						else state <= READY_;	
					end
					else state <= READY_;
				end
				TRANSFER: begin
					if(dma_xfer_req == 0) begin
						dma_xfer_enable <= 1'b0;
						state           <= IDLE_;
					end
					else begin
					    if (P0_trigger) begin	  //4/25/21 found missing trigger
						    turns_cnt <= turns_cnt+1;  
						end  
						state     <= TRANSFER;
					end	
				end

			endcase	
		end	
	end	
	
	
endmodule
	
