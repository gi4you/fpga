//controller
module controller (
    input 	clk, reset,
    input 	fofbCalStart, 
	input	[8:0] CalcLanth,	//for variable 
    output reg [8:0] addRamR,addRamRReg,	            
    output tlast_t,
    output tvalid_t,
	output wire addRamR_Valid
	);
	
	reg oneModeStart;
	reg oneModeStartReg1,oneModeStartReg2,calRunning;
	
	//parameter CAL_LENGTH = 9'h167;   //359
	//parameter CAL_LENGTH = 9'h1DF;   //479
	
	always @(negedge clk) begin
		oneModeStart <= fofbCalStart;
		oneModeStartReg1 <= oneModeStart;
		oneModeStartReg2 <= oneModeStartReg1; 
	end

    reg tlast;
	always @(negedge clk) begin
		if(reset==1'b1) begin
			calRunning <= 1'h0;
			tlast <= 1'b0;
		end	
		else if((oneModeStartReg2==1'b0)&&(oneModeStartReg1==1'b1)) 
			calRunning <= 1'h1;
		//else if (addRamR == CAL_LENGTH)
		else if (addRamR == CalcLanth )	begin //06/08/14		
			calRunning <= 1'h0;
			tlast <= 1'b0;
		end	
		else if (addRamR == CalcLanth-1 )
				tlast <= 1'b1;			
		else
			calRunning <= calRunning;
	end

	reg add_Valid;
	always @(negedge clk) begin	
		if(calRunning==1'b0) begin
			addRamR <= 9'h0;
		end	
		else begin
			addRamR <= addRamR + 9'h1;
		end	
	end
  
	//need one daley for BPM RAM
	reg [8:0] addRamR0;
	always @(posedge clk) begin
	    addRamR0   <= addRamR;
		addRamRReg <= addRamR0;	
	end	
	
	reg tlast_r0, tlast_r1;
	always @(posedge clk) begin
	    tlast_r0   <= tlast;
		tlast_r1   <= tlast_r0;	
	end
	
	reg tvalid_r0, tvalid_r1;
	always @(posedge clk) begin
	    tvalid_r0   <= calRunning;
		tvalid_r1   <= tvalid_r0;	
	end
	assign tvalid_t = tvalid_r1;
		
	assign addRamR_Valid = calRunning;
	//assign tlas_t = tlast;
	assign tlast_t = tlast_r1;
	
endmodule