// -------------------------------------------------------------
//
//	BPM position read from SDI DPRAM.
//		
//		Generate address for SDI Dual port Ram read and also generate address for FOFB input stream data.
//
//		Max signals are 26 ( 12 RF bpm, 6 ID bpm, 6 XBPMs, 1 Cell controller)
//
//	
//	5/11/2013
//	5/21/2013
//		CLK_LATANCY is 2 clock for correct length
//
//	July 6/2013 redesigned for max 13 units
//		For 12 bpm and 1 cell controller
//
//	ALL_MEM_READ	1
//		Reading bpm and cell controller data.
//		Total memory address are 26 * 30 = 780 
// -------------------------------------------------------------

//
// -------------------------------------------------------------

`timescale 1 ns / 1 ns

//`define	CELL_MAX_SIGNAL		14		//with cell controller signal 2
//`define	CELL_BPM_SIGNAL		12		//only bpm signal 6 * 2

`define	ALL_MEM_READ		1			/* all data read */
`define CLK_LATANCY			1

module bpm_pos_read #(
    parameter DEBUG  = "false"
    )(
    clk,
    trig,
    cell_max_number,		// **
    cell_bmp_number,		// **
    address_move,		//0
    AddressStart,		//0
    AddressEnd,			//780
    addr,				//Output, like counter
    even_bit,			//monitoring 
    wr_one,				
    bpmRam_RD,			// SDI read      .sdi2_cc_fofb_en()
    bpmRam_ADDR,			// SDI address   .sdi2_cc_fofb_addr()
    bpmRamCellMark,		//cell signal start
    cell_cnt,			//cell number 
    pos_start_addr,		//cell position start address: 0, 14, 28 ...
    //
    bram_x_wr, bram_y_wr,					//BRAM X and Y write enable, 1 clock delay for DPRAM read latancy
    bram_x_addr, bram_y_addr,   				//BRAM address
    bpmPosDataValid,
    bpmPosFoFbDataValid,	//**
    DspMatrixCalStartTrig					//Trigger for next processing
    );


  input   clk;
  (* mark_debug = DEBUG *)input   trig;
  input   [4:0] cell_max_number;  	//7/03/13
  input   [4:0] cell_bmp_number;  	//7/03/13
  input   [9:0] address_move;  	// ufix10	default 0
  input   [9:0] AddressStart;  	// ufix10	default 0
  input   [9:0] AddressEnd;  	// ufix10	default 
  (* mark_debug = DEBUG *)output  [9:0] addr;  // ufix10
  (* mark_debug = DEBUG *)output	even_bit;
  (* mark_debug = DEBUG *)output  wr_one;
  (* mark_debug = DEBUG *)output  bpmRam_RD;
  (* mark_debug = DEBUG *)output [9:0] bpmRam_ADDR;
  (* mark_debug = DEBUG *)output bpmRamCellMark;
  (* mark_debug = DEBUG *)output [4:0] cell_cnt;
  (* mark_debug = DEBUG *)output [9:0] pos_start_addr;
  (* mark_debug = DEBUG *)output 	reg bram_x_wr, bram_y_wr;
  (* mark_debug = DEBUG *)output 	reg [8:0] bram_x_addr, bram_y_addr;	
  (* mark_debug = DEBUG *)output	bpmPosDataValid;
  (* mark_debug = DEBUG *)output	bpmPosFoFbDataValid;
  (* mark_debug = DEBUG *)output	DspMatrixCalStartTrig;
  
  
  //wire enb;
	reg [9:0] Counter_Free_Running_out1, bpmRamAddrReg;  // ufix10
	wire Relational_Operator2_out1;
	wire Relational_Operator1_out1;
	wire Relational_Operator3_out1;
	wire Logical_Operator2_out1;
	reg [26:0] aa;
	reg [9:0] pos_addr;
	reg	BpmDataValid;
	wire Pos_FoFbDataValid;
	

	//	
  always @ (posedge clk or posedge trig)
    begin: Counter_Free_Running_process
      if (trig == 1'b1) begin
        Counter_Free_Running_out1 <= 10'b0000000000;
        bpmRamAddrReg <= 10'd0;
      end
      else begin
        //if (enb == 1'b1) begin
          if (Counter_Free_Running_out1 == (AddressEnd+`CLK_LATANCY) ) begin
            Counter_Free_Running_out1 <= Counter_Free_Running_out1;		
            bpmRamAddrReg <= bpmRamAddrReg;	
          end
          else begin   	

`ifdef	ALL_MEM_READ	          	
          	bpmRamAddrReg <= bpmRamAddrReg + 1;
`else
            if(CcDataAddr == 1'b1) begin
          		bpmRamAddrReg <= bpmRamAddrReg + 3;
          	end
          	else begin
          		bpmRamAddrReg <= bpmRamAddrReg + 1;            
          	end	
`endif
          	Counter_Free_Running_out1 <= Counter_Free_Running_out1 + 1;            
          end
        end
      //end
    end // Counter_Free_Running_process

  assign addr = Counter_Free_Running_out1;
  assign Relational_Operator2_out1 = (Counter_Free_Running_out1 == address_move) ? 1'b1 : 1'b0;

  assign wr_one = Relational_Operator2_out1;
  assign Relational_Operator1_out1 = (Counter_Free_Running_out1 >= AddressStart) ? 1'b1 : 1'b0;

  assign Relational_Operator3_out1 = (Counter_Free_Running_out1 <= AddressEnd) ? 1'b1 : 1'b0;

  assign Logical_Operator2_out1 =  Relational_Operator1_out1 & Relational_Operator3_out1;

  assign bpmRam_RD = Logical_Operator2_out1;

	////  14: bpm 6, cell 1 = 7*2 = 14
	(* mark_debug = DEBUG *) wire cell_start;  
	//wire even_bit;
	//assign cell_start = ((addr % `CELL_BPM_SIGNAL ) && (addr >= 0)) == 0;
	
	
`ifdef	ALL_MEM_READ	
	assign cell_start = ((addr % cell_max_number ) && (addr >= 0)) == 0;
`else
	assign cell_start = ((addr % cell_bmp_number ) && (addr >= 0)) == 0;
`endif	
	
	assign even_bit = ((bpmRam_ADDR % 2 ) && (addr >= 0)) == 0;
	
	reg [4:0] Reg_cell_cnt;
	always @ (posedge clk) begin
		if (trig == 1'b1) begin
			Reg_cell_cnt <= 5'd0;
		end	
		else if(cell_start == 1'b1) begin
			//pos_addr <= Reg_cell_cnt*`CELL_MAX_SIGNAL;
			pos_addr <= Reg_cell_cnt * cell_max_number;
			Reg_cell_cnt <= Reg_cell_cnt +1;			
		end	
	end	
	assign cell_cnt = Reg_cell_cnt;
	assign pos_start_addr = pos_addr;
	
			
	//////////	
  	always @ (posedge clk or posedge trig)
    begin : addr_process
      if (trig == 1'b1) begin
        aa <= 26'd0;
      end
      else begin
		aa[0] <= cell_start;
		aa[1] <= aa[0];
		aa[2] <= aa[1];
		aa[3] <= aa[2];
		aa[4] <= aa[3];
		aa[5] <= aa[4];
		aa[6] <= aa[5];
		aa[7] <= aa[6];
		aa[8] <= aa[7];
		aa[9] <= aa[8];
		aa[10] <= aa[9];
		aa[11] <= aa[10];
		//
		aa[12] <= aa[11];
		aa[13] <= aa[12];
		aa[14] <= aa[13];
		aa[15] <= aa[14];
		aa[16] <= aa[15];
		aa[17] <= aa[16];
		aa[18] <= aa[17];
		aa[19] <= aa[18];														
		aa[20] <= aa[19];
		aa[21] <= aa[20];
		aa[22] <= aa[21];
		aa[23] <= aa[22];
		aa[24] <= aa[23];											
		
        end
    end // 
    
	//7/06
	wire   bpmDataEnd; 
	//assign bpmDataEnd     = aa[10];		//11 clock delay for 12, 14	
	//assign bpmRamCellMark = aa[11];		//11 clock delay for 12, 14	
	//
	assign bpmDataEnd     = aa[cell_bmp_number-2];		//11 clock delay for 12, 14	
	assign bpmRamCellMark = aa[cell_bmp_number-1];		//11 clock delay for 12, 14		

	// SDI RAM address generator
	//wire	CcDataAddr = aa[10];	// CcDataAddr = 12
	//cell_max_number - 2
	wire	CcDataAddr = aa[22];	// CcDataAddr = 24
	
			
	always @ (posedge clk ) begin
		if(cell_start == 1'b1) begin
			BpmDataValid <= 1'b1;
		end	
		else if (bpmDataEnd == 1'b1)	
			BpmDataValid <= 1'b0;
		else
			BpmDataValid <= BpmDataValid;
	end		
	    
	
	assign bpmRam_ADDR = bpmRamAddrReg; 
	

	// 2 clk delay because dpram output is 2 clk latancy
	// This timing for SDI dpram port-b reading
	// Delay latancy shud be check when you make a IP core	
	reg [8:0] x_addr, y_addr;	
	wire x_wr, y_wr;
	
	always @(posedge clk)
		begin
		if(Logical_Operator2_out1 == 1'b1) begin
			if(trig == 1'b1) 
				x_addr <= 9'd0;
			//else if(even_bit == 1'b1) begin
			else if(even_bit == 1'b1 & Pos_FoFbDataValid == 1'b1) begin			
				x_addr <= x_addr + 1;
			end	
			else begin
				x_addr <= x_addr;
			end	
		end		
	   end

	always @(posedge clk)
		begin
		if(Logical_Operator2_out1 == 1'b1) begin
			if(trig == 1'b1) 
				y_addr <= 9'd0;
			//else if(even_bit == 1'b0) begin
			else if(even_bit == 1'b0 & Pos_FoFbDataValid == 1'b1) begin
				y_addr <= y_addr + 1;
			end	
			else begin
				y_addr <= y_addr;
			end		
		end			
	   end	   	   
	
	reg y;      	
	always @(posedge clk)
	   begin
	   y <= even_bit;
	   end
	   
	  //timing check 
	  assign x_wr =  even_bit & Pos_FoFbDataValid;
	  assign y_wr = y  & Pos_FoFbDataValid;
	  
	// 2 clk delay 
	reg x1, y1; 
	reg [9:0] tpm_x_addr, tpm_y_addr; 
	always @(posedge clk)
		if(trig == 1'b1) begin	//add 06/07/14
			x1 <= 1'b0;
			y1 <= 1'b0;
		end	
		else begin
			x1 <= x_wr;
			y1 <= y_wr;
			tpm_x_addr <= x_addr;
			tpm_y_addr <= y_addr;
			//
			bram_x_wr <= x1;
			bram_y_wr <= y1;
			bram_x_addr <= tpm_x_addr;
			bram_y_addr <= tpm_y_addr;
	   	end
	   		   
	  
	//  
	wire endOfCopy;  
	pos_edge endOfCopy_trig
	(
		.clk(clk),
		.reset(1'b0),
		.enb(1'b1),
		.trig_in(~Logical_Operator2_out1),	
		.trig_out(endOfCopy)
	);
	assign DspMatrixCalStartTrig = endOfCopy;
		  
	reg [2:0] dly;;	  
	always @(posedge clk) begin
		  dly[0] <= bpmRam_RD;
	end	  
		  
	reg [2:0] fdly;	
	assign Pos_FoFbDataValid = BpmDataValid | cell_start;
		  
	always @(posedge clk) begin
		  fdly[0] <= Pos_FoFbDataValid;
		  fdly[1] <= fdly[0];
	end	  
			
	//assign bpmPosDataValid = dly[1]; //2 clock delay, It timing is correct for Isim
	assign bpmPosDataValid = dly[0]; 
	// 06/08/14   added cell_cnt <31	
	assign bpmPosFoFbDataValid = fdly[1] && (bpmRam_ADDR > 9'd1) && (cell_cnt <31) ;	//fast orbit feedback position valid, only 6*2
	
endmodule  // bpm_addr_gen

/*
 *	END
 */