/*
	12/11/2021
		added 64bit data generator for Versal VCK190
		enable is Aurora READY signal
			
	    Tested at VCK190 board.
	12/12/2021
		Test with ZCU102 Local MPS LB+ port
		ZCU102 side no CRC error.
		
		
 */
`timescale 1 ns / 1 ns


module aurora_biComm_pkt_gen_64b # (
	parameter PACKET_MAX_LEN  = 8,    //56+CRC+HEAD  
	parameter DATA_SIZE       = 32,
	parameter DEBUG           = "false"
	//parameter DEBUG           = "true"
    ) (
	clk,
	reset,
	clk_enable,
	trig,		  
	a1,
	a2,
	a3,
	a4,
	a5,
	a6,
	a7,
	a8,
	a9,
	a10,
	a11,
	a12,
	a13,
	a14,
	a15,
	a16,
	a17,
	a18,
	a19,
	a20,
	a21,
	a22,
	a23,
	a24,
	a25,
	a26,
	a27,
	a28,
	a29,
	a30,
	a31,
	a32,
	a33,
	a34,
	a35,
	a36,
	a37,
	a38,
	a39,
	a40,
	a41,
	a42,
	a43,
	a44,	
	a45,	
	a46,
	a47,
	a48,
	a49,
	a50,
	a51,
	a52,
	a53,
	a54,
	a55,
	a56,
	a57,
	a58,
	a59,
	a60,		   
	a61,	
	a62,	
	a63,	
	a64,	
	a65,	
	a66,	
	a67,	
	a68,			   
	//
	mux_addr,
	TDATA,
	TLAST,
	TVALID,
	TDATA64,
	TVALID64
	);


	input   clk;
	input   reset;
	input   clk_enable;
	input	trig;
	input   signed [DATA_SIZE-1:0] a1;  
	input   signed [DATA_SIZE-1:0] a2;  
	input   signed [DATA_SIZE-1:0] a3;  
	input   signed [DATA_SIZE-1:0] a4;  
	input   signed [DATA_SIZE-1:0] a5;  
	input   signed [DATA_SIZE-1:0] a6;  
	input   signed [DATA_SIZE-1:0] a7;  
	input   signed [DATA_SIZE-1:0] a8;  
	input   signed [DATA_SIZE-1:0] a9;  
	input   signed [DATA_SIZE-1:0] a10;  
	input   signed [DATA_SIZE-1:0] a11;  
	input   signed [DATA_SIZE-1:0] a12;  
	input   signed [DATA_SIZE-1:0] a13;  
	input   signed [DATA_SIZE-1:0] a14;  
	input   signed [DATA_SIZE-1:0] a15;  
	input   signed [DATA_SIZE-1:0] a16;  
	input   signed [DATA_SIZE-1:0] a17;  
	input   signed [DATA_SIZE-1:0] a18;  
	input   signed [DATA_SIZE-1:0] a19;  
	input   signed [DATA_SIZE-1:0] a20;  
	input   signed [DATA_SIZE-1:0] a21;  
	input   signed [DATA_SIZE-1:0] a22;  
	input   signed [DATA_SIZE-1:0] a23;  
	input   signed [DATA_SIZE-1:0] a24;  
	input   signed [DATA_SIZE-1:0] a25;  
	input   signed [DATA_SIZE-1:0] a26;  
	input   signed [DATA_SIZE-1:0] a27;  
	input   signed [DATA_SIZE-1:0] a28;  
	input   signed [DATA_SIZE-1:0] a29;  
	input   signed [DATA_SIZE-1:0] a30;  
	input   signed [DATA_SIZE-1:0] a31;  
	input   signed [DATA_SIZE-1:0] a32;  
	input   signed [DATA_SIZE-1:0] a33;  
	input   signed [DATA_SIZE-1:0] a34;  

	input   signed [DATA_SIZE-1:0] a35;
	input   signed [DATA_SIZE-1:0] a36;
	input   signed [DATA_SIZE-1:0] a37;
	input   signed [DATA_SIZE-1:0] a38;
	input   signed [DATA_SIZE-1:0] a39;
	input   signed [DATA_SIZE-1:0] a40;
	input   signed [DATA_SIZE-1:0] a41;
	input   signed [DATA_SIZE-1:0] a42;
	input   signed [DATA_SIZE-1:0] a43;
	input   signed [DATA_SIZE-1:0] a44;	
	input   signed [DATA_SIZE-1:0] a45;	
	input   signed [DATA_SIZE-1:0] a46;
	input   signed [DATA_SIZE-1:0] a47;  
	input   signed [DATA_SIZE-1:0] a48;  
	input   signed [DATA_SIZE-1:0] a49;  
	input   signed [DATA_SIZE-1:0] a50;  
	input   signed [DATA_SIZE-1:0] a51;  
	input   signed [DATA_SIZE-1:0] a52;  
	input   signed [DATA_SIZE-1:0] a53;  
	input   signed [DATA_SIZE-1:0] a54;  
	input   signed [DATA_SIZE-1:0] a55;  
	input   signed [DATA_SIZE-1:0] a56;  
	input   signed [DATA_SIZE-1:0] a57;  
	input   signed [DATA_SIZE-1:0] a58;  
	input   signed [DATA_SIZE-1:0] a59;  
	input   signed [DATA_SIZE-1:0] a60;  
	input   signed [DATA_SIZE-1:0] a61;
	input   signed [DATA_SIZE-1:0] a62;
	input   signed [DATA_SIZE-1:0] a63;
	input   signed [DATA_SIZE-1:0] a64;
	input   signed [DATA_SIZE-1:0] a65;
	input   signed [DATA_SIZE-1:0] a66;
	input   signed [DATA_SIZE-1:0] a67;
	input   signed [DATA_SIZE-1:0] a68;
	
	(* mark_debug = DEBUG *) output  signed [DATA_SIZE-1:0] TDATA;  
	(* mark_debug = DEBUG *) output  TLAST;
    (* mark_debug = DEBUG *) output  TVALID;
	
	output [6:0] mux_addr;
	(* mark_debug = DEBUG *) output  wire signed [64-1:0] TDATA64;
    (* mark_debug = DEBUG *) output  wire TVALID64;
               
	(* mark_debug = DEBUG *) wire [6:0] address; 
	(* mark_debug = DEBUG *) wire enb;
	(* mark_debug = DEBUG *) wire signed [DATA_SIZE-1:0] InPacketDataOUT;  
	wire Relational_Operator1_out1;
	wire Relational_Operator3_out1;
	wire Logical_Operator2_out1;
	reg [6:0] Counter_Free_Running1_out1;  // ufix6
	reg [6:0] Counter_Free_Running2;
	
	(* mark_debug = DEBUG *) wire trig0;
	wire [6:0]  trig_addr;
	wire mTrig;
	
	`define	AIE_CAL_NCLK	3	
	pkt_send_trig_gen pktSendMtrig (
		.clk( clk ), 
		.reset(reset), 
		.trig(trig), 
		.clk_enable   (enb), //add enable and stable with zcu102 receive 
		.address_move (12'd`AIE_CAL_NCLK-2),     
		.AddressStart (12'd0), 
		.AddressEnd   (12'd`AIE_CAL_NCLK), 	
		.aie_mask(64'h0000000000000001),
		.Data16b      (), 
		.aie_addr     (trig_addr), 
		.wr_one       (), 
		.wr           (mTrig),			
		.mask()
	);
	
	
	pedge_detect edge_pktSendMtrig ( 
		.clk(clk), 
		.Reset(reset), 
		.trig( mTrig ), 
		.edge_out(trig0) 
	);	

	reg [6:0] aurora_trig_addr;
	always @ (posedge clk) begin
		if(trig == 1'b1 ) begin
			aurora_trig_addr <= 7'd0;
		end	
		else if( trig0 == 1'b1 ) begin
			aurora_trig_addr <= aurora_trig_addr+1;
		end	
	end
 
	reg auroraTxMtrig;
    always @ (posedge clk) begin
		auroraTxMtrig <= trig0;
	end

		
	(* mark_debug = DEBUG *) wire  [6:0] start_addr = (aurora_trig_addr-1)*(PACKET_MAX_LEN-1); //15;
	(* mark_debug = DEBUG *) wire  [6:0] end_addr   = PACKET_MAX_LEN+((aurora_trig_addr-1)*(PACKET_MAX_LEN-1)); //15);

    //64bit
	(* mark_debug = DEBUG *) wire  [6:0] start_addr64 = (aurora_trig_addr-1)*PACKET_MAX_LEN; 
	
	assign InPacketDataOUT = 
		(address == 7'd0) ? a1 :
		(address == 7'd1) ? a2 :
		(address == 7'd2) ? a3 :
		(address == 7'd3) ? a4 :
		(address == 7'd4) ? a5 :
		(address == 7'd5) ? a6 :
		(address == 7'd6) ? a7 :
		(address == 7'd7) ? a8 :
		(address == 7'd8) ? a9 :
		(address == 7'd9) ? a10 :
		(address == 7'd10) ? a11 :
		(address == 7'd11) ? a12 :
		(address == 7'd12) ? a13 :
		(address == 7'd13) ? a14 :
		(address == 7'd14) ? a15 :
		//
		(address == 7'b0001111) ? a16 :
		(address == 7'b0010000) ? a17 :
		(address == 7'b0010001) ? a18 :
		(address == 7'b0010010) ? a19 :
		(address == 7'b0010011) ? a20 :
		(address == 7'b0010100) ? a21 :
		(address == 7'b0010101) ? a22 :
		(address == 7'b0010110) ? a23 :
		(address == 7'b0010111) ? a24 :
		(address == 7'b0011000) ? a25 :
		(address == 7'b0011001) ? a26 :
		(address == 7'b0011010) ? a27 :
		(address == 7'b0011011) ? a28 :
		(address == 7'b0011100) ? a29 :
		(address == 7'b0011101) ? a30 :
		(address == 7'b0011110) ? a31 :
		(address == 7'b0011111) ? a32 :
		(address == 7'b0100000) ? a33 :
		//a34;
		(address == 7'd33) ? a34 :
		(address == 7'd34) ? a35 :
		(address == 7'd35) ? a36 :
		(address == 7'd36) ? a37 :
		(address == 7'd37) ? a38 :
		(address == 7'd38) ? a39 :
		(address == 7'd39) ? a40 :
		(address == 7'd40) ? a41 :
		(address == 7'd41) ? a42 :
		(address == 7'd42) ? a43 :
		(address == 7'd43) ? a44 :
		(address == 7'd44) ? a45 :
		(address == 7'd45) ? a46 :	
		(address == 7'd46) ? a47 :
		(address == 7'd47) ? a48 :
		(address == 7'd48) ? a49 :
		(address == 7'd49) ? a50 :
		(address == 7'd50) ? a51 :
		(address == 7'd51) ? a52 :
		(address == 7'd52) ? a53 :
		(address == 7'd53) ? a54 :
		(address == 7'd54) ? a55 :
		(address == 7'd55) ? a56 :		
		(address == 7'd56) ? a57 :
		(address == 7'd57) ? a58 :
		(address == 7'd58) ? a59 :	
		(address == 7'd59) ? a60 :
		(address == 7'd60) ? a61 :
		(address == 7'd61) ? a62 :
		(address == 7'd62) ? a63 :
		(address == 7'd63) ? a64 :
		(address == 7'd64) ? a65 :
		(address == 7'd65) ? a66 :
		(address == 7'd66) ? a67 :
		a68;

    assign Relational_Operator1_out1 = (address >= start_addr) ? 1'b1 : 1'b0;
    assign Relational_Operator3_out1 = (address < end_addr)    ? 1'b1 : 1'b0; 
	//~auroraTxMtrig needed for reset timing
    assign Logical_Operator2_out1    =  Relational_Operator1_out1 & Relational_Operator3_out1 &~auroraTxMtrig;
	
    reg TLAST_t;
    always @ (posedge clk)
    begin: Counter_Free_Running1_process
        if (reset == 1'b1 || auroraTxMtrig == 1'b1) begin
            Counter_Free_Running2 <= 7'b0000000;
		    Counter_Free_Running1_out1 <= start_addr;
        end	  
        else begin
            if (enb == 1'b1) 
		    begin
                if (Counter_Free_Running1_out1 == 7'b1111111 ||  (Counter_Free_Running2 > (PACKET_MAX_LEN-1) )) begin
				    Counter_Free_Running1_out1 <= Counter_Free_Running1_out1;
            end
            else begin
                Counter_Free_Running1_out1 <= Counter_Free_Running1_out1 + 1; //address count
			    Counter_Free_Running2      <= Counter_Free_Running2 + 1;
            end
            end
        end
    end 

	
	(* mark_debug = DEBUG *) wire [DATA_SIZE-1:0] crc_out;
	LocalDataCRC crc_bpm (
		.data_in(TDATA), 
		.crc_en (TVALID & ~TLAST), 
		.rst    (auroraTxMtrig), 
		.clk    (clk), 
		.CRCout (crc_out)
	);
  
	  	  
	reg[31:0] data_reg[0:63];
	(* mark_debug = DEBUG *) reg [6:0] pktCnt;
	 
	// TLAST generator
	always @ (posedge clk) begin
		if (auroraTxMtrig == 1'b1) begin
			TLAST_t <= 1'b0;
			pktCnt  <= start_addr64;
		end		
		if (address == end_addr-2 ) begin  //14
		    if(enb == 1'b1) begin
			    TLAST_t <= 1'b1;  //TLAST set 1
			    if(TVALID == 1'b1) begin
			        data_reg[pktCnt] <= #1 TDATA;
					//data_reg[pktCnt] <= crc_out;
				    pktCnt <= pktCnt + 1;				
			    end	
			end	
		end	
		else if(address <= end_addr-1) begin
			if(TVALID == 1'b1 && enb == 1'b1) begin
			    data_reg[pktCnt] <= #1 TDATA;
			    pktCnt <= pktCnt + 1;
			end    
		end
	end
	
	//assign TDATA   = (TLAST_t == 0) ? InPacketDataOUT : crc_out;
	assign TDATA   = InPacketDataOUT;   //Without CRC
	
	
	assign TVALID  = Logical_Operator2_out1 & enb; //added enb
	assign TLAST   = TLAST_t;
    assign enb     = clk_enable;
    assign address = Counter_Free_Running1_out1;

	assign TDATA64 = 
		//SDI
		(pktCnt == 7'd1)  ? {data_reg[1], data_reg[0]} :
		(pktCnt == 7'd3)  ? {data_reg[3], data_reg[2]} :
		(pktCnt == 7'd5)  ? {data_reg[5], data_reg[4]} :
		(pktCnt == 7'd7)  ? {data_reg[7], data_reg[6]} :
		(pktCnt == 7'd9)  ? {data_reg[9], data_reg[8]} :
		(pktCnt == 7'd11) ? {data_reg[11],data_reg[10]} :
		(pktCnt == 7'd13) ? {data_reg[13],data_reg[12]} :
		(pktCnt == 7'd15) ? {data_reg[15],data_reg[14]} :

		(pktCnt == 7'd17) ? {data_reg[17], data_reg[16]} :
		(pktCnt == 7'd19) ? {data_reg[19], data_reg[18]} :
		(pktCnt == 7'd21) ? {data_reg[21], data_reg[20]} :
		(pktCnt == 7'd23) ? {data_reg[23], data_reg[22]} :
		(pktCnt == 7'd25) ? {data_reg[25], data_reg[24]} :
		(pktCnt == 7'd27) ? {data_reg[27], data_reg[26]} :
		(pktCnt == 7'd29) ? {data_reg[29], data_reg[28]} :
		(pktCnt == 7'd31) ? {data_reg[31], data_reg[30]} :		

		(pktCnt == 7'd33) ? {data_reg[33], data_reg[32]} :
		(pktCnt == 7'd35) ? {data_reg[35], data_reg[34]} :
		(pktCnt == 7'd37) ? {data_reg[37], data_reg[36]} :
		(pktCnt == 7'd39) ? {data_reg[39], data_reg[38]} :
		(pktCnt == 7'd41) ? {data_reg[41], data_reg[40]} :
		(pktCnt == 7'd43) ? {data_reg[43], data_reg[42]} :
		(pktCnt == 7'd45) ? {data_reg[45], data_reg[44]} :
		(pktCnt == 7'd47) ? {data_reg[47], data_reg[46]} :		

		(pktCnt == 7'd49) ? {data_reg[49], data_reg[48]} :
		(pktCnt == 7'd51) ? {data_reg[51], data_reg[50]} :
		(pktCnt == 7'd53) ? {data_reg[53], data_reg[52]} :
		(pktCnt == 7'd55) ? {data_reg[55], data_reg[54]} :
		(pktCnt == 7'd57) ? {data_reg[57], data_reg[56]} :
		(pktCnt == 7'd59) ? {data_reg[59], data_reg[58]} :
		(pktCnt == 7'd61) ? {data_reg[61], data_reg[60]} :
		(pktCnt == 7'd63) ? {data_reg[63], data_reg[62]} :	

        ////////////////////////////////////
		//LB+ format head is [63...32]
		/*
		(pktCnt == 7'd1)  ? { data_reg[0], data_reg[1]} :
		(pktCnt == 7'd3)  ? { data_reg[2], data_reg[3]} :
		(pktCnt == 7'd5)  ? { data_reg[4], data_reg[5]} :
		(pktCnt == 7'd7)  ? { data_reg[6], data_reg[7]} :
		(pktCnt == 7'd9)  ? { data_reg[8], data_reg[9]} :
		(pktCnt == 7'd11) ? { data_reg[10],data_reg[11]} :
		(pktCnt == 7'd13) ? { data_reg[12],data_reg[13]} :
		(pktCnt == 7'd15) ? { data_reg[14],data_reg[15]} :
		
		(pktCnt == 7'd17) ? {data_reg[16], data_reg[17]} :
		(pktCnt == 7'd19) ? {data_reg[18], data_reg[19]} :
		(pktCnt == 7'd21) ? {data_reg[20], data_reg[21]} :
		(pktCnt == 7'd23) ? {data_reg[22], data_reg[23]} :
		(pktCnt == 7'd25) ? {data_reg[24], data_reg[25]} :
		(pktCnt == 7'd27) ? {data_reg[26], data_reg[27]} :
		(pktCnt == 7'd29) ? {data_reg[28], data_reg[29]} :
		(pktCnt == 7'd31) ? {data_reg[30], data_reg[31]} :		
		
		(pktCnt == 7'd33) ? {data_reg[32], data_reg[33]} :
		(pktCnt == 7'd35) ? {data_reg[34], data_reg[35]} :
		(pktCnt == 7'd37) ? {data_reg[36], data_reg[37]} :
		(pktCnt == 7'd39) ? {data_reg[38], data_reg[39]} :
		(pktCnt == 7'd41) ? {data_reg[40], data_reg[41]} :
		(pktCnt == 7'd43) ? {data_reg[42], data_reg[43]} :
		(pktCnt == 7'd45) ? {data_reg[44], data_reg[45]} :
		(pktCnt == 7'd47) ? {data_reg[46], data_reg[47]} :		
		
		(pktCnt == 7'd49) ? {data_reg[48], data_reg[49]} :
		(pktCnt == 7'd51) ? {data_reg[50], data_reg[51]} :
		(pktCnt == 7'd53) ? {data_reg[52], data_reg[53]} :
		(pktCnt == 7'd55) ? {data_reg[54], data_reg[55]} :
		(pktCnt == 7'd57) ? {data_reg[56], data_reg[57]} :
		(pktCnt == 7'd59) ? {data_reg[58], data_reg[59]} :
		(pktCnt == 7'd61) ? {data_reg[60], data_reg[61]} :
		(pktCnt == 7'd63) ? {data_reg[62], data_reg[63]} :			
		*/
		64'h0000000000000000;  
				
	assign TVALID64 = 
		(pktCnt == 7'd1)  ? 1'b1 :
		(pktCnt == 7'd3)  ? 1'b1 :
		(pktCnt == 7'd5)  ? 1'b1 :
		(pktCnt == 7'd7)  ? 1'b1 :
		(pktCnt == 7'd9)  ? 1'b1 :
		(pktCnt == 7'd11) ? 1'b1 :
		(pktCnt == 7'd13) ? 1'b1 :
		(pktCnt == 7'd15) ? 1'b1 :
		//
		(pktCnt == 7'd17) ? 1'b1 :
		(pktCnt == 7'd19) ? 1'b1 :
		(pktCnt == 7'd21) ? 1'b1 :
		(pktCnt == 7'd23) ? 1'b1 :
		(pktCnt == 7'd25) ? 1'b1 :
		(pktCnt == 7'd27) ? 1'b1 :
		(pktCnt == 7'd29) ? 1'b1 :
		(pktCnt == 7'd31) ? 1'b1 :	
		//
		(pktCnt == 7'd33) ? 1'b1 :
		(pktCnt == 7'd35) ? 1'b1 :
		(pktCnt == 7'd37) ? 1'b1 :
		(pktCnt == 7'd39) ? 1'b1 :
		(pktCnt == 7'd41) ? 1'b1 :
		(pktCnt == 7'd43) ? 1'b1 :
		(pktCnt == 7'd45) ? 1'b1 :
		(pktCnt == 7'd47) ? 1'b1 :		
		//
		(pktCnt == 7'd49) ? 1'b1 :
		(pktCnt == 7'd51) ? 1'b1 :
		(pktCnt == 7'd53) ? 1'b1 :
		(pktCnt == 7'd55) ? 1'b1 :
		(pktCnt == 7'd57) ? 1'b1 :
		(pktCnt == 7'd59) ? 1'b1 :
		(pktCnt == 7'd61) ? 1'b1 :
		(pktCnt == 7'd63) ? 1'b1 :				
		1'b0; 

    //crc check
	(* mark_debug = DEBUG *) wire [3:0] bpm_crc_status;
	assign bpm_crc_status[0] = ((pktCnt == (PACKET_MAX_LEN-1)) & TLAST & (TDATA != TDATA64[31:0])) ? 1'b1 : 1'b0;
	//assign bpm_crc_status[1] = ((pktCnt == 31) & TLAST & (TDATA != TDATA64[31:0])) ? 1'b1 : 1'b0;
	//assign bpm_crc_status[2] = ((pktCnt == 47) & TLAST & (TDATA != TDATA64[31:0])) ? 1'b1 : 1'b0;
	//assign bpm_crc_status[3] = ((pktCnt == 63) & TLAST & (TDATA != TDATA64[31:0])) ? 1'b1 : 1'b0;
	
	assign mux_addr = address;	
		
endmodule  


