`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/05/2024 09:24:51 PM
// Design Name: 
// Module Name: us_glb_gtx_top
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


module us_glb_gtx_top#(
    parameter DEBUG           = "false"
    )(
    input  wire [0:0] gtyrxn,
    input  wire [0:0] gtyrxp,
    output wire [0:0] gtytxn,
    output wire [0:0] gtytxp,

    input refclk,
    output TxClock,
    output userclk_rx_usrclk2,
    input  [31:0] userdata_tx,
    input  [7:0]  userdata_tx_k,
        
    //
    input trig,
    input CRC_reset,
    input sysClk,
    input reset,
    output wire URxCharIsKOut,
	(* mark_debug  = DEBUG *)output [15:0] LinkDataAddress,	// address from data packet being processed
	output [11:0] LinkPacketAddress,// address for forwarding packet if cell controller
	(* mark_debug  = DEBUG *)output [11:0] LinkPacketLength,	// legth of the data packet being processed
	(* mark_debug  = DEBUG *)output [31:0] LinkData,	 			// data from the packet being processed
	(* mark_debug  = DEBUG *)output LinkDataValid,				// data valid to user application
	(* mark_debug  = DEBUG *)output LinkStartOfPacket,
	output LinkEndOfPacket,
    output [31:0] CWCRCErrorCount,    
    
    output sdi_tvalid,
    output [9:0] sdi_taddr,
    output sdi_tlast,
    (* mark_debug  = DEBUG *)output wire [9:0] status    
    );
    
    
    (* mark_debug = DEBUG *)wire [31:0] CWRxDataOut;
    (* mark_debug = DEBUG *)wire [3:0] RxCharIsKOut;
    (* mark_debug = DEBUG *)wire rxcommadet;
    
    
    gty0_top gty0_top_cw (
        .gtyrxn (gtyrxn),
        .gtyrxp (gtyrxp),
        .gtytxn (gtytxn),
        .gtytxp (gtytxp),

        .gtrefclk0 (refclk),
        .userclk_tx_usrclk2 (TxClock),
        .userclk_rx_usrclk2 (userclk_rx_usrclk2),
        .userdata_tx   (userdata_tx),    //tx data
        .userdata_tx_k (userdata_tx_k),  //tx K data
        .userdata_rx   (CWRxDataOut),
        .RxCharIsKOut  (RxCharIsKOut),   //Not valid ?
        .rxcommadet    (rxcommadet),
        //
        .sysClk (sysClk),
        .reset  (reset),
        .status (status)    
    );    
    
    wire gtxTxReset = 1'b0;
    (* mark_debug = DEBUG *)wire CWRxCharIsKOut;
	wire CWmyReceivedLocalPacketHead;	
	wire CWmyReceivedLength;
    
    //added head comparator
    assign CWRxCharIsKOut  = (CWRxDataOut == 32'h30E0005C) ? 1'b1 : 1'b0;  //Global SDI data
    //assign CWRxCharIsKOut  = RxCharIsKOut[0];
    assign URxCharIsKOut   = CWRxCharIsKOut;
    
	/*
	*	Local Packet timeout check.
	*/		 
	assign CWmyReceivedLocalPacketHead  = ((CWRxCharIsKOut)  && ( CWRxDataOut[7:0] == 8'h5C) &&  (CWRxDataOut[19:8] == 12'd0));
	assign CWmyReceivedLength  = ((CWRxCharIsKOut)  && ( CWRxDataOut[7:0] == 8'h5C) &&  (CWRxDataOut[31:20] == 12'd1024));

    
	reg [10:0] cw_addr_cnt, RxLength;	
	reg rx_data_valid;
	always @ (posedge userclk_rx_usrclk2)
	begin
		//if (Reset == 1'b1 || CWmyReceivedLocalPacketHead == 1'b1) begin
        if (CWmyReceivedLocalPacketHead == 1'b1) begin
			cw_addr_cnt <= 11'd0;
			rx_data_valid <= 1'b1;
			RxLength <= CWRxDataOut[31:20];
		end	
		else if (cw_addr_cnt <= RxLength ) begin
			cw_addr_cnt <= cw_addr_cnt + 1;
			rx_data_valid <= 1'b1;
		end	
		else begin
			cw_addr_cnt <= cw_addr_cnt;
			rx_data_valid <= 1'b0;
		end	
	end
	 
	// Remote Packet received
	//wire IdleBit;
	wire [15:0] MyStopAddress;
	(* mark_debug  = DEBUG *) wire [33:0] CWRemoteBufferDataOut;
	wire [31:0] CRCErrorCount;
	(* mark_debug = DEBUG *)wire RemoteBufferFlush;
	(* mark_debug = DEBUG *)wire RemotePacketReady, RemoteBufferEmpty;
	(* mark_debug = DEBUG *)reg RemotePacketsInQue;
    wire CRCErrorPacketDropped;
    wire cwRxClk_trig;
    irq_forward irq_forward_0 (sysClk, trig, userclk_rx_usrclk2, cwRxClk_trig);
	
    // This subdesign implements the interface to remote data sources
	RemotePacketProcessor #(
		.ILA_ENABLED (0)
	)CWRemotePacketProcessor (
		.Reset    (1'b0),		 
		.CRC_reset(CRC_reset),
		.Trigger  (cwRxClk_trig), 
		.Idle( 1'b0 ),
		.RioRxClock(userclk_rx_usrclk2), 
		.RxData    (CWRxDataOut), 
		.RxCharIsK (CWRxCharIsKOut),
		.MyStopAddress(16'd0),		 
		.RioTxClock    (TxClock), 		        //Fifo read clock
		.TxBufferFlush (RemoteBufferFlush), 
		.TxData        (CWRemoteBufferDataOut), //remote data out. [33:0] for tx
`ifdef	RXFIFO_MONITORING		 
		.RemoteRxFifoData(),
		.RemoteRxFifoDataValid(),		 
`endif		 
		.PacketReady(RemotePacketReady), 
		.TxBufferEmpty(RemoteBufferEmpty), 
		.PacketDropped(CRCErrorPacketDropped),
		.CRCErrorCount (CRCErrorCount),
		.LocalPacketRcvdOnRemote(),	
		.RemoteModeDone(RemoteModeDone),
		.LocalRxPacketDataValid(),
		.RxFifoEmpty_o(),	 
		.myReceivedLocalPacketHead(),
		//RX data without CRC, it used for test...
		.TxPacketBufferData(),
		.TxFifoWriteEnable(),
		.ReStateDbg(ReStateDbg)
		
    );
	 
	 
	// Remote Packets in Q
	always @ (posedge TxClock)
	begin
		if (gtxTxReset == 1'b1) 
			RemotePacketsInQue <= 1'b0;
		
		else if (RemotePacketReady == 1'b1)
			RemotePacketsInQue <= 1'b1;
			
		else if (RemoteBufferEmpty == 1'b1)
			RemotePacketsInQue <= 1'b0;
			
		else
			RemotePacketsInQue <= RemotePacketsInQue;
	end
	assign RemoteBufferFlush = RemotePacketsInQue;
	 
			
    // This subdesign pulls the data from the outgoing stream 
    // and writes it to the local user interface
    StreamDataInterface myStreamDataInterface (
        .Clock  (TxClock), 
        .Reset  (gtxTxReset), 
        .DataIn (CWRemoteBufferDataOut[33:2]), 
        .CharIsK(CWRemoteBufferDataOut[1]), 
        .MemoryAddress(LinkDataAddress),
        .PacketAddress(LinkPacketAddress),
        .PacketLength (LinkPacketLength), 
        .DataOut      (LinkData), 
        .DataValid    (LinkDataValid),
        .LinkStartOfPacket(LinkStartOfPacket),
        .LinkEndOfPacket  (LinkEndOfPacket),
        
        .sdi_tvalid (sdi_tvalid),
        .sdi_taddr  (sdi_taddr),
        .sdi_tlast  (sdi_tlast)        	 
    );


    wire [3:0] dbg;
	assign dbg = {CWmyReceivedLocalPacketHead, RemoteBufferEmpty, RemoteBufferFlush, RemotePacketReady};
	
	
	reg auroraTxValidereg;
	always @ (posedge TxClock)
	begin
		auroraTxValidereg <= RemoteBufferFlush;
	end	
	
    wire [31:0] auroraTxData;
    wire auroraTxValide;
    
    
	assign auroraTxData   = CWRemoteBufferDataOut;
	assign auroraTxValide = auroraTxValidereg;    


    sync_data  #(
        .NUM_OF_BITS(32),
        .ASYNC_CLK(1)
    ) sync_timestamp_i (
        .in_clk  (TxClock),
        .in_data (CRCErrorCount),
        .out_clk (sysClk),
        .out_data(CWCRCErrorCount)
    );
        
    
endmodule
