`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 0/26/2022 02:19:58 AM
// Design Name: 
// Module Name: evrTOP
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

// 5/27/23 for ZCU216
// Tx/Rx buffer bypass


module zcu216_evrTOP (
    input sysClk,
    input reset,
    
//	input evr_refclk_p,
//	input evr_refclk_n,
    input refclk,
    
    // Serial data ports for transceiver channel 0
    input  wire gtyrxn_in,
    input  wire gtyrxp_in,
    output wire gtytxn_out,
    output wire gtytxp_out,
  
    input wire [7:0] DMATrigEventSet,
    (* mark_debug = "true" *) output wire [0:0]  evr_rxcommadet,
    (* mark_debug = "true" *) output wire [15:0] evr_userdata_rx,
    output wire [4:0] status, //gtpowergood,
    
    output wire timeSync_out,
    output wire refclk_o,
    output wire RxCLK,
    
	//
    output wire dma_evt_trig,
    
    output wire [63:0] TimeStamp,
    output wire [31:0] Seconds,
    output wire [31:0] Offset,
    (* mark_debug = "false" *) output wire [7:0]  EventStream,
    (* mark_debug = "false" *) output wire [7:0]  DBUSStream,
    output fa_event_sys	    
    );
    
//
localparam EVR_PPS_EVENT	    = 8'h7A;
localparam EVR_P0_EVENT		    = 8'd129;
localparam EVR_FA_EVENT		    = 8'd48;	//FOFB



wire [0:0] drpclk_int;
wire [0:0] ch0_drpclk_int;
assign drpclk_int[0:0] = ch0_drpclk_int;
assign ch0_drpclk_int = sysClk;
  
wire [0:0] gtrefclk0_int;
wire [0:0] ch0_gtrefclk0_int;
assign gtrefclk0_int[0:0] = ch0_gtrefclk0_int;

//--------------------------------------------------------------------------------------------------------------------
wire [0:0] gtyrxn_int;
assign gtyrxn_int[0:0] = gtyrxn_in;

//--------------------------------------------------------------------------------------------------------------------
wire [0:0] gtyrxp_int;
assign gtyrxp_int[0:0] = gtyrxp_in;

//--------------------------------------------------------------------------------------------------------------------
wire [0:0] gtytxn_int;
assign gtytxn_out = gtytxn_int[0:0];

//--------------------------------------------------------------------------------------------------------------------
wire [0:0] gtytxp_int;
assign gtytxp_out = gtytxp_int[0:0];   


(* mark_debug = "true" *)wire [15:0] rxctrl0;
(* mark_debug = "true" *)wire [15:0] rxctrl1;
(* mark_debug = "true" *)wire [7:0] rxctrl2;
(* mark_debug = "true" *)wire [7:0] rxctrl3; 


    wire [0:0] TxCLK;   


    assign ch0_gtrefclk0_int = refclk;
  
  
    wire [0:0] gtwiz_userclk_rx_reset_int;
    wire [0:0] hb0_gtwiz_userclk_rx_reset_int;
    assign gtwiz_userclk_rx_reset_int[0:0] = hb0_gtwiz_userclk_rx_reset_int;

    wire [0:0] gtwiz_userclk_tx_reset_int;
    wire [0:0] hb0_gtwiz_userclk_tx_reset_int;
    assign gtwiz_userclk_tx_reset_int[0:0] = hb0_gtwiz_userclk_tx_reset_int;


  
    wire [0:0] rxpmaresetdone_int;
    wire [0:0] txpmaresetdone_int;
    // ===================================================================================================================
    // USER CLOCKING RESETS
    // ===================================================================================================================

    // The TX user clocking helper block should be held in reset until the clock source of that block is known to be
    // stable. The following assignment is an example of how that stability can be determined, based on the selected TX
    // user clock source. Replace the assignment with the appropriate signal or logic to achieve that behavior as needed.
    assign hb0_gtwiz_userclk_tx_reset_int = ~(&txpmaresetdone_int);

    // The RX user clocking helper block should be held in reset until the clock source of that block is known to be
    // stable. The following assignment is an example of how that stability can be determined, based on the selected RX
    // user clock source. Replace the assignment with the appropriate signal or logic to achieve that behavior as needed.
    assign hb0_gtwiz_userclk_rx_reset_int = ~(&rxpmaresetdone_int);



    wire reset_tx_done_out;
    wire reset_rx_done_out;
  

  // ===================================================================================================================
  // BUFFER BYPASS CONTROLLER RESETS
  // ===================================================================================================================
  wire gtwiz_buffbypass_rx_reset_int;
  wire gtwiz_userclk_rx_active_int;
  wire gtwiz_buffbypass_tx_done_int;
  wire gtwiz_userclk_tx_active_int;
  wire gtwiz_buffbypass_tx_reset_int;
  wire gtwiz_buffbypass_rx_done_int;
  wire gtwiz_buffbypass_tx_error_int;
  wire gtwiz_buffbypass_rx_error_int;
  
  
  // The TX buffer bypass controller helper block should be held in reset until the TX user clocking network helper
  // block which drives it is active
  (* DONT_TOUCH = "TRUE" *)
  GTY_evr0_example_reset_synchronizer reset_synchronizer_gtwiz_buffbypass_tx_reset_inst (
    .clk_in  (TxCLK),
    .rst_in  (~gtwiz_userclk_tx_active_int),
    .rst_out (gtwiz_buffbypass_tx_reset_int)
  );

  // The RX buffer bypass controller helper block should be held in reset until the RX user clocking network helper
  // block which drives it is active and the TX buffer bypass sequence has completed for this loopback configuration
  (* DONT_TOUCH = "TRUE" *)
  GTY_evr0_example_reset_synchronizer reset_synchronizer_gtwiz_buffbypass_rx_reset_inst (
    .clk_in  (RxCLK),
    .rst_in  (~gtwiz_userclk_rx_active_int || ~gtwiz_buffbypass_tx_done_int),
    .rst_out (gtwiz_buffbypass_rx_reset_int)
  );
  
  
 GTY_evr0_example_wrapper GTY_evr0_example_wrapper_i (
    .gtyrxn_in                               (gtyrxn_int)
   ,.gtyrxp_in                               (gtyrxp_int)
   ,.gtytxn_out                              (gtytxn_int)
   ,.gtytxp_out                              (gtytxp_int)
   //
   ,.gtwiz_userclk_tx_reset_in               (gtwiz_userclk_tx_reset_int)
   ,.gtwiz_userclk_tx_srcclk_out             ()
   ,.gtwiz_userclk_tx_usrclk_out             ()
   ,.gtwiz_userclk_tx_usrclk2_out            (TxCLK)
   ,.gtwiz_userclk_tx_active_out             (gtwiz_userclk_tx_active_int)
   ,.gtwiz_userclk_rx_reset_in               (gtwiz_userclk_rx_reset_int)
   ,.gtwiz_userclk_rx_srcclk_out             ()
   ,.gtwiz_userclk_rx_usrclk_out             ()
   ,.gtwiz_userclk_rx_usrclk2_out            (RxCLK)
   ,.gtwiz_userclk_rx_active_out             (gtwiz_userclk_rx_active_int)
   //
   //tx/rx buffer bypass
   ,.gtwiz_buffbypass_tx_reset_in            (gtwiz_buffbypass_tx_reset_int)
   ,.gtwiz_buffbypass_tx_start_user_in       (1'b0)
   ,.gtwiz_buffbypass_tx_done_out            (gtwiz_buffbypass_tx_done_int)
   ,.gtwiz_buffbypass_tx_error_out           (gtwiz_buffbypass_tx_error_int)
   //
   ,.gtwiz_buffbypass_rx_reset_in            (gtwiz_buffbypass_rx_reset_int)
   ,.gtwiz_buffbypass_rx_start_user_in       (1'b0)
   ,.gtwiz_buffbypass_rx_done_out            (gtwiz_buffbypass_rx_done_int)
   ,.gtwiz_buffbypass_rx_error_out           (gtwiz_buffbypass_rx_error_int)
   //
   //
   ,.gtwiz_reset_clk_freerun_in              (sysClk)
   ,.gtwiz_reset_all_in                      (reset)
   //
   ,.gtwiz_reset_tx_pll_and_datapath_in      (1'b0)
   ,.gtwiz_reset_tx_datapath_in              (1'b0)
   ,.gtwiz_reset_rx_pll_and_datapath_in      ({1{1'b0}})
   ,.gtwiz_reset_rx_datapath_in              ({1{1'b0}})
   ,.gtwiz_reset_rx_cdr_stable_out           ()
   ,.gtwiz_reset_tx_done_out                 (reset_tx_done_out)
   ,.gtwiz_reset_rx_done_out                 (reset_rx_done_out)
   ,.gtwiz_userdata_tx_in                    (16'd0)
   ,.gtwiz_userdata_rx_out                   (evr_userdata_rx)
   //
   ,.drpclk_in                               (drpclk_int)
   ,.gtrefclk0_in                            (gtrefclk0_int)
   //
   ,.rx8b10ben_in                            (1'b1)
   ,.rxcommadeten_in                         (1'b1)
   ,.rxmcommaalignen_in                      (1'b1)
   ,.rxpcommaalignen_in                      (1'b1)
   ,.tx8b10ben_in                            (1'b1)
   ,.txctrl0_in                              (16'd0)
   ,.txctrl1_in                              (16'd0)
   ,.txctrl2_in                              (8'd0)
   //
   ,.gtpowergood_out                         (gtpowergood)
   ,.rxbyteisaligned_out                     ()
   ,.rxbyterealign_out                       ()
   ,.rxcommadet_out                          (evr_rxcommadet)
   ,.rxctrl0_out                             (rxctrl0)
   ,.rxctrl1_out                             (rxctrl1)
   ,.rxctrl2_out                             (rxctrl2)
   ,.rxctrl3_out                             (rxctrl3)
   ,.rxpmaresetdone_out                      (rxpmaresetdone_int)
   ,.txpmaresetdone_out                      (txpmaresetdone_int)
   ,.txprgdivresetdone_out                   (txprgdivresetdone_int)
);

    (* mark_debug = "true" *) wire rxctrl0_0, rxctrl2_0; 
    assign rxctrl0_0 = rxctrl0[0];
    assign rxctrl2_0 = rxctrl2[0];
    
    assign status[0] = gtpowergood;
    assign status[1] = reset_tx_done_out;
    assign status[2] = reset_rx_done_out;
    assign status[3] = rxpmaresetdone_int[0];
    assign status[4] = txpmaresetdone_int[0];
    /////
    
    
	assign EventStream = (rxctrl3 == 8'hfc) ? evr_userdata_rx[7:0] : 8'h00;
	assign DBUSStream  = (rxctrl3 == 8'hfc) ? evr_userdata_rx[15:8] : 8'h00;    
    
    
	wire [63:0] timestamp_in;
	timeofDayReceiver timeofDayReceiver_i (
		.Clock         (RxCLK), 
		.Reset         (reset), 
		.EventStream   (EventStream), 
		.TimeStamp     (timestamp_in), //TimeStamp), 
		.Seconds       (Seconds), 
		.Offset        (Offset), 
		.Position      (), 
		.eventClock    ()
	);
    
    sync_data  #(
        .NUM_OF_BITS(64),
        .ASYNC_CLK(1)
    ) sync_timestamp_i (
        .in_clk  (RxCLK),
        .in_data (timestamp_in),
        .out_clk (sysClk),
        .out_data(TimeStamp)
    );
    

	wire timeSync_event;
	assign timeSync_event = (evr_userdata_rx[7:0] == 8'h7D) ? 1'b1 : 1'b0;
	
	//wire timeSync_out;
	gateDelayFast timeSync_event_ (
		.Clk  (RxCLK), 
		.Inp  (timeSync_event),
		.Delay(32'd1),
		.Width(32'd12500000),
		.Q    (timeSync_out)
	);


    (* mark_debug = "true" *) wire dma_evt_trig_i;
    //assign tbt_evt_trig = evr_userdata_rx[8];
    assign dma_evt_trig_i     = (EventStream[7:0] == DMATrigEventSet[7:0]) ? 1'b1 : 1'b0;
    
    //11/04/22 added
    gateDelayFast evr_trig_delay_i (
        .Clk  (RxCLK),
        .Inp  (dma_evt_trig_i),
        .Delay(32'd1),
        .Width(32'd10), //5 ticks, changed to 10 ticks sloved ADI DMA trigger count
        .Q    (dma_evt_trig)
    );    
    
    //assign dma_evt_trig = dma_evt_trig_i;
    
    wire fa_event;
    assign fa_event =  evr_userdata_rx[7:0] == 8'd31 ? 1'b1: 1'b0;
    irq_forward irq_forward_fa_trig_i (RxCLK, fa_event, sysClk, fa_event_sys);
    
    
endmodule
