`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/27/2023 03:10:09 PM
// Design Name: 
// Module Name: gty0_top
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


module gty0_top(
    input  wire [0:0] gtyrxn,
    input  wire [0:0] gtyrxp,
    output wire [0:0] gtytxn,
    output wire [0:0] gtytxp,

    input  gtrefclk0,
    output userclk_tx_usrclk2,
    output userclk_rx_usrclk2,
    input  [31:0] userdata_tx,
    input  [7:0] userdata_tx_k,
    output [31:0] userdata_rx,
    output 	wire [3:0]  RxCharIsKOut,
    output wire rxcommadet,
    //
    input sysClk,
    input reset,
    input gty_reset,
    (* mark_debug  = "true" *)output wire [9:0] status    
    );
    


(* mark_debug  = "true" *)wire [0:0] gtwiz_userclk_tx_active_int;
(* mark_debug  = "true" *)wire [0:0] gtwiz_userclk_rx_active_int;
(* mark_debug  = "true" *)wire [0:0] gtwiz_reset_rx_cdr_stable_int;
(* mark_debug  = "true" *)wire [0:0] gtwiz_reset_tx_done_int;
(* mark_debug  = "true" *)wire [0:0] gtwiz_reset_rx_done_int;
(* mark_debug  = "true" *)wire [0:0] gtpowergood_int;
(* mark_debug  = "true" *)wire [15:0] rxctrl0_int;
(* mark_debug  = "true" *)wire [15:0] rxctrl1_int;
(* mark_debug  = "true" *)wire [7:0]  rxctrl2_int;
(* mark_debug  = "true" *)wire [7:0]  rxctrl3_int;
(* mark_debug  = "true" *)wire [0:0] rxpmaresetdone_int;
(* mark_debug  = "true" *)wire [0:0] txpmaresetdone_int;  
(* mark_debug  = "true" *)wire [0:0] rxbyteisaligned_int;
(* mark_debug  = "true" *)wire [0:0] rxbyterealign_int;  
  
(* mark_debug  = "true" *)wire [0:0] gtwiz_userclk_tx_reset_int;
(* mark_debug  = "true" *)wire [0:0] gtwiz_userclk_rx_reset_int;
(* mark_debug  = "true" *)wire [0:0] rxcommadet_int;
  
  // The TX user clocking helper block should be held in reset until the clock source of that block is known to be
  // stable. The following assignment is an example of how that stability can be determined, based on the selected TX
  // user clock source. Replace the assignment with the appropriate signal or logic to achieve that behavior as needed.
  assign gtwiz_userclk_tx_reset_int = ~(&txpmaresetdone_int);

  // The RX user clocking helper block should be held in reset until the clock source of that block is known to be
  // stable. The following assignment is an example of how that stability can be determined, based on the selected RX
  // user clock source. Replace the assignment with the appropriate signal or logic to achieve that behavior as needed.
  assign gtwiz_userclk_rx_reset_int = ~(&rxpmaresetdone_int);

wire gtwiz_reset_all_init_int;
wire gtwiz_reset_rx_datapath_init_int;
   // The example initialization module interacts with the reset controller helper block and other example design logic
  // to retry failed reset attempts in order to mitigate bring-up issues such as initially-unavilable reference clocks
  // or data connections. It also resets the receiver in the event of link loss in an attempt to regain link, so please
  // note the possibility that this behavior can have the effect of overriding or disturbing user-provided inputs that
  // destabilize the data stream. It is a demonstration only and can be modified to suit your system needs.
  GTY_sdi0_example_init example_init_inst (
    .clk_freerun_in  (sysClk),
    .reset_all_in    (reset),
    .tx_init_done_in (gtwiz_reset_tx_done_int),
    .rx_init_done_in (gtwiz_reset_rx_done_int),
    .rx_data_good_in (1'b1),
    .reset_all_out   (gtwiz_reset_all_init_int),
    .reset_rx_out    (gtwiz_reset_rx_datapath_init_int),
    .init_done_out   (),
    .retry_ctr_out   ()
  );

  
  // Instantiate the example design wrapper, mapping its enabled ports to per-channel internal signals and example
  // resources as appropriate
  GTY_sdi0_example_wrapper example_wrapper_inst (
    .gtyrxn_in                               (gtyrxn)
   ,.gtyrxp_in                               (gtyrxp)
   ,.gtytxn_out                              (gtytxn)
   ,.gtytxp_out                              (gtytxp)
   // tx
   ,.gtwiz_userclk_tx_reset_in               (gtwiz_userclk_tx_reset_int || gty_reset)
   ,.gtwiz_userclk_tx_srcclk_out             ()
   ,.gtwiz_userclk_tx_usrclk_out             ()
   ,.gtwiz_userclk_tx_usrclk2_out            (userclk_tx_usrclk2)
   ,.gtwiz_userclk_tx_active_out             (gtwiz_userclk_tx_active_int)
   // rx
   ,.gtwiz_userclk_rx_reset_in               (gtwiz_userclk_rx_reset_int)
   ,.gtwiz_userclk_rx_srcclk_out             ()
   ,.gtwiz_userclk_rx_usrclk_out             ()
   ,.gtwiz_userclk_rx_usrclk2_out            (userclk_rx_usrclk2)
   ,.gtwiz_userclk_rx_active_out             (gtwiz_userclk_rx_active_int)
   //
   ,.gtwiz_reset_clk_freerun_in              (sysClk)
   ,.gtwiz_reset_all_in                      (gtwiz_reset_all_init_int)
   //
   ,.gtwiz_reset_tx_pll_and_datapath_in      (gty_reset)  //1'b0)
   ,.gtwiz_reset_tx_datapath_in              (gty_reset) //1'b0)
   ,.gtwiz_reset_rx_pll_and_datapath_in      ({1{gtwiz_reset_rx_datapath_init_int}})
   ,.gtwiz_reset_rx_datapath_in              (gty_reset) //{1{1'b0}})
   // data
   ,.gtwiz_userdata_tx_in                    (userdata_tx)
   ,.gtwiz_userdata_rx_out                   (userdata_rx)
   //
   ,.gtwiz_reset_rx_cdr_stable_out           (gtwiz_reset_rx_cdr_stable_int)
   ,.gtwiz_reset_tx_done_out                 (gtwiz_reset_tx_done_int)
   ,.gtwiz_reset_rx_done_out                 (gtwiz_reset_rx_done_int)   
   //
   ,.drpclk_in                               (sysClk)
   ,.gtrefclk0_in                            (gtrefclk0)
   //
   ,.rx8b10ben_in                            (1'b1)
   ,.rxcommadeten_in                         (1'b1)
   ,.rxmcommaalignen_in                      (1'b1)
   ,.rxpcommaalignen_in                      (1'b1)
   ,.tx8b10ben_in                            (1'b1)
   ,.txctrl0_in                              (16'd0)
   ,.txctrl1_in                              (16'd0)
   ,.txctrl2_in                              (userdata_tx_k) //8'd0)
   //status output
   ,.gtpowergood_out                         (gtpowergood_int)
   ,.rxbyteisaligned_out                     (rxbyteisaligned_int)
   ,.rxbyterealign_out                       (rxbyterealign_int)
   ,.rxcommadet_out                          (rxcommadet_int)
   ,.rxctrl0_out                             (rxctrl0_int)
   ,.rxctrl1_out                             (rxctrl1_int)
   ,.rxctrl2_out                             (rxctrl2_int)
   ,.rxctrl3_out                             (rxctrl3_int)
   ,.rxpmaresetdone_out                      (rxpmaresetdone_int)
   ,.txpmaresetdone_out                      (txpmaresetdone_int)
);

assign RxCharIsKOut[0] =  rxctrl0_int[0];
assign RxCharIsKOut[3:1] = 3'b000; 
assign rxcommadet = rxcommadet_int[0]; 

//
assign status[0] = gtpowergood_int;
assign status[1] = gtwiz_reset_tx_done_int;  
assign status[2] = gtwiz_reset_rx_done_int;
assign status[3] = rxpmaresetdone_int;  
assign status[4] = txpmaresetdone_int;
assign status[5] = gtwiz_userclk_tx_active_int;
assign status[6] = gtwiz_userclk_rx_active_int;
assign status[7] = gtwiz_reset_all_init_int;
assign status[8] = gtwiz_reset_rx_datapath_init_int;


endmodule

