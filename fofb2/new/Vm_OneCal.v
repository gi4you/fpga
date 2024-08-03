`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/08/2024 04:19:53 PM
// Design Name: 
// Module Name: Vm_OneCal
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


module Vm_OneCal #(
    parameter DEBUG   = "false"
    )(
    input sysClk,
    input trig,
    (* mark_debug = DEBUG *)input s_ut_tvalid_t,
    //input s_ut_tlast_t,   //NOT needed
    input Ut_enable,
    input Ut_wren,
    input [8:0]  Ut_RamAddress,
    input [31:0] Ut_RamWData,    
    (* mark_debug = DEBUG *)input ut_outValid,
    (* mark_debug = DEBUG *)input [8:0] ut_ramRdAddr,
    (* mark_debug = DEBUG *)input [31:0] eigen_vec_in,
    (* mark_debug = DEBUG *)input eigen_vec_tvalid, 
    (* mark_debug = DEBUG *)input eigen_vec_tlast, 
    (* mark_debug = DEBUG *)output wire m_result_tvalid,
    (* mark_debug = DEBUG *)output wire m_result_tvalid2,
    output [31:0] ut_memread,
    output eigen_update_trig,
    (* mark_debug = DEBUG *)output [31:0] eigen_sum
    
    
    );
    
    
    (* mark_debug = DEBUG *)wire [31:0] ut_dpRamDataOut;    
    //V DPRAM 512
    V_PIDmem V_PIDmem_i (
        .clka  (sysClk),    // input wire clka
        .ena   (Ut_enable),      // input wire ena
        .wea   (Ut_wren),      // input wire [0 : 0] wea
        .addra (Ut_RamAddress[8:0]),  // input wire [8 : 0] addra
        .dina  (Ut_RamWData),    // input wire [31 : 0] dina
        .douta (ut_memread),  // output wire [31 : 0] douta
        .clkb  (sysClk),    // input wire clkb
        .enb   (ut_outValid),      // input wire enb
        .web   (1'b0),      // input wire [0 : 0] web
        .addrb (ut_ramRdAddr[8:0]),  // input wire [8 : 0] addrb
        .dinb  (32'd0),    // input wire [31 : 0] dinb
        .doutb (ut_dpRamDataOut)  // output wire [31 : 0] doutb
    );


    ///////////////////////////////////
    (* mark_debug = DEBUG *) wire s_axis_a_tready;
    (* mark_debug = DEBUG *) wire m_result_tlast2;
    
    (* mark_debug = DEBUG *) wire [31:0] ut_multOut;
    (* mark_debug = DEBUG *) wire m_axis_mult_result_tlast;
    floating_Mult floating_Mult_Ut_i (
        .aclk                (sysClk),                // input wire aclk
        .s_axis_a_tvalid     (eigen_vec_tvalid),       // input wire s_axis_a_tvalid
        .s_axis_a_tready     (s_axis_a_tready),       // output wire s_axis_a_tready
        .s_axis_a_tdata      (eigen_vec_in),             // input wire [31 : 0] s_axis_a_tdata
        .s_axis_a_tlast      (eigen_vec_tlast),
        //
        .s_axis_b_tvalid     (s_ut_tvalid_t),         // input wire s_axis_b_tvalid
        .s_axis_b_tready     (),                      // output wire s_axis_b_tready
        .s_axis_b_tdata      (ut_dpRamDataOut),       // input wire [31 : 0] s_axis_b_tdata
        //
        .m_axis_result_tvalid(m_result_tvalid),       // output wire m_axis_result_tvalid
        .m_axis_result_tready(1'b1),                  // input wire m_axis_result_tready
        .m_axis_result_tdata (ut_multOut),             // output wire [31 : 0] m_axis_result_tdata
        .m_axis_result_tlast (m_axis_mult_result_tlast)
    );
    

    (* mark_debug = DEBUG *) wire [31:0] eigen;
    // Ut[0][n] * PosError[n]
    floating_accm floating_accm_i (
        .aclk(sysClk),                                  // input wire aclk
        .s_axis_a_tvalid(m_result_tvalid),            // input wire s_axis_a_tvalid
        .s_axis_a_tready(),            // output wire s_axis_a_tready
        .s_axis_a_tdata(ut_multOut),              // input wire [31 : 0] s_axis_a_tdata
        //.s_axis_a_tlast(s_ut_tlast_t), //s_ut_acc_tlast_t),              // input wire s_axis_a_tlast
        .s_axis_a_tlast(m_axis_mult_result_tlast),
        //
        .m_axis_result_tvalid(m_result_tvalid2),  // output wire m_axis_result_tvalid
        .m_axis_result_tready(1'b1),  // input wire m_axis_result_tready
        .m_axis_result_tdata(eigen),    // output wire [31 : 0] m_axis_result_tdata
        .m_axis_result_tlast(m_result_tlast2)    // output wire m_axis_result_tlast
    ); 

    (* mark_debug = DEBUG *) wire eigen_tlast_valid;
    assign eigen_tlast_valid = m_result_tlast2 && m_result_tvalid2;
    
    reg update_eigen_reg;
    (* mark_debug = DEBUG *) reg update_trig;
    always @ (posedge sysClk) begin        
        update_eigen_reg <= eigen_tlast_valid;
        update_trig <= update_eigen_reg;
	end	
		
	    
    //taking accumulator sum
    reg [31:0] eigen_sum_reg;
    always @ (posedge sysClk) begin
        if(eigen_tlast_valid == 1'b1) begin
            eigen_sum_reg <= eigen;
        end
        else eigen_sum_reg <= eigen_sum_reg;
    end
    assign eigen_sum = eigen_sum_reg;
    
    
    assign eigen_update_trig  = update_trig; 
    
endmodule
