`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/07/2024 09:25:17 PM
// Design Name: 
// Module Name: Ut_OneCal
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
/*
6/23/24
     added D = eigen_sum_reg - eigen_sum_delta
     

 */

module Ut_OneCal #(
    parameter DEBUG           = "false"
    )(
    input sysClk,
    (* mark_debug = DEBUG *)input trig,
    (* mark_debug = DEBUG *)input s_ut_tvalid_t,
    (* mark_debug = DEBUG *)input s_ut_tlast_t,   //NOT needed
    input Ut_enable,
    input Ut_wren,
    input Ut_RamB_Out_enable,
    input [9:0]  Ut_RamAddress,
    input [31:0] Ut_RamWData,    
    (* mark_debug = DEBUG *)input ut_outValid,
    (* mark_debug = DEBUG *)input [8:0] ut_ramRdAddr,
    (* mark_debug = DEBUG *)input [31:0] fposError,
    (* mark_debug = DEBUG *)input posError_tvalid, 
    (* mark_debug = DEBUG *)input posError_tlast,
    input [31:0] SGain_Set, 
    input eigen_d_sel,
    (* mark_debug = DEBUG *)output wire m_result_tvalid,
    (* mark_debug = DEBUG *)output wire m_result_tvalid2,
    (* mark_debug = DEBUG *)output [31:0] ut_memread,
    output eigen_update_trig,
    (* mark_debug = DEBUG *)output [31:0] eigen_sum,
    (* mark_debug = DEBUG *)output [31:0] eigen_sum_integral,
    (* mark_debug = DEBUG *)output wire [31:0] eigen_sum_delta
    
    
    );
    
    (* mark_debug = DEBUG *)wire [31:0] ut_dpRamDataOut;
    
    //Ut DPRAM
    Ut_DPRAM Ut_DPRAM_i (
        .clka (sysClk),         // input wire clka
        .ena  (Ut_enable),      // input wire ena
        .wea  (Ut_wren),        // input wire [0 : 0] wea
        .addra(Ut_RamAddress[9:0]),  // input wire [9 : 0] addra
        .dina (Ut_RamWData),    // input wire [31 : 0] dina
        .douta(ut_memread),     // output wire [31 : 0] douta
        //
        .clkb (sysClk),    // input wire clkb
        .enb  (ut_outValid & Ut_RamB_Out_enable),      // input wire enb
        .web  (1'b0),            // input wire [0 : 0] web
        .addrb(ut_ramRdAddr),    // input wire [9 : 0] addrb
        .dinb (32'd0),           // input wire [31 : 0] dinb
        .doutb(ut_dpRamDataOut)  // output wire [31 : 0] doutb
    );  
    //Matrix Calculation
    
    (* mark_debug = DEBUG *) wire s_axis_a_tready;
    (* mark_debug = DEBUG *) wire m_result_tlast2;
    
    (* mark_debug = DEBUG *) wire [31:0] ut_multOut;
    (* mark_debug = DEBUG *) wire m_axis_mult_result_tlast;
    floating_Mult floating_Mult_Ut_i (
        .aclk                (sysClk),                // input wire aclk
        .s_axis_a_tvalid     (posError_tvalid),       // input wire s_axis_a_tvalid
        .s_axis_a_tready     (s_axis_a_tready),       // output wire s_axis_a_tready
        .s_axis_a_tdata      (fposError),             // input wire [31 : 0] s_axis_a_tdata
        .s_axis_a_tlast      (posError_tlast),
        //
        .s_axis_b_tvalid     (s_ut_tvalid_t),         // input wire s_axis_b_tvalid
        .s_axis_b_tready     (),                      // output wire s_axis_b_tready
        .s_axis_b_tdata      (ut_dpRamDataOut),       // input wire [31 : 0] s_axis_b_tdata
        // A*B=C
        .m_axis_result_tvalid(m_result_tvalid),       // output wire m_axis_result_tvalid
        .m_axis_result_tready(1'b1),                  // input wire m_axis_result_tready
        .m_axis_result_tdata (ut_multOut),             // output wire [31 : 0] m_axis_result_tdata
        .m_axis_result_tlast (m_axis_mult_result_tlast)
    );
       

    (* mark_debug = DEBUG *) wire [31:0] eigen;
    // Ut[0][n] * PosError[n]
    floating_accm floating_accm_i (
        .aclk(sysClk),                            // input wire aclk
        .s_axis_a_tvalid(m_result_tvalid),        // input wire s_axis_a_tvalid
        .s_axis_a_tready(),                       // output wire s_axis_a_tready
        .s_axis_a_tdata (ut_multOut),              // input wire [31 : 0] s_axis_a_tdata
        //.s_axis_a_tlast (s_ut_tlast_t),               // input wire s_axis_a_tlast
        .s_axis_a_tlast (m_axis_mult_result_tlast),
        //
        .m_axis_result_tvalid(m_result_tvalid2),  // output wire m_axis_result_tvalid
        .m_axis_result_tready(1'b1),              // input wire m_axis_result_tready
        .m_axis_result_tdata(eigen),              // output wire [31 : 0] m_axis_result_tdata
        .m_axis_result_tlast(m_result_tlast2)     // output wire m_axis_result_tlast
    ); 

    (* mark_debug = DEBUG *) wire eigen_tlast_valid;
    assign eigen_tlast_valid = m_result_tlast2 && m_result_tvalid2;
    
    reg update_eigen_reg;
    (* mark_debug = DEBUG *) reg update_trig;
    always @ (posedge sysClk) begin        
        update_eigen_reg <= eigen_tlast_valid;
        update_trig <= update_eigen_reg;
	end	
		
	    
    //taking accumulator Eigen SUM : P
    reg [31:0] eigen_sum_reg;
    always @ (posedge sysClk) begin
        if(eigen_tlast_valid == 1'b1) begin
            eigen_sum_reg <= eigen;
        end
        else eigen_sum_reg <= eigen_sum_reg;
    end
    assign eigen_sum = eigen_sum_reg;
    
    reg [31:0] eigen_sum_delta_reg;
    //D
    always @ (posedge sysClk) begin
        if(update_eigen_reg)
            eigen_sum_delta_reg <= eigen_sum_reg;
	end	
	
	// on trig delay value
	wire [31:0] eigen_sum_delta_0, eigen_sum_delta_1;
	reg [31:0] eigen_sum_delta_1_reg;
    assign eigen_sum_delta_0 = eigen_sum_delta_reg;
    
    
    wire eigen_D_sub_trig;
    (* mark_debug = DEBUG *) wire eigen_D_sub_out_valid;
    wire eigen_D_sub_trieigen_sum_delta_regg;
    
    //assign eigen_D_sub_trig = update_trig;   //delta is zero
    assign eigen_D_sub_trig = update_eigen_reg;
    //D = eigen_sum_reg - eigen_sum_delta
    //
    floating_sub floating_sub_d_i (
        .aclk(sysClk),                     // input wire aclk
        .s_axis_a_tvalid (eigen_D_sub_trig),            // input wire s_axis_a_tvalid
        .s_axis_a_tready (),                // output wire s_axis_a_tready
        .s_axis_a_tdata  (eigen_sum_reg),    // input wire [31 : 0] s_axis_a_tdata
        .s_axis_b_tvalid (eigen_D_sub_trig),            // input wire s_axis_b_tvalid
        .s_axis_b_tready (),                // output wire s_axis_b_tready
        .s_axis_b_tdata  (eigen_sum_delta_reg), // input wire [31 : 0] s_axis_b_tdata
        //
        .m_axis_result_tvalid(eigen_D_sub_out_valid),              // output wire m_axis_result_tvalid
        .m_axis_result_tready(1'b1),          // input wire m_axis_result_tready
        .m_axis_result_tdata(eigen_sum_delta_1) // output wire [31 : 0] m_axis_result_tdata
    );
    
    //7/10/24 added: update for sub result update
    always @ (posedge sysClk) begin
        if(eigen_D_sub_out_valid)
            eigen_sum_delta_1_reg <= eigen_sum_delta_1;
	end	
	        
    assign eigen_sum_delta =  (eigen_d_sel == 1'b1) ? eigen_sum_delta_1_reg : eigen_sum_delta_0;
    
    
    wire [31:0] eigen_i_sg_out;
    reg [31:0] eigen_integral_out_reg;
    wire [31:0] eigen_integral_out;
    (* mark_debug = DEBUG *) wire eigen_i_sg_out_tvalid;
    (* mark_debug = DEBUG *) wire eigen_i_tvalid;
    
    floating_add floating_add_i (
        .aclk(sysClk),                             // input wire aclk
        .s_axis_a_tvalid(eigen_i_sg_out_tvalid),       // input wire s_axis_a_tvalid
        .s_axis_a_tready(),                        // output wire s_axis_a_tready
        .s_axis_a_tdata (eigen_sum_reg),           // input wire [31 : 0] s_axis_a_tdata
        .s_axis_b_tvalid(eigen_i_sg_out_tvalid),       // input wire s_axis_b_tvalid
        .s_axis_b_tready(),                        // output wire s_axis_b_tready
        .s_axis_b_tdata (eigen_i_sg_out),          // input wire [31 : 0] s_axis_b_tdata
        //
        .m_axis_result_tvalid(eigen_i_tvalid),                   // output wire m_axis_result_tvalid
        .m_axis_result_tready(1'b1),               // input wire m_axis_result_tready
        
        .m_axis_result_tdata (eigen_integral_out) // output wire [31 : 0] m_axis_result_tdata
        //.m_axis_result_tlast ()                    // output wire m_axis_result_tlast
    );    


    floating_Mult floating_Mult_Ut_i2 (
        .aclk                (sysClk),                 // input wire aclk
        .s_axis_a_tvalid     (update_trig),            // input wire s_axis_a_tvalid
        .s_axis_a_tready     (),                       // output wire s_axis_a_tready
        .s_axis_a_tdata      (eigen_integral_out_reg), // input wire [31 : 0] s_axis_a_tdata
        .s_axis_a_tlast      (update_trig),
        //
        .s_axis_b_tvalid     (update_trig),            // input wire s_axis_b_tvalid
        .s_axis_b_tready     (),                       // output wire s_axis_b_tready
        .s_axis_b_tdata      (SGain_Set),              // input wire [31 : 0] s_axis_b_tdata
        //
        .m_axis_result_tvalid(eigen_i_sg_out_tvalid),  // output wire m_axis_result_tvalid
        .m_axis_result_tready(1'b1),                   // input wire m_axis_result_tready
        .m_axis_result_tdata (eigen_i_sg_out),          // output wire [31 : 0] m_axis_result_tdata
        .m_axis_result_tlast ()
    );
    
    
    
    //delay one trig Z-1
    always @ (posedge sysClk) begin
        if(update_eigen_reg)
            eigen_integral_out_reg <= eigen_integral_out;
	end	   
	
    assign eigen_sum_integral = eigen_integral_out;
    assign eigen_update_trig  = update_trig; 
    
endmodule
