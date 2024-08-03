`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/06/2024 11:01:47 PM
// Design Name: 
// Module Name: UtCal
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
6/30/24
  added sa_count_reg

 */
 
`define V_RAM_2ND_ADDR  200    
module UtCal #(
    parameter MAX_CORR_NUM       = 180,
    parameter ONE_CELL_CORR_NUM  = 6,
    parameter DEBUG              = "false"
    )(
    input sysClk,
    input reset,
    input fofbCalStart,
	input fofbOn,
    input [7:0] UtCalDly,
	input [MAX_CORR_NUM-1:0] Ut_enable,		//ram Address line
    input [MAX_CORR_NUM-1:0] Ut_wren,
    input [MAX_CORR_NUM-1:0] Ut_RamB_Out_enable,
	input [9:0]  UtCalcLength,
	input [8:0]  Ut_RamBlockIndex,
	input [8:0]  V_RamBlockIndex,
    (* mark_debug = DEBUG *)input [9:0]  Ut_RamAddress,
    input [31:0] Ut_RamWData,
    input [31:0] SGain_Set,
    input [31:0] f32UserPsSet,
    input [1:0]  vm_sel,
    input eigen_d_sel,
    (* mark_debug = DEBUG *) input [31:0] posError,
    (* mark_debug = DEBUG *) input posError_tvalid,
    (* mark_debug = DEBUG *) input posError_tlast,
    (* mark_debug = DEBUG *) output [31:0] Ut_RamDataMon,
    output wire [31:0] eigenReadData,
    output wire [31:0] eigenSumI_ReadData,
    output wire [31:0] eigenSumD_ReadData,
    //
//    (* mark_debug = DEBUG *) output [31:0] eigen_sum_vin,
//	(* mark_debug = DEBUG *) output [31:0] eigen_sum_integ_vin,
//	(* mark_debug = DEBUG *) output [31:0] eigen_sum_delta_vin,
//	(* mark_debug = DEBUG *) output eigen_vec_tvalid,
//	(* mark_debug = DEBUG *) output eigen_vec_tlast,
    (* mark_debug = DEBUG *)output [95:0]M01_AXIS_0_tdata,
    (* mark_debug = DEBUG *)output [0:0] M01_AXIS_0_tlast,
    (* mark_debug = DEBUG *)output [0:0] M01_AXIS_0_tvalid, 
	
    //
    // V matrix
    input active_Vram,
    input [11:0] VmCalDly,
    input [11:0] PsUpdateDly,
    input [31:0] f32_pid_sum_gain,
    input [31:0] f32_Vout_Amp2DacGain,
    input [31:0] NCO_FreqSet,
    input [31:0] NCO_Kx_Gain,
  
	input [ONE_CELL_CORR_NUM-1:0] V_enable_P,	
    input [ONE_CELL_CORR_NUM-1:0] V_wren_P,
	input [ONE_CELL_CORR_NUM-1:0] V_enable_I,	
    input [ONE_CELL_CORR_NUM-1:0] V_wren_I,
	input [ONE_CELL_CORR_NUM-1:0] V_enable_D,	
    input [ONE_CELL_CORR_NUM-1:0] V_wren_D,        
    input [9:0]  V_RamAddress,
    input [31:0] V_RamWData,    
    output [31:0] V_RamData_pMon,
    output [31:0] V_RamData_iMon,
    output [31:0] V_RamData_dMon,
	output [31:0] PsSet_muxDatOut,
	output ps_mux_tvalid,
	output ps_mux_tlast,
	//VOUT BRAM read
    input VOut_ram_mon_en,
    input [7:0] VOut_ram_mon_addr,
    output [31:0] VOut_ram_mon_data,  
    
    input gtyTxClk,
    input ps_ctl_mode,
    input [19:0] testPsData0,
    input [19:0] testPsData1,
    input [19:0] testPsData2,
    input [19:0] testPsData3,
    input [19:0] testPsData4,
    input [19:0] testPsData5,
    
	output wire [32:0] Ps2GTY_tx_data,
    output wire [32:0] sa_count,
    output calDone    
    );
    

    
    wire ut_outValid;
    wire [8:0] ut_ramRdAddr;
    wire [31:0] eigen_sum_delta[0:MAX_CORR_NUM-1];
    wire [31:0] eigen_sum_integral[0:MAX_CORR_NUM-1];
    wire [31:0] eigen_sum[0:MAX_CORR_NUM-1];
    
    wire [31:0] ut_memread[0:MAX_CORR_NUM-1];
    wire [MAX_CORR_NUM-1:0] eigen_tvalid;
    wire [MAX_CORR_NUM-1:0] eigen_update_trig;
    
    wire ut_tlast_t;
    wire ut_tvalid_t;
    
    /////////////////////////////////////////////////////
    /////////////////////// Ut matrix ////////////////////     
    
    //Ut generator
	genvar ut_n;
	generate
		for (ut_n=0; ut_n < MAX_CORR_NUM; ut_n=ut_n+1) begin  : Ut_ram_i		
 
            Ut_OneCal #(
                .DEBUG        ("false")
                ) Ut_OneCal_i ( 
                .sysClk(sysClk),
                .trig(fofbCalStart),
                .s_ut_tvalid_t(ut_tvalid_t),  //Ut RAM valid
                  .s_ut_tlast_t (ut_tlast_t), //s_ut_acc_tlast_t),
                .Ut_enable    (Ut_enable[ut_n]),
                .Ut_wren      (Ut_wren[ut_n]),
                .Ut_RamB_Out_enable (Ut_RamB_Out_enable),
                .Ut_RamAddress(Ut_RamAddress[9:0]),
                .Ut_RamWData  (Ut_RamWData),    
                .ut_outValid  (ut_outValid),
                .ut_ramRdAddr (ut_ramRdAddr),
                .SGain_Set    (SGain_Set),
                .eigen_d_sel  (eigen_d_sel),
                .fposError    (posError),           //position error floating
                .posError_tvalid (posError_tvalid),
                .posError_tlast  (posError_tlast),                
                .m_result_tvalid (),                
                .ut_memread   (ut_memread[ut_n]), //Ut RAM monitor
                //
                .m_result_tvalid2  (eigen_tvalid[ut_n]),
                .eigen_sum         (eigen_sum[ut_n]),   //monitor
                .eigen_sum_integral(eigen_sum_integral[ut_n]),
                .eigen_sum_delta   (eigen_sum_delta[ut_n]),
                .eigen_update_trig (eigen_update_trig[ut_n])
                               
                );
                                    
        end                        
	endgenerate	   

    //delay control for Ut RAM readout
    wire StartTrig;
    gateDelayFast gateDelayFast_i2 (
        .Clk  (sysClk),
        .Inp  (fofbCalStart),
        .Delay({24'd0, UtCalDly}),
        .Width(32'd1),
        .Q    (StartTrig)  
    );
    
 	controller controller_Ut_i(
		.clk          (sysClk),
		.reset        (reset),
		.fofbCalStart (StartTrig), //fofbCalStart),
		.CalcLanth    (UtCalcLength),
		.addRamR      (ut_ramRdAddr),
		.addRamRReg   (),
		.tlast_t      (ut_tlast_t),
		.tvalid_t     (ut_tvalid_t),        
		.addRamR_Valid(ut_outValid )
	);   


    reg [31:0] Ut_RamReadData_reg;
	always @ (posedge sysClk) begin  
		//if(Ut_enable[Ut_RamAddress] == 1'b1 && Ut_wren[Ut_RamAddress] == 1'b0 ) begin
			Ut_RamReadData_reg <= ut_memread[Ut_RamBlockIndex];
		//end
	end	
	assign Ut_RamDataMon = Ut_RamReadData_reg;

    reg [31:0] eigenReadData_reg;
    reg [31:0] eigenSumI_ReadData_reg;
	reg [31:0] eigenSumD_ReadData_reg;
    
    always @ (posedge sysClk) begin  
		//if(Ut_enable[Ut_RamAddress] == 1'b1 & Ut_wren[Ut_RamAddress] == 1'b0 ) begin
			eigenReadData_reg <= eigen_sum[Ut_RamBlockIndex];
		//end
	end	
	assign eigenReadData = eigenReadData_reg;
	//	        
	always @ (posedge sysClk) begin  
		eigenSumI_ReadData_reg <= eigen_sum_integral[Ut_RamBlockIndex];
	end	    
	assign eigenSumI_ReadData = eigenSumI_ReadData_reg;
    //    
	always @ (posedge sysClk) begin  
		eigenSumD_ReadData_reg <= eigen_sum_delta[Ut_RamBlockIndex];
	end	
	assign eigenSumD_ReadData = eigenSumD_ReadData_reg;
    
            
    
    /////////////////////////////////////////////////////
    /////////////////////// V matrix ////////////////////
    
    reg [31:0] eigen_sum_r;       
    reg [31:0] eigen_sum_integral_r;
    reg [31:0] eigen_sum_delta_r;
    	
	
	localparam IDLE       = 5'd0;
	localparam FIFO1_RD   = 5'd1;

	(* mark_debug = DEBUG *)reg [9:0]   run_count=0;
	(* mark_debug = DEBUG *)reg [1:0]   state = IDLE;
		
	/*
	 * Ut calculation output converting to stream output
	 */	
	always@(posedge sysClk) begin		
		if(reset == 1'b1) begin
			run_count  <= 10'd0;
			eigen_sum_r <= 32'd0;
			eigen_sum_integral_r <= 32'd0;
			eigen_sum_delta_r    <= 32'd0;
			state      <= IDLE;
		end
		else begin
			case (state)		     
				IDLE : begin						
					if(eigen_update_trig[0] == 1'b1) begin
						run_count <= 10'd0;								
						state <= FIFO1_RD;	
					end						
					else state <= IDLE;
				end				
				FIFO1_RD: begin
				    //if(run_count < MAX_CORR_NUM) begin
				    if(run_count < MAX_CORR_NUM-1) begin  //6/10/24, 0..179
                        eigen_sum_r          <= eigen_sum[run_count];
                        eigen_sum_integral_r <= eigen_sum_integral[run_count];
                        eigen_sum_delta_r    <= eigen_sum_delta[run_count];					
                        run_count  <= run_count+1;

						state <= FIFO1_RD;
					end	
					else begin
					   run_count <= 10'd0;
					   state <= IDLE;
					end   
				end
				
			endcase	
		end	
	end	
	
	
	//
	//V input data	    
	
    (* mark_debug = DEBUG *) wire [31:0] eigen_sum_vin;
	(* mark_debug = DEBUG *) wire [31:0] eigen_sum_integ_vin;
	(* mark_debug = DEBUG *) wire [31:0] eigen_sum_delta_vin;
	(* mark_debug = DEBUG *) wire eigen_vec_tvalid;
	(* mark_debug = DEBUG *) wire eigen_vec_tlast;
	
	//Ut Cal output and interface with DMA
	assign eigen_sum_vin       = eigen_sum_r;
	assign eigen_sum_integ_vin = eigen_sum_integral_r;
	assign eigen_sum_delta_vin = eigen_sum_delta_r;
	assign eigen_vec_tvalid    = state[0];
	assign eigen_vec_tlast     = (run_count == MAX_CORR_NUM-1) ? 1'b1 : 1'b0;
	
    
    /////
    //BUS broad cast    
    (* mark_debug = DEBUG *)wire [95:0]M00_AXIS_0_tdata;
    (* mark_debug = DEBUG *)wire [0:0] M00_AXIS_0_tlast;
    (* mark_debug = DEBUG *)wire [0:0] M00_AXIS_0_tvalid;
    
    design_brodcast design_brodcast_i
    (
        .M00_AXIS_0_tdata (M00_AXIS_0_tdata),
        .M00_AXIS_0_tlast (M00_AXIS_0_tlast),
        .M00_AXIS_0_tvalid(M00_AXIS_0_tvalid),
        // to DMA interface
        .M01_AXIS_0_tdata (M01_AXIS_0_tdata),
        .M01_AXIS_0_tlast (M01_AXIS_0_tlast),
        .M01_AXIS_0_tvalid(M01_AXIS_0_tvalid),
        //Input
        .S_AXIS_0_tdata ({eigen_sum_delta_vin,eigen_sum_integ_vin,eigen_sum_vin}),
        .S_AXIS_0_tlast (eigen_vec_tlast),
        .S_AXIS_0_tvalid(eigen_vec_tvalid),
        .aclk_0   (sysClk),
        .aresetn_0(~reset)
    );    
	/////
    

`define __V_ENABLE__    
`ifdef  __V_ENABLE__       
	(* mark_debug = DEBUG *)wire [31:0] vm_p_sum[0:ONE_CELL_CORR_NUM-1];
	(* mark_debug = DEBUG *)wire [31:0] vm_i_sum[0:ONE_CELL_CORR_NUM-1];
	(* mark_debug = DEBUG *)wire [31:0] vm_d_sum[0:ONE_CELL_CORR_NUM-1];
	(* mark_debug = DEBUG *)wire [31:0] vm_pid_sum_out[0:ONE_CELL_CORR_NUM-1];
	(* mark_debug = DEBUG *)wire [31:0] psSP_I32[0:ONE_CELL_CORR_NUM-1];
	(* mark_debug = DEBUG *)wire [31:0] psSP_I32_LO[0:ONE_CELL_CORR_NUM-1];
	wire [19:0] psSP_I20_Set[0:ONE_CELL_CORR_NUM-1];
    (* mark_debug = DEBUG *)wire [19:0] psSP_I20_SetOut[0:ONE_CELL_CORR_NUM-1];
	
	
	wire [31:0] v_memread_p[0:ONE_CELL_CORR_NUM-1];
	wire [31:0] v_memread_i[0:ONE_CELL_CORR_NUM-1];
	wire [31:0] v_memread_d[0:ONE_CELL_CORR_NUM-1];
	(* mark_debug = DEBUG *)wire [ONE_CELL_CORR_NUM-1:0] vm_i_sum_tvalid;
	(* mark_debug = DEBUG *)wire [ONE_CELL_CORR_NUM-1:0] v_p_update_trig;
	
	(* mark_debug = DEBUG *)wire v_outValid;	
	wire [8:0] v_ramRdAddr;
	(* mark_debug = DEBUG *)wire [8:0] v_ramRdAddr_active;   
	(* mark_debug = DEBUG *)wire v_tvalid_t;
	(* mark_debug = DEBUG *)wire [ONE_CELL_CORR_NUM-1:0] v_pid_sum_tvalid;
	wire [31:0] F32_vm_sum[0:ONE_CELL_CORR_NUM-1];
	wire [31:0] nco32;   
	wire [31:0] Vout_Amp2DAC_data[0:ONE_CELL_CORR_NUM-1];
	

    //V generator
	genvar v_n;
	generate
		for (v_n=0; v_n < ONE_CELL_CORR_NUM; v_n=v_n+1) begin  : V_ram_i		
 
            ////// P //////
            Vm_OneCal #(
                .DEBUG        ("true")
                ) V_P ( 
                .sysClk(sysClk),
                .trig  (),
                .s_ut_tvalid_t(v_tvalid_t),
                //.s_ut_tlast_t (s_v_tlast_t), 
                //
                .Ut_enable    (V_enable_P[v_n]),
                .Ut_wren      (V_wren_P[v_n]),
                .Ut_RamAddress(V_RamAddress[9:0]),
                .Ut_RamWData  (V_RamWData),
                //    
                .ut_outValid  (v_outValid),
                .ut_ramRdAddr (v_ramRdAddr_active),
                //
                //.eigen_vec_in     (eigen_sum_vin),   
                //.eigen_vec_tvalid (eigen_vec_tvalid), 
                //
                .eigen_vec_in     (M00_AXIS_0_tdata[31:0]),   
                .eigen_vec_tvalid (M00_AXIS_0_tvalid),
                .eigen_vec_tlast  (M00_AXIS_0_tlast),                   
                               
                .m_result_tvalid  (),                
                .ut_memread       (v_memread_p[v_n]), //P RAM monitor
                //
                .m_result_tvalid2  (),
                .eigen_sum         (vm_p_sum[v_n]),   //monitor
                .eigen_update_trig (v_p_update_trig[v_n])                               
            );
              
            ///// I ///////  
            Vm_OneCal #(
                .DEBUG        ("false")
                ) V_I ( 
                .sysClk(sysClk),
                .trig  (),
                .s_ut_tvalid_t(v_tvalid_t),
                //.s_ut_tlast_t (s_v_tlast_t),
                .Ut_enable    (V_enable_I[v_n]),
                .Ut_wren      (V_wren_I[v_n]),
                .Ut_RamAddress(V_RamAddress[9:0]),
                .Ut_RamWData  (V_RamWData),    
                .ut_outValid  (v_outValid),          //in
                .ut_ramRdAddr (v_ramRdAddr_active),  //in
                //
//                .eigen_vec_in     (eigen_sum_integ_vin),   
//                .eigen_vec_tvalid (eigen_vec_tvalid),   
                //
                .eigen_vec_in     (M00_AXIS_0_tdata[63:32]),   
                .eigen_vec_tvalid (M00_AXIS_0_tvalid),
                .eigen_vec_tlast  (M00_AXIS_0_tlast),                   
                 //            
                .m_result_tvalid  (),                
                .ut_memread       (v_memread_i[v_n]), //I RAM monitor
                //
                .m_result_tvalid2  (vm_i_sum_tvalid[v_n]),
                .eigen_sum         (vm_i_sum[v_n]),   //monitor
                .eigen_update_trig ()                               
             );              
             
             ///// D /////// 
             Vm_OneCal #(
                .DEBUG        ("false")
                ) V_D ( 
                .sysClk(sysClk),
                .trig  (),
                .s_ut_tvalid_t(v_tvalid_t),
                //.s_ut_tlast_t (s_v_tlast_t),
                .Ut_enable    (V_enable_D[v_n]),
                .Ut_wren      (V_wren_D[v_n]),
                .Ut_RamAddress(V_RamAddress[9:0]),
                .Ut_RamWData  (V_RamWData),    
                .ut_outValid  (v_outValid),
                .ut_ramRdAddr (v_ramRdAddr_active),
                //
//                .eigen_vec_in     (eigen_sum_delta_vin),   
//                .eigen_vec_tvalid (eigen_vec_tvalid),
                //
                .eigen_vec_in     (M00_AXIS_0_tdata[95:64]),   
                .eigen_vec_tvalid (M00_AXIS_0_tvalid),
                .eigen_vec_tlast  (M00_AXIS_0_tlast),  
                //                
                .m_result_tvalid  (),                
                .ut_memread       (v_memread_d[v_n]), // D ram monitor
                //
                .m_result_tvalid2  (),
                .eigen_sum         (vm_d_sum[v_n]),   //monitor
                .eigen_update_trig ()                               
             );              
             //////////////// 
                                                                                  
            design_pid_add design_pid_add_i
            (
                .S_AXIS_A_0_tdata  (vm_p_sum[v_n]),
                .S_AXIS_A_0_tready (),
                .S_AXIS_A_0_tvalid (1'b1),
                .S_AXIS_B_0_tdata  (vm_i_sum[v_n]),
                .S_AXIS_B_0_tready (),
                .S_AXIS_B_0_tvalid (1'b1),                
                .S_AXIS_C_0_tdata  (vm_d_sum[v_n]),
                .S_AXIS_C_0_tready (),
                .S_AXIS_C_0_tvalid (1'b1),
                //Gain control
                .S_AXIS_GAIN_tdata  (f32_pid_sum_gain),
                .S_AXIS_GAIN_tready (),
                .S_AXIS_GAIN_tvalid (1'b1),
                .M_AXIS_RESULT_0_tdata  (vm_pid_sum_out[v_n]),
                .M_AXIS_RESULT_0_tready (1'b1),
                .M_AXIS_RESULT_0_tvalid (v_pid_sum_tvalid[v_n]),
                //                
                .aclk_0(sysClk)
            );             
             
            ///
            /// Add Mux for V out, Manual setting, NCO
            ///
             
            assign F32_vm_sum[v_n] = ( vm_sel[1:0] == 2'd0)  ? vm_pid_sum_out[v_n]  :
					  ( vm_sel[1:0] == 2'd1)  ? f32UserPsSet :     //User setting +/- 1.25A			
					  ( vm_sel[1:0] == 2'd2)  ? nco32 :	
                        32'd0 ;                     
                         
            ///// Amp to DAC converter  
            ///// 524287 / 1.25(A) = 419429.6
    
            floating_simple_mut floating_simple_mut_i (
                .aclk(sysClk),                                  // input wire aclk
                .s_axis_a_tvalid(1'b1),            // input wire s_axis_a_tvalid
                .s_axis_a_tready(),            // output wire s_axis_a_tready
                .s_axis_a_tdata (F32_vm_sum[v_n]),              // input wire [31 : 0] s_axis_a_tdata
                .s_axis_b_tvalid(1'b1),            // input wire s_axis_b_tvalid
                .s_axis_b_tready(),            // output wire s_axis_b_tready
                .s_axis_b_tdata (f32_Vout_Amp2DacGain),              // input wire [31 : 0] s_axis_b_tdata
                //
                .m_axis_result_tvalid (),      // output wire m_axis_result_tvalid
                .m_axis_result_tready (1'b1),  // input wire m_axis_result_tready
                .m_axis_result_tdata  (Vout_Amp2DAC_data[v_n])    // output wire [31 : 0] m_axis_result_tdata
            );   
                 
                 
            // Converter 
            float2int32 float2int32_i (
              .aclk(sysClk),                        // input wire aclk
              .s_axis_a_tvalid(1'b1),               // input wire s_axis_a_tvalid
              .s_axis_a_tready(),                   // output wire s_axis_a_tready
              //.s_axis_a_tdata(vm_pid_sum_out[v_n]), // input wire [31 : 0] s_axis_a_tdata
              .s_axis_a_tdata(Vout_Amp2DAC_data[v_n]),
              //
              .m_axis_result_tvalid(),  // output wire m_axis_result_tvalid
              .m_axis_result_tready(1'b1),  // input wire m_axis_result_tready
              .m_axis_result_tdata (psSP_I32[v_n])    // output wire [31 : 0] m_axis_result_tdata
            );             
                                                          
                            
            ////////////////////////////
            ///// Limit -> Converter to Power supply setting 20 bit
            /////            
            over_limit overflow_limit0 ( .Data_In(psSP_I32[v_n]),  .LIMIT(32'd520000), .Data_o( psSP_I32_LO[v_n]) );
            Converter  u_Converter0    ( .two_comp(psSP_I32_LO[v_n][19:0]),  .ps_out(psSP_I20_Set[v_n]) );  
                                                                 
                                    
        end                        
	endgenerate	      
       
 
    //delay control for V RAM readout
    wire vStartTrig;
    gateDelayFast gateDelayFast_i3 (
        .Clk  (sysClk),
        .Inp  (fofbCalStart),
        .Delay({20'd0, VmCalDly}),
        .Width(32'd1),
        .Q    (vStartTrig)  
    );
    
    
    assign 	v_ramRdAddr_active  = ( active_Vram == 1'b1) ? (v_ramRdAddr+`V_RAM_2ND_ADDR) :  v_ramRdAddr;
 	controller V_controller(
		.clk          (sysClk),
		.reset        (reset),
		.fofbCalStart (vStartTrig), 
		//.CalcLanth    (10'd179),  //180-1
		.CalcLanth    (MAX_CORR_NUM-1),
		.addRamR      (v_ramRdAddr),
		.addRamRReg   (),
		.tlast_t      (s_v_tlast_t),   //NOT NEEDED
		.tvalid_t     (v_tvalid_t),        
		.addRamR_Valid(v_outValid )
	);  
	


    // V-PID DPRAM (180) monitoring
    reg [31:0] V_RamReadData_preg;
	always @ (posedge sysClk) begin  
		//if(V_enable_P[V_RamBlockIndex] == 1'b1 & V_wren_P[V_RamBlockIndex] == 1'b0 ) begin
			V_RamReadData_preg <= v_memread_p[V_RamBlockIndex];
		//end
	end	
	assign V_RamData_pMon = V_RamReadData_preg;
	
    reg [31:0] V_RamReadData_ireg;
	always @ (posedge sysClk) begin  
		//if(V_enable_I[V_RamBlockIndex] == 1'b1 & V_wren_I[V_RamBlockIndex] == 1'b0 ) begin
			V_RamReadData_ireg <= v_memread_i[V_RamBlockIndex];
		//end
	end	
	assign V_RamData_iMon = V_RamReadData_ireg;
	
	reg [31:0] V_RamReadData_dreg;
	always @ (posedge sysClk) begin  
		//if(V_enable_D[V_RamBlockIndex] == 1'b1 & V_wren_D[V_RamBlockIndex] == 1'b0 ) begin
			V_RamReadData_dreg <= v_memread_d[V_RamBlockIndex];
		//end
	end	
	assign V_RamData_dMon = V_RamReadData_dreg;
			
			

	/////////////////////////////////////////////////////////	
    nco_sincos32
    (
        .clk   (sysClk),
        .reset (1'b0),
        .clk_enable (fofbCalStart),
        .phase_inc  (NCO_FreqSet),
        .Kx (NCO_Kx_Gain),
        .Ky (NCO_Kx_Gain),        
        .sin(nco32) ,
        .cos()
    );
    // int32 to float conversion and gain control
    
    
                      	
`endif

	(* mark_debug = DEBUG *) wire ps_trig;
    gateDelayFast ps_output_dly_i (
        .Clk  (sysClk),
        .Inp  (fofbCalStart),
        .Delay({20'd0, PsUpdateDly}),
        .Width(32'd1),
        .Q    (ps_trig)  
    );
    
    
 /*
  * Add mux for PS output test mode
  */   
    
    assign psSP_I20_SetOut[0] = (ps_ctl_mode == 1'b1) ? testPsData0 : psSP_I20_Set[0];
    assign psSP_I20_SetOut[1] = (ps_ctl_mode == 1'b1) ? testPsData1 : psSP_I20_Set[1];
    assign psSP_I20_SetOut[2] = (ps_ctl_mode == 1'b1) ? testPsData2 : psSP_I20_Set[2];
    assign psSP_I20_SetOut[3] = (ps_ctl_mode == 1'b1) ? testPsData3 : psSP_I20_Set[3];
    assign psSP_I20_SetOut[4] = (ps_ctl_mode == 1'b1) ? testPsData4 : psSP_I20_Set[4];
    assign psSP_I20_SetOut[5] = (ps_ctl_mode == 1'b1) ? testPsData5 : psSP_I20_Set[5];
    

// with CRC caluclation
wire [7:0] vout_addr;
wire [31:0] COMM_PacketHead;
assign COMM_PacketHead = {12'd46, 12'd0, 8'h5C};

    aurora_biComm_pkt_gen_64b# (
        .PACKET_MAX_LEN (46)  //27+CRC = 28
    ) aurora_biComm_pkt_gen_64b_i 
    (
        .clk   (sysClk),
        .reset (1'b0),
        .clk_enable(1'b1),        
        .trig  (ps_trig),
        //
        .a1(  COMM_PacketHead ),
		.a2({ 8'd0, 4'd0, psSP_I20_SetOut[0]} ),
		.a3({ 8'd0, 4'd0, psSP_I20_SetOut[1]} ),
		.a4({ 8'd0, 4'd0, psSP_I20_SetOut[2]} ),
		.a5({ 8'd0, 4'd0, psSP_I20_SetOut[3]} ),
		.a6({ 8'd0, 4'd0, psSP_I20_SetOut[4]} ),
		.a7({ 8'd0, 4'd0, psSP_I20_SetOut[5]} ),
	    //
		.a8  (vm_p_sum[0] ),
		.a9  (vm_p_sum[1] ),
		.a10 (vm_p_sum[2] ),
		.a11 (vm_p_sum[3] ),
		.a12 (vm_p_sum[4] ),
		.a13 (vm_p_sum[5] ),
		//
		.a14 (vm_i_sum[0] ),
		.a15 (vm_i_sum[1] ),
		.a16 (vm_i_sum[2] ),
		.a17 (vm_i_sum[3] ),
		.a18 (vm_i_sum[4] ),
		.a19 (vm_i_sum[5] ),
		//
		.a20 (vm_d_sum[0] ),
		.a21 (vm_d_sum[1] ),
		.a22 (vm_d_sum[2] ),
		.a23 (vm_d_sum[3] ),
		.a24 (vm_d_sum[4] ),
		.a25 (vm_d_sum[5] ),	
		//
		.a26 (F32_vm_sum[0] ),
		.a27 (F32_vm_sum[1] ),
		.a28 (F32_vm_sum[2] ),
		.a29 (F32_vm_sum[3] ),
		.a30 (F32_vm_sum[4] ),
		.a31 (F32_vm_sum[5] ),		
		//			
		.a32 (Vout_Amp2DAC_data[0] ),
		.a33 (Vout_Amp2DAC_data[1] ),
		.a34 (Vout_Amp2DAC_data[2] ),
		.a35 (Vout_Amp2DAC_data[3] ),
		.a36 (Vout_Amp2DAC_data[4] ),
		.a37 (Vout_Amp2DAC_data[5] ),						
//
		.a38 (psSP_I32[0] ),
		.a39 (psSP_I32[1] ),
		.a40 (psSP_I32[2] ),
		.a41 (psSP_I32[3] ),
		.a42 (psSP_I32[4] ),
		.a43 (psSP_I32[5] ),						
		.a44 (32'h00000044 ),
		.a45 (32'h00000045 ),
					
        //
        .mux_addr (vout_addr),
        .TDATA  (PsSet_muxDatOut),
        .TLAST  (ps_mux_tlast),
        .TVALID (ps_mux_tvalid)
    );  


    ///// VOUT monitor		
    V_PIDmem VOUT_MON_i (
        .clka  (sysClk),    // input wire clka
        .ena   (ps_mux_tvalid),      // input wire ena
        .wea   (ps_mux_tvalid),      // input wire [0 : 0] wea
        .addra (vout_addr),  // input wire [7 : 0] addra
        .dina  (PsSet_muxDatOut),    // input wire [31 : 0] dina
        .douta (),  // output wire [31 : 0] douta
        .clkb  (sysClk),    // input wire clkb
        .enb   (VOut_ram_mon_en),      // input wire enb
        .web   (1'b0),      // input wire [0 : 0] web
        .addrb (VOut_ram_mon_addr),  // input wire [7 : 0] addrb
        .dinb  (32'd0),    // input wire [31 : 0] dinb
        .doutb (VOut_ram_mon_data)  // output wire [31 : 0] doutb
    );
    
  
 
    fifo_ps2gty fifo_ps2gty_i (
        .srst  (reset),                // input wire srst
        .wr_clk(sysClk),            // input wire wr_clk
        .rd_clk(gtyTxClk),            // input wire rd_clk
        .din   ({ps_mux_tvalid, PsSet_muxDatOut}),     // input wire [32 : 0] din
        .wr_en (1'b1),              // input wire wr_en
        .rd_en (1'b1),              // input wire rd_en
        .dout  (Ps2GTY_tx_data),    // output wire [32 : 0] dout
        .full  (),                  // output wire full
        .empty (),                  // output wire empty
        .wr_rst_busy(),  // output wire wr_rst_busy
        .rd_rst_busy()  // output wire rd_rst_busy
    );  
  
  
    //10 Hz counter
    wire clk_10hz;
    blinker #(
        .GEN_CLK_FREQUENCY(10000), //10 kHz          
        .GEN_BLINK_PERIOD (0.1)    //10Hz                  
    )
    clk_10hkz_gen
    (
        .P_I_CLK (fofbCalStart),     
        .P_O_LED (clk_10hz)
    );  
    
    wire trig_10hz;
    pedge_detect pedge_detect_i(
        .clk     (sysClk),
        .Reset   (1'b0),
        .trig    (clk_10hz),
        .edge_out(trig_10hz)
    ); 
         
	reg [31:0] sa_count_reg;
	always @ (posedge sysClk) begin
	   if(trig_10hz == 1'b1) sa_count_reg <= sa_count_reg + 1;
	   else sa_count_reg <= sa_count_reg;	   
	end
	assign sa_count = sa_count_reg;
    
    
endmodule


/*
  sysgen_dut : entity xil_defaultlib.uv_mcal_0 
  port map (
    a_in => a_in,
    b_in => b_in,
    rst => rst,
    clk => clk,
    sum_out => sum_out
  );
  
*/  