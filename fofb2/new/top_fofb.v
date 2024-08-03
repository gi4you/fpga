`timescale 1ns / 1ps

/*
    Target board : Xilinx ZCU216 RFSoC board.
        For FOFB simulator with beam and real BPM data.


    6/4/24
        revisit ZCU216 for FOFB module test
    6/5/24
        Cage setup and interface with CC AI packet output.
        RX packet test and check ILA.        
        Need to adding remote packet processing with CRC checking.
    6/6/24
        Remote packet tst    
        
    6/7/24
        position error test with floating point
        added Ut RAM 180
    6/8/24
        Ut RAM read/write test
        Ut cal block test
        
    6/10/24
        V RAM test and V cal test
        added sysGen model for PID sum
        
    6/14/24
       Fixed Ut RAM, V RAM read register 
       Ut, V, reference RAM (180, 18, 1) write/read error check, all working ok
    6/15/24
       Added VOUT monitoring RAM   
                          
    6/16/24 ~
       DMA tets started.
       issue is xfer_req signal is high from DMA core.
       Fixed using zDFE DMA_TransferSync.v module
       
    6/18/24
       Reference RAM and V ram added 2nd address for selection A/B   
     
       
    6/19/24
       sysClk changed 125 MHz
         - GTY DRP clock also need to change 125 MHz.
       EVR module added and tested
       Trigger count and latched timestamp tested with IOC
       
    6/23/24
       SDI error count monitor added and tested with a new zDFE firmware
       added .eigen_d_sel  (cmds[CMDR_CTRL0][1]),             
                
    6/26/24
       added PS set control register
       added SDI_pkt_782_i          
       
    6/26/24
      epics interface test full matrix.
      VI RAM channel cal output is not right   
    
    6/27/24
      added eigenSumI_ReadData and eigenSumD_ReadData for monitoring Eigen vector

    6/29/24
      added cmds[CMDR_CTRL1][3] for position output enable/disable
      
      
    +--------------------------------------------------------------------------+
    + JTAG 10.0.153.141
    +--------------------------------------------------------------------------+  
    7/9/24  
      install at C22 Rack-D for the beam test
      - EVR Ok
      - Ethernet and IOC working Ok
      - zDFE SDI data receive Ok
            
      cmds[CMDR_CTRL0][2] : CRC reset
         
   7/10/24
      Eigen delta substraction trigger change for Delta value update and confirmed.
      
      SDI CRC error incressed? try to power cycle and reload firmware and works.
      
   7/16/24
      added position error limit default 200 um
      .LIMIT_i (cmds[CMDR_USR_POS_ERR_LMT_SET]),                 
      
   7/18/24
      added wire pos_update_freez; for sync with timing system.
      Test with beam orbit data and cell #1 ref/Ut/V/PID data, Eigen well matched with Python calculation data.                   
   
 */


/*
 *   ZCU216 SFP+ 2x2 front view
 *        ---------------
 *         x0y4  |  x0y8
 *        ---------------
 *         x0y5  |  x0y9 (SDI from AI system)
 *        ---------------
 */


`define	FIRMWARE_VER32				32'h07102024

`define __FEEDBACK_ENABLE__

`define SYSTEM_CLOCK                125000000 //MHz
// Variable for test
`define	MAX_BPM_NUMBER				240		//X:240, Y:240	with ID BPMs
`define	MAX_CORR_NUM 				90*2	//H:90, V:90

`define	ONE_CELL_CORR_NUM           6       //FAST 6, SLOW 6
`define	CELL_PS_SIZE				12		//H:(3), V(3) 


`define	ACTIVE_RAM_BLOCK_SIZE		500		//Reference BRAM offset address
`define	MAX_PACKET_NUMBER			780


parameter CommaChar     = 8'h3C;

//Control Command registers   
parameter   CMDR_CTRL0               = 0; 
parameter   CMDR_CTRL1               = 1; 
parameter   CMDR_SDI_RAM_CTRL        = 2;
parameter   CMDR_UT_RAM_CTRL         = 5;

parameter   CMDR_EVR_TRIG_EVENT_SET  = 6;
parameter	CMDR_UT_RAM_ADDR         = 7;
parameter	CMDR_UT_RAM_DATA         = 8;    
parameter	CMDR_POS_ERROR_UNIT_DATA = 9;
parameter	CMDR_POS_REF_ADDR        = 10;
parameter	CMDR_POS_REF_DATA        = 11;
parameter   CMDR_UT_RDLY             = 12;
parameter   CMDR_UT_CAL_LENGTH       = 13;
parameter   CMDR_UT_SGAIN            = 14;
parameter   CMDR_V_RAM_ADDR          = 15;
parameter   CMDR_V_RAM_DATA          = 16;
parameter   CMDR_V_P_CTRL            = 17;
parameter   CMDR_V_I_CTRL            = 18;
parameter   CMDR_V_D_CTRL            = 19;
parameter   CMDR_V_CAL_DELAY         = 20;
parameter   CMDR_PS_OUT_DELAY        = 21;
parameter   CMDR_PS_PID_SUM_GAIN     = 22;
parameter   CMDR_UT_RAM_BLK_IDX      = 23;
parameter   CMDR_V_RAM_BLK_IDX       = 24;
parameter   CMDR_F32_PS_SET          = 25;  //for test propose
parameter   CMDR_VM_SEL              = 26;
parameter   CMDR_VOUT_RAM_ADDR       = 27;
parameter   CMDR_PS_AMP2DAC_GAIN     = 28;
parameter   CMDR_NCO_FREQ            = 29;
parameter   CMDR_NCO_GAIN            = 30;   
parameter   CMDR_USR_PS0_SET         = 31; 
parameter   CMDR_USR_PS1_SET         = 32;
parameter   CMDR_USR_PS2_SET         = 33;
parameter   CMDR_USR_PS3_SET         = 34;
parameter   CMDR_USR_PS4_SET         = 35;
parameter   CMDR_USR_PS5_SET         = 36;

parameter   CMDR_USR_POS_ERR_LMT_SET = 41;

      
module top_fofb #(
    parameter DEBUG  = "true"
    )(
    //
    input  evr_gty_refclk_p,
    input  evr_gty_refclk_n,    
    //LEDs
    output [3:0] RGB_B_LED
    );
     
    wire [0:0]reset;
    wire sysClk;
    wire intTrigger;    
    //
    reg  up_rack;
    wire [13:0]up_raddr;
    reg  [31:0]up_rdata;
    wire up_rreq;
    reg  up_wack;
    wire [13:0]up_waddr;
    wire [31:0]up_wdata;
    wire up_wreq;
    (* ram_style = "block" *) reg [31:0]  cmds[61:0];
  
    //DMA interface
    wire dam_transfer_start_trig_0;
    wire dam_transfer_start_trig_1;
    wire dam_transfer_start_trig_2;
    
    wire [63:0]s_axis_0_tdata;
    (* mark_debug = DEBUG *)wire s_axis_0_tlast;
    (* mark_debug = DEBUG *)wire s_axis_0_tready;
    wire [0:0]s_axis_0_tuser;
    (* mark_debug = DEBUG *)wire s_axis_0_tvalid;
    wire [31:0]s_axis_1_tdata;
    wire s_axis_1_tlast;
    wire s_axis_1_tready;
    wire [0:0]s_axis_1_tuser;
    wire s_axis_1_tvalid;
    wire s_axis_aclk_0;
    (* mark_debug = DEBUG *)wire s_axis_xfer_req_0;
    wire s_axis_xfer_req_1;
    wire up_dma_req_valid_0;
    wire up_dma_req_valid_1;
    //    
    wire [31:0]s_axis_2_tdata;
    wire s_axis_2_tlast;
    wire s_axis_2_tready;
    wire [0:0]s_axis_2_tuser;
    wire s_axis_2_tvalid;
    wire s_axis_xfer_req_2;
    (* mark_debug = DEBUG *)wire up_dma_req_valid_2;
    wire [31:0] sysStatus;  
    
    
    design_1 design_i
    (
        .reset(reset),
        .sysClk(sysClk),    //system clock output 125 MHz
        //        
        .up_rack_0 (up_rack),
        .up_raddr_0(up_raddr),
        .up_rdata_0(up_rdata),
        .up_rreq_0 (up_rreq),
        .up_wack_0 (up_wack),
        .up_waddr_0(up_waddr),
        .up_wdata_0(up_wdata),
        .up_wreq_0 (up_wreq),
        
        //DMA0
        .s_axis_0_tdata (s_axis_0_tdata),
        .s_axis_0_tlast (s_axis_0_tlast),
        .s_axis_0_tready(s_axis_0_tready),  //out
        .s_axis_0_tuser (),
        .s_axis_0_tvalid(s_axis_0_tvalid),
        
        //DMA1
        .s_axis_1_tdata (s_axis_1_tdata),
        .s_axis_1_tlast (s_axis_1_tlast),
        .s_axis_1_tready(s_axis_1_tready),
        .s_axis_1_tuser (),
        .s_axis_1_tvalid(s_axis_1_tvalid),
        
        //DMA2 - SDI
        .s_axis_2_tdata (s_axis_2_tdata),
        .s_axis_2_tlast (s_axis_2_tlast),
        .s_axis_2_tready(s_axis_2_tready),       //output
        .s_axis_2_tuser (),
        .s_axis_2_tvalid(s_axis_2_tvalid),       
        .s_axis_xfer_req_2  (s_axis_xfer_req_2), //output
        .up_dma_req_valid_2 (up_dma_req_valid_2),
        //
        //.s_axis_aclk_0      (sysClk),
        .s_axis_xfer_req_0  (s_axis_xfer_req_0),
        .s_axis_xfer_req_1  (s_axis_xfer_req_1),        
        .up_dma_req_valid_0 (),
        .up_dma_req_valid_1 (),    
           
        .dam_transfer_start_trig_0(dam_transfer_start_trig_0),
        .dam_transfer_start_trig_1(dam_transfer_start_trig_1),
        .dam_transfer_start_trig_2(dam_transfer_start_trig_2)
        //                
    );    
 
 
    wire [31:0] Ut_RamDataMon; 
    wire [31:0] eigenReadData;   
    wire [31:0] eigenSumI_ReadData;
    wire [31:0] eigenSumD_ReadData;
    wire [31:0] ref_dpram_read;    
    wire [31:0] xy_pos_error_float_mon;
    
    wire [31:0] V_RamData_pMon;
    wire [31:0] V_RamData_iMon;
    wire [31:0] V_RamData_dMon;
    wire [31:0] VOut_ram_mon_data;    
    
    //////// DMA ///////////
    wire usr_soft_trig;
    (* mark_debug = DEBUG *)wire usr_sync_trig;
    (* mark_debug = DEBUG *)wire [31:0] dma_turns_count_0;
    (* mark_debug = DEBUG *)wire  dma_xfer_enable_0;
    (* mark_debug = DEBUG *)wire [31:0] dma_turns_count_1;
    (* mark_debug = DEBUG *)wire  dma_xfer_enable_1;
    (* mark_debug = DEBUG *)wire [31:0] dma_turns_count_2;
    (* mark_debug = DEBUG *)wire  dma_xfer_enable_2;
    //(* mark_debug = DEBUG *)wire [2:0] dma_burst_timeout;

    (* mark_debug = DEBUG *)wire dpram_rd;
    (* mark_debug = DEBUG *)wire [9:0] dpram_addr;
    (* mark_debug = DEBUG *)wire bpmRamCellMark;
    (* mark_debug = DEBUG *)wire bram_x_wr, bram_y_wr;
    (* mark_debug = DEBUG *)wire [8:0] bram_x_addr, bram_y_addr;
    (* mark_debug = DEBUG *)wire bpmPosDataValid;
    (* mark_debug = DEBUG *)wire bpmPosFoFbDataValid;
    (* mark_debug = DEBUG *)wire DspMatrixCalStartTrig;  
    (* mark_debug = DEBUG *)wire DspMatrixCalStartTrig2;  
	 
    (* mark_debug = DEBUG *)wire Sdi_bpm8_480_DataValid;     
    (* mark_debug = DEBUG *)wire [9:0] 	xy_pos_addr;  
    (* mark_debug = DEBUG *)wire bpmPosAddrValid;
    (* mark_debug = DEBUG *)wire [31:0] bpmPos_Xy_Data;
    (* mark_debug = DEBUG *)wire [31:0] ref_xy_data;    
    (* mark_debug = DEBUG *)wire [9:0] bpmPos_xy_Addr;  //from Eigen cal block

	reg xy_pos_mask_reg;
	(* mark_debug = DEBUG *) wire	xy_pos_mask_bit;
	
	wire [63:0] evr_TimeStamp;
	wire [63:0] evr_trigLatchedTimeStamp;
	wire [31:0] trig_count;
	wire [31:0] CWCRCErrorCount;
    wire [31:0] SDI_data_readout; 		
	wire evr_fa_event;
	wire [31:0] sa_count;
		        
    /////////////////////////////////////////////////////////////
    /*
     * AXI user interface.
     */
    //reg [31:0]  cmds[61:0];
    //monitoring
    always @(posedge sysClk) begin
        if (reset == 1'b1) begin
            up_rack <= 'd0;
        end else begin
            up_rack <= up_rreq;
            if (up_rreq) begin
                case (up_raddr)
                    12'd0:  up_rdata <= cmds[0];
                    12'd1:  up_rdata <= cmds[1];
                    12'd2:  up_rdata <= cmds[2];
                    12'd3:  up_rdata <= cmds[3];
                    12'd4:  up_rdata <= cmds[4];
                    12'd5:  up_rdata <= cmds[5];
                    12'd6:  up_rdata <= cmds[6];	
                    12'd7 : up_rdata <= cmds[7];
                    12'd8 : up_rdata <= cmds[8];
                    12'd9 : up_rdata <= cmds[9];
                    12'd10: up_rdata <= cmds[10];
                    12'd11: up_rdata <= cmds[11];
                    12'd12: up_rdata <= cmds[12];
                    12'd13: up_rdata <= cmds[13];
                    12'd14: up_rdata <= cmds[14];
                    12'd15: up_rdata <=	cmds[15];
                    12'd16: up_rdata <= cmds[16];
                    12'd17: up_rdata <= cmds[17];
                    12'd18: up_rdata <= cmds[18];
                    12'd19: up_rdata <= cmds[19];
                    12'd20: up_rdata <= cmds[20];
                    12'd21: up_rdata <= cmds[21];
                    12'd22: up_rdata <= cmds[22];
                    12'd23: up_rdata <= cmds[23];
                    12'd24: up_rdata <= cmds[24];
                    12'd25: up_rdata <=	cmds[25];
                    12'd26: up_rdata <= cmds[26];
                    12'd27: up_rdata <= cmds[27];
                    12'd28: up_rdata <= cmds[28];
                    12'd29: up_rdata <= cmds[29];
                    12'd30: up_rdata <= cmds[30];
                    12'd31: up_rdata <= cmds[31];
                    12'd32: up_rdata <= cmds[32];
                    12'd33: up_rdata <= cmds[33];
                    12'd34: up_rdata <= cmds[34];
                    12'd35: up_rdata <= cmds[35];
                    12'd36: up_rdata <= cmds[36];
                    12'd37: up_rdata <= cmds[37];
                    12'd38: up_rdata <= cmds[38];
                    12'd39: up_rdata <= cmds[39];
                    12'd40: up_rdata <= cmds[40];
                    12'd41: up_rdata <= cmds[41];
                    12'd42: up_rdata <= cmds[42];
                    12'd43: up_rdata <= cmds[43];
                    12'd44: up_rdata <= cmds[44];
                    12'd45: up_rdata <= cmds[45];
                    12'd46: up_rdata <= cmds[46];
                    12'd47: up_rdata <= cmds[47];
                    12'd48: up_rdata <= cmds[48];
                    12'd49: up_rdata <= cmds[49];
                    12'd50: up_rdata <= cmds[50];
                    12'd51: up_rdata <= cmds[51];
                    12'd52: up_rdata <= cmds[52];
                    12'd53: up_rdata <= cmds[53];
                    12'd54: up_rdata <= cmds[54];
                    12'd55: up_rdata <= cmds[55];
                    12'd56: up_rdata <= cmds[56];
                    12'd57: up_rdata <= cmds[57];
                    12'd58: up_rdata <= cmds[58];
                    12'd59: up_rdata <= cmds[59];                    
                    //                           
                    //read registers
                    12'd60: up_rdata <= ref_dpram_read;
                    12'd61: up_rdata <= xy_pos_error_float_mon;
                    12'd62: up_rdata <= Ut_RamDataMon; //Ut DPRAM read 
                    12'd63: up_rdata <= eigenReadData;
                    12'd64: up_rdata <= sysStatus; 
                    12'd65: up_rdata <= SDI_data_readout;                                  
                    12'd66: up_rdata <= V_RamData_pMon;
                    12'd67: up_rdata <= V_RamData_iMon;
                    12'd68: up_rdata <= V_RamData_dMon;
                    12'd69: up_rdata <= VOut_ram_mon_data;
                    //
                    12'd70: up_rdata <= dma_turns_count_0;
                    12'd71: up_rdata <= dma_turns_count_1;
                    12'd72: up_rdata <= dma_turns_count_2;
                    
                    12'd74: up_rdata <= evr_TimeStamp[63:32];
                    12'd75: up_rdata <= evr_TimeStamp[31:0];
                    12'd76: up_rdata <= evr_trigLatchedTimeStamp[63:32];
                    12'd77: up_rdata <= evr_trigLatchedTimeStamp[31:0];
                    12'd78: up_rdata <= CWCRCErrorCount;
                    12'd79: up_rdata <= sa_count;                    
                    
                    12'd80: up_rdata <= trig_count;
                    // read eigne I sum and D sum
                    12'd81: up_rdata <= eigenSumI_ReadData;
                    12'd82: up_rdata <= eigenSumD_ReadData;
                    

                    12'd120: up_rdata <= `FIRMWARE_VER32;
                      
                  default:
                        up_rdata <= 32'h12345678;
                endcase
            end
        end
    end

    // control
    always @(posedge sysClk) begin
        if (reset == 1'b1) begin
            up_wack <= 1'b0;
            //
            cmds[CMDR_EVR_TRIG_EVENT_SET] <= 32'd15;  //event code
            cmds[CMDR_POS_ERROR_UNIT_DATA] <= 32'h3a83126f;  //0.001 um
            cmds[CMDR_UT_RDLY] <= 32'd13;
            cmds[CMDR_UT_CAL_LENGTH] <= 32'd479;            
            cmds[CMDR_PS_AMP2DAC_GAIN] = 32'h48ccccb3; //524287 / 1.25(A) = 419429.6
            
            cmds[CMDR_USR_POS_ERR_LMT_SET] = 32'd200000; //200 um position error limit
        end else begin
                up_wack <= up_wreq;
            if (up_wreq) begin
                case (up_waddr)
                    12'd00: cmds[0]  <= up_wdata;	
                    12'd01: cmds[1]  <= up_wdata;	
                    12'd02: cmds[2]  <= up_wdata;	
                    12'd03: cmds[3]  <= up_wdata;
                    12'd04: cmds[4]  <= up_wdata;
                    12'd05: cmds[5]  <= up_wdata;
                    12'd06: cmds[6]  <= up_wdata;                     
                    12'd07: cmds[7]  <= up_wdata;
                    12'd08: cmds[8]  <= up_wdata;
                    12'd09: cmds[9]  <= up_wdata;
                    //
                    12'd10: cmds[10] <= up_wdata;
                    12'd11: cmds[11] <= up_wdata;
                    12'd12: cmds[12] <= up_wdata;
                    12'd13: cmds[13] <= up_wdata;
                    12'd14: cmds[14] <= up_wdata;
                    12'd15: cmds[15] <= up_wdata;
                    12'd16: cmds[16] <= up_wdata;
                    12'd17: cmds[17] <= up_wdata;
                    12'd18: cmds[18] <= up_wdata;   
                    12'd19: cmds[19] <= up_wdata;   
                    12'd20: cmds[20] <= up_wdata;   
                    //
                    12'd21: cmds[21] <= up_wdata;  
                    12'd22: cmds[22] <= up_wdata;
                    12'd23: cmds[23] <= up_wdata;
                    12'd24: cmds[24] <= up_wdata;  
                    12'd25: cmds[25] <= up_wdata;
                    12'd26: cmds[26] <= up_wdata;
                    12'd27: cmds[27] <= up_wdata;  
                    12'd28: cmds[28] <= up_wdata;
                    12'd29: cmds[29] <= up_wdata;
                    12'd30: cmds[30] <= up_wdata;  
                    12'd31: cmds[31] <= up_wdata;
                    12'd32: cmds[32] <= up_wdata;                   
                    12'd33: cmds[33] <= up_wdata;
                    12'd34: cmds[34] <= up_wdata;
                    12'd35: cmds[35] <= up_wdata;
                    12'd36: cmds[36] <= up_wdata;
                    12'd37: cmds[37] <= up_wdata;
                    12'd38: cmds[38] <= up_wdata;
                    12'd39: cmds[39] <= up_wdata;
                    12'd40: cmds[40] <= up_wdata;
                    12'd41: cmds[41] <= up_wdata;
                    12'd42: cmds[42] <= up_wdata;                   
                    12'd43: cmds[43] <= up_wdata;
                    12'd44: cmds[44] <= up_wdata;
                    12'd45: cmds[45] <= up_wdata;
                    12'd46: cmds[46] <= up_wdata;
                    12'd47: cmds[47] <= up_wdata;
                    12'd48: cmds[48] <= up_wdata;
                    12'd49: cmds[49] <= up_wdata;
                    12'd50: cmds[50] <= up_wdata;                   
                    12'd51: cmds[51] <= up_wdata;
                    12'd52: cmds[52] <= up_wdata;                   
                    12'd53: cmds[53] <= up_wdata;
                    12'd54: cmds[54] <= up_wdata;
                    12'd55: cmds[55] <= up_wdata;
                    12'd56: cmds[56] <= up_wdata;
                    12'd57: cmds[57] <= up_wdata;
                    12'd58: cmds[58] <= up_wdata;
                    12'd59: cmds[59] <= up_wdata;
                                              
                    
                endcase
            end
        end
    end

    
    /////////////////////////////////////////////////////////////////////
    (* mark_debug = DEBUG *) wire [31:0] eigen_sum_out;
	(* mark_debug = DEBUG *) wire [31:0] eigen_sum_integ_out;
	(* mark_debug = DEBUG *) wire [31:0] eigen_sum_delta_out;
	(* mark_debug = DEBUG *) wire eigen_vec_tvalid;
	(* mark_debug = DEBUG *) wire eigen_vec_tlast;
	(* mark_debug = DEBUG *) wire [31:0] PsSet_muxDatOut;
	(* mark_debug = DEBUG *) wire ps_mux_tvalid;
	(* mark_debug = DEBUG *) wire ps_mux_tlast;
    (* mark_debug = DEBUG *)wire ref_ram_out_tvalid;
    (* mark_debug = DEBUG *)wire ref_ram_out_tlast;
    wire dma_s_axis_0_tlast;
    wire dma_s_axis_1_tlast;
    wire dma_s_axis_2_tlast;
    
    	    
    assign usr_soft_trig = cmds[CMDR_CTRL0][0];  //soft trigger
    // trigger generator
    soft_event_trig  usr_trig_sync
    (
        .clk           (sysClk),
        .reset         (reset),
        .clk_enable    (1'b1),
        .evg_trig      (evr_fa_event), //DspMatrixCalStartTrig2 ),	//10 kHz trigger
        .evg_soft_event(usr_soft_trig),		        //my soft trigger
        .evr_TS        (evr_TimeStamp),
        .trig_out      (usr_sync_trig),
        .trig_count    (trig_count),
        .evr_trigLatchedTimeStamp (evr_trigLatchedTimeStamp)
    );

    //////////////////////////////////
    // ADMA-0 Eigen PI
    //////////////////////////////////    
    
	DMA_TransferSync # (
        .DEBUG ("false")
    ) DMA_TransferSync_ch0 
    (
        .sysClk         (sysClk),
        .reset          (1'b0),
        .P0_trigger     (DspMatrixCalStartTrig2),
		.xfer_trigger   (dam_transfer_start_trig_0),	//start trigger
        .dma_xfer_req   (s_axis_xfer_req_0 & s_axis_0_tready),  //
        .dma_xfer_enable(dma_xfer_enable_0),
        .turns_cnt      (dma_turns_count_0)
    );  

    
    assign dam_transfer_start_trig_0 = usr_sync_trig ;

    //ADMA-0    
    assign s_axis_0_tdata[7:0]   = eigen_sum_out[31:24];
    assign s_axis_0_tdata[15:8]  = eigen_sum_out[23:16];
    assign s_axis_0_tdata[23:16] = eigen_sum_out[15:8];
    assign s_axis_0_tdata[31:24] = eigen_sum_out[7:0];
    //
    assign s_axis_0_tdata[32+7:32+0]   = eigen_sum_delta_out[31:24];
    assign s_axis_0_tdata[32+15:32+8]  = eigen_sum_delta_out[23:16];
    assign s_axis_0_tdata[32+23:32+16] = eigen_sum_delta_out[15:8];
    assign s_axis_0_tdata[32+31:32+24] = eigen_sum_delta_out[7:0];
        
    assign s_axis_0_tvalid       = dma_xfer_enable_0 & eigen_vec_tvalid;
    assign s_axis_0_tlast        = dma_s_axis_0_tlast;     
        
    //////////////////////////////////
    // ADMA-1 PS OUT
    //////////////////////////////////        
	DMA_TransferSync # (
        .DEBUG ("false")
    ) DMA_TransferSync_ch1 
    (
        .sysClk         (sysClk),
        .reset          (1'b0),
        .P0_trigger     (DspMatrixCalStartTrig2),
		.xfer_trigger   (dam_transfer_start_trig_1),	//start trigger
        .dma_xfer_req   (s_axis_xfer_req_1 & s_axis_1_tready),  //
        .dma_xfer_enable(dma_xfer_enable_1),
        .turns_cnt      (dma_turns_count_1)
    ); 
      
    assign dam_transfer_start_trig_1 = usr_sync_trig ;

    //ADMA-0    
    assign s_axis_1_tdata[7:0]   = PsSet_muxDatOut[31:24];
    assign s_axis_1_tdata[15:8]  = PsSet_muxDatOut[23:16];
    assign s_axis_1_tdata[23:16] = PsSet_muxDatOut[15:8];
    assign s_axis_1_tdata[31:24] = PsSet_muxDatOut[7:0];
    
    assign s_axis_1_tvalid       = dma_xfer_enable_1 & ps_mux_tvalid;
    assign s_axis_1_tlast        = dma_s_axis_1_tlast;          

    
    //////////////////////////////////
    // ADMA-2 SDI OUT
    //////////////////////////////////        
	DMA_TransferSync # (
        .DEBUG ("false")
    ) DMA_TransferSync_ch2 
    (
        .sysClk         (sysClk),
        .reset          (1'b0),
        .P0_trigger     (DspMatrixCalStartTrig2),
		.xfer_trigger   (dam_transfer_start_trig_2),	//start trigger
        .dma_xfer_req   (s_axis_xfer_req_2 & s_axis_2_tready),  //
        .dma_xfer_enable(dma_xfer_enable_2),
        .turns_cnt      (dma_turns_count_2)
    );  
    assign dam_transfer_start_trig_2 = usr_sync_trig ;

    //ADMA-2    
    assign s_axis_2_tdata[7:0]   = bpmPos_Xy_Data[31:24];
    assign s_axis_2_tdata[15:8]  = bpmPos_Xy_Data[23:16];
    assign s_axis_2_tdata[23:16] = bpmPos_Xy_Data[15:8];
    assign s_axis_2_tdata[31:24] = bpmPos_Xy_Data[7:0];
    
    assign s_axis_2_tvalid       = dma_xfer_enable_2 & xy_pos_mask_bit;
    //assign s_axis_2_tlast        = dma_s_axis_2_tlast;    //tlast not used and also working
    


    wire gty_reset;   
    vio_0 vio_0_i (
        .clk(sysClk),           // input wire clk
        .probe_out0(gty_reset)  // output wire [0 : 0] probe_out0
    );
   

    //GTY reference clock
    wire refclk;
    IBUFDS_GTE4 #(
        .REFCLK_EN_TX_PATH  (1'b0),
        .REFCLK_HROW_CK_SEL (2'b00),
        .REFCLK_ICNTL_RX    (2'b00)
    ) IBUFDS_GTE4_MGTREFCLK1_X0Y4_INST (
        .I     (evr_gty_refclk_p),
        .IB    (evr_gty_refclk_n),
        .CEB   (1'b0),
        .O     (refclk),
        .ODIV2 ()
    );


////////////////////////////////////
`define __EVR__
`ifdef __EVR__   
    /*
     * EVR
     */
    wire evr_rxcommadet;
    wire evr_refClk;
    wire evr_RxCLK;
    wire evr_gtpowergood;
    wire [4:0] evr_status;
    wire evr_timeSync;
    wire [7:0] EventStream;
    wire [7:0] DBUSStream;
    wire evr_dma_evt_trig;
    wire FrevTBTCLK;
    wire dma_trig;    
    wire evr_timeSync_sys;
    
    
    zcu216_evrTOP evrTOP_i (
        .sysClk(sysClk),
        .reset (reset),
        .refclk (refclk),

        // Serial data ports for transceiver channel 0
        .gtyrxn_in (),
        .gtyrxp_in (),
        .gtytxn_out(),
        .gtytxp_out(),
        //
        .evr_rxcommadet (),
        .evr_userdata_rx(),
        .status    (evr_status), 
        .timeSync_out (evr_timeSync),
        //setting
        .DMATrigEventSet (cmds[CMDR_EVR_TRIG_EVENT_SET][7:0]),
        
        .dma_evt_trig(evr_dma_evt_trig),
        .TimeStamp(evr_TimeStamp),
        .Seconds  (),
        .Offset   (),
        .EventStream(EventStream),
        .DBUSStream (DBUSStream),
        .fa_event_sys (evr_fa_event),	        
        .refclk_o     (evr_refClk),    //156.25MHz
        .RxCLK        (evr_RxCLK)      //124.92 MHz
    );
        
     
`endif
    
           
    ////////////////////////////////////////////
    wire [9:0] gty0_status;  
    wire TxClk;
    (* mark_debug = DEBUG *)wire [31:0] LinkData;
    (* mark_debug = DEBUG *)wire LinkDataValid;
    (* mark_debug = DEBUG *)wire [15:0] LinkDataAddress;
    wire URxCharIsKOut;
    (* mark_debug = DEBUG *)wire LinkStartOfPacket;
    (* mark_debug = DEBUG *)wire LinkEndOfPacket;
    
    
    ////////////////////////////////////////////
    ///// PS OUTPU
    ////////////////////////////////////////////
    wire [32:0] Ps2GTY_tx_data;          
    (* mark_debug = DEBUG *) wire [31:0] CWTxDataIn;
    (* mark_debug = DEBUG *) wire  [7:0] CWTxCharIsKin;
    wire [31:0] LocalData;
    wire LocalDataValid;    
    reg [9:0] IdleSeqCounter=0; 
        
        
    assign LocalData = Ps2GTY_tx_data[31:0];   
    assign LocalDataValid = Ps2GTY_tx_data[32];
    
    always @ (posedge TxClk)
    begin
       //if (reset == 1'b1) IdleSeqCounter <= 10'b0;
       IdleSeqCounter <= IdleSeqCounter + 1;
    end
   
    wire [34:0] IdleSeqOut;            
    assign IdleSeqOut = (IdleSeqCounter[4:0] == 5'b10000) ? ({24'b0,CommaChar,2'b10}) : 33'b0;
      
    assign CWTxDataIn    = (LocalDataValid == 1'b1) ? LocalData : IdleSeqOut[33:2];
    assign CWTxCharIsKin = (LocalDataValid == 1'b1) ? 8'b00000001 : {6'b0000000, IdleSeqOut[1]};   
    
    wire sdi_tvalid;
    wire [9:0] sdi_taddr;
    wire sdi_tlast;  
             
                     
    /*
     * V6 GTX interface
     * sysClk 100 MHz -> GTH core DRP clock 100MHz
     * sysClk 125 MHz -> GTH core DRP clock 125 MHz 
     */
    us_glb_gtx_top  # (
        .DEBUG ("true")
    ) us_glb_gtx_top_i 
    (
        .gtyrxn(),
        .gtyrxp(),
        .gtytxn(),
        .gtytxp(),
        .refclk           (refclk),  //156.25 MHz
        
        // TX for PS OUT
        .TxClock          (TxClk),        
        .userdata_tx      (CWTxDataIn),     //32-bit 
        .userdata_tx_k    (CWTxCharIsKin),  //8-bit
             
        // RECEIVE IN from SDI
        .userclk_rx_usrclk2(rx_usrclk),
        .URxCharIsKOut    (URxCharIsKOut),    
        .LinkDataAddress  (LinkDataAddress),	// address from data packet being processed
        .LinkPacketLength (),   // legth of the data packet being processed
        .LinkData         (LinkData),   // data from the packet being processed
        .LinkDataValid    (LinkDataValid),   // data valid to user application
        .LinkStartOfPacket(LinkStartOfPacket),
        .LinkEndOfPacket  (LinkEndOfPacket),
        //
        .trig             (evr_fa_event),
        .CRC_reset        (cmds[CMDR_CTRL0][2]),  //CRC reset
        .sysClk           (sysClk),
        .reset            (reset || gty_reset),
        
        .CWCRCErrorCount (CWCRCErrorCount),
        // for DPRAM
        .sdi_tvalid (sdi_tvalid),
        .sdi_taddr  (sdi_taddr),
        .sdi_tlast  (sdi_tlast),
        //        
        .status(gty0_status)
    );

      
    //this block includes HEAD__DATA__CRC : total 782 
    XYposDPRAM SDI_pkt_782_i (
        .clka (TxClk),    // input wire clka
        .ena  (1'b1),      // input wire ena
        .wea  (sdi_tvalid), //LinkDataValid),      // input wire [0 : 0] wea
        .addra(sdi_taddr[9:0]),  // input wire [9 : 0] addra
        .dina (LinkData),     // packet Data from SDI link after CRC check
        .douta(),  // output wire [31 : 0] douta
        .clkb (sysClk),    // input wire clkb
        .enb  (cmds[CMDR_SDI_RAM_CTRL][11]),      //B port enable
        .web  (1'b0),     //
        .addrb(cmds[CMDR_SDI_RAM_CTRL][9:0]),  // input wire [9 : 0] addrb
        .dinb (32'd0),    // input wire [31 : 0] dinb
        .doutb(SDI_data_readout)  // output wire [31 : 0] doutb
    );
  
        
    
    //assign  sdi_max_signal    = plb_w_reg[14][19:10];		//max 780 26*30        
	//7/31/13 replaced new module, for variable position length	select
    
    wire DspTrig;
    assign DspTrig = LinkStartOfPacket;
	bpm_pos_read # (
        .DEBUG ("false")
    ) SDI_PositionRead 
    (
		.clk (TxClk), 
		.trig(DspTrig), 
		.cell_max_number(5'd26), //plb_w_reg[14][4:0]),		//5bit  : 7/03/13
		.cell_bmp_number(5'd16), //plb_w_reg[14][9:5]),		//5bit
		.address_move   (10'd0), 
		.AddressStart   (10'd0), 
		.AddressEnd     (10'd780),		    ////MAX 780 = 26(13x2) x 30
		.even_bit(),
		.addr(), 
		.wr_one(), 
		.bpmRam_RD      (dpram_rd),		//SDI DPRAM Read
		.bpmRam_ADDR    (dpram_addr),	//SDI DPRAM address OUTPUT
		.bpmRamCellMark (bpmRamCellMark),
		.cell_cnt(),
		.pos_start_addr(),
		.bram_x_wr  (bram_x_wr),		//Write SDI DATA to Next DPRAM
		.bram_y_wr  (bram_y_wr),		//
		.bram_x_addr(bram_x_addr ),	//Address to Next DPRAM
		.bram_y_addr(bram_y_addr ),	//
		.bpmPosDataValid      (bpmPosDataValid),
		.bpmPosFoFbDataValid  (bpmPosFoFbDataValid),
		.DspMatrixCalStartTrig(DspMatrixCalStartTrig)
	);	    
    
	////////////////////////////////////
	// X,y position data are same memory
	// y offset address is 180
	// Copy only RF BPM data for feedback calculations	
	assign xy_pos_addr = ( (bram_x_wr && bpmPosFoFbDataValid )== 1'b1) ? bram_x_addr : bram_y_addr + `MAX_BPM_NUMBER;	
	
	//7/18/24 for sync with timing
	wire pos_update_freez;
    event_sync event_sync_pos_freez_i(
        .sysClk        (sysClk),
        .reset         (1'b0),
        .valid         (cmds[CMDR_CTRL1][3]),
        .evr_trig      (evr_fa_event),
        .evr_trig_valid(pos_update_freez)  
    ); 
    	
	//cmds[CMDR_CTRL1][3] control for bpm x/y position data 0: Normal, 1: Disable
	//assign Sdi_bpm8_480_DataValid = bpmPosDataValid && bpmPosFoFbDataValid && ~cmds[CMDR_CTRL1][3];
	    
	assign Sdi_bpm8_480_DataValid = bpmPosDataValid && bpmPosFoFbDataValid && ~pos_update_freez;
    
    /*
      Important:
        2 clock delay for align with "Sdi_bpm8_480_DataValid"
        DspTrig and LinkData timing adjustment.
        DspTring and LinkData require 2 clock delay because "bpm_pos_read" module timing.        
     */ 
    reg [31:0] LinkData_r0, LinkData_r1;
    always @ (posedge TxClk ) begin
        LinkData_r0 <= LinkData;
        LinkData_r1 <= LinkData_r0;
    end
    XYposDPRAM XYposDPRAM_sdi_i (
        .clka (TxClk),    // input wire clka
        .ena  (1'b1),      // input wire ena
        .wea  (Sdi_bpm8_480_DataValid), //LinkDataValid),      // input wire [0 : 0] wea
        .addra(xy_pos_addr), //LinkDataAddress[9:0]),  // input wire [9 : 0] addra
        .dina (LinkData_r1),     // packet Data from SDI link after CRC check
        .douta(),  // output wire [31 : 0] douta
        .clkb (sysClk),    // input wire clkb
        .enb  (bpmPosAddrValid),      //B port enable
        .web  (~bpmPosAddrValid),     //
        .addrb(bpmPos_xy_Addr),  // input wire [9 : 0] addrb
        .dinb (32'd0),    // input wire [31 : 0] dinb
        .doutb(bpmPos_Xy_Data)  // output wire [31 : 0] doutb
    );
        
      
    //////////////////////////////////////////////////////////////////
	////// Global RX, Y reference and Position buffer  
	//////////////////////////////////////////////////////////////////
	// Double Buffer A/B
	// 0 - 360 RAM A		x,y same buffer
	// 400 - 511 RAM B
	wire [9:0] ref_rd_addr;
	wire activeRefRAMsel;
	//reference will selectable
	assign 	ref_rd_addr  = ( activeRefRAMsel == 1'b1) ? (bpmPos_xy_Addr+`ACTIVE_RAM_BLOCK_SIZE) :  bpmPos_xy_Addr;
	//assign 	ref_rd_addr  = bpmPos_xy_Addr;  	
    
    XYposDPRAM XYposDPRAM_ref_i (
        .clka (sysClk),    // input wire clka
        .ena  (cmds[CMDR_POS_REF_ADDR][30]),      // input wire ena
        //Input data
        .wea  (cmds[CMDR_POS_REF_ADDR][31]),    // input wire [0 : 0] wea
        .addra(cmds[CMDR_POS_REF_ADDR][9:0]),   // input wire [9 : 0] addra
        .dina (cmds[CMDR_POS_REF_DATA]),        // input wire [31 : 0] dina
        .douta(ref_dpram_read),                        // output wire [31 : 0] douta
        // OUTPUT REFERENCE
        .clkb  (sysClk),            // input wire clkb
        .enb   (bpmPosAddrValid),   // input wire enb
        .web   (1'b0),              // input wire [0 : 0] web
        .addrb (ref_rd_addr),       // input wire [9 : 0] addrb
        .dinb  (32'd0),             // input wire [31 : 0] dinb
        .doutb (ref_xy_data)        // output wire [31 : 0] doutb
    );   
  
  
    wire start_0;
    //(* mark_debug = DEBUG *)wire start;
    gateDelayFast gateDelayFast_i (
        .Clk  (TxClk),
        .Inp  (DspTrig),
        .Delay(32'd10),
        .Width(32'd2),
        .Q    (start_0)  
    );
    irq_forward irq_forward_start_i (TxClk, start_0, sysClk, DspMatrixCalStartTrig2);    
    
    //address generator for Ut matrix calculation
    
	controller ref_read_i (
		.clk          (sysClk),
		.reset        (reset),
		.fofbCalStart (DspMatrixCalStartTrig2),
		.CalcLanth    (cmds[CMDR_UT_CAL_LENGTH][9:0]), // 9'd479), //SysCtrlRegs[8][8:0] ),
		.addRamR      (bpmPos_xy_Addr),
		.addRamR_Valid(bpmPosAddrValid),
		.tlast_t      (ref_ram_out_tlast),
		.tvalid_t     (ref_ram_out_tvalid)		
	);	
        
    
    (* mark_debug = DEBUG *) wire signed [31:0] xy_pos_nm_error;
	//wire signed	[31:0] xy_pos_nm_error_m;
	//assign  xy_pos_nm_error_m = (plb_Reg10[25] == 1'b0) ? xy_pos_nm_error : (bpmPos_Xy_Data - ref_xy_data);
	//assign  xy_pos_nm_error_m = xy_pos_nm_error;
    
    
	assign start_xy_bit = ( bpmPos_xy_Addr[8:0] == 9'd0 || bpmPos_xy_Addr[8:0] == 9'd1 ) ?  0 : 1; 	//0 or 1 
	//assign xy_pos_mask_0 = xy_Mask_data[0] && start_xy_bit;
    assign xy_pos_mask_0 = start_xy_bit;
    
	// orbit error calculation	xy_pos_error_float_mon
	// clk_enable : mask input, if clk_enable is 0 the output data will be zero
	// 6/6/13 for one clock stretching for position mask

	always @ (posedge sysClk) begin	
		xy_pos_mask_reg <= xy_pos_mask_0;
	end	
	assign xy_pos_mask_bit = xy_pos_mask_reg || xy_pos_mask_0;
    
	/////	11/14/2015 reference data selectable user data 		
	//assign  ref_xy_data_mod = (xy_err_fifo_mask_bit == 1'b0) ? SdiGblData2EigenReg : ref_xy_data;
	assign  ref_xy_data_mod = ref_xy_data;

    
	/////////////////////////////////////////////////////
	// November/14/2014
	// +/- 20 mm limit values, if bigger or less then +/- 20 mm, output will 0
//	wire	[31:0] bpmPos_Xy_Data_lmt_o;
//	wire 	out_of_limit_st;
//	aie_limit	_pos_limit
//      (
//       .pos_i(bpmPos_Xy_Data),
//       .valid(),
//       .pos_o(bpmPos_Xy_Data_lmt_o),
//       .valid_n(out_of_limit_st)
//      );
          
	// 04/01/2014 nm_err = rea_ Position - Ref
	nm_sub	xy_err_cal
	(
		.clk  (sysClk),
		.reset(reset),
		.clk_enable ( xy_pos_mask_bit ),	 	// mask control by user, xy_pos_mask_0 need one more width
		.pos_in_nm_i( ref_xy_data ), 		
		.ref_in_nm_i( bpmPos_Xy_Data ),
		//.pos_in_nm_i( ref_xy_data_mod ), 	// 11/14/2015	        
		//.ref_in_nm_i( bpmPos_Xy_Data_lmt_o ),
		.Kn_s32_31b_i(32'd2147),		//mm gain :2.1475e+003, for mm unit:  (0.000001*2^31) = hex
		//
		.nm_err_s32_o (xy_pos_nm_error),
		.err_s32_24b_o(),
		.err_reg_s32_24b_o()
	);

    (* mark_debug = DEBUG *) wire [31:0] xy_pos_nm_error_lmt;
	aie_limit	pos_error_limit (
	   .LIMIT_i (cmds[CMDR_USR_POS_ERR_LMT_SET]),
       .pos_i   (xy_pos_nm_error),
       .valid   (),
       .pos_o   (xy_pos_nm_error_lmt),
       .valid_n ()
    );

    (* mark_debug = DEBUG *) wire	m_axis_result_tvalid;
    (* mark_debug = DEBUG *) wire pos_error_tvalid;
    //conversion floating point for fofb calculation   
	(* mark_debug = DEBUG *) wire [31:0] xy_pos_float_error_nm, xy_pos_float_error; 
	(* mark_debug = DEBUG *) wire m_axis_result_int2float_tlast;
	(* mark_debug = DEBUG *) wire m_axis_result_float_mult_tlast;
    int32tofloat int32tofloat_pos_err_i (
        .aclk(sysClk),                                  // input wire aclk
        .s_axis_a_tvalid(xy_pos_mask_bit),            // input wire s_axis_a_tvalid
        .s_axis_a_tready(),            // output wire s_axis_a_tready
        //.s_axis_a_tdata (xy_pos_nm_error),              // input wire [31 : 0] s_axis_a_tdata
        .s_axis_a_tdata (xy_pos_nm_error_lmt),
        .s_axis_a_tlast (ref_ram_out_tlast),
        //F32 output
        .m_axis_result_tvalid(m_axis_result_tvalid),  // output wire m_axis_result_tvalid
        .m_axis_result_tready(1'b1),  // input wire m_axis_result_tready
        .m_axis_result_tdata(xy_pos_float_error_nm),    // output wire [31 : 0] m_axis_result_tdata
        .m_axis_result_tlast(m_axis_result_int2float_tlast)
    );
    floating_Mult floating_Mult_pos_err_um_i (
        .aclk (sysClk),                                  // input wire aclk
        .s_axis_a_tvalid (m_axis_result_tvalid),            // input wire s_axis_a_tvalid
        .s_axis_a_tready (),            // output wire s_axis_a_tready
        .s_axis_a_tdata  (xy_pos_float_error_nm),              // input wire [31 : 0] s_axis_a_tdata
        .s_axis_a_tlast  (m_axis_result_int2float_tlast),
        //
        .s_axis_b_tvalid (1'b1),            // input wire s_axis_b_tvalid
        .s_axis_b_tready (),            // output wire s_axis_b_tready
        .s_axis_b_tdata  (cmds[CMDR_POS_ERROR_UNIT_DATA]),   // 0.001 nm -> um conversion           
        //
        .m_axis_result_tvalid (pos_error_tvalid),  // output wire m_axis_result_tvalid
        .m_axis_result_tready (1'b1),  // input wire m_axis_result_tready
        .m_axis_result_tdata  (xy_pos_float_error),    // output wire [31 : 0] m_axis_result_tdata
        .m_axis_result_tlast  (m_axis_result_float_mult_tlast)
    );    
    ////////////////////////////////////////////////////
    
    //Position error monitoring
    (* mark_debug = DEBUG *) reg [9:0] pos_err_addr;
	always @ (posedge sysClk) begin  
		if(pos_error_tvalid == 1'b1 ) begin
			pos_err_addr <= pos_err_addr+1;
		end
		else 
		    pos_err_addr <=  10'd0;
	end	    	
    XYposDPRAM XYposDPRAM_float_error_i (
        .clka (sysClk),    // input wire clka
        .ena  (pos_error_tvalid),      // input wire ena
        //Input data
        .wea  (pos_error_tvalid),    // input wire [0 : 0] wea
        .addra(pos_err_addr),   // input wire [9 : 0] addra
        .dina (xy_pos_float_error),        // input wire [31 : 0] dina
        .douta(),                        // output wire [31 : 0] douta
        //
        .clkb (sysClk),     // input wire clkb
        .enb  (cmds[CMDR_POS_REF_ADDR][29]),      // input wire enb
        .web  (1'b0),       // input wire [0 : 0] web
        .addrb(cmds[CMDR_POS_REF_ADDR][9:0]),  // input wire [9 : 0] addrb
        .dinb (32'd0),      // input wire [31 : 0] dinb
        .doutb(xy_pos_error_float_mon) // output wire [31 : 0] doutb
    );      
    
       
        
    reg [`MAX_CORR_NUM:0] Ut_RamEnable, Ut_RamWr, Ut_RamB_Out_enable;    
    wire [31:0] Ut_RamEnable_ctrl = cmds[CMDR_UT_RAM_CTRL];
    //wire [31:0] Ut_RamWr_ctrl     = cmds[CMDR_UT_RAM_WR]; 
           
	//Ut RAM Masks enable/disable
	//11:enable
	//12:WR
	//13: Data, 0/1
	//[9:0] address	
	always @ (posedge sysClk) begin  
		if(Ut_RamEnable_ctrl[11] == 1'b1 & Ut_RamEnable_ctrl[12] == 1'b1 ) begin
			Ut_RamEnable[Ut_RamEnable_ctrl[9:0]] <= Ut_RamEnable_ctrl[13];
			Ut_RamWr[Ut_RamEnable_ctrl[9:0]]     <= Ut_RamEnable_ctrl[14];
			Ut_RamB_Out_enable[Ut_RamEnable_ctrl[9:0]] <= Ut_RamEnable_ctrl[15];
		end
	end	
    
    
    
    (* mark_debug = DEBUG *)wire activeVRAMsel;         
`ifdef  __FEEDBACK_ENABLE__    
    
    /*
     * eigne component calculation (Ut and Vpid)
     */
    UtCal #(
        .MAX_CORR_NUM      (`MAX_CORR_NUM),
        .ONE_CELL_CORR_NUM (`ONE_CELL_CORR_NUM),  
        .DEBUG        ("false")
        ) eigenCompCal (
        .sysClk      (sysClk),
        .reset       (1'b0),
        .fofbCalStart(DspMatrixCalStartTrig2),
        .fofbOn      (),
        //
        .SGain_Set(cmds[CMDR_UT_SGAIN]),
        .UtCalDly (cmds[CMDR_UT_RDLY]),        
        .UtCalcLength(cmds[CMDR_UT_CAL_LENGTH][9:0]), //10'd479),  //480-1
        
        .Ut_RamBlockIndex (cmds[CMDR_UT_RAM_BLK_IDX][8:0]),
        .V_RamBlockIndex  (cmds[CMDR_V_RAM_BLK_IDX][8:0]),        
        //DPRAM Port-A interface
        .Ut_enable       (Ut_RamEnable),   //ram enable/disable
        .Ut_wren         (Ut_RamWr),       //ram wr/rd     
        .Ut_RamB_Out_enable (Ut_RamB_Out_enable),         
        .Ut_RamAddress   (cmds[CMDR_UT_RAM_ADDR][9:0]),      //DPRMA address  
        .Ut_RamWData     (cmds[CMDR_UT_RAM_DATA]),      //DPRAM data write
        .Ut_RamDataMon   (Ut_RamDataMon),  //DPRAM read
        //
        .eigenReadData   (eigenReadData),    
        .eigenSumI_ReadData (eigenSumI_ReadData),
        .eigenSumD_ReadData (eigenSumD_ReadData),     
        .posError        (xy_pos_float_error),  //Position floating value (um) error
        .posError_tvalid (pos_error_tvalid),    //Position error data valid
        .posError_tlast  (m_axis_result_float_mult_tlast),
        // Eigen OUTPUT: F32 type
//        .eigen_sum_vin      (eigen_sum_out),
//        .eigen_sum_integ_vin(eigen_sum_integ_out),
//        .eigen_sum_delta_vin(eigen_sum_delta_out),
//        .eigen_vec_tvalid   (eigen_vec_tvalid),
//        .eigen_vec_tlast    (eigen_vec_tlast),  
        //PID output for DMA interface
        .M01_AXIS_0_tdata  ({eigen_sum_delta_out, eigen_sum_integ_out, eigen_sum_out}),
        .M01_AXIS_0_tlast  (eigen_vec_tlast),
        .M01_AXIS_0_tvalid (eigen_vec_tvalid),       
        // V
        .active_Vram  (activeVRAMsel),
        .VmCalDly     (cmds[CMDR_V_CAL_DELAY][11:0]),
        .PsUpdateDly  (cmds[CMDR_PS_OUT_DELAY][11:0]),
        //
        .f32_pid_sum_gain (cmds[CMDR_PS_PID_SUM_GAIN]),
        .f32UserPsSet (cmds[CMDR_F32_PS_SET]),
        .vm_sel       (cmds[CMDR_VM_SEL]),
        .eigen_d_sel  (cmds[CMDR_CTRL0][1]),    
        .f32_Vout_Amp2DacGain (cmds[CMDR_PS_AMP2DAC_GAIN]),
        .NCO_FreqSet  (cmds[CMDR_NCO_FREQ]),
        .NCO_Kx_Gain  (cmds[CMDR_NCO_GAIN]),        
        .V_enable_P   (cmds[CMDR_V_P_CTRL][`ONE_CELL_CORR_NUM:0]),	
        .V_wren_P     (cmds[CMDR_V_P_CTRL][`ONE_CELL_CORR_NUM+10:10]),
        .V_enable_I   (cmds[CMDR_V_I_CTRL][`ONE_CELL_CORR_NUM:0]),	
        .V_wren_I     (cmds[CMDR_V_I_CTRL][`ONE_CELL_CORR_NUM+10:10]),
        .V_enable_D   (cmds[CMDR_V_D_CTRL][`ONE_CELL_CORR_NUM:0]),	
        .V_wren_D     (cmds[CMDR_V_D_CTRL][`ONE_CELL_CORR_NUM+10:10]),        
        .V_RamAddress (cmds[CMDR_V_RAM_ADDR]),
        .V_RamWData   (cmds[CMDR_V_RAM_DATA]),          
        //V ram read
        .V_RamData_pMon   (V_RamData_pMon),
        .V_RamData_iMon   (V_RamData_iMon),
        .V_RamData_dMon   (V_RamData_dMon),
        //
	    .PsSet_muxDatOut  (PsSet_muxDatOut),
	    .ps_mux_tvalid    (ps_mux_tvalid),   
	    .ps_mux_tlast     (ps_mux_tlast),     
        //
        .VOut_ram_mon_en  (cmds[CMDR_VOUT_RAM_ADDR][9]),
        .VOut_ram_mon_addr(cmds[CMDR_VOUT_RAM_ADDR][7:0]),
        .VOut_ram_mon_data(VOut_ram_mon_data),
        // PS output test mode
        .ps_ctl_mode(cmds[CMDR_CTRL1][2]),
        .testPsData0(cmds[CMDR_USR_PS0_SET][19:0]),
        .testPsData1(cmds[CMDR_USR_PS1_SET][19:0]),
        .testPsData2(cmds[CMDR_USR_PS2_SET][19:0]),
        .testPsData3(cmds[CMDR_USR_PS3_SET][19:0]),
        .testPsData4(cmds[CMDR_USR_PS4_SET][19:0]),
        .testPsData5(cmds[CMDR_USR_PS5_SET][19:0]),
        //
        .gtyTxClk       (TxClk),
	    .Ps2GTY_tx_data (Ps2GTY_tx_data), 
	    .sa_count       (sa_count),                         
        .calDone()    
    );


`endif


    //////////////////////////////////////////////////////
    ///// DAC output
    ///// Send Power Supply Control packets
    
    
    
     event_sync event_sync_ref_i(
        .sysClk        (sysClk),
        .reset         (1'b0),
        .valid         (cmds[CMDR_CTRL1][0]),
        .evr_trig      (evr_fa_event),
        .evr_trig_valid(activeRefRAMsel)  
    );      
   
    event_sync event_sync_i(
        .sysClk        (sysClk),
        .reset         (1'b0),
        .valid         (cmds[CMDR_CTRL1][1]),
        .evr_trig      (evr_fa_event),
        .evr_trig_valid(activeVRAMsel)  
    );   
  
        
    //System status
    assign sysStatus[0] = evr_timeSync;
    assign sysStatus[1] = activeRefRAMsel;   //reference active RAM
    assign sysStatus[2] = activeVRAMsel;
    assign sysStatus[3] = dma_xfer_enable_0;
    assign sysStatus[4] = dma_xfer_enable_1;
    assign sysStatus[5] = dma_xfer_enable_2;
    assign sysStatus[6] = 0; 
    
    ////////////////////////////////////////////////////
    wire clk_10khz;
    blinker #(
        .GEN_CLK_FREQUENCY(`SYSTEM_CLOCK),           
        .GEN_BLINK_PERIOD (0.0001)   //10kHz                  
    )
    clk_10hkz_gen
    (
        .P_I_CLK(sysClk),     
        .P_O_LED(clk_10khz)
    );           
    pedge_detect pedge_detect_i(
        .clk     (sysClk),
        .Reset   (1'b0),
        .trig    (clk_10khz),
        .edge_out(intTrigger)
    );  
    
    wire clk_10hz_rx;
    blinker #(
        .GEN_CLK_FREQUENCY(`SYSTEM_CLOCK),           
        .GEN_BLINK_PERIOD (0.1)   //10Hz                  
    )
    clk_10kz_gen_rx
    (
        .P_I_CLK(rx_usrclk),     
        .P_O_LED(clk_10hz_rx)
    ); 
    
    wire clk_10hz_tx;
    blinker #(
        .GEN_CLK_FREQUENCY(`SYSTEM_CLOCK),           
        .GEN_BLINK_PERIOD (0.1)   //10Hz                  
    )
    clk_10kz_gen_tx
    (
        .P_I_CLK(TxClk),     
        .P_O_LED(clk_10hz_tx)
    ); 
    
    wire clk_10hz_ref;
    blinker #(
        .GEN_CLK_FREQUENCY(156250000),           
        .GEN_BLINK_PERIOD (0.1)   //10Hz                  
    )
    clk_10kz_gen_ref
    (
        .P_I_CLK(refclk_o),     
        .P_O_LED(clk_10hz_ref)
    ); 
    
            
assign RGB_B_LED[0] = clk_10hz_rx;
assign RGB_B_LED[1] = clk_10hz_tx;
assign RGB_B_LED[2] = clk_10hz_ref;
assign RGB_B_LED[3] = evr_timeSync; 

        
endmodule
