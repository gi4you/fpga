`timescale 1ns / 1ps
/*
	2014
	ZYNQ processor

	Timing fixed at ZYNQ
		1) PacketStartBit
		2) TxBufferFill
		3) TxBufferFlush
		
	6/09/16
		CRC reset, CRC error count port added.
	
	
	8/08/16
		Trigger is not syncronized with V6, so do not use Trigger for reset.
		
*/


module RemotePacketProcessor #
(
	parameter ILA_ENABLED           =   0
)
(
		// RocketIO receiver interface
		input Reset,
		input CRC_reset,		//kha
		input Trigger,
		input Idle,
		input RioRxClock,
		input [31:0] RxData,
		input RxCharIsK,
		input [15:0] MyStopAddress,		
		// RocketIO transmitter interface
		input RioTxClock,
		input TxBufferFlush,
						
		output wire [33:0] TxData, 	// bit 1 = start of packet (CharIsK) and bit 0 = end of packet
`ifdef	RXFIFO_MONITORING		
		output [31:0] RemoteRxFifoData,		//kha for monitoring remote packets
		output RemoteRxFifoDataValid,
`endif		
		// flow control
		output PacketReady,
		output wire TxBufferEmpty,
		output PacketDropped,
		 output reg [31:0] CRCErrorCount,
		 output reg LocalPacketRcvdOnRemote,
		output reg RemoteModeDone,
		/* Local loopback data */
		output  LocalRxPacketDataValid, 
		output	RxFifoEmpty_o,
		output  myReceivedLocalPacketHead,
		output	[5:0] ReStateDbg,
		
		//packt drop debugging
		input	RxPktFifoRead,
		output	wire [71:0] rxPktFifoOutData,
		output  wire [33:0] TxPacketBufferData,
		output	wire		TxFifoWriteEnable
	
    );

	
	parameter K_START = 8'h5C;	  		// k28.2 character marks packet start
	
	parameter CRC_INIT = 32'hFFFFFFFF;
	parameter CONST_REMOTE_TIMEOUT = 16'd6500; 	// 24 us 
	  	
	parameter IDLE    = 4'b001;
	parameter SIGNAL  = 4'b010;
	parameter FLUSH   = 4'b011;
	parameter CLEAR   = 4'b100;	 
	parameter WAIT0   = 4'b101;
	parameter WAIT1   = 4'b110;
	

	//(* FSM_ENCODING="ONE-HOT", SAFE_IMPLEMENTATION="NO" *) 
	reg [2:0] state = IDLE;	 

	reg RxBufferFlush;
	reg [31:0] InputData_reg;
	reg [11:0] LinkPacketLength;
	reg [15:0] LinkPacketAddress;
	reg [11:0] PacketLengthCounter;
	reg RxFIFOfillEna;
	reg CheckCRC;
	reg	CheckCRC2;
	wire	TxBufferFill;
	reg [11:0] PacketFlushCounter;				 
	//reg PacketStartBit;
	wire	PacketStartBit;	//09/18/13 for Zynq
	reg PacketStopBit;
	//reg [31:0]RxCalcCRCvalue_reg;

	wire [31:0] RxTxData;
	wire [11:0] myPacketLength;
	wire StartOfPacket; 
	//wire [31:0] RxCalcCRCvalue;
	wire PacketReadyFIFOwrEna;
	wire NoPacketsInQue;
	//wire [33:0] TxPacketBufferData;	
	wire RxBufferEmpty;
	wire [31:0] CRCout;
	reg	PacketDropped_i;		
	reg	[31:0] LoopCnt; 	 
	reg RxFifoValid;
	reg LocalRxBufferFill;
	wire   RemoteRxFifoDataValid; 
	reg [33:0] TxPacketBufferData_Reg, TxPacketBufferData_Reg2;
	reg TxBufferFill_Reg, TxBufferFill_Reg2;
	wire [33:0] tx_data_mon;
	
	 // add a variable for the packet length coming in on the RocketIO link	 
	// assign myPacketLength = RxData[31:24];	 	
	assign myPacketLength = RxData[31:20];	 
	assign PacketReady = ~NoPacketsInQue;
	 
	 /*
	 *	test skip 'PacketDropped_i' status for fifo control
	 */
	 //assign PacketReadyFIFOwrEna = (PacketStopBit) && (state!=FLUSH) && (!PacketDropped_i);	//if PacketStopBit detected transmitter next cell
	 
	 // 06/07/16 : removed state because timing issue
	 assign PacketReadyFIFOwrEna = (PacketStopBit) && (!PacketDropped_i);
	 
	 // detect an incoming packet
	 assign StartOfPacket = ((RxCharIsK) && (RxData[7:0] == K_START) ); 	//&& (RxData[31:24] == PACKET_LENGTH ));	 	//08/03/14 added Length
	 
	 assign RemoteRxFifoDataValid = RxBufferFlush && RxFifoValid; 		
	 assign RemoteRxFifoData      = RxTxData;	 
	 assign RxFifoEmpty_o         = RxBufferEmpty;	/* July 17 for timeout check */

	 wire   ReceivedLocalPacketHead;
	// assign ReceivedLocalPacketHead =  ( (RxTxData[7:0] == K_START) && RxTxData[23:8] == MyStopAddress );	 
	// assign myReceivedLocalPacketHead = ((RxCharIsK) && (RxData[7:0] == K_START) && (RxData[23:8] == MyStopAddress));
	 
	 assign ReceivedLocalPacketHead =  ( (RxTxData[7:0] == K_START) && RxTxData[19:8] == MyStopAddress );	 
	 assign myReceivedLocalPacketHead = ((RxCharIsK) && (RxData[7:0] == K_START) && (RxData[19:8] == MyStopAddress));
	 
	 /*
	  *  Local Loopback remote Data Valid
	  *        --[HEAD][X][Y][CRC][00][00]-------
	  *           --------------- 
	  *        __|               |_______           RemoteRxFifoDataValid
	  *
	  *                 ------------------          LocalRxBufferFill
	  *        ________|                  |_______  
      *	  
	  *            ----    
	  *        ___|    |___________________________
	  *            --------------
	  *		   ___|              |________________  LocalRxPacketDataValid
	  */
	 
	 
	 assign LocalRxPacketDataValid = (LocalRxBufferFill || ReceivedLocalPacketHead) && RemoteRxFifoDataValid;

	 
	 always @ (posedge RioRxClock)
	 begin
		if (Reset==1'b1) RxFifoValid <= 1'b1;
		else if (PacketFlushCounter <= 12'd2) RxFifoValid <= 1'b0;
		else RxFifoValid <= RxFifoValid <= 1'b1;
	 end	 
	 
	 
	 // delay incoming data by one clock cycle
	 always @ (posedge RioRxClock)
	 begin
		InputData_reg <= RxData;
		//RxCalcCRCvalue_reg <= RxCalcCRCvalue;
	 end
	 	 
	 // if the start of packet is detected store the packet length and address
	 always @ (posedge RioRxClock)
	 begin
		if (StartOfPacket)
			begin
				LinkPacketLength  <= myPacketLength;	// RxData[31:24] Packet length in words
				//LinkPacketAddress <= RxData[23:8];
				LinkPacketAddress <= RxData[19:8];
			end	 
		else if (state == CLEAR)
			begin
				LinkPacketAddress <= 16'b0;	// Packet length in words
			end
		else
			begin
				LinkPacketLength <= LinkPacketLength;	// Packet length in words
				LinkPacketAddress <= LinkPacketAddress;
			end
	 end
	 
	 // packet length counter is used to enable the input data going to
	 // the packet FIFO "myPacketBuffFIFO" for the lenght of the current packet 
	 always @ (posedge RioRxClock)
	 begin
		if (Reset==1'b1) 
			PacketLengthCounter <= 12'b0;
		else if (StartOfPacket) 
			PacketLengthCounter <= ({1'b0,((myPacketLength)+1)}); //add one for crc check
		else if (PacketLengthCounter > 12'b0) 
			PacketLengthCounter <= PacketLengthCounter - 1;
		else 
			PacketLengthCounter <= PacketLengthCounter;
	 end
	 
	 // packet FIFO input enable signal active for the length of the packet
	 always @ (posedge RioRxClock)
	 begin
		if ((Reset==1'b1) || ((RxFIFOfillEna == 1'b1) && (PacketLengthCounter == 12'd0) && (StartOfPacket == 1'b0))) 
			RxFIFOfillEna <= 1'b0;
		else if (StartOfPacket == 1'b1) 
			RxFIFOfillEna <= 1'b1;
		else 
			RxFIFOfillEna <= RxFIFOfillEna;
	 end

	 ////////////////////////////////////////////////////////////////////////////////
	 // CRC checking section
	 // Uses the CRC hardware built into the Virtex 5. The incoming CRC value is stored
	 // in the packet. The clock tick after the packet FIFO is filled the CRC block has 
	 // the correct CRC value to be compared against the packet CRC value.	

	 // signal to initiate a CRC check
	 always @ (posedge RioRxClock)
	 begin
		if (Reset == 1'b1 )	
			CheckCRC <= 1'b0;
		else if (PacketLengthCounter == 12'd3)				
			CheckCRC <= 1'b1;
		else if (PacketLengthCounter == 12'd2)
			CheckCRC <= 1'b0;
		else CheckCRC <= CheckCRC;
	 end
	 
	 ////////////////////////////////////////////////////////////////////////////////
	 // After the CRC value is verified the input packet buffer needs to be flushed.
	 // If the CRC value is correct the buffer is flushed into the transmit buffer
	 // so it can be forwarded. If not the data is dumped. This packet length counter
	 // is used to time the buffer flush wherever the data goes.
	 
	 // packet FIFO Packet flush counter  
	 always @ (posedge RioRxClock)
	 begin
		if (Reset==1'b1) 
			PacketFlushCounter <= 12'b0;
		else if (CheckCRC == 1'b1) 
			//PacketFlushCounter <= ({1'b0,(LinkPacketLength + 1)}); // initialize the counter
			PacketFlushCounter <= LinkPacketLength + 1; // initialize the counter
		else if (RxBufferFlush == 1'b1) 
			PacketFlushCounter <= PacketFlushCounter - 1;
		else 
			PacketFlushCounter <= PacketFlushCounter;
	 end
	 
	 // packet FIFO flush enable signal active for the length of the packet	 
	 always @ (posedge RioRxClock)
	 begin
		if ((Reset==1'b1) || ((PacketFlushCounter == 12'd0) && !CheckCRC) || (RxBufferEmpty))
			RxBufferFlush <= 1'b0;
		else if (CheckCRC || (state == FLUSH) || (Idle == 1'b1)) 
			RxBufferFlush <= 1'b1;
		else 
			RxBufferFlush <= RxBufferFlush;
	 end	 

	 
	 // If the CRC check is good and this is not the local data 
	 // then fill the transmit buffer otherwise dump the data
	 // 
/*	 
	 always @ (posedge RioRxClock)
	 begin
		if ((Reset==1'b1) || ((PacketFlushCounter == 9'd0) && !CheckCRC) || (RxBufferEmpty))
			TxBufferFill <= 1'b0;
		else if ((CheckCRC) && (CRCout == InputData_reg) && !(state == FLUSH) && (RxTxData[23:8] != MyStopAddress) && !Idle)
			TxBufferFill <= 1'b1;
		else 
			TxBufferFill <= TxBufferFill;
	 end	 
*/

	 // 09/21/13
	 // Correction timing for ZYNQ
	 // RxTxData is 2 clock delayed from CheckCRC
	 reg 	IsLocakPacket;
	 reg	[0:0] dly_reg;
	 always @ (posedge RioRxClock)
	 begin
		if ((Reset==1'b1) || ((PacketFlushCounter == 12'd0) && !CheckCRC) || (RxBufferEmpty))
			dly_reg[0] <= 1'b0;	//clear
		else if ((CheckCRC) && (CRCout == InputData_reg) && !(state == FLUSH) && !Idle) begin
			dly_reg[0] <= 1'b1;	
		end	else 
			dly_reg[0] <= dly_reg[0];
	 end
	 
	always @ (posedge RioRxClock) begin
		IsLocakPacket <= dly_reg[0];	//1 clk delay
	end		
	 
	 reg	regTxBufferFill;
	 always @ (posedge RioRxClock)
	 begin
		if ((Reset==1'b1) || ((PacketFlushCounter == 12'd0) && !CheckCRC) || (RxBufferEmpty))
			regTxBufferFill <= 1'b0;
		//else if ( IsLocakPacket == 1'b1 && (RxTxData[19:8] != MyStopAddress) && (RxTxData[7:0] == K_START) && (RxTxData[31:20] == LinkPacketLength) )  
		else if ( IsLocakPacket == 1'b1  && (RxTxData[7:0] == K_START) && (RxTxData[31:20] == LinkPacketLength) )  
			regTxBufferFill <= 1'b1;
		else 
			regTxBufferFill <= regTxBufferFill;
	 end		
	 
	 //assign TxBufferFill = PacketStartBit | regTxBufferFill;
	 //assign	TxFifoWriteEnable = TxBufferFill;

	/*
		Kha for Local loopback
		Local loopback data push to FIFO memory		
	*/
	 // packet FIFO input enable signal active for the length of the packet	 	 
	 always @ (posedge RioRxClock)
	 begin
		if ((Reset==1'b1) || ((PacketFlushCounter == 12'd0) && !CheckCRC) || (RxBufferEmpty))
			LocalRxBufferFill <= 1'b0;
		else if ( RxBufferFlush && (RxTxData[19:8] == MyStopAddress) )
			LocalRxBufferFill <= 1'b1;		//write to Local received FIFO
		else 
			LocalRxBufferFill <= LocalRxBufferFill;
	 end
	 
	 	 
	 // if the CRC check fails flag the error	 
	 always @ (posedge RioRxClock)
	 begin
		if ((Reset==1'b1) || (PacketFlushCounter == 12'd2))
			PacketDropped_i <= 1'b0;
		else if ((CheckCRC) && (CRCout != InputData_reg) && !Idle)
			PacketDropped_i <= 1'b1;
		else 
			PacketDropped_i <= PacketDropped_i;
	 end	 
	 
	 always @ (posedge RioRxClock)
	 begin
		if ( Reset==1'b1  || CRC_reset == 1'b1 ) 	//crc clear
			begin
				CRCErrorCount <= 32'b0;
			end	
		else if ((CheckCRC) && (CRCout != InputData_reg) && !Idle)
			begin
				CRCErrorCount <= (CRCErrorCount + 1);
			end
		else
			begin 
				CRCErrorCount <= CRCErrorCount;
			end
	 end	 	 
	 
	 ////////////////////////////////////////////////////////////////////////////
	 // Flow control section. This section adds markers for the beginning and end
	 // of an incoming packet. The begining of a packet is also used to signal
	 // a "k" character to the Tx Rio port. It's placed in bit '1' of the Tx FIFO
	 // The last word of the packet is marked by a 1 in bit '0' of the Tx FIFO.
`ifdef	AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA	 
	 always @ (posedge RioRxClock)
	 begin
		if (Reset==1'b1) PacketStartBit <= 1'b0;
		//else if (CheckCRC) PacketStartBit <= 1'b1; //&& (CRCout == InputData_reg) && !(state == FLUSH) && !(LinkPacketAddress == MyStopAddress)) PacketStartBit <= 1'b1;  //1 clock before
		else if ( CheckCRC && (CRCout == InputData_reg) && !(state == FLUSH) && !(LinkPacketAddress == MyStopAddress)) PacketStartBit <= 1'b1;	//9/18/13 switch 
		else PacketStartBit <= 1'b0;
	 end
	 
`endif

//	assign PacketStartBit = (IsLocakPacket == 1'b1) && (RxTxData[23:8] != MyStopAddress) && (RxTxData[7:0] == K_START) && (RxTxData[31:24] == LinkPacketLength) ? 1'b1 : 1'b0;
	assign PacketStartBit = (IsLocakPacket == 1'b1)  && (RxTxData[7:0] == K_START) && (RxTxData[31:20] == LinkPacketLength) ? 1'b1 : 1'b0;
	
	
	 always @ (posedge RioRxClock)
	 begin
		if (Reset==1'b1) PacketStopBit <= 1'b0;
		else if (PacketFlushCounter == 12'd3) PacketStopBit <= 1'b1;
		else PacketStopBit <= 1'b0;
	 end	  
	 
	 
	 // need to know if all data has been circulated. To detect this look for a 
	 // local packet that comes in on the remote data stream. If this occurs don't
	 // signal packet ready and flush the packet from the input buffer. This only
	 // works if the link is complete and all nodes in the loop are connected
	 /*
	parameter IDLE    = 4'b001;
	parameter SIGNAL  = 4'b010;
	parameter FLUSH   = 4'b011;
	parameter CLEAR   = 4'b100;	 
	parameter WAIT0   = 4'b101;
	parameter WAIT1   = 4'b110;
*/	
    always@(posedge RioRxClock)
      if (Reset) begin
	  //if (Reset | Trigger) begin
         state <= IDLE;
      end
      else
         //(* FULL_CASE, PARALLEL_CASE *)case (state)
		 case (state)
            IDLE : begin
				//if  ((LinkPacketAddress == MyStopAddress) && (CheckCRC))	
				if  ( CheckCRC )		//07/20/15  Skip Stop Address
					state <= SIGNAL;	//2
				else
					state <= IDLE;
				end				
            SIGNAL : state <= FLUSH; //3				
            FLUSH : begin
				if (RxBufferEmpty)
					state <= CLEAR;	//4
				else 
					state <= FLUSH;	//3
				end				
			CLEAR: state <= WAIT0;		// 2 clock extension for sync with TX clock
			WAIT0: state <= WAIT1;	
			WAIT1: state <= IDLE;		// 1
         endcase
   
   
	//LocalPacketRcvdOnRemote -- sometimes missing then time out		
    always@(posedge RioRxClock)
	 begin
      if (state == IDLE) LocalPacketRcvdOnRemote <= 1'b0;
		else if (state == SIGNAL) LocalPacketRcvdOnRemote <= 1'b1;			
		else if (state == CLEAR || state == WAIT0  || state == WAIT1) LocalPacketRcvdOnRemote <= 1'b0;	//VIRTEX-6  2 clock extension
		//else if (state == WAIT1) LocalPacketRcvdOnRemote <= 1'b0;		//ZYNQ 2014 4/6  2 clock extention......
		else LocalPacketRcvdOnRemote <= LocalPacketRcvdOnRemote;
	 end
	 
	 //  RemoteModeDone -- always good
	 always@(posedge RioRxClock)
	 begin
      if (Reset) RemoteModeDone <= 1'b0;
		if (((state == FLUSH) && RxBufferEmpty) || (state == CLEAR)) RemoteModeDone <= 1'b1;
		//if ( state == CLEAR) RemoteModeDone <= 1'b1;	//July 17
		else RemoteModeDone <= 1'b0;
	 end
	
	
			 
	 //assign TxPacketBufferData = {RxTxData,PacketStartBit,PacketStopBit};	 
	 
	 /*
	  * 06/01/2015
	  *	Added double register 
	  *
	  */
    always @(posedge RioRxClock)
    begin
        begin
            TxPacketBufferData_Reg <=  #1  {RxTxData,PacketStartBit,PacketStopBit};
			TxPacketBufferData_Reg2 <= #1  TxPacketBufferData_Reg;
        end
    end
		
    always @(posedge RioRxClock)
    begin
        begin
            TxBufferFill_Reg <=  #1  PacketStartBit | regTxBufferFill;
			TxBufferFill_Reg2 <= #1  TxBufferFill_Reg;
        end
    end
	
	assign TxPacketBufferData = TxPacketBufferData_Reg2;
	assign TxBufferFill      = TxBufferFill_Reg2;
	assign TxFifoWriteEnable = TxBufferFill_Reg2;
	 
	 
	 
	//FIFO reset
	//Trigger reset need to correct timing
	wire   FIFO_Rst;
	assign FIFO_Rst = ( Reset==1'b1 || Trigger == 1'b1 );	 
	//assign FIFO_Rst = ( Reset==1'b1  || CRC_reset == 1'b1 || Trigger == 1'b1 );	 
	//assign FIFO_Rst = ( Reset==1'b1  || CRC_reset == 1'b1 );
	///////////////////////////////////	  
	 
	 // FIFO memory for storing incoming RIO packets while CRC is verified
	 ReceivePacketFIFO myReceivePacketFIFO (
		.clk   (RioRxClock),
		.rst   (FIFO_Rst),
		.din   (InputData_reg),  
		.wr_en (RxFIFOfillEna),
		.rd_en (RxBufferFlush),
		.dout  (RxTxData), 	//[31:0]
		.full  (),
		.empty (RxBufferEmpty)
		);
	  	
		
	wire [33:0] TxDataFifoOut;
	reg [31:0] TxData_tmpReg;	 
	wire TxBufferFull;
	 // FIFO memory for storing incoming packets after CRC check and before
	 // transmitting. This is a holding place while loacal data is transmitted
	 // This FIFO crosses clock domains
	 TxPacketFIFO myTxPacketFIFO (
		.rst    (FIFO_Rst),
		.wr_clk (RioRxClock),
		.rd_clk (RioTxClock),
		.din    (TxPacketBufferData),  	//[33:0] {RxTxData,PacketStartBit,PacketStopBit}
		.wr_en  (TxBufferFill),
		//.rd_en((TxBufferFlush || Idle)),
		.rd_en  (TxBufferFlush ),		
		.dout   (TxData),  			// 33:0]
		//.dout(TxDataFifoOut), 		
		.full   (TxBufferFull),
		.empty  (TxBufferEmpty)
		);

	// added double buffer	
	/*
    always @(posedge RioTxClock)
    begin
        begin
            TxData_tmpReg <=  #1  TxDataFifoOut;
			TxData <=  #1  TxData_tmpReg;
        end
    end
	*/
	assign tx_data_mon = TxData;
		
	 // this FIFO is used for flow control if a complete validated packet 
	 // is ready the output goes high. If a local packet is detected the
	 // output remains low
	 PacketQue myPacketQue (
		.rst(FIFO_Rst),
		.wr_clk(RioRxClock),
		.rd_clk(RioTxClock),
		.din(1'b1), 
		.wr_en(PacketReadyFIFOwrEna),
		.rd_en((TxData[0] || Idle)),
		.dout(),  
		.full(),
		.empty(NoPacketsInQue)
		);
   
    //CRC Calculation
	LocalDataCRC CRC(
		.clk    (RioRxClock),
		.rst    (StartOfPacket),		
		.data_in(InputData_reg),
		.crc_en (RxFIFOfillEna),
		.CRCout (CRCout)
	);	
	assign PacketDropped = PacketDropped_i;
	
	
	/*
	 * Packet debugging when Packet dropped.
	 */
	wire  fifo_rd; 
	
	wire fifo_CheckCRC;
	wire fifo_empty;		
	wire  [31:0] fifo_InData, fifo_CRCout;
	wire   [5:0] fifo_PLcount;
	

	assign fifo_rd = RxPktFifoRead; 	//timing sync with TxPacketFifo
	
`ifdef	AAAAAAAA	
	fifo_72bitx512 	packet_dbg_fifo (
		//.clk(RioRxClock),      // input wire clk
		.wr_clk(RioRxClock),  // input wire wr_clk
		.rd_clk(RioTxClock),  // input wire rd_clk			
		.rst(FIFO_Rst),      // input wire rst
		.din( { PacketDropped_i, CheckCRC, PacketLengthCounter[5:0], CRCout, InputData_reg} ),      // input wire [71 : 0] din
		.wr_en(  RxFIFOfillEna ),  // input wire wr_en
		.rd_en( fifo_rd ),  // input wire rd_en
		//.dout({ fifo_CheckCRC, fifo_PLcount, fifo_CRCout, fifo_InData}),    // output wire [71 : 0] dout
		.dout( rxPktFifoOutData ),
		.full(),    // output wire full
		.empty(fifo_empty)  // output wire empty
	);
	
`endif	
	

	
`ifdef	WATCHDOG	
	 // WD 150 us if over 150 us, timeout_cnt increment
	 //wire [31:0] crc_wd_timeout_cnt;
	 //
	 //  CheckCRC  __|___|___|___|___|___|___|_______~~  __|___|___|___|___|___|___|____
	 //              1   2   3   4   5   6  LOCAL_ID 
	 trig_watchdog crc_wd (
	 		.trig_watchdog_rst(crc_watchdog_rst),
	   		.timeout_cnt(crc_wd_timeout_cnt),
			.trig_cnt(),
	   		.sys_clk_i(RioRxClock), 
	   		.reset(Reset),
			.UsrClear(CRC_reset)
	   		.EXT_TRIG_B(CheckCRC)	//myReceivedLocalPacketHead
	   	);	
`endif


	//Added 04/03/2014	
	wire IdleState, SignalState, FlashState, ClearState;	
	assign IdleState   = (state == IDLE)   ? 1'b1 : 1'b0;
	assign SignalState = (state == SIGNAL) ? 1'b1 : 1'b0;
	assign FlashState  = (state == FLUSH)  ? 1'b1 : 1'b0;
	assign ClearState  = (state == CLEAR)  ? 1'b1 : 1'b0;
	
	
	assign	ReStateDbg = {StartOfPacket, CheckCRC, IdleState, RxBufferEmpty, RxBufferFlush, RxFIFOfillEna};


//generate
//if (ILA_ENABLED == 1) 
//	begin : ILA   	
//		ila_RemotePacketProcessor ILA_RemotePacketProcessor_ (
//			.clk(RioRxClock), // input wire clk
//			.probe0(RxData), // input wire [31:0]  probe0  
//			.probe1(RxTxData), // input wire [31:0]  probe1 
//			.probe2(tx_data_mon[33:2]), // input wire [31:0]  probe2 
//			.probe3(CRCout), // input wire [31:0]  probe3 
//			.probe4(tx_data_mon[0]), // input wire [0:0]  probe4 
//			.probe5(tx_data_mon[1]), // input wire [0:0]  probe5 
//			.probe6(CheckCRC), // input wire [0:0]  probe6 
//			.probe7(StartOfPacket), // input wire [0:0]  probe7 
//			.probe8(RxBufferFlush), // input wire [0:0]  probe8 
//			.probe9(RxFIFOfillEna), // input wire [0:0]  probe9 
//			.probe10(PacketReadyFIFOwrEna), // input wire [0:0]  probe10 
//			.probe11(IdleState), // input wire [0:0]  probe11 
//			.probe12(TxBufferFill), // input wire [0:0]  probe12 
//			.probe13(TxBufferFlush), // input wire [0:0]  probe13 
//			.probe14(TxBufferFull), //Trigger), // input wire [0:0]  probe14
//			.probe15(state), // input wire [2:0]  probe15
//			.probe16(PacketFlushCounter), // input wire [8:0]  probe16
//			.probe17(PacketStartBit), // input wire [0:0]  probe17
//			.probe18(PacketStopBit), // input wire [0:0]  probe18
//			.probe19(RxCharIsK),  // input wire [0 : 0] probe19
//			.probe20(IsLocakPacket),  // input wire [0 : 0] probe20
//			.probe21(PacketLengthCounter),  // input wire [8 : 0] probe21			
//			.probe22(InputData_reg),
//			.probe23(PacketDropped_i),
//			.probe24(LocalPacketRcvdOnRemote),
//			.probe25(RxBufferEmpty),
//			.probe26(TxPacketBufferData[33:2]), //fifo_InData), // input wire [31:0]  probe26 
//			.probe27(32'd0), //fifo_CRCout), // input wire [31:0]  probe27 
//			.probe28(8'd0), //fifo_PLcount), // input wire [7:0]  probe28 
//			.probe29(TxBufferEmpty), //fifo_empty), // input wire [0:0]  probe29 
//			.probe30(PacketReady), 	//fifo_rd), // input wire [0:0]  probe30
//			.probe31(TxPacketBufferData[1]) //fifo_CheckCRC)						
//		);
				
//	end
//endgenerate 



	
 endmodule