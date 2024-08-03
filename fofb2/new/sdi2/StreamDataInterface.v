`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:56:40 01/12/2010 
// Design Name: 
// Module Name:    StreamDataInterface 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//	Aug/28 AddressCounter length 8 -> 16
//		
//
//////////////////////////////////////////////////////////////////////////////////

module StreamDataInterface#(
    parameter DEBUG           = "true"
    )(
        input Clock,
        input Reset,
        (* mark_debug  = DEBUG *)input [31:0] DataIn,
        (* mark_debug  = DEBUG *)input CharIsK,
        (* mark_debug  = DEBUG *)output reg [15:0] MemoryAddress,
        (* mark_debug  = DEBUG *)output reg [11:0] PacketAddress,
        (* mark_debug  = DEBUG *)output reg [11:0] PacketLength,
        (* mark_debug  = DEBUG *)output reg [31:0] DataOut,
        (* mark_debug  = DEBUG *)output reg DataValid,
        (* mark_debug  = DEBUG *)output reg LinkStartOfPacket,
        (* mark_debug  = DEBUG *)output reg LinkEndOfPacket,
         output sdi_tvalid,
         output [9:0] sdi_taddr,
         output sdi_tlast         
    );

	//HEAD
	//34 bit    12+12+8+1+1
	//{SourcePacketSize, SourcePacketDest, KStart, 1'b1, 1'b0}
	
	 parameter KStart = 8'h5C;		//b 01011100; 	// k28.2 character marks packet start

	 reg [31:0] InputData_reg;		
	 (* mark_debug  = DEBUG *)reg [11:0] PacketLengthCounter;
	 reg DataValid_reg;
	 (* mark_debug  = DEBUG *)reg LinkStartOfPacket_reg, LinkEndOfPacket_reg;
	 //reg [7:0] AddressCounter;
	 (* mark_debug  = DEBUG *)reg [15:0] AddressCounter;		// need extension
	 
	 (* mark_debug  = DEBUG *)wire StartOfPacket;
	 wire [11:0] RIOpacketLength;
	 
	 
	 assign StartOfPacket = ((CharIsK) && (DataIn[7:0] == KStart));
	 assign RIOpacketLength = DataIn[31:20];
	 
	 
	 
	 always @ (posedge Clock)
	 begin
		InputData_reg <= DataIn;
		DataOut <= InputData_reg;
		DataValid <= DataValid_reg;
		LinkEndOfPacket_reg <= (PacketLengthCounter == 1);
		LinkStartOfPacket_reg <= StartOfPacket;
		LinkEndOfPacket <= LinkEndOfPacket_reg;
		LinkStartOfPacket <= LinkStartOfPacket_reg;
	 end
	 
	 // detect the begining of a packet and store address and length
	 always @ (posedge Clock)
	 if (StartOfPacket)
	 begin
		PacketAddress <= DataIn[19:8];	// address from packet sent to user
		PacketLength <= RIOpacketLength;	// Packet length in words
	 end
	 
	 // packet length counter is used to enable the input data going to
	 // the packet FIFO "myPacketBuffFIFO" for the length of the current packet 
	 always @ (posedge Clock)
	 begin
		if (Reset==1'b1) 
			PacketLengthCounter <= 9'b0;
		else if (StartOfPacket) 
			PacketLengthCounter <= (RIOpacketLength - 1); //remove one for header
		else if (PacketLengthCounter > 9'b0) 
			PacketLengthCounter <= PacketLengthCounter - 1;
		else 
			PacketLengthCounter <= PacketLengthCounter;
	 end
	 
	 
	 always @ (posedge Clock)
	 begin
		if (Reset) DataValid_reg <= 1'b0;
		
		else if (PacketLengthCounter >= 9'd2)
			DataValid_reg <= 1'b1;			
		else DataValid_reg <= 1'b0;
	 end

	 // Counter for generating an address for each word in the packet
	 always @ (posedge Clock)
	 begin
		//if (Reset) AddressCounter <= 8'd0;		
		//else if (StartOfPacket) AddressCounter <= DataIn[23:8];		
		if (Reset | StartOfPacket) AddressCounter <= 16'd0;
		//else if (StartOfPacket) AddressCounter <= DataIn[19:8];	
		else if (DataValid_reg) AddressCounter <= (AddressCounter + 1);		
		else AddressCounter <= AddressCounter;
	 end
	 
  	 always @ (posedge Clock) MemoryAddress <= AddressCounter;
	 
	 
	 (* mark_debug  = DEBUG *)reg tvalid;
	 always @ (posedge Clock)
	 begin
	   if(LinkStartOfPacket_reg) begin
	       tvalid <= 1'b1;
	   end    
	   else if(LinkEndOfPacket) tvalid <= 1'b0;
	 end
	 
	 (* mark_debug  = DEBUG *)reg [9:0] laddr;
	 always @ (posedge Clock)
	 begin
	   if(StartOfPacket) laddr  <= 0;
	   else if(tvalid) begin
	       laddr  <= laddr+1;
	   end 
	   else   laddr  <= laddr;
	 end     
 
	 
	 assign sdi_tvalid = tvalid;
	 assign sdi_taddr  = laddr;
	 assign sdi_tlast  = LinkEndOfPacket;
	 
endmodule
