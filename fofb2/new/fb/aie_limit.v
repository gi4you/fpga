// -------------------------------------------------------------
//
//	7FFFFFFF    +2,147,483,647
//  80000000    -2,147,483,648
//	FFFFFFFF	-1
//
//	00FFFFFF 	+16,777,215
//	FF000001	-16,777,215
//
//
// -------------------------------------------------------------


`timescale 1 ns / 1 ns

module aie_limit
          (
           LIMIT_i,
           pos_i,
           valid,
           pos_o,
           valid_n
          );

  input   signed [31:0] LIMIT_i;
  input   signed [31:0] pos_i;  // int32
  output  valid;
  output  signed [31:0] pos_o;  // int32
  output  valid_n;


  wire signed [31:0] const1_out1;  // int32
  wire Relational_Operator1_out1;
  wire signed [31:0] const2_out1;  // int32
  wire Relational_Operator3_out1;
  wire Logical_Operator2_out1;
  wire signed [31:0] um_const_out1;  // int32
  wire signed [31:0] Enable1_out1;  // int32
  wire Logical_Operator1_out1;

  assign const1_out1 = LIMIT_i; //32'h01312D00;	//20.000000 mm

  assign Relational_Operator1_out1 = (pos_i <= const1_out1) ? 1'b1 : 1'b0;

  assign const2_out1 = LIMIT_i * -1; //32'hFECED300;	//-20.000000 mm

  assign Relational_Operator3_out1 = (pos_i >= const2_out1) ? 1'b1 : 1'b0;	// > -20 mm

  //  Out = Pos < 20  &&  Pos > -20 
  assign Logical_Operator2_out1 =  Relational_Operator1_out1 & Relational_Operator3_out1;

  assign valid = Logical_Operator2_out1;
  assign um_const_out1 = 32'h00000000;	//zero position

  assign Enable1_out1 = (Logical_Operator2_out1 == 1'b1) ? pos_i : um_const_out1;
  assign pos_o = Enable1_out1;
  assign Logical_Operator1_out1 = !Logical_Operator2_out1;

  assign valid_n = Logical_Operator1_out1;

endmodule  // aie_limit

