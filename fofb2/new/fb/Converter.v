
`timescale 1 ns / 1 ns

module Converter
          (
           two_comp,
           ps_out
          );


  input   signed [19:0] two_comp;  // sfix20
  output  [19:0] ps_out;  // ufix20


  wire [19:0] ToFixPt_out1;  // ufix20
  wire signed [19:0] Const2_19_out1;  // sfix20
  wire Relational_Operator3_out1;
  wire [19:0] Sub_out1;  // ufix20
  wire [19:0] zero2_out1;  // ufix20
  wire signed [19:0] ToFixPt_tmp;  // sfix20
  wire signed [20:0] Sub_out1_tmp;  // sfix21
  wire signed [20:0] sub_cast;  // sfix21
  wire signed [20:0] sub_cast_1;  // sfix21
  wire signed [21:0] sub_temp;  // sfix22

  assign ToFixPt_tmp = two_comp;

  assign ToFixPt_out1 = (ToFixPt_tmp[19] == 1'b1 ) ? 20'b00000000000000000000 : $unsigned(ToFixPt_tmp);

  assign Const2_19_out1 = 20'b01111111111111111111;

  assign Relational_Operator3_out1 = (two_comp >= Const2_19_out1) ? 1'b1 : 1'b0;

  assign sub_cast = $signed({{1{two_comp[19]}}, two_comp});
  assign sub_cast_1 = $signed({{1{Const2_19_out1[19]}}, Const2_19_out1});
  assign sub_temp = sub_cast - sub_cast_1;
  assign Sub_out1_tmp = sub_temp[20:0];

  assign Sub_out1 = $unsigned(Sub_out1_tmp[19:0]);

  assign zero2_out1 = (Relational_Operator3_out1 == 1'b1) ? ToFixPt_out1 :
                Sub_out1;
  assign ps_out = zero2_out1;

endmodule  // Converter

