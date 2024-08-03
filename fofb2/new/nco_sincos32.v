// -------------------------------------------------------------
//

// Generated by MATLAB 7.6 and Simulink HDL Coder 1.3
//
//
// -------------------------------------------------------------
// 
//
// -------------------------------------------------------------

`timescale 1 ns / 1 ns

module nco_sincos32
          (
           clk,
           reset,
           clk_enable,
           phase_inc,
           Kx,
           Ky,
           ce_out,
           sin,
           cos
          );


  input   clk;
  input   reset;
  input   clk_enable;
  input   signed [31:0] phase_inc;  // int32
  input   signed [31:0] Kx;  // int32
  input   signed [31:0] Ky;  // int32
  output  ce_out;
  output  signed [31:0] sin;  // int32
  output  signed [31:0] cos;  // int32

  parameter [5:0] C_NCO_ADDR_MAX = 6'b100000;  // ufix6_E25
  parameter signed [31:0] C_NCO_EXTRA_QTR_WAVE_VAL = 32'h7fffffff;  // sfix32_En31
  parameter [6:0] C_NCO_90DEG = 7'b0100000;  // ufix7_E25

  wire enb;
  wire enb_1_1_1;
  reg signed [31:0] NCO_indel_out1;  // int32
  wire signed [31:0] In2_out1;  // int32
  wire signed [31:0] NCO1_out1;  // sfix32_En31
  reg signed [31:0] NCO_outdel_out1;  // sfix32_En31
  wire signed [31:0] Divide1_out1;  // int32
  reg signed [31:0] NCO_outdel1_out1;  // int32
  wire signed [31:0] Divide2_out1;  // int32
  reg signed [31:0] NCO_outdel2_out1;  // int32
  wire signed [31:0] TmpGroundAtScopeInport3_out1;  // int32
  wire signed [15:0] Divide_out1;  // sfix16_En15
  wire signed [31:0] phase_increment;  // int32
  reg signed [31:0] accumulator_reg;  // int32
  wire signed [31:0] accumulator_input;  // int32
  wire signed [31:0] add_signext;  // int32
  wire signed [31:0] add_signext_1;  // int32
  wire signed [32:0] add_temp;  // sfix33
  wire signed [31:0] phase_offset;  // int32
  wire signed [31:0] total_phase;  // int32
  wire signed [31:0] add_signext_2;  // int32
  wire signed [31:0] add_signext_3;  // int32
  wire signed [32:0] add_temp_1;  // sfix33
  wire signed [31:0] dithered_phase;  // int32
  reg [11:0] pn_reg;  // ufix12
  wire pn_out;
  wire pn_xorout;
  wire [11:0] pn_newvalue [0:1];  // ufix12 [2]
  wire [10:0] pn_value_shifted;  // ufix11_E1
  wire [6:0] quantized_phase;  // ufix7_E25
  wire [4:0] lutaddr_quadrant1;  // ufix5_E25
  wire [4:0] lutaddr_quadrant2;  // ufix5_E25
  wire [5:0] sub_signext;  // ufix6_E25
  wire [5:0] sub_signext_1;  // ufix6_E25
  wire [6:0] sub_temp;  // ufix7_E25
  wire [5:0] cos_extra_value_cmp_in;  // ufix6_E25
  wire cos_inv_hwoutput;
  wire cos_addr_mux_sel;
  wire [1:0] quantized_phase_2msbs;  // ufix2_E30
  wire [1:0] cos_control_bits;  // ufix2_E30
  wire [6:0] add_signext_4;  // ufix7_E25
  wire [6:0] add_signext_5;  // ufix7_E25
  wire [7:0] add_temp_2;  // ufix8_E25
  wire [4:0] coslutaddr;  // ufix5_E25
  wire signed [31:0] coslut_output;  // sfix32_En31
  wire signed [31:0] cos_hw;  // sfix32_En31
  reg [30:0] coslut_output_unsigned;  // ufix31_En31
  wire signed [31:0] cos_hw_inv;  // sfix32_En31
  wire signed [32:0] unaryminus_temp;  // sfix33_En31
  wire signed [63:0] mul_temp;  // sfix64_En31
  wire signed [63:0] mul_temp_1;  // sfix64_En31
  wire signed [63:0] mul_temp_2;  // sfix64_En62

  assign enb_1_1_1 = clk_enable;
  assign ce_out = enb_1_1_1;
  always @ (posedge clk)
    begin: NCO_indel_process
      if (reset == 1'b1) begin
        NCO_indel_out1 <= 0;
      end
      else begin
        if (enb == 1'b1) begin
          NCO_indel_out1 <= phase_inc;
        end
      end
    end // NCO_indel_process

  assign In2_out1 = 32'h00000000;


// *** NCO ***

  assign phase_increment = NCO_indel_out1;

// ********************************
// PHASE ACCUMULATION
// ********************************

  assign add_signext = accumulator_reg;
  assign add_signext_1 = phase_increment;
  assign add_temp = add_signext + add_signext_1;
  assign accumulator_input = add_temp[31:0];

  always @ ( posedge clk)
    begin: NCO1_phase_accumulator_temp_process1
      if (reset == 1'b1) begin
        accumulator_reg <= 0;
      end
      else begin
        if (enb == 1'b1) begin
          accumulator_reg <= accumulator_input;
        end
      end
    end // NCO1_phase_accumulator_temp_process1

  assign phase_offset = In2_out1;

// ********************************
// PHASE OFFSET ADDITION
// ********************************

  assign add_signext_2 = accumulator_reg;
  assign add_signext_3 = phase_offset;
  assign add_temp_1 = add_signext_2 + add_signext_3;
  assign total_phase = add_temp_1[31:0];

// ********************************
// DITHER
// ********************************

  assign pn_newvalue[0] = pn_reg;

  assign pn_xorout = pn_newvalue[0][0] ^ pn_newvalue[0][6] ^ pn_newvalue[0][8] ^ pn_newvalue[0][11];

  assign pn_value_shifted = pn_newvalue[0][11:1];

  assign pn_newvalue[1] = {pn_xorout, pn_value_shifted};

  assign pn_out = pn_newvalue[0][0];

  always @ ( posedge clk)
    begin: PN_generation_temp_process2
      if (reset == 1'b1) begin
        pn_reg <= 12'b000000000001;
      end
      else begin
        if (enb == 1'b1) begin
          pn_reg <= pn_newvalue[1];
        end
      end
    end // PN_generation_temp_process2

  assign dithered_phase = (pn_out == 1)? $signed({{1{total_phase[31]}}, total_phase}) + 1 : $signed({{1{total_phase[31]}}, total_phase});

// ********************************
// PHASE QUANTIZATION
// ********************************

  assign quantized_phase = $unsigned(dithered_phase[31:25]);

// ********************************
// QUARTER WAVE LOOKUP TABLE
// The sinusoid is implemented via a quarter wave lookup table.
// The values of the first quadrant of a SINE function are stored in the lookup table.
// The lower 5 quantized phase bits form the address to the lookup table.
// There are 2^5 values in the table on the interval [0,pi/2).
// One extra value, sin(pi/2), is muxed into the output of the table.
// The most significant two quantized phase bits determine how the quarter wave should be
// reflect and/or inverted to create a full sinusoid.
// ********************************



// generation of LUT address and control signals

  assign lutaddr_quadrant1 = quantized_phase[4:0];

  assign sub_signext = C_NCO_ADDR_MAX;
  assign sub_signext_1 = {1'b0, lutaddr_quadrant1};
  assign sub_temp = sub_signext - sub_signext_1;
  assign lutaddr_quadrant2 = sub_temp[4:0];

  assign quantized_phase_2msbs = quantized_phase[6:5];

  assign add_signext_4 = quantized_phase;
  assign add_signext_5 = C_NCO_90DEG;
  assign add_temp_2 = add_signext_4 + add_signext_5;
  assign cos_control_bits = add_temp_2[6:5];

  assign cos_inv_hwoutput = {cos_control_bits[1]};

  assign cos_addr_mux_sel = {cos_control_bits[0]};

  assign cos_extra_value_cmp_in = {cos_control_bits[0], lutaddr_quadrant1};

  assign coslutaddr = (cos_addr_mux_sel == 1'b0) ? lutaddr_quadrant1 :
                lutaddr_quadrant2;


// quarter wave LUT

  always @(coslutaddr)
  begin
    case(coslutaddr)
      5'b00000 : coslut_output_unsigned = 31'b0000000000000000000000000000000;
      5'b00001 : coslut_output_unsigned = 31'b0000110010001111101100101111100;
      5'b00010 : coslut_output_unsigned = 31'b0001100100010111101001101011110;
      5'b00011 : coslut_output_unsigned = 31'b0010010110010000001000001101111;
      5'b00100 : coslut_output_unsigned = 31'b0011000111110001011100000111100;
      5'b00101 : coslut_output_unsigned = 31'b0011111000110011111100101111011;
      5'b00110 : coslut_output_unsigned = 31'b0100101001010000000110001011110;
      5'b00111 : coslut_output_unsigned = 31'b0101011000111110011010011101011;
      5'b01000 : coslut_output_unsigned = 31'b0110000111110111100010101001101;
      5'b01001 : coslut_output_unsigned = 31'b0110110101110100010000000010100;
      5'b01010 : coslut_output_unsigned = 31'b0111100010101101011101001110000;
      5'b01011 : coslut_output_unsigned = 31'b1000001110011100001111001100101;
      5'b01100 : coslut_output_unsigned = 31'b1000111000111001110110011100111;
      5'b01101 : coslut_output_unsigned = 31'b1001100001111111101111111110100;
      5'b01110 : coslut_output_unsigned = 31'b1010001001100111100110010010100;
      5'b01111 : coslut_output_unsigned = 31'b1010101111101011010010011010010;
      5'b10000 : coslut_output_unsigned = 31'b1011010100000100111100110011010;
      5'b10001 : coslut_output_unsigned = 31'b1011110110101110111110010001010;
      5'b10010 : coslut_output_unsigned = 31'b1100010111100100000000110101100;
      5'b10011 : coslut_output_unsigned = 31'b1100110110011111000000100100000;
      5'b10100 : coslut_output_unsigned = 31'b1101010011011011001100010100100;
      5'b10101 : coslut_output_unsigned = 31'b1101101110010100000110100010100;
      5'b10110 : coslut_output_unsigned = 31'b1110000111000101100101111000110;
      5'b10111 : coslut_output_unsigned = 31'b1110011101101011110101111010001;
      5'b11000 : coslut_output_unsigned = 31'b1110110010000011010111100111101;
      5'b11001 : coslut_output_unsigned = 31'b1111000100001001000010000010100;
      5'b11010 : coslut_output_unsigned = 31'b1111010011111010000010101011011;
      5'b11011 : coslut_output_unsigned = 31'b1111100001010011111101111101110;
      5'b11100 : coslut_output_unsigned = 31'b1111101100010100101111101000000;
      5'b11101 : coslut_output_unsigned = 31'b1111110100111010101010111111100;
      5'b11110 : coslut_output_unsigned = 31'b1111111011000100011011010001111;
      5'b11111 : coslut_output_unsigned = 31'b1111111110110001000011110001110;
      default : coslut_output_unsigned = 31'b1111111110110001000011110001110;
    endcase
  end

  assign coslut_output = $signed({1'b0, coslut_output_unsigned});


// mux in sin(pi/2) for efficent ROM usage

  assign cos_hw = (cos_extra_value_cmp_in != 6'b100000) ? coslut_output :
            C_NCO_EXTRA_QTR_WAVE_VAL;


// invert halfwave cos to create a fullwave cos

  assign unaryminus_temp = (cos_hw==32'b10000000000000000000000000000000) ? $signed({1'b0, cos_hw}) : -cos_hw;
  assign cos_hw_inv = unaryminus_temp[31:0];

  assign NCO1_out1 = (cos_inv_hwoutput == 1'b0) ? cos_hw :
               cos_hw_inv;

  always @ (posedge clk)
    begin: NCO_outdel_process
      if (reset == 1'b1) begin
        NCO_outdel_out1 <= 0;
      end
      else begin
        if (enb == 1'b1) begin
          NCO_outdel_out1 <= NCO1_out1;
        end
      end
    end // NCO_outdel_process

  assign mul_temp = NCO_outdel_out1 * Kx;
  assign Divide1_out1 = mul_temp[62:31];

  always @ (posedge clk)
    begin: NCO_outdel1_process
      if (reset == 1'b1) begin
        NCO_outdel1_out1 <= 0;
      end
      else begin
        if (enb == 1'b1) begin
          NCO_outdel1_out1 <= Divide1_out1;
        end
      end
    end // NCO_outdel1_process

  assign sin = NCO_outdel1_out1;
  assign mul_temp_1 = NCO1_out1 * Ky;
  assign Divide2_out1 = mul_temp_1[62:31];

  always @ (posedge clk)
    begin: NCO_outdel2_process
      if (reset == 1'b1) begin
        NCO_outdel2_out1 <= 0;
      end
      else begin
        if (enb == 1'b1) begin
          NCO_outdel2_out1 <= Divide2_out1;
        end
      end
    end // NCO_outdel2_process

  assign cos = NCO_outdel2_out1;
  assign TmpGroundAtScopeInport3_out1 = 32'h00000000;

  assign mul_temp_2 = NCO1_out1 * NCO1_out1;
  assign Divide_out1 = mul_temp_2[62:47];

  assign enb = clk_enable;

endmodule  // nco_sincos32
