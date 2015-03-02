module MultDiv(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock, data_result, data_exception, data_inputRDY, data_resultRDY);
   input [31:0] data_operandA;
   input [31:0] data_operandB;
   input ctrl_MULT, ctrl_DIV, clock;             
   output [31:0] data_result; 
   output data_exception;
   output [31:0] data_inputRDY, data_resultRDY;
   
	
   wire [32:0] Countout;
   wire productEnable;
   wire ctrl;
   or(ctrl, ctrl_MULT, ctrl_DIV);
   counter_array Counter(.out(Countout), .in(ctrl), .clk(clock), .reset(0));
   
   assign productEnable = Countout[32] | Countout[31] | Countout[30] | Countout[29] | Countout[28] | Countout[27] | Countout[26] | Countout[25] | Countout[24] |
						  Countout[23] | Countout[22] | Countout[21] | Countout[20] | Countout[19] | Countout[18] | Countout[17] | Countout[16] | Countout[15] |
						  Countout[14] | Countout[13] | Countout[12] | Countout[10] | Countout[9] | Countout[8] | Countout[7] | Countout[6] | Countout[5] |
						  Countout[4] | Countout[3] | Countout[2] | Countout[1] | Countout[0] | Countout[11]; 
	assign data_resultRDY = Countout[0];
	assign data_inputRDY = ~productEnable | Countout[0];

	wire [31:0] adder_result, loop, adjMultiplicand;
	wire [1:0] check;
	wire checkState, checkZero;
	product_array RM(.out(loop), .in(adder_result[31:1]), .clk(clock), .reset(ctrl_MULT), .enable(productEnable));
	multiplier_array PM(.sumOut(data_result), .ctrlOut(check), .in(data_operandB), .lastSum(adder_result[0]), .clk(clock), .state(ctrl_MULT), .enable(productEnable));
	assign checkState = check[1] & ~check[0];
	
	xor(checkZero, check[1], check[0]);
	wire [31:0] expand;
	genvar i;
	generate
		for(i = 0; i < 32; i=i+1) begin: expnd
			assign expand[i] = checkZero;
		end
	endgenerate
	assign adjMultiplicand = data_operandA & expand;
	adder MULTadd(.out(adder_result), .multiplier(adjMultiplicand), .multiplicand(loop), .state(checkState)); //state = add/sub	

	// *********** divide **********//
	wire [31:0] dividend_A, divisor_B, negate_A, negate_B;
	
	adder NegA(.out(negate_A), .multiplier(data_operandA), .multiplicand(0), .state(1));

	adder NegB(.out(negate_B), .multiplier(data_operandB), .multiplicand(0), .state(1));
	
	genvar y;
	generate
		for(y = 0; y < 32; y=y+1) begin: negSelect
			Mux21 negSelectionA(.out(dividend_A[y]), .in1(negate_A[y]), .in2(data_operandA[y]), .select(data_operandA[31]));
			Mux21 negSelectionB(.out(divisor_B[y]), .in1(negate_B[y]), .in2(data_operandB[y]), .select(data_operandB[31]));
		end
	endgenerate
	
	assign data_exception = ~(divisor_B[31] | divisor_B[30] | divisor_B[29] | divisor_B[28] | divisor_B[27] | divisor_B[26] | divisor_B[25] |
							  divisor_B[24] | divisor_B[23] | divisor_B[22] | divisor_B[21] | divisor_B[20] | divisor_B[19] | divisor_B[18] |
							  divisor_B[17] | divisor_B[16] | divisor_B[15] | divisor_B[14] | divisor_B[13] | divisor_B[12] | divisor_B[11] |
							  divisor_B[10] | divisor_B[9] | divisor_B[8] | divisor_B[7] | divisor_B[6] | divisor_B[5] | divisor_B[4] |
							  divisor_B[3] | divisor_B[2] | divisor_B[1] | divisor_B[0]); 
	
	wire [31:0] divLoop, muxSelect, div_res, sub_result;
	wire comparator;
	wire connect;

	remainder_array Remainder(.sumOut(divLoop),.nextBit(connect), .in(muxSelect), .clk(clock), .reset(ctrl_DIV), .enable(productEnable), .state(comparator));
	quotient_array(.bitOut(connect), .in(dividend_A),.QuotientOut(data_result), .quotientIn(~comparator), .clk(clock), .state(ctrl_DIV), .enable(productEnable));	
	
	adder DIVadd(.out(sub_result), .multiplier(divisor_B), .multiplicand(divLoop), .state(1));
	assign comparator = sub_result[31];
	genvar m;
	generate
		for (m = 0; m <32; m=m+1)begin: selection
			Mux21 selectMUX(.out(muxSelect[m]), .in1(divLoop[m]), .in2(sub_result[m]), .select(comparator));
		end
	endgenerate
endmodule

module multiplier_array(sumOut, ctrlOut, in, lastSum, clk, state, enable); // state will determine to shift or to load new operation
	input [31:0] in; // in is value to load
	input clk, state, lastSum, enable;
	output [31:0] sumOut;
	output [1:0] ctrlOut;
	wire [32:0] mux_in;
	wire [31:0] mux_out;
	
	assign mux_in[32] = lastSum;
	genvar d;
	generate
		for(d = 32; d > 0; d = d-1) begin: ProductGen
			Mux21 productMUX(.out(mux_out[d-1]), .in1(in[d-1]), .in2(mux_in[d]), .select(state));
			dff_sync_clear productDFF(.d(mux_out[d-1]), .sclr(0), .clk(clk), .f(mux_in[d-1]));
			assign sumOut[d-1] = mux_in[d-1] & enable;
		end
	endgenerate
	assign ctrlOut[1] = mux_in[0];
	dff_sync_clear productDFF(.d(mux_in[0]), .sclr(state), .clk(clk), .f(ctrlOut[0]));

endmodule

module quotient_array(bitOut, QuotientOut, in, quotientIn, clk, state, enable); // state will determine to shift or to load new operation
	input [31:0] in; // in is value to load
	input clk, state, quotientIn, enable;
	output  bitOut;
	output [31:0] QuotientOut;
	wire [32:0] mux_in;
	wire [31:0] mux_out;
	
	assign mux_in[0] = quotientIn;
	genvar d;
	generate
		for(d = 0; d < 32; d = d+1) begin: quotientGen
			Mux21 productMUX(.out(mux_out[d]), .in1(in[d]), .in2(mux_in[d]), .select(state));
			dff_sync_clear productDFF(.d(mux_out[d]), .sclr(0), .clk(clk), .f(mux_in[d+1]));
			assign QuotientOut[d] = mux_in[d+1];
		end
	endgenerate
	assign bitOut = mux_in[32]; 

endmodule


module product_array(out, in, clk, reset, enable);
	input [30:0] in; //in is the sum wire IT HAS TO NOT INCLUDE THE LAST BIT
	input reset, enable, clk;
	output [31:0] out;
	wire [31:0]result;
	
	dff_sync_clear FirstDFF(.d(in[30]),. sclr(reset), .clk(clk), .f(result[31]));
	assign out[31] = result[31] & enable;
	genvar c;
	generate
		for (c = 0; c < 31; c = c+1) begin: ProductGen
			dff_sync_clear productDFF(.d(in[c]),. sclr(reset), .clk(clk), .f(result[c]));
			assign out[c] = result[c] & enable;
		end
	endgenerate
endmodule

module remainder_array(sumOut, in, nextBit, clk, state, enable, reset); // state will determine to shift or to load new operation 1 - negative
	input [31:0] in; // load the sub result
	input clk, state, nextBit, enable, reset;
	output [31:0] sumOut; // take the other bit fro quotient
	wire [32:0] mux_in;
	wire [31:0] mux_out;
	
	assign mux_in[0] = nextBit;
	genvar d;
	generate
		for(d = 0; d < 32; d = d+1) begin: RemainderGen
			Mux21 productMUX(.out(mux_out[d]), .in1(mux_in[d]), .in2(in[d]), .select(state));
			dff_sync_clear productDFF(.d(mux_out[d]), .sclr(reset), .clk(clk), .f(mux_in[d+1]));
			assign sumOut[d] = mux_in[d];
		end
	endgenerate
endmodule

module counter_array(out, in, clk, reset);
	input in, clk, reset;
	output [32:0] out;
	wire [33:0] in_dff;
	
	assign in_dff[33] = in;
	
	genvar d;
	generate
		for(d = 32; d >= 0; d = d-1) begin: CounterGen
			dff_sync_clear counterDFF(.d(in_dff[d+1]), .sclr(reset), .clk(clk), .f(in_dff[d]));
			assign out[d] = in_dff[d];
		end
	endgenerate
	
endmodule
	
module dff_sync_clear(d, sclr, clk, f);
	input d, sclr, clk;
	output f;
	reg f;
	always @(posedge clk) begin 
		case(sclr)
			1'b0: f = d;
			1'b1: f = 0;
		endcase
	end
endmodule

module Mux21(out, in1, in2, select);
	input in1, in2, select;
	output out;
	
	wire selectn;
	not(selectn, select);
	
	wire select1, select2;
	and(select1, in1, select);
	and(select2, in2, selectn);
	or(out, select1, select2);
endmodule

module FourBitLookAhead(Sum, Cout, A, B, Cin);
    input [3:0] A,B;
    input Cin;
    output [3:0] Sum;
    output Cout;
    
    wire [3:0] G,P,C;
 
    assign G = A & B; 
    assign P = A ^ B;
	assign C[0] = Cin;
	// Stage 1
	wire prop1;
	and(prop1, P[0],Cin);
	or(C[1], G[0], prop1);
	
    // Stage 2
    wire prop21, prop22;
    and(prop21, P[1], P[0], Cin);
    and(prop22, P[1], G[0]);
    or(C[2], G[1], prop21, prop22);
    
    //Stage 3
    wire prop31, prop32, prop33;
    and(prop31, P[2], P[1], P[0], Cin);
    and(prop32, P[2], P[1], G[0]);
    and(prop33, P[2], G[1]);
    or(C[3], G[2], prop31, prop32, prop33);
    
    //Stage 4
    wire prop41, prop42, prop43, prop44;
    and(prop41, P[3], P[2], P[1], P[0], Cin);
    and(prop42, P[3], P[2], P[1], G[0]);
    and(prop43, P[3], P[2], G[1]);
    and(prop44, P[3], G[2]);
    or (Cout, G[3], prop41, prop42, prop43, prop44);

    assign Sum = (A ^ B) ^ C;
endmodule

module Lookahead8Bit(Sum, Cout, A, B, Cin);
	input [7:0] A,B;
	input Cin;
	output[7:0] Sum;
	output Cout;
	
	//First a 4 bit look ahead
	wire carryOut1;
	FourBitLookAhead CLA1(.Sum(Sum[3:0]), .Cout(carryOut1), .A(A[3:0]), .B(B[3:0]), .Cin(Cin));
	FourBitLookAhead CLA2(.Sum(Sum[7:4]), .Cout(Cout), .A(A[7:4]), .B(B[7:4]), .Cin(carryOut1)); 
endmodule

module CarrySelectMux(Sum, CarryOut, A, B, CarryA, CarryB, Select); // B is 0,  A is 1
	input [7:0] A, B;
	input CarryA, CarryB, Select;
	output [7:0] Sum;
	output CarryOut;
	
	genvar c;
	generate
		for(c = 0; c < 8; c = c + 1) begin: Mux8bit 
			Mux21 mux8(.out(Sum[c]), .in1(A[c]), .in2(B[c]), .select(Select));
		end
	endgenerate
	
	Mux21 CarryOutMux(.out(CarryOut), .in1(CarryA), .in2(CarryB), .select(Select));
endmodule

module CarrySelectMux16(Sum, CarryOut, A, B, CarryA, CarryB, Select); // B is 0,  A is 1
	input [15:0] A, B;
	input CarryA, CarryB, Select;
	output [15:0] Sum;
	output CarryOut;
	
	genvar c;
	generate
		for(c = 0; c < 16; c = c + 1) begin: Mux16bit
			Mux21 mux16(.out(Sum[c]), .in1(A[c]), .in2(B[c]), .select(Select));
		end
	endgenerate
	
	Mux21 CarryOutMux(.out(CarryOut), .in1(CarryA), .in2(CarryB), .select(Select));
endmodule

module adder(out, multiplier, multiplicand, state); //state = add/sub
	input [31:0] multiplier, multiplicand;
	input state;
	output [31:0] out;
	
	wire carryIn; 
    wire [31:0] adder_result, data_operandBn, result_Bin;
   
    assign carryIn = state;
    genvar n;
    generate
		for (n = 0; n < 32; n = n+1) begin: data_Bn
			not(data_operandBn[n], multiplier[n]);
		end
	endgenerate
	
    genvar m;
    generate
		for (m = 0; m < 32; m = m+1) begin: mux_B
			Mux21 muxB(.out(result_Bin[m]), .in1(data_operandBn[m]), .in2(multiplier[m]), .select(carryIn));
		end
	endgenerate
	
   // *** Stage 1***
   // bits 7:0
   wire Mux50;
   Lookahead8Bit CLA25(.Sum(adder_result[7:0]), .Cout(Mux50), .A(multiplicand[7:0]), .B(result_Bin[7:0]), .Cin(carryIn));
   // bits 15:8
   wire [7:0]result0, result1;
   wire carry0, carry1, Mux75;
   Lookahead8Bit CLA50_0(.Sum(result0), .Cout(carry0), .A(multiplicand[15:8]), .B(result_Bin[15:8]), .Cin(0));
   Lookahead8Bit CLA50_1(.Sum(result1), .Cout(carry1), .A(multiplicand[15:8]), .B(result_Bin[15:8]), .Cin(1));
   
   CarrySelectMux Stage1(.Sum(adder_result[15:8]), .CarryOut(Mux75), .A(result1), .B(result0), .CarryA(carry1),
						 .CarryB(carry0), .Select(Mux50));
						 
	//*** Stage 2***
	wire [15:0] LastMux_0, LastMux_1;
	//bits 23:16
	wire Mux100_0, Mux100_1;
	Lookahead8Bit CLA75_0(.Sum(LastMux_0[7:0]), .Cout(Mux100_0), .A(multiplicand[23:16]), 
						.B(result_Bin[23:16]), .Cin(0));
	Lookahead8Bit CLA75_1(.Sum(LastMux_1[7:0]), .Cout(Mux100_1), .A(multiplicand[23:16]), 
						.B(result_Bin[23:16]), .Cin(1));
						
	//bits 31:24
	wire [7:0] DoubleMux_0, DoubleMux_1;
	wire Cout0, Cout1;
	Lookahead8Bit CLA100_0(.Sum(DoubleMux_0), .Cout(Cout0), .A(multiplicand[31:24]), 
						.B(result_Bin[31:24]), .Cin(0));
	Lookahead8Bit CLA100_1(.Sum(DoubleMux_1), .Cout(Cout1), .A(multiplicand[31:24]), 
						.B(result_Bin[31:24]), .Cin(1));	
	
	wire CarryOutt0, CarryOutt1;			
	CarrySelectMux Stage2_0(.Sum(LastMux_0[15:8]), .CarryOut(CarryOutt0), .A(DoubleMux_1), .B(DoubleMux_0), .CarryA(Cout1),
						 .CarryB(Cout0), .Select(Mux100_0));
	CarrySelectMux Stage2_1(.Sum(LastMux_1[15:8]), .CarryOut(CarryOutt1), .A(DoubleMux_1), .B(DoubleMux_0), .CarryA(Cout1),
						 .CarryB(Cout0), .Select(Mux100_1));
		
	wire Cout;
	CarrySelectMux16 FinalMux16(.Sum(adder_result[31:16]), .CarryOut(Cout), .A(LastMux_1), .B(LastMux_0), .CarryA(CarryOutt1), 
					 .CarryB(CarryOutt0), .Select(Mux75));
	
	assign out = adder_result;
endmodule
