module ALU(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan);
    input [31:0] data_operandA, data_operandB;
    input [4:0] ctrl_ALUopcode, ctrl_shiftamt;
    output [31:0] data_result;
    output isNotEqual, isLessThan;
    
    wire opCode2n, opCode1n;
    not(opCode2n, ctrl_ALUopcode[2]);
    not(opCode1n, ctrl_ALUopcode[1]);
   
    // Adder/Sub
    wire carryIn; 
    wire [31:0] adder_result, data_operandBn, result_Bin;
   
    assign carryIn = ctrl_ALUopcode[0];
    genvar n;
    generate
		for (n = 0; n < 32; n = n+1) begin: data_Bn
			not(data_operandBn[n], data_operandB[n]);
		end
	endgenerate
	
    genvar m;
    generate
		for (m = 0; m < 32; m = m+1) begin: mux_B
			Mux21(.out(result_Bin[m]), .in1(data_operandBn[m]), .in2(data_operandB[m]), .select(carryIn));
		end
	endgenerate
	
   // *** Stage 1***
   // bits 7:0
   wire Mux50;
   Lookahead8Bit CLA25(.Sum(adder_result[7:0]), .Cout(Mux50), .A(data_operandA[7:0]), .B(result_Bin[7:0]), .Cin(carryIn));
   // bits 15:8
   wire [7:0]result0, result1;
   wire carry0, carry1, Mux75;
   Lookahead8Bit CLA50_0(.Sum(result0), .Cout(carry0), .A(data_operandA[15:8]), .B(result_Bin[15:8]), .Cin(0));
   Lookahead8Bit CLA50_1(.Sum(result1), .Cout(carry1), .A(data_operandA[15:8]), .B(result_Bin[15:8]), .Cin(1));
   
   CarrySelectMux Stage1(.Sum(adder_result[15:8]), .CarryOut(Mux75), .A(result1), .B(result0), .CarryA(carry1),
						 .CarryB(carry0), .Select(Mux50));
						 
	//*** Stage 2***
	wire [15:0] LastMux_0, LastMux_1;
	//bits 23:16
	wire Mux100_0, Mux100_1;
	Lookahead8Bit CLA75_0(.Sum(LastMux_0[7:0]), .Cout(Mux100_0), .A(data_operandA[23:16]), 
						.B(result_Bin[23:16]), .Cin(0));
	Lookahead8Bit CLA75_1(.Sum(LastMux_1[7:0]), .Cout(Mux100_1), .A(data_operandA[23:16]), 
						.B(result_Bin[23:16]), .Cin(1));
						
	//bits 31:24
	wire [7:0] DoubleMux_0, DoubleMux_1;
	wire Cout0, Cout1;
	Lookahead8Bit CLA100_0(.Sum(DoubleMux_0), .Cout(Cout0), .A(data_operandA[31:24]), 
						.B(result_Bin[31:24]), .Cin(0));
	Lookahead8Bit CLA100_1(.Sum(DoubleMux_1), .Cout(Cout1), .A(data_operandA[31:24]), 
						.B(result_Bin[31:24]), .Cin(1));	
	
	wire CarryOutt0, CarryOutt1;			
	CarrySelectMux Stage2_0(.Sum(LastMux_0[15:8]), .CarryOut(CarryOutt0), .A(DoubleMux_1), .B(DoubleMux_0), .CarryA(Cout1),
						 .CarryB(Cout0), .Select(Mux100_0));
	CarrySelectMux Stage2_1(.Sum(LastMux_1[15:8]), .CarryOut(CarryOutt1), .A(DoubleMux_1), .B(DoubleMux_0), .CarryA(Cout1),
						 .CarryB(Cout0), .Select(Mux100_1));
		
	wire Cout;
	CarrySelectMux16 FinalMux16(.Sum(adder_result[31:16]), .CarryOut(Cout), .A(LastMux_1), .B(LastMux_0), .CarryA(CarryOutt1), 
					 .CarryB(CarryOutt0), .Select(Mux75));
	
	// ********** wire adder_result has result and Cout has overflow flag **********
	wire [31:0] adderALUmux_result; //******************** <- this 
	wire adderALUlogic;
	and(adderALUlogic, opCode2n, opCode1n);
	
	genvar a;
	generate
		for(a = 0; a < 32; a =a+1) begin: adderMux
			and(adderALUmux_result[a], adder_result[a], adderALUlogic);
		end
	endgenerate
		
	wire [31:0] andOutput, orOutput, ALUout;
	genvar o;
	generate
		for (o = 0; o <32; o = o+1) begin: ALUandOr
			and(andOutput[o], data_operandA[o], data_operandB[o]);
			or(orOutput[o], data_operandA[o], data_operandB[o]);
		end
	endgenerate
	
	genvar w;
    generate
		for (w = 0; w < 32; w = w+1) begin: ALUandOrMux
			Mux21(.out(ALUout[w]), .in1(orOutput[w]), .in2(andOutput[w]), .select(carryIn));
		end
	endgenerate
	
	wire [31:0] andorALUmux_result; //******************** <- this 
	wire andorALUlogic;
	and(andorALUlogic, opCode2n, ctrl_ALUopcode[1]);
	
	genvar x;
	generate
		for(x = 0; x < 32; x =x+1) begin: andorMux
			and(andorALUmux_result[x], ALUout[x], andorALUlogic);
		end
	endgenerate
	
	wire [31:0] leftShift_result, rightShift_result, shift_result;
	leftBarrelShift LeftShift(.out(leftShift_result), .in(data_operandA), .select(ctrl_shiftamt));
	rightBarrelShift RightShift(.out(rightShift_result), .in(data_operandA), .select(ctrl_shiftamt));
	
	genvar s;
    generate
		for (s = 0; s < 32; s = s+1) begin: shiftMux
			Mux21(.out(shift_result[s]), .in1(rightShift_result[s]), .in2(leftShift_result[s]), .select(carryIn));
		end
	endgenerate
	
	wire [31:0] shiftALU_result; //******************** <- this 
	wire shiftALUlogic;
	and(shiftALUlogic, opCode1n, ctrl_ALUopcode[2]);
	
	genvar t;
	generate
		for(t = 0; t < 32; t =t+1) begin: shiftOutMux
			and(shiftALU_result[t], shift_result[t], shiftALUlogic);
		end
	endgenerate
	
	wire[31:0] ALUMuxout_result;
	genvar p;
	generate
		for(p = 0; p < 32; p =p+1) begin: ALUoutMux
			or(ALUMuxout_result[p], adderALUmux_result[p], andorALUmux_result[p], shiftALU_result[p]);
		end
	endgenerate
	
	wire opcodeCheck;
	nor(opcodeCheck, ctrl_ALUopcode[4], ctrl_ALUopcode[3]);
	
	genvar l;
	generate
		for (l = 0; l < 32; l = l+1) begin: OpcodeCheck
			and(data_result[l], opcodeCheck, ALUMuxout_result[l]);
		end
	endgenerate
	
	//Comparator
	wire sub_Opcode, notEqual_result;
	and(sub_Opcode, opCode2n, opCode1n, carryIn);
	assign notEqual_result = data_result[31]|data_result[30]|data_result[29]|data_result[28]|data_result[27]|data_result[26]|
							 data_result[25]|data_result[24]|data_result[23]|data_result[22]|data_result[21]|data_result[20]|
							 data_result[19]|data_result[18]|data_result[17]|data_result[16]|data_result[15]|data_result[14]|
							 data_result[13]|data_result[12]|data_result[11]|data_result[10]|data_result[9]|data_result[8]|
							 data_result[7]|data_result[6]|data_result[5]|data_result[4]|data_result[3]|data_result[2]|
							 data_result[1]|data_result[0];
	assign isNotEqual = sub_Opcode & notEqual_result;
	assign isLessThan = sub_Opcode & data_result[31];
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

module CarrySelectMux(Sum, CarryOut, A, B, CarryA, CarryB, Select); // B is 0,  A is 1
	input [7:0] A, B;
	input CarryA, CarryB, Select;
	output [7:0] Sum;
	output CarryOut;
	
	genvar c;
	generate
		for(c = 0; c < 8; c = c + 1) begin: Mux8bit 
			Mux21(.out(Sum[c]), .in1(A[c]), .in2(B[c]), .select(Select));
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
		for(c = 0; c < 16; c = c + 1) begin:
			Mux8bit Mux21(.out(Sum[c]), .in1(A[c]), .in2(B[c]), .select(Select));
		end
	endgenerate
	
	Mux21 CarryOutMux(.out(CarryOut), .in1(CarryA), .in2(CarryB), .select(Select));
endmodule

module leftShift_1Bit(out, in);
	input [31:0] in;
	output [31:0] out;
	
	genvar i;
	generate
		for (i = 1; i < 32; i = i+1) begin:
			ShiftLeft1 assign out[i] = in[i-1];
		end
	endgenerate
	
	assign out[0] = 0;
endmodule

module leftShift_2Bit(out, in);
	input [31:0] in;
	output [31:0] out;
	
	genvar i;
	generate
		for (i = 2; i < 32; i = i+1) begin:
			ShiftLeft2 assign out[i] = in[i-2];
		end
	endgenerate
	
	assign out[1:0] = 0;
endmodule

module leftShift_4Bit(out, in);
	input [31:0] in;
	output [31:0] out;
	
	genvar i;
	generate
		for (i = 4; i < 32; i = i+1) begin:
			ShiftLeft4 assign out[i] = in[i-4];
		end
	endgenerate
	
	assign out[3:0] = 0;
endmodule

module leftShift_8Bit(out, in);
	input [31:0] in;
	output [31:0] out;
	
	genvar i;
	generate
		for (i = 8; i < 32; i = i+1) begin:
			ShiftLeft8 assign out[i] = in[i-8];
		end
	endgenerate
	
	assign out[7:0] = 0;
endmodule

module leftShift_16Bit(out, in);
	input [31:0] in;
	output [31:0] out;
	
	genvar i;
	generate
		for (i = 16; i < 32; i = i+1) begin:
			ShiftLeft16 assign out[i] = in[i-16];
		end
	endgenerate
	
	assign out[15:0] = 0;
endmodule

module leftBarrelShift(out, in, select);
	input [31:0] in;
	input [4:0] select;
	output [31:0] out;
	
	wire [31:0] out16, out8, out4, out2, out1, mux16, mux8, mux4, mux2;
		
	leftShift_16Bit Stage1(.out(out16), .in(in));
	barrelShiftMux Mux16(.out(mux16), .inA(out16), .inB(in), .select(select[4]));  //inA is 1, inB is 0
	leftShift_8Bit Stage2(.out(out8), .in(mux16));
	barrelShiftMux Mux8(.out(mux8), .inA(out8), .inB(mux16), .select(select[3]));
	leftShift_4Bit Stage3(.out(out4), .in(mux8));
	barrelShiftMux Mux4(.out(mux4), .inA(out4), .inB(mux8), .select(select[2]));
	leftShift_2Bit Stage4(.out(out2), .in(mux4));
	barrelShiftMux Mux2(.out(mux2), .inA(out2), .inB(mux4), .select(select[1]));
	leftShift_1Bit Stage5(.out(out1), .in(mux2));
	barrelShiftMux Mux1(.out(out), .inA(out1), .inB(mux2), .select(select[0]));
endmodule

module rightShift_1Bit(out, in);
	input [31:0] in;
	output [31:0] out;

	genvar i;
	generate
		for (i = 0; i < 31; i = i+1) begin:
			ShiftRight1 assign out[i] = in[i+1];
		end
	endgenerate

	assign out[31] = in[31];
endmodule

module rightShift_2Bit(out, in);
	input [31:0] in;
	output [31:0] out;

	genvar i;
	generate
		for (i = 0; i < 30; i = i+1) begin:
			ShiftRight2 assign out[i] = in[i+2];
		end
	endgenerate

	assign out[31] = in[31];
	assign out[30] = in[31];
endmodule

module rightShift_4Bit(out, in);
	input [31:0] in;
	output [31:0] out;

	genvar i;
	generate
		for (i = 0; i < 28; i = i+1) begin:
			ShiftRight4 assign out[i] = in[i+4];
		end
	endgenerate

	genvar j;
	generate
		for (j = 31; j >= 28; j = j-1) begin: Arith4
			assign out[j] = in[31];
		end
	endgenerate
endmodule

module rightShift_8Bit(out, in);
	input [31:0] in;
	output [31:0] out;

	genvar i;
	generate
		for (i = 0; i < 24; i = i+1) begin:
			ShiftRight8 assign out[i] = in[i+8];
		end
	endgenerate

	genvar j;
	generate
		for (j = 31; j >= 24; j = j-1) begin: Arith8
			assign out[j] = in[31];
		end
	endgenerate
endmodule

module rightShift_16Bit(out, in);
	input [31:0] in;
	output [31:0] out;

	genvar i;
	generate
		for (i = 0; i < 16; i = i+1) begin:
			Shift2 assign out[i] = in[i+16];
		end
	endgenerate

	genvar j;
	generate
		for (j = 31; j >= 16; j = j-1) begin: Arith16
			assign out[j] = in[31];
		end
	endgenerate
endmodule

module barrelShiftMux(out, inA, inB, select);  //inA is 1, inB is 0
	input[31:0] inA, inB;
	input select;
	output [31:0] out;
	
	wire selectn;
	not (selectn, select);
	
	wire [31:0] selectA, selectB;
	genvar n;
	generate
		for (n = 0; n < 32; n = n+1) begin: selection
			and(selectA[n], inA[n], select);
			and(selectB[n], inB[n], selectn);
			or(out[n], selectA[n], selectB[n]);
		end
	endgenerate
endmodule

module rightBarrelShift(out, in, select);
	input [31:0] in;
	input [4:0] select;
	output [31:0] out;
	
	wire [31:0] out16, out8, out4, out2, out1, mux16, mux8, mux4, mux2;
		
	rightShift_16Bit Stage1(.out(out16), .in(in));
	barrelShiftMux Mux16(.out(mux16), .inA(out16), .inB(in), .select(select[4]));  //inA is 1, inB is 0
	rightShift_8Bit Stage2(.out(out8), .in(mux16));
	barrelShiftMux Mux8(.out(mux8), .inA(out8), .inB(mux16), .select(select[3]));
	rightShift_4Bit Stage3(.out(out4), .in(mux8));
	barrelShiftMux Mux4(.out(mux4), .inA(out4), .inB(mux8), .select(select[2]));
	rightShift_2Bit Stage4(.out(out2), .in(mux4));
	barrelShiftMux Mux2(.out(mux2), .inA(out2), .inB(mux4), .select(select[1]));
	rightShift_1Bit Stage5(.out(out1), .in(mux2));
	barrelShiftMux Mux1(.out(out), .inA(out1), .inB(mux2), .select(select[0]));
endmodule
