module ALU(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan);
   input [31:0] data_operandA, data_operandB;
   input [4:0] ctrl_ALUopcode, ctrl_shiftamt;
   output [31:0] data_result;
   output isNotEqual, isLessThan;
 
   // Adder
   // *** Stage 1***
   // bits 7:0
   wire Mux50;
   Lookahead8Bit CLA25(.Sum(data_result[7:0]), .Cout(Mux50), .A(data_operandA[7:0]), .B(data_operandB[7:0]), .Cin(0));
   // bits 15:8
   wire [7:0]result0, result1;
   wire carry0, carry1, Mux75;
   Lookahead8Bit CLA50_0(.Sum(result0), .Cout(carry0), .A(data_operandA[15:8]), .B(data_operandB[15:8]), .Cin(0));
   Lookahead8Bit CLA50_1(.Sum(result1), .Cout(carry1), .A(data_operandA[15:8]), .B(data_operandB[15:8]), .Cin(1));
   
   CarrySelectMux Stage1(.Sum(data_result[15:8]), .CarryOut(Mux75), .A(result1), .B(result0), .CarryA(carry1),
						 .CarryB(carry0), .Select(Mux50));
						 
	//*** Stage 2***
	wire [15:0] LastMux_0, LastMux_1;
	//bits 23:16
	wire Mux100_0, Mux100_1;
	Lookahead8Bit CLA75_0(.Sum(LastMux_0[7:0]), .Cout(Mux100_0), .A(data_operandA[23:16]), 
						.B(data_operandB[23:16]), .Cin(0));
	Lookahead8Bit CLA75_1(.Sum(LastMux_1[7:0]), .Cout(Mux100_1), .A(data_operandA[23:16]), 
						.B(data_operandB[23:16]), .Cin(1));
						
	//bits 31:24
	wire [7:0] DoubleMux_0, DoubleMux_1;
	wire Cout0, Cout1;
	Lookahead8Bit CLA100_0(.Sum(DoubleMux_0), .Cout(Cout0), .A(data_operandA[31:24]), 
						.B(data_operandB[31:24]), .Cin(0));
	Lookahead8Bit CLA100_1(.Sum(DoubleMux_1), .Cout(Cout1), .A(data_operandA[31:24]), 
						.B(data_operandB[31:24]), .Cin(1));	
	
	wire CarryOutt0, CarryOutt1;			
	CarrySelectMux Stage2_0(.Sum(LastMux_0[15:8]), .CarryOut(CarryOutt0), .A(DoubleMux_1), .B(DoubleMux_0), .CarryA(Cout1),
						 .CarryB(Cout0), .Select(Mux100_0));
	CarrySelectMux Stage2_1(.Sum(LastMux_1[15:8]), .CarryOut(CarryOutt1), .A(DoubleMux_1), .B(DoubleMux_0), .CarryA(Cout1),
						 .CarryB(Cout0), .Select(Mux100_1));
		
	wire Cout;
	CarrySelectMux16 FinalMux16(.Sum(data_result[31:16]), .CarryOut(Cout), .A(LastMux_1), .B(LastMux_0), .CarryA(CarryOutt1), 
					 .CarryB(CarryOutt0), .Select(Mux75));
	
	wire [7:0] stage1notEqual, stage1lessThan;
	LookAheadComparator_4Bit MSBCompare(.notEqual(stage1notEqual[7]), .lessThan(stage1lessThan[7]), .A(data_operandA[31:27]), 
							 .B(data_operandB[31:27]), .MSB_flag(1));
	genvar c;
	generate
		for (c = 0; c < 7; c = c +1) begin:
			Stage1Compare LookAheadComparator_4Bit(.notEqual(stage1notEqual[c]), .lessThan(stage1lessThan[c]), 
									 .A(data_operandA[(c+1)*4-1:c*4]), .B(data_operandB[(c+1)*4-1:c*4]), .MSB_flag(0));
		end
	endgenerate
	
	wire [1:0]stage2notEqual, stage2lessThan;
	LookAheadComparator_16Bit Stage2MSB(.notEqual(stage2notEqual[1]), .lessThan(stage2lessThan[1]), .A(stage1notEqual[7:4]), .B(stage1lessThan[7:4]));
	LookAheadComparator_16Bit Stage2LSB(.notEqual(stage2notEqual[0]), .lessThan(stage2lessThan[0]), .A(stage1notEqual[3:0]), .B(stage1lessThan[3:0]));
	
	FinalComparator Final(.notEqual(isNotEqual), .lessThan(isLessThan), .A(stage2notEqual), .B(stage2lessThan));
	
	wire [31:0] andOutput, orOutput;
	genvar a;
	generate
		for (a = 0; a <32; a = a+1) begin:
			ALUand and(andOutput[a], data_operandA[a], data_operandB[a]);
		end
	endgenerate
	
	genvar o;
	generate
		for (o = 0; o < 32; o = o+1) begin:
			ALUor or(orOutput[o], data_operandA[o], data_operandB[o]);
		end
	endgenerate
	
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
		for(c = 0; c < 8; c = c + 1) begin:
			Mux8bit Mux21(.out(Sum[c]), .in1(A[c]), .in2(B[c]), .select(Select));
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

module LookAheadComparator_4Bit(notEqual, lessThan, A, B, MSB_flag);
	input[3:0] A, B;
	input MSB_flag;
	output notEqual, lessThan;
	wire Compare3, Compare2, Compare1, Compare0, Not3, Not2, Not1, Res2, Res1, Res0;
	
	//bit 3
	xor(Compare3, A[3], B[3]);
	not(Not3, Compare3);
	
	//bit 2
	xor(Res2, A[2], B[2]);
	not(Not2, Res2);
	and(Compare2, Not3, Res2);
	
	//bit 1
	xor(Res1, A[1], B[1]);
	not(Not1, Res1);
	and(Compare1, Not3, Not2, Res1);
	
	//bit 0
	xor(Res0, A[0], B[0]);
	and(Compare0, Not3, Not2, Not1, Res0);
	
	or(notEqual, Compare3, Compare2, Compare1, Compare0);
	
	// A < B
	wire Notflag, NotA3, NotB3, NotA2, NotA1, NotA0, Comp31, Comp30, Comp3, Comp2, Comp1, Comp0, Out3, Out2, Out1, Out0, Unsigned_Res, FirstBitCheck;
	
	not(NotA3, A[3]);
	not(NotA2, A[2]);
	not(NotA1, A[1]);
	not(NotA0, A[0]);
	not(NotB3, B[3]);
	not(Notflag, MSB_flag);
	
	and(Comp2, NotA2, B[2]);
	and(Comp1, NotA1, B[1]);
	and(Comp0, NotA0, B[0]);

	and(Out2, Comp2, Compare2);
	and(Out1, Comp1, Compare1);
	and(Out0, Comp0, Compare0);
	
	and(Comp30, Notflag, NotA3, B[3], Compare3);
	and(Comp31, MSB_flag, A[3], NotB3, Compare3);
	or(Out3, Comp31, Comp30);
	or(lessThan, Out3, Out2, Out1, Out0);

endmodule

module LookAheadComparator_16Bit(notEqual, lessThan, A, B);
	input [3:0] A, B;
	output notEqual, lessThan;
	
	wire Compare3, Compare2, Compare1, Compare0, Not3, Not2, Not1, Res2, Res1, Res0;
	
	//bit 3
	or(Compare3, A[3], B[3]);
	not(Not3, Compare3);
	
	//bit 2
	or(Res2, A[2], B[2]);
	not(Not2, Res2);
	and(Compare2, Not3, Res2);
	
	//bit 1
	or(Res1, A[1], B[1]);
	not(Not1, Res1);
	and(Compare1, Not3, Not2, Res1);
	
	//bit 0
	or(Res0, A[0], B[0]);
	and(Compare0, Not3, Not2, Not1, Res0);
	
	or(notEqual, Compare3, Compare2, Compare1, Compare0);
	
	wire Comp3, Comp2, Comp1, Comp0;
	and(Comp3, Compare3, B[3]);
	and(Comp2, Compare2, B[2]);
	and(Comp1, Compare1, B[1]);
	and(Comp0, Compare0, B[0]);
	
	or(lessThan, Comp3, Comp2, Comp1, Comp0);
endmodule

module FinalComparator(notEqual, lessThan, A, B);
	input[1:0] A, B;
	output notEqual, lessThan;
	wire Compare1, Compare0, not1, Res1;
	
	//bit 1
	or(Compare1, A[1], B[1]);
	not(not1, Compare1);
	
	//bit 0
	or(Res1, A[0], B[0]);
	and(Compare0, not1, Res1);
	
	or(notEqual, Compare1, Compare0);
	
	wire Out1, Out0;
	and(Out1, Compare1, B[1]);
	and(Out0, Compare0, B[0]);
	
	or(lessThan, Out1, Out0);
endmodule
