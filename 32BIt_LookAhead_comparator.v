module main(data_operandA, data_operandB, isLessThan, isNotEqual);
  input [31:0] data_operandA, data_operandB;
  output isNotEqual, isLessThan;
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
