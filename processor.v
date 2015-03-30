module processor(clock, reset, debug_data, debug_addr);

	input 	clock, reset;

	// GRADER OUTPUTS - YOU MUST CONNECT TO YOUR DMEM
	output 	[31:0] 	debug_data;
	output	[11:0]	debug_addr;
	// ########################Stage 1#########################
	wire stall, final_stall, decideTake, PC_jump_overflow;
	wire [11:0] PC_to_imem, PC_data;
	wire [31:0] outA, insn, sw_insn, final_insn, branchNotTaken, branchTaken, branchDecide31;
	
	wire JII_check, JExtendCheck, normal_check;
	assign JExtendCheck = ~FD_connect[30]&~FD_connect[29]&((FD_connect[28]&FD_connect[27])|(~FD_connect[28]&FD_connect[27]));
	assign JII_check = FD_connect[29]&~FD_connect[28]&~FD_connect[27];
	assign normal_check = ~JII_check & ~JExtendCheck;
	
	
	mux31 PCMux(.out(PC_data), .in1(outA[11:0]), .in2(branchDecide31[11:0]), .in3(J_to_PC[11:0]), 
				.select1(JII_check), .select2(normal_check), .select3(JExtendCheck));
	PC pc(.data(PC_data), .clreset(reset), .clock(clock), .out(PC_to_imem), .stall_select(stall));
	wire [31:0] ALU_Fetch_in;
	ALU PC_ALU(.data_operandA({{20{1'b0}}, PC_to_imem}), .data_operandB({{31{1'b0}},1'b1}), .ctrl_ALUopcode({5{1'b0}}), 
		.ctrl_shiftamt({5{1'b0}}), .data_result(branchNotTaken), .isNotEqual(), .isLessThan(), .overflow());
		
	wire [31:0]PCbranchFD;
	pipeline_latch FD_PC(.data(branchNotTaken), .clreset(reset), .clock(clock), .out(PCbranchFD));
	
	mux21 branch_decide(.out(branchDecide31), .in1(branchTaken), .in2(branchNotTaken), .select(decideTake));
	sw_instruction_change swTransform(.insn_out(sw_insn), .insn_in(insn));
	
	wire swap_select;
	checkRegSwap fetch_swap(.out(swap_select), .opcode(insn[31:27]));
	mux21 sw_insn_select(.out(final_insn), .in1(sw_insn), .in2(insn), .select(swap_select));
	
	wire test_write;
	check_write_enable writeEnable(.enable(test_write), .opcode(insn[31:27]), .write_reg(insn[26:22]));
	
	wire [32:0] FD_connect, pre_connect, connect, prev_insn, chain_PC; // <------------
	assign connect[32] = test_write;
	assign connect[31:0] = final_insn;
	
	mux21_33 FDstallMux(.out(pre_connect), .in1(FD_connect), .in2(connect), .select(stall));
	IR FD(.data(pre_connect), .clreset(reset), .clock(clock), .out(FD_connect));
	// ##################### Stage 2 ##########################
	wire ALU_overflow, setStatusReg;
	wire[4:0] readA_in, readB_in, writeReg, readA_final;
	wire [31:0] outB, write_val, jal_val;
	wire [32:0] MW_connect, jal_connect;
	checkRegA RegA(.out(readA_in), .opcode(FD_connect[31:27]), .register(FD_connect[21:17]));
	mux21_imm JII_RegA(.out(readA_final), .in1(FD_connect[26:22]), .in2(readA_in), .select(JII_check));          
	checkRegB RegB(.out(readB_in), .opcode(FD_connect[31:27]), .register(FD_connect[16:12]));
	mux21_imm writeEnMux(.out(writeReg), .in1({5{1'b1}}), .in2(MW_connect[26:22]), .select(jal_connect[32]));
	mux21 jal_writeVal(.out(jal_val), .in1(jal_connect[31:0]), .in2(write_val), .select(jal_connect[32]));
	wire jal_reg_enable;
	xor(jal_reg_enable, MW_connect[32], jal_connect[32]); 
	regfile Register(.clock(clock), .ctrl_writeEnable(jal_reg_enable), .ctrl_reset(reset), .ctrl_writeReg(writeReg),
					 .ctrl_readRegA(readA_final), .ctrl_readRegB(readB_in), .data_writeReg(jal_val), .data_readRegA(outA), .data_readRegB(outB));
	
	wire [31:0] final_outA, final_outB, DX_connectA, DX_connectB;
	mux21 RegA_nop(.out(final_outA), .in1({32{1'b0}}), .in2(outA), .select(final_stall));
	mux21 RegB_nop(.out(final_outB), .in1({32{1'b0}}), .in2(outB), .select(final_stall));
	pipeline_latch FD_A(.data(final_outA), .clreset(reset), .clock(clock), .out(DX_connectA));
	pipeline_latch FD_B(.data(final_outB), .clreset(reset), .clock(clock), .out(DX_connectB));
	
	wire[31:0] I_signExtend, JI_signExtend, J_to_PC;
	wire[16:0] I_restored;
	sign_extension_JI JExtend(.out(JI_signExtend), .immediate(FD_connect[26:0]));
	
	sw_immediate I_imm(.imm(I_restored), .insn(FD_connect));
	sign_extension IExtend(.out(I_signExtend), .immediate(I_restored));
	wire Iextend_check;
	assign Iextend_check = blt_check | bne_check;
	
	wire [31:0] IExtendCheck, jumpALU_B;
	
	genvar d;
	generate
		for(d = 0; d < 32; d=d+1) begin: JILoop
			assign J_to_PC[d] = JExtendCheck & JI_signExtend[d];
			assign IExtendCheck[d] = Iextend_check & I_signExtend[d];
		end
	endgenerate
	
	wire BEX_check;
	assign BEX_check = FD_connect[31] & ~FD_connect[27];
	
	mux21 jump_ALUMux(.out(jumpALU_B), .in1(JI_signExtend), .in2(IExtendCheck), .select(BEX_check));
	
	ALU jump_ALU(.data_operandA(PCbranchFD), .data_operandB(jumpALU_B), .ctrl_ALUopcode({5{1'b0}}), 
		.ctrl_shiftamt({5{1'b0}}), .data_result(branchTaken), .isNotEqual(), .isLessThan());
	
	wire [1:0] outBypassA, outBypassB;
	wire [32:0] XM_connect, DX_connect;
	wire blt_check, bne_check;
	assign blt_check = ~FD_connect[31] & ~FD_connect[30] & FD_connect[29] & FD_connect[28] & ~FD_connect[27];
	assign bne_check = ~FD_connect[31] & ~FD_connect[30] & ~FD_connect[29] & FD_connect[28] & ~FD_connect[27];
	
	bypassJump_select bypassRegA(.out(outBypassA), .reg_writeXM(XM_connect[26:22]), .reg_writeWM(MW_connect[26:22]), 
								 .reg_writeDX(DX_connect[26:22]), .writeXM_en(XM_connect[32]), .writeWM_en(MW_connect[32]), 
								 .writeDX_en(DX_connect[32]), .reg_readFD(readA_final));
	bypassJump_select bypassRegB(.out(outBypassB), .reg_writeXM(XM_connect[26:22]), .reg_writeWM(MW_connect[26:22]), 
								 .reg_writeDX(DX_connect[26:22]), .writeXM_en(XM_connect[32]), .writeWM_en(MW_connect[32]),
								 .writeDX_en(DX_connect[32]), .reg_readFD(readB_in));
	wire [31:0] bypassOutA, bypassOutB;
	mux41p bypassJumpMuxA(.out(bypassOutA), .in1(result_out), .in2(XM_ALU_out), .in3(write_val), .in4(outA), .select(outBypassA));
	mux41p bypassJumpMuxB(.out(bypassOutB), .in1(result_out), .in2(XM_ALU_out), .in3(write_val), .in4(outB), .select(outBypassB));
	
	wire [31:0] compareA, compareB, statusOut;
	mux21 compare_A(.out(compareA), .in1({32{1'b0}}), .in2(bypassOutB), .select(BEX_check));
	mux21 compare_B(.out(compareB), .in1(statusOut), .in2(bypassOutA), .select(BEX_check));
	
	wire jumpNotEqual, jumpLessThan, finalNotEqual, finalLessThan;
	ALU if_ALU(.data_operandA(compareA), .data_operandB(compareB), .ctrl_ALUopcode(00001), 
		.ctrl_shiftamt({5{1'b0}}), .data_result(), .isNotEqual(jumpNotEqual), .isLessThan(jumpLessThan));
	/////// ########### this is the STATUS REG OVERFLOW = 1 ###########//////////
	or(PC_jump_overflow, branchTaken[11], branchNotTaken[11]);
	assign setStatusReg = PC_jump_overflow; //ALU_overflow not working
	wire [31:0] write_status;
	mux21 Status_input(.out(write_status), .in1({{31{1'b0}}, 1'b1}), .in2(JI_signExtend), .select(setStatusReg));
	/////////////////////////////////////////////////////////////////////////////
	wire statusEn;
	assign statusEn = setStatusReg | (FD_connect[31] & FD_connect[27]);
	statusReg STATUS(.data(write_status), .clreset(reset), .clock(clock), .out(statusOut), .en(statusEn));
	
	wire lessCheck;
	or(lessCheck, blt_check, BEX_check);
	and(finalNotEqual, jumpNotEqual, bne_check);
	and(finalLessThan, jumpLessThan, lessCheck);
	or(decideTake, finalNotEqual, finalLessThan);
	
	jal_write_chain jal_write(.writeOut(jal_connect), .opcode(FD_connect[31:27]), .link_val(PCbranchFD), .reset(reset), .clock(clock));

	wire [32:0] stallSelect; //<--------
	mux21_33 IR_stall(.out(stallSelect), .in1({33{1'b0}}), .in2(FD_connect), .select(final_stall));
	IR DX(.data(stallSelect), .clreset(reset), .clock(clock), .out(DX_connect));
	
	stall_logic stallUnit(.out(stall), .DX_opcode(DX_connect[31:27]), .FD_opcode(FD_connect[31:27]), 
					  .DX_write(DX_connect[26:22]), .FD_Rone(FD_connect[21:17]), .FD_Rtwo(FD_connect[16:12]));
	or(final_stall, stall, decideTake, JExtendCheck);
	//###################### Stage 3 ##########################
	wire [31:0] ALU_A, imm_mux;
	wire [2:0] mux_selectA, mux_selectB;
	wire [31:0] XM_connectB, XM_ALU_out;
	mux31 bypassA_mux(.out(ALU_A), .in1(XM_ALU_out), .in2(write_val), .in3(DX_connectA), 
					  .select1(mux_selectA[0]), .select2(mux_selectA[1]), .select3(mux_selectA[2])); // 0 - is neighbour, 1 is gap and 2 is normal
	mux31 bypassB_mux(.out(imm_mux), .in1(XM_ALU_out), .in2(write_val), .in3(DX_connectB), 
					  .select1(mux_selectB[0]), .select2(mux_selectB[1]), .select3(mux_selectB[2]));
	
	wire [4:0] regA_in, regB_in;
	checkRegA DX_RegA(.out(regA_in), .opcode(DX_connect[31:27]), .register(DX_connect[26:22]));
	bypassReg_select bypassA(.out(mux_selectA), .reg_writeXM(XM_connect[26:22]), .reg_writeWM(MW_connect[26:22]), 
							  .writeXM_en(XM_connect[32]), .writeWM_en(MW_connect[32]), .reg_readDX(DX_connect[21:17]));
							  
	checkRegB DX_RegB(.out(regB_in), .opcode(DX_connect[31:27]), .register(DX_connect[26:22]));
	bypassReg_select bypassB(.out(mux_selectB), .reg_writeXM(XM_connect[26:22]), .reg_writeWM(MW_connect[26:22]), 
							  .writeXM_en(XM_connect[32]), .writeWM_en(MW_connect[32]), .reg_readDX(DX_connect[16:12]));
							  
	wire [16:0] sw_imm;
	wire [31:0] sign, extended_imm, extended_sw_imm, ALU_B;
	wire sw_select, ALU_B_select;
	and(sw_select,DX_connect[29], DX_connect[28], DX_connect[27]);
	sw_immediate swap_imm(.imm(sw_imm), .insn(DX_connect[31:0]));
	sign_extension imm_extend(.out(extended_imm), .immediate(DX_connect[16:0]));
	sign_extension sw_imm_extend(.out(extended_sw_imm), .immediate(sw_imm));
	mux21 signExtendMux(.out(sign), .in1(extended_sw_imm), .in2(extended_imm), .select(sw_select));
	
	and(ALU_B_select, ~DX_connect[31], ~DX_connect[30], ~DX_connect[29], ~DX_connect[28], ~DX_connect[27]);
	mux21 immMux(.out(ALU_B), .in1(imm_mux), .in2(sign), .select(ALU_B_select));
	
	wire immediate_select;
	assign immediate_select = DX_connect[30] | (~DX_connect[31]&DX_connect[29]&~DX_connect[28]&DX_connect[27])
								| (DX_connect[29] &DX_connect[28]&DX_connect[27]);
								
	wire[4:0] opcode_select;
	mux21_imm opcodeSelect(.out(opcode_select), .in1({5{1'b0}}), .in2(DX_connect[6:2]), .select(immediate_select));

	wire [31:0] result_out, latchB;
	ALU DX_ALU(.data_operandA(ALU_A), .data_operandB(ALU_B), .ctrl_ALUopcode(opcode_select), 
		.ctrl_shiftamt(DX_connect[11:7]), .data_result(result_out), .isNotEqual(), .isLessThan(), .overflow(ALU_overflow));

	pipeline_last XM_ALUout(.data(result_out), .clreset(reset), .clock(clock), .out(XM_ALU_out));
	pipeline_last XM_B(.data(imm_mux), .clreset(reset), .clock(clock), .out(latchB));
	
	IR XM(.data(DX_connect), .clreset(reset), .clock(clock), .out(XM_connect));

	// ########################### Stage 4 ##########################
	//wire[31:0] 	XM_ALU_out; assigned in stage 3
	wire [31:0] dmem_data, dmem_out, MW_connectDmem, MW_connectOut;
	pipeline_last MW_ALUout(.data(XM_ALU_out), .clreset(reset), .clock(clock), .out(MW_connectOut));
	
	wire save_enable;
	assign save_enable = DX_connect[29] & DX_connect[28] & DX_connect[27];
	wire XM_lwsw_select;
	bypass_lwsw XM_select(.out(XM_lwsw_select), .opcode_MW(MW_connect[31:27]), .opcode_XM(DX_connect[31:27]), 
						  .write_MW(MW_connect[26:22]), .read_RegB(DX_connect[16:12]));
						  
	mux21 dmemMux(.out(dmem_data), .in1(write_val), .in2(latchB), .select(XM_lwsw_select));
	
	pipeline_last XM_dmem_out(.data(dmem_out), .clreset(reset), .clock(clock), .out(MW_connectDmem));
	/////////////////////////////////////
	////// THIS IS REQUIRED FOR GRADING
	// CHANGE THIS TO ASSIGN YOUR DMEM WRITE ADDRESS ALSO TO debug_addr
	assign debug_addr = XM_ALU_out[11:0];
	// CHANGE THIS TO ASSIGN YOUR DMEM DATA INPUT (TO BE WRITTEN) ALSO TO debug_data
	assign debug_data = dmem_data;
	
	//wire [32:0] MW_connect; in stage 2
	pipeline_last MW_out(.data(XM_connect), .clreset(reset), .clock(clock), .out(MW_connect));

	// #################### Stage 5 ##################################
	//wire [31:0] write_val; <--- assigned in stage 2
	mux21 writeValMux(.out(write_val), .in1(MW_connectDmem), .in2(MW_connectOut), .select(MW_connect[30]));
	////////////////////////////////////////////////////////////
		
	// You'll need to change where the dmem and imem read and write...
	dmem mydmem(	.address	(debug_addr),
					.clock		(clock),
					.data		(debug_data),
					.wren		(save_enable),
					.q			(dmem_out) // change where output q goes...
	);
	
	imem myimem(	.address 	(PC_to_imem),
					.clken		(1'b1),
					.clock		(clock),
					.q 			(insn) // change where output q goes...
	); 
	
endmodule

module PC(data, clreset, clock, out, stall_select);
	input [11:0] data;
	input clreset, clock, stall_select; 
	output [11:0] out; 
	reg [11:0] out;
	wire clear;

	always @(negedge clock or posedge clreset) begin 
		if(clreset) begin
			out = {12{1'b0}};
		end else begin
			if(~stall_select) begin
				out = data;
			end else begin
				out = out;
			end
		end 	
	end
endmodule

module IR(data, clreset, clock, out);
	input [32:0] data;
	input clreset, clock; 
	output [32:0] out; 
	reg [32:0] out; // 0 bit is write enable. Remember to write on negedge for the last flip flop
	wire clear;

	always @(posedge clock or posedge clreset) begin 
		if(clreset) begin
			out = {33{1'b0}};
		end else begin
				out = data;
		end 	
	end
endmodule

module jal_write_chain(writeOut, opcode, link_val, reset, clock);
	input [4:0] opcode;
	input [31:0] link_val;
	input reset, clock;
	output [32:0] writeOut;
	
	wire op_test;
	assign op_test = ~opcode[2] & opcode[1] & opcode[0];
	
	wire[32:0] firstOut, secondOut, init;
	assign init [32] = op_test;
	assign init [31:0] = link_val;
	IR first(.data(init), .clreset(reset), .clock(clock), .out(firstOut));
	IR second(.data(firstOut), .clreset(reset), .clock(clock), .out(secondOut));
	pipeline_last last(.data(secondOut), .clreset(reset), .clock(clock), .out(writeOut));
endmodule

module statusReg(data, clreset, clock, out, en);
	input [31:0]data;
	input clreset, clock, en; 
	output [31:0] out; 
	reg [31:0] out;

	always @(posedge clock or posedge clreset) begin 
		if(clreset) begin
			out = {32{1'b0}};
		end else begin
			if(en) begin
				out = data;
			end else begin
				out = out;
			end
		end 	
	end
endmodule

module pipeline_latch(data, clreset, clock, out);
	input [31:0] data;
	input clreset, clock; 
	output [31:0] out; 
	reg [31:0] out;
	wire clear;

	always @(posedge clock or posedge clreset) begin 
		if(clreset) begin
			out = {32{1'b0}};
		end else begin
				out = data;
		end 	
	end
endmodule

module pipeline_last(data, clreset, clock, out);
	input [32:0] data;
	input clreset, clock; 
	output [32:0] out; 
	reg [32:0] out;
	wire clear;

	always @(negedge clock or posedge clreset) begin 
		if(clreset) begin
			out = {33{1'b0}};
		end else begin
				out = data;
		end 	
	end
endmodule

module check_write_enable(enable, opcode, write_reg);
	input [4:0] opcode, write_reg;
	output enable; 
	
	//test enable
	wire RTypeTest, ZeroRegTest, Addi, Reg31Test;
	assign Addi = ~opcode[4] & opcode[2] & ~opcode[1] & opcode[0];
	nor(RTypeTest, opcode[0], opcode[1], opcode[2], opcode[3], opcode[4]); //00000 case
	or(ZeroRegTest, write_reg[0], write_reg[1], write_reg[2], write_reg[3], write_reg[4]);
	nand(Reg31Test, write_reg[0], write_reg[1], write_reg[2], write_reg[3], write_reg[4]);
	assign enable = (Addi | RTypeTest | opcode[3]) & ZeroRegTest & Reg31Test;
endmodule

module checkRegSwap(out, opcode);
	input[4:0] opcode;
	output out;
	
	assign out = (opcode[2]&opcode[1]&opcode[0])|(~opcode[2]&opcode[1]&~opcode[0]) | (~opcode[4]&opcode[2]&opcode[1]&~opcode[0]);
endmodule

module checkRegA(out, opcode, register);
	input[4:0] opcode, register;
	output[4:0] out;
	
	wire JType;
	assign JType = opcode[4] | (~opcode[2]&~opcode[1]&opcode[0]) | (~opcode[2]&opcode[1]&opcode[0]);
	
	genvar r;
	generate
		for(r = 0; r < 5; r=r+1) begin: regALoop
			assign out[r] = register[r] & ~JType;
		end
	endgenerate
endmodule

module checkRegB(out, opcode, register);
	input[4:0] opcode, register;
	output[4:0] out;
	
	wire inB, RType;
	nor(RType, opcode[4], opcode[3], opcode[2], opcode[1], opcode[0]);
	assign inB = RType | (~opcode[4]&opcode[2]&opcode[1]) | (~opcode[2]&opcode[1]&~opcode[0]);
	
	genvar r;
	generate
		for(r = 0; r < 5; r=r+1) begin: regBLoop
			assign out[r] = register[r] & inB;
		end
	endgenerate
endmodule

module sign_extension(out, immediate);
	input [16:0] immediate;
	output [31:0] out;
	
	genvar i;
	generate
	for(i = 0; i <= 16; i=i+1) begin: copyLoop
		assign out[i] = immediate[i];
	end
	endgenerate
	
	genvar j;
	generate
	for(j = 17; j < 32; j=j+1) begin: signLoop
		assign out[j] = immediate[16];
	end
	endgenerate
endmodule

module sign_extension_JI(out, immediate);
	input [26:0] immediate;
	output [31:0] out;
	
	genvar i;
	generate
	for(i = 0; i <= 26; i=i+1) begin: copy26Loop
		assign out[i] = immediate[i];
	end
	endgenerate
	
	genvar j;
	generate
	for(j = 27; j < 32; j=j+1) begin: sign26Loop
		assign out[j] = immediate[16];
	end
	endgenerate
endmodule

module mux21(out, in1, in2, select);
	input [31:0] in1, in2; 
	input select;
	output [31:0] out;
	
	wire selectn;
	not(selectn, select);
	
	wire [31:0] select1, select2;
	genvar c;
	generate
		for(c = 0; c < 32; c=c+1) begin: mux21Loop
			and(select1[c], in1[c], select);
			and(select2[c], in2[c], selectn);
			or(out[c], select1[c], select2[c]);
		end
	endgenerate
endmodule

module mux21_imm(out, in1, in2, select);
	input [4:0] in1, in2; 
	input select;
	output [4:0] out;
	
	wire selectn;
	not(selectn, select);
	
	wire [4:0] select1, select2;
	genvar c;
	generate
		for(c = 0; c < 5; c=c+1) begin: mux21immLoop
			and(select1[c], in1[c], select);
			and(select2[c], in2[c], selectn);
			or(out[c], select1[c], select2[c]);
		end
	endgenerate
endmodule

module mux21_PC(out, in1, in2, select);
	input [11:0] in1, in2; 
	input select;
	output [11:0] out;
	
	wire selectn;
	not(selectn, select);
	
	wire [11:0] select1, select2;
	genvar c;
	generate
		for(c = 0; c < 12; c=c+1) begin: mux21PCLoop
			and(select1[c], in1[c], select);
			and(select2[c], in2[c], selectn);
			or(out[c], select1[c], select2[c]);
		end
	endgenerate
endmodule

module mux21_33(out, in1, in2, select);
	input [32:0] in1, in2; 
	input select;
	output [32:0] out;
	
	wire selectn;
	not(selectn, select);
	
	wire [32:0] select1, select2;
	genvar c;
	generate
		for(c = 0; c < 33; c=c+1) begin: mux21_33Loop
			and(select1[c], in1[c], select);
			and(select2[c], in2[c], selectn);
			or(out[c], select1[c], select2[c]);
		end
	endgenerate
endmodule

module mux31(out, in1, in2, in3, select1, select2, select3);
	//note this mux is 1 hot to simplify life
	input [31:0] in1, in2, in3; 
	input select1, select2, select3;
	output [31:0] out;
	
	wire [31:0] final1, final2, final3;
	genvar c;
	generate
		for(c = 0; c < 32; c=c+1) begin: mux31Loop
			and(final1[c], in1[c], select1);
			and(final2[c], in2[c], select2);
			and(final3[c], in3[c], select3);
			or(out[c], final1[c], final2[c], final3[c]);
		end
	endgenerate
endmodule

module mux41p(out, in1, in2, in3, in4, select);
	input [31:0] in1, in2, in3, in4;
	input [1:0] select;
	output [31:0] out;
	
	wire zerozero, zeroone, onezero, oneone;
	assign zerozero = ~select[0] & ~select[1];
	assign zeroone = ~select[1] & select[0];
	assign onezero = select[1] & ~select[0];
	assign oneone = select[1] & select[0]; 
	
	wire [31:0] final1, final2, final3, final4;
	genvar c;
	generate
		for(c = 0; c < 32; c=c+1) begin: mux41Loop
			and(final1[c], in1[c], zerozero);
			and(final2[c], in2[c], zeroone);
			and(final3[c], in3[c], onezero);
			and(final4[c], in4[c], oneone);
			or(out[c], final1[c], final2[c], final3[c], final4[c]);
		end
	endgenerate
endmodule

module sw_instruction_change(insn_out, insn_in);
	input [31:0] insn_in;
	output [31:0] insn_out;
	
	genvar c;
	generate
		for(c = 0; c < 5; c=c+1) begin: rearrangeLoop
			assign insn_out[27+c] = insn_in[27+c];
			assign insn_out[17+c] = insn_in[17+c];
			assign insn_out[22+c] = insn_in[12+c];
			assign insn_out[12+c] = insn_in[22+c];
		end
	endgenerate
	
	genvar s;
	generate
		for(s = 0; s < 12; s=s+1) begin: insnEndLoop
			assign insn_out[s] = insn_in[s];
		end
	endgenerate		
endmodule

module sw_immediate(imm, insn);
	input [31:0] insn;
	output [16:0] imm;
	
	genvar s;
	generate
		for(s = 0; s < 12; s=s+1) begin: immediateTailLoop
			assign imm[s] = insn[s];
		end
	endgenerate
	
	genvar c;
	generate
		for(c = 0; c < 5; c=c+1) begin: immediateHeadLoop
			assign imm[12+c] = insn[22+c];
		end
	endgenerate
endmodule

module bypassReg_select(out, reg_writeXM, reg_writeWM, writeXM_en, writeWM_en, reg_readDX);
	input [4:0] reg_writeXM, reg_writeWM, reg_readDX;
	input writeXM_en, writeWM_en;
	output [2:0]out; // 0 - is neighbour, 1 is gap and 2 is normal
	
	//check neighor instruction and gap
	wire [4:0] equality, equality2;
	genvar c;
	generate
		for (c = 0; c < 5; c=c+1) begin: neigbourALoop
			xnor(equality[c], reg_readDX[c], reg_writeXM[c]);
			xnor(equality2[c], reg_readDX[c], reg_writeWM[c]);
		end
	endgenerate
	
	wire outNeighbour, outNeighbour2;
	and(outNeighbour, equality[0], equality[1], equality[2], equality[3], equality[4], writeXM_en);
	and(outNeighbour2, equality2[0], equality2[1], equality2[2], equality2[3], equality2[4], writeWM_en);
	
	//Enforce priority of neighbour over gap
	wire enforce;
	assign enforce = ~outNeighbour & outNeighbour2;
	
	assign out[0] = outNeighbour;
	assign out[1] = enforce;
	assign out[2] = ~outNeighbour & ~outNeighbour2;

endmodule

module bypassJump_select(out, reg_writeXM, reg_writeWM, reg_writeDX, writeXM_en, writeWM_en, writeDX_en, reg_readFD);
	input [4:0] reg_writeXM, reg_writeWM, reg_writeDX, reg_readFD;
	input writeXM_en, writeWM_en, writeDX_en;
	output [1:0]out; // 00 - DX, 01 - XM, 10 - WM, 11-normal
	
	wire [4:0] equalityDX, equalityXM, equalityWM;
	genvar c;
	generate
		for (c = 0; c < 5; c=c+1) begin: JumpBypassLoop
			xnor(equalityDX[c], reg_readFD[c], reg_writeDX[c]);
			xnor(equalityXM[c], reg_readFD[c], reg_writeXM[c]);
			xnor(equalityWM[c], reg_readFD[c], reg_writeWM[c]);
		end
	endgenerate
	
	wire bypassDX, bypassXM, bypassWM;
	and(bypassDX, equalityDX[0], equalityDX[1], equalityDX[2], equalityDX[3], equalityDX[4], writeDX_en);
	and(bypassWM, equalityWM[0], equalityWM[1], equalityWM[2], equalityWM[3], equalityWM[4], writeWM_en);
	and(bypassXM, equalityXM[0], equalityXM[1], equalityXM[2], equalityXM[3], equalityXM[4], writeXM_en);
	
	//Enforce priority of younger insn
	wire outDX, outXM, outWM, normal;
	assign outDX = bypassDX;
	assign outXM = bypassXM & ~outDX;
	assign outWM = bypassWM & ~outXM & ~outDX;
	assign normal = ~outDX & ~outXM & ~outWM;
	
	assign out[0] = normal | outXM;
	assign out[1] = normal | outWM;
endmodule

module bypass_lwsw(out, opcode_MW, opcode_XM, write_MW, read_RegB);
	//0 is normal 1 is jump
	input [4:0] opcode_XM, opcode_MW, write_MW, read_RegB;
	output out;
	
	wire opcodeCheck;
	and(opcodeCheck, opcode_MW[3], opcode_XM[2], opcode_XM[1], opcode_XM[0]);
	
	wire [4:0]regCheck;
	genvar s;
	generate
		for(s = 0; s < 5; s=s+1) begin: lwswCheckLoop
			xnor(regCheck[s], write_MW[s], read_RegB[s]);
		end
	endgenerate
	
	and(out, opcodeCheck, regCheck[4], regCheck[3], regCheck[2], regCheck[1], regCheck[0]);
endmodule

module stall_logic(out, DX_opcode, FD_opcode, DX_write, FD_Rone, FD_Rtwo);
	input [4:0] DX_opcode, FD_opcode, DX_write, FD_Rone, FD_Rtwo;
	output out;
	
	wire checkSW;
	wire [4:0] checkRone, checkRtwo;
	nand(checkSW, FD_opcode[2], FD_opcode[1], FD_opcode[0]);
	
	genvar c;
	generate
		for (c = 0; c < 5; c=c+1) begin: neigbourALoop
			xnor(checkRone[c], DX_write[c], FD_Rone[c]);
			xnor(checkRtwo[c], DX_write[c], FD_Rtwo[c]);
		end
	endgenerate
	
	wire Rone, Rtwo;
	and(Rone, checkRone[0], checkRone[1], checkRone[2], checkRone[3], checkRone[4]);
	and(Rtwo, checkRtwo[0], checkRtwo[1], checkRtwo[2], checkRtwo[3], checkRtwo[4]);
	
	wire checkRAW;
	or(checkRAW, Rone, Rtwo);
	
	and(out, checkRAW, checkSW, DX_opcode[3]);
endmodule
