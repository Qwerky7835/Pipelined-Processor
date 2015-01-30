module regfile(clock, ctrl_writeEnable, ctrl_reset, ctrl_writeReg, ctrl_readRegA, ctrl_readRegB, data_writeReg, data_readRegA, data_readRegB);
	input clock, ctrl_writeEnable, ctrl_reset;
	input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
	input [31:0] data_writeReg;
	output [31:0] data_readRegA, data_readRegB;
	wire [31:0]decode_to_dff, decode_to_tristateA, decode_to_tristateB;
	wire [31:0] dffOut [31:0];
   
   //Decode the write
	decoder32 write_decode(.out(decode_to_dff), .select(ctrl_writeReg), .enable(ctrl_writeEnable));
   
   //Wire the dffarray
	genvar i;
	generate
		for (i=0; i<32; i=i+1) begin:
			DFF32 dff_Async_Reset32(.data(data_writeReg), .clreset(ctrl_reset), .clock(clk), .out(dffOut[i]), .decode_select(decode_to_dff[i]));
		end
	endgenerate
	
	//Decode the read
	decoder32 readA_decode(.out(decode_to_tristateA), .select(ctrl_readRegA), .enable(1));
	decoder32 readB_decode(.out(decode_to_tristateB), .select(ctrl_readRegB), .enable(1));
	
	//Wire the tristate
	genvar j;
	generate
		for (j=0; j<32; j=j+1) begin:
			TRI_A tri32(.in(dffOut[j]), .out(data_readRegA), .enable(decode_to_tristateA[j]));
		end
	endgenerate
	
	genvar k;
	generate
		for (k=0; k<32; k=k+1) begin:
			TRI_B tri32(.in(dffOut[k]), .out(data_readRegB), .enable(decode_to_tristateB[k]));
		end
	endgenerate

endmodule

module decoder32(out, select, enable);
	input [4:0] select;
	input enable;
	output [31:0] out;
	wire [3:0]firstout;

	decode2_4 ROOT(.out(firstout),.in({select[4],select[3]}));
	decode3_8 LEAF0(.out(out[7:0]), .in({select[2], select[1], select[0]}), .wireselect(firstout[0]), .enable(enable));
	decode3_8 LEAF1(.out(out[15:8]), .in({select[2], select[1], select[0]}), .wireselect(firstout[1]), .enable(enable));
	decode3_8 LEAF2(.out(out[23:16]), .in({select[2], select[1], select[0]}), .wireselect(firstout[2]), .enable(enable));
	decode3_8 LEAF3(.out(out[31:24]), .in({select[2], select[1], select[0]}), .wireselect(firstout[3]), .enable(enable));

endmodule

module decode2_4(out, in);
	input [1:0] in;
	output [3:0] out;
	wire notIn1, notIn2;
	
	not(notIn2, in[0]);
	not(notIn1, in[1]);
	
	and(out[0], notIn1, notIn2);
	and(out[1], notIn1, in[0]);
	and(out[2], in[1], notIn2);
	and(out[3], in[1], in[0]);
endmodule

module decode3_8(out, in , wireselect, enable);
	input [2:0] in;
	input wireselect, enable;
	output [7:0] out;
	wire notIn1, notIn2, notIn3;
	
	not(notIn3, in[0]);
	not(notIn2, in[1]);
	not(notIn1, in[2]);
	
	and(out[0], notIn1, notIn2,notIn3, wireselect, enable);
	and(out[1], notIn1, notIn2, in[0], wireselect, enable);
	and(out[2], notIn1, in[1], notIn3, wireselect, enable);
	and(out[3], notIn1, in[1], in[0], wireselect, enable);
	and(out[4], in[2], notIn2,notIn3, wireselect, enable);
	and(out[5], in[2], notIn2, in[0], wireselect, enable);
	and(out[6], in[2], in[1], notIn3, wireselect, enable);
	and(out[7], in[2], in[1], in[0], wireselect, enable);
endmodule

module dff_Async_Reset32(data, clreset, clock, out, decode_select);
	input [31:0] data;
	input clreset, clock, decode_select; 
	output [31:0] out; 
	reg [31:0] out;

	always @(posedge clock or posedge clreset or posedge decode_select) begin 
		if(clreset) begin 
			integer i;
			for (i = 0; i < 32; i = i + 1) begin
					out[i] = 0;
			end
		end
			
		else if (decode_select) begin
			integer i;
			for (i = 0; i < 32; i = i + 1) begin
					out[i] = data[i];
			end
		end
	end
endmodule

module tri32(in, out, enable);
	input [31:0] in;
	input enable;
	output [31:0] out;
	reg [31:0] out;
	
	always @(*) begin
		if(enable) begin
			integer i;
			for (i = 0; i < 32; i = i + 1) begin
				out[i] = in[i];
			end
		end else begin
			integer i;
			for (i = 0; i < 32; i = i + 1) begin
				out[i] = 1'bz;
			end
		end
	end
endmodule
