module harness # (
	parameter IW = 32, // instr width
	parameter REGS = 32 // number of registers
)(
	input clk,
	input reset,
	
	// read only port
	output [IW-1:0] o_pc_addr,
	output o_pc_rd,
	input [IW-1:0] i_pc_rddata,
	output [3:0] o_pc_byte_en,

	// read/write port
	output [IW-1:0] o_ldst_addr,
	output o_ldst_rd,
	output o_ldst_wr,
	input [IW-1:0] i_ldst_rddata,
	output [IW-1:0] o_ldst_wrdata,
	output [3:0] o_ldst_byte_en
);
	// Register inputs and outputs
	logic [IW-1:0] o_pc_addr_r;
	logic o_pc_rd_r;
	logic [IW-1:0] i_pc_rddata_r;
	logic [3:0] o_pc_byte_en_r;
	logic [IW-1:0] o_ldst_addr_r;
	logic o_ldst_rd_r;
	logic o_ldst_wr_r;
	logic [IW-1:0] i_ldst_rddata_r;
	logic [IW-1:0] o_ldst_wrdata_r;
	logic [3:0] o_ldst_byte_en_r;
	
	always_ff @ (posedge clk) begin
		o_pc_addr <= o_pc_addr_r;
		o_pc_rd <= o_pc_rd_r;
		i_pc_rddata_r <= i_pc_rddata;
		o_pc_byte_en <= o_pc_byte_en_r;
		o_ldst_addr <= o_ldst_addr_r;
		o_ldst_rd <= o_ldst_rd_r;
		o_ldst_wr <= o_ldst_wr_r;
		i_ldst_rddata_r <= i_ldst_rddata;
		o_ldst_wrdata <= o_ldst_wrdata_r;
		o_ldst_byte_en <= o_ldst_byte_en_r;
	end
	
	// Instantiate the thing
	cpu DUT
	(
		.clk(clk),
		.reset(reset),
		.o_pc_addr(o_pc_addr_r),
		.o_pc_rd(o_pc_rd_r),
		.i_pc_rddata(i_pc_rddata_r),
		.o_pc_byte_en(o_pc_byte_en_r),
		.o_ldst_addr(o_ldst_addr_r),
		.o_ldst_rd(o_ldst_rd_r),
		.o_ldst_wr(o_ldst_wr_r),
		.i_ldst_rddata(i_ldst_rddata_r),
		.o_ldst_wrdata(o_ldst_wrdata_r),
		.o_ldst_byte_en(o_ldst_byte_en_r),
		.o_tb_regs()	// leave unconnected
	);

endmodule
