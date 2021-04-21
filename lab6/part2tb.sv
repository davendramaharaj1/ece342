`timescale 1 ps / 1 ps
module tb
();

logic clk;
logic reset;
logic [31:0] o_pc_addr;
logic o_pc_rd;
logic [31:0] i_pc_rddata;
logic [3:0] o_pc_byte_en;
logic [31:0] o_ldst_addr;
logic o_ldst_rd;
logic o_ldst_wr;
logic [31:0] i_ldst_rddata;
logic [31:0] o_ldst_wrdata;
logic [3:0] o_ldst_byte_en;
logic i_ldst_waitrequest;
logic [31:0] o_tb_regs [0:31];

always begin
	#5 clk = ~clk;
end

logic [31:0] o_pc_addr2;

initial begin
	wait(o_pc_rd == 1 && o_pc_addr == 0);
	i_pc_rddata = 32'h00402503;
	wait(o_pc_rd == 1 && o_pc_addr == 4);
	i_pc_rddata = 32'h00a02423;
	wait(o_pc_rd == 1 && o_pc_addr >= 8);
	i_pc_rddata = 32'h00000013;
	#1000
	$exit;
end

initial begin
	clk = 0;
	reset = 1;
	i_ldst_waitrequest = 0;
	#20
	reset = 0;
	wait(o_ldst_rd == 1 && o_ldst_addr == 32'h4);
	i_ldst_rddata = 32'h12345678;
	i_ldst_waitrequest = 1;
	o_pc_addr2 = o_pc_addr;
	#200
	if (o_pc_addr2 != o_pc_addr) begin
		$display("Processor did not wait, FAIL.");
		$exit;
	end
	i_ldst_waitrequest = 0;
	#11
	if (o_tb_regs[10] == 32'h12345678) begin
		$display("Processor read the data, PASS.");
	end else begin
		$display("Processor did not read the data, FAIL.");
		$exit;
	end
	wait(o_ldst_wr == 1 && o_ldst_addr == 32'h8);
	i_ldst_waitrequest = 1;
	#200
	if (o_ldst_wr != 1 && o_ldst_addr != 32'h8) begin
		$display("Processor did not wait, FAIL.");
		$exit;
	end else begin
		$display("Processor wrote the data, PASS.");
	end
	$finish;
end

cpu uut
(
	.*
);

endmodule
