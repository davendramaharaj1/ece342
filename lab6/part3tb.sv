`timescale 1 ps / 1 ps
module tb
();

logic clk;
logic reset;

always begin
	#5 clk = ~clk;
end

initial begin
	clk = 0;
	reset = 1;
	#20
	reset = 0;
	uut.mem_inst.ram[8189] = 32'h40500000;
	uut.mem_inst.ram[8190] = 32'h4101EB85;
	#10000
	if (uut.mem_inst.ram[8191] == 32'h41D31EB8) begin
		$display("Multiplication correct, PASS.");
	end else begin
		$display("Multiplication incorrect, FAIL.");
	end
	uut.mem_inst.ram[8189] = 32'h41D31EB8;
	uut.mem_inst.ram[8190] = 32'h40B1999A;
	#10000
	if (uut.mem_inst.ram[8191] == 32'h431276EA) begin
		$display("Multiplication correct, PASS.");
	end else begin
		$display("Multiplication incorrect, FAIL.");
	end
	$finish;
end

part3 uut
(
	.clk(clk),
    .reset(reset)
);

endmodule
