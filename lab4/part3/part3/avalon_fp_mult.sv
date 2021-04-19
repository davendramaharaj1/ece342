module avalon_fp_mult
(
	input clk,
	input reset,
	
	// Avalon Slave
	input [2:0] avs_s1_address,
	input avs_s1_read,
	input avs_s1_write,
	input [31:0] avs_s1_writedata,
	output logic [31:0] avs_s1_readdata,
	output logic avs_s1_waitrequest
);

	// This module contains the entire FPM peripheral containing both the FPM Multiplier (FPM)
	// and the Avalon Slave Controller (AVS)

	// The following are some logic connectors between the AVS and FPM to exhange data
	logic [31:0] input_1, input_2;		// Input operands
	logic [31:0] result;				// Output result
	logic [3:0]	flags;					// Status Flags
	logic clk_en;

	// The following module is the Avalon Slave Controller 
	avalon_slave avs
	(
		.clk(clk),
		.reset(reset),
		.avs_s1_address(avs_s1_address),
		.avs_s1_read(avs_s1_read),
		.avs_s1_write(avs_s1_write),
		.avs_s1_writedata(avs_s1_writedata),
		.avs_s1_readdata(avs_s1_readdata),
		.avs_s1_waitrequest(avs_s1_waitrequest),

		// FP_MULT connections
		.input_1(input_1),
		.input_2(input_2),
		.result(result),
		.flags(flags),
		.clk_en(clk_en)
	);

	// The Following module is the Floating Point Multiplier
	fp_mult fpm
	(
		.aclr(reset),
		.clk_en(clk_en),
		.clock(clk),
		.dataa(input_1),
		.datab(input_2),
		.result(result),
		.overflow(flags[3]),
		.underflow(flags[2]),
		.zero(flags[1]),
		.nan(flags[0])
	);

endmodule
