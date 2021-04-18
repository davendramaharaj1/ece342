module mem # (
	parameter WIDTH = 32,
	parameter DEPTH = 8192,
	parameter HEX_FILE = "part1.hex"
)(
	input clk,
	input reset,

	// Read/Write port
	input [$clog2(DEPTH)-1:0] p1_addr,
	input p1_write,
	input p1_read,
	input [(WIDTH/8)-1:0] p1_byteenable,
	input [WIDTH-1:0] p1_writedata,
	output logic [WIDTH-1:0] p1_readdata,

	// Read only port
	input [$clog2(DEPTH)-1:0] p2_addr,
	input p2_read,
	input [(WIDTH/8)-1:0] p2_byteenable,
	output logic [WIDTH-1:0] p2_readdata
);

integer i;
logic [WIDTH-1:0] ram [0:DEPTH-1];
logic [7:0] temp_ram [0:(WIDTH/8)*DEPTH-1];

initial begin
	$readmemh(HEX_FILE, temp_ram);
end

always_comb begin
	p1_readdata = 32'h0;
	p2_readdata = 32'h0;
	if (p1_read) begin
		if (p1_byteenable[0]) begin
			p1_readdata[7:0] = ram[p1_addr][7:0];
		end
		if (p1_byteenable[1]) begin
			p1_readdata[15:8] = ram[p1_addr][15:8];
		end
		if (p1_byteenable[2]) begin
			p1_readdata[23:16] = ram[p1_addr][23:16];
		end
		if (p1_byteenable[3]) begin
			p1_readdata[31:24] = ram[p1_addr][31:24];
		end
	end
	if (p2_read) begin
		if (p2_byteenable[0]) begin
			p2_readdata[7:0] = ram[p2_addr][7:0];
		end
		if (p2_byteenable[1]) begin
			p2_readdata[15:8] = ram[p2_addr][15:8];
		end
		if (p2_byteenable[2]) begin
			p2_readdata[23:16] = ram[p2_addr][23:16];
		end
		if (p2_byteenable[3]) begin
			p2_readdata[31:24] = ram[p2_addr][31:24];
		end
	end
end

always_ff @(posedge clk) begin
	if (reset) begin
		for (i = 0; i < DEPTH; i++)
			ram[i] = {temp_ram[4*i+3], temp_ram[4*i+2], temp_ram[4*i+1], temp_ram[4*i]};
	end else begin
		if (p1_write) begin
			if (p1_byteenable[0]) begin
				ram[p1_addr][7:0] = p1_writedata[7:0];
			end
			if (p1_byteenable[1]) begin
				ram[p1_addr][15:8] = p1_writedata[15:8];
			end
			if (p1_byteenable[2]) begin
				ram[p1_addr][23:16] = p1_writedata[23:16];
			end
			if (p1_byteenable[3]) begin
				ram[p1_addr][31:24] = p1_writedata[31:24];
			end
		end
	end
end

endmodule
