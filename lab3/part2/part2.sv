module part2
	(
		input [31:0] X,
		input [31:0] Y,
		output inf, nan, zero, overflow, underflow,
		output reg[31:0] result
);

// Design your 32-bit Floating Point unit here. 
// declare all logic signals
logic sign;
logic[7:0] exponent;
logic[47:0] product_normalized;
logic[24:0] product_round;
logic [31:0] product;
logic _inf, _nan, _zero, _overflow, _underflow;

// declare some local parameters
localparam E_B = 8'd255;
localparam UPPER = 9'sd255;
localparam LOWER = 9'sd0;

// XOR the signs
assign product[31] = X[31] ^ Y[31];

// Add the exponents
assign exponent = X[30:23] + Y[30:23] - 8'd127;

// Multiple X & Y Mantissas to yield a 48-bit result
assign product_normalized = {1'b1, X[22:0]} * {1'b1, Y[22:0]};

// Truncate and normalize the mantissa
assign product_round = product_normalized[47] ? product_normalized[47:23] >> 1 : product_normalized[47:23] >> 0;

// Add to exponent if the mantissa is greater than 1
assign product[30:23] = product_normalized[47] ? exponent + 1 : exponent;

// Truncate he 2 hidden bits in the rounded mantissa with 25-bits
assign product[22:0] = product_round[22:0];

// check for any special cases
always_comb begin : special_cases
	// check for zero
	if((X[30:0] == 0) || (Y[30:0] == 0) || product[30:0] == 0) begin
		result = 0;
		{_inf, _nan, _zero, _overflow, _underflow} = 5'b00100;
	end

	// check for NaN
	else if(((X[30:23] == E_B) && X[22:0] != 0) || ((Y[30:23] == E_B) && Y[22:0] != 0) || ((product[30:23] == E_B) && product[22:0] != 0)) begin
		result = {1'b0, E_B, 23'b0};
		{_inf, _nan, _zero, _overflow, _underflow} = 5'b01000;
	end

	// check for infinity
	else if(((X[30:23] == E_B) && X[22:0] == 0) || ((Y[30:23] == E_B) && Y[22:0] == 0) || ((product[30:23] == E_B) && product[22:0] == 0))begin
		result = {1'b0, E_B, 23'b0};
		{_inf, _nan, _zero, _overflow, _underflow} = 5'b10000;
	end

	// check for underflow
	else if({1'b0, X[30:23]} + {1'b0, Y[30:23]} < LOWER + 9'sd127)begin
		result = 32'b0;
		{_inf, _nan, _zero, _overflow, _underflow} = 5'b00001;
	end

	//check for overflow
	else if({1'b0, X[30:23]} + {1'b0, Y[30:23]} - 9'sd127 > UPPER)begin
		result = {1'b0, E_B, 23'b0};
		{_inf, _nan, _zero, _overflow, _underflow} = 5'b00010;
	end

	// no special cases therefore results is within range
	else begin
		result = product;
		{_inf, _nan, _zero, _overflow, _underflow} = 5'b0;
	end
end

// assign the special case output signals
assign {inf, nan, zero, overflow, underflow} = {_inf, _nan, _zero, _overflow, _underflow};
endmodule