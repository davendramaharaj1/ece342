// This module uses parameterized instantiation. If values are not set in the testbench the default values specified here are used. 
// The values for EXP and MAN set here are for a IEEE-754 32-bit Floating point representation. 
// TO DO: Edit BITS and BIAS based on the EXP and MAN parameters. 

module part3
	#(parameter EXP = 8,			// Number of bits in the Exponent
	  parameter MAN = 23, 			// Number of bits in the Mantissa
	  parameter BITS = MAN + EXP + 1'd1,	// Total number of bits in the floating point number
	  parameter BIAS = 2**(EXP - 1'd1) - 1'd1		// Value of the bias, based on the exponent. 
	  )
	(
		input [BITS - 1:0] X,
		input [BITS - 1:0] Y,
		output inf, nan, zero, overflow, underflow,
		output reg[BITS - 1:0] result
);

// Design your 32-bit Floating Point unit here. 
// declare all logic signals
logic [EXP - 1:0] exponent;
logic [(2*MAN)+1:0] product_normalized;
logic [MAN + 1:0] product_round;
logic [BITS - 1:0] product;
logic _inf, _nan, _zero, _overflow, _underflow;

//declare some local parameters
localparam UPPER = 2**EXP - 1;
localparam LOWER = 0;
localparam E_B = {EXP{1'b1}};

// XOR the signs
assign product[BITS - 1] = X[BITS -1] ^ Y[BITS - 1];

// Add the exponents
assign exponent = X[BITS - 2:MAN] + Y[BITS - 2:MAN] - BIAS;

// Multiple X & Y Mantissas to yield a 48-bit result
assign product_normalized = {1'b1, X[MAN - 1:0]} * {1'b1, Y[MAN - 1:0]};

// Truncate and normalize the mantissa
assign product_round = product_normalized[(2*MAN)+1] ? product_normalized[(2*MAN)+1:MAN] >> 1 : product_normalized[(2*MAN)+1:MAN];

// Add to exponent if the mantissa is greater than 1
assign product[BITS - 2:MAN] = product_normalized[2*(MAN) + 1] ? exponent + 1 : exponent;

// Truncate he 2 hidden bits in the rounded mantissa with 25-bits
assign product[MAN - 1:0] = product_round[MAN - 1:0];

// check for any special cases
always_comb begin : special_cases
	// check for zero
	if((X[BITS - 2:0] == 0) || (Y[BITS - 2:0] == 0) || product[BITS - 2:0] == 0) begin
		result = 0;
		{_inf, _nan, _zero, _overflow, _underflow} = 5'b00100;
	end

	// check for NaN
	else if(((X[BITS - 2:MAN] == E_B) && X[MAN - 1:0] != 0) || ((Y[BITS - 2:MAN] == E_B) && Y[MAN-1:0] != 0) || ((product[BITS - 2:MAN] == E_B) && product[MAN - 1:0] != 0)) begin
		result = {1'b0, E_B, {MAN{1'b0}}};
		{_inf, _nan, _zero, _overflow, _underflow} = 5'b01000;
	end

	// check for infinity
	else if(((X[BITS - 2:MAN] == E_B) && X[MAN - 1:0] == 0) || ((Y[BITS - 2:MAN] == E_B) && Y[MAN - 1:0] == 0) || ((product[BITS - 2:MAN] == E_B) && product[MAN - 1:0] == 0))begin
		result = {1'b0, E_B, {MAN{1'b0}}};
		{_inf, _nan, _zero, _overflow, _underflow} = 5'b10000;
	end

	// check for underflow
	else if({1'b0, X[BITS - 2:MAN]} + {1'b0, Y[BITS - 2:MAN]} < LOWER + BIAS)begin
		result = {BITS{1'b0}};
		{_inf, _nan, _zero, _overflow, _underflow} = 5'b00001;
	end

	//check for overflow
	else if({1'b0, X[BITS - 2:MAN]} + {1'b0, Y[BITS - 2:MAN]} - BIAS > UPPER)begin
		result = {1'b0, E_B, {MAN{1'b0}}};
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