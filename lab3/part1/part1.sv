module part1
	(
		input [31:0] X,
		input [31:0] Y,
		output [31:0] result
);

// Design your 32-bit Floating Point unit here. 
// declaration of variables for floating point multiplication
logic sign;	// sign of mantissa
logic [23:0] mantissa_X, mantissa_Y;
logic [22:0] mantissa_final;
logic [47:0] product_normalized;
logic [7:0] exponent;
logic [24:0] product_rounded;

// XOR sign of both numbers X and Y to get the sign of the result
assign sign = X[31] ^ Y[31];

// Extract the mantissas for each number and add a 1 for normalization to the MSB
assign mantissa_X = {1'b1, X[22:0]};
assign mantissa_Y = {1'b1, Y[22:0]};

// Multiple both mantissas ; yields a 48-bit result with 46-bits to the right of the binary point
assign product_normalized = mantissa_X * mantissa_Y;

// Truncate the LSB 23 bits of the mantissa_product to get a 25-bit number --> 2 hidden bits and 23 mantissa bits
assign product_rounded = product_normalized[47:23];

// Normalize the mantissa by shifting it right if it is greater than 1
always_comb begin : normalize_mantissa
	if(product_rounded > 1'd1) begin
		product_rounded = product_rounded >> 1;
		exponent = exponent + 1;
	end
	else begin
		product_rounded = product_rounded >> 0;
		exponent = exponent + 0;
	end
end

// Truncate the 2 most significant bits of the result mantissa
assign mantissa_final = product_rounded[22:0];

// Add both exponents
assign exponent = X[30:23] + Y[30:23] - 8'd127;

// product the result
assign result = {sign, exponent, mantissa_final};
end

endmodule
