/*******************************************************/
/********************Multiplier module********************/
/*****************************************************/
// add additional modules as needs, such as full adder, and others

// multiplier module
module part1
(
	input [7:0] x,
	input [7:0] y,
	output [15:0] out,   // Result of the multiplication
	output [15:0] pp [9] // for automarker to check partial products of a multiplication 
);
	// Declare a 9-high, 16-deep array of signals holding sums of the partial products.
	// They represent the _input_ partial sums for that row, coming from above.
	// The input for the "ninth row" is actually the final multiplier output.
	// The first row is tied to 0.
	assign pp[0] = '0;
	
	// Make another array to hold the carry signals
	logic [7:0] cin[9];
	assign cin[0] = '0;
	
	// Cin signals for the final (fast adder) row
	logic [16:8] cin_final;
	assign cin_final[8] = '0;
	
	// TODO: complete the following digital logic design of a carry save multiplier (unsigned)
	// Note: generate_hw tutorial can help you describe duplicated modules efficiently
	// Note: partial product of each row is the result coming out from a full adder at the end of that row
	genvar row, col;
	
	//iterate over an 8x8 matrix to instantiate 8 FA per row
	generate
		// iterate over 8 rows of the carry save array 
		for(row = 0; row < 8; row++) begin: rows
			//iterate over 8 columns per row in the carry save array
			for(col = 0; col < 8; col++) begin: columns
				logic a, b;
				assign a = x[col] & y[row];
				assign b = pp[row][col + row];
				//instantiate  the full adder and produce the appropriate cin outputs and new partial products
				full_adder FA(.a(a), .b(b), .cin(cin[row][col]), .cout(cin[row + 1][col]), .s(pp[row + 1][row + col]));
			end
		end
	endgenerate
	
	// Note: a "Fast adder" operates on columns 8 through 15 of final row.
	genvar i;
	generate
		//iterate over columns 8 to 16 to input the partial products from columns 8 to 16 as input into the FA of the fast adder
		for(i = 8; i < 16; i++) begin: fast_adder_circuit
			full_adder fa(.a(pp[8][i]), .b(cin[8][i-8]), .cin(cin_final[i]), .cout(cin_final[i+1]), .s(out[i]));
		end
	endgenerate
	
	// pad the remainder of the partial product bits to the left to 0 
	genvar j;
	generate
		//padding the bits from columns 8 to 15 depending on the row number from 1 to 8
		for(j = 1; j <= 8; j++) begin: pp_padding_8_15
			assign pp[j][15:j+7] = '0;
		end
	endgenerate
	
	// pass the partial product bits when col < row from the previous row
	genvar k;
	generate
		for(k = 2; k <= 8; k++) begin: pp_passing
			assign pp[k][k-2:0] = pp[k-1][k-2:0];
		end
	endgenerate
	
	// the output is equal to pp[9]
	assign out[7:0] = pp[8][7:0];
		  
endmodule

// The following code is provided for you to use in your design

module full_adder(
    input a,
    input b,
    input cin,
    output cout,
    output s
);

assign s = a ^ b ^ cin;
assign cout = a & b | (cin & (a ^ b));

endmodule


