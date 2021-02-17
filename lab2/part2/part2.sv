module part2 (
  input [7:0] x,
  input [7:0] y,
  output [15:0] out,
  output [15:0] pp [4]
);

// These signals are created to help you map the wires you need with the diagram provided in the lab document.

wire [15:0] s_lev01; //the first "save" output of level0's CSA array
wire [15:0] c_lev01; //the first "carry" output of level0's CSA array
wire [15:0] s_lev02; //the second "save" output of level0's CSA array
wire [15:0] c_lev02;
wire [15:0] s_lev11;
wire [15:0] c_lev11;
wire [15:0] s_lev12; //the second "save" output of level1's CSA array
wire [15:0] c_lev12;
wire [15:0] s_lev21;
wire [15:0] c_lev21;
wire [15:0] s_lev31;
wire [15:0] c_lev31;

// TODO: complete the hardware design for instantiating the CSA blocks per level.

// form the partial products i0 --> i7
logic [15:0] i0, i1, i2, i3, i4, i5, i6, i7;
logic overflow;
assign i0 = {8'b0, x & {8{y[0]}}};
assign i1 = {7'b0, x & {8{y[1]}}, 1'b0};
assign i2 = {6'b0, x & {8{y[2]}}, 2'b0};
assign i3 = {5'b0, x & {8{y[3]}}, 3'b0};
assign i4 = {4'b0, x & {8{y[4]}}, 4'b0};
assign i5 = {3'b0, x & {8{y[5]}}, 5'b0};
assign i6 = {2'b0, x & {8{y[6]}}, 6'b0};
assign i7 = {1'b0, x & {8{y[7]}}, 7'b0};


//level 0
csa #(.width(16)) csa_1(.op1(i0), .op2(i1), .op3(i2), .S(s_lev01), .C(c_lev01));
csa #(.width(16)) csa_2(.op1(i3), .op2(i4), .op3(i5), .S(s_lev02), .C(c_lev02));

//level 1
csa #(.width(16)) csa_3(.op1(s_lev01), .op2(c_lev01 << 1), .op3(s_lev02), .S(s_lev11), .C(c_lev11));
csa #(.width(16)) csa_4(.op1(c_lev02 << 1), .op2(i6), .op3(i7), .S(s_lev12), .C(c_lev12));

//level 2, the save and carry output of level 2 will be pp[2] and pp[3]
csa #(.width(16)) csa_5(.op1(c_lev11 << 1), .op2(s_lev11), .op3(s_lev12), .S(s_lev21), .C(c_lev21));
  
assign pp[0] = s_lev21;
assign pp[1] = c_lev21;

//level 3, the save and carry output of level 3 will be pp[2] and pp[3]
csa #(.width(16)) csa_6(.op1(s_lev21), .op2(c_lev21 << 1), .op3(c_lev12 << 1), .S(s_lev31), .C(c_lev31));
  
assign pp[2] = s_lev31;
assign pp[3] = c_lev31;

// Ripple carry adder to calculate the final output.
rca #(.width(16)) rca_1(.op1(s_lev31), .op2(c_lev31 << 1), .cin(1'b0), .sum(out), .cout(overflow));

endmodule





// These modules are provided for you to use in your designs.
// They also serve as examples of parameterized module instantiation.
module rca #(width=16) (
    input  [width-1:0] op1,
    input  [width-1:0] op2,
    input  cin,
    output [width-1:0] sum,
    output cout
);

wire [width:0] temp;
assign temp[0] = cin;
assign cout = temp[width];

genvar i;
generate
for( i=0; i<width; i=i+1) begin: ripple_carry
    full_adder u_full_adder(
        .a      (   op1[i]     ),
        .b      (   op2[i]     ),
        .cin    (   temp[i]    ),
        .cout   (   temp[i+1]  ),
        .s      (   sum[i]     )
    );
end
endgenerate
endmodule


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

module csa #(width=16) (
	input [width-1:0] op1,
	input [width-1:0] op2,
	input [width-1:0] op3,
	output [width-1:0] S,
	output [width-1:0] C
);

genvar i;
generate
	for(i=0; i<width; i++) begin: carry_save
		full_adder u_full_adder(
			.a      (   op1[i]    ),
			.b      (   op2[i]    ),
			.cin    (   op3[i]    ),
			.cout   (   C[i]	  ),
			.s      (   S[i]      )
		);
	end
endgenerate

endmodule

