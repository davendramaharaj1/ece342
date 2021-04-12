module cpu # (
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
	output [3:0] o_ldst_byte_en,
	
	output [IW-1:0] o_tb_regs [0:REGS-1]
);

	/* Process Memory Interface Registers */ 
	logic [IW-1] IR;	// instruction register
	logic [IW-1] immediate // holds immediate value --> extended to 32 bits (if arithemtic-op then signed else zero-padded)

	/* Address Generator Registers and signals  */
	logic [IW-1] PC;	// holds address to the next instruction
	logic [3:0] pc_byte_en 	// size of pc to read : word/half-word/byte
	logic decode;		// decode the IR instruction
	logic fetch;		// control signal to fetch instruction from memory
	logic pc_increment;		// control signal to increment pc 

	/* Register File Registers and breakdown logic from IR */
	logic [4:0] rs1, rs2, rd;
	logic [2:0] funct3;
	logic [6:0] funct7;
	logic [IW-1] REG_FILE [0:REGS-1]

	/* ALU Registers and signals */
	logic [2:0] ALU_op;
	logic [IW-1] result;
	logic Alu_en;

	/* define states to represent stages in processing instructions */
	enum int unsigned {
		FETCH,
		DECODE,
		EXECUTE,
		MEM_ACCESS,
		DEST_REG
	} state, nextstate;

	/* ALU operation enums */
	localparam [2:0] R_TYPE	= 3'd0;
	localparam [2:0] I_IMM	= 3'd1;
	localparam [2:0] I_LD	= 3'd2;
	localparam [2:0] I_JUMP	= 3'd3;
	localparam [2:0] S_TYPE	= 3'd4;
	localparam [2:0] B_TYPE	= 3'd5;
	localparam [2:0] U_TYPE	= 3'd6;
	localparam [2:0] J_TYPE	= 3'd7;

	/* store PC instruction into IR */ 
	assign IR = i_pc_rddata;

	/* define the different opcodes */
	localparam [6:0] R = 7'b0110011;
	localparam [6:0] I_imm = 7'b0010011;
	localparam [6:0] I_ld = 7'b0000011;
	localparam [6:0] I_jump = 7'b1100111;
	localparam [6:0] S = 7'b0110011;
	localparam [6:0] B = 7'b0110011;
	localparam [6:0] U = 7'b0110011; 
	localparam [6:0] J = 7'b0110011;
	
	/***************************************########### RISC V FSM ###############***************************************/

	/* Control FSM Flip Flops */
	always_ff @(posedge clk) begin : FSMTransition
		if(reset) begin
			/* set the PC to point to the first instruction */
			PC <= 32'b0;
			/* Ensure the register file is zeroed */
			integer i;
			for(i = 0; i < IW; i=i+1)begin
				REG_FILE[i] <= 32'b0;
			end
			state <= FETCH;
		end
		else state <= nextstate;
	end

	/* Control FSM Outputs */
	always_comb begin : State_Machine

		/* default control signals to prevent latches */
		decode = 1'b0;
		fetch = 1'b0;
		pc_increment = 1'b0;
		pc_byte_en = 4'b1111;
		Alu_en = 1'b0;
		nextstate = state;

		case(state)
			FETCH: begin
				/* send signal to processor interface to read the pc address */
				fetch = 1'b1;
				/* get the entire word for PC */
				pc_byte_en = 4'b1111;
				/* increment the PC */
				pc_increment = 1'b1;
				/* next state transition */
				nextstate = DECODE;
			end

			DECODE:begin
				/* signal to decode instruction and load appropriate registers */ 
				decode = 1'b1;
				nextstate = EXECUTE;
			end

			EXECUTE: begin
				/* enable alu to perform appropriate operation */
				Alu_en = 1'b1;
				/* next state depends on the ALU operation */
				case (ALU_op)
					R_TYPE, I_IMM, I_JUMP, U_TYPE, J_TYPE: begin
						nextstate = DEST_REG;
					end
					B_TYPE: begin
						nextstate = FETCH;
					end
					I_LD, S_TYPE: begin
						nextstate = MEM_ACCESS;
					end
				endcase
			end

			MEM_ACCESS: begin
				
			end

			DEST_REG: begin
				
			end
		endcase

	end

	/***************************************########## RISC V FSM ################***************************************/


	/***************************************######### RISC V DATAPATH #############***************************************/

	/* decoder to load the appropriate registers after receiving instruction on IR*/
	always_comb begin : Decoder
		if(decode) begin
			case (IR[6:0])
				/* R Type instruction */
				R: begin
					rs1 = IR[19:15];
					rs2 = IR[24:20];
					rd = IR[11:7];
					funct3 = IR[14:12];
					funct7 = IR[31:25];
				end
				/* I Type instruction */
				I_imm, I_ld, I_jump: begin
					rs1 = IR[19:15];
					rd = IR[11:7];
					funct3 = IR[14:12];
					immediate = {{21{IR[31]}},IR[30:20]};
				end
				/* S Type instruction */
				S: begin
					rs1 = IR[19:15];
					rs2 = IR[24:20];
					funct3 = IR[14:12];
					immediate = {{21{IR[31]}},IR[30:25],IR[11:7]};
				end
				/* B Type instruction */
				B: begin
					rs1 = IR[19:15];
					rs2 = IR[24:20];
					funct3 = IR[14:12];
					immediate = {{20{IR[31]}},IR[7],IR[30:25],IR[11:8],1'b0};
				end
				/* U Type instruction */
				U: begin
					rd = IR[11:7];
					immediate = {IR[31:12],12'b0};
				end
				/* J Type instruction */
				J: begin
					rd = IR[11:7];
					immediate = {{12{IR[31]}},IR[19:12],IR[20],IR[30:21],1'b0};
				end
			endcase
		end
	end

	/* increment PC by 4 from the control signal */
	always_ff @(clk) begin : PC_Increment
		if(pc_increment) begin
			PC <= PC + 3'd4;
		end
	end

	/* Get the ALU Op code to know which ALU Operation to perform */
	always_comb begin : ALUOP
		case (IR[6:0])
			R: 		ALU_op = R_TYPE;
			I_imm:	ALU_op = I_IMM;
			I_ld: 	ALU_op = I_LD;
			I_jump: ALU_op = I_JUMP;
			S: 		ALU_op = S_TYPE;
			B:		ALU_op = B_TYPE;
			U:		ALU_op = U_TYPE;
			J:		ALU_op = J_TYPE; 
		endcase
	end

	/* ALU logic */
	always_comb begin : ALU_logic
		if(Alu_en) begin
			/* registers as operands for arithmetic */
			if(Alu_op == R_TYPE) begin
				// add
				if(funct3 = 4'h0 and funt7 = 8'h00) begin
					result = REG_FILE[rs1] + REG_FILE[rs2];
				end
				// sub
				else if (funct3 = 4'h0 and funct7 == 8'h20) begin
					result = REG_FILE[rs1] - REG_FILE[rs2];
				end
				// xor
				else if(funct3 = 4'h4 and funct7 == 8'h00)begin
					result = REG_FILE[rs1] ^ REG_FILE[rs2];
				end
				// or
				else if(funct3 == 4'h6 and funct7 == 8'h00)begin
					result = REG_FILE[rs1] | REG_FILE[rs2];
				end
				// and
				else if(funct3 == 4'h7 and funct7 == 8'h00)begin
					result = REG_FILE[rs1] & REG_FILE[rs2];
				end
				//sll
				else if(funct3 == 4'h1 and funct7 == 8'h00)begin
					result = REG_FILE[rs1] << REG_FILE[rs2][4:0];
				end
				//srl
				else if(funct3 == 4'h5 and funct7 == 8'h00)begin
					result = REG_FILE[rs1] >> REG_FILE[rs2][4:0];
				end
				//sra
				else if(funct3 == 4'h5 and funct7 == 8'h20)begin
					result = $signed(REG_FILE[rs1]) >>> REG_FILE[rs2][4:0];
				end
				//slt
				else if(funct3 == 4'h2 and funct7 == 8'h00)begin
					result = $signed(REG_FILE[rs1]) < $signed(REG_FILE[rs2]) ? 1 : 0;
				end
				//sltu
				else if(funct3 == 4'h3 and funct7 == 8'h00)begin
					result = REG_FILE[rs1] < REG_FILE[rs2] ? 1 : 0;
				end
			end

			/* arithmetic I type with register and immediate value */
			else if(Alu_op == I_IMM)begin
				// addi
				if(funct3 = 4'h0) begin
					result = REG_FILE[rs1] + immediate;
				end
				// xori
				else if(funct3 = 4'h4)begin
					result = REG_FILE[rs1] ^ immediate;
				end
				// ori
				else if(funct3 == 4'h6)begin
					result = REG_FILE[rs1] | immediate;
				end
				// andi
				else if(funct3 == 4'h7)begin
					result = REG_FILE[rs1] & immediate;
				end
				//slli
				else if(funct3 == 4'h1 and immediate[11:5] == 8'h00)begin
					result = REG_FILE[rs1] << immediate[4:0];
				end
				//srli
				else if(funct3 == 4'h5 and immediate[11:5] == 8'h00)begin
					result = REG_FILE[rs1] >> immediate[4:0];
				end
				//srai
				else if(funct3 == 4'h5 and immediate[11:5] == 8'h20)begin
					result = $signed(REG_FILE[rs1]) >>> immediate[4:0];
				end
				//slti
				else if(funct3 == 4'h2) begin
					result = $signed(REG_FILE[rs1]) < $signed(immediate) ? 1 : 0;
				end
				//sltiu
				else if(funct3 == 4'h3 and funct7 == 8'h00)begin
					result = REG_FILE[rs1] < immediate ? 1 : 0;
				end
			end

			/* arithmetic I type with register and PC */
			else if(Alu_op == I_JUMP)begin
				if(funct3 == 4'h0)begin
					result = PC + 2'd4;
					PC = REG_FILE[rs1] + immediate;
				end
			end

			/* branching */
			else if(Alu_op == B_TYPE)begin
				// beq
				if(funct3 == 4'h0)begin
					PC = $signed(REG_FILE[rs1]) == $signed(REG_FILE[rs2]) ? PC + immediate : PC + 2'd4;
				end
				// bne
				if(funct3 == 4'h1)begin
					PC = $signed(REG_FILE[rs1]) != $signed(REG_FILE[rs2]) ? PC + immediate : PC + 2'd4;
				end
				//blt
				if(funct3 == 4'h4)begin
					PC = $signed(REG_FILE[rs1]) < $signed(REG_FILE[rs2]) ? PC + immediate : PC + 2'd4;
				end
				//bge
				if(funct3 == 4'h5)begin
					PC = $signed(REG_FILE[rs1]) >= $signed(REG_FILE[rs2]) ? PC + immediate : PC + 2'd4;
				end
				//bltu
				if(funct3 == 4'h6)begin
					PC = REG_FILE[rs1] < REG_FILE[rs2] ? PC + immediate : PC + 2'd4;
				end
				//bgeu
				if(funct3 == 4'h7)begin
					PC = REG_FILE[rs1] >= REG_FILE[rs2] ? PC + immediate : PC + 2'd4;
				end
			end
		end
	end
	/***************************************########## RISC V DATAPATH ############***************************************/

	/* outputs to the Processor Signal Interface */
	assign o_pc_rd = fetch;
	assign o_tb_regs = REG_FILE;
	assign o_pc_addr = PC;
	assign o_pc_byte_en = pc_byte_en;
endmodule

