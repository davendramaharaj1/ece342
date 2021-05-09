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
	input i_ldst_waitrequest,
	
	output [IW-1:0] o_tb_regs [0:REGS-1]
);

	/* Process Memory Interface Registers */ 
	logic [IW-1:0] IR;	// instruction register
	logic [IW-1:0] immediate; // holds immediate value --> extended to 32 bits (if arithemtic-op then signed else zero-padded)

	/* Address Generator Registers and signals  */
	logic [IW-1:0] PC_1, PC_2;	// holds address to the next instruction
	logic [3:0] pc_byte_en;	// size of pc to read : word/half-word/byte
	logic decode;		// decode the IR instruction
	logic fetch;		// control signal to fetch instruction from memory
	logic pc_increment;		// control signal to increment pc 

	/* Register File Registers and breakdown logic from IR */
	logic [4:0] rs1, rs2, rd, rd_stage4;
	logic [2:0] funct3, funct3_stage4;
	logic [6:0] funct7;
	logic [IW-1:0] REG_FILE [0:REGS-1];

	/* ALU Registers and signals */
	logic [3:0] Alu_op, opcode;
	logic [IW-1:0] result;
	logic resultIn;
	logic Alu_en;

	/* Load/Store reg control signals */
	logic [IW-1:0] ldst_addr;
	logic [IW-1:0] ldst_rddata;
	logic [IW-1:0] ldst_wrdata;
	logic [3:0] ldst_byte_en;
	logic ldst_rd;
	logic ldst_wr;
	logic loadIn;

	/* valid registers for each state */
	logic stage1, stage2, stage3, stage4;

	/* ALU operation types */
	localparam [3:0] R_TYPE	= 4'd0;
	localparam [3:0] I_IMM	= 4'd1;
	localparam [3:0] I_LD	= 4'd2;
	localparam [3:0] I_JUMP	= 4'd3;
	localparam [3:0] S_TYPE	= 4'd4;
	localparam [3:0] B_TYPE	= 4'd5;
	localparam [3:0] U_LD	= 4'd6;
	localparam [3:0] U_PC	= 4'd7;
	localparam [3:0] J_TYPE	= 4'd8;

	/* store PC instruction into IR */ 
	//assign IR = i_pc_rddata;
	assign ldst_rddata = i_ldst_rddata;

	/* define the different opcodes */
	localparam [6:0] R		= 7'b0110011;
	localparam [6:0] I_imm 	= 7'b0010011;
	localparam [6:0] I_ld 	= 7'b0000011;
	localparam [6:0] I_jump = 7'b1100111;
	localparam [6:0] S 		= 7'b0100011;
	localparam [6:0] B 		= 7'b1100011;
	localparam [6:0] U_ld 	= 7'b0110111; 
	localparam [6:0] U_pc 	= 7'b0010111;
	localparam [6:0] J 		= 7'b1101111;

	/***************************************########## RISC V CONTROL PATH #########***************************************/
	/* Control path for pipelined stages */
	always_ff @( posedge clk ) begin : PipelinedStages

		/* default to eliminate latches */
		fetch = 1'b0;
		decode = 1'b0;
		Alu_en = 1'b0;
		pc_increment = 1'b0;

		/* STAGE 1: FETCH */
		if(stage1) begin
			/* read from pc memory port */
			fetch = 1'b1;
			/* increment pc */
			pc_increment = 1'b1;
			/* move to decode stage */
			stage2 = 1'b1;
		end

		/* STAGE 2: DECODE */
		if(stage2) begin
			/* place instruction from memory into IR*/
			IR = i_pc_rddata;
			/* implement decoding of instructions */
			decode = 1'b1;
			/* move to execute stage */
			stage3 = 1'b1;
		end

		/* STAGE 3: EXECUTE */
		if(stage3) begin
			/* Enable ALU to perform appropriate operation */
			Alu_en = 1'b1;
		end

		/* STAGE 4: WRITE_BACK */
		if(stage4) begin
			
		end
	end
	/***************************************########## RISC V CONTROL PATH #########***************************************/


	/***************************************######### RISC V DATAPATH #############***************************************/
	/* Control Reset */
	integer i;
	always_ff @(posedge clk or posedge reset) begin : ControlReset
		if(reset) begin
			/* reset all valid registers */
			stage1 <= 1'b1;
			stage2 <= 1'b0;
			stage3 <= 1'b0;
			stage4 <= 1'b0;
			/* set the PC and PC_next to point to the first instruction */
			PC_1 <= 32'b0;
			PC_2 <= 32'b0;
			/* Ensure the register file is zeroed */
			for(i = 0; i < IW; i=i+1)begin
				REG_FILE[i] <= 32'b0;
			end
		end
	end

	/* increment PC by 4 from the control signal */
	always_ff @(posedge clk or posedge reset) begin : PC_Increment
		if(reset)begin
			PC_1 <= 32'b0;
			PC_2 <= 32'b0;
		end
		else if(pc_increment) begin
			PC_1 <= PC_1 + 4;
			PC_2 <= PC_1;
		end
	end

	/* decoder to load the appropriate registers after receiving instruction on IR*/
	always_ff@(posedge clk) begin : Decoder
		if(decode) begin
			case (IR[6:0])
				/* R Type instruction */
				R: begin
					rs1 <= IR[19:15];
					rs2 <= IR[24:20];
					rd <= IR[11:7];
					funct3 <= IR[14:12];
					funct7 <= IR[31:25];
				end
				/* I Type instruction */
				I_imm, I_ld, I_jump: begin
					rs1 <= IR[19:15];
					rd <= IR[11:7];
					funct3 <= IR[14:12];
					immediate <= {{21{IR[31]}},IR[30:20]};
				end
				/* S Type instruction */
				S: begin
					rs1 <= IR[19:15];
					rs2 <= IR[24:20];
					funct3 <= IR[14:12];
					immediate <= {{21{IR[31]}},IR[30:25],IR[11:7]};
				end
				/* B Type instruction */
				B: begin
					rs1 <= IR[19:15];
					rs2 <= IR[24:20];
					funct3 <= IR[14:12];
					immediate <= {{20{IR[31]}},IR[7],IR[30:25],IR[11:8],1'b0};
				end
				/* U Type instruction */
				U_ld, U_pc: begin
					rd <= IR[11:7];
					immediate <= {IR[31:12],12'b0};
				end
				/* J Type instruction */
				J: begin
					rd = IR[11:7];
					immediate <= {{12{IR[31]}},IR[19:12],IR[20],IR[30:21],1'b0};
				end
			endcase
		end
	end

	/* Get the ALU Op code to know which ALU Operation to perform */
	always_ff@(posedge clk) begin : ALUOP
		case (IR[6:0])
			R: 		Alu_op <= R_TYPE;
			I_imm:	Alu_op <= I_IMM;
			I_ld: 	Alu_op <= I_LD;
			I_jump: Alu_op <= I_JUMP;
			S: 		Alu_op <= S_TYPE;
			B:		Alu_op <= B_TYPE;
			U_ld:	Alu_op <= U_LD;
			U_pc:	Alu_op <= U_PC;
			J:		Alu_op <= J_TYPE; 
		endcase
	end

	/* ALU logic */
	always_ff @(posedge clk) begin : ALU_logic
		if(Alu_en) begin

			/* save the destination register, function and opcodes for stage 4 */
			rd_stage4 <= rd;
			opcode <= Alu_op;
			funct3_stage4 <= funct3;

			/* registers as operands for arithmetic */
			if(Alu_op == R_TYPE) begin
				// add
				if(funct3 == 4'h0 && funct7 == 8'h00) begin
					result <= REG_FILE[rs1] + REG_FILE[rs2];
				end
				// sub
				else if (funct3 == 4'h0 && funct7 == 8'h20) begin
					result <= REG_FILE[rs1] - REG_FILE[rs2];
				end
				// xor
				else if(funct3 == 4'h4 && funct7 == 8'h00)begin
					result <= REG_FILE[rs1] ^ REG_FILE[rs2];
				end
				// or
				else if(funct3 == 4'h6 && funct7 == 8'h00)begin
					result <= REG_FILE[rs1] | REG_FILE[rs2];
				end
				// and
				else if(funct3 == 4'h7 && funct7 == 8'h00)begin
					result <= REG_FILE[rs1] & REG_FILE[rs2];
				end
				//sll
				else if(funct3 == 4'h1 && funct7 == 8'h00)begin
					result <= REG_FILE[rs1] << REG_FILE[rs2][4:0];
				end
				//srl
				else if(funct3 == 4'h5 && funct7 == 8'h00)begin
					result <= REG_FILE[rs1] >> REG_FILE[rs2][4:0];
				end
				//sra
				else if(funct3 == 4'h5 && funct7 == 8'h20)begin
					result <= $signed(REG_FILE[rs1]) >>> REG_FILE[rs2][4:0];
				end
				//slt
				else if(funct3 == 4'h2 && funct7 == 8'h00)begin
					result <= $signed(REG_FILE[rs1]) < $signed(REG_FILE[rs2]) ? 1 : 0;
				end
				//sltu
				else if(funct3 == 4'h3 && funct7 == 8'h00)begin
					result <= REG_FILE[rs1] < REG_FILE[rs2] ? 1 : 0;
				end
			end

			/* arithmetic I type with register and immediate value */
			else if(Alu_op == I_IMM)begin
				// addi
				if(funct3 == 4'h0) begin
					result <= REG_FILE[rs1] + immediate;
				end
				// xori
				else if(funct3 == 4'h4)begin
					result <= REG_FILE[rs1] ^ immediate;
				end
				// ori
				else if(funct3 == 4'h6)begin
					result <= REG_FILE[rs1] | immediate;
				end
				// andi
				else if(funct3 == 4'h7)begin
					result <= REG_FILE[rs1] & immediate;
				end
				//slli
				else if(funct3 == 4'h1 && immediate[11:5] == 8'h00)begin
					result <= REG_FILE[rs1] << immediate[4:0];
				end
				//srli
				else if(funct3 == 4'h5 && immediate[11:5] == 8'h00)begin
					result <= REG_FILE[rs1] >> immediate[4:0];
				end
				//srai
				else if(funct3 == 4'h5 && immediate[11:5] == 8'h20)begin
					result <= $signed(REG_FILE[rs1]) >>> immediate[4:0];
				end
				//slti
				else if(funct3 == 4'h2) begin
					result <= $signed(REG_FILE[rs1]) < $signed(immediate) ? 1 : 0;
				end
				//sltiu
				else if(funct3 == 4'h3 && funct7 == 8'h00)begin
					result <= REG_FILE[rs1] < immediate ? 1 : 0;
				end
			end

			/* arithmetic I type with register and PC */
			else if(Alu_op == I_JUMP)begin
				if(funct3 == 4'h0)begin
					result <= PC + 4;
					PC <= REG_FILE[rs1] + immediate;
				end
			end

			/* branching */
			else if(Alu_op == B_TYPE)begin
				// beq
				if(funct3 == 4'h0)begin
					PC <= $signed(REG_FILE[rs1]) == $signed(REG_FILE[rs2]) ? PC + immediate : PC + 4;
				end
				// bne
				else if(funct3 == 4'h1)begin
					PC <= $signed(REG_FILE[rs1]) != $signed(REG_FILE[rs2]) ? PC + immediate : PC + 4;
				end
				//blt
				else if(funct3 == 4'h4)begin
					PC <= $signed(REG_FILE[rs1]) < $signed(REG_FILE[rs2]) ? PC + immediate : PC + 4;
				end
				//bge
				else if(funct3 == 4'h5)begin
					PC <= $signed(REG_FILE[rs1]) >= $signed(REG_FILE[rs2]) ? PC + immediate : PC + 4;
				end
				//bltu
				else if(funct3 == 4'h6)begin
					PC <= REG_FILE[rs1] < REG_FILE[rs2] ? PC + immediate : PC + 4;
				end
				//bgeu
				else if(funct3 == 4'h7)begin
					PC <= REG_FILE[rs1] >= REG_FILE[rs2] ? PC + immediate : PC + 4;
				end
			end

			/************* u type ****************/
			/* lui */
			else if(Alu_op == U_LD)begin
				result <= immediate;
			end
			
			/* auipc */
			else if(Alu_op == U_PC)begin
				result <= PC + (immediate);
			end
			/************* u type ****************/

			/* jump type */
			else if(Alu_op == J_TYPE)begin
				result <= PC + 4;
				PC <= PC + immediate;
			end

			/* loading instructions */
			else if(Alu_op == I_LD) begin
				// load byte
				if(funct3 == 4'h0)begin
					ldst_addr <= ($signed(REG_FILE[rs1]) + immediate);
				end

				// load half
				else if(funct3 == 4'h1) begin
					ldst_addr <= ($signed(REG_FILE[rs1]) + immediate);
				end

				// load word
				else if(funct3 == 4'h2) begin
					ldst_addr <= ($signed(REG_FILE[rs1]) + immediate);
				end

				// load byte (U)
				else if(funct3 == 4'h4) begin
					ldst_addr <= (REG_FILE[rs1] + immediate);
				end

				// Load Half
				else if(funct3 == 4'h5)begin
					ldst_addr <= (REG_FILE[rs1] + immediate);
				end
			end

			/* Store */
			else if(Alu_op == S_TYPE)begin
				// store byte
				if(funct3 == 4'h0)begin
					ldst_addr <= ($signed(REG_FILE[rs1]) + immediate);
					ldst_wrdata <= $signed(REG_FILE[rs2][7:0]);
				end

				//store half
				else if(funct3 == 4'h1)begin
					ldst_addr <= $signed(REG_FILE[rs1]) + immediate;
					ldst_wrdata <= $signed(REG_FILE[rs2][15:0]);
				end

				//store word
				else if(funct3 == 4'h2)begin
					ldst_addr <= ($signed(REG_FILE[rs1]) + immediate);
					ldst_wrdata <= $signed(REG_FILE[rs2]);
				end
			end
		end
	end

	/* load result into reg file */
	always_ff @(posedge clk) begin : Reg_file_load
		if(resultIn) begin
			if(rd == 5'b0)begin
				REG_FILE[rd] <= 32'b0;
			end
			else begin
				REG_FILE[rd] <= result;
			end
		end
	end

	/* load value from memory into reg file */
	always_ff @(posedge clk) begin : Load
		if(loadIn) begin
			case(funct3)
				//load byte
				4'h0: begin
					REG_FILE[rd] <= $signed(ldst_rddata[7:0]);
				end
				//load half
				4'h1: begin
					REG_FILE[rd] <= $signed(ldst_rddata[15:0]);
				end
				// load word
				4'h2: begin
					REG_FILE[rd] <= $signed(ldst_rddata);
				end
				// load byte (U)
				4'h4: begin
					REG_FILE[rd] <= ldst_rddata[7:0];
				end
				//load half (U)
				4'h5: begin
					REG_FILE[rd] <= ldst_rddata[15:0];
				end
			endcase 
		end
	end
	/***************************************######### RISC V DATAPATH ############***************************************/

	/* outputs to the Processor Signal Interface */
	assign o_pc_rd = fetch;
	assign o_tb_regs = REG_FILE;
	assign o_pc_addr = PC;
	assign o_pc_byte_en = pc_byte_en;
	assign o_ldst_addr = ldst_addr;
	assign o_ldst_rd = ldst_rd;
	assign o_ldst_wr = ldst_wr;
	assign o_ldst_byte_en = ldst_byte_en;
	assign o_ldst_wrdata = ldst_wrdata;
endmodule