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
	logic [IW-1:0] IR;	// instruction register
	logic [IW-1:0] immediate; // holds immediate value --> extended to 32 bits (if arithemtic-op then signed else zero-padded)

	/* Address Generator Registers and signals  */
	logic [IW-1:0] PC_1, PC_2;	// holds address to the next instruction
	logic [3:0] pc_byte_en;	// size of pc to read : word/half-word/byte

	/* Register File Registers and breakdown logic from IR */
	logic [4:0] rs1, rs2, rd, rd_stage4;
	logic [2:0] funct3, funct3_stage4;
	logic [6:0] funct7;
	logic [IW-1:0] REG_FILE [0:REGS-1];

	/* ALU Registers and signals */
	logic [6:0] Alu_op, opcode;
	logic [IW-1:0] result;

	/* Load/Store reg control signals */
	logic [IW-1:0] ldst_addr;
	logic [IW-1:0] ldst_rddata;
	logic [IW-1:0] ldst_wrdata;
	logic [3:0] ldst_byte_en;
	logic ldst_rd, pc_read;
	logic ldst_wr;


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
	
	integer i;

	assign IR = i_pc_rddata;

	/* outputs to the Processor Signal Interface */
	assign o_tb_regs = REG_FILE;
	assign o_ldst_addr = ldst_addr;
	assign o_ldst_rd = ldst_rd;
	assign o_ldst_wr = ldst_wr;
	assign o_ldst_byte_en = ldst_byte_en;
	assign o_ldst_wrdata = ldst_wrdata;		

	always_comb begin : readPC
		if(stage1 & !reset)begin
			pc_read = 1'b1;
		end
		else begin
			pc_read = 1'b0;
		end
	end

	assign o_pc_rd = pc_read;
	assign o_pc_addr = PC_1;
	assign o_pc_byte_en = 4'b1111;

	/***************************************########## RISC V CONTROL PATH #########***************************************/
	/* Control path for pipelined stages */
	always_ff @(posedge clk) begin : PipelinedStages
		/************ Control Reset ***********/
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

			result <= 0;
		end
		
		else begin
			/* STAGE 1: FETCH */
			if(stage1) begin
				/* increment pc */
				PC_1 <= PC_1 + 4;
				PC_2 <= PC_1;
				/* move to decode stage */
				stage2 <= 1'b1;
			end

			/* STAGE 2: DECODE */
			if(stage2) begin
				case (IR[6:0])
					/* R Type instruction */
					R: begin
						rs1 <= IR[19:15];
						rs2 <= IR[24:20];
						rd <= IR[11:7];
						funct3 <= IR[14:12];
						funct7 <= IR[31:25];
						Alu_op <= IR[6:0];
					end
					/* I Type instruction */
					I_imm, I_ld, I_jump: begin
						rs1 <= IR[19:15];
						rd <= IR[11:7];
						funct3 <= IR[14:12];
						immediate <= {{21{IR[31]}},IR[30:20]};
						Alu_op <= IR[6:0];
					end
					/* S Type instruction */
					S: begin
						rs1 <= IR[19:15];
						rs2 <= IR[24:20];
						funct3 <= IR[14:12];
						immediate <= {{21{IR[31]}},IR[30:25],IR[11:7]};
						Alu_op <= IR[6:0];
					end
					/* B Type instruction */
					B: begin
						rs1 <= IR[19:15];
						rs2 <= IR[24:20];
						funct3 <= IR[14:12];
						immediate <= {{20{IR[31]}},IR[7],IR[30:25],IR[11:8],1'b0};
						Alu_op <= IR[6:0];
					end
					/* U Type instruction */
					U_ld, U_pc: begin
						rd <= IR[11:7];
						immediate <= {IR[31:12],12'b0};
						Alu_op <= IR[6:0];
					end
					/* J Type instruction */
					J: begin
						rd <= IR[11:7];
						immediate <= {{12{IR[31]}},IR[19:12],IR[20],IR[30:21],1'b0};
						Alu_op <= IR[6:0];
					end
				endcase

				stage3 <= 1'b1;
			end

			/* STAGE 3: EXECUTE */
			if(stage3) begin
				rd_stage4 <= rd;
				opcode <= Alu_op;
				funct3_stage4 <= funct3;

				/* registers as operands for arithmetic */
				if(Alu_op == R) begin
					// add
					if(funct3 == 4'h0 && funct7 == 8'h00) begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result + REG_FILE[rs2];
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							result <= result + REG_FILE[rs1];
						end
						else begin
							result <= REG_FILE[rs1] + REG_FILE[rs2];
						end
					end
					// sub
					else if (funct3 == 4'h0 && funct7 == 8'h20) begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result - REG_FILE[rs2];
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							result <= REG_FILE[rs1] - result;
						end
						else begin
							result <= REG_FILE[rs1] - REG_FILE[rs2];
						end
					end
					// xor
					else if(funct3 == 4'h4 && funct7 == 8'h00)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result ^ REG_FILE[rs2];
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							result <= REG_FILE[rs1] ^ result;
						end
						else begin
							result <= REG_FILE[rs1] ^ REG_FILE[rs2];
						end
					end
					// or
					else if(funct3 == 4'h6 && funct7 == 8'h00)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result | REG_FILE[rs2];
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							result <= REG_FILE[rs1] | result;
						end
						else begin
							result <= REG_FILE[rs1] | REG_FILE[rs2];
						end
					end
					// and
					else if(funct3 == 4'h7 && funct7 == 8'h00)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result & REG_FILE[rs2];
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							result <= REG_FILE[rs1] & result;
						end
						else begin
							result <= REG_FILE[rs1] & REG_FILE[rs2];
						end
					end
					//sll
					else if(funct3 == 4'h1 && funct7 == 8'h00)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result << REG_FILE[rs2];
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							result <= REG_FILE[rs1] << result;
						end
						else begin
							result <= REG_FILE[rs1] << REG_FILE[rs2];
						end
					end
					//srl
					else if(funct3 == 4'h5 && funct7 == 8'h00)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result >> REG_FILE[rs2];
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							result <= REG_FILE[rs1] >> result;
						end
						else begin
							result <= REG_FILE[rs1] >> REG_FILE[rs2];
						end
					end
					//sra
					else if(funct3 == 4'h5 && funct7 == 8'h20)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= $signed(result) >>> REG_FILE[rs2][4:0];
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							result <= $signed(REG_FILE[rs1]) >>> result[4:0];
						end
						else begin
							result <= $signed(REG_FILE[rs1]) >>> REG_FILE[rs2][4:0];
						end
					end
					//slt
					else if(funct3 == 4'h2 && funct7 == 8'h00)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= $signed(result) < $signed(REG_FILE[rs2]) ? 1 : 0;
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							result <= $signed(REG_FILE[rs1]) < $signed(result) ? 1 : 0;
						end
						else begin
							result <= $signed(REG_FILE[rs1]) < $signed(REG_FILE[rs2]) ? 1 : 0;
						end
					end
					//sltu
					else if(funct3 == 4'h3 && funct7 == 8'h00)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result < REG_FILE[rs2] ? 1 : 0;
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							result <= REG_FILE[rs1] < result ? 1 : 0;
						end
						else begin
							result <= REG_FILE[rs1] < REG_FILE[rs2] ? 1 : 0;
						end
					end
					stage4 <= 1'b1;
				end

				/* arithmetic I type with register and immediate value */
				else if(Alu_op == I_imm)begin
					// addi
					if(funct3 == 4'h0) begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result + immediate;
						end
						else begin
							result <= REG_FILE[rs1] + immediate;
						end
					end
					// xori
					else if(funct3 == 4'h4)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result ^ immediate;
						end
						else begin
							result <= REG_FILE[rs1] ^ immediate;
						end
					end
					// ori
					else if(funct3 == 4'h6)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result | immediate;
						end
						else begin
							result <= REG_FILE[rs1] | immediate;
						end
					end
					// andi
					else if(funct3 == 4'h7)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result & immediate;
						end
						else begin
							result <= REG_FILE[rs1] & immediate;
						end
					end
					//slli
					else if(funct3 == 4'h1 && immediate[11:5] == 8'h00)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result << immediate[4:0];
						end
						else begin
							result <= REG_FILE[rs1] << immediate[4:0];
						end
					end
					//srli
					else if(funct3 == 4'h5 && immediate[11:5] == 8'h00)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result >> immediate[4:0];
						end
						else begin
							result <= REG_FILE[rs1] >> immediate[4:0];
						end
					end
					//srai
					else if(funct3 == 4'h5 && immediate[11:5] == 8'h20)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= $signed(result) >>> immediate[4:0];
						end
						else begin
							result <= $signed(REG_FILE[rs1]) >>> immediate[4:0];
						end
					end
					//slti
					else if(funct3 == 4'h2) begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= $signed(result) < $signed(immediate) ? 1 : 0;
						end
						else begin
							result <= $signed(REG_FILE[rs1]) < $signed(immediate) ? 1 : 0;
						end
					end
					//sltiu
					else if(funct3 == 4'h3 && funct7 == 8'h00)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							result <= result < immediate ? 1 : 0;
						end
						else begin
							result <= REG_FILE[rs1] < immediate ? 1 : 0;
						end
					end
					stage4 <= 1'b1;
				end

				/* arithmetic I type with register and PC (jalr) */
				else if(Alu_op == I_jump)begin
					if(funct3 == 4'h0)begin
						/* flush instructions at stages 2 & 3 */
						stage2 <= 1'b0;
						stage3 <= 1'b0;
						/* rd = PC + 4 */
						result <= PC_2;
						/* account for forwarding */
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= result + immediate;
							PC_2 <= result + immediate - 4;
						end
						else begin
							PC_1 <= REG_FILE[rs1] + immediate;
							PC_2 <= (REG_FILE[rs1] + immediate) - 4;
						end
					end
					stage4 <= 1'b1;
				end

				/* branching */
				else if(Alu_op == B)begin
					// beq
					if(funct3 == 4'h0)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= $signed(result) == $signed(REG_FILE[rs2]) ? PC_1 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(result) == $signed(REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= $signed(result) == $signed(REG_FILE[rs2]) ? 0 : stage3;
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= $signed(REG_FILE[rs1]) == $signed(result) ? PC_1 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(REG_FILE[rs1]) == $signed(result) ? 0 : stage2;
							stage3 <= $signed(REG_FILE[rs1]) == $signed(result) ? 0 : stage3;
						end
						else begin
							PC_1 <= $signed(REG_FILE[rs1]) == $signed(REG_FILE[rs2]) ? PC_1 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(REG_FILE[rs1]) == $signed(REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= $signed(REG_FILE[rs1]) == $signed(REG_FILE[rs2]) ? 0 : stage3;
						end
					end
					// bne
					else if(funct3 == 4'h1)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= $signed(result) != $signed(REG_FILE[rs2]) ? PC_2 - 8 + immediate : PC_2;
							stage2 <= $signed(result) != $signed(REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= $signed(result) != $signed(REG_FILE[rs2]) ? 0 : stage3;
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= $signed(REG_FILE[rs1]) != $signed(result) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(REG_FILE[rs1]) != $signed(result) ? 0 : stage2;
							stage3 <= $signed(REG_FILE[rs1]) != $signed(result) ? 0 : stage3;
						end
						else begin
							PC_1 <= $signed(REG_FILE[rs1]) != $signed(REG_FILE[rs2]) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(REG_FILE[rs1]) != $signed(REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= $signed(REG_FILE[rs1]) != $signed(REG_FILE[rs2]) ? 0 : stage3;
						end
					end
					//blt
					else if(funct3 == 4'h4)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= $signed(result) < $signed(REG_FILE[rs2]) ? PC_1 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(result) < $signed(REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= $signed(result) < $signed(REG_FILE[rs2]) ? 0 : stage3;
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= $signed(REG_FILE[rs1]) < $signed(result) ? PC_1 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(REG_FILE[rs1]) < $signed(result) ? 0 : stage2;
							stage3 <= $signed(REG_FILE[rs1]) < $signed(result) ? 0 : stage3;
						end
						else begin
							PC_1 <= $signed(REG_FILE[rs1]) < $signed(REG_FILE[rs2]) ? PC_1 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(REG_FILE[rs1]) < $signed(REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= $signed(REG_FILE[rs1]) < $signed(REG_FILE[rs2]) ? 0 : stage3;
						end
					end
					//bge
					else if(funct3 == 4'h5)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= $signed(result) >= $signed(REG_FILE[rs2]) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(result) >= $signed(REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= $signed(result) >= $signed(REG_FILE[rs2]) ? 0 : stage3;
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= $signed(REG_FILE[rs1]) >= $signed(result) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(REG_FILE[rs1]) >= $signed(result) ? 0 : stage2;
							stage3 <= $signed(REG_FILE[rs1]) >= $signed(result) ? 0 : stage3;
						end
						else begin
							PC_1 <= $signed(REG_FILE[rs1]) >= $signed(REG_FILE[rs2]) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= $signed(REG_FILE[rs1]) >= $signed(REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= $signed(REG_FILE[rs1]) >= $signed(REG_FILE[rs2]) ? 0 : stage3;
						end
					end
					//bltu
					else if(funct3 == 4'h6)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= (result) < (REG_FILE[rs2]) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= (result) < (REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= (result) < (REG_FILE[rs2]) ? 0 : stage3;
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= (REG_FILE[rs1]) < (result) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= (REG_FILE[rs1]) < (result) ? 0 : stage2;
							stage3 <= (REG_FILE[rs1]) < (result) ? 0 : stage3;
						end
						else begin
							PC_1 <= (REG_FILE[rs1]) < (REG_FILE[rs2]) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= (REG_FILE[rs1]) < (REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= (REG_FILE[rs1]) < (REG_FILE[rs2]) ? 0 : stage3;
						end
					end
					//bgeu
					else if(funct3 == 4'h7)begin
						if(rs1 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= (result) >= (REG_FILE[rs2]) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= (result) >= (REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= (result) >= (REG_FILE[rs2]) ? 0 : stage3;
						end
						else if(rs2 == rd_stage4 && rd_stage4 != 0)begin
							PC_1 <= (REG_FILE[rs1]) >= (result) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= (REG_FILE[rs1]) >= (result) ? 0 : stage2;
							stage3 <= (REG_FILE[rs1]) >= (result) ? 0 : stage3;
						end
						else begin
							PC_1 <= (REG_FILE[rs1]) >= (REG_FILE[rs2]) ? PC_2 - 8 + immediate : PC_1 + 4;
							stage2 <= (REG_FILE[rs1]) >= (REG_FILE[rs2]) ? 0 : stage2;
							stage3 <= (REG_FILE[rs1]) >= (REG_FILE[rs2]) ? 0 : stage3;
						end
					end
					stage4 <= 1'b0;
				end

				/************* u type ****************/
				/* lui */
				else if(Alu_op == U_ld)begin
					result <= immediate;
					stage4 <= 1'b1;
				end
				
				/* auipc */
				else if(Alu_op == U_pc)begin
					result <= (PC_2 - 4) + (immediate);
					stage4 <= 1'b1;
				end
				/************* u type ****************/

				/* jump type (jal) */
				else if(Alu_op == J)begin
					/* flush instructions at stages 2 & 3 */
					stage2 <= 1'b0;
					stage3 <= 1'b0;
					/* rd = PC + 4 */
					result <= PC_2;
					PC_1 <= PC_1 + immediate - 8;
					PC_2 <= PC_2 - 4 + immediate;
					stage4 <= 1'b1;
				end

				/* loading instructions */
				else if(Alu_op == I_ld) begin
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
					stage4 <= 1'b0;
				end

				/* Store */
				else if(Alu_op == S)begin
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
					stage4 <= 1'b0;
				end
			end

			/* STAGE 4: WRITE_BACK */
			if(stage4) begin
				if(rd_stage4 == 5'b0)begin
					REG_FILE[rd_stage4] <= 32'b0;
				end
				else begin
					REG_FILE[rd_stage4] <= result;
				end
			end
		end
	end
endmodule