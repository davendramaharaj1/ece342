module part3
(
    input                       clk,
    input				        reset
);

	
	logic wait_request;							/* logic connect for waitrequest */
	logic [31:0] REGISTER_FILE [0:31]; 			/* logic connect to avoid a floating reg file output from cpu */
	logic [3:0] pc_byte_en, ldst_byte_en;
	logic pc_memory_read, pc_read, mem_read, avs_read, cpu_read, cpu_write;
	logic mem_write, avs_write;
	logic [2:0] avs_address;
	logic [31:0] pc_instruction, pc_byte_address, pc_word_address, ldst_address, memory_address, cpu_writedata, avs_writedata, mem_writedata;
	logic [31:0] cpu_readdata, mem_readdata, avs_readdata;
	
	localparam OP1 = 3'd0, OP2 = 3'd1, S = 3'd2, RESULT = 3'd3, STATUS = 3'd4;
	localparam OP1_addr = 16'hA000, OP2_addr = 16'hA004, S_addr = 16'hA008, RESULT_addr = 16'hA00C, STATUS_addr = 16'hA010;

	assign pc_word_address = pc_byte_address >> 2;
	assign pc_memory_read = pc_read | pc_instruction != 32'b0;

	/* read signal to memory */
	assign mem_read = cpu_read && ldst_address[15:12] != 4'hA;
	/* read signal to avs */
	assign avs_read = cpu_read && ldst_address[15:12] == 4'hA;
	/* write signal to memory */
	assign mem_write = cpu_write && ldst_address[15:12] != 4'hA;
	/* write signal to avs */
	assign avs_write = cpu_write && ldst_address[15:12] == 4'hA;

	/* decoder for addressing to memory */
	//assign memory_address = ldst_address[15:12] != 4'hA ? (ldst_address[12:0] >> 2) : 13'b0;
	assign memory_address = ldst_address >> 2;

	/* decoder for addressing to avs */
	always_comb begin : decoder_avs_address
		if(avs_read || avs_write) begin
			case (ldst_address[15:0])
				OP1_addr	: avs_address = OP1;
				OP2_addr	: avs_address = OP2;
				S_addr		: avs_address = S;
				RESULT_addr : avs_address = RESULT;
				STATUS_addr : avs_address = STATUS;
				//default: 
			endcase
		end
	end

	/* write_data to avs or memory */
	always_comb begin : WriteData
		if(avs_write)begin
			avs_writedata <= cpu_writedata;
		end
		else if (mem_write) begin
			mem_writedata <= cpu_writedata;
		end
	end

	/* readdata to avs or memory */
	always_comb begin : ReadData
		if(avs_read) begin
			cpu_readdata <= avs_readdata;
		end
		else if(mem_read) begin
			cpu_readdata <= mem_readdata;
		end
	end

	/* instantiation of avalon fp_mult to interaface with the fp peripheral */
	avalon_fp_mult fp_peripheral(
		.clk(clk),
		.reset(reset),
		
		// Avalon Slave
		.avs_s1_address(avs_address),
		.avs_s1_read(avs_read),
		.avs_s1_write(avs_write),
		.avs_s1_writedata(avs_writedata),
		.avs_s1_readdata(avs_readdata),
		.avs_s1_waitrequest(wait_request)
	);

	/* RISC-V Instantiation */
	cpu#(.IW(32), .REGS(32)) cpu(

		.clk(clk),
		.reset(reset),

		//Read only port 
		.o_pc_addr(pc_byte_address),
		.o_pc_rd(pc_read),
		.o_pc_byte_en(pc_byte_en),
		.i_pc_rddata(pc_instruction),

		// read/write port connecting to read/write port on 32KB RAM or LED
		.o_ldst_addr(ldst_address),
		.o_ldst_rd(cpu_read),
		.o_ldst_wr(cpu_write),
		.i_ldst_rddata(cpu_readdata),
		.o_ldst_wrdata(cpu_writedata),
		.o_ldst_byte_en(ldst_byte_en),
		.i_ldst_waitrequest(wait_request),

		// testbench reg file --> not used in lab 6 so can safely ignore 
		.o_tb_regs(REGISTER_FILE)
	);

	/* RAM instantiation */
	mem#(.WIDTH(32), .DEPTH(8192), .HEX_FILE("part3.hex")) mem_inst(

		.clk(clk),
		.reset(reset),

		/* Read only port */
		.p2_addr(pc_word_address[12:0]),
		.p2_read(pc_memory_read),
		.p2_byteenable(pc_byte_en),
		.p2_readdata(pc_instruction),

		/* Read/Write Port */
		.p1_addr(memory_address[12:0]),
		.p1_read(mem_read),
		.p1_write(mem_write),
		.p1_readdata(mem_readdata),
		.p1_writedata(mem_writedata),
		.p1_byteenable(ldst_byte_en)
	);

endmodule

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
	logic [IW-1:0] PC;	// holds address to the next instruction
	logic [3:0] pc_byte_en;	// size of pc to read : word/half-word/byte
	logic decode;		// decode the IR instruction
	logic fetch;		// control signal to fetch instruction from memory
	logic pc_increment;		// control signal to increment pc 

	/* Register File Registers and breakdown logic from IR */
	logic [4:0] rs1, rs2, rd;
	logic [2:0] funct3;
	logic [6:0] funct7;
	logic [IW-1:0] REG_FILE [0:REGS-1];

	/* ALU Registers and signals */
	logic [3:0] Alu_op;
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
	

	/* define states to represent stages in processing instructions */
	enum int unsigned {
		// INIT,
		FETCH,
		DECODE,
		EXECUTE,
		MEM_ACCESS,
		DEST_REG
	} state, nextstate;

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
	
	/***************************************########### RISC V FSM ###############***************************************/

	/* Control FSM Flip Flops */
	integer i;
	always_ff @(posedge clk or posedge reset) begin : FSMTransition
		if(reset) begin
			/* set the PC to point to the first instruction */
			PC <= 32'b0;
			/* Ensure the register file is zeroed */
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
		resultIn = 1'b0;
		ldst_rd = 1'b0;
		ldst_wr = 1'b0;
		ldst_byte_en = 4'b1111;
		loadIn = 1'b0;
		nextstate = state;

		case(state)
			// INIT:begin
			// 	nextstate = FETCH;
			// end
			FETCH: begin
				/* send signal to processor interface to read the pc address */
				fetch = 1'b1;
				/* get the entire word for PC */
				pc_byte_en = 4'b1111;
				/* increment the PC */
				//pc_increment = 1'b1;
				/* next state transition */
				nextstate = DECODE;
			end

			DECODE:begin
				IR = i_pc_rddata;
				/* signal to decode instruction and load appropriate registers */ 
				decode = 1'b1;
				nextstate = EXECUTE;
			end

			EXECUTE: begin
				/* enable alu to perform appropriate operation */
				Alu_en = 1'b1;
				/* next state depends on the ALU operation */
				case (Alu_op)
					R_TYPE, I_IMM, I_JUMP, U_LD, U_PC, J_TYPE: begin
						/* non memory instructions so store result into reg file */
						nextstate = DEST_REG;
					end
					B_TYPE: begin
						/* fetch another instruction from the branched PC */
						//pc_increment = 1'b1;
						nextstate = FETCH;
					end
					I_LD, S_TYPE: begin
						/* wait to get the memory accessed target */
						nextstate = MEM_ACCESS;
					end
				endcase
			end

			MEM_ACCESS: begin
				// load instruction
				if(Alu_op == I_LD)begin
					//load byte
					if(funct3 == 4'h0)begin
						ldst_rd = 1'b1;
						ldst_byte_en = 4'b0001;
						//nextstate = DEST_REG;
					end
					// load half word
					else if(funct3 == 4'h1)begin
						ldst_rd = 1'b1;
						ldst_byte_en = 4'b0011;
						//nextstate = DEST_REG;
					end
					// load word
					else if(funct3 == 4'h2)begin
						ldst_rd = 1'b1;
						ldst_byte_en = 4'b1111;
						//nextstate = DEST_REG;
					end
					// load byte unsigned
					else if(funct3 == 4'h4)begin
						ldst_rd = 1'b1;
						ldst_byte_en = 4'b0001;
						//nextstate = DEST_REG;
					end		
					//load half unsigned
					else if(funct3 == 4'h5)begin
						ldst_rd = 1'b1;
						ldst_byte_en = 4'b0011;
						//nextstate = DEST_REG;
					end	
					loadIn = 1'b1;	
				end
				//store instruction
				else if(Alu_op == S_TYPE)begin
					// store byte
					if(funct3 == 4'h0) begin
						ldst_wr = 1'b1;
						ldst_byte_en = 4'b0001;
						//nextstate = DEST_REG;
					end
					// store half
					else if(funct3 == 4'h1) begin
						ldst_wr = 1'b1;
						ldst_byte_en = 4'b0011;
						//nextstate = DEST_REG;
					end
					// store word
					if(funct3 == 4'h2) begin
						ldst_wr = 1'b1;
						ldst_byte_en = 4'b1111;
						//nextstate = DEST_REG;
					end
				end
				if(i_ldst_waitrequest == 1'b1)begin
					nextstate = MEM_ACCESS;
				end
				if(i_ldst_waitrequest == 1'b0)begin
					nextstate = DEST_REG;
				end
			end

			DEST_REG: begin
				case (Alu_op)
					// write result to corresponsding register_file[rd] for R-type, U-type, J-type, I_imm/jump-type
					R_TYPE, I_IMM, U_LD, U_PC: begin
						pc_increment = 1'b1;
						resultIn = 1'b1;
						/* non memory instructions so store result into reg file */
						nextstate = FETCH;
					end
					I_JUMP, J_TYPE: begin
						resultIn = 1'b1;
						/* non memory instructions so store result into reg file */
						nextstate = FETCH;
					end
					I_LD: begin
						/* copy value loaded from memory into register file*/
						pc_increment = 1'b1;
						nextstate = FETCH;
					end
					S_TYPE:begin
						pc_increment = 1'b1;
						nextstate = FETCH;
					end
				endcase 
			end
		endcase

	end

	/***************************************########## RISC V FSM ################***************************************/


	/***************************************######### RISC V DATAPATH #############***************************************/

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

	/* increment PC by 4 from the control signal */
	always_ff @(posedge clk or posedge reset) begin : PC_Increment
		if(reset)begin
			PC <= 32'b0;
		end
		else if(pc_increment) begin
			PC <= PC + 4;
		end
	end

	/* Get the ALU Op code to know which ALU Operation to perform */
	always_comb begin : ALUOP
		case (IR[6:0])
			R: 		Alu_op = R_TYPE;
			I_imm:	Alu_op = I_IMM;
			I_ld: 	Alu_op = I_LD;
			I_jump: Alu_op = I_JUMP;
			S: 		Alu_op = S_TYPE;
			B:		Alu_op = B_TYPE;
			U_ld:	Alu_op = U_LD;
			U_pc:	Alu_op = U_PC;
			J:		Alu_op = J_TYPE; 
		endcase
	end

	/* ALU logic */
	always_ff @(posedge clk) begin : ALU_logic
		if(Alu_en) begin
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
	/***************************************########## RISC V DATAPATH ############***************************************/

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

module avalon_slave
(
	input clk,
	input reset,
	
	// Avalon Slave
	input [2:0] avs_s1_address,
	input avs_s1_read,
	input avs_s1_write,
	input [31:0] avs_s1_writedata,
	output logic [31:0] avs_s1_readdata,
	output logic avs_s1_waitrequest,

	// FP_MULT connections
    output [31:0] input_1,
	output [31:0] input_2,
	output clk_en,
	input [31:0] result,
	input [3:0] flags
);

// Mem-mapped regs
// Reg0-32:			A
// Reg1-32:			B
// Reg2-04:			Start/Busy
// Reg3-32:			Result
// Reg4-04:			Status (Flags)
reg [31:0] mul_op_1;
reg [31:0] mul_op_2;
reg [31:0] mul_start;
reg [31:0] mul_result;
reg [31:0] mul_status;

// control-dp connectors
logic done, ld_op1, ld_op2, ld_start, ld_status, ld_count, ld_result, reset_count, enable;
logic [2:0] mmreg;
logic [31:0] op1, op2, s, d_result, status;

// STUDENTS TO ADD THESE
control_path ctrl
(
	.clk(clk),
	.reset(reset),
	.read(avs_s1_read),
	.write(avs_s1_write),
	.address(avs_s1_address),
	.waitrequest(avs_s1_waitrequest),
	.done(done),
	.ld_op1(ld_op1),
	.ld_op2(ld_op2),
	.ld_start(ld_start),
	.ld_result(ld_result),
	.ld_status(ld_status),
	.ld_count(ld_count),
	.mmreg(mmreg),
	.count_reset(reset_count),
	.enable(enable)
);

datapath dp
(
	.clk(clk),
	.reset(reset),
	.readdata(avs_s1_readdata),
	.writedata(avs_s1_writedata),
	.fpm_result(result),
	.fpm_status(flags),
	.ld_op1(ld_op1),
	.ld_op2(ld_op2),
	.ld_start(ld_start),
	.ld_result(ld_result),
	.ld_status(ld_status),
	.ld_count(ld_count),
	.count_reset(reset_count),
	.mm_reg(mmreg),
	.done(done),
	.op1(op1),
	.op2(op2),
	.s(s),
	.result(d_result),
	.status(status)
);

always_comb begin : MM_REGS
	mul_op_1 = op1;
	mul_op_2 = op2;
	mul_start = s;
	mul_result = d_result;
	mul_status = status;
end

// assign inputs from datatype to go to FPM
assign input_1 = op1;
assign input_2 = op2;
assign clk_en = enable;

endmodule

module control_path
(
	// from master controller 
	input clk,
	input reset,
	input read,
	input write, 
	input [2:0] address,
	output logic waitrequest,

	// from datapath
	input done,
	// to datapath
	output logic count_reset,

	// control signals to datapath
	output logic ld_op1,
	output logic ld_op2,
	output logic ld_start,
	output logic ld_result,
	output logic ld_status,
	output logic ld_count,
	output logic [2:0] mmreg,
	//output logic reset_count,

	// control signal to FPM to begin multiplication, stop when done 
	output logic enable
);

	// define the states
	enum int unsigned{
		S_INIT,
		S_RUN,
		S_WAIT
	} state, nextstate;

	// internal logic bus
	logic [2:0] mm_reg;

	// define the memory-mapped registers
	localparam [2:0] OP1 = 3'd0;
	localparam [2:0] OP2 = 3'd1;
	localparam [2:0] S = 3'd2;
	localparam [2:0] RESULT = 3'd3;
	localparam [2:0] STATUS = 3'd4;

	// states
	always_ff @(posedge clk or posedge reset) begin : flip_flops 
		if(reset) begin
			state <= S_INIT;
		end
		else begin
			state <= nextstate;
		end
	end

	// state transition table
	always_comb begin : state_table 

		// prevent latches
		waitrequest = 1'b0;
		ld_start = 1'b0;
		ld_status = 1'b0;
		ld_count = 1'b0;
		ld_result = 1'b0;
		ld_op1 = 1'b0;
		ld_op2 = 1'b0;
		count_reset = 1'b0;
		nextstate = state;
		enable = 1'b0;

		case(state)
			S_INIT: begin
				if(write) begin
					case(mm_reg)
						// if master wants to write to OP1
						OP1: 	ld_op1 = 1'b1;
						// master wants to write to OP2
						OP2: 	ld_op2 = 1'b1;
						// if the master wants to start multiplcation
						S:begin
							ld_start = 1'b1;
							enable = 1'b1;
							waitrequest = 1'b1;
							nextstate = S_RUN;
						end
						//default: // do nothing		
					endcase
				end
			end

			S_RUN: begin
				// if not done, keep counting
				if(!done) begin
					ld_count = 1;
					ld_start = 1;
					enable = 1;
					waitrequest = 1;
				end
				// if done, waitrequest is zero, load the result registers and status
				else begin
					waitrequest = 0;
					ld_result = 1;
					ld_status = 1;
					count_reset = 1;
					nextstate = S_WAIT;
				end
			end

			S_WAIT:begin
				nextstate = S_INIT;
			end
		endcase 
	end

	// always block assigning memory mapped addresses
	always_comb begin : control_signals
		// address decoder
		case(address)
			3'd0: mm_reg = OP1;
			3'd1: mm_reg = OP2;
			3'd2: mm_reg = S;
			3'd3: mm_reg = RESULT;
			3'd4: mm_reg = STATUS;
			//default: // do nothing
		endcase

		// if master reads, get the MM register to be read from
		if(read) begin
			mmreg = mm_reg;
		end
	end


endmodule

module datapath
(
	input clk,
	input reset,

	// to avs
	output logic [31:0] readdata,
	//from avs
	input [31:0] writedata,

	// from multipler
	input [31:0] fpm_result,
	input [3:0] fpm_status,

	// control signals from controller
	input ld_op1,
	input ld_op2,
	input ld_start,
	input ld_result,
	input ld_status,
	input ld_count,
	input count_reset,
	input [2:0] mm_reg,

	//output to control path
	output  done,

	// output to Master Avalon
	output   [31:0] op1,
	output   [31:0] op2,
	output   [31:0] s,
	output   [31:0] result,
	output   [31:0] status
);
	// create memory mapped IO
	logic [4:0][31:0] mm;
	
	// create count variable
	logic [3:0] count;

	assign op1 	= mm[0];
	assign op2 	= mm[1];
	assign s   	= mm[2];
	assign result	= mm[3];
	assign status	= mm[4];
	
	//always_comb begin: mm_reg_allocation
		//case(mm_reg) 
		//	3'd0:  readdata = mm[0];
		//	3'd1:  readdata = mm[1];
		//	3'd2:  readdata = mm[2];
		//	3'd3:  readdata = mm[3];
		//	3'd4:  readdata = mm[4];
	//	endcase
	//end
	always_comb begin
		if(mm_reg == 3'd0) readdata = mm[0];
		if(mm_reg == 3'd1) readdata = mm[1];
		if(mm_reg == 3'd2) readdata = mm[2];
		if(mm_reg == 3'd3) readdata = mm[3];
		if(mm_reg == 3'd4) readdata = mm[4];
	end

	// writing in op1, op2, assigning result and status sequential logic
	always_ff @(posedge clk or posedge reset ) begin : assign_mm
		if(reset) begin
			mm[0] = 32'b0;
			mm[1] = 32'b0;
			mm[2] = 32'b0;
			mm[3] = 32'b0;
			mm[4] = 32'b0;
		end
		else begin
			if(ld_op1) mm[0] <= writedata;
			if(ld_op2) mm[1] <= writedata;
			if(ld_result) mm[3] <= fpm_result;
			if(ld_status) mm[4] <= fpm_status;

			if(ld_start) mm[2] <= {31'b0, 1'b1};
			if(!ld_start) mm[2] <= {32'b0};
		end
	end

	// initiate counting
	always_ff @( posedge clk or posedge reset  ) begin : counting
		if(reset) begin
			count <= 11;
		end
		else if (count_reset) begin
			count <= 11;
		end
		else if (ld_count)begin
			count <= count - 1;
		end
	end

	assign done = count == 0 ? 1 : 0;
endmodule


