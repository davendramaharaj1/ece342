module part1 
(
    input                       clk,
    input				        reset,
    input       [ 7: 0]         SW,
    output      [ 7: 0]         LEDR
);


/* just connect to the reg file output to avoid a floating wire */
logic [31:0] REG_FILE [0:31];

/* create a register to hold switch and led values */
logic [31:0] switch;
logic [7:0] led;

/* selector signals in mux */
logic sel_data;

// logic wires to connect Mmeory Read only port
logic [3:0] pc_byte_enable;
logic pc_read;
logic [31:0] instruction, pc_address;
logic [12:0] memory_address;

/* Logic wires for read/write port */
logic read, write, write_led, write_mem;
logic[3:0] byte_en;
logic [12:0] data_address;
logic [31:0] mem_ldst_address, i_readdata, mem_readdata, write_data;

/* decoder for processor to memory (accessing instructions) */
assign memory_address = pc_address[12:0] >> 2;

/* decoder for inputing memory data or switch data into cpu */
assign data_address = mem_ldst_address[15:12] != 4'hA ? (mem_ldst_address[12:0] >> 2) : 13'b0;
assign sel_data = (read && mem_ldst_address[15:12] != 4'hA);    // if last byte is not A and read = 1, load data from memory
assign i_readdata = sel_data ? mem_readdata : switch;           // 2:1 mux to select data from memory or switches

/* decoder for writing data to ledr reg or memory */
assign write_mem = (write && mem_ldst_address[15:12] != 4'hA);
assign write_led = (write && mem_ldst_address[15:0] == 16'hA000);

logic mem_pc_read;

assign mem_pc_read = pc_read | (instruction != 32'b0);

/* instantiate RISC-V CPU from Lab 5 */
cpu#(.IW(32), .REGS(32)) cpu(

    .clk(clk),
    .reset(reset),

    // read only port for Instructions
    .o_pc_addr(pc_address),
    .o_pc_rd(pc_read),
    .o_pc_byte_en(pc_byte_enable),
    .i_pc_rddata(instruction),

    // read/write port connecting to read/write port on 32KB RAM or LED
    .o_ldst_addr(mem_ldst_address),
    .o_ldst_rd(read),
    .o_ldst_wr(write),
    .i_ldst_rddata(i_readdata),
    .o_ldst_wrdata(write_data),
    .o_ldst_byte_en(byte_en),

    // testbench reg file --> not used in lab 6 so can safely ignore 
    .o_tb_regs(REG_FILE)
);

/* Instantiate 32KB memory module */
mem#(.WIDTH(32), .DEPTH(8192), .HEX_FILE("part1.hex")) memory(

    .clk(clk),
    .reset(reset),

     /* Read only port */
    .p2_addr(memory_address),
    .p2_read(mem_pc_read),
    .p2_byteenable(pc_byte_enable),
    .p2_readdata(instruction),

    /* Read/Write Port */
    .p1_addr(data_address),
    .p1_read(read),
    .p1_write(write_mem),
    .p1_readdata(mem_readdata),
    .p1_writedata(write_data),
    .p1_byteenable(byte_en)
);

/* SW Register to hold the switch values */
always_ff @(posedge clk) begin : SW_Reg
    if(reset)begin
        switch <= 32'b0;
    end
    else begin
        switch <= {24'b0, SW};
    end
end

/* ledr register */
always_ff @( posedge clk ) begin : LEDR_Reg
    if(reset)begin
        led <= 8'b0;
    end
    else if(write_led) begin
        led <= write_data[7:0];
    end
end

assign LEDR = led;

endmodule


/* processor code for RISC-V*/
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
				nextstate = DEST_REG;
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
