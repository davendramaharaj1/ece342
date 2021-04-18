module part1 
(
    input                       clk,
    input				        reset,
    input       [ 7: 0]         SW,
    output      [ 7: 0]         LEDR
);

// logic wires to connect the memory, cpu and sw/ledr registers together
logic [3:0] pc_byte_enable;
logic pc_read;
logic [31:0] instruction, pc_address;
logic [12:0] memory_addr;

/* decoder for processor to memory (accessing instructions) */
assign memory_address = (pc_address >> 2)[12:0];

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
    .o_ldst_addr(),
    .o_ldst_rd(),
    .o_ldst_wr(),
    .i_ldst_rddata(),
    .o_ldst_wrdata(),
    .o_ldst_byte_en(),

    // testbench reg file --> not used in lab 6 so can safely ignore 
    .o_tb_regs()
);

/* Instantiate 32KB memory module */
mem#(.WIDTH(32), .DEPTH(8192), .HEX_FILE("part1.hex")) memory(

    .clk(clk),
    .reset(reset),

     /* Read only port */
    .p2_addr(memory_address),
    .p2_read(pc_read),
    .p2_byteenable(pc_byte_enable),
    .p2_readdata(instruction),

    /* Read/Write Port */
    .p1_addr(),
    .p1_read(),
    .p1_write(),
    .p1_readdata(),
    .p1_writedata(),
    .p1_byteenable()
);

endmodule
