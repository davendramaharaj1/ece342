module part1 
(
    input                       clk,
    input				        reset,
    input       [ 7: 0]         SW,
    output      [ 7: 0]         LEDR
);

/* instantiate RISC-V CPU from Lab 5 */
cpu#(.IW(32), .REGS(32)) cpu(

    .clk(),
    .reset(),

    // read only port connecting to read only port on 32KB RAM or SW
    .o_pc_addr(),
    .o_pc_rd(),
    .i_pc_rddata(),
    .o_pc_byte_en(),

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

endmodule
