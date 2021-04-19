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
logic [12:0] memory_addr;

/* Logic wires for read/write port */
logic read, write, write_led, write_mem;
logic byte_en;
logic [12:0] data_address;
logic [31:0] mem_ldst_address, i_readdata, mem_readdata, write_data;

/* decoder for processor to memory (accessing instructions) */
assign memory_address = (pc_address >> 2)[12:0];

/* decoder for inputing memory data or switch data into cpu */
assign data_address = mem_ldst_address[15:12] != 4'hA ? (mem_ldst_address >> 2)[12:0] : 13'b0;
assign sel_data = (read && mem_ldst_address[15:12] != 4'hA);    // if last byte is not A and read = 1, load data from memory
assign i_readdata = sel_data ? mem_readdata : switch;           // 2:1 mux to select data from memory or switches

/* decoder for writing data to ledr reg or memory */
assign write_mem = (write && mem_ldst_address[15:12] != 4'hA);
assign write_led = (write && mem_ldst_address[15:0] == 16'hA000);

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
    .p2_read(pc_read),
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
