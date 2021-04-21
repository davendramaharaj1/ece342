
main:
    #s2 <- 0x00007ff4 (mem_inst base)
    lui s2, 0x7ff4
    srli s2, s2, 0xC

    #s5 <- 0x0000A000 (avs memory_mapped base)
    lui s5, 0xA

    #s11 <- 0x00000001 (has constant value 1)
    addi s11, zero, 0x1

loop:
    # load word op1 from memory op1
    lw s3, s2, 0x0
    # load word op2 from memory op2
    lw s4, s2, 0x4
    # store word op1 into avs op1
    sw s5, s3, 0x0
    # store word op2 into avs op2
    sw s5, s4, 0x4
    # write 1 to avs S register to begin multiplication
    sb s5, s11, 0x8
    # try to load result from the fp multiplier
    lw s6, s5, 0xC
    # write result in s6 to memory result register
    sw s2, s6, 0x8
    # loop to begin another multiplcation
    jal zero, loop, 0x0