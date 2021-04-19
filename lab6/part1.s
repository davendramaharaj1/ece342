
#############################
main:
    lui s2, 0xA
    addi s2, s2, 0x10 
    lui s3, 0xA 

loop:
    lb s4, s2, 0x0
    sb s3, s4, 0x0

    jal zero, loop
