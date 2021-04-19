lui s2, 0xa
addi s2, s2, 0x10   # s2 points to the switch address

lui s3, 0xa         # s3 points to the led address


loop:
LB s4, 0(s2)
SB s4, 0(s3) 
jal zero, loop