#include "../fpmult_sw_bsp/system.h"

#define MEMORY_BASE (ONCHIP_MEMORY2_0_BASE + 0x3000)

int main(void)
{
	volatile float* memory_op1 = (float*) MEMORY_BASE;
	volatile float* memory_op2 = (float*) (MEMORY_BASE + 4);
	volatile float* memory_out = (float*) (MEMORY_BASE + 8);
	volatile float* memory_out2 = (float*) (MEMORY_BASE + 12);
	volatile int* memory_start = (int*) (MEMORY_BASE + 16);
	volatile float* acc_op1 = (float*) AVS_FP_MULT_0_BASE;
	volatile float* acc_op2 = (float*) (AVS_FP_MULT_0_BASE + 4);
	volatile float* acc_out = (float*) (AVS_FP_MULT_0_BASE + 12);
	volatile int* acc_start = (int*) (AVS_FP_MULT_0_BASE + 8);

	while (1)
	{
		// STUDENTS TO ADD THES
        // 1. Write 1 to the memory start location
        // This will be automatically cleared later by the marker/tester
		*memory_start = 1;
		
		// 2. Perform software multiplication (just using the C++ multiplication operand)
        // The operands will be set for you by the marker/tester
		*memory_out = (*memory_op1) * (*memory_op2);
		
		// 3. Copy multiplication operands from memory operand locations to accelerator 
        // operand locations
		*acc_op1 = *memory_op1;
		*acc_op2 = *memory_op2;
		
		// 4. Perform hardware multiplication by writing to the accelerator start register 
        // and waiting for the bus flag to become deasserted
		*acc_start = 1;

		while(*acc_start){

		}
        
        // 5. Copy the accelerator result to memory_out2
		*memory_out2 = *acc_out;
	}
    return 0;
}
