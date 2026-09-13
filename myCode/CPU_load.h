#ifndef _CPU_LOAD_H_
#define _CPU_LOAD_H_
#include "main.h"
#define CPU_CYCLES 170000000U //170MHz
#define TEST_CYCLES	8500U			//8500cyc 50us
extern uint32_t RUN_CYC;
void CPU_CycleCounter_Init(void);
#endif 
