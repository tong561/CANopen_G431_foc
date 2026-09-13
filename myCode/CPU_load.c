#include "CPU_load.h"
#include "main.h"


uint32_t RUN_CYC=0;

void CPU_CycleCounter_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}



