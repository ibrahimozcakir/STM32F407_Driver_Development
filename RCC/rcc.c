#include "rcc.h"

static const uint8_t AHB_Prescaler[] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static const uint8_t APB_Prescaler[] = {0, 0, 0, 0, 1, 2, 3, 4};

uint32_t RCC_GetSystemClock(void)
{
	uint32_t SystemCoreClock = 0;
	uint8_t ClockSource = 0;

	ClockSource = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3U);

	switch (ClockSource) {

	case 0: SystemCoreClock = 16000000; break;
	case 1: SystemCoreClock = 8000000;  break;
	default : SystemCoreClock = 16000000;
	}

	return SystemCoreClock;
}


uint32_t RCC_GetHClock(void)
{
	uint32_t HClock = 0;
	uint32_t SystemCoreClock = RCC_GetSystemClock();
	uint8_t HPRE_Value = ( (RCC->CFGR >> RCC_CFGR_HPRE) & 0xF );

	HClock = SystemCoreClock >> AHB_Prescaler[HPRE_Value];

	return HClock;
}


uint32_t RCC_GetPCLK1(void)
{
	uint32_t PCLK1 = 0;
	uint32_t HClock = RCC_GetHClock();
	uint8_t PPRE1_Value = ( (RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7U );

	PCLK1 = HClock >> APB_Prescaler[PPRE1_Value];

	return PCLK1;
}

uint32_t RCC_GetPCLK2(void)
{
	uint32_t PCLK2 = 0;
	uint32_t HClock = RCC_GetHClock();
	uint8_t PPRE2_Value = ( (RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7U );

	PCLK2 = HClock >> APB_Prescaler[PPRE2_Value];

	return PCLK2;

}
