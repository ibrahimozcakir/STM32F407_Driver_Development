#include "exti.h"


/**
 * @brief This function configures the port and pin for SYSCFG
 *
 * @param EXTI_PortSource = Port Value A - I @def_group EXTI_Port_Values
 *
 * @param EXTI_LineSource =  Pin Numbers & Line Numbers  0 - 15.  @def_group EXTI_Line_Values
 *
 * @retval None
 */

void EXTI_LineConfig(uint8_t EXTI_PortSource, uint8_t EXTI_LineSource)
{

	uint32_t tempValue;

	tempValue = SYSCFG->EXTI_CR[EXTI_LineSource >> 2];
	tempValue &= ~(0xFU << (EXTI_LineSource & 0x3U) * 4);
	tempValue = (EXTI_PortSource << (EXTI_LineSource & 0x3U) * 4);
	SYSCFG->EXTI_CR[EXTI_LineSource >> 2] = tempValue;

}


/**
 * @brief This function Configures the port and line number for GPIO
 *
 * @param EXTI_InitStruct  = User Config Structure
 *
 * @retval None
 */

void EXTI_Init(EXTI_InitTypeDef_t* EXTI_InitStruct)
{
	uint32_t tempValue = 0;

	tempValue = (uint32_t)EXTI_BASE_ADDR;

	EXTI->IMR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
	EXTI->EMR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);

	if (EXTI_InitStruct->EXTI_LineCmd != DISABLE) {

		tempValue += EXTI_InitStruct->EXTI_Mode;

		*((__IO uint32_t*)tempValue) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);

		tempValue = (uint32_t)EXTI_BASE_ADDR;

		EXTI->RTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
		EXTI->FTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);

		if (EXTI_InitStruct->TriggerSelection == EXTI_Trigger_RF) {

			EXTI->RTSR |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
			EXTI->FTSR |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
		}
		else {

			tempValue += EXTI_InitStruct->TriggerSelection;

			*((__IO uint32_t*) tempValue) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
		}
	}

	else {

		tempValue = (uint32_t)EXTI_BASE_ADDR;

		tempValue += EXTI_InitStruct->EXTI_Mode;

		*((__IO uint32_t*)tempValue) &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
	}
}


/**
 * @brief This function enable NVIC
 *
 * @param IRQNumber  = IRQ number of line
 *
 * @retval None
 */

void NVIC_EnableInterrupt(IRQNumber_Typedef_t IRQNumber)
{
	uint32_t tempValue = 0;

	tempValue = *((IRQNumber >> 5U) + NVIC_ISER0);

	tempValue &= ~(0x1U << (IRQNumber & 0x1FU));

	tempValue |= (0x1U << (IRQNumber & 0x1FU));

	*((IRQNumber >> 5U) + NVIC_ISER0) = tempValue;
}
