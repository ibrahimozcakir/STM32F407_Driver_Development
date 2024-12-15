#include "gpio.h"

/**
 * @brief This function Configures the port and pin
 *
 * @param GPIOx = GPIO Port Base Address.
 *
 * @param GPIO_Init = User Config Structures
 *
 * @retval None
 */

void GPIO_Init(GPIO_TypeDef_t* GPIOx, GPIO_InitTypeDef_t *GPIO_Init)
{

	uint32_t positions;
	uint32_t fakePositions = 0;
	uint32_t lastPositions = 0;
	uint32_t temp = 0;

	for (positions = 0U; positions < 16; ++positions) {

		fakePositions = (0x1 << positions);
		lastPositions = (uint32_t) (GPIO_Init->pinNumber) & fakePositions;

		if (fakePositions == lastPositions) {

			/* MODE CONFIG */
			temp = GPIOx->MODER;
			temp &= ~(0x3U << (positions * 2));
			temp |= (GPIO_Init->Mode << (positions * 2));
			GPIOx->MODER = temp;

			if (GPIO_Init->Mode == GPIO_MODE_OUTPUT
					|| GPIO_Init->Mode == GPIO_MODE_AF) {

				/* OUTPUT TYPE CONFIG */
				temp = GPIOx->OTYPER;
				temp &= ~(0x1U << positions);
				temp |= (GPIO_Init->Otype << positions);
				GPIOx->OTYPER = temp;

				/* OUTPUT SPEED CONFIG */
				temp = GPIOx->OSPEEDR;
				temp &= ~(0x3U << (positions * 2));
				temp |= (GPIO_Init->Speed << (positions * 2));
				GPIOx->OSPEEDR = temp;

			}

			/* PUSH PULL CONFIG */
			temp = GPIOx->PUPDR;
			temp &= ~(0x3U << (positions * 2));
			temp |= (GPIO_Init->PuPd << (positions * 2));
			GPIOx->PUPDR = temp;

			/* ALTERNATE FUNCTION CONFIG  */
			if (GPIO_Init->Mode == GPIO_MODE_AF) {

				temp = GPIOx->AFR[positions >> 0x3U];
				temp &= ~(0xFU << ((positions & 0x7U) * 4));
				temp |= (GPIO_Init->Alternate << ((positions & 0x7U) * 4));
				GPIOx->AFR[positions >> 0x3U] = temp;
			}
		}
	}
}

/**
 * @brief This function makes pin High or Low of GPIOx Port
 *
 * @param GPIOx = GPIO Port Base Address.
 *
 * @param pinNumber = GPIO Pin Numbers  0 - 15.
 *
 * @param pinState = GPIO_Pin_Set or GPIO_Pin_Reset
 *
 * @retval None
 */
void GPIO_WritePin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState)
{
	if (pinState == GPIO_Pin_Set) {

		GPIOx->BSRR = pinNumber;
	}

	else {

		GPIOx->BSRR = (pinNumber << 16U);
	}
}

/**
 * @brief This function read the pin of GPIOx Port
 *
 * @param GPIOx = GPIO Port Base Address.
 *
 * @param pinNumber = GPIO Pin Numbers  0 - 15.
 *
 * @retval GPIO_PinState_t
 */

GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber) {
	GPIO_PinState_t bitStatus = GPIO_Pin_Reset;

	if ((GPIOx->IDR & pinNumber) != GPIO_Pin_Reset) {
		bitStatus = GPIO_Pin_Set;
	}
	return bitStatus;
}

/**
 * @brief This function locks the pin of GPIOx Port
 *
 * @param GPIOx = GPIO Port Base Address.
 *
 * @param pinNumber = GPIO Pin Numbers  0 - 15.
 *
 * @retval None
 */

void GPIO_LockPin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber)
{
	uint32_t tempValue = ((0x1U << 16) | pinNumber);

	GPIOx->LCKR = tempValue;
	GPIOx->LCKR = pinNumber;
	GPIOx->LCKR = tempValue;
	tempValue = GPIOx->LCKR;
}

/**
 * @brief This function toggle the pin of GPIOx Port
 *
 * @param GPIOx = GPIO Port Base Address.
 *
 * @param pinNumber = GPIO Pin Numbers  0 - 15.
 *
 * @retval None
 */

void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber) {

	uint32_t tempODR = GPIOx->ODR;

	GPIOx->BSRR = ((tempODR & pinNumber) << 16U) | (~tempODR & pinNumber);

}

