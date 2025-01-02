#include "i2c.h"
#include <stdint.h>

/*
 *
 */
static void I2C_GenerateStartCondition(I2C_Typedef_t* I2Cx)
{

	I2Cx->CR1 |= (0x1U << I2C_CR1_START);
}

/*
 *
 */

static void I2C_GenerateStopCondition(I2C_Typedef_t* I2Cx)
{

	I2Cx->CR1 |= (0x1U << I2C_CR1_STOP);
}
/*
 *
 */
static void I2C_ManageAdressPhase(I2C_Typedef_t* I2Cx, uint8_t Adress)
{
	Adress = Adress << 1;
	Adress &= ~(0x1U);
	I2Cx->DR = Adress;
}

/*
 *
 */
static void I2C_ClearAddrFlag(I2C_Typedef_t* I2Cx)
{
	uint32_t temp = I2Cx->SR1;
	temp = I2Cx->SR2;
	(void)temp;


}
/**
 * @brief This function Enable or Disable I2C Peripheral
 *
 * @param I2Cx = I2C Peripheral
 *
 * @param stateOfI2C = Enable or Disable
 *
 * @retval None
 */

void I2C_PeriphCmd(I2C_Typedef_t *I2Cx, FunctionalState_t stateOfI2C)
{
	if (stateOfI2C == ENABLE) {

		I2Cx->CR1 |= (0x1U << I2C_CR1_PE);
	}

	else {

		I2Cx->CR1 &= ~(0x1U << I2C_CR1_PE);
	}
}


/**
 * @brief This function Configures the I2C Peripheral
 *
 * @param I2C_Handle = User Config structure
 *
 * @retval None
 */

void I2C_Init(I2C_HandleTypedef_t *I2C_Handle)
{
	uint32_t periphClock = 0;

	periphClock = RCC_GetPCLK1();

	if (CONTROL_PCLOCK_VALUE(periphClock, I2C_Handle->Init.ClockSpeed) != 0x1U) {

		uint32_t tempreg = 0x0U;
		uint32_t freqValue = 0x0U;
		uint16_t ccrValue = 0;
		uint8_t trise = 0;

		/***   ACK State -- Stretching Mode ***/

		tempreg = I2C_Handle->Instance->CR1;

		tempreg |= (I2C_Handle->Init.ACK_State) | (I2C_Handle->Init.ClockStretch);

		I2C_Handle->Instance->CR1 = tempreg;

		/***   FREQ Value of PCLOCK ***/

		freqValue = I2C_GET_FREQ_VALUE(periphClock);

		tempreg = I2C_Handle->Instance->CR2;

		tempreg &= ~(0x3FU << 0x0U);

		tempreg |= (freqValue << 0x0U);

		I2C_Handle->Instance->CR2 = tempreg;

		/***   Address Config  ***/

		tempreg = I2C_Handle->Instance->OAR1;

		tempreg &= ~(0xC3FFU << 0);

		tempreg |= (I2C_Handle->Init.AddressingMode);

		if (I2C_Handle->Init.AddressingMode == I2C_ADDRMODE_7) {

			tempreg |= (I2C_Handle->Init.DeviceAddress << 0x1U);
		}

		else {

			tempreg |= (I2C_Handle->Init.DeviceAddress << 0x0U);
		}

		I2C_Handle->Instance->OAR1 = tempreg;

		/***   I2C SCL Speed (CCR) Config  ***/

		tempreg = I2C_Handle->Instance->CCR;

		tempreg &= ~(0xCFFFU << 0);

		tempreg |= I2C_Handle->Init.DutyCycle;

		if (I2C_Handle->Init.ClockSpeed <= I2C_SPEED_Standart) {

			ccrValue = periphClock / (2 * I2C_Handle->Init.ClockSpeed);
		}

		else {

			if (I2C_Handle->Init.DutyCycle == I2C_DUTY_FAST_2) {

				ccrValue = periphClock / (3 * I2C_Handle->Init.ClockSpeed);
			}

			else {

				ccrValue = periphClock / (25 * I2C_Handle->Init.ClockSpeed);
			}
		}

		tempreg |= (ccrValue & 0xFFFU);

		I2C_Handle->Instance->CCR = tempreg;

		/***   Trise Config  ***/

		tempreg = I2C_Handle->Instance->TRISE;

		tempreg &= ~(0x3FU << 0);

		if (I2C_Handle->Init.ClockSpeed <= I2C_SPEED_Standart) {

			trise = (periphClock / 1000000U) + 1;
		}

		else {
			trise = ((periphClock * 300) / 1000000000U) + 1;
		}

		tempreg |= trise & 0x3FU;

		I2C_Handle->Instance->TRISE = tempreg;


	}
}


/**
 * @brief This function Transmits data to the slave
 *
 * @param I2C_Handle = User Config structure
 *
 * @param pdata = Address of data to send
 *
 * @param sizeOfData = Size of your data
 *
 * @retval None
 */
void I2C_MasterTransmitData(I2C_HandleTypedef_t* I2C_Handle,uint8_t Adress, uint8_t *pdata, uint16_t sizeOfData)
{

	I2C_GenerateStartCondition(I2C_Handle->Instance);

	while (!(I2C_GetFlagStatus(I2C_Handle, I2C_FLAG_SB)));

	I2C_ManageAdressPhase(I2C_Handle->Instance, Adress);

	while (!(I2C_GetFlagStatus(I2C_Handle, I2C_FLAG_ADDR1)));

	I2C_ClearAddrFlag(I2C_Handle->Instance);

	while (sizeOfData > 0) {

		while (!(I2C_GetFlagStatus(I2C_Handle, I2C_FLAG_TXE)));
		I2C_Handle->Instance->DR = (uint8_t)*pdata;
		sizeOfData -= sizeof(uint8_t);
		pdata += sizeof(uint8_t);
	}

	while (!(I2C_GetFlagStatus(I2C_Handle, I2C_FLAG_TXE)) && !(I2C_GetFlagStatus(I2C_Handle, I2C_FLAG_ADDR1)));

	I2C_GenerateStopCondition(I2C_Handle->Instance);
}
/**
 * @brief This function return the flag of I2C_SR register
 *
 * @param I2C_Handle = User Config structure
 *
 * @param I2C_Flag = Flag name of SR register
 *
 * @retval I2C_FlagStatus_t
 */

I2C_FlagStatus_t I2C_GetFlagStatus(I2C_HandleTypedef_t* I2C_Handle, uint16_t I2C_Flag)
{
	return (I2C_Handle->Instance->SR1 & I2C_Flag) ? I2C_FLAG_SET : I2C_FLAG_RESET;
}



