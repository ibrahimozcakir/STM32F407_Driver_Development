#include "spi.h"
#include <stdint.h>


/**
 * @brief This function Disables the interrupt for Transmission
 *
 * @param SPI_Handle = User Config structure
 *
 * @retval None
 */

static void SPI_CloseIT_Tx(SPI_HandleTypedef_t* SPI_Handle)
{
	SPI_Handle->Instance->CR2 &= ~(0x1U << SPI_CR2_TXEIE);
	SPI_Handle->TxDataSize = 0U;
	SPI_Handle->pTxDataAddr = NULL;
	SPI_Handle->busStateTx = SPI_BUS_FREE;
}


/**
 * @brief This function Disables the interrupt for Reception
 *
 * @param SPI_Handle = User Config structure
 *
 * @retval None
 */

static void SPI_CloseIT_Rx(SPI_HandleTypedef_t* SPI_Handle)
{
	SPI_Handle->Instance->CR2 &= ~(0x1U << SPI_CR2_RXNEIE);
	SPI_Handle->RxDataSize = 0U;
	SPI_Handle->pRxDataAddr = NULL;
	SPI_Handle->busStateRx = SPI_BUS_FREE;
}


/**
 * @brief This function stores the user data into the DR register for 16 bits format
 *
 * @param SPI_Handle = User Config structure
 *
 * @retval None
 */

static void SPI_TransmitHelper_16Bits(SPI_HandleTypedef_t* SPI_Handle)
{
	*(uint16_t*)SPI_Handle->Instance->DR = *((uint16_t*)(SPI_Handle->pTxDataAddr));
	SPI_Handle->pTxDataAddr += sizeof(uint16_t);
	SPI_Handle->TxDataSize -= sizeof(uint16_t);

	if (SPI_Handle->TxDataSize == 0) {

		SPI_CloseIT_Tx(SPI_Handle);
	}
}


/**
 * @brief This function stores the user data into the DR register for 8 bits format
 *
 * @param SPI_Handle = User Config structure
 *
 * @retval None
 */

static void SPI_TransmitHelper_8Bits(SPI_HandleTypedef_t* SPI_Handle)
{
	SPI_Handle->Instance->DR = *((uint8_t*)(SPI_Handle->pTxDataAddr));
	SPI_Handle->pTxDataAddr += sizeof(uint8_t);
	SPI_Handle->TxDataSize -= sizeof(uint8_t);

	if (SPI_Handle->TxDataSize == 0) {

		SPI_CloseIT_Tx(SPI_Handle);
	}
}


/**
 * @brief This function reads the DR register and stores into the user variable for 8 bits format
 *
 * @param SPI_Handle = User Config structure
 *
 * @retval None
 */

static void SPI_ReceiveHelper_8Bits(SPI_HandleTypedef_t *SPI_Handle)
{
	*((uint8_t*)SPI_Handle->pRxDataAddr) = *((__IO uint8_t*)&SPI_Handle->Instance->DR);

	SPI_Handle->RxDataSize -= sizeof(uint8_t);
	SPI_Handle->pRxDataAddr += sizeof(uint8_t);

	if (SPI_Handle->RxDataSize == 0) {

		SPI_CloseIT_Rx(SPI_Handle);
	}
}


/**
 * @brief This function reads the DR register and stores into the user variable for 16 bits format
 *
 * @param SPI_Handle = User Config structure
 *
 * @retval None
 */

static void SPI_ReceiveHelper_16Bits(SPI_HandleTypedef_t *SPI_Handle)
{
	*(uint16_t*)SPI_Handle->pRxDataAddr = (uint16_t)SPI_Handle->Instance->DR;

	SPI_Handle->RxDataSize -= sizeof(uint16_t);
	SPI_Handle->pRxDataAddr += sizeof(uint16_t);

	if (SPI_Handle->RxDataSize == 0) {

		SPI_CloseIT_Rx(SPI_Handle);
	}
}

/**
 * @brief This function Configures the SPI Peripheral
 *
 * @param SPI_Handle = User Config structure
 *
 * @retval None
 */

void SPI_Init(SPI_HandleTypedef_t *SPI_Handle)
{
	uint32_t tempValue = 0;

	tempValue = SPI_Handle->Instance->CR1;

	tempValue |=  (SPI_Handle->Init.BaudRate) | (SPI_Handle->Init.CPHA)
			 | (SPI_Handle->Init.CPOL) | (SPI_Handle->Init.FrameFormat)
			 | (SPI_Handle->Init.DFF_Format) | (SPI_Handle->Init.Mode)
			 | (SPI_Handle->Init.Bus_Config) | (SPI_Handle->Init.SSM_Cmd);

	SPI_Handle->Instance->CR1 = tempValue;

}


/**
 * @brief This function Enable or Disable SPI Peripheral
 *
 * @param SPI_Handle = User Config structure
 *
 * @param StateOfSPI = Enable or Disable
 *
 * @retval None
 */

void SPI_PeriphCmd(SPI_HandleTypedef_t *SPI_Handle, FunctionalState_t StateOfSPI)
{
	if (StateOfSPI == ENABLE) {

		SPI_Handle->Instance->CR1 |=  (0x1U << SPI_CR1_SPE);
	}
	else {

		SPI_Handle->Instance->CR1 &= ~(0x1U << SPI_CR1_SPE);
	}
}

/**
 * @brief This function Transmits data to the slave
 *
 * @param SPI_Handle = User Config structure
 *
 * @param pdata = Address of data to send
 *
 * @param sizeOfData = Size of your data
 *
 * @retval None
 */

void SPI_TransmitData(SPI_HandleTypedef_t* SPI_Handle, uint8_t * pdata, uint16_t sizeOfData)
{

	if (SPI_Handle->Init.DFF_Format == SPI_DFF_16BITS) {

		while (sizeOfData > 0) {

			if (SPI_GetFlagStatus(SPI_Handle, SPI_FLAG_TXE)) {

				SPI_Handle->Instance->DR = *((uint16_t*)pdata);
				pdata += sizeof(uint16_t);
				sizeOfData -= sizeof(uint16_t);
			}
		}
	}
	else {

		while (sizeOfData > 0) {

			if (SPI_GetFlagStatus(SPI_Handle, SPI_FLAG_TXE)) {

				SPI_Handle->Instance->DR = *pdata;
				pdata += sizeof(uint8_t);
				sizeOfData -= sizeof(uint8_t);
			}
		}
	}

	while (SPI_GetFlagStatus(SPI_Handle, SPI_FLAG_BSY));
}

/**
 * @brief This function Receive data from the slave
 *
 * @param SPI_Handle = User Config structure
 *
 * @param pdata = Address of data to store the values that I get
 *
 * @param sizeOfData = Size of your data
 *
 * @retval None
 */

void SPI_ReceiveData(SPI_HandleTypedef_t* SPI_Handle, uint8_t *pBuffer, uint16_t sizeOfData)
{
	if (SPI_Handle->Init.DFF_Format == SPI_DFF_16BITS) {

		while (sizeOfData > 0) {

			if (SPI_GetFlagStatus(SPI_Handle, SPI_FLAG_RXNE)) {

				*((uint16_t*)pBuffer) = (uint16_t)SPI_Handle->Instance->DR;
				pBuffer += sizeof(uint16_t);
				sizeOfData -= sizeof(uint16_t);
			}

		}
	}

	else {

		while (sizeOfData > 0) {

			if (SPI_GetFlagStatus(SPI_Handle, SPI_FLAG_RXNE)) {

				*pBuffer = *((__IO uint8_t*)&SPI_Handle->Instance->DR);
				 pBuffer += sizeof(uint8_t);
				 sizeOfData -= sizeof(uint8_t);
			}
		}
	}
}

/**
 * @brief This function return the flag of SPI_SR register
 *
 * @param SPI_Handle = User Config structure
 *
 * @param SPI_Flag = Flag name of SR register
 *
 * @retval SPI_FlagStatus_t
 */

SPI_FlagStatus_t SPI_GetFlagStatus(SPI_HandleTypedef_t* SPI_Handle, uint16_t SPI_Flag)
{
	return (SPI_Handle->Instance->SR & SPI_Flag) ? SPI_FLAG_SET : SPI_FLAG_RESET;
}


/**
 * @brief This function send the data to external world with Interrupt method
 *
 * @param SPI_Handle = User Config structure
 *
 * @param pdata = Carries the user data
 *
 * @param sizeOfData = Sizeof data that will send
 *
 * @retval None
 */

void SPI_TransmitData_IT(SPI_HandleTypedef_t* SPI_Handle, uint8_t * pdata, uint16_t sizeOfData)
{
	SPI_BusStatus_t busState = SPI_Handle->busStateTx;

	if (busState != SPI_BUS_BUSY_TX) {

		SPI_Handle->pTxDataAddr = (uint8_t*)pdata;
		SPI_Handle->TxDataSize = (uint16_t)sizeOfData;

		SPI_Handle->busStateTx = SPI_BUS_BUSY_TX;

		if ((SPI_Handle->Instance->CR1 >> SPI_CR1_DFF) & 0x1U) {

			SPI_Handle->TxISRFunction = SPI_TransmitHelper_16Bits;
		}
		else {

			SPI_Handle->TxISRFunction = SPI_TransmitHelper_8Bits;
		}

		SPI_Handle->Instance->CR2 |= (0x1U << SPI_CR2_TXEIE);
	}
}


/**
 * @brief This function call the helper function if the flag is set
 *
 * @param SPI_Handle = User Config structure
 *
 * @retval None
 */

void SPI_InterruptHandler(SPI_HandleTypedef_t* SPI_Handle)
{
	uint8_t InterruptSource = 0;
	uint8_t InterruptFlag = 0;

	InterruptSource = ((SPI_Handle->Instance->CR2 >> SPI_CR2_TXEIE) & 0x1U);
	InterruptFlag = ((SPI_Handle->Instance->SR >> SPI_SR_TXE) & 0x1U);


	if (InterruptSource && InterruptFlag) {

		SPI_Handle->TxISRFunction(SPI_Handle);
	}

	InterruptSource = ((SPI_Handle->Instance->CR2 >> SPI_CR2_RXNEIE) & 0x1U);
	InterruptFlag = ((SPI_Handle->Instance->SR >> SPI_SR_RXNE) & 0x1U);

	if (InterruptSource && InterruptFlag) {

		SPI_Handle->RxISRFunction(SPI_Handle);
	}


}


/**
 * @brief This function reads the data from external world with Interrupt method
 *
 * @param SPI_Handle = User Config structure
 *
 * @param pBuffer = Stores the data in this variable
 *
 * @param sizeOfData = Sizeof data that will read
 *
 * @retval None
 */

void SPI_ReceiveData_IT(SPI_HandleTypedef_t* SPI_Handle, uint8_t *pBuffer, uint16_t sizeOfData)
{
	SPI_BusStatus_t busState = SPI_Handle->busStateRx;

	if (busState != SPI_BUS_BUSY_RX) {

		SPI_Handle->busStateRx = SPI_BUS_BUSY_RX;

		SPI_Handle->pRxDataAddr = (uint8_t*)pBuffer;
		SPI_Handle->RxDataSize = sizeOfData;

		if ((SPI_Handle->Instance->CR1 >> SPI_CR1_DFF) & 0x1U) {

			SPI_Handle->RxISRFunction = SPI_ReceiveHelper_16Bits;
		}
		else {

			SPI_Handle->RxISRFunction = SPI_ReceiveHelper_8Bits;
		}

		SPI_Handle->Instance->CR2 |= (0x1U << SPI_CR2_RXNEIE);
	}

}




