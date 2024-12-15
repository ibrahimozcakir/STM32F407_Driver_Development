#include "usart.h"
#include <stdint.h>


/**
 * @brief This function Disables the interrupt for Transmission
 *
 * @param USART_Handle = User Config structure
 *
 * @retval None
 */

static void USART_CloseIT_Tx(USART_HandleTypedef_t * USART_Handle)
{
	USART_Handle->DataSizeTx = 0;
	USART_Handle->pDataAddrTx = NULL;
	USART_Handle->busStateTx = USART_BUS_FREE;
	USART_Handle->Instance->CR1 &= ~(0x1U << USART_CR1_TXEIE);
}


/**
 * @brief This function Disables the interrupt for Reception
 *
 * @param USART_Handle = User Config structure
 *
 * @retval None
 */

static void USART_CloseIT_Rx(USART_HandleTypedef_t * USART_Handle)
{
	USART_Handle->DataSizeRx = 0;
	USART_Handle->pDataAddrRx = NULL;
	USART_Handle->busStateRx = USART_BUS_FREE;
	USART_Handle->Instance->CR1 &= ~(0x1U << USART_CR1_RXNEIE);
}



/**
 * @brief This function stores the user data into the DR register
 *
 * @param USART_Handle = User Config structure
 *
 * @retval None
 */

static void USART_TransmitHelper_IT(USART_HandleTypedef_t * USART_Handle)
{

	if ( (USART_Handle->Init.WordLength == USART_WORDLENGTH_9Bits) && (USART_Handle->Init.Parity == USART_PARITY_NONE) ) {

	uint16_t * data16Bits = (uint16_t*)USART_Handle->pDataAddrTx;

	USART_Handle->Instance->DR = (uint16_t)(*data16Bits & (uint16_t)0x01FF);
	USART_Handle->pDataAddrTx += sizeof(uint16_t);
	USART_Handle->DataSizeTx -= sizeof(uint16_t);

	}

	else {

		USART_Handle->Instance->DR = (uint8_t)(*(USART_Handle->pDataAddrTx) & (uint8_t)0x00FF);
		USART_Handle->pDataAddrTx += sizeof(uint8_t);
		USART_Handle->DataSizeTx -= sizeof(uint8_t);
	}

	if (USART_Handle->DataSizeTx == 0) {

		 USART_CloseIT_Tx(USART_Handle);
	}
}


/**
 * @brief This function reads the DR register and stores into the user variable
 *
 * @param USART_Handle = User Config structure
 *
 * @retval None
 */

static void USART_ReceiveHelper_IT(USART_HandleTypedef_t * USART_Handle)
{
	uint16_t *p16BitsBuffer;
	uint8_t *p8BitsBuffer;

	if (USART_Handle->Init.WordLength == USART_WORDLENGTH_9Bits && USART_Handle->Init.Parity == USART_PARITY_NONE) {

		p16BitsBuffer = (uint16_t*)USART_Handle->pDataAddrRx;
		p8BitsBuffer = NULL;
	}

	else {

		p8BitsBuffer = (uint8_t*)USART_Handle->pDataAddrRx;
		p16BitsBuffer = NULL;
	}

	if (p8BitsBuffer == NULL) {

		*p16BitsBuffer = (uint16_t)(USART_Handle->Instance->DR & 0x1FF);
		USART_Handle->pDataAddrRx += sizeof(uint16_t);
		USART_Handle->DataSizeRx -= sizeof(uint16_t);
	}

	else {

		if (USART_Handle->Init.WordLength == USART_WORDLENGTH_9Bits && USART_Handle->Init.Parity != USART_PARITY_NONE) {

			*p8BitsBuffer = (uint8_t)(USART_Handle->Instance->DR & 0xFF);
			USART_Handle->pDataAddrRx += sizeof(uint8_t);
			USART_Handle->DataSizeRx -= sizeof(uint8_t);
		}

		else if (USART_Handle->Init.WordLength == USART_WORDLENGTH_8Bits && USART_Handle->Init.Parity == USART_PARITY_NONE) {

			*p8BitsBuffer = (uint8_t)(USART_Handle->Instance->DR & 0xFF);
			USART_Handle->pDataAddrRx += sizeof(uint8_t);
			USART_Handle->DataSizeRx -= sizeof(uint8_t);
		}

		else {

			*p8BitsBuffer = (uint8_t)(USART_Handle->Instance->DR & 0x7F);
			USART_Handle->pDataAddrRx += sizeof(uint8_t);
			USART_Handle->DataSizeRx -= sizeof(uint8_t);
		}
	}

	if (USART_Handle->DataSizeRx == 0) {

		USART_CloseIT_Rx(USART_Handle);
	}
}



/**
 * @brief This function Configures the USART Peripheral
 *
 * @param USART_Handle = User Config structure
 *
 * @retval None
 */

void USART_Init(USART_HandleTypedef_t *USART_Handle) {

	uint32_t periphClock = 0;
	uint32_t tempreg = 0;
	uint32_t mantissaPart = 0;
	uint32_t freactionPart = 0;
	uint32_t USART_DIV_Value = 0;
	uint32_t tempValue = 0;

	/************** Over Sampling -- WordLength -- Parity -- Mode ********************/
	tempreg = USART_Handle->Instance->CR1;

	tempreg |= (USART_Handle->Init.OverSampling)
			| (USART_Handle->Init.WordLength) | (USART_Handle->Init.Mode)
			| (USART_Handle->Init.Parity);

	USART_Handle->Instance->CR1 = tempreg;

	/************** Stop Bits ********************/

	tempreg = USART_Handle->Instance->CR2;

	tempreg &= ~(0x3U << USART_CR2_STOP);

	tempreg |= (USART_Handle->Init.StopBits);

	USART_Handle->Instance->CR2 = tempreg;

	/************** HardWare Flow Control ********************/

	tempreg = USART_Handle->Instance->CR3;

	tempreg |= (USART_Handle->Init.HardWareFlowControl);

	USART_Handle->Instance->CR3 = tempreg;

	/************** Baud Rate Config ********************/

	if (USART_Handle->Instance == USART1 || USART_Handle->Instance == USART6) {

		periphClock = RCC_GetPCLK2();
	}

	else {

		periphClock = RCC_GetPCLK1();
	}

	if (USART_Handle->Init.OverSampling == USART_OVERSAMPLE_8) {

		USART_DIV_Value = __USART_DIV_VALUE_8(periphClock, USART_Handle->Init.BaudRate);
		mantissaPart = (USART_DIV_Value / 100);
		freactionPart = (USART_DIV_Value - (mantissaPart * 100));

		freactionPart = ( ((freactionPart * 8U) + 50U) / 100U ) & (0x07U);
	}

	else {

		USART_DIV_Value = __USART_DIV_VALUE_16(periphClock, USART_Handle->Init.BaudRate);
		mantissaPart = (USART_DIV_Value / 100);
		freactionPart = (USART_DIV_Value - (mantissaPart * 100));

		freactionPart = ( ((freactionPart * 16U) + 50U) / 100U ) & (0x0FU) ;
	}

	tempValue |= (mantissaPart << 4U);
	tempValue |= (freactionPart & 0x0F);

	USART_Handle->Instance->BRR = tempValue;
}


/**
 * @brief This function Transmits data
 *
 * @param USART_Handle = User Config structure
 *
 * @param pData = Address of data to send
 *
 * @param dataSize = Size of your data
 *
 * @retval None
 */

void USART_TransmitData(USART_HandleTypedef_t* USART_Handle, uint8_t*pData, uint16_t dataSize)
{
    uint16_t *data16Bits;

    if ((USART_Handle->Init.WordLength == USART_WORDLENGTH_9Bits) &&
        (USART_Handle->Init.Parity == USART_PARITY_NONE)) {

    	data16Bits = (uint16_t*)pData;
    }

    else {

    	data16Bits = NULL;
    }

    while (dataSize > 0) {

    	while ( !(USART_GetFlagStatus(USART_Handle, USART_FLAG_TXE)) )
    		  	  	  	  ; // Null Statement

    	/*** 7 or 8 Bits Data ***/
    	if (data16Bits == NULL) {

    		USART_Handle->Instance->DR = (uint8_t)(*pData & 0xFFU);
    		++pData;
    		dataSize -= sizeof(uint8_t);
    	}

    	/*** 9 Bits Data No Parity ***/
    	else {

    		USART_Handle->Instance->DR = (uint16_t)(*data16Bits & (0x01FF));
    		++data16Bits;
    		dataSize -= sizeof(uint16_t);
    	}
    }

    while( !(USART_GetFlagStatus(USART_Handle, USART_FLAG_TC)) )
    			; // Null Statement
}


/**
 * @brief This function Receive data
 *
 * @param USART_Handle = User Config structure
 *
 * @param pBuffer = Address of data to store the values that I get
 *
 * @param pBuffer = Size of your data
 *
 * @retval None
 */

void USART_ReceiveData(USART_HandleTypedef_t* USART_Handle, uint8_t*pBuffer, uint16_t dataSize)
{
	uint16_t * pBuffer16Bits;
	uint8_t * pBuffer8Bits;

	if (USART_Handle->Init.WordLength == USART_WORDLENGTH_9Bits && USART_Handle->Init.Parity == USART_PARITY_NONE) {

		pBuffer16Bits = (uint16_t*)pBuffer;
		pBuffer8Bits = NULL;
	}

	else {
		pBuffer8Bits = (uint8_t*)pBuffer;
		pBuffer16Bits = NULL;
	}

	while (dataSize > 0) {

		while ( !(USART_GetFlagStatus(USART_Handle, USART_FLAG_RXNE)) )
				; // Null Statement

		/*** 9 Bits Data ***/
		if (pBuffer8Bits == NULL) {

			*pBuffer16Bits = (uint16_t)(USART_Handle->Instance->DR & 0x01FFU);
			dataSize -= sizeof(uint16_t);
			++pBuffer16Bits;
		}

		/*** 7 or 8 Bits Data ***/

		else {

			/*** 8 Bits Data  ***/
			if (USART_Handle->Init.WordLength == USART_WORDLENGTH_9Bits && USART_Handle->Init.Parity != USART_PARITY_NONE) {

				*pBuffer8Bits = (uint8_t)(USART_Handle->Instance->DR & 0xFFU);
				++pBuffer8Bits;
				dataSize -= sizeof(uint8_t);
			}

			/*** 8 Bits Data ***/

			else if (USART_Handle->Init.WordLength == USART_WORDLENGTH_8Bits && USART_Handle->Init.Parity == USART_PARITY_NONE) {

				*pBuffer8Bits = (uint8_t)(USART_Handle->Instance->DR & 0xFFU);
				++pBuffer8Bits;
				dataSize -= sizeof(uint8_t);
			}

			/*** 7 Bits Data ***/
			else {

				*pBuffer8Bits = (uint8_t)(USART_Handle->Instance->DR & 0x7FU);
				++pBuffer8Bits;
				dataSize -= sizeof(uint8_t);
			}
		}
	}
}


/**
 * @brief This function send the data to external world with Interrupt method
 *
 * @param USART_Handle = User Config structure
 *
 * @param pData = Carries the user data
 *
 * @param dataSize = Sizeof data that will send
 *
 * @retval None
 */

void USART_TransmitData_IT(USART_HandleTypedef_t* USART_Handle, uint8_t*pData, uint16_t dataSize)
{
	USART_BusStatus_t busState = USART_Handle->busStateTx;

	if (busState != USART_BUS_BUSY_TX) {

		USART_Handle->pDataAddrTx = (uint8_t*)pData;
		USART_Handle->DataSizeTx = (uint16_t)dataSize;
		USART_Handle->busStateTx = USART_BUS_BUSY_TX;

		USART_Handle->TxISR_Function = USART_TransmitHelper_IT;

		USART_Handle->Instance->CR1 |= (0x1U << USART_CR1_TXEIE);
	}
}


/**
 * @brief This function reads the data from external world with Interrupt method
 *
 * @param USART_Handle = User Config structure
 *
 * @param pBuffer = Stores the data in this variable
 *
 * @param dataSize = Sizeof data that will read
 *
 * @retval None
 */

void USART_ReceiveData_IT(USART_HandleTypedef_t* USART_Handle, uint8_t*pBuffer, uint16_t dataSize)
{
	USART_BusStatus_t busState = USART_Handle->busStateRx;

	if (busState != USART_BUS_BUSY_RX) {

		USART_Handle->pDataAddrRx = (uint8_t*)pBuffer;
		USART_Handle->DataSizeRx = (uint16_t)dataSize;
		USART_Handle->busStateRx = USART_BUS_BUSY_RX;

		USART_Handle->RxISR_Function = USART_ReceiveHelper_IT;

		USART_Handle->Instance->CR1 |= (0x1U << USART_CR1_RXNEIE);
	}
}



/**
 * @brief This function call the helper function if the flag is set
 *
 * @param USART_Handle = User Config structure
 *
 * @retval None
 */

void USART_InterruptHandler(USART_HandleTypedef_t* USART_Handle)
{
	uint8_t InterruptSource = 0;
	uint8_t InterruptFlag = 0;

	InterruptFlag = (uint8_t)((USART_Handle->Instance->SR >> USART_SR_TXE) & 0x1U);
	InterruptSource = (uint8_t)((USART_Handle->Instance->CR1 >> USART_CR1_TXEIE) & 0x1U);

	if (InterruptFlag && InterruptSource) {

		USART_Handle->TxISR_Function(USART_Handle);
	}

	InterruptFlag = (uint8_t)((USART_Handle->Instance->SR >> USART_SR_RXNE) & 0x1U);
	InterruptSource = (uint8_t)((USART_Handle->Instance->CR1 >> USART_CR1_RXNEIE) & 0x1U);

	if (InterruptFlag && InterruptSource) {

		USART_Handle->RxISR_Function(USART_Handle);
	}

}


/**
 * @brief This function Enable or Disable USART Peripheral
 *
 * @param USART_Handle = User Config structure
 *
 * @param stateOfUSART = Enable or Disable
 *
 * @retval None
 */

void USART_PeriphCmd(USART_HandleTypedef_t* USART_Handle, FunctionalState_t StateOfUSART)
{
	if (StateOfUSART == ENABLE) {

		USART_Handle->Instance->CR1 |= (0x1U << USART_CR1_UE);
	}

	else {
		USART_Handle->Instance->CR1 &= ~(0x1U << USART_CR1_UE);
	}
}

/**
 * @brief This function return the flag of USART_SR register
 *
 * @param USART_Handle = User Config structure
 *
 * @param USART_Flag = Flag name of SR register
 *
 * @retval USART_FlagStatus_t
 */

USART_FlagStatus_t USART_GetFlagStatus(USART_HandleTypedef_t* USART_Handle, uint16_t USART_Flag)
{
	return (USART_Handle->Instance->SR & USART_Flag) ? USART_FLAG_SET : USART_FLAG_RESET;
}




