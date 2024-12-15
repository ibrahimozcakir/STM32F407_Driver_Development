#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f407xx.h"


typedef enum {

	USART_FLAG_RESET = 0x0U,
	USART_FLAG_SET = !USART_FLAG_RESET

}USART_FlagStatus_t;


typedef enum {

	USART_BUS_FREE = 0x0U,
	USART_BUS_BUSY_TX = 0x1U,
	USART_BUS_BUSY_RX = 0x2U

}USART_BusStatus_t;
/*
 * @def_group USART_Mode_Types
 *
 */

#define USART_MODE_Rx			    ((uint32_t)(0x00000004))
#define USART_MODE_Tx			    ((uint32_t)(0x00000008))
#define USART_MODE_Tx_Rx			((uint32_t)(0x0000000C))

/*
 * @def_group USART_WordLength_Types
 *
 */

#define USART_WORDLENGTH_8Bits		((uint32_t)(0x00000000))
#define USART_WORDLENGTH_9Bits		((uint32_t)(0x00001000))


/*
 * @def_group USART_Parity_Modes
 *
 */

#define USART_PARITY_NONE			((uint32_t)(0x00000000))
#define USART_PARITY_EVEN			((uint32_t)(0x00000400))
#define USART_PARITY_ODD			((uint32_t)(0x00000600))


/*
 * @def_group USART_StopBits_Modes
 *
 */

#define USART_STOPBITS_1			((uint32_t)(0x00000000))
#define USART_STOPBITS_0_5			((uint32_t)(0x00001000))
#define USART_STOPBITS_2			((uint32_t)(0x00002000))
#define USART_STOPBITS_1_5			((uint32_t)(0x00003000))

/*
 * @def_group USART_OverSampling_Modes
 *
 */

#define USART_OVERSAMPLE_16 		((uint32_t)(0x00000000))
#define USART_OVERSAMPLE_8 			((uint32_t)(0x00008000))


/*
 * @def_group USART_HardWare_Flow_Control_Modes
 *
 */

#define USART_HardWFlowControl_NONE		((uint32_t)(0x00000000))
#define USART_HardWFlowControl_RTS		((uint32_t)(0x00000100))
#define USART_HardWFlowControl_CTS		((uint32_t)(0x00000200))
#define USART_HardWFlowControl_RTS_CTS	((uint32_t)(0x00000300))


/*
 * Function Like Macro Definations
 */

#define __USART_DIV_VALUE_8(PCLOCK, BAUDRATE)   		((25 * (uint32_t)(PCLOCK)) / (2 * (uint32_t)(BAUDRATE)))
#define __USART_DIV_VALUE_16(PCLOCK, BAUDRATE) 			((25 * (uint32_t)(PCLOCK)) / (4 * (uint32_t)(BAUDRATE)))


typedef struct {

	uint32_t Mode;    			 	/*!< Transmission and Reception Modes @def_group USART_Mode_Types 				*/
	uint32_t BaudRate;				/*!< User Value For USART BaudRate												*/
	uint32_t WordLength;		 	/*!< 8 Bits & 9 Bits Modes @def_group USART_WordLength_Types    				*/
	uint32_t Parity;			 	/*!< Even & Odd Parity Modes @def_group USART_Parity_Modes 	   		    		*/
	uint32_t StopBits;			 	/*!< Stop Bits Modes @def_group USART_StopBits_Modes 	 		  	    		*/
	uint32_t OverSampling;		 	/*!< Over Sampling Modes @def_group USART_OverSampling_Modes 	 	    		*/
	uint32_t HardWareFlowControl;	/*!< HardWare Flow Control Modes @def_group USART_HardWare_Flow_Control_Modes 	*/

}USART_InitTypeDef_t;


typedef struct __USART_HandleTypedef_t {

	USART_Typedef_t* Instance;
	USART_InitTypeDef_t Init;
	USART_BusStatus_t busStateTx;
	USART_BusStatus_t busStateRx;
	void(*TxISR_Function)(struct __USART_HandleTypedef_t *USART_Handle);
	void(*RxISR_Function)(struct __USART_HandleTypedef_t *USART_Handle);
	uint16_t DataSizeTx;
	uint16_t DataSizeRx;
	uint8_t * pDataAddrTx;
	uint8_t * pDataAddrRx;

}USART_HandleTypedef_t;

void USART_Init(USART_HandleTypedef_t* USART_Handle);
void USART_TransmitData(USART_HandleTypedef_t* USART_Handle, uint8_t*pData, uint16_t dataSize);
void USART_PeriphCmd(USART_HandleTypedef_t* USART_Handle, FunctionalState_t StateOfUSART);
void USART_ReceiveData(USART_HandleTypedef_t* USART_Handle, uint8_t*pBuffer, uint16_t dataSize);
void USART_TransmitData_IT(USART_HandleTypedef_t* USART_Handle, uint8_t*pData, uint16_t dataSize);
void USART_ReceiveData_IT(USART_HandleTypedef_t* USART_Handle, uint8_t*pBuffer, uint16_t dataSize);
void USART_InterruptHandler(USART_HandleTypedef_t* USART_Handle);
USART_FlagStatus_t USART_GetFlagStatus(USART_HandleTypedef_t* USART_Handle, uint16_t USART_Flag);
#endif /* INC_USART_H_ */
