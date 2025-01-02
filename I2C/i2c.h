#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f407xx.h"


typedef enum {

	I2C_FLAG_RESET = 0x0U,
	I2C_FLAG_SET = !I2C_FLAG_RESET

}I2C_FlagStatus_t;

#define CONTROL_PCLOCK_VALUE(PCLOCK, I2C_CLOCK)			((I2C_CLOCK <= I2C_SPEED_Standart) ? (PCLOCK <= 2000000) : (PCLOCK <= 4000000))
#define I2C_GET_FREQ_VALUE(PCLOCK)						((PCLOCK) / 1000000)


/*
 * @def_group I2C_ClockSpeeds
 */

#define  I2C_SPEED_Standart			(100000U)
#define  I2C_SPEED_Fast				(400000U)


/*
 * @def_group I2C_ACK_State
 */

#define I2C_ACK_DISABLE				(0x00000000U)
#define I2C_ACK_ENABLE				(0x00000400U)


/*
 * @def_group I2C_Clock_Stretch
 */

#define I2C_STRETCH_ENABLE 			(0x00000000U)
#define I2C_STRETCH_DISABLE 		(0x00000080U)


/*
 * @def_group I2C_Address_Mode
 */

#define I2C_ADDRMODE_7				(0x00004000U)
#define I2C_ADDRMODE_10				(0x0000C000U)


/*
 * @def_group I2C_Duty_Cycle
 */

#define I2C_DUTY_STANDART			(0x00000000U)
#define I2C_DUTY_FAST_2				(0x00008000U)
#define I2C_DUTY_FAST_16_9			(0x0000C000U)

typedef struct {

	uint32_t ClockSpeed;		/*!> I2C Clock Speed Mode         	 @def_group I2C_ClockSpeeds             */
	uint32_t ACK_State;			/*!> I2C ACK State Mode           	 @def_group I2C_ACK_State            	*/
	uint32_t AddressingMode;    /*!> I2C Address Mode           	 @def_group I2C_Address_Mode            */
	uint32_t ClockStretch;		/*!> I2C Clock Stretch Mode          @def_group I2C_Clock_Stretch           */
	uint32_t DutyCycle;			/*!> I2C Duty Cycle Mode             @def_group I2C_Duty_Cycle            	*/
	uint32_t DeviceAddress;		/*!> I2C Slave Mode Address              						            */
}I2C_InitTypedef_t;


typedef struct {

	I2C_Typedef_t * Instance;
	I2C_InitTypedef_t Init;

}I2C_HandleTypedef_t;

void I2C_Init(I2C_HandleTypedef_t *I2C_Handle);
void I2C_PeriphCmd(I2C_Typedef_t *I2Cx, FunctionalState_t stateOfI2C);
void I2C_MasterTransmitData(I2C_HandleTypedef_t* I2C_Handle,uint8_t Adress, uint8_t * pdata, uint16_t sizeOfData);
void I2C_ReceiveData(I2C_HandleTypedef_t* I2C_Handle, uint8_t *pBuffer, uint16_t sizeOfData);
void I2C_ReceiveData_IT(I2C_HandleTypedef_t* I2C_Handle, uint8_t *pBuffer, uint16_t sizeOfData);
void I2C_TransmitData_IT(I2C_HandleTypedef_t* I2C_Handle, uint8_t * pdata, uint16_t sizeOfData);
void I2C_InterruptHandler(I2C_HandleTypedef_t* I2C_Handle);
I2C_FlagStatus_t I2C_GetFlagStatus(I2C_HandleTypedef_t* I2C_Handle, uint16_t I2C_Flag);
#endif /* INC_I2C_H_ */
