#ifndef INC_RCC_H_
#define INC_RCC_H_

#include "stm32f407xx.h"
/*
 * RCC AHB1 Peripherals Clock Control Macro Definitions
 *
 *
 */

#define RCC_GPIOA_CLK_ENABLE()       do { uint32_t temp = 0; \
	                                      SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);  \
										  temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);	\
										  UNUSED(temp);\
										}while(0)

#define RCC_GPIOB_CLK_ENABLE()       do { uint32_t temp = 0; \
	                                      SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);  \
										  temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);	\
										  UNUSED(temp);\
										}while(0)

#define RCC_GPIOC_CLK_ENABLE()      do { uint32_t temp = 0;      \
                                          SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);     \
										  temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);	  \
										  UNUSED(temp);\
  	  	  	  	  	  	  	  	  	  }while(0)

#define RCC_GPIOD_CLK_ENABLE()      do { uint32_t temp = 0;\
										 SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);   \
										 temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);	\
										 UNUSED(temp);	\
										}while(0)

#define RCC_GPIOE_CLK_ENABLE()      do { uint32_t temp = 0;\
										 SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);   \
										 temp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);	\
										 UNUSED(temp);	\
										}while(0)


#define RCC_GPIOA_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN)
#define RCC_GPIOB_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN)
#define RCC_GPIOC_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN)
#define RCC_GPIOD_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN)
#define RCC_GPIOE_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN)


/*
 * RCC APB1 Peripherals Clock Control Macro Definitions
 *
 *
 */

#define RCC_SPI2_CLK_ENABLE() 	do { uint32_t temp = 0; \
									SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);		\
									temp = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN)			\
									UNUSED(temp);			\
									}while(0)

#define RCC_SPI3_CLK_ENABLE() 	do { uint32_t temp = 0; \
									SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN);		\
									temp = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN)			\
									UNUSED(temp);			\
									}while(0)

#define RCC_USART2_CLK_ENABLE()   do { uint32_t temp =  0;		\
									SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);			\
									temp = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);		\
									UNUSED(temp);			\
									}while(0)

#define RCC_USART3_CLK_ENABLE()   do { uint32_t temp =  0;		\
									SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);			\
									temp = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);		\
									UNUSED(temp);			\
									}while(0)

#define RCC_UART4_CLK_ENABLE()   do { uint32_t temp =  0;		\
									SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN);			\
									temp = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN);		\
									UNUSED(temp);			\
									}while(0)

#define RCC_UART5_CLK_ENABLE()   do { uint32_t temp =  0;		\
									SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN);			\
									temp = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN);		\
									UNUSED(temp);			\
									}while(0)

#define RCC_I2C1_CLK_ENABLE()	do { uint32_t temp = 0;\
									SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);		\
									temp = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);		\
									UNUSED(temp);		\
									}while(0)

#define RCC_I2C2_CLK_ENABLE()	do { uint32_t temp = 0;\
									SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);		\
									temp = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);		\
									UNUSED(temp);		\
									}while(0)

#define RCC_I2C3_CLK_ENABLE()	do { uint32_t temp = 0;\
									SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN);		\
									temp = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN);		\
									UNUSED(temp);		\
									}while(0)


#define RCC_SPI2_CLK_DISABLE()		CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN)
#define RCC_SPI3_CLK_DISABLE()		CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN)

#define RCC_USART2_CLK_DISABLE()    CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN)
#define RCC_USART3_CLK_DISABLE()    CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN)
#define RCC_UART4_CLK_DISABLE()     CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN)
#define RCC_UART5_CLK_DISABLE()     CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN)

#define RCC_I2C1_CLK_DISABLE()		CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN)
#define RCC_I2C2_CLK_DISABLE()		CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN)
#define RCC_I2C3_CLK_DISABLE()		CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN)


/*
 * RCC APB2 Peripherals Clock Control Macro Definitions
 *
 *
 */

#define RCC_SYSCFG_CLK_ENABLE()   do {	uint32_t temp = 0;\
										SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);	   \
										temp = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);	\
										UNUSED(temp); \
                                     }while(0)

#define RCC_SPI1_CLK_ENABLE()	 do { 	uint32_t temp = 0;		\
										SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);	\
										temp = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);\
										UNUSED(temp);	\
									} while(0)

#define RCC_USART1_CLK_ENABLE()   do { uint32_t temp =  0;		\
									SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);			\
									temp = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);		\
									UNUSED(temp);			\
									}while(0)

#define RCC_USART6_CLK_ENABLE()   do { uint32_t temp =  0;		\
									SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN);			\
									temp = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN);		\
									UNUSED(temp);			\
									}while(0)


#define RCC_SPI1_CLK_DISABLE()		CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN)

#define RCC_SYSCFG_CLK_DISABLE()    CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN)

#define RCC_USART1_CLK_DISABLE()    CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN)
#define RCC_USART6_CLK_DISABLE()    CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN)



/*
 * Functions Declaretion
 */

uint32_t RCC_GetSystemClock(void);
uint32_t RCC_GetHClock(void);
uint32_t RCC_GetPCLK1(void);
uint32_t RCC_GetPCLK2(void);



#endif /* INC_RCC_H_ */
