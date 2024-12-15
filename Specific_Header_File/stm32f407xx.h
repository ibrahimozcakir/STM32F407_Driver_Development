#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

/*
 * Microprocessor Defines
 *
 */

#define NVIC_ISER0 				((uint32_t*)(0xE000E100UL))





#define __IO volatile

#define SET_BIT(REG, BIT)          ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)		   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)         ((REG) & (BIT))
#define UNUSED(x)                  ((void)(x))

typedef enum {

	DISABLE = 0x0U,
	ENABLE = !DISABLE

}FunctionalState_t;

/*
 *  IRQ Numbers of MCU == Vector Table
 *
 */

typedef enum {

	EXTI0_IRQNumber = 6,
	EXTI1_IRQNumber = 7,
	EXTI2_IRQNumber = 8,
	EXTI3_IRQNumber = 9,
	SPI1_IRQNumber = 35,
	USART2_IRQNumber = 38

}IRQNumber_Typedef_t;

/*
 *  Memory Base Address
 *
 */

#define FLASH_BASE_ADDR          	 	(0x08000000UL)   /* Flash Base Address (up to 1 MB) */
#define SRAM1_BASE_ADDR				 	(0x20000000UL)	 /* SRAM1 Base Address 112 KB       */
#define SRAM2_BASE_ADDR				 	(0x2001C000UL)   /* SRAM2 Base Address 16 KB        */

/*
 *  Peripheral Base Address
 *
 */

#define PERIPH_BASE_ADDR             	(0x40000000UL)                    /* Base Address for All peripherals  */

#define APB1_BASE_ADDR               	(PERIPH_BASE_ADDR)                /* APB1 Bus Domain Base Address      */
#define APB2_BASE_ADDR               	(PERIPH_BASE_ADDR + 0x00010000UL) /* APB2 Bus Domain Base Address      */
#define AHB1_BASE_ADDR				 	(PERIPH_BASE_ADDR + 0x00020000UL) /* AHB1 Bus Domain Base Address      */
#define AHB2_BASE_ADDR				 	(PERIPH_BASE_ADDR + 0x10000000UL) /* AHB2 Bus Domain Base Address      */

/*
 *  APB1 Peripheral Base Address
 *
 */

#define TIM2_BASE_ADDR             		(APB1_BASE_ADDR + 0x00000000UL)    /* Timer 2 Base Address       */
#define TIM3_BASE_ADDR             		(APB1_BASE_ADDR + 0x00000400UL)    /* Timer 3 Base Address       */
#define TIM4_BASE_ADDR             		(APB1_BASE_ADDR + 0x00000800UL)    /* Timer 4 Base Address       */
#define TIM5_BASE_ADDR             		(APB1_BASE_ADDR + 0x00000C00UL)    /* Timer 5 Base Address       */
#define TIM6_BASE_ADDR             		(APB1_BASE_ADDR + 0x00001000UL)    /* Timer 6 Base Address       */
#define TIM7_BASE_ADDR             		(APB1_BASE_ADDR + 0x00001400UL)    /* Timer 7 Base Address       */
#define TIM12_BASE_ADDR            		(APB1_BASE_ADDR + 0x00001800UL)    /* Timer 12 Base Address      */
#define TIM13_BASE_ADDR            		(APB1_BASE_ADDR + 0x00001C00UL)    /* Timer 13 Base Address      */
#define TIM14_BASE_ADDR            		(APB1_BASE_ADDR + 0x00002000UL)    /* Timer 14 Base Address      */

#define SPI2_BASE_ADDR			   		(APB1_BASE_ADDR + 0x00003800UL)	   /* SPI 2 Base Address         */
#define SPI3_BASE_ADDR			   		(APB1_BASE_ADDR + 0x00003C00UL)	   /* SPI 3 Base Address         */

#define USART2_BASE_ADDR		   		(APB1_BASE_ADDR + 0x00004400UL)	   /* USART 2 Base Address       */
#define USART3_BASE_ADDR		   		(APB1_BASE_ADDR + 0x00004800UL)	   /* USART 3 Base Address       */
#define UART4_BASE_ADDR		       		(APB1_BASE_ADDR + 0x00004C00UL)	   /* UART 4 Base Address        */
#define UART5_BASE_ADDR		       		(APB1_BASE_ADDR + 0x00005000UL)	   /* UART 5 Base Address        */
#define UART7_BASE_ADDR		       		(APB1_BASE_ADDR + 0x00007800UL)	   /* UART 7 Base Address        */
#define UART8_BASE_ADDR		       		(APB1_BASE_ADDR + 0x00007C00UL)	   /* UART 8 Base Address        */

#define I2C1_BASE_ADDR            	 	(APB1_BASE_ADDR + 0x00005400UL)	   /* I2C 1 Base Address         */
#define I2C2_BASE_ADDR             		(APB1_BASE_ADDR + 0x00005800UL)	   /* I2C 2 Base Address         */
#define I2C3_BASE_ADDR             		(APB1_BASE_ADDR + 0x00005C00UL)	   /* I2C 3 Base Address         */

#define DAC_BASE_ADDR             		(APB1_BASE_ADDR + 0x00007400UL)	   /* DAC Base Address           */

/*
 *  APB2 Peripheral Base Address
 *
 */

#define TIM1_BASE_ADDR             		(APB2_BASE_ADDR + 0x00000000UL)    /* Timer 1 Base Address       */
#define TIM8_BASE_ADDR             		(APB2_BASE_ADDR + 0x00000400UL)    /* Timer 8 Base Address       */

#define USART1_BASE_ADDR		   		(APB2_BASE_ADDR + 0x00001000UL)	   /* USART 1 Base Address       */
#define USART6_BASE_ADDR		   		(APB2_BASE_ADDR + 0x00001400UL)	   /* USART 6 Base Address       */

#define ADC1_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00002000UL)	   /* ADC 1 Base Address         */
#define ADC2_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00002100UL)	   /* ADC 2 Base Address         */
#define ADC3_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00002200UL)	   /* ADC 3 Base Address         */

#define SPI1_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00003000UL)	   /* SPI 1 Base Address         */
#define SPI4_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00003400UL)	   /* SPI 4 Base Address         */
#define SPI5_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00005000UL)	   /* SPI 5 Base Address         */
#define SPI6_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00005400UL)	   /* SPI 6 Base Address         */

#define SYSCFG_BASE_ADDR		   		(APB2_BASE_ADDR + 0x00003800UL)	   /* SYSCFG Base Address        */

#define EXTI_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00003C00UL)	   /* EXTI Base Address          */

#define TIM9_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00004000UL)	   /* Timer 9 Base Address       */
#define TIM10_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00004400UL)	   /* Timer 10 Base Address      */
#define TIM11_BASE_ADDR		       		(APB2_BASE_ADDR + 0x00004800UL)	   /* Timer 11 Base Address      */

/*
 *  AHB1 Peripheral Base Address
 *
 */

#define GPIOA_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00000000UL)    /* GPIOA Base Address                          */
#define GPIOB_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00000400UL)    /* GPIOB Base Address                          */
#define GPIOC_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00000800UL)    /* GPIOC Base Address                          */
#define GPIOD_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00000C00UL)    /* GPIOD Base Address                          */
#define GPIOE_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00001000UL)    /* GPIOE Base Address                          */
#define GPIOF_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00001400UL)    /* GPIOF Base Address                          */
#define GPIOG_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00001800UL)    /* GPIOG Base Address                          */
#define GPIOH_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00001C00UL)    /* GPIOH Base Address                          */
#define GPIOI_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00002000UL)    /* GPIOI Base Address                          */
#define GPIOJ_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00002400UL)    /* GPIOJ Base Address                          */
#define GPIOK_BASE_ADDR           		(AHB1_BASE_ADDR + 0x00002800UL)    /* GPIOK Base Address                          */

#define CRC_BASE_ADDR             		(AHB1_BASE_ADDR + 0x00003000UL)    /* CRC Base Address                            */
#define RCC_BASE_ADDR             		(AHB1_BASE_ADDR + 0x00003800UL)    /* RCC Base Address                            */

#define FLASHIFR_BASE_ADDR        		(AHB1_BASE_ADDR + 0x00003C00UL)    /* Flash interface register Base Address       */
#define BKPSRAM_BASE_ADDR         		(AHB1_BASE_ADDR + 0x00004000UL)    /* BKPSRAM Base Address       				  */

#define DMA1_BASE_ADDR            		(AHB1_BASE_ADDR + 0x00006000UL)    /* DMA 1 Base Address       				   	  */
#define DMA2_BASE_ADDR            		(AHB1_BASE_ADDR + 0x00006400UL)    /* DMA 2 Base Address       				      */

/*
 *  Peripheral GPIO Structure Definitions
 *
 */

typedef struct {

	 __IO uint32_t MODER;        /*!< GPIO port mode register 					Address Offset = 0x0000 		 */
	 __IO uint32_t OTYPER;       /*!< GPIO port output type register 			Address Offset = 0x0004 		 */
	 __IO uint32_t OSPEEDR; 	 /*!< GPIO port output speed register 			Address Offset = 0x0008 		 */
	 __IO uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register  	Address Offset = 0x000C 		 */
	 __IO uint32_t IDR;			 /*!< GPIO port input data register 			Address Offset = 0x0010 		 */
	 __IO uint32_t ODR;			 /*!< GPIO port output data register 			Address Offset = 0x0014 		 */
	 __IO uint32_t BSRR;         /*!< GPIO port bit set/reset register 			Address Offset = 0x0018 		 */
	 __IO uint32_t LCKR;		 /*!< GPIO port configuration lock register 	Address Offset = 0x001C			 */
	 __IO uint32_t AFR[2];		 /*!< GPIO alternate function register 			Address Offset = 0x0020 - 0x0024 */

} GPIO_TypeDef_t;



/*
 *  RCC Structure Definitions
 *
 */

typedef struct {
	__IO uint32_t CR;  				/*!< RCC clock control register 									  Address Offset = 0x0000 		   */
	__IO uint32_t PLLCFGR;			/*!< RCC PLL configuration register  								  Address Offset = 0x0004 		   */
	__IO uint32_t CFGR;				/*!< RCC clock configuration register 								  Address Offset = 0x0008 		   */
	__IO uint32_t CIR;				/*!< RCC clock interrupt register 				    				  Address Offset = 0x000C 		   */
	__IO uint32_t AHB1RSTR;			/*!< RCC AHB1 peripheral reset register 		    				  Address Offset = 0x0010 		   */
	__IO uint32_t AHB2RSTR;			/*!< RCC AHB2 peripheral reset register 		   			 		  Address Offset = 0x0014 		   */
	__IO uint32_t AHB3RSTR;    	 	/*!< RCC AHB3 peripheral reset register 		   				 	  Address Offset = 0x0018 		   */
	     uint32_t RESERVED0;   		/*!< RESERVED register 		                        				  Address Offset = 0x001C 		   */
	__IO uint32_t APB1RSTR;			/*!< RCC APB1 peripheral reset register  		   			 		  Address Offset = 0x0020 		   */
	__IO uint32_t APB2RSTR;			/*!< RCC APB2 peripheral reset register  		    				  Address Offset = 0x0024 		   */
	     uint32_t RESERVED1[2];	 	/*!< RESERVED register 		                       					  Address Offset = 0x0028 - 0x002C */
	__IO uint32_t AHB1ENR;			/*!< RCC AHB1 peripheral clock enable register      				  Address Offset = 0x0030 		   */
	__IO uint32_t AHB2ENR;			/*!< RCC AHB2 peripheral clock enable register      				  Address Offset = 0x0034 		   */
	__IO uint32_t AHB3ENR;			/*!< RCC AHB3 peripheral clock enable register      				  Address Offset = 0x0038 		   */
		 uint32_t RESERVED2;		/*!< RESERVED register 		                        				  Address Offset = 0x003C          */
	__IO uint32_t APB1ENR;          /*!< RCC APB1 peripheral clock enable register      				  Address Offset = 0x0040 		   */
	__IO uint32_t APB2ENR; 			/*!< RCC APB2 peripheral clock enable register      				  Address Offset = 0x0044 		   */
	     uint32_t RESERVED3[2];     /*!< RESERVED register 		                        				  Address Offset = 0x0048 - 0x004C */
	__IO uint32_t AHB1LPENR;        /*!< RCC AHB1 peripheral clock enable in low power mode register      Address Offset = 0x0050 		   */
	__IO uint32_t AHB2LPENR;        /*!< RCC AHB2 peripheral clock enable in low power mode register      Address Offset = 0x0054 		   */
	__IO uint32_t AHB3LPENR;	    /*!< RCC AHB3 peripheral clock enable in low power mode register      Address Offset = 0x0058 		   */
	__IO uint32_t RESERVED4;		/*!< RESERVED register 		                                          Address Offset = 0x005C          */
	__IO uint32_t APB1LPENR;		/*!< RCC APB1 peripheral clock enable in low power mode register      Address Offset = 0x0060 		   */
	__IO uint32_t APB2LPENR;        /*!< RCC AHB2 peripheral clock enable in low power mode register      Address Offset = 0x0064 		   */
	     uint32_t RESERVED5[2];     /*!< RESERVED register 		                         				  Address Offset = 0x0068 - 0x006C */
	__IO uint32_t BDCR;             /*!< RCC Backup domain control register        						  Address Offset = 0x0070 		   */
	__IO uint32_t CSR;              /*!< RCC clock control & status register       						  Address Offset = 0x0074 		   */
	     uint32_t RESERVED6[2];     /*!< RESERVED register 		                        				  Address Offset = 0x0078 - 0x007C */
	__IO uint32_t SSCGR;            /*!< RCC spread spectrum clock generation register       			  Address Offset = 0x0080 		   */
	__IO uint32_t PLLI2SCFGR;       /*!< RCC PLLI2S configuration register       						  Address Offset = 0x0084 		   */

}RCC_TypeDef_t;



/*
 *  SYSCFG Structure Definitions
 *
 */

typedef struct {

	__IO uint32_t MEMRMP;			/*!< SYSCFG memory remap register 									  Address Offset = 0x0000 		   */
	__IO uint32_t PMC;				/*!< SYSCFG peripheral mode configuration register 					  Address Offset = 0x0004 		   */
	__IO uint32_t EXTI_CR[4];		/*!< SYSCFG external interrupt configuration register 1...4			  Address Offset = 0x0008 - 0x0014 */
	uint32_t RESERVED[2];			/*!< RESERVED register 		                        		          Address Offset = 0x0018 - 0x001C */
	__IO uint32_t CMPCR;			/*!< Compensation cell control register  							  Address Offset = 0x0020 		   */

}SYSCFG_Typedef_t;


/*
 *  EXTI Structure Definitions
 *
 */

typedef struct {

	__IO uint32_t IMR;		        /*!< Interrupt mask register  				 Address Offset = 0x0000 		   */
	__IO uint32_t EMR;			    /*!< Event mask register 					 Address Offset = 0x0004 		   */
	__IO uint32_t RTSR;				/*!< Rising trigger selection register 		 Address Offset = 0x0008 		   */
	__IO uint32_t FTSR;				/*!< Falling trigger selection register 	 Address Offset = 0x000C 		   */
	__IO uint32_t SWIER;			/*!< Software interrupt event register 		 Address Offset = 0x0010 		   */
	__IO uint32_t PR;				/*!< Pending register 						 Address Offset = 0x0014 		   */

}EXTI_Typedef_t;


/*
 *  SPI Structure Definitions
 *
 */

typedef struct {

	__IO uint32_t CR1;			    /*!<  SPI control register 1  				 Address Offset = 0x0000 	 */
	__IO uint32_t CR2;				/*!<  SPI control register 2 				 Address Offset = 0x0004 	 */
	__IO uint32_t SR;				/*!<  SPI status register  				 	 Address Offset = 0x0008     */
	__IO uint32_t DR;				/*!<  SPI data register  				 	 Address Offset = 0x000C     */
	__IO uint32_t CRCPR;			/*!<  SPI CRC polynomial register  			 Address Offset = 0x0010     */
	__IO uint32_t RXCRCR;			/*!<  SPI RX CRC register  				 	 Address Offset = 0x0014     */
	__IO uint32_t TXCRCR;			/*!<  SPI TX CRC register  				 	 Address Offset = 0x0018     */
	__IO uint32_t I2SCFGR;			/*!<  SPI_I2S configuration register  		 Address Offset = 0x001C     */
	__IO uint32_t I2SPR;			/*!<  SPI_I2S prescaler register  			 Address Offset = 0x0020     */

}SPI_TypeDef_t;



/*
 *  USART Structure Definitions
 *
 */

typedef struct {

	__IO uint32_t SR;				/*!<  USART Status register   				 	 	Address Offset = 0x0000 	 */
	__IO uint32_t DR;				/*!<  USART Data register  		     		 	 	Address Offset = 0x0004 	 */
	__IO uint32_t BRR;				/*!<  USART Baud rate register    			 	  	Address Offset = 0x0008 	 */
	__IO uint32_t CR1;				/*!<  USART Control register 1   				 	Address Offset = 0x000C 	 */
	__IO uint32_t CR2;				/*!<  USART Control register 2   				 	Address Offset = 0x0010 	 */
	__IO uint32_t CR3;				/*!<  USART Control register 3   				 	Address Offset = 0x0014 	 */
	__IO uint32_t GTPR;				/*!<  USART Guard time and prescaler register     	Address Offset = 0x0018 	 */

}USART_Typedef_t;



/*
 *  I2C Structure Definitions
 *
 */

typedef struct {

	__IO uint32_t CR1;				/*!<  I2C Control register 1   				 	 	Address Offset = 0x0000 	 */
	__IO uint32_t CR2;				/*!<  I2C Control register 2   				 	 	Address Offset = 0x0004 	 */
	__IO uint32_t OAR1;				/*!<  I2C Own address register 1    				Address Offset = 0x0008 	 */
	__IO uint32_t OAR2;				/*!<  I2C Own address register 2   				 	Address Offset = 0x000C 	 */
	__IO uint32_t DR;				/*!<  I2C Data register    				 	 		Address Offset = 0x0010 	 */
	__IO uint32_t SR1;				/*!<  I2C Status register 1   				 	 	Address Offset = 0x0014 	 */
	__IO uint32_t SR2;				/*!<  I2C Status register 2   				 	 	Address Offset = 0x0018 	 */
	__IO uint32_t CCR;				/*!<  I2C Clock control register   				 	Address Offset = 0x001C 	 */
	__IO uint32_t TRISE;			/*!<  I2C TRISE register   				 	 		Address Offset = 0x0020 	 */
	__IO uint32_t FLTR;				/*!<  I2C FLTR register   				 	 		Address Offset = 0x0024 	 */

}I2C_Typedef_t;


/*
 *Bases address definations of ports
 */

#define GPIOA							((GPIO_TypeDef_t*)(GPIOA_BASE_ADDR))
#define GPIOB							((GPIO_TypeDef_t*)(GPIOB_BASE_ADDR))
#define GPIOC							((GPIO_TypeDef_t*)(GPIOC_BASE_ADDR))
#define GPIOD							((GPIO_TypeDef_t*)(GPIOD_BASE_ADDR))
#define GPIOE							((GPIO_TypeDef_t*)(GPIOE_BASE_ADDR))
#define GPIOF	 						((GPIO_TypeDef_t*)(GPIOF_BASE_ADDR))
#define GPIOG							((GPIO_TypeDef_t*)(GPIOG_BASE_ADDR))
#define GPIOH							((GPIO_TypeDef_t*)(GPIOH_BASE_ADDR))
#define GPIOI							((GPIO_TypeDef_t*)(GPIOI_BASE_ADDR))
#define GPIOJ							((GPIO_TypeDef_t*)(GPIOJ_BASE_ADDR))
#define GPIOK							((GPIO_TypeDef_t*)(GPIOK_BASE_ADDR))

#define RCC								((RCC_TypeDef_t*)(RCC_BASE_ADDR))

#define SYSCFG 							((SYSCFG_Typedef_t*)(SYSCFG_BASE_ADDR))

#define EXTI 							((EXTI_Typedef_t*)(EXTI_BASE_ADDR))

#define SPI1							((SPI_TypeDef_t*)(SPI1_BASE_ADDR))
#define SPI2							((SPI_TypeDef_t*)(SPI2_BASE_ADDR))
#define SPI3							((SPI_TypeDef_t*)(SPI3_BASE_ADDR))
#define SPI4							((SPI_TypeDef_t*)(SPI4_BASE_ADDR))
#define SPI5							((SPI_TypeDef_t*)(SPI5_BASE_ADDR))
#define SPI6							((SPI_TypeDef_t*)(SPI6_BASE_ADDR))

#define USART1 							((USART_Typedef_t*)USART1_BASE_ADDR)
#define USART2 							((USART_Typedef_t*)USART2_BASE_ADDR)
#define USART3 							((USART_Typedef_t*)USART3_BASE_ADDR)
#define USART6 							((USART_Typedef_t*)USART6_BASE_ADDR)

#define UART4							((USART_Typedef_t*)UART4_BASE_ADDR)
#define UART5							((USART_Typedef_t*)UART5_BASE_ADDR)
#define UART7							((USART_Typedef_t*)UART7_BASE_ADDR)
#define UART8							((USART_Typedef_t*)UART8_BASE_ADDR)


#define I2C1							((I2C_Typedef_t*)I2C1_BASE_ADDR)
#define I2C2							((I2C_Typedef_t*)I2C2_BASE_ADDR)
#define I2C3							((I2C_Typedef_t*)I2C3_BASE_ADDR)


/*
 * Bit Definations
 */
#define RCC_AHB1ENR_GPIOAEN_POS         (0U)         					       /* RCC AHB1ENR register GPIOAEN Bit Position     */
#define RCC_AHB1ENR_GPIOAEN_MSK         (0x1 << RCC_AHB1ENR_GPIOAEN_POS )      /* RCC AHB1ENR register GPIOAEN Bit Mask         */
#define RCC_AHB1ENR_GPIOAEN				(RCC_AHB1ENR_GPIOAEN_MSK)			   /* RCC AHB1ENR register GPIOAEN Macro            */

#define RCC_AHB1ENR_GPIOBEN_POS         (1U)         					       /* RCC AHB1ENR register GPIOBEN Bit Position     */
#define RCC_AHB1ENR_GPIOBEN_MSK         (0x1 << RCC_AHB1ENR_GPIOBEN_POS )      /* RCC AHB1ENR register GPIOBEN Bit Mask         */
#define RCC_AHB1ENR_GPIOBEN				(RCC_AHB1ENR_GPIOBEN_MSK)			   /* RCC AHB1ENR register GPIOBEN Macro            */

#define RCC_AHB1ENR_GPIOCEN_POS         (2U)         					       /* RCC AHB1ENR register GPIOCEN Bit Position     */
#define RCC_AHB1ENR_GPIOCEN_MSK         (0x1 << RCC_AHB1ENR_GPIOCEN_POS )      /* RCC AHB1ENR register GPIOCEN Bit Mask         */
#define RCC_AHB1ENR_GPIOCEN				(RCC_AHB1ENR_GPIOCEN_MSK)			   /* RCC AHB1ENR register GPIOCEN Macro            */

#define RCC_AHB1ENR_GPIODEN_POS         (3U)         					       /* RCC AHB1ENR register GPIODEN Bit Position     */
#define RCC_AHB1ENR_GPIODEN_MSK         (0x1 << RCC_AHB1ENR_GPIODEN_POS )      /* RCC AHB1ENR register GPIODEN Bit Mask         */
#define RCC_AHB1ENR_GPIODEN				(RCC_AHB1ENR_GPIODEN_MSK)			   /* RCC AHB1ENR register GPIODEN Macro            */

#define RCC_AHB1ENR_GPIOEEN_POS         (4U)         					       /* RCC AHB1ENR register GPIOEEN Bit Position     */
#define RCC_AHB1ENR_GPIOEEN_MSK         (0x1 << RCC_AHB1ENR_GPIOEEN_POS )      /* RCC AHB1ENR register GPIOEEN Bit Mask         */
#define RCC_AHB1ENR_GPIOEEN				(RCC_AHB1ENR_GPIOEEN_MSK)			   /* RCC AHB1ENR register GPIOEEN Macro            */



#define RCC_APB1ENR_SPI2EN_POS			(14U)								   /* RCC APB1ENR register SPI2EN Bit Position      */
#define RCC_APB1ENR_SPI2EN_MSK			(0x1 << RCC_APB1ENR_SPI2EN_POS)		   /* RCC APB1ENR register SPI2EN Bit Mask          */
#define RCC_APB1ENR_SPI2EN				(RCC_APB1ENR_SPI2EN_MSK)			   /* RCC APB1ENR register SPI2EN Macro             */

#define RCC_APB1ENR_SPI3EN_POS			(15U)								   /* RCC APB1ENR register SPI3EN Bit Position      */
#define RCC_APB1ENR_SPI3EN_MSK			(0x1 << RCC_APB1ENR_SPI3EN_POS)		   /* RCC APB1ENR register SPI3EN Bit Mask          */
#define RCC_APB1ENR_SPI3EN				(RCC_APB1ENR_SPI3EN_MSK)			   /* RCC APB1ENR register SPI3EN Macro      	    */

#define RCC_APB1ENR_USART2EN_POS		(17U)								   /* RCC APB1ENR register USART2EN Bit Position    */
#define RCC_APB1ENR_USART2EN_MSK		(0x1 << RCC_APB1ENR_USART2EN_POS)	   /* RCC APB1ENR register USART2EN Bit Mask        */
#define RCC_APB1ENR_USART2EN			(RCC_APB1ENR_USART2EN_MSK)			   /* RCC APB1ENR register USART2EN Macro           */

#define RCC_APB1ENR_USART3EN_POS		(18U)								   /* RCC APB1ENR register USART3EN Bit Position    */
#define RCC_APB1ENR_USART3EN_MSK		(0x1 << RCC_APB1ENR_USART3EN_POS)	   /* RCC APB1ENR register USART3EN Bit Mask        */
#define RCC_APB1ENR_USART3EN			(RCC_APB1ENR_USART3EN_MSK)			   /* RCC APB1ENR register USART3EN Macro           */

#define RCC_APB1ENR_UART4EN_POS			(19U)								   /* RCC APB1ENR register UART4EN Bit Position     */
#define RCC_APB1ENR_UART4EN_MSK			(0x1 << RCC_APB1ENR_UART4EN_POS)   	   /* RCC APB1ENR register UART4EN Bit Mask         */
#define RCC_APB1ENR_UART4EN				(RCC_APB1ENR_UART4EN_MSK)			   /* RCC APB1ENR register UART4EN Macro            */

#define RCC_APB1ENR_UART5EN_POS			(20U)								   /* RCC APB1ENR register UART5EN Bit Position     */
#define RCC_APB1ENR_UART5EN_MSK			(0x1 << RCC_APB1ENR_UART5EN_POS)	   /* RCC APB1ENR register UART5EN Bit Mask         */
#define RCC_APB1ENR_UART5EN				(RCC_APB1ENR_UART5EN_MSK)			   /* RCC APB1ENR register UART5EN Macro            */


#define RCC_APB2ENR_SYSCFGEN_POS		(14U)								   /* RCC APB2ENR register SYSCFGEN Bit Position    */
#define RCC_APB2ENR_SYSCFGEN_MSK		(0x1 << RCC_APB2ENR_SYSCFGEN_POS)	   /* RCC APB2ENR register SYSCFGEN Bit Mask        */
#define RCC_APB2ENR_SYSCFGEN			(RCC_APB2ENR_SYSCFGEN_MSK)			   /* RCC APB2ENR register SYSCFGEN Bit Macro       */

#define RCC_APB2ENR_SPI1EN_POS			(12U)								   /* RCC APB2ENR register SPI1EN Bit Position      */
#define RCC_APB2ENR_SPI1EN_MSK			(0x1 << RCC_APB2ENR_SPI1EN_POS)		   /* RCC APB2ENR register SPI1EN Bit Mask          */
#define RCC_APB2ENR_SPI1EN 				(RCC_APB2ENR_SPI1EN_MSK)			   /* RCC APB2ENR register SPI1EN Bit Macro         */

#define RCC_APB2ENR_USART1EN_POS		(4U)								   /* RCC APB2ENR register USART1EN Bit Position    */
#define RCC_APB2ENR_USART1EN_MSK		(0x1 << RCC_APB2ENR_USART1EN_POS)	   /* RCC APB2ENR register USART1EN Bit Mask        */
#define RCC_APB2ENR_USART1EN			(RCC_APB2ENR_USART1EN_MSK)			   /* RCC APB2ENR register USART1EN Bit Macro       */

#define RCC_APB2ENR_USART6EN_POS		(5U)								   /* RCC APB2ENR register USART6EN Bit Position    */
#define RCC_APB2ENR_USART6EN_MSK		(0x1 << RCC_APB2ENR_USART6EN_POS)	   /* RCC APB2ENR register USART6EN Bit Mask        */
#define RCC_APB2ENR_USART6EN			(RCC_APB2ENR_USART6EN_MSK)			   /* RCC APB2ENR register USART6EN Bit Macro       */


#define RCC_APB1ENR_I2C1EN_POS			(21U)								   /* RCC APB2ENR register I2C1EN Bit Position      */
#define RCC_APB1ENR_I2C1EN_MSK			(0x1 << RCC_APB1ENR_I2C1EN_POS)		   /* RCC APB2ENR register I2C1EN Bit Mask          */
#define RCC_APB1ENR_I2C1EN				(RCC_APB1ENR_I2C1EN_MSK)			   /* RCC APB2ENR register I2C1EN Bit Macro         */

#define RCC_APB1ENR_I2C2EN_POS			(22U)								   /* RCC APB2ENR register I2C2EN Bit Position      */
#define RCC_APB1ENR_I2C2EN_MSK			(0x1 << RCC_APB1ENR_I2C2EN_POS)		   /* RCC APB2ENR register I2C2EN Bit Mask          */
#define RCC_APB1ENR_I2C2EN				(RCC_APB1ENR_I2C2EN_MSK)			   /* RCC APB2ENR register I2C2EN Bit Macro         */

#define RCC_APB1ENR_I2C3EN_POS			(23U)								   /* RCC APB2ENR register I2C1EN Bit Position      */
#define RCC_APB1ENR_I2C3EN_MSK			(0x1 << RCC_APB1ENR_I2C3EN_POS)		   /* RCC APB2ENR register I2C2EN Bit Mask          */
#define RCC_APB1ENR_I2C3EN				(RCC_APB1ENR_I2C3EN_MSK)			   /* RCC APB2ENR register I2C2EN Bit Macro         */





#define SPI_CR1_SPE						(6U)
#define SPI_CR1_DFF						(11U)

#define SPI_SR_RXNE						(0U)
#define SPI_SR_TXE						(1U)
#define SPI_SR_CHSIDE					(2U)
#define SPI_SR_UDR						(3U)
#define SPI_SR_CRCERR					(4U)
#define SPI_SR_MODF 					(5U)
#define SPI_SR_OVR						(6U)
#define SPI_SR_BSY						(7U)
#define SPI_SR_FRE						(8U)

#define SPI_CR2_TXEIE					(7U)
#define SPI_CR2_RXNEIE					(6U)

#define USART_SR_RXNE					(5U)
#define USART_SR_TC						(6U)
#define USART_SR_TXE					(7U)
#define USART_CR2_STOP  				(12U)
#define USART_CR1_UE					(13U)
#define USART_CR1_TXEIE					(7U)
#define USART_CR1_RXNEIE				(5U)

#define RCC_CFGR_SWS					(2U)
#define RCC_CFGR_HPRE					(4U)
#define RCC_CFGR_PPRE1					(10U)
#define RCC_CFGR_PPRE2					(13U)

#define I2C_CR1_PE						(0U)

/*
 * Flag Definations
 */

#define SPI_FLAG_TXE					(0x1U << SPI_SR_TXE)
#define SPI_FLAG_BSY					(0x1U << SPI_SR_BSY)
#define SPI_FLAG_RXNE					(0x1U << SPI_SR_RXNE)

#define USART_FLAG_TXE					(0x1U << USART_SR_TXE)
#define USART_FLAG_TC					(0x1U << USART_SR_TC)
#define USART_FLAG_RXNE					(0x1U << USART_SR_RXNE)

#include "rcc.h"
#include "gpio.h"
#include "exti.h"
#include "spi.h"
#include "usart.h"
#include "i2c.h"






































#endif /* INC_STM32F407XX_H_ */
