# STM32F407 Peripheral Libraries

This repository contains custom libraries for working with the STM32F407 microcontroller. The libraries provide abstractions and functionalities for configuring and using the following peripherals:

- **EXTI (External Interrupt)**
- **GPIO (General Purpose Input/Output)**
- **SPI (Serial Peripheral Interface)**
- **USART (Universal Synchronous/Asynchronous Receiver/Transmitter)**
- **RCC (Reset and Clock Control)**

Additionally, a device-specific header file (`stm32f407xx.h`) is included to define the peripheral registers, bit definitions, and clock addresses specific to the STM32F407.

---

## Device-Specific Header File: `stm32f407xx.h`
This header file includes:
- Base addresses for STM32F407 peripherals.
- Register definitions for GPIO, USART, SPI, EXTI, RCC, and other peripherals.
- Bitfield definitions and macros to manipulate specific bits in registers.

### Example Usage:
```c
#include "stm32f407xx.h"

// Enable GPIOA clock
SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
```

---

## Libraries Overview

### 1. EXTI (External Interrupt)
The EXTI library provides functionality to configure and use external interrupt lines.

#### Key Functions:
- **`EXTI_LineConfig(uint8_t EXTI_PortSource, uint8_t EXTI_LineSource)`**: Configures the port and line for the external interrupt.
- **`EXTI_Init(EXTI_InitTypeDef_t* EXTI_InitStruct)`**: Initializes the EXTI line with the specified parameters.
- **`NVIC_EnableInterrupt(IRQNumber_Typedef_t IRQNumber)`**: Enables the specified interrupt in the NVIC.

#### Example Usage:
```c
EXTI_InitTypeDef_t extiConfig;

extiConfig.EXTI_LineNumber = EXTI_Line0;
extiConfig.TriggerSelection = EXTI_Trigger_Rising;
extiConfig.EXTI_Mode = EXTI_Mode_Interrupt;
extiConfig.EXTI_LineCmd = ENABLE;

EXTI_Init(&extiConfig);
EXTI_LineConfig(EXTI_PortSourceGPIOA, EXTI_Line0);
NVIC_EnableInterrupt(EXTI0_IRQn);
```

---

### 2. GPIO (General Purpose Input/Output)
The GPIO library provides functionality to configure and control GPIO pins.

#### Key Functions:
- **`GPIO_Init(GPIO_TypeDef_t* GPIOx, GPIO_InitTypeDef_t *GPIO_Init)`**: Initializes the GPIO pin with the specified parameters.
- **`GPIO_WritePin(GPIO_TypeDef_t* GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState)`**: Sets the state of a GPIO pin.
- **`GPIO_ReadPin(GPIO_TypeDef_t* GPIOx, uint16_t pinNumber)`**: Reads the state of a GPIO pin.

#### Example Usage:
```c
GPIO_InitTypeDef_t gpioConfig;

gpioConfig.pinNumber = GPIO_PIN_5;
gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
gpioConfig.PuPd = GPIO_NOPULL;
gpioConfig.Speed = GPIO_SPEED_HIGH;

GPIO_Init(GPIOA, &gpioConfig);
GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
```

---

### 3. SPI (Serial Peripheral Interface)
The SPI library enables the initialization and communication over the SPI protocol.

#### Key Functions:
- **`SPI_Init(SPI_HandleTypedef_t* SPI_Handle)`**: Initializes the SPI peripheral with the specified parameters.
- **`SPI_TransmitData(SPI_HandleTypedef_t* SPI_Handle, uint8_t *pdata, uint16_t sizeOfData)`**: Sends data over SPI.
- **`SPI_ReceiveData(SPI_HandleTypedef_t* SPI_Handle, uint8_t *pBuffer, uint16_t sizeOfData)`**: Receives data over SPI.

#### Example Usage:
```c
SPI_HandleTypedef_t spiConfig;

spiConfig.Init.Mode = SPI_MODE_MASTER;
spiConfig.Init.BaudRate = SPI_BAUDRATE_DIV16;
spiConfig.Init.CPOL = SPI_CPOL_LOW;
spiConfig.Init.CPHA = SPI_CPHA_1EDGE;

SPI_Init(&spiConfig);
uint8_t dataToSend = 0x5A;
SPI_TransmitData(&spiConfig, &dataToSend, 1);
```

---

### 4. USART (Universal Synchronous/Asynchronous Receiver/Transmitter)
The USART library allows for communication over serial interfaces.

#### Key Functions:
- **`USART_Init(USART_HandleTypedef_t* USART_Handle)`**: Initializes the USART peripheral with the specified parameters.
- **`USART_TransmitData(USART_HandleTypedef_t* USART_Handle, uint8_t *pData, uint16_t dataSize)`**: Sends data over USART.
- **`USART_ReceiveData(USART_HandleTypedef_t* USART_Handle, uint8_t *pBuffer, uint16_t dataSize)`**: Receives data over USART.

#### Example Usage:
```c
USART_HandleTypedef_t usartConfig;

usartConfig.Init.BaudRate = 115200;
usartConfig.Init.WordLength = USART_WORDLENGTH_8B;
usartConfig.Init.StopBits = USART_STOPBITS_1;
usartConfig.Init.Parity = USART_PARITY_NONE;
usartConfig.Init.Mode = USART_MODE_TX_RX;

USART_Init(&usartConfig);
uint8_t message[] = "Hello, USART!";
USART_TransmitData(&usartConfig, message, sizeof(message));
```

---

### 5. RCC (Reset and Clock Control)
The RCC library provides functions to configure and retrieve clock information for the STM32F407 microcontroller.

#### Key Functions:
- **`RCC_GetSystemClock()`**: Returns the system clock frequency.
- **`RCC_GetHClock()`**: Returns the AHB bus clock frequency.
- **`RCC_GetPCLK1()`**: Returns the APB1 bus clock frequency.
- **`RCC_GetPCLK2()`**: Returns the APB2 bus clock frequency.

#### Example Usage:
```c
uint32_t systemClock = RCC_GetSystemClock();
uint32_t hClock = RCC_GetHClock();
```

---

## How to Use
1. Clone this repository:
   ```bash
   git clone https://github.com/ibrahimozcakir/STM32F407_Driver_Development
   ```
2. Include the required library header files in your project.
3. Follow the provided examples to configure and use the peripherals in your STM32F407 project.

## Requirements
- STM32F407 microcontroller.
- STM32CubeIDE or any compatible IDE for development.
- Basic knowledge of embedded C programming.

---

## Contributions
Feel free to fork this repository, make changes, and submit a pull request. Contributions are welcome!

