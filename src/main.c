/**
 ******************************************************************************
 * @file    main.c
 * @author  Prekop, Komorowski, Joštiak, Motyèáková
 * @version V1.0.0
 * @date    3.12.2016
 * @brief   Main program body
 ******************************************************************************
 **/

#include <stddef.h>
#include "stm32l1xx.h"

#include "RFID.h"
#include "spi.h"
#include "ili9163.h"

char poleChar[10];

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

char bufferRFID[30];

char bufferDisplay[10];

unsigned char MyID[5] = { 0x1a, 0x18, 0x3a, 0x45, 0x7d };	//My card on my keys

void unless_loop(void);
void RCC_Configuration(void);

void NVIC_Configuration(void);
void initUART2(void);
void Delay(uint32_t);
int Send_int_uart(int);
void Send_string_uart(const char*);
void Send_char_uart(char);

void USART1_IRQHandler(void);


int main(void) {
	setLed();
	unsigned char CardID[5];
	RCC_Configuration();
	/**/
	NVIC_Configuration();
	initUART2();
	//----RFID-----
	TM_MFRC522_Init();
	// pre LCD displej
	initSPI();
	initCD_Pin();
	initCS_Pin();
	initRES_Pin();
	lcdInitialise(LCD_ORIENTATION0);
	lcdClearDisplay(decodeRgbValue(255, 255, 255));

	uint8_t modreTlac = 0;
	uint8_t cerveneTlac = 0;

	kontrola_zoznamu();

	int pom = 0;
	welcome();
	GPIO_SetBits(GPIOA, GPIO_Pin_0);


	while (1) {

		modreTlac = ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) + 1) % 2;
		if (modreTlac == 0) {
			GPIO_SetBits(GPIOA, GPIO_Pin_4);
			pom=0;
			while (((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) + 1) % 2 == 0 && (pom<32))  {
				pom++;
				Delay_us(100000);
			}
			if (pom > 30) {
				vypis_pristupov();
			} else {

				zapisanie();

			}
			GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		}

		cerveneTlac = ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)) + 1) % 2;
		if (cerveneTlac == 0) {
			GPIO_SetBits(GPIOA, GPIO_Pin_4);
			vymazanie();
			//vypis_pristupov();
			GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		}

		//-----------------------------RFID Analayzer------------------------------------------
		if (TM_MFRC522_Check(CardID) == MI_OK) {
			zapis_pristupu(CardID);
			sprintf(bufferRFID, "[%x-%x-%x-%x-%x]", CardID[0], CardID[1],
					CardID[2], CardID[3], CardID[4]);
			Send_string_uart(bufferRFID);
			Send_string_uart("\n\r");
			//Check if this is my card
			if (karta_v_zozname(CardID) == MI_OK)
			//if (TM_MFRC522_Compare(CardID, MyID) == MI_OK)
					{
				Send_string_uart("Pristup povoleny\n\r");
				sprintf(bufferDisplay, "Pristup povoleny");
				lcdClearDisplay(decodeRgbValue(255, 255, 255));
				lcdPutS(bufferDisplay, 10, 50, decodeRgbValue(0, 0, 255),
						decodeRgbValue(255, 255, 255));
				//GPIO_SetBits(GPIOB, GPIO_Pin_4);
				//vykreslenie OK
				ok();
				GPIO_ResetBits(GPIOA, GPIO_Pin_0);
				//otvor zamok
				GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_6);
				Delay(5); //cas pre otvoreny zamok [s]
				GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_6);
				Delay(1);
				GPIO_SetBits(GPIOA, GPIO_Pin_0);
				welcome();

			} else {
				Send_string_uart("Pristup zamietnuty\n\r");
				sprintf(bufferDisplay, "Pristup zamietnuty");
				lcdClearDisplay(decodeRgbValue(255, 255, 255));
				lcdPutS(bufferDisplay, 10, 50, decodeRgbValue(255, 0, 0),
						decodeRgbValue(255, 255, 255));

				//vykreslenie Vykricnika a spustenie buzzera
				vykricnik();
				welcome();
			}

		}
		Send_string_uart("Waiting for RFID Card...!\n\r");

		Delay(1);
	}

} /*--------------End Main------------------*/

/*
 *************************************************************************************************************************************
 *							  								LOCAL FUNCTIONS															*
 *************************************************************************************************************************************
 */

void RCC_Configuration(void) {
	/* Setup the microcontroller system. Initialize the Embedded Flash Interface,
	 initialize the PLL and update the SystemFrequency variable. */
	//SystemInit();
	/* Enable GPIOA B C E and AFIO clocks */
///RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
	/**/
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

/**
 * @brief  	Inserts a delay time with resolution is 10 milisecond..
 * @param  	nCount: specifies the delay time length.
 * @retval 	None
 */
void Delay(__IO uint32_t num) {
	__IO uint32_t index = 0;

	/* default system clock is 72MHz */
	for (index = (720000 * num); index != 0; index--) {
	}
}

/**
 * @brief  	Make Delay using Loop CPU
 * @param  	uint32_t Delay in milisecond
 * @retval 	None
 */
void Delay_us(uint32_t us) {
	uint32_t temp;
	temp = us;			// * 6; // cho 24 MHZ
	while (temp > 0) {
		temp--;
	}
}

void unless_loop() {
	while (1) {
		Delay(20);
	}
}
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */

/*----------------------------------------------------------------------------
 SendChar
 Write character to Serial Port.
 *----------------------------------------------------------------------------*/
int Send_int_uart(int ch) {

	while (!(USART2->SR & USART_FLAG_TXE))
		;
	USART2->DR = (ch & 0x1FF);

	return (ch);
}

/*----------------------------------------------------------------------------
 GetKey
 Read character to Serial Port.
 *----------------------------------------------------------------------------*/
int GetKeyt(void) {

	while (!(USART2->SR & USART_FLAG_RXNE))
		;

	return ((int) (USART2->DR & 0x1FF));
}
/*----------------------------------------------------------------------------
 Send string
 *----------------------------------------------------------------------------*/
void Send_string_uart(const char *str) {
	while (*str) {
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
			;
		USART_SendData(USART2, *str++);
	}
}
/*----------------------------------------------------------------------------
 Send char
 *----------------------------------------------------------------------------*/
void Send_char_uart(char str) {

	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART2, str);

}
///////////

/**
 * @brief  Configures the nested vectored interrupt controller.
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void initUART2(void) {

	/* Configure USART pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* Enable USART2 Receive and Transmit interrupts */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	/* USART configuration */
	USART_Init(USART2, &USART_InitStructure);
	/* Enable USART */
	USART_Cmd(USART2, ENABLE);

}

void USART2_IRQHandler() {
	if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)) {
		//todo nieco
	}
}

void setLed(void) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitTypeDef gpioInitStruc;

	gpioInitStruc.GPIO_Pin = GPIO_Pin_0;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_PuPd = GPIO_PuPd_UP;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &gpioInitStruc);

	gpioInitStruc.GPIO_Pin = GPIO_Pin_1;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_PuPd = GPIO_PuPd_UP;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &gpioInitStruc);

	gpioInitStruc.GPIO_Pin = GPIO_Pin_4;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_PuPd = GPIO_PuPd_UP;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &gpioInitStruc);

	gpioInitStruc.GPIO_Pin = GPIO_Pin_6;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_PuPd = GPIO_PuPd_UP;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &gpioInitStruc);

	gpioInitStruc.GPIO_Pin = GPIO_Pin_8;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_IN;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpioInitStruc);

	gpioInitStruc.GPIO_Pin = GPIO_Pin_9;
	gpioInitStruc.GPIO_Mode = GPIO_Mode_IN;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpioInitStruc);
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func,
		const char *failedexpr) {
	while (1) {
	}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr) {
	__assert_func(file, line, NULL, failedexpr);
}
