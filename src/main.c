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



int main(void) {

	while (1) {


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


void initUART2(void) {

	/* Configure USART pins */


}

void USART2_IRQHandler() {
	if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)) {
		//todo nieco
	}
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

