/**
  ******************************************************************************
  * @file    main.c 
  * @author  Prekop, Komorowski, Joštiak, Motyèáková
  * @version V1.0.0
  * @date    3.12.2016
  * @brief   Main program body
  ******************************************************************************

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup IO_Toggle
  * @{
  */ 

int main(void)
{

  
  /* Infinite loop */
  while (1)
  {

  }
}


void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  
  while(TimingDelay != 0);
}

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
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

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
