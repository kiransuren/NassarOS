/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "xnucleoihm02a1.h"
//#include "example.h"
#include "example_usart.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdbool.h>

/**
  * @defgroup   MotionControl
  * @{
  */

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @}
  */ /* End of BSP */

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup   ExampleTypes
  * @{
  */

#define GPIO_POLLING_EXAMPLE        //!< Uncomment to performe the standalone example
//#define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
#define GPIO_INTERRUPT_EXAMPLE
#if ((defined (GPIO_POLLING_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
#elif ((!defined (GPIO_POLLING_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option!"
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		static bool prev_val;
		if (prev_val == false)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			prev_val = true;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			prev_val = false;
		}
	}
  /*switch (GPIO_Pin)
  {
  case GPIO_PIN_13:
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    break;
  }*/
}

/**
  * @}
  */ /* End of ExampleTypes */

/**
  * @brief The FW main module
  */
int main(void)
{
  /* NUCLEO board initialization */
  NUCLEO_Board_Init();
  
  /* X-NUCLEO-IHM02A1 initialization */
  //BSP_Init();
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure PC13 as input (onboard blue button)
  GPIO_InitTypeDef PC13_InitStruct;
  PC13_InitStruct.Pin = GPIO_PIN_13;
  PC13_InitStruct.Mode = GPIO_MODE_IT_RISING;
  PC13_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &PC13_InitStruct);

  // configure PB5 as output (for LED)
  GPIO_InitTypeDef PB5_InitStruct;
  PB5_InitStruct.Pin = GPIO_PIN_5;
  PB5_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  PB5_InitStruct.Pull = GPIO_NOPULL;
  PB5_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &PB5_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);


#if defined (GPIO_POLLING_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  
  /* Infinite loop */
  while (1){
      uint8_t buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
      if(buttonState == GPIO_PIN_RESET) {
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      }

      /*
      // Read the button state
      uint8_t buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      // Check if the button is pressed (active low)
      //USART_Transmit(&huart2, (uint8_t* )"Button Read");

      if (buttonState == GPIO_PIN_RESET) {
          // Button is pressed
          // Do something
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      } else {
          // Button is not pressed
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      }
      */
  }

#elif defined (GPIO_INTERRUPT_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  //MicrosteppingMotor_Example_01();

  /* Infinite loop */
  while (1) {
      // Read the button state
      uint8_t buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      // Check if the button is pressed (active low)
      //USART_Transmit(&huart2, (uint8_t* )"Button Read");

      if (buttonState == GPIO_PIN_RESET) {
          // Button is pressed
          // Do something
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      } else {
          // Button is not pressed
          // Do something else
    	  USART_Transmit(&huart2, (uint8_t* )"Button Pressed");
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      }
  }

#elif defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)
  /* Fill the L6470_DaisyChainMnemonic structure */
  Fill_L6470_DaisyChainMnemonic();
  
  /* Infinite loop */
  while (1)
  {
    /* Check if any Application Command for L6470 has been entered by USART */
    USART_CheckAppCmd();
  }
#endif
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
