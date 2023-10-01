/*

The MIT License (MIT)

Copyright (c) 2022 Ryan Edwards

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_gs_can.h"
#include "can.h"
#include "led.h"

static LED_HandleTypeDef hled_pwr;
static LED_HandleTypeDef hled_can_pwr_en_fault;
static LED_HandleTypeDef hled_can1en;
static LED_HandleTypeDef hled_can1rx;
static LED_HandleTypeDef hled_can1tx;
static LED_HandleTypeDef hled_can2en;
static LED_HandleTypeDef hled_can2rx;
static LED_HandleTypeDef hled_can2tx;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

extern USBD_GS_CAN_HandleTypeDef hGS_CAN;

static uint8_t can_term_pwr_flags;
#define CAN_TERM_PWR_FLAGS_BOTH_INTFC (0x03)

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/** Configure pins
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CAN1_STBY_Pin|CAN2_STBY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_PWR_Pin|LED_CAN_PWR_EN_FAULT_Pin|LED_CAN1_RX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_CAN1_TX_Pin|LED_CAN1_EN_Pin|LED_CAN2_TX_Pin|LED_CAN2_RX_Pin
                          |LED_CAN2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAN1_STBY_Pin CAN2_STBY_Pin LED_PWR_Pin LED_CAN_PWR_EN_FAULT_Pin
                           LED_CAN1_RX_Pin */
  GPIO_InitStruct.Pin = CAN1_STBY_Pin|CAN2_STBY_Pin|LED_PWR_Pin|LED_CAN_PWR_EN_FAULT_Pin
                          |LED_CAN1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_PWR_FAULT_Pin */
  GPIO_InitStruct.Pin = CAN_PWR_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CAN_PWR_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_PWR_EN_Pin */
  GPIO_InitStruct.Pin = CAN_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_CAN1_TX_Pin LED_CAN1_EN_Pin LED_CAN2_TX_Pin LED_CAN2_RX_Pin
                           LED_CAN2_EN_Pin */
  GPIO_InitStruct.Pin = LED_CAN1_TX_Pin|LED_CAN1_EN_Pin|LED_CAN2_TX_Pin|LED_CAN2_RX_Pin
                          |LED_CAN2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void main_init_cb(void)
{
	/* Enable power led */
	HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, GPIO_PIN_SET);

	/* Startup RX / TX led dance */
	for (uint8_t i=0; i<20; i++)
	{
		HAL_GPIO_TogglePin(LED_CAN1_RX_GPIO_Port, LED_CAN1_RX_Pin);
		HAL_GPIO_TogglePin(LED_CAN2_RX_GPIO_Port, LED_CAN2_RX_Pin);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LED_CAN1_TX_GPIO_Port, LED_CAN1_TX_Pin);
		HAL_GPIO_TogglePin(LED_CAN2_TX_GPIO_Port, LED_CAN2_TX_Pin);
	}

	/* Init CAN channels */
	hGS_CAN.channels[0] = &hfdcan1;
	hGS_CAN.channels[1] = &hfdcan2;
	can_init(hGS_CAN.channels[0], FDCAN1);
	can_init(hGS_CAN.channels[1], FDCAN2);

	/* Init LEDs */
	led_init(&hled_pwr, LED_PWR_GPIO_Port, LED_PWR_Pin, LED_MODE_ACTIVE, LED_ACTIVE_HIGH);
	led_init(&hled_can_pwr_en_fault, LED_CAN_PWR_EN_FAULT_GPIO_Port, LED_CAN_PWR_EN_FAULT_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
	led_init(&hled_can1en, LED_CAN1_EN_GPIO_Port, LED_CAN1_EN_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
	led_init(&hled_can1rx, LED_CAN1_RX_GPIO_Port, LED_CAN1_RX_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
	led_init(&hled_can1tx, LED_CAN1_TX_GPIO_Port, LED_CAN1_TX_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
	led_init(&hled_can2en, LED_CAN2_EN_GPIO_Port, LED_CAN2_EN_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
	led_init(&hled_can2rx, LED_CAN2_RX_GPIO_Port, LED_CAN2_RX_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
	led_init(&hled_can2tx, LED_CAN2_TX_GPIO_Port, LED_CAN2_TX_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
}

void main_task_cb(void)
{
	static bool bLastCANPwrFault;

	/* Check can power fault status */
	bool bCANPwrFault = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(CAN_PWR_FAULT_GPIO_Port, CAN_PWR_FAULT_Pin));
	if (bCANPwrFault != bLastCANPwrFault)
	{
		/* Fault status changed, set / unset flashing */
		if (bCANPwrFault) {
			led_blink_start(&hled_can_pwr_en_fault, 250);
		} else {
			led_blink_stop(&hled_can_pwr_en_fault);
		}

		/* Wait for next change */
		bLastCANPwrFault = bCANPwrFault;
	}

	/* update all the LEDs */
	led_update(&hled_pwr);
	led_update(&hled_can_pwr_en_fault);
	led_update(&hled_can1en);
	led_update(&hled_can1rx);
	led_update(&hled_can1tx);
	led_update(&hled_can2en);
	led_update(&hled_can2rx);
	led_update(&hled_can2tx);
}

void can_on_enable_cb(uint8_t channel)
{
	if (channel == 0) {
		HAL_GPIO_WritePin(CAN1_STBY_GPIO_Port, CAN1_STBY_Pin, GPIO_PIN_RESET);
		led_set_active(&hled_can1en);
	}
	if (channel == 1) {
		HAL_GPIO_WritePin(CAN2_STBY_GPIO_Port, CAN2_STBY_Pin, GPIO_PIN_RESET);
		led_set_active(&hled_can2en);
	}
}

void can_on_disable_cb(uint8_t channel)
{
	if (channel == 0) {
		HAL_GPIO_WritePin(CAN1_STBY_GPIO_Port, CAN1_STBY_Pin, GPIO_PIN_SET);
		led_set_inactive(&hled_can1en);
	}
	if (channel == 1) {
		HAL_GPIO_WritePin(CAN2_STBY_GPIO_Port, CAN2_STBY_Pin, GPIO_PIN_SET);
		led_set_inactive(&hled_can2en);
	}
}

void can_on_tx_cb(uint8_t channel, struct gs_host_frame *frame)
{
	UNUSED(frame);
	if (channel == 0)
		led_indicate_rxtx(&hled_can1tx);
	else
		led_indicate_rxtx(&hled_can2tx);
}

void can_on_rx_cb(uint8_t channel, struct gs_host_frame *frame)
{
	UNUSED(frame);
	if (channel == 0)
		led_indicate_rxtx(&hled_can1rx);
	else
		led_indicate_rxtx(&hled_can2rx);
}

void can_identify_cb(uint32_t do_identify)
{
	if (do_identify) {
		led_blink_start(&hled_pwr, 250);
	}
	else {
		led_blink_stop(&hled_pwr);
	}
}

void can_set_term_cb(uint8_t channel, GPIO_PinState state)
{
	if (GPIO_PIN_SET == state) {
		can_term_pwr_flags |= (1U << channel);
	} else {
		can_term_pwr_flags &= ~(1U << channel);
	}

	/* enable can power if termination is specified for both busses (bit of a hack...ssh)*/
	if (CAN_TERM_PWR_FLAGS_BOTH_INTFC == can_term_pwr_flags) {
		HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_RESET);
		led_set_active(&hled_can_pwr_en_fault);
	} else {
		HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_SET);
		led_set_inactive(&hled_can_pwr_en_fault);
	}
}

GPIO_PinState can_get_term_cb(uint8_t channel)
{
	/* Return status for channel */
	return (can_term_pwr_flags & (1U << channel)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
