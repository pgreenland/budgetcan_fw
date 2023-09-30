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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

#define BOARD_SYSMEM_RESET_VECTOR 0x1FFF0000

#define USBD_VID				  0x1D50
#define USBD_LANGID_STRING		  1033
#define USBD_MANUFACTURER_STRING  (uint8_t*) "Quantulum Ltd"
#define USBD_PID_FS				  0x606F
#define USBD_PRODUCT_STRING_FS	  (uint8_t*) "USBtoCANFD"
#define USBD_CONFIGURATION_STRING (uint8_t*) "gs_usb"
#define USBD_INTERFACE_STRING	  (uint8_t*) "gs_usb interface"
#define DFU_INTERFACE_STRING	  (uint8_t*) "USBtoCANFD DFU interface"

/* FDCAN init values for this board */
#define FDCAN_SJW_INIT				 1
#define FDCAN_BRP_INIT				 8
#define FDCAN_TS1_INIT				 13
#define FDCAN_TS2_INIT				 2

#define FDCAN_DATA_SJW_INIT			 4
#define FDCAN_DATA_BRP_INIT			 2
#define FDCAN_DATA_TS1_INIT			 15
#define FDCAN_DATA_TS2_INIT			 4

#define FDCAN_CLOCK_DIV_INIT		 FDCAN_CLOCK_DIV1
#define FDCAN_FRAME_FMT_INIT		 FDCAN_FRAME_FD_BRS
#define FDCAN_MODE_INIT				 FDCAN_MODE_NORMAL
#define FDCAN_AUTO_RETX_INIT		 ENABLE
#define FDCAN_AUTO_TX_PAUSE_INIT	 DISABLE
#define FDCAN_PROT_EXCPTN_INIT		 ENABLE
#define FDCAN_STD_FLTR_NUM_INIT		 0
#define FDCAN_EXT_FLTR_NUM_INIT		 0
#define FDCAN_TX_FIFO_OPERATION_INIT FDCAN_TX_FIFO_OPERATION

#define CAN_NUM_CHANNELS			 2
#define CAN_CLOCK_SPEED				 80000000
#define CAN_TERM_FEATURE_ENABLED
#define CANFD_FEATURE_ENABLED

#define QUEUE_SIZE_HOST_TO_DEV	  32
#define QUEUE_SIZE_DEV_TO_HOST	  32

#define CAN1_TX_Pin GPIO_PIN_9
#define CAN1_TX_GPIO_Port GPIOB
#define OSC_IN_Pin GPIO_PIN_14
#define OSC_IN_GPIO_Port GPIOC
#define CAN1_STBY_Pin GPIO_PIN_0
#define CAN1_STBY_GPIO_Port GPIOA
#define CAN2_STBY_Pin GPIO_PIN_1
#define CAN2_STBY_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_2
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_3
#define UART_RX_GPIO_Port GPIOA
#define CAN2_RX_Pin GPIO_PIN_0
#define CAN2_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_1
#define CAN2_TX_GPIO_Port GPIOB
#define CAN_PWR_FAULT_Pin GPIO_PIN_8
#define CAN_PWR_FAULT_GPIO_Port GPIOA
#define LED_PWR_Pin GPIO_PIN_9
#define LED_PWR_GPIO_Port GPIOA
#define CAN_PWR_EN_Pin GPIO_PIN_6
#define CAN_PWR_EN_GPIO_Port GPIOC
#define LED_CAN_PWR_EN_FAULT_Pin GPIO_PIN_10
#define LED_CAN_PWR_EN_FAULT_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_BOOT0_Pin GPIO_PIN_14
#define SWCLK_BOOT0_GPIO_Port GPIOA
#define LED_CAN1_RX_Pin GPIO_PIN_15
#define LED_CAN1_RX_GPIO_Port GPIOA
#define LED_CAN1_TX_Pin GPIO_PIN_3
#define LED_CAN1_TX_GPIO_Port GPIOB
#define LED_CAN1_EN_Pin GPIO_PIN_4
#define LED_CAN1_EN_GPIO_Port GPIOB
#define LED_CAN2_TX_Pin GPIO_PIN_5
#define LED_CAN2_TX_GPIO_Port GPIOB
#define LED_CAN2_RX_Pin GPIO_PIN_6
#define LED_CAN2_RX_GPIO_Port GPIOB
#define LED_CAN2_EN_Pin GPIO_PIN_7
#define LED_CAN2_EN_GPIO_Port GPIOB
#define CAN1_RX_Pin GPIO_PIN_8
#define CAN1_RX_GPIO_Port GPIOB

void SystemClock_Config(void);
void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ BOARD_H__ */

