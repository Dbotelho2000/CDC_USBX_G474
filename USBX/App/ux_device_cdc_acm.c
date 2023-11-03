/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    ux_device_cdc_acm.c
 * @author  MCD Application Team
 * @brief   USBX Device CDC ACM applicative source file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2020-2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;
uint8_t UserRxBuffer[64];
extern TX_QUEUE ux_command_queue;

enum
{
	LED_ON,
	LED_OFF,
	BUTTON_PRESSED
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  USBD_CDC_ACM_Activate
 *         This function is called when insertion of a CDC ACM device.
 * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
 * @retval none
 */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
	/* USER CODE BEGIN USBD_CDC_ACM_Activate */
	/* Save the instance */
	cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;

	/* Turn the Led ON when USB is connected */
	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	/* USER CODE END USBD_CDC_ACM_Activate */

	return;
}

/**
 * @brief  USBD_CDC_ACM_Deactivate
 *         This function is called when extraction of a CDC ACM device.
 * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
 * @retval none
 */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
	/* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
	UX_PARAMETER_NOT_USED(cdc_acm_instance);
	/* USER CODE END USBD_CDC_ACM_Deactivate */

	return;
}

/**
 * @brief  USBD_CDC_ACM_ParameterChange
 *         This function is invoked to manage the CDC ACM class requests.
 * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
 * @retval none
 */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
	/* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
	UX_PARAMETER_NOT_USED(cdc_acm_instance);
	/* USER CODE END USBD_CDC_ACM_ParameterChange */

	return;
}

/* USER CODE BEGIN 1 */
VOID usbx_cdc_acm_read_thread_entry(ULONG thread_input)
{
	UX_PARAMETER_NOT_USED(thread_input);

	/* Private Variables */
	ULONG rx_actual_length;
	uint8_t message;

	/* Infinite Loop */
	while(1)
	{
		if(cdc_acm != UX_NULL)
		{
			ux_device_class_cdc_acm_read(cdc_acm, (UCHAR *)UserRxBuffer, 64, &rx_actual_length);

			switch(UserRxBuffer[rx_actual_length-1])
			{
				case '1':
					message = LED_ON;
					tx_queue_send(&ux_command_queue, &message, TX_NO_WAIT);
					HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
					break;

				case '0':
					message = LED_OFF;
					tx_queue_send(&ux_command_queue, &message, TX_NO_WAIT);
					HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
					break;
			}
		}
	}
}

VOID usbx_cdc_acm_write_thread_entry(ULONG thread_input)
{
	UX_PARAMETER_NOT_USED(thread_input);

	/* Private Variables */
	ULONG tx_actual_length;
	uint8_t queue_message = 0xFF;
	const uint8_t UserLedOn[] = " LED ON\r\n";
	const uint8_t UserLedOff[] = " LED OFF\r\n";
	const uint8_t UserButtonEvent[] = "BUTTON PRESSED\r\n";

	while(1)
	{
		tx_queue_receive(&ux_command_queue, &queue_message, TX_WAIT_FOREVER);

		switch(queue_message)
		{
			case LED_ON:
				ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)(UserLedOn), sizeof(UserLedOn), &tx_actual_length);
				break;
			case LED_OFF:
				ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)(UserLedOff), sizeof(UserLedOff), &tx_actual_length);
				break;
			case BUTTON_PRESSED:
				ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)(UserButtonEvent), sizeof(UserButtonEvent), &tx_actual_length);
				break;
			default:
				break;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Private Variables */
	uint8_t message = BUTTON_PRESSED;

	/* Check which button was pressed */
	if(GPIO_Pin == JOYSTICK_SEL_Pin)
	{
		tx_queue_send(&ux_command_queue, &message, TX_NO_WAIT);
	}
}
/* USER CODE END 1 */
