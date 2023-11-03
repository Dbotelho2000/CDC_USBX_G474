/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_usbx_device.c
 * @author  MCD Application Team
 * @brief   USBX Device applicative file
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
#include "app_usbx_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb.h"
#include "ux_dcd_stm32.h"
#include "ux_device_cdc_acm.h"
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

static ULONG cdc_acm_interface_number;
static ULONG cdc_acm_configuration_number;
static UX_SLAVE_CLASS_CDC_ACM_PARAMETER cdc_acm_parameter;
static TX_THREAD ux_device_app_thread;

/* USER CODE BEGIN PV */
static TX_THREAD ux_cdc_read_thread;
static TX_THREAD ux_cdc_write_thread;

TX_QUEUE ux_command_queue;
uint8_t queue_stack[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static VOID app_ux_device_thread_entry(ULONG thread_input);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
 * @brief  Application USBX Device Initialization.
 * @param  memory_ptr: memory pointer
 * @retval status
 */
UINT MX_USBX_Device_Init(VOID *memory_ptr)
{
	UINT ret = UX_SUCCESS;
	UCHAR *device_framework_high_speed;
	UCHAR *device_framework_full_speed;
	ULONG device_framework_hs_length;
	ULONG device_framework_fs_length;
	ULONG string_framework_length;
	ULONG language_id_framework_length;
	UCHAR *string_framework;
	UCHAR *language_id_framework;
	UCHAR *pointer;
	TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

	/* USER CODE BEGIN MX_USBX_Device_Init0 */

	/* USER CODE END MX_USBX_Device_Init0 */

	/* Allocate the stack for USBX Memory */
	if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
			USBX_DEVICE_MEMORY_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
	{
		/* USER CODE BEGIN USBX_ALLOCATE_STACK_ERORR */
		while(1);
		return TX_POOL_ERROR;
		/* USER CODE END USBX_ALLOCATE_STACK_ERORR */
	}

	/* Initialize USBX Memory */
	if (ux_system_initialize(pointer, USBX_DEVICE_MEMORY_STACK_SIZE, UX_NULL, 0) != UX_SUCCESS)
	{
		/* USER CODE BEGIN USBX_SYSTEM_INITIALIZE_ERORR */
		while(1);
		return UX_ERROR;
		/* USER CODE END USBX_SYSTEM_INITIALIZE_ERORR */
	}

	/* Get Device Framework High Speed and get the length */
	device_framework_high_speed = USBD_Get_Device_Framework_Speed(USBD_HIGH_SPEED,
			&device_framework_hs_length);

	/* Get Device Framework Full Speed and get the length */
	device_framework_full_speed = USBD_Get_Device_Framework_Speed(USBD_FULL_SPEED,
			&device_framework_fs_length);

	/* Get String Framework and get the length */
	string_framework = USBD_Get_String_Framework(&string_framework_length);

	/* Get Language Id Framework and get the length */
	language_id_framework = USBD_Get_Language_Id_Framework(&language_id_framework_length);

	/* Install the device portion of USBX */
	if (ux_device_stack_initialize(device_framework_high_speed,
			device_framework_hs_length,
			device_framework_full_speed,
			device_framework_fs_length,
			string_framework,
			string_framework_length,
			language_id_framework,
			language_id_framework_length,
			UX_NULL) != UX_SUCCESS)
	{
		/* USER CODE BEGIN USBX_DEVICE_INITIALIZE_ERORR */
		while(1);
		return UX_ERROR;
		/* USER CODE END USBX_DEVICE_INITIALIZE_ERORR */
	}

	/* Initialize the cdc acm class parameters for the device */
	cdc_acm_parameter.ux_slave_class_cdc_acm_instance_activate   = USBD_CDC_ACM_Activate;
	cdc_acm_parameter.ux_slave_class_cdc_acm_instance_deactivate = USBD_CDC_ACM_Deactivate;
	cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change    = USBD_CDC_ACM_ParameterChange;

	/* USER CODE BEGIN CDC_ACM_PARAMETER */

	/* USER CODE END CDC_ACM_PARAMETER */

	/* Get cdc acm configuration number */
	cdc_acm_configuration_number = USBD_Get_Configuration_Number(CLASS_TYPE_CDC_ACM, 0);

	/* Find cdc acm interface number */
	cdc_acm_interface_number = USBD_Get_Interface_Number(CLASS_TYPE_CDC_ACM, 0);

	/* Initialize the device cdc acm class */
	if (ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
			ux_device_class_cdc_acm_entry,
			cdc_acm_configuration_number,
			cdc_acm_interface_number,
			&cdc_acm_parameter) != UX_SUCCESS)
	{
		/* USER CODE BEGIN USBX_DEVICE_CDC_ACM_REGISTER_ERORR */
		while(1);
		return UX_ERROR;
		/* USER CODE END USBX_DEVICE_CDC_ACM_REGISTER_ERORR */
	}

	/* Allocate the stack for device application main thread */
	if (tx_byte_allocate(byte_pool, (VOID **) &pointer, UX_DEVICE_APP_THREAD_STACK_SIZE,
			TX_NO_WAIT) != TX_SUCCESS)
	{
		/* USER CODE BEGIN MAIN_THREAD_ALLOCATE_STACK_ERORR */
		while(1);
		return TX_POOL_ERROR;
		/* USER CODE END MAIN_THREAD_ALLOCATE_STACK_ERORR */
	}

	/* Create the device application main thread */
	if (tx_thread_create(&ux_device_app_thread, UX_DEVICE_APP_THREAD_NAME, app_ux_device_thread_entry,
			0, pointer, UX_DEVICE_APP_THREAD_STACK_SIZE, UX_DEVICE_APP_THREAD_PRIO,
			UX_DEVICE_APP_THREAD_PREEMPTION_THRESHOLD, UX_DEVICE_APP_THREAD_TIME_SLICE,
			UX_DEVICE_APP_THREAD_START_OPTION) != TX_SUCCESS)
	{
		/* USER CODE BEGIN MAIN_THREAD_CREATE_ERORR */
		while(1);
		return TX_THREAD_ERROR;
		/* USER CODE END MAIN_THREAD_CREATE_ERORR */
	}

	/* USER CODE BEGIN MX_USBX_Device_Init1 */
	/* Allocate memory for the UX RX thread */
	if(tx_byte_allocate(byte_pool, (VOID **)&pointer, 1024, TX_NO_WAIT) != TX_SUCCESS)
	{
		return TX_POOL_ERROR;
	}

	/* Create the UX RX thread */
	if(tx_thread_create(&ux_cdc_read_thread, "cdc_acm_read_usbx_app_thread_entry", usbx_cdc_acm_read_thread_entry, 1, pointer, 1024, 20, 20, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
	{
		return TX_THREAD_ERROR;
	}

	/* Allocate memory for the UX TX thread */
	if(tx_byte_allocate(byte_pool, (VOID **)&pointer, 1024, TX_NO_WAIT) != TX_SUCCESS)
	{
		return TX_POOL_ERROR;
	}

	/* Create the UX TX thread */
	if(tx_thread_create(&ux_cdc_write_thread, "cdc_acm_write_usbx_app_thread_entry", usbx_cdc_acm_write_thread_entry, 1, pointer, 1025, 20, 20, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
	{
		return TX_THREAD_ERROR;
	}

	/* Create the Command Queue */
	if(tx_queue_create(&ux_command_queue, "Command Queue", 1, queue_stack, sizeof(queue_stack)) != TX_SUCCESS)
	{
		return TX_QUEUE_ERROR;
	}
	/* USER CODE END MX_USBX_Device_Init1 */

	return ret;
}

/**
 * @brief  Function implementing app_ux_device_thread_entry.
 * @param  thread_input: User thread input parameter.
 * @retval none
 */
static VOID app_ux_device_thread_entry(ULONG thread_input)
{
	/* USER CODE BEGIN app_ux_device_thread_entry */
	MX_USB_PCD_Init();

	HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00 , PCD_SNG_BUF, 0x40);
	HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80 , PCD_SNG_BUF, 0x80);

	HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x81, PCD_SNG_BUF, 0xC0);
	HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x02, PCD_SNG_BUF, 0x100);
	HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x82, PCD_SNG_BUF, 0x140);

	ux_dcd_stm32_initialize((ULONG)USB, (ULONG)&hpcd_USB_FS);

	HAL_PCD_Start(&hpcd_USB_FS);
	/* USER CODE END app_ux_device_thread_entry */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
