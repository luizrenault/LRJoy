/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v2.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */

		//T-RUDDER HID REPORT DESCRIPTOR OBTIDO DO USBLYZER
		0x05, 0x01,
		0x09, 0x04,
		0xA1, 0x01,
		0x15, 0x00,
		0x35, 0x00,
		//0x26, 0xFF, 0x03,
		0x26, 0xff, 0x0f,   // Trocando o logical max para se adequar ao ADC desse uC
		//0x46, 0xFF, 0x03,
		0x46, 0xff, 0x0f, // Trocando o physical max para manter coerencia com logical max
		0x09, 0x30,
		0x09, 0x31,
		0x09, 0x32,
		0x75, 0x10,
		0x95, 0x03,
		0x81, 0x02,
		0x06, 0x00, 0xFF,
		0x09, 0x20,
		0x75, 0x08,
		0x95, 0x25,
		0x81, 0x02,
		0x09, 0x21,
		0x95, 0x20,
		0x91, 0x02,
		0xC0

//	    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
//	    0x09, 0x04,                    // USAGE (Joystick)
//	    0xa1, 0x01,                    // COLLECTION (Application)
//	    0xa1, 0x00,                    //   COLLECTION (Physical)
//	    0x09, 0x36,                    //     USAGE (Slider)
//	    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
//	    0x26, 0xff, 0x0f,              //     LOGICAL_MAXIMUM (4095)
//	    0x35, 0x00,                    //     PHYSICAL_MINIMUM (0)
//	    0x46, 0xff, 0x0f,              //     PHYSICAL_MAXIMUM (4095)
//	    0x75, 0x0c,                    //     REPORT_SIZE (12)
//	    0x95, 0x01,                    //     REPORT_COUNT (1)
//	    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
//	    0x75, 0x04,                    //     REPORT_SIZE (4)
//	    0x81, 0x01,                    //     INPUT (Cnst,Ary,Abs)
//	    0x09, 0x36,                    //     USAGE (Slider)
//	    0x75, 0x0c,                    //     REPORT_SIZE (12)
//	    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
//	    0x75, 0x04,                    //     REPORT_SIZE (4)
//	    0x81, 0x01,                    //     INPUT (Cnst,Ary,Abs)
//	    0x09, 0x36,                    //     USAGE (Slider)
//	    0x75, 0x0c,                    //     REPORT_SIZE (12)
//	    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
//	    0x75, 0x04,                    //     REPORT_SIZE (4)
//	    0x81, 0x01,                    //     INPUT (Cnst,Ary,Abs)
//	    0xc0,                          //   END_COLLECTION
//	    0x05, 0x09,                    //   USAGE_PAGE (Button)
//	    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//	    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
//	    0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
//	    0x45, 0x01,                    //   PHYSICAL_MAXIMUM (1)
//	    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
//	    0x29, 0x28,                    //   USAGE_MAXIMUM (Button 40)
//	    0x75, 0x01,                    //   REPORT_SIZE (1)
//	    0x95, 0x28,                    //   REPORT_COUNT (40)
//	    0x81, 0x02,                    //   INPUT (Data,Var,Abs)

//		0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
//	    0x09, 0x04,                    // USAGE (Joystick)
//	    0xa1, 0x01,                    // COLLECTION (Application)
//	    0x05, 0x09,                    //   USAGE_PAGE (Button)
//	    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//	    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
//	    0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
//	    0x45, 0x01,                    //   PHYSICAL_MAXIMUM (1)
//	    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
//	    0x29, 0x20,                    //   USAGE_MAXIMUM (Button 32)
//	    0x75, 0x01,                    //   REPORT_SIZE (1)
//	    0x95, 0x20,                    //   REPORT_COUNT (32)
//	    0x81, 0x02,                    //   INPUT (Data,Var,Abs)

  /* USER CODE END 0 */
//  0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

