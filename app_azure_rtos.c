
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_azure_rtos.c
  * @author  MCD Application Team
  * @brief   azure_rtos application implementation file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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

#include "app_azure_rtos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_SIZE 4096
#define MAX_STUDENTS 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN TX_Pool_Buffer */
/* USER CODE END TX_Pool_Buffer */
static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE];
static TX_BYTE_POOL tx_app_byte_pool;

/* USER CODE BEGIN PV */
TX_SEMAPHORE sem1;
TX_SEMAPHORE sem2;
TX_THREAD task1;
TX_THREAD task2;
UCHAR task1_stack[STACK_SIZE];
UCHAR task2_stack[STACK_SIZE];
char rfid[3];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi2;
typedef struct
{
    char rf[20];
    int in;
} record;
record rec[MAX_STUDENTS];
int count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void MotionDetected(ULONG thread_input);
void ScanAndStore(ULONG thread_input);
int FindRFID(char *s);
uint8_t BCD_To_Dec(uint8_t);
uint8_t recieveSPI(uint8_t reg);
void getTimestamp();
/* USER CODE END PFP */

/**
  * @brief  Define the initial system.
  * @param  first_unused_memory : Pointer to the first unused memory
  * @retval None
  */
VOID tx_application_define(VOID *first_unused_memory)
{
  /* USER CODE BEGIN  tx_application_define */
	 tx_semaphore_create(&sem1, "s1", 1);
	 tx_semaphore_create(&sem2, "s2", 0);
	 tx_thread_create(&task1, "Task 1", MotionDetected, 0,
	                     task1_stack, STACK_SIZE,
	                     5, 5, TX_NO_TIME_SLICE, TX_AUTO_START);
 	 tx_thread_create(&task2, "Task 2", ScanAndStore, 0,
	                     task2_stack, STACK_SIZE,
	                     5, 5, TX_NO_TIME_SLICE, TX_AUTO_START);
  /* USER CODE END  tx_application_define */

  VOID *memory_ptr;

  if (tx_byte_pool_create(&tx_app_byte_pool, "Tx App memory pool", tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN TX_Byte_Pool_Error */

    /* USER CODE END TX_Byte_Pool_Error */
  }
  else
  {
    /* USER CODE BEGIN TX_Byte_Pool_Success */

    /* USER CODE END TX_Byte_Pool_Success */

    memory_ptr = (VOID *)&tx_app_byte_pool;

    if (App_ThreadX_Init(memory_ptr) != TX_SUCCESS)
    {
      /* USER CODE BEGIN  App_ThreadX_Init_Error */

      /* USER CODE END  App_ThreadX_Init_Error */
    }

    /* USER CODE BEGIN  App_ThreadX_Init_Success */

    /* USER CODE END  App_ThreadX_Init_Success */

  }

}

/* USER CODE BEGIN  0 */
void MotionDetected(ULONG thread_input)
{
    while (1)
    {
//    	tx_thread_sleep(1000);//pir sensor is high about 1 sec after detection that is why this delay
    	if(tx_semaphore_get(&sem1, TX_WAIT_FOREVER) == TX_SUCCESS)
    	{
    		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
    		{
    			HAL_UART_Transmit(&huart1, (uint8_t*)"motion detected\r\n", 17, HAL_MAX_DELAY);
    			tx_semaphore_put(&sem2);
    			tx_thread_sleep(500);
    		}
    		else
    		{
    			tx_semaphore_put(&sem1);
    		}
    	}
    }
}

void ScanAndStore(ULONG thread_input)
{
    while (1)
    {
        if (tx_semaphore_get(&sem2, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
        	HAL_UART_Transmit(&huart1, (uint8_t*)"scan\r\n",6, HAL_MAX_DELAY);
        	HAL_UART_Receive(&huart1, (uint8_t*)rfid, 2, HAL_MAX_DELAY);
        	rfid[2] = '\0';
        	char buffer1[5];
        	snprintf(buffer1, 5,"%s\n\r", rfid);
        	HAL_UART_Transmit(&huart1, (uint8_t*)buffer1, 4, HAL_MAX_DELAY);
//        	char s[2];
//        	getTimestamp(s);
        	int ch = FindRFID(rfid);
        	char buffer2[12];
        	if (ch==-1)
        	{
        		if (count < MAX_STUDENTS)
        	    {
        	        strcpy(rec[count].rf, rfid);
        	        rec[count].in = 1;
        	        count++;
        	        snprintf(buffer2, 12,"%s, ENTRY\r\n", rfid);
        	        HAL_UART_Transmit(&huart2, (uint8_t*)buffer2, 11, HAL_MAX_DELAY);
        	    }
        	}
        	else
        	{
        	   if (rec[ch].in == 1)
        	   {
        	       rec[ch].in = 0;
        	       snprintf(buffer2, 12,"%s, EXIT!\r\n", rfid);
        	       HAL_UART_Transmit(&huart2, (uint8_t*)buffer2, 11, HAL_MAX_DELAY);
        	   }
        	   else
        	   {
        		   rec[ch].in = 1;
        		   snprintf(buffer2, 12,"%s, ENTRY\r\n", rfid);
        		   HAL_UART_Transmit(&huart2, (uint8_t*)buffer2, 11, HAL_MAX_DELAY);
        	   }
        	}
        	char b[6];
        	HAL_UART_Receive(&huart2, (uint8_t*)b, 6, HAL_MAX_DELAY);
        	HAL_UART_Transmit(&huart1, (uint8_t*)b, 6, HAL_MAX_DELAY);
        	tx_semaphore_put(&sem1);
            tx_thread_sleep(500);
        }
    }
}
int FindRFID(char *s)
{
    for (int i = 0; i < count; i++)
    {
        if (strcmp(rec[i].rf,s)==0)
        {
            return i;
        }
    }
    return -1;
}

//uint8_t BCD_To_Dec(uint8_t val)
//{
//	return ((val >> 4) * 10) + (val & 0x0F);
//}

//uint8_t recieveSPI(uint8_t reg)
//{
//    uint8_t rxData = 0;
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//    HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
//    HAL_SPI_Receive(&hspi2, &rxData, 1, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//    return rxData;
//}
//void getTimestamp(char *s)
//{
////    uint8_t sec = BCD_To_Dec(recieveSPI(0x00));
////    uint8_t min = BCD_To_Dec(recieveSPI(0x01));
//      uint8_t hr  = recieveSPI(0x01);
//      sprintf(s, "%02d", hr);
//      HAL_UART_Transmit(&huart1, (uint8_t*)s, 2, HAL_MAX_DELAY);
//}
/* USER CODE END  0 */
