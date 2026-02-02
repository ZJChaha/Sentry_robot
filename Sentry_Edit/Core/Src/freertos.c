/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "INS_task.h"
#include "Chassis_task.h"
#include "Gimbal_task.h"
#include "Detect_left_task.h"
#include "Detect_right_task.h"
#include "Buzzer_task.h"
#include "Shoot_task.h"
#include "referee_usart_task.h"
#include "USB_task.h"
#include "Information_task.h"
#include "Super_cap_Task.h"
#include "spi.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

osThreadId calibrate_task_handle;
osThreadId chassisTaskHandle;
osThreadId detectleftHandle;
osThreadId detectrightHandle;
osThreadId gimbalTaskHandle;
osThreadId imuTaskHandle;
osThreadId referee_usart_task_handle;
osThreadId battery_voltage_handle;
osThreadId shootTaskHandle;
osThreadId buzzerTaskHandle;
osThreadId usbTaskHandle;
osThreadId informationTaskHandle;
osThreadId supercapTaskHandle;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId INSTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartINSTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = &xTimerStack[0];
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of INSTask */
  osThreadDef(INSTask, StartINSTask, osPriorityNormal, 0, 1024);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

//	    osThreadDef(cali, Calibrate_task, osPriorityNormal, 0, 512);
//    calibrate_task_handle = osThreadCreate(osThread(cali), NULL);

    osThreadDef(ChassisTask, Chassis_task, osPriorityAboveNormal, 0, 512);
    chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

    osThreadDef(detectleftTask, Detect_left_task, osPriorityAboveNormal, 0, 64);
    detectleftHandle = osThreadCreate(osThread(detectleftTask), NULL);

    osThreadDef(detectrightTask, Detect_right_task, osPriorityAboveNormal, 0, 64);
    detectrightHandle = osThreadCreate(osThread(detectrightTask), NULL);


    osThreadDef(gimbalTask, Gimbal_task, osPriorityHigh, 0, 512);
    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);


    osThreadDef(REFEREE, referee_usart_task, osPriorityLow, 0, 128);
    referee_usart_task_handle = osThreadCreate(osThread(REFEREE), NULL);

    osThreadDef(ShootTask, Shoot_task, osPriorityNormal, 0, 64);
    shootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

    osThreadDef(BuzzerTask, Buzzer_task, osPriorityLow, 0, 64);
    buzzerTaskHandle = osThreadCreate(osThread(BuzzerTask), NULL);

    osThreadDef(USBTask, USB_task, osPriorityLow, 0, 128);
    usbTaskHandle = osThreadCreate(osThread(USBTask), NULL);

    osThreadDef(InformationTask, Information_task, osPriorityHigh, 0, 128);
    informationTaskHandle = osThreadCreate(osThread(InformationTask), NULL);


    osThreadDef(supercapTask, Super_cap_Task, osPriorityNormal, 0, 128);
    supercapTaskHandle = osThreadCreate(osThread(supercapTask), NULL);




  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartINSTask */
/**
  * @brief  Function implementing the INSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartINSTask */
void StartINSTask(void const * argument)
{
  /* init code for USB_DEVICE */
  //MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartINSTask */
    while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR)
        ;
    INS_Init();

    Gimbal.Mother.IMU_init_finish=1;

    /* Infinite loop */
    for (;;)
    {
        INS_Task();
        osDelay(1);
    }
  /* USER CODE END StartINSTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
