/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality-controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wwrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
#include <stdlib.h>
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/*-----------------------------------------------------------*/
//traffic light states

#define yellow 1
#define green 2
#define red 0

#define car_speed 500 //2 leds per second

//sample rate of pot - lab manual
// #define sample_rate_pot 500
#define default_light_time 2000

//////////////////////////////////////////////////////////

#define adc_pin GPIO_Pin_3
#define red_light GPIO_Pin_0 
#define yellow_light GPIO_Pin_1 
#define green_light GPIO_Pin_2 
#define shift_reg_data GPIO_Pin_6
#define shift_reg_clock GPIO_Pin_7
#define shift_reg_reset  GPIO_Pin_8
//////////////////////////////////////////////////////////

//**********************************
//default yellow time (const)
#define mainQUEUE_LENGTH 100
#define yellow_time 1000
#define minimum_time 4000 
#define maximum_time 8000

//////////////////////////////////////////////////////////
//ques
static xQueueHandle light_state;
static xQueueHandle trafficFlowLightQueue; 
static xQueueHandle trafficFlowCarsQueue;
static xQueueHandle trafficLightQueue;
//////////////////////////////////////////////////////////

//Handles for sl timer
//street light timers - when timer goes off rtos will run this handle and this will wake the traffic light control to switch states
//Timer handle lets us stop start, change the wait of the timer
// static TimerHandle_t traffic_light_timer;

//task handles lets us wake up a specific task, stop and start the task if higher priority comes in
// static TaskHandle_t tl_task_handle;

//////////////////////////////////////////////////////////
//hardware functions
	/*
	 * TODO: Implement this function for any hardware specific clock configuration
	 * that was not already performed before main() was called.
	 */
static void prvSetupHardware( void );//sets up clocks and gpio pins
	//adc poll helper fucntion
static uint16_t poll_adc_function(void);//polls the adc reading from pot to get new rate of gen

//changes the output of the tl leds
static void tl_change_state(uint8_t light);

//////////////////////////////////////////////////////////
//tasks
static void update_traffic_rate_task(void *pvParameters); //polls pot and updates the rate of gen
static void traffic_light_control_task(void *pvParameters);//changes light states
static void generate_car_task(void *pvParameters);//produces car generation

static void Traffic_Light(void *pvParameters);
static void ADC_callBack(TimerHandle_t xTimer);
static void Adjust_Traffic(void *pvParameters);

/////////////////////////////////////////////////////////////////////

static TaskHandle_t adjust_traffic_handles
static TaskHandle_t traffic_light_handle;

static uint32_t light_time(uint16_t adc_value, uint8_t light);

/*-----------------------------------------------------------*/
main(void)
{
	prvSetupHardware();

	//light_state holds the current light state of the tl
    light_state           = xQueueCreate(1, sizeof(uint8_t));
	//this holds the normalized adc value 
		//set by the light control function
    trafficFlowLightQueue = xQueueCreate(1, sizeof(double));
    trafficFlowCarsQueue  = xQueueCreate(1, sizeof(double));
    trafficLightQueue     = xQueueCreate(1, sizeof(uint8_t));
	
    // create tasks
	xTimerCreate(
		"ADC_Timer",
		100,
		pdFALSE,
		0,
		ADC_callBack);
	xTaskCreate(
		Adjust_Traffic,
		"AdjustTraffic",
		configMINIMAL_STACK_SIZE + 120,
		NULL, 2, &adjust_traffic_handle
	);
	xTaskCreate(
		Traffic_Light,
		"Traffic_Light",
		configMINIMAL_STACK_SIZE + 120,
		NULL, 2, &traffic_light_handle
	);
	
	// xTaskCreate(update_traffic_rate_task, "rate", configMINIMAL_STACK_SIZE +128,NULL,2,NULL);

	// Register for kernel-aware debugging
	// vQueueAddToRegistry(trafficFlowQueue, "trafficFlowQueue");

	//turn on scheduler
	vTaskStartScheduler();

	return 0;
}

static void ADC_callBack(TimerHandle_t xTimer) {
	vTaskNotifyGive(adjust_traffic_handle);
}

static void Adjust_Traffic(void *pvParameters) {
	uint16_t ADC_Result = poll_adc_function();
	double ADC_Norm = (double)ADC_Result / 4095.0;
	// check if notified then send, check 
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	xQueueOverwrite(trafficFlowCarsQueue, &ADC_Norm);
	xQueueOverwrite(trafficFlowLightQueue, &ADC_Norm);
	// after send, notify traffic light and create traffic tasks
	vTaskNotifyGive(traffic_light_handle);
}

//notified by the adjust traffic
static void Traffic_Light(void *pvParameters) {
	uint32_t delay = 0;
	uint8_t received = 0;
	uint8_t traffic_light = red;
	double difference; 
	uint32_t green_delay = 0;
	
	// once notified do calculation
	if (xQueueReceive(trafficFlowLightQueue, &received, pdMS_TO_TICKS(1000))) {}
	TickType_t CurrentTime = xTaskGetTickCount();
	difference = maximum_time - (minimum_time)*received; 
	
	// traffic light state machine
	green_delay = (uint32_t)(((1.0+received)*difference)/3.0);
	switch (traffic_light)
	{
		case red:
			traffic_light = green;
			delay = green_delay;
		case green:
			traffic_light = yellow;
			delay = yellow_time;
		default:
			traffic_light = red;
			delay = maximum_time - green_delay;
	}
	// send light state to queue
	xQueueSend(trafficLightQueue, &traffic_light, pdMS_TO_TICKS(1000));
	tl_change_state(traffic_light);
	vTaskDelay(pdMS_TO_TICKS(delay));
}



//light_time - get time for light

static uint32_t light_time(uint16_t adc_value, uint8_t light){
//2 cases are light green, 1 if it is red, 0
//yellow light is constant
	uint32_t time = 0;
	if(light){
		//light is green
		time = default_light_time +  default_light_time*(adc_value / 4095);
	}
	else{
		//light is red
		time = 2*default_light_time +  default_light_time*(adc_value / 4095);
	}
	return (uint32_t)time;
}


//traffic light changer helper fun
static void tl_change_state(uint8_t light){
	//reset all lights
	GPIO_ResetBits(GPIOC, red | green | yellow);
	//set light
	if(light == green){
		GPIO_SetBits(GPIOC, green);
	}else if(light == yellow){
		GPIO_SetBits(GPIOC, yellow);
	}else{
		GPIO_SetBits(GPIOC, red);
	}
}





void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	// TODO: Setup the clocks, etc. here

	//Enable Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	//Enable pins
	GPIO_InitTypeDef GPIO_config;
	//output pins

	//** might make shift reg pull down active low

	GPIO_config.GPIO_Pin = red_light | green_light | yellow_light ;
	GPIO_config.GPIO_Mode = GPIO_Mode_OUT; //output
	GPIO_config.GPIO_OType = GPIO_OType_PP;//push pull

	//** make pulldown
	GPIO_config.GPIO_PuPd = GPIO_PuPd_DOWN;//pull up
	//**

	GPIO_config.GPIO_Speed = GPIO_Speed_50MHz;//speed good enough for shift reg
	GPIO_Init(GPIOC, &GPIO_config);

	//configure shift register output
	GPIO_InitTypeDef GPIO_config2;
	GPIO_config2.GPIO_Pin = shift_reg_clock | shift_reg_reset |shift_reg_data;
	GPIO_config2.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOC, &GPIO_config2);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; // Analog mode
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3; // PC3
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // No pull resistor otherwise would affect the voltage from pot
	GPIO_Init(GPIOC, &GPIO_InitStruct);


	ADC_Cmd(ADC1, ENABLE); // Power on ADC1
	// Configure Channel 13 (PC3), rank 1, 84-cycle sample time
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_84Cycles);
}

//technically not polling, I am not changing the name
static uint16_t poll_adc_function(void){
	uint16_t converted_data;
	// 1. Trigger the conversion
	ADC_SoftwareStartConv(ADC1);
	// 2. Wait for the End-of-Conversion flag
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	// 3. Read the result (0-4095)
	converted_data = ADC_GetConversionValue(ADC1);
	return converted_data;
}




























//
