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
     *    robust, strictly quality controlled, supported, and cross          *
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

//added
#include "stm32f4xx_tim.h"

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
#define green 0
#define red 2

//sample rate of pot - lab manual
#define sample_rate_pot 100

//////////////////////////////////////////////////////////
//pin names
	//pin names for adc
#define adc_pin GPIO_Pin_3//lab manual pin 3
	//pin names for tl
#define red_light GPIO_Pin_0 //lab manual pin 0
#define yellow_light GPIO_Pin_1 //lab manual pin 1
#define green_light GPIO_Pin_2 //lab manual pin 2
	//shift register pins
#define shift_reg_data GPIO_Pin_6
	//shift reg clock pin
#define shift_reg_clock GPIO_Pin_7
	//shift reg reset pin
#define shift_reg_reset  GPIO_Pin_8
//////////////////////////////////////////////////////////


//**********************************
//might have to change?
#define mainQUEUE_LENGTH 100

//default yellow time (const)
#define yellow_time = 2500


//////////////////////////////////////////////////////////
//ques
static xQueueHandle poll_adc_que; //que for polling adc values from pot
static xQueueHandle generator_que; //generate cars based on pot
static xQueueHandle light_state;//current light state
//////////////////////////////////////////////////////////

//Handles for sl timer
//street light timers - when timer goes off rtos will run this handle and this will wake the traffic light control to switch states
//Timer handle lets us stop start, change the wait of the timer
static TimerHandle traffic_light_timer;//

//task handles lets us wake up a specific task, stop and start the task if higher priority comes in
static TaskHandle tl_task_handle;


//////////////////////////////////////////////////////////
//hardware functions
	/*
	 * TODO: Implement this function for any hardware specific clock configuration
	 * that was not already performed before main() was called.
	 */
static void prvSetupHardware( void );//sets up clocks and gpio pins
	//adc poll helper fucntion
static uint16_t poll_adc_function(void);//polls the adc reading from pot to get new rate of gen
	//display street sends output to daisy chain to update screen(cars moving)
static void sr_delay_timer(uint32_t street);

	//changes the output of the tl leds
static void tl_change_state(uint32_t light);

//////////////////////////////////////////////////////////
//tasks
static void update_traffic_rate_task(void *pvParameters); //polls pot and updates the rate of gen
static void traffic_light_control_task(void *pvParameters);//changes light states
static void generate_car_task(void *pvParameters);//produces car generation
static void display_street(void *pvParameters);//updates the street leds


/////////////////////////////////////////////////////////////////////
//random number generator - need to generate number to compare to the rate provided by the adc




/////////////////////////////////////////////////////////////////////
//need shift register delay function 1us will use timer for accuracy
void static shift_reg_delay();

static void delay_us(uint32_t time);


static uint32_t light_time(uint16_t adc_value, uint8_t light);

/*-----------------------------------------------------------*/

int main(void)
{
	//run set up hardwar function
	prvSetupHardware();

	//build tasks xQueueCreat(length, size)
	poll_adc_que = xQueueCreate(1, sizeof(unint16_t)); //adc resolution is 12
	generator_que = xQueueCreate(mainQUEUE_LENGTH, sizeof(unint8_t)); //generate cars based on pot
	light_state = xQueueCreate(1, sizeof(unint8_t));//current light state

	//build tasks xTaskCreate(function, name for debuging, size

	xTaskCreate(update_traffic_rate_task, "rate", configMINIMAL_STACK_SIZE +128,NULL,2,NULL);
	xTaskCreate(traffic_light_control_task, "light control", configMINIMAL_STACK_SIZE +128,NULL,2,NULL);
	xTaskCreate(generate_car_task, "generate car", configMINIMAL_STACK_SIZE +128,NULL,3,NULL);
	xTaskCreate(display_street, "display street", configMINIMAL_STACK_SIZE +128,NULL,2,NULL);

	//turn on scheduler
	vTaskStartScheduler();

	return 0;
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
	GPIO_config.GPIO_Pin = red_light | green_light | yellow_light | shift_reg_clock | shift_reg_reset;
	GPIO_config.GPIO_Mode = GPIO_Mode_OUT; //output
	GPIO_config.GPIO_OType = GPIO_OType_PP;//push pull
	GPIO_config.GPIO_PuPd = GPIO_PuPd_UP;//pull up
	GPIO_config.GPIO_Speed = GPIO_Speed_50MHz;//speed good enough for shift reg
	GPIO_Init(GPIOC, &GPIO_config);

	//Enable ADC
	// ADC1 is on APB2 - enables adc clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	// The GPIO pin (PC3) still needs AHB1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; // Analog mode
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3; // PC3
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // No pull resistor otherwise would effect voltage from pot
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	ADC_Init(ADC1, &GPIO_InitStruct);
	ADC_Cmd(ADC1, ENABLE); // Power on ADC1
	// Configure Channel 13 (PC3), rank 1, 84-cycle sample time
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_84Cycles);

	//turn on adc
	ADC_Cmd(ADC1,ENABLE);

	//shift_reg_delay();
}

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


//***
//static void delay_us(uint32_t time){
//	uint32_t start = TIM2->CNT;
//
//}

//**
//shift reg functions
static void sr_falling_edge(void){
	//make shift register clock high to wait for the next falling edge
	GPIO_SetBits(GPIOC, shift_reg_clock);
	//wait 2 ms enough time for clock to be high

	GPIO_ResetBit(GPIOC, shift_reg_clock);
	//wait 2 ms enough time for clock to be high

}



//traffic light changer helper fun
static void tl_change_state(uint32_t light){
	//reset all lights
	GPIO_ResetBit(GPIOC, red | green | yellow);
	//set light
	if(light == green){
		GPIO_SetBits(GPIOC, green);
	}else if(light == yellow){
		GPIO_SetBits(GPIOC, yellow);
	}else{
		GPIO_SetBits(GPIOC, red);
	}
}




/////////Tasks///////////////////////////////////////
static void update_traffic_rate_task(void *pvParameters){
	//TickType_t used to measure time
 	TickType_t current_poll_time = xTaskGetTickCount();
	//poll adc value from pot
	for(;;){
		uint16_t adc_value = poll_adc_function();
		//give that to the que for other tasks to use 
		xQueueOverwrite(poll_adc_que, &adc_value);
		
		//vTaskDelay used to delay periodic tast
			//pdMS_TO_TICKS converts micro sec to ticks
		vTaskDelay(&current_poll_time, pdMS_TO_TICKS(sample_rate_pot));

	}
}


/////////Car Gen///////////////////////////////////////
//need to generate randomly ~ to the adc value converted from pot
static void generate_car_task(void *pvParameters){
	uint16_t adc_value;
	#define min_delay = 500
	#define max_delay = 3500//max is technically 4000 if the rand is completely added 
	#define rand_max = 501
	
	for(;;){
		//get the first item in the queue for adc value 
		xQueuePeek(generator_que, &adc_value, portMax_DELAY);
		//normalize ADC value
			//need float for normalization
		float normalize_adc = (float)adc_value / (float)4095;
		//Makes sure we always get the minimum delay if the normalized adc was close to zero which would be way to fast of a rate
		//originally had no max_delay*adc norm, and that did not work
		float none_delay = max_delay - normalize_adc*(max delay - min_delay);
		//delay time is none_delay + rand
		//note tecnically the max is 4000
		uint32_t delay_time_gen = (uint32_t)none_delay (uint32_t)(rand() % rand_max);
		//give this delay to the periodic task
		vTaskDelay(pdMS_TO_TICKS(delay_time_gen));
		//send to generator que to gen car 
		uint8_t gen_car_flag = 1;
		xQueueSend(generator_que, &gen_car_flag, 0);
	}
}


/////////Taffic ligh control task///////////////////////////////////////
	//Traffic light timer handle/callback - wakes up the traffic light task when timer goes off 
static TimerHandle traffic_light_timer(TimerHandle tim){
	//When the timer goes off, we are going to trigger a traffic control event
	//we want this code to be short because it has low priority and the light has high priority
	//signals an event. the event is changing traffic light state
	vTaskNotifyGive(tl_task_handle);
}

	//light_time - get time for light 

static uint32_t light_time(uint16_t adc_value, uint8_t light){
	//2 cases is light green 1 if it is red 0
	//yellow light is const
	uint32_t time;
	#define default_light_time = 2000
	if(light){
		//light is green
		time = default_light_time +  default_light_time*(uint32_t(adc_value) / 4095)
	}else{
		//light is red 
		time = 2*default_light_time +  default_light_time*(uint32_t(adc_value) / 4095)
	}
	return time;
}

	//Traffic light task
static void traffic_light_control_task(void *pvParameters){
	//default to red light
	xQueueOverwrite(light_state,@red_light);
	//build periodic timer  xLightTimer - anme, time, one shot, 
		//vLightTimerCallback
	traffic_light_timer = xTimerCreate("tl_timer",pdMS_TO_TICKS(2000),pdFALSE,NULL,  tl_task_handle);
	configASSERT(traffic_light_timer);
}


/////////Display street control///////////////////////////////////////











