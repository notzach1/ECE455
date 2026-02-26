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
#define mainQUEUE_LENGTH 5
//////////////////////////////////////////////////////////

//**********************************
//default yellow time (const)
#define mainQUEUE_LENGTH 100
#define yellow_time 1000
#define minimum_time 4000 
#define maximum_time 8000

//////////////////////////////////////////////////////////
//queues
static xQueueHandle gen_car_que; // uint8_t 1 gen car 0 no car
	//light state handles hold current light state
static xQueueHandle light_state_street;//uint8_t holds state of the light
static xQueueHandle light_state;
	//flow ques hold the normalized ADC rate 
static xQueueHandle trafficFlowLightQueue; 
static xQueueHandle trafficFlowCarsQueue;

//task handles
static TaskHandle_t adjust_traffic_handle;
static TaskHandle_t traffic_light_handle;
static TaskHandle_t gen_handle;

//timers
static TimerHandle_t tl_timer;
static TimerHandle_t adc_timer;

//tasks 
//static void Traffic_Light(void *pvParameters);
//static void ADC_callBack(TimerHandle_t xTimer);
//static void Adjust_Traffic(void *pvParameters);


//////////////////////////////////////////////////////////
//hardware functions
static void prvSetupHardware( void );//sets up clocks and gpio pins
	//adc poll helper function
static uint16_t poll_adc_function(void);//polls the adc reading from pot to get new rate of gen
//changes the output of the TL LEDs
static void tl_change_state(uint8_t light);

/*-----------------------------------------------------------*/
main(void)
{	
	//initialize hardware
	prvSetupHardware();
	//build ques 
	xQueueHandle gen_car_que = xQueueCreate(1, sizeof(uint8_t));
		//2 ques for light state
	xQueueHandle light_state = xQueueCreate(1, sizeof(uint8_t));
	xQueueHandle light_state_street = xQueueCreate(1 sizeof(uint8_t));
		//2 ques for traffic flow 
	xQueueHandle trafficFlowLightQueue = xQueueCreate(1, sizeof(double));
	xQueueHandle trafficFlowCarsQueue = xQueueCreate(1, sizeof(double));

	//build tasks 
	xTaskCreate(Adjust_Traffic,"Adjust_Traffic",configMINIMAL_STACK_SIZE + 200, NULL, 1, &adjust_traffic_handle);
    xTaskCreate(Traffic_Light,"Traffic_Light",configMINIMAL_STACK_SIZE + 200, NULL, 1, &traffic_light_handle);
    xTaskCreate(TL_Display,"TL_Display",configMINIMAL_STACK_SIZE + 200, NULL, 1, NULL);
    xTaskCreate(Car_Gen,"Car_Gen",configMINIMAL_STACK_SIZE + 200, NULL, 1, &gen_handle);
    xTaskCreate(Display_Street,"Display_Street", configMINIMAL_STACK_SIZE + 200, NULL, 1, NULL);

	//one-shot timer for traffic light
	tl_timer = xTimerCreate("Traffic_Timer",pdMS_TO_TICKS(maximum_time),pdFALSE,NULL,tl_timer_callback);
	//periodic timer for ADC
	adc_timer = xTimerCreate(
        "Adc_timer",
        pdMS_TO_TICKS(500),
        pdTRUE,
        NULL,
        ADC_callBack
    );
	//start ADC conversion
	xTimerStart(adc_timer,0);//no delay

	vTaskStartScheduler();

	while(1){
	}
	return 0;
}

//complete
//ADC_callBack - wakes the traffic adjust task when the adc timer goes off
//periodic timer every 500ms
static void ADC_callBack(TimerHandle_t xTimer) {
	vTaskNotifyGive(adjust_traffic_handle);
}

//complete
//tl_timer_callback calls itself back
//one shot timer 
static void tl_timer_callback(TimerHandle_t xTimer) {
	vTaskNotifyGive(traffic_light_handle);
}

static void Adjust_Traffic(void *pvParameters) {
	uint16_t ADC_Result = 0;
	double ADC_Norm = 0;
	// check if notified then send, check 
	while(1){
		//wait for notification
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		//get ADC value
		ADC_Result = poll_adc_function();
		//normalize adc value
		ADC_Norm = (double)ADC_Result / 4095.0;
		//give to ques 
		xQueueOverwrite(trafficFlowCarsQueue, &ADC_Norm);
		xQueueOverwrite(trafficFlowLightQueue, &ADC_Norm);
		// after send, notify traffic light and create traffic tasks
			//notify traffic_light task
			//dont think we need the traffic light task will deal with timer 
		//vTaskNotifyGive(traffic_light_handle);
			//notify 
		vTaskNotifyGive(gen_handle);
	}
}

//notified by the adjust traffic
static void Traffic_Light(void *pvParameters) {
	//initialization - runs once
	uint32_t delay = 0;
	double received = 0;
	uint8_t traffic_light = red;//start with red light
	double difference = 0;
	uint32_t green_delay = 0;
	uint32_t red_delay = 0;
	double temp =0;

	xQueueOverwrite(light_state, &traffic_light);
	xQueueOverwrite(light_state_street, &traffic_light);
	xTimerChangePeriod(tl_timer, pdMS_TO_TICKS(maximum_time), 0);

	while(1){
		// once notified, do the calculation
			//might not need the pdMs could be set to zero
		//get the latest adc value
		if (xQueueReceive(trafficFlowLightQueue, &received, 0)) {}
		//do calculations for light delay
		temp = (double)minimum_time + received*(double(maximum_time - minimum_time));
		
		difference = 2.0 - 1.5 * received;
		red_delay =(uint32_t)(cycle_time / (1.0 + difference));
        green_delay = (uint32_t)(cycle_time - (double)red_delay);

		//change light state 
		switch (traffic_light)
		{
			case red:
				traffic_light = green;
				delay = green_delay;
				break;
			case green:
				traffic_light = yellow;
				delay = yellow_time;
				break;
			default:
				traffic_light = red;
				delay = maximum_time - green_delay;
		}
		// send light state to queues
		xQueueOverwrite(light_state, &traffic_light);
		xQueueOverwrite(light_state_street, &traffic_light);
		//delay task to call itself again 
		xTimerChangePeriod(tl_timer, pdMS_TO_TICKS(delay_ms), 0);
	}
}

//traffic light output task 
	//should just run tl_change helper function based on the light state in the que
static void TL_Display(void *pvParameters){
	uint8_t state = red
	while(1){
		 xQueueReceive(light_state, &state, portMAX_DELAY);
		tl_change_state(state);
	}
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


//car_generator task 
	//notified by the flow adjust task that a new adc value is ready 
	//generates an array to drive the shift register
	//updates the generate car que
	//notifies the street display task 
static void Car_Gen(void *pvParameters){
	double recieved =0;
	uint8_t car =0;
	int percent =0;
	int rand = 0;//convert adc to a int that denotes the prob of car spawning ie 0.5 -> 50%
	
	while(1){
		xQueueReceive(trafficFlowCarsQueue, &received, 0);
		percent = (int)(received * 100.0);//get chance of spawn based on norm adc
		rand = rand() %100; gives us value from 0 to 99
		if(rand<percent){
			car =1; //gen car
			//only sends a message when the car is generated 
			xQueueSend(Car_Gen, &car, 0);
		}else{
			car = 0;//dont gen car
			
		}
		///run itself again every 500 ms 
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

//street display task
static void Display_Street(void *pvParameters){
	car_postion[19]= {0};
	uint32_t new_car = 0; 
	uint8_t light = red;
	uint car_count = 0;
	
	while(1){
		//get latest light state
		//want to count the number of cars recieved 
		while(xQueueReceive(light_state_street, &light, 0)){
			new car_count++;
		}
		//get most recent light state 
		xQueueReceive(light_state_street, &light, 0);
		//we want to clear the last led - car is leaving
		car_postion[18] =0;
		//move car along by one led
		for(int i = 18; i>0;i--){


		}
		
		
		



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
