/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
	#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

	TaskHandle_t Task_1_Handler = NULL;
	TaskHandle_t Task_2_Handler = NULL;
	TaskHandle_t Task_3_Handler = NULL;
	TaskHandle_t Task_4_Handler = NULL;
	TaskHandle_t Task_5_Handler = NULL;
	TaskHandle_t Task_6_Handler = NULL;
	
	int task_1_in_time = 0, task_1_out_time = 0, task_1_total_time;
	int task_2_in_time = 0, task_2_out_time = 0, task_2_total_time;
	int system_time = 0;
	int cpu_load = 0;
	
	char runTimeStatsBuff[190];
	
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
	
	//signed char output[17];
	const signed char button1OFF[] = "B1 L";
	const signed char button1ON[] = "B1 H";
	const signed char button2OFF[] = "B2 L";
	const signed char button2ON[] = "B2 H";
	const signed char periodicTransmit[] = "Test";
	
	QueueHandle_t xQueue;



static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

pinState_t button1State,button2State;

/* Task to be created. */
void Button_1_Monitor( void * pvParameters )
{
	int xLastWakeTime = xTaskGetTickCount();
	
       for( ;; )
    {
			GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
			button1State = GPIO_read(PORT_0,PIN0);
			
			if (button1State == PIN_IS_LOW)
			{				
					xQueueSend(xQueue, ( void * )&button1OFF, ( TickType_t ) 10  );
				
			}
			else if (button1State == PIN_IS_HIGH)
			{
					xQueueSend(xQueue, ( void * )&button1ON, ( TickType_t ) 10 );
			}
			GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
			
      vTaskDelayUntil(&xLastWakeTime,50);
			
    }
    
}
void Button_2_Monitor( void * pvParameters )
{
	int xLastWakeTime = xTaskGetTickCount();
	
       for( ;; )
    {
			GPIO_write(PORT_0,PIN3,PIN_IS_HIGH);
			button2State = GPIO_read(PORT_0,PIN2);
			
			if (button2State == PIN_IS_LOW)
			{				
					xQueueSend(xQueue, ( void * )&button2OFF, ( TickType_t ) 10  );
				
			}
			else if (button2State == PIN_IS_HIGH)
			{
					xQueueSend(xQueue, ( void * )&button2ON, ( TickType_t ) 10 );
			}
			GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
      vTaskDelayUntil(&xLastWakeTime,50);
			
    }
    
}

void Periodic_Transmitter( void * pvParameters )
{
	int xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		GPIO_write(PORT_0,PIN4,PIN_IS_HIGH);
		xQueueSend(xQueue, ( void * )&periodicTransmit, ( TickType_t ) 10  );
		GPIO_write(PORT_0,PIN4,PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime,100);
	}
}

void Uart_Receiver( void * pvParameters )
{
	int xLastWakeTime = xTaskGetTickCount();
	char recieved[10];
	for( ;; )
    {		
			GPIO_write(PORT_0,PIN5,PIN_IS_HIGH);
		if( xQueue != NULL )
   {
      /* Receive a message from the created queue to hold pointers.  Wait for 10
      ticks if a message is not immediately available.   */
      if( xQueueReceive( xQueue, ( void * )recieved, ( TickType_t ) 10 ) != pdTRUE )
      {
        //vSerialPutString("Error Receiving from Queue",5);
      }
			else
			{
				vSerialPutString((const signed char*)&recieved,7);
				
			}
					
		}
	 GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
	 vTaskDelayUntil(&xLastWakeTime,20);
	}
}

void Load_1_Simulation( void * pvParameters )
{
	int xLastWakeTime = xTaskGetTickCount();
	int i = 0;
	
	for( ;; )
    {
			GPIO_write(PORT_0,PIN6,PIN_IS_HIGH);
      for (i=0; i<33333; i++)  							//Execution time: 5ms
			{
				i=i;
			}
	
			GPIO_write(PORT_0,PIN6,PIN_IS_LOW);
		//vTaskGetRunTimeStats (runTimeStatsBuff);
		//vSerialPutString((const signed char*)runTimeStatsBuff,150);
			vTaskDelayUntil(&xLastWakeTime,10);
		}
	}
		
void Load_2_Simulation( void * pvParameters )
{
	int xLastWakeTime = xTaskGetTickCount();
	int i = 0;
	
	for( ;; )
    {
			GPIO_write(PORT_0,PIN7,PIN_IS_HIGH);
      for (i=0; i<80000; i++)								//Execution time: 12ms
			{
				i=i;
			}
			GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
	
		vTaskDelayUntil(&xLastWakeTime,100);
		}
}

/*Implement Tick Hook*/

void vApplicationTickHook (void)
{
	GPIO_write(PORT_0,PIN8,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN8,PIN_IS_LOW);
}

/*Implement Idle Hook*/

void vApplicationIdleHook (void)
{
	GPIO_write(PORT_0,PIN9,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN9,PIN_IS_LOW);
}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	xQueue = xQueueCreate( 20, sizeof( char* ) );
	
    /* Create Tasks here */
	xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button1 Monitor",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,				      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,								/* Priority at which the task is created. */
                    &Task_1_Handler,	/* Used to pass out the created task's handle. */
										50
										);      				/* Task Period */
	xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button2 Monitor",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,				      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,								/* Priority at which the task is created. */
                    &Task_2_Handler,	/* Used to pass out the created task's handle. */
										50
										);      				/* Task Period */

xTaskPeriodicCreate(
                    Periodic_Transmitter,       /* Function that implements the task. */
                    "Periodic Transmitter",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,				      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,								/* Priority at which the task is created. */
                    &Task_3_Handler,	/* Used to pass out the created task's handle. */
										100								/* Task Period */
										); 	
										
xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "Uart Receiver",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,				      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,								/* Priority at which the task is created. */
                    &Task_4_Handler,	/* Used to pass out the created task's handle. */
										20								/* Task Period */
										);      				
										
	

xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load 1 Simulation",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,				      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,								/* Priority at which the task is created. */
                    &Task_5_Handler,	/* Used to pass out the created task's handle. */
										10								/* Task Period */
										); 

xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load 2 Simulation",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,				      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,								/* Priority at which the task is created. */
                    &Task_6_Handler,	/* Used to pass out the created task's handle. */
										100								/* Task Period */
										);  										
											
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


