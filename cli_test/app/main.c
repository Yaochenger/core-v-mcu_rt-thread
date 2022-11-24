/*
 * FreeRTOS Kernel V10.2.1
 * 1 tab == 8 spaces!
 */

/*
 * Create implementation of vPortSetupTimerInterrupt() if the CLINT is not
 * available, but make sure the configCLINT_BASE_ADDRESS constant is still
 * defined.
 * 
 * Define vPortHandleInterrupt to whatever the interrupt handler is called.  In
 * this case done by defining vPortHandleInterrupt=SystemIrqHandler on the
 * assembler command line as SystemIrqHandler is referenced from both FreeRTOS
 * code and the libraries that come with the Vega board.
 *

/* FreeRTOS kernel includes. */ 
#include <FreeRTOS.h>
#include <hal/include/hal_gpio_pulp.h>
#include <task.h>

/* c stdlib */
#include <stdio.h>

/* PULPissimo includes. */
#include "target/core-v-mcu/include/core-v-mcu-config.h"
#include "target/core-v-mcu/include/core-v-mcu-system.h"
#include "hal/include/hal_timer_irq.h"
#include "hal/include/hal_fll.h"
#include "hal/include/hal_irq.h"
#include "drivers/include/udma_uart_driver.h"  //

#include <stdint.h>
#include "hal/include/hal_udma_ctrl_reg_defs.h"
#include "hal/include/hal_udma_uart_reg_defs.h"


#include <target/core-v-mcu/include/core_pulp_cluster.h>
#include <target/core-v-mcu/include/core-v-mcu-config.h>

//#include "pmsis/implem/drivers/fc_event/fc_event.h"
#include "hal/include/hal_fc_event.h"
/* TODO: weird include */
#include "target/core-v-mcu/include/core-v-mcu-properties.h"
#include "hal/include/hal_irq.h"
#include "hal/include/hal_soc_eu.h"
#include "hal/include/hal_apb_soc_ctrl_reg_defs.h"

#include "drivers/include/udma_uart_driver.h"
#include "drivers/include/udma_i2cm_driver.h"
#include "drivers/include/udma_qspi_driver.h"

#include "hal/include/hal_apb_i2cs.h"

//#include <rthw.h>
#include "rtthread.h"
#include "portmacro.h"

#include "hal/include/hal_udma_ctrl_reg_defs.h"
#include "hal/include/hal_udma_uart_reg_defs.h"

#include "rtconfig.h"
/******************************************************************************
 * Aim:
 * This project provides two demo applications.  A simple blinky style project, and a more comprehensive test and demo application.  The
 * The mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting (defined in this file) is used to select between the two.
 * The simply blinky demo is implemented and described in main_blinky.c.
 * The more comprehensive test and demo application is implemented and described in main_full.c.
 *
 * This file implements the code that is not demo specific, including the hardware setup and standard FreeRTOS hook functions.
 *
 */

/* Set mainCREATE_SIMPLE_BLINKY_DEMO_ONLY to one to run the simple blinky demo,or 0 to run the more comprehensive test and demo application. */
/* #define mainCREATE_SIMPLE_BLINKY_DEMO_ONLY	0*/

/*
 * main_blinky()    
 * is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 * main_full()       
 *  is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 0.
 */

extern void main_blinky( void );

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file.  See https://www.freertos.org/a00016.html */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/* Prepare hardware to run the demo. */
static void prvSetupHardware( void );



/*-----------------------------------------------------------*/
#include <app/include/i2c_task.h>
#include <app/include/efpga_tests.h>

#include "libs/cli/include/cli.h"
#include "hal/include/hal_udma_i2cm_reg_defs.h"
#include "ringbuffer.h"
#define mainCREATE_SIMPLE_BLINKY_DEMO_ONLY   1

char  note[]="A";
char  note2[]="B";
char  note3[]="C";
extern rt_uint8_t uart_rxbuffer[16];
extern struct rt_ringbuffer uart_rxTCB;
extern struct rt_semaphore  shell_rx_semaphore;
void _deley(int i)
{
	for(; i > 0; i--)
	{
		for (int a = 0; a < 10000; a++)
		{
			a++;
		}
	}
}

uint16_t writeraw(uint8_t uart_id, uint16_t write_len, uint8_t* write_buffer) {
	UdmaUart_t*				puart = (UdmaUart_t*)(UDMA_CH_ADDR_UART + uart_id * UDMA_CH_SIZE);

	while (puart->status_b.tx_busy) {  // ToDo: Why is this necessary?  Thought the semaphore should have protected
	}

	puart->tx_saddr = (uint32_t)write_buffer;
	puart->tx_size = write_len;
	puart->tx_cfg_b.en = 1; //enable the transfer

	return 0;
}

rt_hw_console_output(const char *str)
{
	writeraw(0, strlen(str), (uint8_t*)str);
}

char rt_hw_console_getchar(void)
{
	char ch=0;
	while(rt_ringbuffer_getchar(&uart_rxTCB,(rt_uint8_t*)&ch)!=0)
	{
		rt_sem_take(&shell_rx_semaphore,RT_WAITING_FOREVER);
	}
	return ch;
	//return udma_uart_getchar(0);
}

extern void vPortSetupTimerInterrupt(void);

static struct rt_thread led1_thread;
static rt_thread_t led2_thread = RT_NULL;

rt_uint8_t rt_irq_stack[1024];
//rt_uint32_t rt_irq_stack_top = (&rt_irq_stack[0] + 1024);
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t rt_led1_thread_stack[1024];
static void led1_thread_entry(void* parameter);
static void led2_thread_entry(void* parameter);

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
	#define RT_HEAP_SIZE (20*1024)
	static rt_uint8_t rt_heap[RT_HEAP_SIZE];
	void *rt_heap_begin_get(void)
	{
		return rt_heap;
	}
	void *rt_heap_end_get(void)
	{
		return rt_heap + RT_HEAP_SIZE;
	}
#endif

void led_init(void)
{
	rt_kprintf("Hello RT-Thread 1\r\n");
	rt_kprintf("RT-Thread INIT TEST \r\n");
	rt_kprintf("RT-Thread TEST SUCCESS \r\n");
}
INIT_APP_EXPORT(led_init);

int main(void)
{
//	rt_kprintf("Hello RT-Thread 2\r\n");
//	rt_thread_init(&led1_thread,                 /* 线程控制块 */
//                   "led1",                       /* 线程名字 */
//                   led1_thread_entry,            /* 线程入口函数 */
//                   RT_NULL,                      /* 线程入口函数参数 */
//                   &rt_led1_thread_stack[0],     /* 线程栈起始地址 */
//                   sizeof(rt_led1_thread_stack), /* 线程栈大小 */
//                   6,                            /* 线程的优先级 */
//                   20);                          /* 线程时间片 */

//	/* 启动线程，开启调度 */
//    rt_thread_startup(&led1_thread);             /* 启动线程，开启调度 */

//
//                       		/* 线程控制块指针 */
//    led2_thread = rt_thread_create( "led2",      /* 线程名字 */
//                      led2_thread_entry,   		/* 线程入口函数 */
//                      RT_NULL,             		/* 线程入口函数参数 */
//                      512,                 		/* 线程栈大小 */
//                      5,                   		/* 线程的优先级 */
//                      20);                 		/* 线程时间片 */
//	/* 启动线程，开启调度 */
//    rt_thread_startup(led2_thread);
//    while (1)
//    {
//        rt_enter_critical();
//    	  writeraw(0,sizeof(note), (uint8_t*)note);
//    	  rt_kprintf("");
//        rt_exit_critical();
//        rt_thread_delay(500);   /* 延时500个tick */
//    }

	while(1)
	{
		 rt_thread_delay(500);
	}
}


static void led1_thread_entry(void* parameter)
{
    while (1)
    {
    	writeraw(0,sizeof(note), (uint8_t*)note2);
        rt_thread_delay(500);   /* 延时500个tick */
    }
}

static void led2_thread_entry(void* parameter)
{
	while (1)
	{
		writeraw(0,sizeof(note), (uint8_t*)note3);
		rt_thread_delay(500);   /* 延时500个tick */
	}
}
void rt_hw_board_init()
{
    /* System Clock Update */
	system_init();

	vPortSetupTimerInterrupt();

	volatile uint32_t mtvec = 0;
	__asm volatile( "csrr %0, mtvec" : "=r"( mtvec ) );
	__asm volatile( "csrs mie, %0" :: "r"(0x880) );
    /* System Tick Configuration */
    //SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif

	rt_ringbuffer_init(&uart_rxTCB,uart_rxbuffer,16);
	rt_sem_init(&(shell_rx_semaphore),"shell_rx",0,0);
}
/*------------------------------------------------------*/

void printf_task( void *pParameter )
{
	while(1)
	{
		rt_kprintf("Hello RT-Thread\r\n");
		rt_thread_delay(1000);
	}
}
static void prvSetupHardware( void )
{
	/* Init board hardware. */
	system_init();
}
/*-----------------------------------------------------------*/

void vToggleLED( void )
{
	gpio_pin_toggle( 0x5 );
}
/*---------------------------------------------------------*/


void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. 
	It is a hook function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
//	printf( "error: application malloc failed\n" );
	__asm volatile( "ebreak" );
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	__asm volatile( "ebreak" );
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* The tests in the full demo expect some interaction with interrupts. */
	#if( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
	{
		extern void vFullDemoTickHook( void );
		vFullDemoTickHook();
	}
	#endif
}
/*-----------------------------------------------------------*/
