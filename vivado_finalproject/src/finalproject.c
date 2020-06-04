/**
 * @file myapp.c
 * @author Michael Escue (michael.escue@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-25
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/*******************************************************************/
/*  Included Source Files    */
/*******************************************************************/

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* BSP includes. */
#include "xtmrctr.h"
#include "xgpio.h"
#include "sleep.h"
#include "xwdttb.h"	// Watchdog Timer
#include "nexys4IO.h"
#include "xintc.h"
#include "xuartlite.h"
#include "PmodESP32.h"

/*  Function Prototypes Includes    */
#include "finalproject.h"

/*  O/S Concurrency Mechanisms    */
/*******************************************************************/

/* Queues   */

/* Semaphores   */

/* Mutexes  */

/*  Global Variables    */
/*******************************************************************/

/*  Health Checks   */
static bool  master_health_check = TRUE; // Initial health of Master is TRUE.

/* UART Buffers  */
static u8 buf_uart[BUF_LEN] = {0};   // Initialize buffer
static u8 buf_esp[BUF_LEN] = {0};   // Initialize buffer
static u32 length = 0;   // Number of bytes to read/write.

/* Return Checks    */
static BaseType_t xStatus;


/*  Device Instances    */
/*******************************************************************/

/* WDT  */
XWdtTb inst_wdt;

/*  INTC    */
XIntc inst_intc;

/* UART    */
XUartLite inst_uart0;

/* ESP32    */
PmodESP32 inst_esp;

/*  Task Handles    */
/*******************************************************************/

/*  ISRs     */
/*******************************************************************/

/* WDT Interrupt Handler    */
static void isr_wdt(void *pvUnused)
{

	/*	Debug Exceptions	*/
	#ifdef DEBUG_NO_MASTER
        master_health_check = TRUE; // Provides an exception to the crash condition.
        #endif

    /* Check crash conditions   */
    if(master_health_check)
    {
		// No violation of health check, reset all for next update.
		master_health_check = FALSE;

		// Restart Timer to prevent reset
		XWdtTb_RestartWdt(&inst_wdt);
	}
	else
    {
		if(master_health_check == FALSE)
        {
			print("WDT:\tMaster Thread Crashed.\r\n");
		}
        else
		    XWdtTb_IntrClear(&inst_wdt);
	}

	#ifdef DEBUG_WDT_TICK
        print(" (W) \r\n");
        #endif

}

/*  Main Program    */
/*******************************************************************/

/*  Main    */
int main(void)
{

    /*  Handle WDT reset    */

    /* Initialize Hardware  */
    prvSetupHardware();

    /*  Initialize O/S Concurrency Mechanisms   */

    /* Create Master Task   */
	#ifndef DEBUG_NO_MASTER
	xTaskCreate(    task_master,
                    "MASTER",
                    2048,
                    NULL,
                    2,
                    NULL );
	#endif


    // Begin O/S Process Scheduler
	vTaskStartScheduler();

    // End Program.
    return 0;
}

/*  Tasks     */
/*******************************************************************/

/* Master Task  */
void task_master(void *p)
{
    /* Variables    */
    int count = 0;
    
	/* Create the queue */

	/* Sanity check that the queue was created. */

	/* Register Interrupt handlers	*/

    // Register WDT isr handler
    #ifndef DEBUG_NO_WDT    

    register_interrupt_handler( XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR,
                                    isr_wdt,
                                    NULL,
                                    "WDT");

    #endif

	/* Enable WDT	*/
    #ifndef DEBUG_NO_WDT

    XWdtTb_Start(&inst_wdt);

    #endif

    #ifdef DEBUG_MASTER

    print("MASTER THREAD:\tEntering While Loop\r\n");

    #endif

    u8 * cp;

   	/*	Quiescent operations	*/
	for( ; ; ){

        memset(buf_uart, 0, sizeof(buf_uart));
        memset(buf_esp, 0, sizeof(buf_esp));
        count = 0;

        cp = buf_uart;

        while( ( ESP32_Recv(&inst_esp, cp, 1) ) != 0){
            if(cp == buf_uart) print("FromESP:");
        	xil_printf("%c", *cp);
            cp++;
        }

        cp = buf_esp;

         while( ( count = XUartLite_Recv(&inst_uart0, cp, 1) ) != 0){
                if(cp == buf_esp) 
                {
                    print("FromTERM:");
                    vTaskDelay(5);
                }
                if(*cp == '\r')
                {
                    cp++;
                    *cp = '\n';
                    cp++;
                    xil_printf("%c", *cp);

                }
                else
                {
                    cp+= count;
                }     
         }

        count = 0;
        while(count != (cp - buf_esp)){
            count += ESP32_Send(&inst_esp, buf_esp, (cp - buf_esp));
        }



		/*	Set Health check	*/
		master_health_check = TRUE;

	}

    #ifdef DEBUG_MASTER

        print("MASTER THREAD:\tExited While Loop!!!\r\n");

    #endif
}


/* Initialization Routines    */    
/*******************************************************************/

/*  Top Level Hardware Routine  */
void prvSetupHardware( void )
{

    #ifdef DEBUG_MAIN

        print("SETUP HARDWARE:\tStarting.\r\n");

    #endif

    #ifndef DEBUG_NO_WDT
        // Initialize WDT.
        init_wdt(); 
        if(xStatus != XST_SUCCESS) print("WDT:\tInitialization Failed.\r\n");

    #endif

    #ifndef DEBUG_NO_UART0
        // UART0 Init.
        init_uart(&inst_uart0, XPAR_AXI_UARTLITE_0_DEVICE_ID);
        if(xStatus != XST_SUCCESS) print("UART0:\tInitialization Failed.\r\n");

    #endif

    #ifndef DEBUG_NO_ESP
        // ESP32 Init.
        xStatus = ESP32_Initialize(&inst_esp, 
            XPAR_PMODESP32_0_AXI_LITE_UART_BASEADDR,   
            XPAR_PMODESP32_0_AXI_LITE_GPIO_BASEADDR);
        if(xStatus != XST_SUCCESS) print("ESP32:\tInitialization Failed.\r\n");

    #endif



    #ifdef DEBUG_MAIN

    print("SETUP HARDWARE:\tDone.\r\n");

    #endif

}

/* Initialize UART */
void init_uart(XUartLite *UartLite, u16 DeviceId)
{

/*
	 * Initialize the UartLite driver so that it's ready to use.
	 */
	xStatus = XUartLite_Initialize(UartLite, DeviceId);
	if (xStatus  != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built correctly.
	 */
	xStatus  = XUartLite_SelfTest(UartLite);
	if (xStatus  != XST_SUCCESS) {
		return XST_FAILURE;
	}
}

/* Initialzie WDT   */
void init_wdt(void){

    /*  Configuration Structure for WDT  */
    XWdtTb_Config * wdtConfig;

	#ifdef DEBUG_WDT_INIT
	print("WDT:\tInitializing.\r\n");
	#endif

	/*
	 * Initialize the WDTTB driver so that it's ready to use look up
	 * configuration in the config table, then initialize it.
	 */
	wdtConfig = XWdtTb_LookupConfig(XPAR_WDTTB_0_DEVICE_ID);
	if (NULL == wdtConfig) {
		return XST_FAILURE;
	}

	/*
	 * Initialize the watchdog timer and timebase driver so that
	 * it is ready to use.
	 */
	xStatus = XWdtTb_CfgInitialize(&inst_wdt, wdtConfig,
			wdtConfig->BaseAddr);
	if (xStatus != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built correctly.
	 */
	xStatus = XWdtTb_SelfTest(&inst_wdt);
	if (xStatus != XST_SUCCESS) {
		print("WDT:\tSelf test failed.\r\n");
		return XST_FAILURE;
	}

	/*
	 * Stop the timer.
	 */
	XWdtTb_Stop(&inst_wdt);

	#ifdef DEBUG_WDT_INIT
	print("WDT:\tStopped (Hardware Config).\r\n");
	#endif

}

/* ISR Registration Routine    */
/*******************************************************************/

/*  Register Interrupt Routine  */
void register_interrupt_handler(uint8_t ucInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef, char name[]){

	#ifdef DEBUG_ISR_REGISTRATION
		xil_printf("ISR REGISTRATION:\t%s ISR Registration Start.\r\n", name);
        #endif

	/* Install the handler defined in this task for the button input.
	*NOTE* The FreeRTOS defined xPortInstallInterruptHandler() API function
	must be used for this purpose. */

	xStatus = xPortInstallInterruptHandler( ucInterruptID, pxHandler, pvCallBackRef );


	if( xStatus == pdPASS )
	{

		/* Enable the button input interrupts in the interrupt controller.
		*NOTE* The vPortEnableInterrupt() API function must be used for this
		purpose. */

		vPortEnableInterrupt( ucInterruptID );

		#ifdef DEBUG_ISR_REGISTRATION
            xil_printf("ISR REGISTRATION:\t%s ISR Registration done.\r\n", name);
            #endif
	}

	configASSERT( ( xStatus == pdPASS ) );

	#ifdef DEBUG_ISR_REGISTRATION
		xil_printf("ISR REGISTRATION:\t%s Assertion Passed.\r\n", name);
        #endif
}
