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
xSemaphoreHandle binarysemaphore;

/* Mutexes  */

/*  Global Variables    */
/*******************************************************************/

/*  Health Checks   */
static bool  master_health_check = TRUE; // Initial health of Master is TRUE.

/* UART Buffers  */
static u8 rx_buf[BUF_LEN] = {0};   // Initialize buffer
static u8 tx_buf[BUF_LEN] = {0};   // Initialize buffer
static u8 message0[BUF_LEN]= "AT\r\n";
static u8 message1[BUF_LEN] = "AT+CWMODE=1\r\n";
static u8 message2[BUF_LEN] = WIFI_LOGIN_INFO;
static u8 message3[BUF_LEN] = "AT+CWJAP?\r\n";
static u32 length = 0;   // Number of bytes to read/write.

/* Return Checks    */
static BaseType_t xStatus;

/* Read Send Flags  */
static bool rx_valid = FALSE;
static bool tx_empty = FALSE;


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

static void tx_uart1(void *pvUnused)
{
    tx_empty = Xil_In32(ESP32_STATUS_REG & MASK_TX_EMPTY);
    if(tx_empty)
        xSemaphoreGiveFromISR(binarysemaphore, pdFALSE);


}

static void rx_uart1(void *pvUnused)
{
        while(XUartLite_Recv(&inst_esp.ESP32_Uart, rx_buf, 1) != 0);
        xil_printf("%s", rx_buf);

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
    binarysemaphore = xSemaphoreCreateBinary();

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

    // Register WDT isr handler.
    #ifndef DEBUG_NO_WDT    

    register_interrupt_handler( XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR,
                                    isr_wdt,
                                    NULL,
                                    "WDT");

    #endif

    // Register ESP UART handlers
    register_interrupt_handler(XPAR_MICROBLAZE_0_AXI_INTC_PMODESP32_0_UART_INTERRUPT_INTR,
                                XUartLite_InterruptHandler, &inst_esp.ESP32_Uart, "UARTESP");
    XUartLite_SetRecvHandler(&inst_esp.ESP32_Uart, rx_uart1, NULL);
    XUartLite_SetSendHandler(&inst_esp.ESP32_Uart, tx_uart1, NULL);
    XUartLite_EnableInterrupt(&inst_esp.ESP32_Uart);

    /* Initialize ESP32 */
    connect_to_wifi();

    print("Tx FIFO emptied\r\n");

	/* Enable WDT	*/
    #ifndef DEBUG_NO_WDT

    XWdtTb_Start(&inst_wdt);

    #endif

    #ifdef DEBUG_MASTER

        print("MASTER THREAD:\tEntering While Loop\r\n");

    #endif

   	/*	Quiescent operations	*/
	for( ; ; ){

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

/* ESP32    */
/*******************************************************************/

void connect_to_wifi(void)
{
    strncpy(tx_buf, message0, sizeof(tx_buf));

    length = strlen(message0);

    XUartLite_Send(&inst_esp.ESP32_Uart, message0, length);

    xSemaphoreTake(binarysemaphore, portMAX_DELAY); // Block until relased by isr.

    strncpy(tx_buf, message1, sizeof(tx_buf));

    length = strlen(message1);

    XUartLite_Send(&inst_esp.ESP32_Uart, message1, length);

    xSemaphoreTake(binarysemaphore, portMAX_DELAY); // Block until relased by isr.

    strncpy(tx_buf, message2, sizeof(tx_buf));

    length = strlen(message2);

    XUartLite_Send(&inst_esp.ESP32_Uart, message2, length);

    xSemaphoreTake(binarysemaphore, portMAX_DELAY); // Block until relased by isr.

    strncpy(tx_buf, message3, sizeof(tx_buf));

    length = strlen(message3);

    XUartLite_Send(&inst_esp.ESP32_Uart, message3, length);

    xSemaphoreTake(binarysemaphore, portMAX_DELAY); // Block until relased by isr.
}