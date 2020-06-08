/**
 * @file myapp.c
 * @author Michael Escue (michael.escue@outlook.com)
 * @brief Written for ECE 544 Final project. Spring 2020.
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
xSemaphoreHandle esp_binary_semaphore;

/* Mutexes  */

/*  Global Variables    */
/*******************************************************************/

/*  Health Checks   */
static bool  master_health_check = TRUE; // Initial health of Master is TRUE.
static bool connected = FALSE;

/* Conditional Flags    */
bool error_message = FALSE;

/* UART Buffers  */
static u8 rx_buf[BUF_LEN] = {0};   // Initialize buffer
static u8 tx_buf[BUF_LEN] = {0};   // Initialize buffer
static u8 stdin_buf[BUF_LEN] = {0};
static u8 circ_buf[CIRC_BUF_LEN] = {0};
static u8 ate[BUF_LEN] = "ATE1" CRLF;
static u8 cwmode[BUF_LEN] = "AT+CWMODE=1" CRLF;
static u8 connectwifi[BUF_LEN] = WIFI_LOGIN_INFO;
static u8 connect_status[BUF_LEN] = "AT+CWJAP?\r\n";
static u8 num_of_connects[BUF_LEN] = "AT+CIPMUX=0" CRLF;
// static u8 ssl_connect[BUF_LEN] = "AT+CIPSTART=\"SSL\",\"iotirrigationv1r1.firebaseio.com\",443,3600" CRLF;
static u8 ssl_connect[BUF_LEN] = "AT+CIPSTART=\"SSL\",\"final-524b8.firebaseio.com\",443,5" CRLF;
static u8 ssl_status[BUF_LEN] = "AT+CIPSTATUS" CRLF;
static u8 presend[BUF_LEN] = "AT+CIPSENDEX=256" CRLF;
// static u8 message8[BUF_LEN] = "GET /status0.json HTTP/1.1\r\nHost: final-524b8.firebaseio.com\r\n\r\n\\0";

static u8 get1[BUF_LEN] =       "GET /.json HTTP/1.1" CRLF \
                                "Host: final-524b8.firebaseio.com" CRLF \
                                CRLF NULLCH;

static u8 options1[BUF_LEN] =   "OPTIONS /status0.json HTTP/1.1" CRLF\
                                "Host: final-524b8.firebaseio.com" CRLF\
                                CRLF NULLCH;

static u8 patch1[BUF_LEN] =     "PATCH /.json HTTP/1.1" CRLF \
                                "Host: final-524b8.firebaseio.com" CRLF \
                                "Content-Type: application/x-www-form-urlencoded" CRLF \
                                "Content-Length: 15"  CRLF \
                                CRLF\
                                "{\"status0\":4}"
                                CRLF NULLCH;

// static u8 get2[BUF_LEN] = "GET /OfficeBusProject/hb/Last Updated Stop.json HTTP/1.1\r\nHost: iotirrigationv1r1.firebaseio.com\r\nAccept: text/plain\r\n\r\n\\0";
// static u8 get1[BUF_LEN] = "GET /OfficeBusProject/hb/Comments.json HTTP/1.1\r\nHost: iotirrigationv1r1.firebaseio.com\r\nAccept: text/plain\r\n\r\n\\0";

/*  Message Length  */
static u32 length = 0;   // Number of bytes to read/write.

/* Return Checks    */
static BaseType_t xStatus;

/* ISR  */
static u32 strsize = 0;
static u8 *circ_buf_p = NULL;

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

/* HB3  */


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
			print("WDT:\tMaster Thread Crashed." CRLF);
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

}

static void rx_uart1(void *pvUnused)
{

        while(XUartLite_Recv(&inst_esp.ESP32_Uart, rx_buf, 1) != 0);
        xil_printf("%s", rx_buf);
        circ_buf[CIRC_BUF_LEN - 1] = rx_buf[0];

        strsize = strlen("OK\r");

        circ_buf_p = circ_buf + (CIRC_BUF_LEN - strsize);

        if(strncmp(circ_buf_p, "OK\r", strsize) == 0)    // If circ_buf is equal to trigger, give up semaphore.
        {
            error_message = FALSE;
            xSemaphoreGiveFromISR(esp_binary_semaphore, pdFALSE);
        }

        strsize = strlen("CLOSED");

        circ_buf_p = circ_buf + (CIRC_BUF_LEN - strsize);

        if(strncmp(circ_buf, "CLOSED", strsize) == 0)
        {
            connected = FALSE;
        }

        for(int i = 0; i < (CIRC_BUF_LEN - 1); i++)     
        {
            circ_buf[i] = circ_buf[i+1];
        }
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
    esp_binary_semaphore = xSemaphoreCreateBinary();

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

    /* Connect to Server    */
    connect_to_wifi();

	/* Enable WDT	*/
    #ifndef DEBUG_NO_WDT

    XWdtTb_Start(&inst_wdt);

    #endif

    #ifdef DEBUG_MASTER

        print("MASTER THREAD:\tEntering While Loop\r\n");

    #endif

   	/*	Quiescent operations	*/
	for( ; ; ){

        int recv_count = 0;

        int* p = HB3_HIGH_PULSE_ADDR;

        do
        {
            vTaskDelay(1);
            recv_count = XUartLite_Recv(&inst_uart0, stdin_buf, BUF_LEN);

            if(recv_count > 0){
                xil_printf("\r\n%s", stdin_buf);
            }

        }while( recv_count > 0);

        if(strncmp(stdin_buf, "ON", strlen("ON")) == 0)
        {
			 *p = 0xFFFFFFFF;
        }
        else if(strncmp(stdin_buf, "OFF", strlen("OFF")) == 0)
        {
			 *p = 0;
        }
       else if(strncmp(stdin_buf, "DIR0", strlen("DIR0")) == 0)
        {
             p = HB3_DIR_ADDR;
			 *p = 0;
        }
       else if(strncmp(stdin_buf, "DIR1", strlen("DIR1")) == 0)
        {
            p = HB3_DIR_ADDR;
			 *p = 0x1;
        }
       else if(strncmp(stdin_buf, "READ", strlen("READ")) == 0)
        {
            p = HB3_LPM_ADDR;
			xil_printf("FROM REG: %d\r\n", *p);
        }

        memset(stdin_buf, 0x0, sizeof(stdin_buf));

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
    send_message(ate);

    send_message(cwmode);

    send_message(num_of_connects);

    send_message(connectwifi);

    send_message(connect_status);

    send_message(ssl_connect);

    send_message(ssl_status);

    ssl_send_message(get1);

    ssl_send_message(options1);

    ssl_send_message(patch1);

    connected = TRUE;

 }

void send_message(u8 *message)
{

        strncpy(tx_buf, message, sizeof(tx_buf));

        length = strlen(message);

        XUartLite_Send(&inst_esp.ESP32_Uart, message, length);

        xSemaphoreTake(esp_binary_semaphore, 600); // Block until relased by isr.
    
}

void ssl_send_message(u8 *message)
{
        if(connected == FALSE)
        {
            send_message(ssl_connect);
            connected = TRUE;
        }

        send_message(presend);

        send_message(message);
    
}
