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
#include "HB3.h"

/*  O/S Concurrency Mechanisms    */
/*******************************************************************/

/* Queues   */

/* Semaphores   */
xSemaphoreHandle ssl_binarysemaphore;
xSemaphoreHandle esp_binary_semaphore;

/* Mutexes  */

/*  Global Variables    */
/*******************************************************************/

/*  Health Checks   */
static bool  master_health_check = TRUE; // Initial health of Master is TRUE.
static bool connected = TRUE;

/* Conditional Fl   ags    */
static bool error_message = FALSE;
static bool data_collect = FALSE;
static bool register_values = FALSE;
static bool write_pid   = TRUE;

/* UART Buffers  */
static u8 rx_buf[BUF_LEN] = {0};   // Initialize buffer
static u8 tx_buf[BUF_LEN] = {0};   // Initialize buffer
static u8 stdin_buf[16] = {0};
static u8 circ_buf[CIRC_BUF_LEN] = {0};
static u8 value_buf[BUF_LEN] = {0};
static u8 control[16] = {0};
/* ESP32 Commands   */
static u8 reset[32] = "AT+RST" CRLF;
static u8 close[32] = "AT+CIPCLOSE" CRLF;
static u8 ate0[16] = "ATE0" CRLF;
static u8 ate1[16] = "ATE1" CRLF;
static u8 cwmode[32] = "AT+CWMODE=1" CRLF;
static u8 connectwifi[64] = WIFI_LOGIN_INFO;
static u8 connect_status[32] = "AT+CWJAP?\r\n";
static u8 num_of_connects[32] = "AT+CIPMUX=0" CRLF;
// static u8 ssl_connect[128] = "AT+CIPSTART=\"SSL\",\"final-524b8.firebaseio.com\",443,7200" CRLF;
static u8 ssl_connect[BUF_LEN] = "AT+CIPSTART=\"SSL\",\"iotirrigationv1r1.firebaseio.com\",443,3600" CRLF;
static u8 ssl_status[32] = "AT+CIPSTATUS" CRLF;
static u8 presend[32] = "AT+CIPSENDEX=512" CRLF;

// static u8 ssl_get1[128] =       "GET /.json HTTP/1.1" CRLF \
//                                     "Host: final-524b8.firebaseio.com" CRLF \
//                                     "Connection: keep-alive" CRLF \
//                                     CRLF NULLCH;

static u8 ssl_get1[128] =       "GET /final_prj_544.json HTTP/1.1" CRLF \
                                "Host: iotirrigationv1r1.firebaseio.com" CRLF \
                                CRLF NULLCH;                                

// static u8 ssl_patch1[200] =     "PATCH /.json HTTP/1.1" CRLF \
//                                 "Host: final-524b8.firebaseio.com" CRLF \
//                                 "Content-Type: application/application/x-www-form-urlencoded" CRLF \
//                                 "Content-Length: 15"  CRLF \
//                                 CRLF\
//                                 "{\"status0\":2}"
//                                 CRLF NULLCH;

static u8 patch1[200] =     "PATCH /final_prj_544.json HTTP/1.1" CRLF \
                                "Host: iotirrigationv1r1.firebaseio.com" CRLF \
                                "Content-Type: application/x-www-form-urlencoded" CRLF \
                                "Content-Length: 27"  CRLF \
                                CRLF\
                                "{\"global_motor_status\":1}"
                                CRLF NULLCH;

static u8 patch0[200] =     "PATCH /final_prj_544.json HTTP/1.1" CRLF \
                                "Host: iotirrigationv1r1.firebaseio.com" CRLF \
                                "Content-Type: application/x-www-form-urlencoded" CRLF \
                                "Content-Length: 27"  CRLF \
                                CRLF\
                                "{\"global_motor_status\":0}"
                                CRLF NULLCH;

static u8 patch3[200] =     "PATCH /final_prj_544.json HTTP/1.1" CRLF \
                                "Host: iotirrigationv1r1.firebaseio.com" CRLF \
                                "Content-Type: application/x-www-form-urlencoded" CRLF \
                                "Content-Length: 29"  CRLF \
                                CRLF\
                                "{\"global_motor_status\":";

 static u8 tail[16] =       "}" CRLF NULLCH;                               

/*  Message Length  */
static u32 length = 0;   // Number of bytes to read/write.

/* Return Checks    */
static BaseType_t xStatus;

/* ISR  */
static u32 strsize = 0;
static u8 *circ_buf_p = NULL;
static u8 *value_buf_p = NULL;
static u8 *val_p = value_buf;
static int backoff = 1;
static int adjustment = 0;

/*  HB3 */
static u8 status = 0;
static int control_dc = 0;
static u16 setpoint_dc = 0;
static u16 setpoint_lpm = 0;
static u8  position_lpm = 0;
static u8  position_dc = 0;
static int error_dc = 0;
static u8 lpm = 0;
static u8 direction = 0;
static u32 last_values = 0;

/* PID  */
static bool updatePID = FALSE;
static int pTerm = 0;
static int dTerm = 0;
static int iTerm = 0;


/* Dispense */
u32 dispense_target = 0;
u32 amount_dispensed = 0;

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
//         xil_printf("%s", rx_buf);
        circ_buf[CIRC_BUF_LEN - 1] = rx_buf[0];      

         circ_buf_p = circ_buf + (CIRC_BUF_LEN - 1);
         
         if(*circ_buf_p == '>')
         {
             xSemaphoreGiveFromISR(esp_binary_semaphore, pdFALSE);

         }


        strsize = strlen("OK\r");

        circ_buf_p = circ_buf + (CIRC_BUF_LEN - strsize);

        if(strncmp(circ_buf_p, "OK\r", strsize) == 0)    // If circ_buf is equal to trigger, give up semaphore.
        {
            error_message = FALSE;
            xSemaphoreGiveFromISR(esp_binary_semaphore, pdFALSE);
        }

        strsize = strlen("CLOSED");

        circ_buf_p = circ_buf + (CIRC_BUF_LEN - strsize);

        if(strncmp(circ_buf_p, "CLOSED", strsize) == 0)
        {
            connected = FALSE;
            backoff = 1;
        }

        strsize = strlen("ERR");

        circ_buf_p = circ_buf + (CIRC_BUF_LEN - strsize);

        if(strncmp(circ_buf_p, "ERR", strsize) == 0)
        {
            backoff = backoff * 10;
            print("ERROR");
        }

        strsize = strlen("}");

        circ_buf_p = circ_buf + (CIRC_BUF_LEN - strsize);

        if(strncmp(circ_buf_p, "}", strsize) == 0)
        {
            data_collect = FALSE;
            *value_buf_p = ',';
            register_values = TRUE;
            xSemaphoreGiveFromISR(ssl_binarysemaphore, pdFALSE);

        }

        if(data_collect)
        {
            *value_buf_p = rx_buf[0];
            value_buf_p += 1;
        }
        else
        {
            value_buf_p =value_buf;
        }

        strsize = strlen("{");

        circ_buf_p = circ_buf + (CIRC_BUF_LEN - strsize);

        if(strncmp(circ_buf_p, "{", strsize) == 0)
        {
            data_collect = TRUE;
        }

        for(int i = 0; i < (CIRC_BUF_LEN - 1); i++)     
        {
            circ_buf[i] = circ_buf[i+1];
        }
}

static void hb3_handler(void *pvUnused)
{
    u32 *p = NULL;
    u32 *c = NULL;
    p = HB3_LPM_ADDR;
    c = HB3_COUNTER_COUNT_ADDR;
    *p = (*p)*(100000000/(*c));
    position_lpm = *p & 0x3FF;
    if(control_dc > 0) status = 1;
    else status = 0;
    if(position_lpm == 0) position_dc = 0;
    else position_dc = ((position_lpm*100)+18200)/163;
    error_dc = setpoint_dc - position_dc;
    write_pid = TRUE;
    xil_printf("\r\npos:%u,set:%u,ctrl:%u", position_dc, setpoint_dc, control_dc);
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
    ssl_binarysemaphore = xSemaphoreCreateBinary();
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
    int *pid_param = NULL;

    // Create PID struct
	SPid pid;

    // Initial PID values
    pid.derState = 0;
    pid.intergratState = 0;
    pid.intergratMax = 10;
    pid.intergratMin = -10;
    pid.intergratGain = 0;
    pid.propGain = 60;
    pid.derGain = 13;
	
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

    // Register HB3 Interrupt
    register_interrupt_handler( XPAR_MICROBLAZE_0_AXI_INTC_HB3_0_INTERRUPT_INTR,
                                hb3_handler,
                                NULL,
                                "HB3");

    // Register ESP UART handlers
    register_interrupt_handler(XPAR_MICROBLAZE_0_AXI_INTC_PMODESP32_0_UART_INTERRUPT_INTR,
                                XUartLite_InterruptHandler, &inst_esp.ESP32_Uart, "UARTESP");
    XUartLite_SetRecvHandler(&inst_esp.ESP32_Uart, rx_uart1, NULL);
    XUartLite_SetSendHandler(&inst_esp.ESP32_Uart, tx_uart1, NULL);
    XUartLite_EnableInterrupt(&inst_esp.ESP32_Uart);

    /* Characterize the system	*/
	#ifdef CHARACTERIZE
		for(int i = 50; i < 255; i++)
		{
			int * addrp = NULL;
			addrp = HB3_DIR_ADDR;
			*addrp = 0;
			addrp = HB3_HIGH_PULSE_ADDR;
			setpoint = i;
			*addrp = setpoint;
			addrp = HB3_COUNTER_COUNT_ADDR;
			*addrp = 100000000;
			vTaskDelay(200);
		}
	#endif

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
        int length = 0;
        int temp = 0;
        TickType_t ticks = 0;
        int *addr = NULL;
   
        ticks = xTaskGetTickCount();

        memset(stdin_buf, 0x0, sizeof(stdin_buf));

        if(write_pid)
        {
            temp = UpdatePID(&pid);

            control_dc += temp;
            if(setpoint_dc == 0)
            {
            	control_dc =0;
            }
            else if (control_dc > 255)
            {
                control_dc = 255;
            }
            else if(control_dc < 0)
            {
                control_dc = 0;
            }

            addr = HB3_HIGH_PULSE_ADDR;

            *addr = control_dc & 0xFF;

            addr = HB3_DIR_ADDR;

            *addr = 1;

            addr = HB3_COUNTER_COUNT_ADDR;
			
            *addr = 100000000;

            write_pid = FALSE;
        }


        if(connected == FALSE)
        {
            print("Connection lost. Connecting.");
            vTaskDelay(backoff);
            send_message(ssl_connect, esp_binary_semaphore);
            connected = TRUE;
            vTaskDelay(800);
            print("Reconnected.");
        }

        if(!(ticks % 100))
        {
            ssl_send_message(ssl_get1);
        }
        if(!(ticks % 300))
        {
            if(position_lpm > 0) ssl_send_message(patch1);
            else ssl_send_message(patch0);

            control[2] = (position_lpm % 10) | 0x30;
            control[1] = (((position_lpm % 100) )/10)| 0x30;
            control[0] = (((position_lpm % 1000))/100)| 0x30;
            
            strncat(patch3, control, 3);

            strncat(patch3, tail, strlen(tail));

            ssl_send_message(patch3);
        }

        
        if(register_values)
        {
            val_p = value_buf;
            
            temp = 0;

            length = strlen("status0");

            for(int l = 0; l < strlen(value_buf); l++)
            {
                val_p++;

                if( strncmp(val_p,"status0", length) == 0 )
                {
                    val_p += length + 2; // \":x

                    while((*val_p != ','))
                    {
                        temp = (temp * 10) + (*val_p & 0xF);
                        val_p++;
                    }
                    
                    dispense_target = temp;

                    break;                
                } 
            }

            val_p = value_buf;
            
            temp = 0;

            length = strlen("status1");

            for(int l = 0; l < strlen(value_buf); l++)
            {
                val_p++;

                if( strncmp(val_p,"status1", length) == 0 )
                {
                    val_p += length + 2; // \":x

                    while((*val_p != ','))
                    {
                        temp = (temp * 10) + (*val_p & 0xF);
                        val_p++;
                    }

                    direction = temp;
                    val_p += 1;

                    break;                
                } 
            }

            val_p = value_buf;
            
            temp = 0;

            length = strlen("Set_point");

            for(int l = 0; l < strlen(value_buf); l++)
            {
                val_p++;

                if( strncmp(val_p,"Set_point", length) == 0 )
                {
                    val_p += length + 5; // \":x

                    while((*val_p == '\\') || (*val_p == '"')){
                        val_p++;
                    }
                    
                    while((*val_p != ',') & (*val_p != '\\') & (*val_p != "'"))
                    {
                        temp = (temp * 10) + (*val_p & 0xF);
                        val_p++;
                    }

                    last_values = temp;
                    
                    if(temp > 115) setpoint_lpm = 115;
                    else if (temp == 0) setpoint_dc = 0;
                    else
                    	{
                    	setpoint_lpm = temp;
                        setpoint_dc = ((setpoint_lpm*100)+18200)/163;
                    	}

                    break;                
                } 
            }

            val_p = value_buf;
            
            temp = 0;

            length = strlen("status3");

            for(int l = 0; l < strlen(value_buf); l++)
            {
                val_p++;

                if( strncmp(val_p,"status3", length) == 0 )
                {
                    val_p += length + 2; // \":x

                    while((*val_p != ','))
                    {
                        temp = (temp * 10) + (*val_p & 0xF);
                        val_p++;
                    }

                    val_p = HB3_COUNTER_COUNT_ADDR;
                    *val_p = temp;

                    break;                
                } 
            }


            register_values = FALSE;

        }
        else
        {
            val_p++;
        }

        do
        {
            recv_count = XUartLite_Recv(&inst_uart0, stdin_buf, BUF_LEN);
            vTaskDelay(1);
            if(recv_count > 0){

                xil_printf("\r\n%s", stdin_buf);

                if( strncmp(stdin_buf, "kp", strlen("kp")) == 0 )
                {
                    pid_param = &pid.propGain;
                    print(CRLF"kp selected");
                }
                else if( strncmp(stdin_buf, "ki", strlen("ki")) == 0 )
                {
                    pid_param = &pid.intergratGain;
                    print(CRLF"ki selected");

                }
                else if( strncmp(stdin_buf, "kd", strlen("kd")) == 0 )
                {
                    pid_param = &pid.derGain;
                    print(CRLF"kd selected");

                }
                 else if( strncmp(stdin_buf, "maxki", strlen("maxki")) == 0 )
                {
                    pid_param = &pid.intergratMax;
                    print(CRLF"maxki selected");

                }
                 else if( strncmp(stdin_buf, "minki", strlen("minki")) == 0 )
                {
                    pid_param = &pid.intergratMin;
                    print(CRLF"minki selected");

                }
                else if( ( length = ( strlen(stdin_buf) - 1 ) ) < MAX_GAIN_SIZE)
                {
                
                    temp = 0;


                    for(int i = 0; i < length; i++)
                    {
                        int decade = 1;

                        for(int k = i; k < (length - 1); k++)
                        {
                            decade *= 10;
                        }

                        switch(stdin_buf[i]){
                            case '1':
                                temp += ( 1 * decade);
                                break;
                            case '2':
                                temp += ( 2 * decade);
                                break;
                            case '3':
                                temp += ( 3 * decade);
                                break;
                            case '4':
                                temp += ( 4 * decade);
                                break;
                            case '5':
                                temp += ( 5 * decade);
                                break;
                            case '6':
                                temp += ( 6 * decade);
                                break;
                            case '7':
                                temp += ( 7 * decade);
                                break;
                            case '8':
                                temp += ( 8 * decade);
                                break;
                            case '9':
                                temp += ( 9 * decade);
                                break;

                            default:
                                break;
                        }
                    }

                    if(stdin_buf[0] == '-') temp = -(*pid_param);
                    
                    *pid_param = temp;

                    xil_printf(CRLF"%d", *pid_param);

                    write_pid = TRUE;
                }
            }

        }while( recv_count > 0);

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
    print("Connecting.");
    send_message(close, esp_binary_semaphore);
    print(".");
    send_message(reset, esp_binary_semaphore);
    print(".");

    vTaskDelay(400);

    send_message(ate1, esp_binary_semaphore);
    print(".");

    send_message(cwmode, esp_binary_semaphore);
    print(".");

    send_message(num_of_connects, esp_binary_semaphore);
    print(".");

    send_message(connectwifi, esp_binary_semaphore);
    print(".");

    send_message(connect_status, esp_binary_semaphore);
    print(".");

    send_message(ssl_connect, esp_binary_semaphore);
    print(".");

    send_message(ssl_status, esp_binary_semaphore);
    print(".");

    send_message(ate0, esp_binary_semaphore);
    print("\r\nConnected\r\n");

    connected = TRUE;

 }

void send_message(u8 *message, QueueHandle_t queue)
{

        strncpy(tx_buf, message, sizeof(tx_buf));

        length = strlen(message);

        XUartLite_Send(&inst_esp.ESP32_Uart, message, length);

        xSemaphoreTake(queue, 800); // Block until relased by isr.
    
}


void ssl_send_message(u8 *message)
{

        send_message(presend, esp_binary_semaphore);

        vTaskDelay(1);

        send_message(message, ssl_binarysemaphore);
    
}


void ssl_send_data(u8 status, u8 control, u32 lpm, QueueHandle_t queue)
{
    // MUST BE NON BLOCKING

//    send_nb_message(presend);

//    send_nb_message();

    

}

/* PID   */
/*******************************************************************/

int UpdatePID(SPid * pid)
{

	pTerm = (pid->propGain * error_dc)>>7; // calculate the proportional term

	/* Calculate the integral state with appropriate limiting	*/
	pid->intergratState += error_dc;

//	Limit te integrator state if necessary
	if(pid->intergratState > pid->intergratMax){
		pid->intergratState = pid->intergratMax;
	}
	else if (pid->intergratState < pid->intergratMin){
		pid->intergratState = pid->intergratMin;
	}

	// Calculate the integral term
	iTerm =(pid->intergratGain * pid->intergratState)>>7;

	// Calculate the derivative
	dTerm = (pid->derGain * (pid->derState - position_dc))>>7;
	pid->derState = position_dc;

	// Return control value
	return pTerm + dTerm + iTerm;
}
