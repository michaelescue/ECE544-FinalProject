/**
 * @file finalproject.h
 * @author Michael Escue (michael.escue@outlook.com)
 * @brief Include file which contains all functin prototypes for source file.
 * @version 0.1
 * @date 2020-05-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef FINALPROJECT_H
#define FINALPROJECT_H

#include "HB3.h"

/*******************************************************************/
/* Constants    */
/*******************************************************************/

/* Send Buffer Length   */
#define BUF_LEN 512

/* Circ Buffer Length   */
#define CIRC_BUF_LEN 8

/* Esp32 */
#define ESP32_STATUS_REG  XPAR_PMODESP32_0_AXI_LITE_UART_BASEADDR & 0x8
#define UART0_STATUS_REG   XPAR_UARTLITE_0_BASEADDR & 0x8
#define MASK_RX_VALID_DATA 0x1
#define MASK_TX_FULL 0x8
#define MASK_TX_EMPTY 0x4
// #define WIFI_LOGIN_INFO "AT+CWJAP=\"Me\",\"1Chester\"\r\n"
#define WIFI_LOGIN_INFO "AT+CWJAP=\"freecandy\",\"!1Chester\"\r\n"
#define CRLF "\r\n"
#define NULLCH "\\0"
#define RAMSDATA "iotirrigationv1r1.firebaseio.com"
#define MYDATA  "final-524b8.firebaseio.com"

/* HB3  */
#define HB3_HIGH_PULSE_ADDR     XPAR_HB3_0_S00_AXI_BASEADDR | HB3_HIGH_PULSE_REG0_OFFSET
#define HB3_LPM_ADDR            XPAR_HB3_0_S00_AXI_BASEADDR | HB3_LPM_REG1_OFFSET
#define HB3_DIR_ADDR            XPAR_HB3_0_S00_AXI_BASEADDR | HB3_DIR_OUT_REG2_OFFSET

/* PID  */
#define MAX_GAIN_SIZE 4

/*******************************************************************/
/* Data Structures    */
/*******************************************************************/

typedef struct pid{
	int derState;	// Last position input
	int intergratState; // Integrator state
	int intergratMax,		// Maximum and Minimum
        intergratMin;	// allowable state
	int intergratGain,	// Integral gain
        propGain,		// Proportional gain
        derGain;		// Derivative gain
}SPid;

/*******************************************************************/
/* Debug Options    */
/*******************************************************************/

/* Top Level  */
#define DEBUG_MAIN  // Debug top level.

/* Master Task  */
// #define DEBUG_NO_MASTER // Remove Master Task related code.
// #define DEBUG_MASTER // Debug Master Task.
#define DEBUG_ISR_REGISTRATION // Debug ISR Registration routine.

/*  WDT */
#define DEBUG_NO_WDT // Remove WDT related code.
#define DEBUG_WDT_ISR // Debug WDT ISR messages.
// #define DEBUG_WDT_TICK // Enable WDT tick in terminal.
#define DEBUG_WDT_INIT // Debug WDT initialization.

/* ESP32  */
// #define DEBUG_NO_ESP32_UART
// #define DEBUG_NO_ESP32
#define DEBUG_ESP32_UART_ISR
// #define DEBUG_NO_ESP32_UART_ISR
#define DEBUG_ESP32_STDIN   // Used to poll GPIO 0 for commands from stdin and pass them to esp32.



/*******************************************************************/
/* Prototypes    */
/*******************************************************************/
 
/* ISR Registration Routine    */
void register_interrupt_handler(uint8_t ucInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef, char name[]);
int SetupInterruptSystem(XUartLite *UartLitePtr, u32 InterruptID);

/*  Initializations  */
void prvSetupHardware( void );  // Top Level
void init_wdt(void); // WDT initialization 
void init_uart(XUartLite *UartLite, u16 DeviceId);

/* ISR Routines    */
static void isr_wdt(void *pvUnused); // WDT Interrupt Handler 
static void rx_uart1(void *pvUnused);
static void tx_uart1(void *pvUnused);
static void hb3_handler(void *pvUnused);

/* Tasks    */
void task_master(void *p); // Master Task
void rx_task(void *p); // UART1 rx_task/

/* ESP32    */
void send_message(u8 *message, QueueHandle_t queue);
void ssl_send_message(u8 *message);
void send_nb_message(u8 *message, QueueHandle_t queue);
void ssl_send_data(u8 status, u8 control, u32 lpm, QueueHandle_t queue);
void connect_to_wifi(void);


/* PID  */
int UpdatePID(SPid * pid);

#endif
