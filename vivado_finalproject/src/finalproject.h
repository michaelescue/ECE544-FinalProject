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

/*******************************************************************/
/* Constants    */
/*******************************************************************/

/* Send Buffer Length   */
#define BUF_LEN 128

/*  Instances   */
#define ESP32_DEVICE_ID

/*******************************************************************/
/* Macros    */
/*******************************************************************/

/* Send Buffer Macro. A: ESP32 Instance B: Target Send Buffer, C: Command String, D: (length) Byte Count Variable   */
#define SEND_MESSAGE(A, B, C, D) ({   for(int x = 0; x < D; x++) B[x] = C[x]; \
                                    strncat(B, "\r\n", strlen("\r\n")); \
                                    ESP32_SendBuffer(A, B, D); \
                                    })

/*******************************************************************/
/* Debug Options    */
/*******************************************************************/

/* Top Level  */
#define DEBUG_MAIN  // Debug top level.

/* Master Task  */
// #define DEBUG_NO_MASTER // Remove Master Task related code.
#define DEBUG_MASTER // Debug Master Task.
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
void init_intc(void);

/* ISR Routines    */
static void isr_wdt(void *pvUnused); // WDT Interrupt Handler 
static void uart1_rx_handler(void *CallBackRef, unsigned int ByteCount);

/* Tasks    */
void task_master(void *p); // Master Task
void rx_task(void *p); // UART1 rx_task/

/* Other Functions  */
void esp32recv(void);
void esp32send(const char *text);




#endif