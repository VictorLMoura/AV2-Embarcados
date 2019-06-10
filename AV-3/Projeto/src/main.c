#include "asf.h"
#include "main.h"
#include <string.h>

#include "OLED1/gfx_mono_ug_2832hsweg04.h"
#include "OLED1/gfx_mono_text.h"
#include "OLED1/sysfont.h"

/************************************************************************/
/* GENERIC DEFINES                                                      */
/************************************************************************/

/************************************************************************/
/* generic globals                                                      */
/************************************************************************/

/************************************************************************/
/*  RTOS    (defines + globals)                                         */
/************************************************************************/

#define TASK_STRING_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_STRING_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_IO_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_IO_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_OLED1_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_OLED1_STACK_PRIORITY        (tskIDLE_PRIORITY)

//botao 1 oled
#define EBUT1_PIO PIOD
#define EBUT1_PIO_ID ID_PIOD
#define EBUT1_PIO_IDX 28
#define EBUT1_PIO_IDX_MASK (1u << EBUT1_PIO_IDX)

//led 1 oled
#define OLED1_PIO PIOA
#define OLED1_PIO_ID ID_PIOA
#define OLED1_PIO_IDX 0
#define OLED1_PIO_IDX_MASK (1u << OLED1_PIO_IDX)

//botao 2 oled
#define EBUT2_PIO PIOC
#define EBUT2_PIO_ID ID_PIOC
#define EBUT2_PIO_IDX 31
#define EBUT2_PIO_IDX_MASK (1u << EBUT2_PIO_IDX)

//led 2 oled
#define OLED2_PIO PIOC
#define OLED2_PIO_ID ID_PIOC
#define OLED2_PIO_IDX 30
#define OLED2_PIO_IDX_MASK (1u << OLED2_PIO_IDX)

//botao 3 oled
#define EBUT3_PIO PIOA
#define EBUT3_PIO_ID ID_PIOA // piod ID
#define EBUT3_PIO_IDX 19
#define EBUT3_PIO_IDX_MASK (1u << EBUT3_PIO_IDX)

//led 3	 oled
#define OLED3_PIO PIOB
#define OLED3_PIO_ID ID_PIOB
#define OLED3_PIO_IDX 2
#define OLED3_PIO_IDX_MASK (1u << OLED3_PIO_IDX)


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}
extern void vApplicationIdleHook(void)
{
	
}
extern void vApplicationTickHook(void)
{
}
extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

QueueHandle_t xQueueUART;
QueueHandle_t xQueueOLED;
SemaphoreHandle_t xSemaphoreLED;

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

int protocol_check_led(char *string);
int io_init(void);
void led_on(uint id, uint on);

/************************************************************************/
/* IRQS / callbacks                                                     */
/************************************************************************/

void USART1_Handler(void){

  BaseType_t xHigherPriorityTaskWoken = pdFALSE; // freeRTOs
  uint32_t ret = usart_get_status(CONF_UART);  // ACK IRQ
  char b;
  uint i = 0;

  // Por qual motivo entrou na interrupçcao ? Pode ser varios!
  //  1. RX: Dado disponível para leitura
  if(ret & US_IER_RXRDY){
		// LER DADO DA UART
		usart_serial_getchar(CONF_UART, &b);
		xQueueSendFromISR( xQueueUART, &b, 0);      
  }
}

void button0_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
}


void button1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
}


void button2_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
}



/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_string(void *pvParameters){
  xQueueUART = xQueueCreate( 10, sizeof( char ) );
  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  char b[52];
  uint i = 0;
  while(1){
	if (xQueueReceiveFromISR( xQueueUART, &(b[i]), ( TickType_t )  500 / portTICK_PERIOD_MS)) {
		if(b[i]=='\n'){
			b[i] = NULL;
			i=0;
			printf("recebido: %s\n", b);
			int equal = protocol_check_led(b);
			if(equal) xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
			if(strstr(b, "OLED:") != NULL) xQueueSendFromISR(xQueueOLED, &b, 0);
		}else{
			i++;
		}  
	}     
  }    
}  


void task_io(void *pvParameters){
   xSemaphoreLED = xSemaphoreCreateBinary();

   io_init();
   int pisca =1;
   
   while(1){
	   
	   if(pisca){
			vTaskDelay(100);
			led_on(1,0);
			led_on(2,0);
			led_on(3,0);
			vTaskDelay(100);
			led_on(1,1);
			led_on(2,1);
			led_on(3,1);
			vTaskDelay(100);
			led_on(1,0);
			led_on(2,0);
			led_on(3,0);
			vTaskDelay(100);
			led_on(1,1);
			led_on(2,1);
			led_on(3,1);
			vTaskDelay(100);
			led_on(1,0);
			led_on(2,0);
			led_on(3,0);
			vTaskDelay(100);
			led_on(1,1);
			led_on(2,1);
			led_on(3,1);
			pisca--;
	   }
	   if(xSemaphoreTake(xSemaphoreLED, ( TickType_t ) 500) == pdTRUE) {
			pisca++;
	   }
   }
}

void task_oled1(void *pvParameters){
    xQueueOLED = xQueueCreate( 2, sizeof( char[52] ) );
	gfx_mono_ssd1306_init();
	char buffer[52];
	while(1){
		//gfx_mono_draw_string("mundo", 50,16, &sysfont);
		if (xQueueReceiveFromISR( xQueueOLED, &buffer, ( TickType_t )  500 / portTICK_PERIOD_MS)) {
			gfx_mono_draw_string(&buffer[5], 25,20, &sysfont);
		}
		vTaskDelay(100);
	}
}  




/************************************************************************/
/* CONFIGS/ INITS                                                       */
/************************************************************************/

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
	// Ativa interrupcao UART1 na recpcao de daods
	usart_enable_interrupt(CONF_UART, US_IER_RXRDY);
	NVIC_EnableIRQ(CONSOLE_UART_ID);
	NVIC_SetPriority(CONSOLE_UART_ID, 5);

}


/************************************************************************/
/* functions                                                            */
/************************************************************************/

int protocol_check_led(char *string){
	if(strcmp(string, "LEDS") == 0){
		return 1;
	}
	return 0;
}

int io_init(void){
		// configura botoes do oled
		pmc_enable_periph_clk(EBUT1_PIO_ID);
		pmc_enable_periph_clk(EBUT2_PIO_ID);
		pmc_enable_periph_clk(EBUT3_PIO_ID);
		pmc_enable_periph_clk(OLED1_PIO_ID);
		pmc_enable_periph_clk(OLED2_PIO_ID);
		pmc_enable_periph_clk(OLED3_PIO_ID);

		
		// configura botoes do oled como input;
		pio_configure(EBUT1_PIO, PIO_INPUT, EBUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		pio_configure(EBUT2_PIO, PIO_INPUT, EBUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		pio_configure(EBUT3_PIO, PIO_INPUT, EBUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		
		pio_configure(OLED1_PIO, PIO_OUTPUT_0, OLED1_PIO_IDX_MASK, PIO_DEFAULT);
		pio_configure(OLED2_PIO, PIO_OUTPUT_0, OLED2_PIO_IDX_MASK, PIO_DEFAULT);
		pio_configure(OLED3_PIO, PIO_OUTPUT_0, OLED3_PIO_IDX_MASK, PIO_DEFAULT);



		
		// Configura interrup??o no pino referente ao botao e associa
		// fun??o de callback caso uma interrup??o for gerada
		// a fun??o de callback ? a: but_callback()
		pio_handler_set(EBUT1_PIO,
		EBUT1_PIO_ID,
		EBUT1_PIO_IDX_MASK,
		PIO_IT_FALL_EDGE,
		button0_callback);
		
		pio_handler_set(EBUT2_PIO,
		EBUT2_PIO_ID,
		EBUT2_PIO_IDX_MASK,
		PIO_IT_FALL_EDGE,
		button1_callback);
		
		pio_handler_set(EBUT3_PIO,
		EBUT3_PIO_ID,
		EBUT3_PIO_IDX_MASK,
		PIO_IT_FALL_EDGE,
		button2_callback);
		
		NVIC_SetPriority(EBUT1_PIO_ID, 6); // Prioridade 4
		NVIC_SetPriority(EBUT3_PIO_ID, 6); // Prioridade 4
		NVIC_SetPriority(EBUT2_PIO_ID, 6); // Prioridade 4

		// Ativa interrup??o
		pio_enable_interrupt(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
		pio_enable_interrupt(EBUT2_PIO, EBUT2_PIO_IDX_MASK);
		pio_enable_interrupt(EBUT3_PIO, EBUT3_PIO_IDX_MASK);


		// Configura NVIC para receber interrupcoes do PIO do botao
		// com prioridade 4 (quanto mais pr?ximo de 0 maior)

		NVIC_EnableIRQ(EBUT1_PIO_ID);
		NVIC_EnableIRQ(EBUT2_PIO_ID);
		NVIC_EnableIRQ(EBUT3_PIO_ID);
}
  
void led_on(uint id, uint on){
	if (id == 1){
		if(on == 1){
			pio_set(OLED1_PIO, OLED1_PIO_IDX_MASK);
		}
		else{
			pio_clear(OLED1_PIO, OLED1_PIO_IDX_MASK);
		}
	}
	else if(id == 2){
		if(on == 1){
			pio_set(OLED2_PIO, OLED2_PIO_IDX_MASK);
		}
		else{
			pio_clear(OLED2_PIO, OLED2_PIO_IDX_MASK);	
		}
	}
	else if(id == 3){
		if(on == 1){
			pio_set(OLED3_PIO, OLED3_PIO_IDX_MASK);
		}
		else{
			pio_clear(OLED3_PIO, OLED3_PIO_IDX_MASK);
		}
	}
}


/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/
int main(void)
{
	/* Initialize the board. */
	sysclk_init();
	board_init();

	/* Initialize the UART console. */
	configure_console();
   
	if (xTaskCreate(task_string, "string", TASK_STRING_STACK_SIZE, NULL, TASK_STRING_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Wifi task\r\n");
	}
	if (xTaskCreate(task_io, "io", TASK_IO_STACK_SIZE, NULL, TASK_IO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Wifi task\r\n");
	}
	if (xTaskCreate(task_oled1, "oled1", TASK_OLED1_STACK_SIZE, NULL, TASK_OLED1_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Wifi task\r\n");
	}

	vTaskStartScheduler();
	
	while(1) {};
	return 0;

}
