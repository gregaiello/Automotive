// Includes
#include "chip.h"
#include <cr_section_macros.h>

// Defines
#define port_LED 0
#define pin_LED_RED 7
#define pin_LED_GREEN 8
#define pin_LED_BLUE 9
#define DELAY 500
#define NUMBYTES 1
#define BAUDRATE 115200

// Functions
void LED_init(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin);
void LED_setvalue(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin, bool value);
void UART_Init(LPC_USART_T *pUART, uint32_t baudrate);
void delay(long delay);

// Variables
uint8_t data = 0;

int main(void) {

    SystemCoreClockUpdate();

    LED_init(LPC_GPIO, port_LED, pin_LED_RED);
    LED_init(LPC_GPIO, port_LED, pin_LED_GREEN);

    UART_Init(LPC_USART, BAUDRATE);

    bool temp = false;
    while(1) {

    	LED_setvalue(LPC_GPIO, port_LED, pin_LED_RED, temp);
    	LED_setvalue(LPC_GPIO, port_LED, pin_LED_GREEN, !temp);
    	Chip_UART_Send(LPC_USART, &data, NUMBYTES);

    	delay(DELAY);

    	data++;
    	temp = !temp;

    	if(data>100)
    	{
    		data = 0;
    	}
    }
    return 0 ;
}

void UART_Init(LPC_USART_T *pUART, uint32_t baudrate)
{

	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT)); // Changing the pin function: 1 per UART RX, 0 per GPIO
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT)); // Changing the pin function: 0 per UART TX, 0 per GPIO

	Chip_UART_Init(pUART); // UART clock and generic initialization

	Chip_UART_SetBaud(pUART, baudrate); // Baud rate

	Chip_UART_ConfigData(pUART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT)); // Configuration length of the data

	Chip_UART_SetupFIFOS(pUART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2)); // ENABLE + how many receiver UART FIFO characters must be written before an interrupt is activated (NOT NEEDED HERE).

	Chip_UART_TXEnable(pUART); // Transmit Enable Register (automatically set to 1 after the reset but better to double check)
}

void LED_init(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin)
{
	// LED initialization: i) clock to the GPIO (all of them), ii) data direction register as output for the pin in the selected port
	LPC_SYSCTL->SYSAHBCLKCTRL |= (1 << SYSCTL_CLOCK_GPIO);
	pGPIO[port].DIR |= (1UL << pin);
}

void LED_setvalue(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin, bool value)
{
	// LED setting: putting a 1 in the bit for the selected pin in the data register
	pGPIO[port].DATA[1 << pin] = value << pin;
}

void delay(long delay)
{
	for(int i=0;i<delay*1000;i++);
}

