// Includes
#include "chip.h"
#include <cr_section_macros.h>

// Defines
#define port_LED 0
#define pin_LED_RED 7
#define pin_LED_GREEN 8
#define pin_LED_BLUE 9
#define DELAY 100
#define NUMBYTES 8
#define BAUDRATE 115200
#define BAUDRATE_CAN 500000

// Functions
void LED_Init(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin);
void LED_setvalue(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin, bool value);
void UART_Init(LPC_USART_T *pUART, uint32_t baudrate);
void CAN_Init();
void CAN_rx(uint8_t msg_obj_num);
void CAN_tx(uint8_t msg_obj_num);
void CAN_error(uint32_t error_info);
void delay(long delay);

void CAN_IRQHandler(void);

// Variables
uint8_t data = 0;
uint32_t CanApiClkInitTable[2]; // CanApiClkInitTable[0]: CANCLKDIV, this register determines the CAN clock signal, CanApiClkInitTable[1]: CANBT register (page 291 datasheet)

CCAN_MSG_OBJ_T msg_obj; // Data struct of the packet architecture

CCAN_CALLBACKS_T callbacks = {
	CAN_rx,
	CAN_tx,
	CAN_error,
	NULL, //from here the spots are reserved for CANOpen, we do not need them now
	NULL,
	NULL,
	NULL,
	NULL,
};


int main(void) {

    SystemCoreClockUpdate();

    LED_Init(LPC_GPIO, port_LED, pin_LED_RED);
    LED_Init(LPC_GPIO, port_LED, pin_LED_GREEN);
    LED_Init(LPC_GPIO, port_LED, pin_LED_BLUE);

    UART_Init(LPC_USART, BAUDRATE);

    CAN_Init();

    msg_obj.msgobj = 1;
    msg_obj.mode_id = 0x450;
    msg_obj.mask = 0x450;
    LPC_CCAN_API->config_rxmsgobj(&msg_obj);
	LED_setvalue(LPC_GPIO, port_LED, pin_LED_GREEN, false);

    while(1) {
    	delay(DELAY);
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

void CAN_Init()
{
	uint32_t pCLK, div, quanta, segs, seg1, seg2, clk_per_bit, can_sjw;
	LPC_SYSCTL->SYSAHBCLKCTRL |= (1 << 17); // Enable clock to CAN peripheral
	pCLK = Chip_Clock_GetMainClockRate(); // Get clock rate
	clk_per_bit = pCLK / BAUDRATE_CAN; // Calculate the number of ticks for every bit (every bit composed by multiple quanta)

	// Algorithm to get the (Re)synchronization jump width (online I found that we can set it to 3 and we would be fine)
	for (div = 0; div <= 15; div++) // 4 bit -> 0..15 (where 0 is division by 1, ... , and 15 is division by 16)
	{
		for (quanta = 1; quanta <= 32; quanta++) // Idk why here it stops at 32 instead of 63
		{
			for (segs = 3; segs <= 17; segs++)
			{
				if (clk_per_bit == (segs * quanta * (div + 1)))
				{
					segs -= 3;
					seg1 = segs / 2;
					seg2 = segs - seg1;
					can_sjw = seg1 > 3 ? 3 : seg1;
					CanApiClkInitTable[0] = div; // CANCLKDIV register
					CanApiClkInitTable[1] =	((quanta - 1) & 0x3F) | (can_sjw & 0x03) << 6 | (seg1 & 0x0F) << 8 | (seg2 & 0x07) << 12; // CANBT register
					// CANBT register:
					// 5:0   BRP - Baud rate prescaler, the value by which the oscillator frequency is divided for
					//					generating the bit time quanta. The bit time is built up from
					//					a multiple of this quanta.
					//					Values from 0 to 63 (Hardware interprets the value programmed into these bits as the bit value +1)
					// 7:6   SJW - (Re)synchronization jump width. Valid programmed values are 0 to 3
					// 11:8  TSEG1 - Time segment before the sample point including	propagation segment.
					// 14:12 TSEG2 - Time segment after the sample point
					LPC_CCAN_API->init_can(&CanApiClkInitTable[0], TRUE);
					LPC_CCAN_API->config_calb(&callbacks);
					NVIC_EnableIRQ(CAN_IRQn);
					return;
				}
			}
		}
	}
}

void CAN_IRQHandler(void) {
	LPC_CCAN_API->isr();
}

void CAN_rx(uint8_t msg_obj_num)
{
	msg_obj.msgobj = msg_obj_num;
	LPC_CCAN_API->can_receive(&msg_obj);
	LED_setvalue(LPC_GPIO, port_LED, pin_LED_BLUE, false);
	if(	msg_obj.data[0] == 'G')
	{
    	LED_setvalue(LPC_GPIO, port_LED, pin_LED_RED, false);
    	for(int i=0;i<100;i++){}
    	if(Chip_UART_SendBlocking(LPC_USART, &(msg_obj.data), NUMBYTES) == NUMBYTES)
    	{
    		msg_obj.dlc = 6;
    		msg_obj.data[0] = 'A';
    		msg_obj.data[1] = 'I';
    		msg_obj.data[2] = 'E';
    		msg_obj.data[3] = 'L';
    		msg_obj.data[4] = 'L';
    		msg_obj.data[5] = 'O';
    		LPC_CCAN_API->can_transmit(&msg_obj);
    	}
	}
}

void CAN_tx(uint8_t msg_obj_num)
{
	//TODO
}
void CAN_error(uint32_t error_info)
{
	//TODO
}

void LED_Init(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin)
{
	Chip_GPIO_Init(LPC_GPIO);
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
	for(int i=0;i<delay*100;i++);
}
