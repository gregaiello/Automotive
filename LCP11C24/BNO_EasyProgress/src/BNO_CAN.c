// Includes
#include "chip.h"
#include <cr_section_macros.h>

// Defines
#define port_LED 0
#define pin_LED_RED 7
#define pin_LED_GREEN 8
#define pin_LED_BLUE 9
#define NUMBYTES 8
#define BAUDRATE 115200
#define BAUDRATE_CAN 1000000

// Defines I2C
#define SPEED_100KHZ         (uint32_t) 100000
#define SPEED_400KHZ         (uint32_t) 400000
#define SLAVE_ADDRESS 		 (uint8_t) 0xC
#define I2C_DATA_LENGTH		 (int) 7
#define I2C_TRIAXIS_READ     (uint8_t) 0x4E
#define I2C_TRIAXIS_RESET    (uint8_t) 0xF0
#define I2C_TRIAXIS_POLLING  (uint8_t) 0x3E
#define I2C_TRIAXIS_SB_MODE  (uint8_t) 0xE
#define BAUDRATE_UART 		 115200
#define UART_DATA_LENGTH     (int) I2C_DATA_LENGTH-1+5
#define pin_LED_RED 		 7
#define pin_LED_GREEN 		 8
#define pin_LED_BLUE 		 9
#define OK_TRIAXIS 		 	 1 << 4

// Functions
void LED_Init(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin);
void LED_setvalue(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin, bool value);
void I2C_Init(void);
void CAN_Init();
void CAN_rx(uint8_t msg_obj_num);
void CAN_tx(uint8_t msg_obj_num);
void CAN_error(uint32_t error_info);

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

    CAN_Init();


    while(1){
    	LED_setvalue(LPC_GPIO, port_LED, pin_LED_RED, false);
    	LED_setvalue(LPC_GPIO, port_LED, pin_LED_GREEN, false);
    	LED_setvalue(LPC_GPIO, port_LED, pin_LED_BLUE, false);
    }
    return 0 ;
}

void I2C_Init(void)
{
	Chip_SYSCTL_PeriphReset(RESET_I2C0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_4, IOCON_FUNC1 | SPEED_400KHZ); // Default speed = 100kHz
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_5, IOCON_FUNC1 | SPEED_400KHZ);
	Chip_I2C_Init(I2C0);
	Chip_I2C_SetClockRate(I2C0, SPEED_400KHZ);
	NVIC_DisableIRQ(I2C0_IRQn);
	Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandlerPolling);
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
	if(	msg_obj.data[0] == 'A')
	{
    	LED_setvalue(LPC_GPIO, port_LED, pin_LED_RED, false);
    	Chip_UART_SendBlocking(LPC_USART, &(msg_obj.data), (int) 6);
	}
}
void CAN_tx(uint8_t msg_obj_num)
{
	LED_setvalue(LPC_GPIO, port_LED, pin_LED_GREEN, false);
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);
}
void CAN_error(uint32_t error_info)
{
	//TODO
}

void LED_Init(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t pin)
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
	for(int i=0;i<delay*100;i++){__NOP();}
}
