/**
  ******************************************************************************
  * @file    peripherals_STM32F405.c
  * @author  Aiello Gregorio
  * @version V0.0.1
  * @date    March 2018
  * @brief   BLDC motor control with no libraries, bare metal test
  *  
  ******************************************************************************
  */ 

// TODO: move to half word (16bits)


#include "peripherals_STM32F405.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// User Parameters
extern volatile uint32_t current[NUM_SAMPLES_ADC_CURRENT_DMA];
extern volatile uint32_t hall[NUM_SAMPLES_ADC_HALL_DMA];
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

void initNVIC(){
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	uint32_t prioritygroup = 0x00U;
	prioritygroup = NVIC_GetPriorityGrouping();

	NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(prioritygroup, 0, 0));
	NVIC_EnableIRQ(DMA2_Stream0_IRQn); // enable stream 0 IRQ in NVIC

	NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(prioritygroup, 0, 0));
	NVIC_EnableIRQ(DMA2_Stream2_IRQn); // enable stream 2 IRQ in NVIC

	NVIC_SetPriority(TIM1_CC_IRQn, NVIC_EncodePriority(prioritygroup, 0, 0));
	NVIC_EnableIRQ(TIM1_CC_IRQn); // enable TIM1 IRQ in NVIC

	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioritygroup, 0, 0));
	NVIC_EnableIRQ(SysTick_IRQn); // enable systick IRQ in NVIC
}

void initLED(){
	/**
	LED GPIO Configuration:    
	PC3     ------> LED BLUE
    PC4     ------> LED RED
    PC5     ------> LED GREEN **/

	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIOCEN; // clock to AHB1-GPIOC enabled
	uint32_t tmp;

	/* Set pin as output */
	tmp = GPIOC->MODER; // support variable to set the LED pin as output
	tmp &= (~LEDS_REG_MASK); // reset
	tmp |= LEDS_REG; // set the correct pattern
	GPIOC->MODER = (uint32_t)tmp; // write the MODER register
} 

void initButton(){
	/**
	User Button GPIO Configuration:    
    PD2     ------> User Button (HW Pull down) **/

	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIODEN; // clock to AHB1-GPIOD enabled
	uint32_t tmp;

	/* Set pin as output */
	tmp = GPIOD->MODER; // support variable to set the BUTTON pin as output
	tmp &= (~BUTTON_REG_MASK); // reset
	GPIOD->MODER = (uint32_t)tmp; // write the MODER register

}

void initADC_Hall(){
	/**
	ADC2 GPIO Configuration:
	PC0     ------> ADC2_IN10    
    PC1     ------> ADC2_IN11 
	PC2     ------> ADC2_IN12 **/

	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    uint32_t tmp;

	/* Set ADC2 pin as Analog */
	tmp = GPIOC->MODER; // support variable to set the ADC pin as analog
	tmp |= ADC2_REG_MASK; // set the correct pattern 
	GPIOC->MODER = (uint32_t)tmp; // write the MODER register

	/* Set ADC2 no pullup nor pull down  */
	tmp = GPIOC->PUPDR; 
	tmp &= (~ADC2_NOPULL_MASK); 
	GPIOC->PUPDR = (uint32_t)tmp; 

	ADC2->CR1 = ((uint32_t) (ADC_CR1_SCAN));
	ADC2->CR2 = ((uint32_t) (ADC_CR2_EOCS | ADC_CR2_DMA | ADC_CR2_DDS)); // ADC_CR2_CONT
	ADC2->SMPR1 = ((uint32_t) ADC2_SMPR1);
	ADC2->SQR1 = ((uint32_t) ADC2_SQR1);
	ADC2->SQR3 = ((uint32_t) ADC2_SQR3);
	ADC2->CR2 |= ((uint32_t) ADC_CR2_ADON);
	while((ADC2->CR2 & ADC_CR2_ADON) == (~ADC_CR2_ADON)){} // wait until updated
}

void initADC_Shunt(){
	/**
	ADC1 GPIO Configuration:
	PB0     ------> ADC1_IN8    
    PB1     ------> ADC1_IN9 **/

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    uint32_t tmp;

	/* Set ADC1 pin as Analog */
	tmp = GPIOB->MODER; // support variable to set the ADC pin as analog
	tmp |= ADC1_REG_MASK; // set the correct pattern 
	GPIOB->MODER = (uint32_t)tmp; // write the MODER register

	/* Set ADC1 no pullup nor pull down  */
	tmp = GPIOB->PUPDR; 
	tmp &= (~ADC1_NOPULL_MASK); 
	GPIOB->PUPDR = (uint32_t)tmp; 

	ADC1->CR1 = ((uint32_t) (ADC_CR1_SCAN));
	ADC1->CR2 = ((uint32_t) (ADC_CR2_EOCS | ADC_CR2_DMA | ADC_CR2_DDS)); // ADC_CR2_CONT
	ADC1->SMPR2 = ((uint32_t) ADC1_SMPR2);
	ADC1->SQR1 = ((uint32_t) ADC1_SQR1);
	ADC1->SQR3 = ((uint32_t) ADC1_SQR3);
	ADC1->CR2 |= ((uint32_t) ADC_CR2_ADON);
	while((ADC1->CR2 & ADC_CR2_ADON) == (~ADC_CR2_ADON)){} // wait until updated
}

void initDMA(){
	uint32_t tmp;
	RCC->AHB1ENR |= ((uint32_t)RCC_AHB1ENR_DMA2EN); // enable clock

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Init DMA CURRENT SENSING
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	DMA2_Stream0->CR &= (~((uint32_t)DMA_SxCR_EN)); // disable DMA
	while((DMA2_Stream0->CR & DMA_SxCR_EN) == DMA_SxCR_EN){} // wait until updated

	DMA2->LIFCR = ((uint32_t)0xFFFFFFFF); // clear interrupt flags
	DMA2->HIFCR = ((uint32_t)0xFFFFFFFF); // clear interrupt flags

	DMA2_Stream0->CR &= ~((uint32_t)DMA_SxCR_DIR); // peripheral to memory transfer

	tmp = (uint32_t)DMA2_Stream0->CR;
	tmp &= ~((uint32_t)(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE));
	tmp |= ((uint32_t)(DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1)); // TODO: move to half word (16bits)
	DMA2_Stream0->CR = (uint32_t)tmp; // set peripheral and memory size to word (32 bits)

	tmp = (uint32_t)DMA2_Stream0->CR;
	tmp &= ~((uint32_t)(DMA_SxCR_CHSEL_Msk));
	DMA2_Stream0->CR = (uint32_t)tmp; // set channel 0

	DMA2_Stream0->CR |= ((uint32_t)DMA_SxCR_MINC); // memory increment after each transaction
	DMA2_Stream0->CR |= ((uint32_t)DMA_SxCR_CIRC); // circular mode to refill the array from the start the next iteration

	DMA2_Stream0->PAR = ((uint32_t)(&(ADC1->DR))); // ADC1 address 

	DMA2_Stream0->M0AR = ((int32_t)current); // memory base address

	DMA2_Stream0->NDTR = ((uint32_t)NUM_SAMPLES_ADC_CURRENT_DMA); // number of data

	DMA2_Stream0->FCR &= (~((uint32_t)DMA_SxFCR_DMDIS)); // direct mode (FIFO disabled)

	DMA2_Stream0->CR |= ((uint32_t)(DMA_SxCR_TCIE | DMA_SxCR_TEIE)); // enable transfer complete interrupt

	DMA2_Stream0->CR |= ((uint32_t)DMA_SxCR_EN); // enable DMA

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Init DMA HALL SENSING
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	DMA2_Stream2->CR &= (~((uint32_t)DMA_SxCR_EN)); // disable DMA
	while((DMA2_Stream2->CR & DMA_SxCR_EN) == DMA_SxCR_EN){} // wait until updated

	DMA2->LIFCR = ((uint32_t)0xFFFFFFFF); // clear interrupt flags
	DMA2->HIFCR = ((uint32_t)0xFFFFFFFF); // clear interrupt flags

	DMA2_Stream2->CR &= ~((uint32_t)DMA_SxCR_DIR); // peripheral to memory transfer

	tmp = (uint32_t)DMA2_Stream2->CR;
	tmp &= ~((uint32_t)(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE));
	tmp |= ((uint32_t)(DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1)); // TODO: move to half word (16bits)
	DMA2_Stream2->CR = (uint32_t)tmp; // set peripheral and memory size to word (32 bits)

	tmp = (uint32_t)DMA2_Stream2->CR;
	tmp &= ~((uint32_t)(DMA_SxCR_CHSEL_Msk));
	tmp |= ((uint32_t)(DMA_SxCR_CHSEL_0));
	DMA2_Stream2->CR = (uint32_t)tmp; // set channel 1	

	DMA2_Stream2->CR |= ((uint32_t)DMA_SxCR_MINC); // memory increment after each transaction
	DMA2_Stream2->CR |= ((uint32_t)DMA_SxCR_CIRC); // circular mode to refill the array from the start the next iteration

	DMA2_Stream2->PAR = ((uint32_t)(&(ADC2->DR))); // ADC2 address 

	DMA2_Stream2->M0AR = ((int32_t)hall); // memory base address

	DMA2_Stream2->NDTR = ((uint32_t)NUM_SAMPLES_ADC_HALL_DMA); // number of data

	DMA2_Stream2->FCR &= (~((uint32_t)DMA_SxFCR_DMDIS)); // direct mode (FIFO disabled)

	DMA2_Stream2->CR |= ((uint32_t)(DMA_SxCR_TCIE | DMA_SxCR_TEIE)); // enable transfer complete interrupt

	DMA2_Stream2->CR |= ((uint32_t)DMA_SxCR_EN); // enable DMA
}

void initTIM1(){
	/**
	TIM1 GPIO Configuration:    
    PA8      ------> PWM1_CH1
    PB13     ------> PWM1_CH1N
    PA9      ------> PWM1_CH2
    PB14     ------> PWM1_CH2N
    PA10     ------> PWM1_CH3
    PB15     ------> PWM1_CH3N  **/

	uint32_t tmp;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	
    tmp = GPIOB->MODER; 
	tmp &= (~PWM_B_REG_MASK); 
	tmp |= PWM_B_MODER; 	
	GPIOB->MODER = ((uint32_t)tmp);
	
    tmp = GPIOA->MODER; 
	tmp &= (~PWM_A_REG_MASK); 
	tmp |= PWM_A_MODER; 	
	GPIOA->MODER = ((uint32_t)tmp);

	tmp = GPIOB->AFR[1]; 
	tmp &= (~GPIOB_AF1_PWM_SEL_MASK);
	tmp |= GPIOB_AF1_PWM; 
	GPIOB->AFR[1] = ((uint32_t)tmp); 

	tmp = GPIOA->AFR[1]; 
	tmp &= (~GPIOA_AF1_PWM_SEL_MASK); 
	tmp |= GPIOA_AF1_PWM; 
	GPIOA->AFR[1] = ((uint32_t)tmp); 

	TIM1->ARR = ((uint16_t)TIM1_ARR); // frequency of the PWM
	TIM1->EGR |= (TIM_EGR_UG | TIM_EGR_CC4G); // generate an update event to reload the prescaler automatically and abilitate the compare module ch4
	
	TIM1->CCER &= (~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)); // reset bits to enable CH1,CH1N,CH2,CH2N,CH3,CH3N

	uint16_t tmp2;
	tmp2 = TIM1->CCMR1; 
	tmp2 &= (~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M));
	tmp2 |= (TIM1_CCMR1_PWM1_OC1 | TIM1_CCMR1_PWM1_OC2);
	TIM1->CCMR1 = ((uint16_t)tmp2);

	tmp2 = TIM1->CCMR2; 
	tmp2 &= (~TIM_CCMR2_OC3M);
	tmp2 |= (TIM1_CCMR2_PWM1_OC3);
	TIM1->CCMR2 = ((uint16_t)tmp2);

	TIM1->CCR1 = ((uint16_t)TIM1_CCR_CH1);
	TIM1->CCR2 = ((uint16_t)TIM1_CCR_CH2);
	TIM1->CCR3 = ((uint16_t)TIM1_CCR_CH3);

	tmp2 = TIM1->BDTR;
	tmp2 &= ~(TIM_BDTR_DTG);
	tmp2 |= (TIM1_DT);
	TIM1->BDTR = tmp2;

	TIM1->DIER |= TIM_DIER_CC4IE;
	//TIM1->CCER |= TIM_CCER_CC4E;
	TIM1->CCR4 = FREQ_PWM; // when low side on for all FET

	//TIM1->PSC = 0x1; // 10kHz PWM 

}

void startADC_Hall(){
	ADC2->CR2 |= ADC_CR2_SWSTART; // start sampling ADC and automatically store the data in current[0:1] using circular DMA
}

void stopADC_Hall(){
	ADC2->CR2 &= (uint16_t)(~ADC_CR2_ADON);
}

void startPWM_ADC_Shunt(){
	TIM1->CCER = ((uint16_t)TIM1_CCER_PWM1_COMPLEMENTARY_ACTIVE_HIGH); // Start normal + complementary active high
	//TIM1->CCER = ((uint16_t)TIM1_CCER_PWM1_COMPLEMENTARY_ACTIVE_LOW); // Start normal + complementary active low
	//TIM1->CCER = ((uint16_t)TIM1_CCER_PWM1_NORMAL); // Start normal

	TIM1->BDTR |= TIM_BDTR_MOE;

	TIM1->CR1 |= (TIM_CR1_CEN | TIM_CR1_CMS_0); 
}

void initDRV8302(){
	/**
	Enable GPIO Configuration:    
    PC10     ------> Enable pin for DRV8302 **/

	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIOCEN; // clock to AHB1-GPIOC enabled
	uint32_t tmp;

	/* Set pin as output */
	tmp = GPIOC->MODER;
	tmp &= (~DRV8302_EN_REG_MASK); // reset
	tmp |= DRV8302_EN; // set the correct pattern
	GPIOC->MODER = (uint32_t)tmp; // write the MODER register

	/* Set pin as pull-down */
	//tmp = GPIOC->PUPDR;
	//tmp &= (~DRV8302_EN_PULL_DOWN_REG_MASK); // reset
	//tmp |= DRV8302_EN_PULL_DOWN; // set the correct pattern
	//GPIOC->PUPDR = (uint32_t)tmp; // write the MODER register

	GPIOC->BSRR &= (uint16_t)(~DRV8302_ON);
	GPIOC->BSRR |= (uint16_t)DRV8302_OFF;
}

void en_DRV8302(){
	//GPIOC->BSRR &= (uint16_t)(~DRV8302_OFF);
	GPIOC->BSRR |= (uint16_t)DRV8302_ON;
}

void initENCODER(){
	/**
	I use Timer 5 because it is 32 bit and I do not wanna risk any overflow
	TIM5 GPIO Configuration:    
    PA0      ------> TI1
    PA1      ------> TI2  **/

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	int32_t tmp;
	tmp = TIM5->SMCR; 
	tmp &= (~TIM5_SMCR_MASK);
	tmp |= (TIM5_SMCR);
	TIM5->SMCR = ((uint16_t)tmp); // set encoder mode, counting on both edges
	
	tmp = TIM5->CCMR1; 
	tmp &= (~TIM5_CCMR1_MASK);
	tmp |= (TIM5_CCMR1);
	TIM5->CCMR1 = ((uint16_t)tmp); // assign TI1 and TI2

	tmp = TIM5->CCER; 
	tmp &= (~TIM5_CCER_MASK);
	tmp |= (TIM5_CCER);
	TIM5->CCER = ((uint16_t)tmp); // enable capture compare module
    
    TIM5->ARR |= 0xFFFFFFFF; // Auto reload at 4 294 967 296 -> at 20000 counts per turn we can make 214748.36 turns (enough I dare to say)

	tmp = GPIOA->MODER; 
	tmp &= (~ENCODER_A_REG_MASK); 
	tmp |= ENCODER_A_MODER; 	
	GPIOA->MODER = ((uint32_t)tmp); // select alternate function

	tmp = GPIOA->PUPDR;
	tmp &= (~ENCODER_A_REG_MASK); 
	tmp |= ENCODER_A_PUPDR; 	
	GPIOA->PUPDR = ((uint32_t)tmp); // select pull down

	tmp = GPIOA->AFR[0]; 
	tmp &= (~GPIOA_AF2_ENC_SEL_MASK); 
	tmp |= GPIOA_AF2_ENCODER; 
	GPIOA->AFR[0] = ((uint32_t)tmp); // select alternate function
}

void startENCODER(){
	TIM5->CNT = 0x80000000;  //set the counter to half the dynamic before we use it
	TIM5->CR1 |= TIM_CR1_CEN; 

	// The position in saved in TIM5->CNT
}

void initDAC(){
	/**
	DAC GPIO Configuration:    
    PA4     ------> DAC1  **/

	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	/* Set DAC pin as Analog */
	uint32_t tmp; 
	tmp = GPIOA->MODER; // support variable to set the DAC pin as analog
	tmp |= DAC_REG_MASK; // set the correct pattern 
	GPIOA->MODER = (uint32_t)tmp; // write the MODER register

	DAC->CR |= DAC_CR_EN1; // ENABLE DAC channel 1
	//DAC->DHR12R1 = 0x00000FFF; // test dac with maximum value (12 bit)
}

void initRelay(){
	/**
	Enable GPIO Configuration:    
    PC13     ------> Enable pin for Relay **/

	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIOCEN; // clock to AHB1-GPIOC enabled
	uint32_t tmp;

	/* Set pin as output */
	tmp = (uint32_t)GPIOC->MODER;
	tmp &= (uint32_t)(~RELAY_EN_REG_MASK); // reset
	tmp |= (uint32_t)RELAY_EN; // set the correct pattern
	GPIOC->MODER = (uint32_t)tmp; // write the MODER register

	GPIOC->BSRR |= (uint16_t)RELAY_OFF;
}