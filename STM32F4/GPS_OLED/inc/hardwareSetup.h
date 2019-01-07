/**
  ******************************************************************************
  * @file    hardwareSetup.h
  * @author  Lucas M. Angarola
  * @version V1.0.0
  * @date    19-Apr-2018
  * @brief   
  ******************************************************************************
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HARDWARESETUP_H
#define __HARDWARESETUP_H

#define MA_POS            0x8
#define MA_READ_MASK        0x100
#define MA_SET            0x100
#define MA_RESET          0x1000000
#define MA_SET_MASK         0x1000100

#define SLO_POS           0x9
#define SLO_READ_MASK       0x200

// Pin functions
#define MA_set            do {  (GPIOB->BSRR &= (uint32_t)(~MA_SET_MASK)); \
                      (GPIOB->BSRR |= (uint32_t)MA_SET);    } while(0)

#define MA_reset          do {  (GPIOB->BSRR &= (uint32_t)(~MA_SET_MASK)); \
                      (GPIOB->BSRR |= (uint32_t)MA_RESET);  } while(0)
            

#define MA_read           (( GPIOB->ODR & (uint16_t)MA_READ_MASK) >> MA_POS )
#define SLO_state         (( GPIOB->IDR & (uint16_t)SLO_READ_MASK) >> SLO_POS )

#endif /* __HARDWARESETUP_H */

/*****************************END OF FILE****/
