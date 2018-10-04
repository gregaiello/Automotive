/**
  ******************************************************************************
  * @file    biss.h
  * @author  Lucas M. Angarola
  * @version V1.0.0
  * @date    19-Apr-2018
  * @brief   
  ******************************************************************************
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BISS_H
#define __BISS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/* Biss state machine enum */
typedef enum{
	BISS_STOP = 0,
	BISS_ACK,
	BISS_CDS,
	BISS_DATA
}tDefEnum_bissState;

/* Exported constants --------------------------------------------------------*/

/* Function status definition */
#define BISS_TIMEOUT 				-2		/* Timeout error */
#define BISS_ERROR					-1		/* General error */
#define BISS_OK						   0		/* OK status */

/* Clock line timing */
#define MA_TIMING					    10		/* Does not have a time unit... it is a loop counting value */
#define MA_INIT_TIMING				25		/* Does not have a time unit... it is a loop counting value */

/* Error condition defines */
#define BISS_TIMEOUT_TIME			1000	/* Does not have a time unit... it is a for loop counting value */

/* SCD cycle parameters
 * The SCD Cycle structure is like follows: 
 * 
 * [<-MSB     Read from the sensor       LSB->]
 * [ACK:x][START][CDS][DATA:12][nE][nW][CRC6:6]
 * 
 * There will be one ACK bit per device in the line, that is why the
 * of bits received at this point is identified as "x".
 */
#define SCD_FIXED_BITS				8								/* These bits are: nE + nW + CRC[6] */
#define SCD_DATA_BITS				12								/* These bits are: DATA[12] */
#define SCD_BITS					SCD_DATA_BITS + SCD_FIXED_BITS	/* These bits are: DATA[n] + nE + nW + CRC[6] */

/* CRC definitions */
#define CRC4_POLYNOM_RECIPROCAL 	((uint32_t)	0x00000009)	/* Orginal CRC 4 polynom for BISS => 0x13 */
#define CRC4_MAXIMUM_DATA_LEN		((uint8_t)	11)			/* BISS maximum data length for CRC 4 */

/* Delay function definition */
#define MA_delay					for(k=0;k<MA_TIMING;k++){ asm ("nop"); }
#define MA_init_state_delay 		for(k=0;k<MA_INIT_TIMING;k++){ asm ("nop"); }


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

int8_t 		biss_scdCycle(uint32_t *sData, uint8_t cdmBit);
int8_t 		biss_resetCommand(void);
int16_t 	biss_readRegister(uint8_t regAddress, uint8_t deviceID);
int8_t 		biss_writeRegister(uint8_t regAddress, uint8_t deviceID, uint8_t regData);

#ifdef __cplusplus
}
#endif

#endif /* __BISS_H */

/*****************************END OF FILE****/
