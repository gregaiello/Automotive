/**
  ******************************************************************************
  * @file    biss.c
  * @author  Lucas M. Angarola
  * @version V1.0.0
  * @date    19-Apr-2018
  * @brief   
  ******************************************************************************
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f405xx.h"
#include "biss.h"
#include "hardwareSetup.h"

/* Local functions ------------------------------------------------------- */
uint32_t biss_crc4(uint32_t crcStart, uint32_t data);

/**
  * @brief  SCD Cycle function
  * This function reads the sensor data and writes only one bit (CDM) of
  * the register operation data. Check Biss documentation AN11.
  * @param  *sData 		 = Data received from the sensor.
  * 		cdmBitStatus = Register/command bit send to the sensor.
  * 		cdsBitStatus = Register/command bit received from the sensor.
  * @retval return 1 when there is a timout in the communication.
   */
int8_t biss_scdCycle(uint32_t *sData, uint8_t cdmBit){
	uint8_t 			counter 		= 0;					/* Total bits to send depending on the amount of devices connected */
	uint8_t 			devices 		= 0;					/* Detected devices on the line */
	uint16_t 			timeout 		= BISS_TIMEOUT_TIME;	/* Timout protection time */
	tDefEnum_bissState 	scdState 		= BISS_ACK;				/* Biss state machine first state */
	int8_t 				cdsBit = BISS_OK;									/* Read CDS bit from sensor */

	 /* Check current status of MA line 
	  * When MA is zero (due previous CDM bit), it should be set to
	  * one before start the read cycle */
	if(MA_read == 0){
		MA_set;
		uint32_t k;
		MA_init_state_delay;
	}

	/* Main SCD cycle loop*/
	while(scdState){
		/* Read the device with a negative clock slope */
		MA_reset;
		
		switch(scdState){
			/* The ACK of every device is being looked for one ACK per clock
			 * it is equal to the number of devices */
			case BISS_ACK:	
				if( SLO_state == 0 ){			
					devices++;				
					/* The number of bits that are going to be read per 
					 * each device is being updated */	
					counter += SCD_BITS;
				}
				/* If the input is positive and we detected at least one
				 *  ACK, it is the case of the START BIT, so... we move 
				 * forward in the state machine */
				else if(devices)
					scdState = BISS_CDS;

				/* Of course the timeout should be taken in 
				 * consideration in the case that we don't have ACK's or 
				 * START bit */
				if( (timeout--) == 0 ){
					cdsBit 		= BISS_TIMEOUT;
					scdState 	= BISS_STOP;
				}

				break;

			/* The CDS bit is being read */
			case BISS_CDS:	
				cdsBit 		= SLO_state;	
				scdState 	= BISS_DATA;
				break;

			/* At this point the data is being shifted in */
			case BISS_DATA:	
				sData[devices-1] |= (SLO_state & 0x01);			
				sData[devices-1] = sData[devices-1] << 1;

				counter--;

				/* This is the finishing flag */
				if(counter == 0){
					scdState = BISS_STOP;			
				}
				else if(counter == SCD_BITS)
					devices--;
					
				break;
				
			default: 
				break;
		}

		uint32_t k;
		MA_delay;
		
		/* If it is the last bit, the CDM bit should be sent in a 
		 * negate state. */
		if( !cdmBit || scdState )		
			MA_set;

		MA_delay;
	}

	uint32_t k;
	MA_init_state_delay;
	MA_init_state_delay;
	MA_init_state_delay;
	MA_init_state_delay;
	MA_init_state_delay;
	MA_init_state_delay;
	MA_init_state_delay;
	MA_init_state_delay;

	return cdsBit;
}

/**
  * @brief  
  * @param  None
  * @retval None
   */
int8_t biss_resetCommand(void){
	uint32_t sensor_data[2];

	/*14 Trailing cycles with CDM = 0 to abort a possible previously 
	 * started command frame. */
	uint8_t i;
	for(i=0; i<14; i++){	
		if( biss_scdCycle(sensor_data, 0) < 0 )
			return BISS_TIMEOUT;
	}
	
	return BISS_OK;
}


/**
  * @brief  Read register from the sensor
  * The register reading process takes 32 cycles.
  * First the ID, Address and CRC of these both is send to the sensor, and
  * the remaning bits, the sensor would replay with the content of register.
  * The packet construction is as follows:
  * 
  * [<-MSB              Send to sensor               ][  Read sensor ]
  * [START][CTS][ID:3 ][ADDR:7][CRC4:4][nR][nW][START][DATA:8][CRC4:4]
  * [  31 ][30 ][29-27][26-20 ][19-16 ][15][14][ 13  ][12-05 ][04-01 ]
  * 
  * Each bit (nCDM) of this packet is encapsulated on the SDC Cycles.
  * 
  * @param  regAddress: The address of the register that neds to be read.
  * 		deviceID: The device identifier.
  * @retval -2 : Timeout error.
  * 		-1 : General error, in this case CRC error.
  * 		Register Data is returned if no errors. 
   */
int16_t biss_readRegister(uint8_t regAddress, uint8_t deviceID){
	uint32_t 	wPacket, rPacket = 0; 			/* Write and receive packages */
	uint32_t 	mask 			 = 0x80000000;	/* Send word bit mask */
	uint32_t 	crc4;
	uint32_t 	sensorData;
	int8_t 		cdsBit;
	uint8_t 	regData;

	/* Start, CTS=1(register access), Read, Second Start */
	wPacket = 0xC000A000;
	
	/* Setup device ID and Address of the register */
	wPacket |= ((((uint32_t)deviceID) 		<< 27) & 0x38000000);
	wPacket |= ((((uint32_t)regAddress) 	<< 20) & 0x07F00000);

	/* Calculate CRC 4 */
	crc4 = biss_crc4(0, ((wPacket >> 20) & 0x00007FF)) ^ 0x0F;

	/* Setup CRC4 */
	wPacket |= ((((uint32_t)crc4) << 16) & 0x000F0000);

	/* Reset comm line before accessing address */
	if( biss_resetCommand() < 0 )
		return BISS_TIMEOUT;

	/* Send 32 CDM bits */
	uint8_t i;
	for(i=0; i<32; i++){
		if( wPacket & mask )
			cdsBit = biss_scdCycle(&sensorData, 1);
		else
			cdsBit = biss_scdCycle(&sensorData, 0);
		
		if( cdsBit < 0 )
			return BISS_TIMEOUT;

		mask >>= 1;

		rPacket |= (uint32_t)(cdsBit & 0x01);
		rPacket <<= 1;
	}

	regData 	= (uint8_t)((rPacket >> 5) & 0x000000FF);
	crc4 		= (uint8_t)((rPacket >> 1) & 0x0000000F);

	if( crc4 != (biss_crc4(0, (uint32_t)(regData)) ^ 0x0F) )
		return BISS_ERROR;

	return (int16_t)regData;
}

/**
  * @brief  Write register to the sensor
  * The register writing process takes 32 cycles.
  * First the ID, Address and CRC of these both plus the data that has
  * to be stored and its CRC calulation are sent to the sensor.
  * The packet construction is as follows:
  * 
  * [<-MSB                   Send to sensor                     LSB->]
  * [START][CTS][ID:3 ][ADDR:7][CRC4:4][nR][nW][START][DATA:8][CRC4:4]
  * [  31 ][30 ][29-27][26-20 ][19-16 ][15][14][ 13  ][12-05 ][04-01 ]
  * 
  * Each bit (nCDM) of this packet is encapsulated on the SDC Cycles.
  * 
  * @param  regAddress: The address of the register that neds to be read.
  * 		deviceID: The device identifier.
  * @retval -2 : Timeout error.
  * 		 0 : OK.
   */
int8_t biss_writeRegister(uint8_t regAddress, uint8_t deviceID, uint8_t regData){
	uint32_t 	wPacket, rPacket = 0; 	/* Write and receive packages */
	uint32_t 	mask = 0x80000000;		/* Send word bit mask */
	uint32_t 	crc4;
	int8_t 		cdsBit;
	uint32_t 	sensorData;

	/* Start, CTS=1(register access), Write, Second Start */
	wPacket = 0xC0006000;
	
	/* Setup device ID and Address of the register */
	wPacket |= ((((uint32_t)deviceID) 		<< 27) & 0x38000000);
	wPacket |= ((((uint32_t)regAddress) 	<< 20) & 0x07F00000);

	/* calculate CRC4 of regAddress plus ID plus CTS bit
	 * CRC4 should be placed in the packet inverted */
	crc4 = biss_crc4(0, ((wPacket >> 20) & 0x00007FF)) ^ 0x0F;

	/* Set CRC4 */
	wPacket |= ((((uint32_t)crc4) << 16) & 0x000F0000);
	
	/* Set register data */
	wPacket |= (((uint32_t)regData << 5) & 0x00001FE0);

	/* calculate CRC4 of the data to write */
	crc4 = biss_crc4(0, regData) ^ 0x0F;

	/* Set CRC4 into wPacket*/
	wPacket |= ((((uint32_t)crc4) << 1) & 0x0000001E);

	/* Reset comm line before accesing address */
	biss_resetCommand();

	/* Send 32 CDM bits */
	uint8_t i;
	for(i=0; i<32; i++){
		if( wPacket & mask )
			cdsBit = biss_scdCycle(&sensorData, 1);
		else
			cdsBit = biss_scdCycle(&sensorData, 0);
		
		if( cdsBit < 0 )
			return BISS_TIMEOUT;

		mask >>= 1;

		rPacket |= (uint32_t)(cdsBit & 0x01);
		rPacket <<= 1;
	}

	return BISS_OK;
}

/**
  * @brief  
  * @param  None
  * @retval None
   */
uint32_t biss_crc4(uint32_t crcStart, uint32_t data){
	uint8_t i;
	crcStart ^= data;

	for(i=0; i < CRC4_MAXIMUM_DATA_LEN; i++)
		if (crcStart & 0x1)
		  crcStart = (crcStart >> 1) ^ CRC4_POLYNOM_RECIPROCAL;
		else
		  crcStart >>= 1;

	return crcStart;
}

/*****************************END OF FILE****/
