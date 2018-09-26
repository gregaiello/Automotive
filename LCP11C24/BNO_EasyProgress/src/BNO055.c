/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Gregorio Aiello 2018
 * | BNO055 libs for LPC
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */

#include <BNO055.h>
#include "chip.h"

int init_BNO055(){
	uint8_t register_address = ST_ADDRESS, register_value = 0x0, reset_chip[2];
	reset_chip[0] = SYS_TRIGGER_ADDRESS;
	reset_chip[1] = SYS_TRIGGER_RESET_VALUE;
	Chip_I2C_MasterSend(I2C0, I2C_DEVICE_ADD, reset_chip, 2); // RESET CHIP
	for(uint32_t i = 0; i < 10000000; i++){__NOP();}
	Chip_I2C_MasterCmdRead(I2C0, I2C_DEVICE_ADD, register_address, &register_value, 1);
	if(register_value == ST_VALUE){
		uint8_t set_ext_tal[2];
		set_ext_tal[0] = SYS_TRIGGER_ADDRESS;
		set_ext_tal[1] = SYS_TRIGGER_EXT_XTAL_VALUE;
		Chip_I2C_MasterSend(I2C0, I2C_DEVICE_ADD, set_ext_tal, 2);
		register_address = OPR_MODE_ADDRESS;
		uint8_t tx_buffer[2];
		tx_buffer[0] = OPR_MODE_ADDRESS;
		tx_buffer[1] = OPR_MODE;
		Chip_I2C_MasterSend(I2C0, I2C_DEVICE_ADD, tx_buffer, 2); // GO TO NDOF MODE
		Chip_I2C_MasterCmdRead(I2C0, I2C_DEVICE_ADD, register_address, &register_value, 1);  // CHECK IF THE MODE IS OK
		if(register_value == OPR_MODE){
			return 0;
		}
		else return 1;
	}
	else return 2;
}
