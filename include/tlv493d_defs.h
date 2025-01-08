// Copyright (C) 2025 Juen Rene´
//
// This file is part of esp-idf-tlv493d.
//
// esp-idf-tlv493d is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// esp-idf-tlv493d is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with esp-idf-tlv493d.  If not, see <https://www.gnu.org/licenses/>.
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef CONFIG_TLV493D_I2C_ADDRESS
    #define CONFIG_TLV493D_I2C_ADDRESS      0x5E
    #define CONFIG_TLV493D_I2C_ADDRESS1
#endif

#ifndef CONFIG_TLV493D_INT_ENABLE
    #define CONFIG_TLV493D_INT_ENABLE       0
#endif

#ifdef CONFIG_TLV493D_I2C_ADDRESS1
    #define RESET_VALUE 0xFF
#else
    #define RESET_VALUE 0x00
#endif

#define TLV493D_RESET_ADDRESS   0x00
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TIMEOUT_MS   100

#define WRITE_BIT 0
#define READ_BIT 1

#define ACK     1
#define N_ACK    0

#define READOUT_HIGH_PERIOD     12
#define READOUT_LOW_PERIOD      100
#define READOUT_MAX_TIMES       10

/******************
 *  Read register *
 ******************/
/**
 * \defgroup Readregister  
 * @{ 
 */

/** 
 * [7:0] x significant bits 11...4
 */
#define BANK_X1 0x00

/** 
 * [7:0] y significant bits 11...4
 */
#define BANK_Y1 0x01

/** 
 * [7:0] z significant bits 11...4
 */
#define BANK_Z1 0x02

/** 
 * [7:4] temp significant bits 3...0 \n 
 * [3:2] frame count \n 
 * [1:0] channel \n 
 */
#define BANK_TEMP1 0x03

/** 
 * [7:4] x siginificant bits 3...0 \n 
 * [3:0] y siginificant bits 3...0 \n 
 */
#define BANK_XY2 0x04

/** 
 * [7] not used \n 
 * [6] testmode flag \n 
 * [5] parity check \n 
 * [4] power down flag (must be 1 if complete otherwise reading is running) \n 
 * [3:0] z significant bits 3..0 \n 
 */
#define BANK_Z2 0x05

/** 
 * temperatur significant bits 11...4 
 */
#define BANK_TEMP2 0x06

/** 
 * [6:3] must be written from register BANK_Z2
 */
#define BANK_RES1 0x07

/** 
 * [7:0] must be written into register REG_MOD2
 */
#define BANK_RES2 0x08

/** 
 * [4:0] must be written into register REG_MOD3
 */
#define BANK_RES3 0x09

/**@}*/

/*******************
 *  Write register *
 *******************/

/**
 * \defgroup Writeregister  
 * @{ 
 */

/** 
 * reserved 
 */
#define REG_MOD0 0x00

/** 
 *  [7]     parity bit:  Sum of all 32 bits from write registers 0 H , 1H , 2 H and 3H must be odd. \n 
 *  [6:5]   i2c address bits: Bits can be set to “00”, “01”, “10” or “11” to define the slave address
 *          in bus configuration. Caution! must be filled from BANK_RES1 \n 
 *  [4:3]   must be filled from BANK_RES1 \n 
 *  [2]     interuped enabled \n 
 *  [1]     fast mode \n 
 *  [0]     low power mode \n 
 */
#define REG_MOD1 0x01

/** 
 *  fill witch BANK_RES2
 */
#define REG_MOD2 0x02

/** 
 *  [7]     Temperature measurement enabled \n 
 *  [6]     Low-power period “0” period is 100ms (ultra low-power period), “1” period is 12ms \n 
 *  [5]     Parity test enabled \n 
 *  [4:0]   must be filled from BANK_RES3 \n 
 */
#define REG_MOD3 0x03

/**@}*/

/**
 * \defgroup Defaults 
 * @{ 
 */

#define TLV493D_AXE_MULT 0.098
#define TLV493D_TEMP_MULT 1.1
#define TLV493D_TEMP_OFFSET 324
/**@}*/

#ifdef __cplusplus
}
#endif