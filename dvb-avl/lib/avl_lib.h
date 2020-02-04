/*
 * Library routines and defines for Availink demod and tuner drivers
 *
 * Copyright (C) 2020 Availink, Inc. (opensource@availink.com)
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _AVL_LIB_H_
#define _AVL_LIB_H_

#include "avl_bsp.h"

#define nullptr NULL
#define AVL_FALSE 0
#define AVL_TRUE 1
#define AVL_CONSTANT_10_TO_THE_9TH 1000000000 //10e9

#define AVL_MAX_II2C_READ_SIZE 64
#define AVL_MAX_II2C_WRITE_SIZE 64

#define AVL_MIN(x, y) (((x) < (y)) ? (x) : (y))
#define AVL_MAX(x, y) (((x) < (y)) ? (y) : (x))
#define AVL_ABS(a) (((a) > 0) ? (a) : (-(a)))

#define AVL_EC_OK		0 // There is no error.
#define AVL_EC_GENERAL_FAIL	1 // A general failure has occurred.
#define AVL_EC_RUNNING		2
#define AVL_EC_MemoryRunout	4
#define AVL_EC_TimeOut		8
#define AVL_EC_COMMAND_FAILED	16
#define AVL_EC_ConvertXLFSRToN_FAIL 0x0FFFFFFF


typedef uint8_t avl_bool_t;

struct avl_uint64
{
	uint32_t high_word;
	uint32_t low_word;
};

uint16_t avl_bms_initialize(void);
uint16_t avl_bms_read(uint16_t slave_addr,
		      uint32_t offset,
		      uint8_t *buf,
		      uint32_t size);
uint16_t avl_bms_read8(uint16_t slave_addr, uint32_t addr, uint8_t *data);
uint16_t avl_bms_read16(uint16_t slave_addr, uint32_t addr, uint16_t *data);
uint16_t avl_bms_read32(uint16_t slave_addr, uint32_t addr, uint32_t *data);
uint16_t avl_bms_read_direct(uint16_t slave_addr, uint8_t *buf, uint16_t size);
uint16_t avl_bms_write_direct(uint16_t slave_addr, uint8_t *buf, uint16_t size);
uint16_t avl_bms_write(uint16_t slave_addr, uint8_t *buf, uint32_t size);
uint16_t avl_bms_write8(uint16_t slave_addr, uint32_t addr, uint8_t data);
uint16_t avl_bms_write16(uint16_t slave_addr, uint32_t addr, uint16_t data);
uint16_t avl_bms_write32(uint16_t slave_addr, uint32_t addr, uint32_t data);

void avl_add_32to64(struct avl_uint64 *sum, uint32_t addend);
uint32_t avl_divide_64(struct avl_uint64 divisor, struct avl_uint64 dividend);
uint32_t avl_gte_64(struct avl_uint64 a, struct avl_uint64 b);
void avl_sub_64(struct avl_uint64 *a, struct avl_uint64 b);
void avl_mult_32to64(struct avl_uint64 *product, uint32_t m1, uint32_t m2);
void avl_add_scaled32to64(struct avl_uint64 *sum, uint32_t addend);
uint32_t avl_min_32(uint32_t a, uint32_t b);
uint32_t avl_max_32(uint32_t a, uint32_t b);

uint16_t avl_bytes_to_short(const uint8_t *buf);
uint32_t avl_bytes_to_int(const uint8_t *buf);
void avl_short_to_bytes(uint16_t data, uint8_t *buf);
void avl_int_to_bytes(uint32_t data, uint8_t *buf);
void avl_int_to_3bytes(uint32_t data, uint8_t *buf);


#endif
