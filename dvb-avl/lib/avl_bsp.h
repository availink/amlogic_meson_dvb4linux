/*
 * Platform interface definition for Availink demod/tuner drivers
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

#ifndef _AVL_BSP_H_
#define _AVL_BSP_H_

/* Platform-specific includes */
#include <linux/i2c.h>
#include <linux/dvb/frontend.h>
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <linux/types.h>
#endif
#include <stddef.h>

typedef uint8_t avl_sem_t;
/********************************/


extern avl_sem_t avl_bsp_i2c_sem;

struct i2c_adapter *avl_bsp_assoc_i2c_adapter(uint16_t slave_addr,
					      struct i2c_adapter *i2c_adpt);
int32_t avl_bsp_initialize(void);
int32_t avl_bsp_reset(void);
int32_t avl_bsp_delay(uint32_t delay_ms);
int32_t avl_bsp_i2c_read(uint16_t slave_addr, uint8_t *buf, uint16_t *size);
int32_t avl_bsp_i2c_write(uint16_t slave_addr, uint8_t *buf, uint16_t *size);
int32_t avl_bsp_dispose(void);
int32_t avl_bsp_init_semaphore(avl_sem_t *sem);
int32_t avl_bsp_release_semaphore(avl_sem_t *sem);
int32_t avl_bsp_wait_semaphore(avl_sem_t *sem);



#endif
