/*
 * Airoha Technology AV201x silicon tuner driver
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2014 Luis Alves <ljalvs@gmail.com>
 * Copyright (C) 2020 Availink, Inc. (opensource@availink.com)
 *
 */

#ifndef AV201X_AVL_TOP_PRIV_H
#define AV201X_AVL_TOP_PRIV_H

#include "av201x_avl_drv.h"
#include "avl_tuner.h"

struct av201x_avl_priv {
	struct av201x_avl_config *cfg;
	struct i2c_adapter *i2c;
  struct avl_tuner * pTuner;
};

enum av201x_regs_addr {
	REG_FN		= 0x00,
	REG_BWFILTER	= 0x05,
	REG_TUNER_STAT	= 0x0b,
	REG_TUNER_CTRL	= 0x0c,
	REG_FT_CTRL	= 0x25,
};

/* REG_TUNER_STAT */
#define AV201X_PLLLOCK		(1<<0)

/* REG_TUNER_CTRL */
#define AV201X_SLEEP		(1<<5)
#define AV201X_RFLP		(1<<6)

/* REG_FT_CTRL */
#define AV201X_FT_EN		(1<<1)
#define AV201X_FT_BLK		(1<<2)


#endif /* AV201X_PRIV_TOP_H */
