/*
 * Airoha Technology AV201x silicon tuner driver
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2014 Luis Alves <ljalvs@gmail.com>
 * Copyright (C) 2020 Availink, Inc. (opensource@availink.com)
 *
 */

#ifndef AV201X_TOP_H
#define AV201X_TOP_H

#include <linux/kconfig.h>
#include <media/dvb_frontend.h>
//#include "dvb_frontend.h"
#include "av201x_avl_top_priv.h"

typedef enum av201x_id
{
	ID_AV2011,
	ID_AV2012,
	ID_AV2018,
} av201x_id_t;

struct av201x_avl_config
{
	/* tuner i2c address */
	uint8_t i2c_address;
	/* tuner type */
	av201x_id_t id;

	/* crystal freq in kHz */
	uint32_t xtal_freq;
};

extern struct dvb_frontend *av201x_avl_attach(struct dvb_frontend *fe,
					      struct av201x_avl_config *cfg,
					      struct i2c_adapter *i2c,
					      struct avl_tuner **demod_tuner_ptr);

#endif /* AV201X_H */
