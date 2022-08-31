/*
 * UP Element driver and EC configuration support
 *
 * Copyright (c) AAEON. All rights reserved.
 *
 * Author: Gary Wang <garywang@aaeon.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_UPBOARD_EC_H
#define __LINUX_MFD_UPBOARD_EC_H

#define NUC_AMR "NUC-AMR"

#define UPEC_STATUS_PORT	0x66
#define UPEC_CMD_PORT           0x66
#define UPEC_DATA_PORT          0x62
#define UPEC_CFG0_PORT          0x2e
#define UPEC_CFG1_PORT          0x4e
#define UPEC_CMD_READ          	0xfa
#define UPEC_CMD_WRITE          0xfb
#define UPEC_OBF                0x01
#define UPEC_IBF                0x02

#define UPEC_REGISTER_SIZE 8

enum upboard_ecreg {
	UPEC_REG_LED   = 0x2,
	UPEC_REG_DIR1  = 0x3,
	UPEC_REG_VAL1  = 0x4,
	UPEC_REG_DIR2  = 0x5,
	UPEC_REG_VAL2  = 0x6,
	UPEC_REG_MAX,
};

#define	UPEC_REG_DAT1  0x1D0C
#define	UPEC_REG_DAT2  0x1D09
#define	UPEC_REG_DAT_GP15  0x1607
#define	UPEC_REG_DAT_GP16  0x1601
#define	UPEC_REG_DAT_GP17  0x1604
#define	UPEC_REG_DAT_GP18  0x1605

struct upboard_ec {
	struct device *dev;
	struct regmap *regmap;
	const struct regmap_config *regmapconf;
};


#endif /*  __LINUX_MFD_UPBOARD_EC_H */
