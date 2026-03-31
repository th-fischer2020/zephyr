/*
 * Copyright (c) 2026 JUMO GmbH & Co. KG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DT_BINDINGS_LT7680_H_
#define ZEPHYR_DT_BINDINGS_LT7680_H_

#include <dt-bindings/dt-util.h>


#define LT7680_MEM_WRITE_DIRECTION_LR_TB    0x00  // left→right, top→bottom
#define LT7680_MEM_WRITE_DIRECTION_RL_TB    0x02  // right→left, top→bottom
#define LT7680_MEM_WRITE_DIRECTION_TD_LR    0x04  // top→down,   left→right
#define LT7680_MEM_WRITE_DIRECTION_DT_LR    0x06  // down→top,   left→right

#define LT7680_VSCAN_DIRECTION_T_TO_B       0x00   // vertical scan top→bottom
#define LT7680_VSCAN_DIRECTION_B_TO_T       0x08   // vertical scan bottom→top

#endif /* ZEPHYR_DT_BINDINGS_LT7680_H_ */