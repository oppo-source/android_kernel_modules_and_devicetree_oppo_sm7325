/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015, 2018-2019 The Linux Foundation. All rights reserved.
 */

#ifndef _SWRM_REGISTERS_H
#define _SWRM_REGISTERS_H

#define SWRM_BASE_ADDRESS				0x00

#define SWRM_COMP_HW_VERSION                     SWRM_BASE_ADDRESS
#define SWRM_COMP_CFG_ADDR			(SWRM_BASE_ADDRESS+0x00000004)
#define SWRM_COMP_CFG_RMSK				0x3
#define SWRM_COMP_CFG_IRQ_LEVEL_OR_PULSE_BMSK		0x2
#define SWRM_COMP_CFG_IRQ_LEVEL_OR_PULSE_SHFT		0x1
#define SWRM_COMP_CFG_ENABLE_BMSK			0x1
#define SWRM_COMP_CFG_ENABLE_SHFT			0x0

#define SWRM_COMP_SW_RESET		(SWRM_BASE_ADDRESS+0x00000008)
#define SWRM_COMP_STATUS		(SWRM_BASE_ADDRESS+0x00000014)

#define SWRM_COMP_PARAMS		(SWRM_BASE_ADDRESS+0x100)
#define SWRM_COMP_PARAMS_DOUT_PORTS_MASK	0x0000001F
#define SWRM_COMP_PARAMS_DIN_PORTS_MASK		0x000003E0
#define SWRM_COMP_PARAMS_WR_FIFO_DEPTH		0x00007C00
#define SWRM_COMP_PARAMS_RD_FIFO_DEPTH		0x000F8000
#define SWRM_COMP_PARAMS_AUTO_ENUM_SLAVES	0x00F00000
#define SWRM_COMP_PARAMS_DATA_LANES		0x07000000

#define SWRM_COMP_MASTER_ID			(SWRM_BASE_ADDRESS+0x104)

#define SWRM_INTERRUPT_STATUS		(SWRM_BASE_ADDRESS+0x00000200)
#define SWRM_INTERRUPT_STATUS_RMSK		0x1FFFD

#define SWRM_INTERRUPT_STATUS_SLAVE_PEND_IRQ			0x1
#define SWRM_INTERRUPT_STATUS_NEW_SLAVE_ATTACHED		0x2
#define SWRM_INTERRUPT_STATUS_CHANGE_ENUM_SLAVE_STATUS		0x4
#define SWRM_INTERRUPT_STATUS_MASTER_CLASH_DET			0x8
#define SWRM_INTERRUPT_STATUS_RD_FIFO_OVERFLOW			0x10
#define SWRM_INTERRUPT_STATUS_RD_FIFO_UNDERFLOW			0x20
#define SWRM_INTERRUPT_STATUS_WR_CMD_FIFO_OVERFLOW		0x40
#define SWRM_INTERRUPT_STATUS_CMD_ERROR				0x80
#define SWRM_INTERRUPT_STATUS_DOUT_PORT_COLLISION		0x100
#define SWRM_INTERRUPT_STATUS_READ_EN_RD_VALID_MISMATCH		0x200
#define SWRM_INTERRUPT_STATUS_SPECIAL_CMD_ID_FINISHED		0x400
#define SWRM_INTERRUPT_STATUS_NEW_SLAVE_AUTO_ENUM_FINISHED	0x800
#define SWRM_INTERRUPT_STATUS_AUTO_ENUM_FAILED			0x1000
#define SWRM_INTERRUPT_STATUS_AUTO_ENUM_TABLE_IS_FULL		0x2000
#define SWRM_INTERRUPT_STATUS_BUS_RESET_FINISHED		0x4000
#define SWRM_INTERRUPT_STATUS_CLK_STOP_FINISHED			0x8000
#define SWRM_INTERRUPT_STATUS_ERROR_PORT_TEST			0x10000

#define SWRM_INTERRUPT_STATUS_AUTO_ENUM_FAILED_V2               0x800
#define SWRM_INTERRUPT_STATUS_AUTO_ENUM_TABLE_IS_FULL_V2        0x1000
#define SWRM_INTERRUPT_STATUS_BUS_RESET_FINISHED_V2             0x2000
#define SWRM_INTERRUPT_STATUS_CLK_STOP_FINISHED_V2              0x4000
#define SWRM_INTERRUPT_STATUS_ERROR_PORT_TEST_V2                0x8000
#define SWRM_INTERRUPT_STATUS_EXT_CLK_STOP_WAKEUP               0x10000

#define SWRM_INTERRUPT_MASK_ADDR		(SWRM_BASE_ADDRESS+0x00000204)
#define SWRM_INTERRUPT_MASK_RMSK		0x1FFFF

#define SWRM_INTERRUPT_MASK_SLAVE_PEND_IRQ_BMSK			0x1
#define SWRM_INTERRUPT_MASK_SLAVE_PEND_IRQ_SHFT			0x0

#define SWRM_INTERRUPT_MASK_NEW_SLAVE_ATTACHED_BMSK		0x2
#define SWRM_INTERRUPT_MASK_NEW_SLAVE_ATTACHED_SHFT		0x1

#define SWRM_INTERRUPT_MASK_CHANGE_ENUM_SLAVE_STATUS_BMSK	0x4
#define SWRM_INTERRUPT_MASK_CHANGE_ENUM_SLAVE_STATUS_SHFT	0x2

#define SWRM_INTERRUPT_MASK_MASTER_CLASH_DET_BMSK		0x8
#define SWRM_INTERRUPT_MASK_MASTER_CLASH_DET_SHFT		0x3

#define SWRM_INTERRUPT_MASK_RD_FIFO_OVERFLOW_BMSK		0x10
#define SWRM_INTERRUPT_MASK_RD_FIFO_OVERFLOW_SHFT		0x4

#define SWRM_INTERRUPT_MASK_RD_FIFO_UNDERFLOW_BMSK		0x20
#define SWRM_INTERRUPT_MASK_RD_FIFO_UNDERFLOW_SHFT		0x5

#define SWRM_INTERRUPT_MASK_WR_CMD_FIFO_OVERFLOW_BMSK		0x40
#define SWRM_INTERRUPT_MASK_WR_CMD_FIFO_OVERFLOW_SHFT		0x6

#define SWRM_INTERRUPT_MASK_CMD_ERROR_BMSK			0x80
#define SWRM_INTERRUPT_MASK_CMD_ERROR_SHFT			0x7

#define SWRM_INTERRUPT_MASK_DOUT_PORT_COLLISION_BMSK		0x100
#define SWRM_INTERRUPT_MASK_DOUT_PORT_COLLISION_SHFT		0x8

#define SWRM_INTERRUPT_MASK_READ_EN_RD_VALID_MISMATCH_BMSK	0x200
#define SWRM_INTERRUPT_MASK_READ_EN_RD_VALID_MISMATCH_SHFT	0x9

#define SWRM_INTERRUPT_MASK_SPECIAL_CMD_ID_FINISHED_BMSK	0x400
#define SWRM_INTERRUPT_MASK_SPECIAL_CMD_ID_FINISHED_SHFT	0xA

#define SWRM_INTERRUPT_MASK_NEW_SLAVE_AUTO_ENUM_FINISHED_BMSK	0x800
#define SWRM_INTERRUPT_MASK_NEW_SLAVE_AUTO_ENUM_FINISHED_SHFT	0xB

#define SWRM_INTERRUPT_MASK_AUTO_ENUM_FAILED_BMSK		0x1000
#define SWRM_INTERRUPT_MASK_AUTO_ENUM_FAILED_SHFT		0xC

#define SWRM_INTERRUPT_MASK_AUTO_ENUM_TABLE_IS_FULL_BMSK	0x2000
#define SWRM_INTERRUPT_MASK_AUTO_ENUM_TABLE_IS_FULL_SHFT	0xD

#define SWRM_INTERRUPT_MASK_BUS_RESET_FINISHED_BMSK		0x4000
#define SWRM_INTERRUPT_MASK_BUS_RESET_FINISHED_SHFT		0xE

#define SWRM_INTERRUPT_MASK_CLK_STOP_FINISHED_BMSK		0x8000
#define SWRM_INTERRUPT_MASK_CLK_STOP_FINISHED_SHFT		0xF

#define SWRM_INTERRUPT_MASK_ERROR_PORT_TEST_BMSK		0x10000
#define SWRM_INTERRUPT_MASK_ERROR_PORT_TEST_SHFT		0x10

#define SWRM_INTERRUPT_MAX					0x11

#define SWRM_INTERRUPT_CLEAR		(SWRM_BASE_ADDRESS+0x00000208)

#define SWR_MSTR_RX_SWRM_CPU_INTERRUPT_EN	(SWRM_BASE_ADDRESS+0x00000210)

#define SWRM_CMD_FIFO_WR_CMD		(SWRM_BASE_ADDRESS + 0x00000300)
#define SWRM_CMD_FIFO_WR_CMD_MASK	0xFFFFFFFF
#define SWRM_CMD_FIFO_RD_CMD		(SWRM_BASE_ADDRESS + 0x00000304)
#define SWRM_CMD_FIFO_RD_CMD_MASK	0xFFFFFFF
#define SWRM_CMD_FIFO_CMD		(SWRM_BASE_ADDRESS + 0x00000308)
#define SWRM_CMD_FIFO_STATUS		(SWRM_BASE_ADDRESS + 0x0000030C)

#define SWRM_CMD_FIFO_STATUS_WR_CMD_FIFO_CNT_MASK	0x1F00
#define SWRM_CMD_FIFO_STATUS_RD_CMD_FIFO_CNT_MASK	0x7C00000

#define SWRM_CMD_FIFO_CFG_ADDR			(SWRM_BASE_ADDRESS+0x00000314)
#define SWRM_CMD_FIFO_CFG_NUM_OF_CMD_RETRY_BMSK		0x7
#define SWRM_CMD_FIFO_CFG_NUM_OF_CMD_RETRY_SHFT		0x0

#define SWRM_CMD_FIFO_RD_FIFO_ADDR	(SWRM_BASE_ADDRESS + 0x00000318)

#define SWRM_ENUMERATOR_CFG_ADDR		(SWRM_BASE_ADDRESS+0x00000500)
#define SWRM_ENUMERATOR_CFG_AUTO_ENUM_EN_BMSK		0x1
#define SWRM_ENUMERATOR_CFG_AUTO_ENUM_EN_SHFT		0x0

#define SWRM_ENUMERATOR_STATUS              (SWRM_BASE_ADDRESS+0x00000504)
#define SWRM_ENUMERATOR_SLAVE_DEV_ID_1(m)   (SWRM_BASE_ADDRESS+0x530+0x8*m)
#define SWRM_ENUMERATOR_SLAVE_DEV_ID_2(m)   (SWRM_BASE_ADDRESS+0x534+0x8*m)

#define SWRM_MCP_FRAME_CTRL_BANK_ADDR(m)    (SWRM_BASE_ADDRESS+0x101C+0x40*m)
#define SWRM_MCP_FRAME_CTRL_BANK_RMSK			0x00ff07ff
#define SWRM_MCP_FRAME_CTRL_BANK_SHFT			0
#define SWRM_MCP_FRAME_CTRL_BANK_SSP_PERIOD_BMSK	0xff0000
#define SWRM_MCP_FRAME_CTRL_BANK_SSP_PERIOD_SHFT	16
#define SWRM_MCP_FRAME_CTRL_BANK_PHASE_BMSK		0xf800
#define SWRM_MCP_FRAME_CTRL_BANK_PHASE_SHFT		11
#define SWRM_MCP_FRAME_CTRL_BANK_CLK_DIV_VALUE_BMSK	0x700
#define SWRM_MCP_FRAME_CTRL_BANK_CLK_DIV_VALUE_SHFT	8
#define SWRM_MCP_FRAME_CTRL_BANK_ROW_CTRL_BMSK		0xF8
#define SWRM_MCP_FRAME_CTRL_BANK_ROW_CTRL_SHFT		3
#define SWRM_MCP_FRAME_CTRL_BANK_COL_CTRL_BMSK		0x7
#define SWRM_MCP_FRAME_CTRL_BANK_COL_CTRL_SHFT		0

#define SWRM_MCP_BUS_CTRL_ADDR			(SWRM_BASE_ADDRESS+0x00001044)
#define SWRM_MCP_BUS_CTRL_BUS_RESET_BMSK		0x1
#define SWRM_MCP_BUS_CTRL_BUS_RESET_SHFT		0x0
#define SWRM_MCP_BUS_CTRL_CLK_START_BMSK		0x2
#define SWRM_MCP_BUS_CTRL_CLK_START_SHFT		0x1

#define SWRM_MCP_CFG_ADDR			(SWRM_BASE_ADDRESS+0x00001048)
#define SWRM_MCP_CFG_MAX_NUM_OF_CMD_NO_PINGS_BMSK	0x3E0000
#define SWRM_MCP_CFG_MAX_NUM_OF_CMD_NO_PINGS_SHFT	0x11
#define SWRM_MCP_CFG_BUS_CLK_PAUSE_BMSK			0x02

#define SWRM_MCP_STATUS			(SWRM_BASE_ADDRESS+0x104C)
#define SWRM_MCP_STATUS_BANK_NUM_MASK	0x01

#define SWRM_MCP_SLV_STATUS		(SWRM_BASE_ADDRESS+0x1090)
#define SWRM_MCP_SLV_STATUS_MASK	0x03

#define SWRM_DP_PORT_CTRL_BANK(n, m)		(SWRM_BASE_ADDRESS + \
							0x00001124 + \
							0x100*(n-1) + \
							0x40*m)
#define SWRM_DP_PORT_CTRL_BANK_MASK		0xFFFFFFFF
#define SWRM_DP_PORT_CTRL_EN_CHAN_MASK		0xFF000000
#define SWRM_DP_PORT_CTRL_EN_CHAN_SHFT		0x18
#define SWRM_DP_PORT_CTRL_OFFSET2_SHFT		0x10
#define SWRM_DP_PORT_CTRL_OFFSET1_SHFT		0x08
#define SWRM_DP_PORT_CTRL_SAMPLE_INTERVAL	0x00

#define SWRM_DP_PORT_CTRL_2_BANK(n, m)	(SWRM_BASE_ADDRESS + \
							0x00001128 + \
							0x100*(n-1) + \
							0x40*m)

#define SWRM_DP_BLOCK_CTRL_1(n)		(SWRM_BASE_ADDRESS + \
							0x0000112C + \
							0x100*(n-1))

#define SWRM_DP_BLOCK_CTRL2_BANK(n, m)	(SWRM_BASE_ADDRESS + \
							0x00001130 + \
							0x100*(n-1) + \
							0x40*m)

#define SWRM_DP_PORT_HCTRL_BANK(n, m)	(SWRM_BASE_ADDRESS + \
							0x00001134 + \
							0x100*(n-1) + \
							0x40*m)

#define SWRM_DP_BLOCK_CTRL3_BANK(n, m)	(SWRM_BASE_ADDRESS + \
							0x00001138 + \
							0x100*(n-1) + \
							0x40*m)


#define SWRM_DIN_DPn_PCM_PORT_CTRL(n) (SWRM_BASE_ADDRESS + \
						0x00001054 + 0x100*(n-1))

#define SWRM_MAX_REGISTER SWRM_DIN_DPn_PCM_PORT_CTRL(7)

/* Soundwire Slave Register definition */

#define SWRS_BASE_ADDRESS			0x00

#define SWRS_DP_REG_OFFSET(port, bank)		((0x100*port)+(0x10*bank))

#define SWRS_SCP_INT_STATUS_CLEAR_1             0x40
#define SWRS_SCP_INT_STATUS_MASK_1		0x41

#define SWRS_SCP_CONTROL				0x44
#define SWRS_DP_BLOCK_CONTROL_1(n)		(SWRS_BASE_ADDRESS + 0x103 + \
						0x100 * n)

#define SWRS_DP_CHANNEL_ENABLE_BANK(n, m)	(SWRS_BASE_ADDRESS + 0x120 + \
						 SWRS_DP_REG_OFFSET(n, m))
#define SWRS_DP_BLOCK_CONTROL_2_BANK(n, m)	(SWRS_BASE_ADDRESS + 0x121 + \
						 SWRS_DP_REG_OFFSET(n, m))
#define SWRS_DP_SAMPLE_CONTROL_1_BANK(n, m)	(SWRS_BASE_ADDRESS + 0x122 + \
						 SWRS_DP_REG_OFFSET(n, m))
#define SWRS_DP_SAMPLE_CONTROL_2_BANK(n, m)	(SWRS_BASE_ADDRESS + 0x123 + \
						 SWRS_DP_REG_OFFSET(n, m))
#define SWRS_DP_OFFSET_CONTROL_1_BANK(n, m)	(SWRS_BASE_ADDRESS + 0x124 + \
						 SWRS_DP_REG_OFFSET(n, m))
#define SWRS_DP_OFFSET_CONTROL_2_BANK(n, m)	(SWRS_BASE_ADDRESS + 0x125 + \
						 SWRS_DP_REG_OFFSET(n, m))
#define SWRS_DP_HCONTROL_BANK(n, m)		(SWRS_BASE_ADDRESS + 0x126 + \
						 SWRS_DP_REG_OFFSET(n, m))
#define SWRS_DP_BLOCK_CONTROL_3_BANK(n, m)	(SWRS_BASE_ADDRESS + 0x127 + \
						 SWRS_DP_REG_OFFSET(n, m))
#define SWRS_DP_LANE_CONTROL_BANK(n, m) (SWRS_BASE_ADDRESS + 0x128 + \
						 SWRS_DP_REG_OFFSET(n, m))
#define SWRS_SCP_FRAME_CTRL_BANK(m)		(SWRS_BASE_ADDRESS + 0x60 + \
						 0x10*m)
#define SWRS_SCP_HOST_CLK_DIV2_CTL_BANK(m)	(SWRS_BASE_ADDRESS + 0xE0 + \
						0x10*m)

#endif /* _SWRM_REGISTERS_H */
