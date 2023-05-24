/* SPDX-License-Identifier: GPL-2.0-only*/
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#include<mt-plat/mtk_boot_common.h>
#else
#include <soc/oplus/system/boot_mode.h>
#endif

#include <linux/spi/spi.h>
#include "hx83102j_noflash.h"

extern char Ctp_name[HARDWARE_MAX_ITEM_LONGTH];
extern int g_tp_dev_vendor;

#define OPPO17001TRULY_TD4322_1080P_CMD_PANEL 29

/*******Part0:LOG TAG Declear********************/

#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "himax,hx83102j_nf"
#else
#define TPD_DEVICE "himax,hx83102j_nf"
#endif

#ifdef HX_BOOT_UPGRADE
#define BOOT_UPGRADE_FWNAME "Himax_firmware.bin"
char *g_fw_boot_upgrade_name = BOOT_UPGRADE_FWNAME;
#endif
/*
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
	do {\
		if (LEVEL_DEBUG == tp_debug)\
			pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
	}while(0)

#define TPD_DETAIL(a, arg...)\
	do {\
		if (LEVEL_BASIC != tp_debug)\
			pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
	}while(0)

#define TPD_DEBUG_NTAG(a, arg...)\
	do {\
		if (tp_debug)\
			printk(a, ##arg);\
	}while(0)
*/
struct himax_report_data *hx_touch_data;
struct chip_data_hx83102j *g_chip_info;
int himax_touch_data_size = 128;
int hx_hw_reset_activate = 0;
static int hx_touch_info_point_cnt   = 0;
int g_lcd_vendor = 0;
int irq_en_cnt = 0;

int g_1kind_raw_size = 0;
uint32_t g_rslt_data_len;
char *g_rslt_data;
int **hx83102j_nf_inspection_criteria;
int *hx83102j_nf_inspt_crtra_flag;
int hx_criteria_item = 4;
int hx_criteria_size;
char *g_file_path_ok;
char *g_file_path_ng;
bool isea006proj = false;
bool isbd12proj = false;
bool isread_csv = true;
int hx83102j_nf_fail_write_count;

#ifdef HX_ZERO_FLASH
extern int g_f_0f_updat;
/* 128k+ */
int hx83102j_nf_cfg_crc = -1;
int hx83102j_nf_cfg_sz;
uint8_t hx83102j_nf_sram_min[4];
unsigned char *hx83102j_nf_fw_buf;
/* 128k- */
struct himax_core_fp g_core_fp;
#endif


#ifdef HX_ALG_OVERLAY
	uint8_t g_alg_idx_t = 0;
	bool g_has_alg_overlay;
#endif
int check_point_format;
unsigned char switch_algo;
uint8_t hx_proc_send_flag;
uint8_t in_self_test = 0;

int gflagautotest = 0;

uint32_t g_touch_ver = 0;
int himax_read_FW_status(struct chip_data_hx83102j *chip_info, uint8_t *state_addr, uint8_t *tmp_addr);
int himax_sram_write_crc_check(const struct firmware *fw_entry, uint8_t *addr, int strt_idx, uint32_t len);
/*******Part0: SPI Interface***************/
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
const struct mtk_chip_config hx_spi_ctrdata = {
	.sample_sel = 0,
	.cs_setuptime = 100,
	.cs_holdtime = 50,
	.cs_idletime = 0,
	.tick_delay = 0,
};
#else
const struct mt_chip_conf hx_spi_ctrdata = {
	.setuptime = 25,
	.holdtime = 25,
	.high_time = 3, /* 16.6MHz */
	.low_time = 3,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,

	.cpol = 0,
	.cpha = 0,

	.rx_mlsb = 1,
	.tx_mlsb = 1,

	.tx_endian = 0,
	.rx_endian = 0,

	.com_mod = DMA_TRANSFER,

	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif


int32_t spi_read_write_himax(struct spi_device *client, uint8_t *buf, size_t len,
		       uint8_t *rbuf, SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};
	u8 *tx_buf = NULL;
	u8 *rx_buf = NULL;
	int status;

	switch (rw) {
	case SPIREAD:
		tx_buf = tp_devm_kzalloc(&client->dev, 3, GFP_KERNEL | GFP_DMA);

		if (!tx_buf) {
			status = -ENOMEM;
			goto spi_out;
		}

		memset(tx_buf, 0xFF, 3);
		memcpy(tx_buf, buf, 3);
		rx_buf = tp_devm_kzalloc(&client->dev, len + 3 + DUMMY_BYTES, GFP_KERNEL | GFP_DMA);

		if (!rx_buf) {
			status = -ENOMEM;
			goto spi_out;
		}

		memset(rx_buf, 0xFF, len + DUMMY_BYTES);
		t.tx_buf = tx_buf;
		t.rx_buf = rx_buf;
		t.len    = (len + 3 + DUMMY_BYTES);
		break;

	case SPIWRITE:
		tx_buf = tp_devm_kzalloc(&client->dev, len, GFP_KERNEL | GFP_DMA);

		if (!tx_buf) {
			status = -ENOMEM;
			goto spi_out;
		}

		memcpy(tx_buf, buf, len);
		t.tx_buf = tx_buf;
		break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);


	status = spi_sync(client, &m);

	if (status == 0) {
		if (rw == SPIREAD) {
			memcpy(rbuf, rx_buf+3, len);
		}
	}

spi_out:

	if (tx_buf) {
		tp_devm_kfree(&client->dev, (void **)&tx_buf, len + 3 + DUMMY_BYTES);
	}

	if (rx_buf) {
		tp_devm_kfree(&client->dev, (void **)&rx_buf, len + DUMMY_BYTES);
	}

	return status;
}


static ssize_t himax_spi_sync(struct spi_device *spi, struct spi_message *message)
{
	int status;

	status = spi_sync(spi, message);

	return status;
}

static int himax_spi_write(uint8_t *buf, uint32_t len)
{
	int status = 0;
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};
	u8 *tx_buf = NULL;


	tx_buf = tp_devm_kzalloc(&g_chip_info->hx_spi->dev, len, GFP_KERNEL | GFP_DMA);

	if (!tx_buf) {
		status = -ENOMEM;
		goto spi_out;
	}

	memcpy(tx_buf, buf, len);
	t.tx_buf = tx_buf;


	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	himax_spi_sync(g_chip_info->hx_spi, &m);


spi_out:

	if (tx_buf) {
		tp_devm_kfree(&g_chip_info->hx_spi->dev, (void **)&tx_buf, len + DUMMY_BYTES);
	}

	return status;
}

static int himax_bus_read(uint8_t command, uint32_t length, uint8_t *data)
{
	int result = 0;
	uint8_t spi_format_buf[3];


	mutex_lock(&(g_chip_info->spi_lock));
	spi_format_buf[0] = 0xF3;
	spi_format_buf[1] = command;
	spi_format_buf[2] = 0x00;


	spi_read_write_himax(g_chip_info->hx_spi, spi_format_buf, length, data, SPIREAD);


	mutex_unlock(&(g_chip_info->spi_lock));

	return result;
}

int himax_bus_write_with_addr(uint8_t command, uint8_t *addr, uint8_t *data, uint32_t length)
{
	int result = 0;
	uint8_t *spi_format_buf = NULL;
	int alloc_size = 0;
	uint8_t offset = 0;
	uint32_t tmp_len = length;


	alloc_size = length;

	mutex_lock(&(g_chip_info->spi_lock));
	if (spi_format_buf == NULL) {
		spi_format_buf = kzalloc((alloc_size + 2) * sizeof(uint8_t), GFP_KERNEL);
	}

	if (spi_format_buf == NULL) {
		TPD_INFO("%s: Can't allocate enough buf\n", __func__);
		goto FAIL;
	}
	spi_format_buf[0] = 0xF2;
	spi_format_buf[1] = command;
	offset = 2;
	if (addr != NULL) {
		memcpy(spi_format_buf+offset, addr, 4);
		offset += 4;
		tmp_len -= 4;
	}

	if (data != NULL)
		memcpy(spi_format_buf+offset, data, tmp_len);
#ifdef CONFIG_VMAP_STACK
	result = himax_spi_write(spi_format_buf, length + 2);
#else
	result = himax_spi_write(spi_format_buf, length + 2);
#endif /*CONFIG_VMAP_STACK*/

FAIL:
	if(spi_format_buf != NULL) {
		kfree(spi_format_buf);
		spi_format_buf = NULL;
	}

	mutex_unlock(&(g_chip_info->spi_lock));
	return result;
}

static int himax_bus_write(uint8_t command, uint32_t length, uint8_t *data)
{
	/* uint8_t spi_format_buf[length + 2]; */
	int result = 0;
	static uint8_t *spi_format_buf;
	int alloc_size = 0;
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	alloc_size = MAX_TRANS_SZ + 14;
#else
	alloc_size = 49156;
#endif
	mutex_lock(&(g_chip_info->spi_lock));

	if (spi_format_buf == NULL) {
		spi_format_buf = kzalloc((alloc_size + 2) * sizeof(uint8_t), GFP_KERNEL);
	}

	if (spi_format_buf == NULL) {
		TPD_INFO("%s: Can't allocate enough buf\n", __func__);
		return -ENOMEM;
	}

	spi_format_buf[0] = 0xF2;
	spi_format_buf[1] = command;

	memcpy((uint8_t *)(&spi_format_buf[2]), data, length);
#ifdef CONFIG_VMAP_STACK
	result = himax_spi_write(spi_format_buf, length + 2);
#else
	result = himax_spi_write(spi_format_buf, length + 2);
#endif  /*CONFIG_VMAP_STACK*/
	mutex_unlock(&(g_chip_info->spi_lock));

	return result;
}

/*******Part1: Function Declearation*******/
static int hx83102j_power_control(void *chip_data, bool enable);
static int hx83102j_get_chip_info(void *chip_data);
static int hx83102j_mode_switch(void *chip_data, work_mode mode, int flag);
static uint32_t himax_hw_check_crc(uint8_t *start_addr, int reload_length);
static fw_check_state hx83102j_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data);
static void himax_read_FW_ver(void);
#ifdef HX_RST_PIN_FUNC
static int hx83102j_resetgpio_set(struct hw_resource *hw_res, bool on);
#endif
/*******Part2:Call Back Function implement*******/

/* add for himax */
void himax_flash_write_burst(uint8_t *reg_byte, uint8_t *write_data)
{
	uint8_t data_byte[8];
	int i = 0;
	int j = 0;

	for (i = 0; i < 4; i++) {
		data_byte[i] = reg_byte[i];
	}

	for (j = 4; j < 8; j++) {
		data_byte[j] = write_data[j - 4];
	}

	if (himax_bus_write(0x00, 8, data_byte) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return;
	}
}

void himax_flash_write_burst_length(uint8_t *reg_byte, uint8_t *write_data, int length)
{
	uint8_t *data_byte;
	data_byte = kzalloc(sizeof(uint8_t) * (length + 4), GFP_KERNEL);

	if (data_byte == NULL) {
		TPD_INFO("%s: Can't allocate enough buf\n", __func__);
		return;
	}

	memcpy(data_byte, reg_byte, 4); /* assign addr 4bytes */
	memcpy(data_byte + 4, write_data, length); /* assign data n bytes */

	if (himax_bus_write(0, length + 4, data_byte) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
	}
	kfree(data_byte);
}

void himax_burst_enable(uint8_t auto_add_4_byte)
{
	uint8_t tmp_data[4];
	tmp_data[0] = 0x31;

	if (himax_bus_write(0x13, 1, tmp_data) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return;
	}

	tmp_data[0] = (0x10 | auto_add_4_byte);
	if (himax_bus_write(0x0D, 1, tmp_data) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return;
	}
}

void himax_register_read(uint8_t *read_addr, int read_length, uint8_t *read_data, bool cfg_flag)
{
	uint8_t tmp_data[4];
	int ret;
	if(cfg_flag == false) {
		if(read_length > 256) {
			TPD_INFO("%s: read len over 256!\n", __func__);
			return;
		}
		if (read_length > 4) {
			himax_burst_enable(1);
		} else {
			himax_burst_enable(0);
		}

		tmp_data[0] = read_addr[0];
		tmp_data[1] = read_addr[1];
		tmp_data[2] = read_addr[2];
		tmp_data[3] = read_addr[3];
		ret = himax_bus_write(0x00, 4, tmp_data);
		if (ret < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return;
		}
		tmp_data[0] = 0x00;
		ret = himax_bus_write(0x0C, 1, tmp_data);
		if (ret < 0) {
			TPD_INFO("%s: bus1 access fail!\n", __func__);
			return;
		}

		if (himax_bus_read(0x08, read_length, read_data) < 0) {
			TPD_INFO("%s: bus2 access fail!\n", __func__);
			return;
		}
		if (read_length > 4) {
			himax_burst_enable(0);
		}
	} else if (cfg_flag == true) {
		if(himax_bus_read(read_addr[0], read_length, read_data) < 0) {
			TPD_INFO("%s: bus3 access fail!\n", __func__);
			return;
		}
	} else {
		TPD_INFO("%s: cfg_flag = %d, value is wrong!\n", __func__, cfg_flag);
		return;
	}
}

void himax_register_write(uint8_t *write_addr, int write_length, uint8_t *write_data, bool cfg_flag)
{
	int i = 0;
	int address = 0;
	if (cfg_flag == false) {
		address = (write_addr[3] << 24) + (write_addr[2] << 16) + (write_addr[1] << 8) + write_addr[0];

		for (i = address; i < address + write_length; i++) {
			if (write_length > 4) {
				himax_burst_enable(1);
			} else {
				himax_burst_enable(0);
			}
			himax_flash_write_burst_length(write_addr, write_data, write_length);
		}
	} else if (cfg_flag == true) {
		if (himax_bus_write(write_addr[0], write_length, write_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return;
		}
	} else {
		TPD_INFO("%s: cfg_flag = %d, value is wrong!\n", __func__, cfg_flag);
		return;
	}
}

static int himax_mcu_register_write(uint8_t *write_addr, uint32_t write_length, uint8_t *write_data, uint8_t cfg_flag)
{
	int total_read_times = 0;
	int max_bus_size = 128, test = 0;
	int total_size_temp = 0;
	int address = 0;
	int i = 0;

	uint8_t tmp_addr[4];
	uint8_t *tmp_data;

	total_size_temp = write_length;
	TPD_DETAIL("%s, Entering - total write size=%d\n", __func__, total_size_temp);

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	if (write_length > MAX_TRANS_SZ) {
		max_bus_size = MAX_TRANS_SZ;
	} else {
		max_bus_size = write_length;
	}

#else
	if (write_length > 49152) {
		max_bus_size = 49152;
	} else {
		max_bus_size = write_length;
	}
#endif

	himax_burst_enable(1);

	tmp_addr[3] = write_addr[3];
	tmp_addr[2] = write_addr[2];
	tmp_addr[1] = write_addr[1];
	tmp_addr[0] = write_addr[0];
	TPD_DEBUG("%s, write addr = 0x%02X%02X%02X%02X\n", __func__,
		tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);

	tmp_data = kzalloc(sizeof(uint8_t) * max_bus_size, GFP_KERNEL);
	if (tmp_data == NULL) {
		TPD_INFO("%s: Can't allocate enough buf \n", __func__);
		return -1;
	}

	if (total_size_temp % max_bus_size == 0) {
		total_read_times = total_size_temp / max_bus_size;
	} else {
		total_read_times = total_size_temp / max_bus_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		if (total_size_temp >= max_bus_size) {
			memcpy(tmp_data, write_data + (i * max_bus_size), max_bus_size);
			himax_flash_write_burst_length(tmp_addr, tmp_data, max_bus_size);

			total_size_temp = total_size_temp - max_bus_size;
		} else {
			test = total_size_temp % max_bus_size;
			memcpy(tmp_data, write_data + (i * max_bus_size), test);
			TPD_DEBUG("last total_size_temp=%d\n", total_size_temp % max_bus_size);

			himax_flash_write_burst_length(tmp_addr, tmp_data, max_bus_size);
		}

		address = ((i + 1) * max_bus_size);
		tmp_addr[0] = write_addr[0] + (uint8_t) ((address) & 0x00FF);

		if (tmp_addr[0] <  write_addr[0]) {
			tmp_addr[1] = write_addr[1] + (uint8_t) ((address >> 8) & 0x00FF) + 1;
		} else {
			tmp_addr[1] = write_addr[1] + (uint8_t) ((address >> 8) & 0x00FF);
		}

		udelay(100);
	}
	TPD_DETAIL("%s, End \n", __func__);
	kfree(tmp_data);
	return 0;
}
bool himax_mcu_sys_reset(void)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int retry = 0;

	do {
		/*===========================================
			0x31 ==> 0x27
		===========================================*/
		tmp_data[0] = 0x27;
		if (himax_bus_write(0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*===========================================
			0x32 ==> 0x95
		===========================================*/
		tmp_data[0] = 0x95;
		if (himax_bus_write(0x32, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*===========================================
			0x31 ==> 0x00
		===========================================*/
		tmp_data[0] = 0x00;
		if (himax_bus_write(0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		usleep_range(1000, 1001);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x98;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_addr[0] = 0xE4;
		himax_register_read(tmp_addr, 4, tmp_data, false);
	} while ((tmp_data[1] != 0x02 || tmp_data[0] != 0x00) && retry++ < 5);

	return true;
}
bool himax_sense_off(void)
{
	uint8_t cnt = 0;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	do {
		if (cnt == 0 || (tmp_data[0] != 0xA5 && tmp_data[0] != 0x00 && tmp_data[0] != 0x87)) {
			tmp_addr[3] = 0x90;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x5C;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0xA5;
			himax_flash_write_burst(tmp_addr, tmp_data);
		}

		usleep_range(20000, 20001);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		himax_register_read(tmp_addr, 4, tmp_data, false);
		if (tmp_data[0] != 0x05) {
			TPD_INFO("%s: it already in safe mode=0x%02X\n", __func__, tmp_data[0]);
		break;
		}

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x40;
		himax_register_read(tmp_addr, 4, tmp_data, false);
		TPD_DEBUG("%s: 10007F40: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		himax_register_read(tmp_addr, 4, tmp_data, false);
		TPD_DEBUG("%s: 10000000: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x04;
		himax_register_read(tmp_addr, 4, tmp_data, false);
		TPD_DEBUG("%s: 10007F04: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x02;
		tmp_addr[1] = 0x04;
		tmp_addr[0] = 0xF4;
		himax_register_read(tmp_addr, 4, tmp_data, false);
		TPD_DEBUG("%s: 800204B4: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		usleep_range(20000, 20001);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x5C;
			himax_register_read(tmp_addr, 4, tmp_data, false);

		cnt++;
		TPD_DEBUG("%s: save mode lock cnt = %d, data[0] = %2X!\n", __func__, cnt, tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (cnt < 10));


	cnt = 0;

	do {
		/*===========================================
			0x31 ==> 0x27
		===========================================*/
		tmp_data[0] = 0x27;
		if (himax_bus_write(0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*===========================================
			0x32 ==> 0x95
		===========================================*/
		tmp_data[0] = 0x95;
		if (himax_bus_write(0x32, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}

		/*======================
			Check enter_save_mode
		======================*/
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		himax_register_read(tmp_addr, 4, tmp_data, false);

		TPD_DEBUG("%s: Check enter_save_mode data[0]=%X \n", __func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/*=====================================
				Reset TCON
			=====================================*/
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x20;  /*reset*/
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			himax_flash_write_burst(tmp_addr, tmp_data);
			usleep_range(1000, 1001);

			/*=====================================
				Reset ADC
			=====================================*/
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x94;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			himax_flash_write_burst(tmp_addr, tmp_data);
			usleep_range(1000, 1001);
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x01;
			himax_flash_write_burst(tmp_addr, tmp_data);
			return true;
		} else {
			usleep_range(10000, 10001);
#ifdef HX_RST_PIN_FUNC
			hx83102j_resetgpio_set(g_chip_info->hw_res, false);
			hx83102j_resetgpio_set(g_chip_info->hw_res, true);
#else
			himax_mcu_sys_reset();
#endif
		}
	} while (cnt++ < 15);

	return false;
}

bool himax_sense_off_mptest(void)
{
	uint8_t cnt = 0;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	do {
		if (cnt == 0 || (tmp_data[0] != 0xA5 && tmp_data[0] != 0x00
					&& tmp_data[0] != 0x87)) {
			tmp_addr[3] = 0x90;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x5C;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0xA5;
			himax_flash_write_burst(tmp_addr, tmp_data);
		}

		msleep(20);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		himax_register_read(tmp_addr, 4, tmp_data, false);
		if (tmp_data[0] != 0x05) {
			TPD_INFO("%s: it already in safe mode=0x%02X\n", __func__, tmp_data[0]);
			break;
		}

		msleep(20);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x5C;
		himax_register_read(tmp_addr, 4, tmp_data, false);

		cnt++;
		TPD_INFO("%s: save mode lock cnt = %d, data[0] = %2X!\n", __func__, cnt,
				tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (cnt < 50));

	cnt = 0;

	do {
		/*=========================================
		0x31 ==> 0x27
		===========================================*/

		tmp_data[0] = 0x27;
		if (himax_bus_write(0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*
		===========================================
		0x32 ==> 0x95
		===========================================
		*/
		tmp_data[0] = 0x95;
		if (himax_bus_write(0x32, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*
		======================
		Check enter_save_mode
		======================
		*/
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		himax_register_read(tmp_addr, 4, tmp_data, false);

		TPD_INFO("%s: Check enter_save_mode data[0]=%X \n", __func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/*
			=====================================
			Reset TCON
			=====================================
			*/
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x20;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			himax_flash_write_burst(tmp_addr, tmp_data);
			msleep(1);
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x01;
			himax_flash_write_burst(tmp_addr, tmp_data);
			/*
			=====================================
			Reset ADC
			=====================================
			*/
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x94;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			himax_flash_write_burst(tmp_addr, tmp_data);
			msleep(1);
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x01;
			himax_flash_write_burst(tmp_addr, tmp_data);
			return true;
		} else {
			msleep(10);

#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(g_chip_info->hw_res, true);
		hx83102j_resetgpio_set(g_chip_info->hw_res, false);
		hx83102j_resetgpio_set(g_chip_info->hw_res, true);
#else
		g_core_fp.fp_sys_reset();
#endif
		}
	} while (cnt++ < 15);

	return false;
}

bool himax_enter_safe_mode(void)
{
	uint8_t cnt = 0;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	tmp_addr[3] = 0x90;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x5C;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0xA5;
	himax_flash_write_burst(tmp_addr, tmp_data);

	msleep(20);

	do {
		/*
		===========================================
		0x31 ==> 0x27
		===========================================
		*/
		tmp_data[0] = 0x27;
		if (himax_bus_write(0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*
		===========================================
		0x32 ==> 0x95
		===========================================
		*/
		tmp_data[0] = 0x95;
		if (himax_bus_write(0x32, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*
		===========================================
		0x31 ==> 0x00
		===========================================
		*/
		tmp_data[0] = 0x00;
		if (himax_bus_write(0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*
		===========================================
		0x31 ==> 0x27
		===========================================
		*/
		tmp_data[0] = 0x27;
		if (himax_bus_write(0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*
		===========================================
		0x32 ==> 0x95
		===========================================
		*/
		tmp_data[0] = 0x95;
		if (himax_bus_write(0x32, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*
		======================
		Check enter_save_mode
		======================
		*/
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		himax_register_read(tmp_addr, 4, tmp_data, false);

		TPD_DETAIL("%s: Check enter_save_mode data[0]=%X \n", __func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/*
			=====================================
			Reset TCON
			=====================================
			*/
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x20;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			himax_flash_write_burst(tmp_addr, tmp_data);
			msleep(1);
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x01;
			himax_flash_write_burst(tmp_addr, tmp_data);
			/*
			=====================================
			Reset ADC
			=====================================
			*/
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x94;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			himax_flash_write_burst(tmp_addr, tmp_data);
			msleep(1);
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x01;
			himax_flash_write_burst(tmp_addr, tmp_data);
			return true;
		} else {
			msleep(10);
#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(g_chip_info->hw_res, true);
		hx83102j_resetgpio_set(g_chip_info->hw_res, false);
		hx83102j_resetgpio_set(g_chip_info->hw_res, true);
#else
		g_core_fp.fp_sys_reset();
#endif
		}
	} while (cnt++ < 20);

	return false;
}

void himax_interface_on(void)
{
	uint8_t tmp_data[5];
	uint8_t tmp_data2[2];
	int cnt = 0;

	/*Read a dummy register to wake up I2C.*/
	if (himax_bus_read(0x08, 4, tmp_data) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return;
	}

	do {
		/*===========================================
			Enable continuous burst mode : 0x13 ==> 0x31
		===========================================*/
		tmp_data[0] = 0x31;
		if (himax_bus_write(0x13, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return;
		}
		/*===========================================
			AHB address auto +4 : 0x0D ==> 0x11
			Do not AHB address auto +4 : 0x0D ==> 0x10
		===========================================*/
		tmp_data[0] = (0x10);
		if (himax_bus_write(0x0D, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return;
		}

		/* Check cmd*/
		himax_bus_read(0x13, 1, tmp_data);
		himax_bus_read(0x0D, 1, tmp_data2);

		if (tmp_data[0] == 0x31 && tmp_data2[0] == 0x10) {
			break;
		}
		usleep_range(1000, 1001);
	} while (++cnt < 10);

	if (cnt > 0) {
		TPD_INFO("%s:Polling burst mode: %d times", __func__, cnt);
	}
}

void himax_diag_register_set(uint8_t diag_command)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	TPD_DEBUG("diag_command = %d\n", diag_command);

	himax_interface_on();

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x72;
	tmp_addr[0] = 0xEC;


	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = diag_command;
	himax_flash_write_burst(tmp_addr, tmp_data);

	himax_register_read(tmp_addr, 4, tmp_data, false);
	TPD_DEBUG("%s: tmp_data[3] = 0x%02X, tmp_data[2] = 0x%02X, tmp_data[1] = 0x%02X, tmp_data[0] = 0x%02X!\n",
			 __func__, tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
}


bool wait_wip(int Timing)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t in_buffer[10];
	/*uint8_t out_buffer[20];*/
	int retry_cnt = 0;

	/*=====================================
		SPI Transfer Format : 0x8000_0010 ==> 0x0002_0780
	=====================================*/
	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x10;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x02;
	tmp_data[1] = 0x07;
	tmp_data[0] = 0x80;
	himax_flash_write_burst(tmp_addr, tmp_data);

	in_buffer[0] = 0x01;

	do {
		/*=====================================
			SPI Transfer Control : 0x8000_0020 ==> 0x4200_0003
		=====================================*/
		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x20;
		tmp_data[3] = 0x42;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x03;
		himax_flash_write_burst(tmp_addr, tmp_data);

		/*=====================================
			SPI Command : 0x8000_0024 ==> 0x0000_0005
			read 0x8000_002C for 0x01, means wait success
		=====================================*/
		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x24;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x05;
		himax_flash_write_burst(tmp_addr, tmp_data);

		in_buffer[0] = in_buffer[1] = in_buffer[2] = in_buffer[3] = 0xFF;
		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x2C;
		himax_register_read(tmp_addr, 4, in_buffer, false);

		if ((in_buffer[0] & 0x01) == 0x00) {
			return true;
		}

		retry_cnt++;

		if (in_buffer[0] != 0x00 || in_buffer[1] != 0x00 || in_buffer[2] != 0x00 || in_buffer[3] != 0x00) {
			TPD_INFO("%s:Wait wip retry_cnt:%d, buffer[0]=%d, buffer[1]=%d, buffer[2]=%d, buffer[3]=%d \n", __func__,
					 retry_cnt, in_buffer[0], in_buffer[1], in_buffer[2], in_buffer[3]);
		}

		if (retry_cnt > 100) {
			TPD_INFO("%s: Wait wip error!\n", __func__);
			return false;
		}
		msleep(Timing);
	} while ((in_buffer[0] & 0x01) == 0x01);
	return true;
}

static void himax_hx83102j_reload_to_active(void)
{
	uint8_t addr[4] = {0};
	uint8_t data[4] = {0};
	uint8_t retry_cnt = 0;

	addr[3] = 0x90;
	addr[2] = 0x00;
	addr[1] = 0x00;
	addr[0] = 0x48;

	do {
		data[3] = 0x00;
		data[2] = 0x00;
		data[1] = 0x00;
		data[0] = 0xEC;
		himax_register_write(addr, 4, data, 0);
		usleep_range(1000, 1001);
		himax_register_read(addr, 4, data, 0);
		retry_cnt++;
	} while ((data[1] != 0x01 || data[0] != 0xEC) && retry_cnt < HIMAX_REG_RETRY_TIMES);
}
void himax_sense_on_by_sys_rst(void)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	himax_interface_on();
	tmp_addr[3] = 0x90;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x5C;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	himax_flash_write_burst(tmp_addr, tmp_data);


	tmp_addr[3] = 0x90;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x18;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x55;
	himax_flash_write_burst(tmp_addr, tmp_data);

	himax_hx83102j_reload_to_active();
}
void himax_sense_on(uint8_t FlashMode)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	TPD_DETAIL("Enter %s  \n", __func__);

	himax_interface_on();
	tmp_addr[3] = 0x90;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x5C;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	himax_flash_write_burst(tmp_addr, tmp_data);

	usleep_range(10000, 11000);


	if (!FlashMode) {
		g_core_fp.fp_sys_reset();
	} else {
		/* reset code*/
		tmp_data[0] = 0x00;
		if (himax_bus_write(0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
		}
		if (himax_bus_write(0x32, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
		}
	}
	himax_hx83102j_reload_to_active();
}

/**
 * hx83102j_enable_interrupt -   Device interrupt ability control.
 * @chip_info: struct include i2c resource.
 * @enable: disable or enable control purpose.
 * Return  0: succeed, -1: failed.
 */

static int hx83102j_enable_interrupt(struct chip_data_hx83102j *chip_info, bool enable)
{
	if (enable == true && irq_en_cnt == 0) {
		enable_irq(chip_info->hx_irq);
		irq_en_cnt = 1;
	} else if (enable == false && irq_en_cnt == 1) {
		disable_irq_nosync(chip_info->hx_irq);
		irq_en_cnt = 0;
	} else {
		TPD_DETAIL("irq is not pairing! enable= %d, cnt = %d\n", enable, irq_en_cnt);
	}

	return 0;
}

#ifdef HX_ZERO_FLASH
struct zf_operation *pzf_op = NULL;
bool g_auto_update_flag = false;
int g_poweronof = 1;

#if defined(HX_ALG_OVERLAY)
static int himax_mcu_register_read(uint8_t *addr, uint8_t *buf, uint32_t len)
{
	int ret = -1;

	if (addr[0] == pzf_op->addr_spi200_data[0]
	&& addr[1] == pzf_op->addr_spi200_data[1]
	&& addr[2] == pzf_op->addr_spi200_data[2]
	&& addr[3] == pzf_op->addr_spi200_data[3])
		himax_burst_enable(0);
	else if (len > DATA_LEN_4)
		himax_burst_enable(1);
	else
		himax_burst_enable(0);

	ret = himax_bus_write_with_addr(pzf_op->addr_ahb_addr_byte_0[0], addr, NULL, 4);
	if (ret < 0) {
		TPD_INFO("%s: bus access fail!\n", __func__);

		return -1;
	}

	ret = himax_bus_write_with_addr(pzf_op->addr_ahb_access_direction[0], NULL,
		&pzf_op->data_ahb_access_direction_read[0], 1);
	if (ret < 0) {
		TPD_INFO("%s: bus access fail!\n", __func__);

		return -1;
	}

	ret = himax_bus_read(pzf_op->addr_ahb_rdata_byte_0[0], len, buf);
	if (ret < 0) {
		TPD_INFO("%s: bus access fail!\n", __func__);

		return -1;
	}


	return NO_ERR;
}


static int himax_mcu_assign_sorting_mode(uint8_t *tmp_data)
{
	uint8_t retry = 0;
	uint8_t rdata[4] = {0};

	TPD_INFO("%s:addr: 0x%02X%02X%02X%02X, write to:0x%02X%02X%02X%02X\n",
		__func__,
		pzf_op->addr_sorting_mode_en[3],
		pzf_op->addr_sorting_mode_en[2],
		pzf_op->addr_sorting_mode_en[1],
		pzf_op->addr_sorting_mode_en[0],
		tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);

	while (retry++ < 3) {
		himax_mcu_register_write(pzf_op->addr_sorting_mode_en, DATA_LEN_4,
			tmp_data, 0);
		usleep_range(1000, 1100);
		himax_mcu_register_read(pzf_op->addr_sorting_mode_en,
			rdata, DATA_LEN_4);

		if (rdata[3] == tmp_data[3] && rdata[2] == tmp_data[2]
		&& rdata[1] == tmp_data[1] && rdata[0] == tmp_data[0]) {
			TPD_INFO("%s: success to write sorting mode\n", __func__);
			return NO_ERR;
		}
		TPD_INFO("%s: fail to write sorting mode\n", __func__);
	}
	return -1;
}
#endif
#if defined(HX_ALG_OVERLAY)
int alg_overlay(uint8_t alg_idx_t, struct zf_info *info,
	const struct firmware *fw)
{
	int ret = 0;
	int retry = 0;
	uint8_t tmp_addr[4] = {0xFC, 0x7F, 0x00, 0x10};
	uint8_t rdata[4] = {0};

	uint8_t i = 0;
	uint8_t alg_sdata[4] = {0xA5, 0x5A, 0x5A, 0xA5};

	uint8_t data[4] = {0x01, 0x00, 0x00, 0x00};

	struct chip_data_hx83102j *chip_info = g_chip_info;
	uint8_t tmp_data[64];

	TPD_INFO("%s: Enter \n", __func__);

	if (alg_idx_t == 0 || info[alg_idx_t].write_size == 0) {
		TPD_INFO("%s: wrong alg overlay section[%d, %d]!\n", __func__,
			alg_idx_t, info[alg_idx_t].write_size);
		ret = FW_NOT_READY;
		goto ALOC_CFG_BUF_FAIL;
	}

	retry = 0;
	do {
		himax_mcu_register_write(tmp_addr, DATA_LEN_4, alg_sdata,
			0);
		usleep_range(1000, 1100);
		himax_mcu_register_read(tmp_addr, rdata,
			DATA_LEN_4);
	} while ((rdata[0] != alg_sdata[0]
	|| rdata[1] != alg_sdata[1]
	|| rdata[2] != alg_sdata[2]
	|| rdata[3] != alg_sdata[3])
	&& retry++ < HIMAX_REG_RETRY_TIMES);

	if (retry > HIMAX_REG_RETRY_TIMES) {
		TPD_INFO("%s: init handshaking data FAIL[%02X%02X%02X%02X]!!\n",
			__func__, rdata[0], rdata[1], rdata[2], rdata[3]);
	}

	alg_sdata[3] = OVL_ALG_REPLY;
	alg_sdata[2] = OVL_ALG_REPLY;
	alg_sdata[1] = OVL_ALG_REPLY;
	alg_sdata[0] = OVL_ALG_REPLY;

	g_core_fp.fp_reload_disable(0);


	himax_mcu_register_write(pzf_op->addr_raw_out_sel, sizeof(pzf_op->data_clear),
		pzf_op->data_clear, 0);
	/*DSRAM func initial*/
	himax_mcu_assign_sorting_mode(pzf_op->data_clear);
	/* reset N frame back to default for normal mode */
	himax_mcu_register_write(pzf_op->addr_set_frame_addr, 4, data, 0);
	/*FW reload done initial*/
	himax_mcu_register_write(pzf_op->addr_fw_define_2nd_flash_reload, sizeof(pzf_op->data_clear) ,
		pzf_op->data_clear, 0);

	himax_sense_on(0x00);

	retry = 0;
	do {
		usleep_range(3000, 3100);
		himax_mcu_register_read(tmp_addr, rdata, DATA_LEN_4);
	} while ((rdata[0] != OVL_ALG_REQUEST
	|| rdata[1] != OVL_ALG_REQUEST
	|| rdata[2] != OVL_ALG_REQUEST
	|| rdata[3] != OVL_ALG_REQUEST)
	&& retry++ < 30);

	if (retry > 30) {
		TPD_INFO("%s: fail req data = 0x%02X%02X%02X%02X\n", __func__,
			rdata[0], rdata[1], rdata[2], rdata[3]);
		/* monitor FW status for debug */
		for (i = 0; i < 10; i++) {
			usleep_range(10000, 10100);
			himax_mcu_register_read(tmp_addr, rdata, DATA_LEN_4);
			TPD_DEBUG("%s: req data = 0x%02X%02X%02X%02X\n",
				__func__, rdata[0], rdata[1], rdata[2],
				rdata[3]);
			cmd_set[0] = 0x01;
			himax_read_FW_status(chip_info, cmd_set, tmp_data);
		}
		ret = 3;
		goto BURN_OVL_FAIL;
	}

	TPD_DEBUG("%s: upgrade alg overlay section[%d]\n", __func__, alg_idx_t);

	if (himax_sram_write_crc_check(fw, info[alg_idx_t].sram_addr,
		info[alg_idx_t].fw_addr,
		info[alg_idx_t].write_size) != 0) {
		TPD_INFO("%s: Alg Overlay HW crc FAIL\n", __func__);
		ret = 2;
	}

	retry = 0;
	do {
		himax_mcu_register_write(tmp_addr, DATA_LEN_4, alg_sdata, 0);
		usleep_range(1000, 1100);
		himax_mcu_register_read(tmp_addr, rdata, DATA_LEN_4);
	} while ((alg_sdata[3] != rdata[3]
	|| alg_sdata[2] != rdata[2]
	|| alg_sdata[1] != rdata[1]
	|| alg_sdata[0] != rdata[0])
	&& retry++ < HIMAX_REG_RETRY_TIMES);

	if (retry > HIMAX_REG_RETRY_TIMES) {
		TPD_INFO("%s: fail rpl data = 0x%02X%02X%02X%02X\n", __func__,
			rdata[0], rdata[1], rdata[2], rdata[3]);
	} else {
		TPD_INFO("%s: waiting for FW reload data", __func__);

		retry = 0;
		while (retry++ < 30) {
			himax_mcu_register_read(
				pzf_op->addr_fw_define_2nd_flash_reload,
				data, DATA_LEN_4);

			/* use all 4 bytes to compare */
			if ((data[3] == 0x00 && data[2] == 0x00 &&
			data[1] == 0x72 && data[0] == 0xC0)) {
				TPD_INFO("%s: FW reload done\n", __func__);
					break;
			}
			TPD_INFO("%s: wait FW reload %d times\n", __func__, retry);
			cmd_set[0] = 0x01;
			himax_read_FW_status(chip_info, cmd_set, tmp_data);
			usleep_range(10000, 11000);
		}
	}

BURN_OVL_FAIL:
ALOC_CFG_BUF_FAIL:
	TPD_INFO("%s: Exit \n", __func__);
	return ret;
}
#endif
void himax_in_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len)
{
	/*TPD_INFO("%s: Entering!\n", __func__);*/
	switch (len) {
	case 1:
		cmd[0] = addr;
		/*TPD_INFO("%s: cmd[0] = 0x%02X\n", __func__, cmd[0]);*/
		break;

	case 2:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		/*TPD_INFO("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X\n", __func__, cmd[0], cmd[1]);*/
		break;

	case 4:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		cmd[2] = (addr >> 16) % 0x100;
		cmd[3] = addr / 0x1000000;
		/*  TPD_INFO("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X,cmd[2] = 0x%02X,cmd[3] = 0x%02X\n",
			__func__, cmd[0], cmd[1], cmd[2], cmd[3]);*/
		break;

	default:
		TPD_INFO("%s: input length fault, len = %d!\n", __func__, len);
	}
}

void hx_update_dirly_0f(void)
{
	TPD_DEBUG("It will update fw after esd event in zero flash mode!\n");
	g_core_fp.fp_0f_operation_dirly();
}

int hx_dis_rload_0f(int disable)
{
	/*Diable Flash Reload*/
	int retry = 10;
	int check_val = 0;
	uint8_t tmp_data[4] = {0};

	TPD_DETAIL("%s: Entering !\n", __func__);

	do {
		himax_flash_write_burst(pzf_op->addr_dis_flash_reload,  pzf_op->data_dis_flash_reload);
		himax_register_read(pzf_op->addr_dis_flash_reload, 4, tmp_data, false);
		TPD_DETAIL("Now data: tmp_data[3] = 0x%02X || tmp_data[2] = 0x%02X || tmp_data[1] = 0x%02X || tmp_data[0] = 0x%02X\n",
					tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
		if(tmp_data[3] != 0x00 || tmp_data[2] != 0x00 || tmp_data[1] != 0x9A || tmp_data[0] != 0xA9) {
			TPD_INFO("Now data: tmp_data[3] = 0x%02X || tmp_data[2] = 0x%02X || tmp_data[1] = 0x%02X || tmp_data[0] = 0x%02X\n",
						tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
			TPD_INFO("Not Same,Write Fail, there is %d retry times!\n", retry);
			usleep_range(5000, 5001);
		} else {
			check_val = 1;
			TPD_DETAIL("It's same! Write success!\n");
		}
	} while (check_val == 0 && retry-- > 0);

	TPD_DETAIL("%s: END !\n", __func__);

	return check_val;
}

void himax_mcu_clean_sram_0f(uint8_t *addr, int write_len, int type)
{
	int total_read_times = 0;
	int max_bus_size = MAX_TRANS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	int address = 0;
	int i = 0;
	uint8_t fix_data = 0x00;
	uint8_t tmp_addr[4];
	uint8_t *tmp_data;

	TPD_DETAIL("%s, Entering \n", __func__);
	tmp_data = kzalloc(sizeof(uint8_t) * MAX_TRANS_SZ, GFP_KERNEL);
	total_size = write_len;
	total_size_temp = write_len;

	if (total_size > MAX_TRANS_SZ) {
		max_bus_size = MAX_TRANS_SZ;
	}

	total_size_temp = write_len;

	himax_burst_enable(1);

	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];
	TPD_DETAIL("%s, write addr tmp_addr[3] = 0x%2.2X, tmp_addr[2] = 0x%2.2X, tmp_addr[1] = 0x%2.2X, tmp_addr[0] = 0x%2.2X\n",
				__func__, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);

	switch (type) {
	case 0:
		fix_data = 0x00;
		break;
	case 1:
		fix_data = 0xAA;
		break;
	case 2:
		fix_data = 0xBB;
		break;
	}

	for (i = 0; i < MAX_TRANS_SZ; i++) {
		tmp_data[i] = fix_data;
	}

	TPD_DETAIL("%s, total size=%d\n", __func__, total_size);

	if (total_size_temp % max_bus_size == 0) {
		total_read_times = total_size_temp / max_bus_size;
	} else {
		total_read_times = total_size_temp / max_bus_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		/*TPD_DETAIL("[log]write %d time start!\n", i);*/
		if (total_size_temp >= max_bus_size) {
			himax_flash_write_burst_length(tmp_addr, tmp_data,  max_bus_size);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			TPD_DETAIL("last total_size_temp=%d\n", total_size_temp);
			himax_flash_write_burst_length(tmp_addr, tmp_data,  total_size_temp % max_bus_size);
		}
		address = ((i + 1) * max_bus_size);
		tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF);
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);

		msleep(10);
	}
	kfree(tmp_data);
	TPD_DETAIL("%s, END \n", __func__);
}

void himax_mcu_write_sram_0f(const struct firmware *fw_entry, uint8_t *addr, int start_index, uint32_t write_len)
{
	int total_read_times = 0;
	int max_bus_size = MAX_TRANS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	uint32_t address = 0;
	int i = 0;
	uint8_t tmp_addr[4];
	uint32_t now_addr;

	TPD_DETAIL("%s, ---Entering \n", __func__);

	total_size = fw_entry->size;

	total_size_temp = write_len;
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	if (write_len > MAX_TRANS_SZ) {
		max_bus_size = MAX_TRANS_SZ;
	} else {
		max_bus_size = write_len;
	}
#else
	if (write_len > HX64K) {
		max_bus_size = HX64K;
	} else {
		max_bus_size = write_len;
	}
#endif
	himax_burst_enable(1);

	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];
	TPD_DETAIL("%s, write addr tmp_addr[3] = 0x%2.2X, tmp_addr[2] = 0x%2.2X, tmp_addr[1] = 0x%2.2X, tmp_addr[0] = 0x%2.2X\n",
			__func__, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);
	now_addr = (addr[3] << 24) + (addr[2] << 16) + (addr[1] << 8) + addr[0];
	TPD_DETAIL("now addr= 0x%08X\n", now_addr);

	TPD_DETAIL("%s,  total size=%d\n", __func__, total_size);

	if (g_chip_info->tmp_data == NULL) {
		g_chip_info->tmp_data = kzalloc(sizeof(uint8_t) * firmware_update_space, GFP_KERNEL);
		if (g_chip_info->tmp_data == NULL) {
			TPD_INFO("%s, alloc g_chip_info->tmp_data failed\n", __func__);
			return;
		}
	}
	memcpy(g_chip_info->tmp_data, fw_entry->data, total_size);
	/*
	for (i = 0;i < 10;i++) {
		TPD_INFO("[%d] 0x%2.2X", i, tmp_data[i]);
	}
	TPD_INFO("\n");
	*/
	if (total_size_temp % max_bus_size == 0) {
		total_read_times = total_size_temp / max_bus_size;
	} else {
		total_read_times = total_size_temp / max_bus_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		/*
		TPD_INFO("[log]write %d time start!\n", i);
		TPD_INFO("[log]addr[3] = 0x%02X, addr[2] = 0x%02X, addr[1] = 0x%02X, addr[0] = 0x%02X!\n", tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);
		*/
		if (total_size_temp >= max_bus_size) {
			himax_flash_write_burst_length(tmp_addr, &(g_chip_info->tmp_data[start_index + i * max_bus_size]),  max_bus_size);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			TPD_DETAIL("last total_size_temp=%d\n", total_size_temp);
			himax_flash_write_burst_length(tmp_addr, &(g_chip_info->tmp_data[start_index + i * max_bus_size]),  total_size_temp % max_bus_size);
		}

		/*TPD_INFO("[log]write %d time end!\n", i);*/
		address = ((i + 1) * max_bus_size)+now_addr;
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);

		tmp_addr[3] = (address>>24) & 0x00FF;
		tmp_addr[2] = (address>>16) & 0x00FF;
		tmp_addr[1] = (address>>8) & 0x00FF;
		tmp_addr[0] = address & 0x00FF;
		/*
		if (tmp_addr[0] < addr[0]) {
			tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF) + 1;
		} else {
			tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF);
		}*/


		udelay(100);
	}
	TPD_DETAIL("%s, ----END \n", __func__);

	memset(g_chip_info->tmp_data, 0, total_size);
}
int himax_sram_write_crc_check(const struct firmware *fw_entry, uint8_t *addr, int strt_idx, uint32_t len)
{
	int retry = 0;
	int crc = -1;

	TPD_DETAIL("%s, addr=0x%02X%02X%02X%02X, start_idx=%d=0x%08X, len=%d\n",
		__func__, addr[3], addr[2], addr[1], addr[0], strt_idx, strt_idx, len);

	if (len <= 0) {
		TPD_INFO("%s,len=%d, Fail!!\n", __func__, len);
		goto END;
	}


	do {
		g_core_fp.fp_write_sram_0f(fw_entry, addr, strt_idx, len);
		crc = himax_hw_check_crc(addr, len);
		retry++;
		TPD_DETAIL("%s, HW crc %s in %d time\n", __func__, (crc == 0)?"OK":"Fail", retry);
	} while (crc != 0 && retry < 10);

END:
	return crc;
}

static int himax_mcu_Calculate_crc_with_AP(unsigned char *FW_content, int crc_from_FW, int len)
{
	int i, j, length = 0;
	int fw_data;
	int fw_data_2;
	int crc = 0xFFFFFFFF;
	int polynomial = 0x82F63B78;

	length = len / 4;

	for (i = 0; i < length; i++) {
		fw_data = FW_content[i * 4];

		for (j = 1; j < 4; j++) {
			fw_data_2 = FW_content[i * 4 + j];
			fw_data += (fw_data_2) << (8 * j);
		}

		crc = fw_data ^ crc;

		for (j = 0; j < 32; j++) {
			if ((crc % 2) != 0) {
				crc = ((crc >> 1) & 0x7FFFFFFF) ^ polynomial;
			} else {
				crc = (((crc >> 1) & 0x7FFFFFFF)/*& 0x7FFFFFFF*/);
			}
		}
		/*I("crc = %x, i = %d \n", crc, i);*/
	}

	return crc;
}

#if defined(HX_CODE_OVERLAY)
static int hx83102j_0f_overlay(int ovl_type, int mode)
{
	uint8_t count = 0;
	uint8_t count2 = 0;
	uint8_t part_num = 0;
	uint8_t buf[16];
	uint8_t handshaking_addr[4] = {0xFC, 0x7F, 0x00, 0x10};
	uint8_t ovl_idx_t;
	uint8_t request;
	uint8_t reply;
	uint8_t sram_addr[4] = {0};
	uint32_t offset = 0;
	uint32_t size = 0;
	uint8_t send_data[4] = {0};
	uint8_t recv_data[4] = {0};
	int ret = 0;
	uint32_t cfg_table_pos = cfg_table_flash_addr;
	struct firmware *request_fw_headfile = NULL;
	const struct firmware *tmp_fw_entry = NULL;

	 /*1. check fw status*/
	TPD_INFO("Get FW from buffer or headfile\n");
	if (request_fw_headfile == NULL) {
		request_fw_headfile = kzalloc(sizeof(struct firmware), GFP_KERNEL);
	}
	if(request_fw_headfile == NULL) {
		TPD_INFO("%s kzalloc is failed!\n", __func__);
		ret = MEM_ALLOC_FAIL;
		goto FAIL_OUT1;
	}
	if (g_chip_info->g_fw_sta) {
		TPD_INFO("get from g_fw_buf\n");
		request_fw_headfile->size = g_chip_info->g_fw_len;
		request_fw_headfile->data = g_chip_info->g_fw_buf;
		tmp_fw_entry = request_fw_headfile;
	} else {
		TPD_INFO("get from g_fw_buf failed, get from headfile\n");
		if(g_chip_info->p_firmware_headfile->firmware_data) {
			request_fw_headfile->size = g_chip_info->p_firmware_headfile->firmware_size;
			request_fw_headfile->data = g_chip_info->p_firmware_headfile->firmware_data;
			tmp_fw_entry = request_fw_headfile;
			g_chip_info->using_headfile = true;
		} else {
			TPD_INFO("firmware_data is NULL! exit firmware update!\n");
			ret = FW_NOT_READY;
			goto FAIL_OUT2;
		}
	}

	/*2. check in self_test or not*/
	if (in_self_test == 0) {
	/*1. get number of partition*/
			TPD_DETAIL("%s, cfg_table_pos=0x%08X\n", __func__, cfg_table_pos);
			part_num = tmp_fw_entry->data[cfg_table_pos + 12];
			if (part_num <= 1) {
				TPD_INFO("%s, size of cfg part failed! part_num = %d\n",
						__func__, part_num);
				ret = LENGTH_FAIL;
				goto FAIL_OUT2;
			}

			TPD_DETAIL("%s: overlay section %d\n", __func__, ovl_type-1);
			if (ovl_type == 2) {
				request = OVL_GESTURE_REQUEST;
				reply = OVL_GESTURE_REPLY;

			} else if (ovl_type == 3) {
				request = OVL_BORDER_REQUEST;
				reply = OVL_BORDER_REPLY;

			} else {
				TPD_INFO("%s: error overlay type %d\n", __func__, ovl_type);
				ret = HX_INIT_FAIL;
				goto FAIL_OUT2;
			}
			ovl_idx_t = ovl_idx[ovl_type - 1];
			memcpy(buf, &tmp_fw_entry->data[ovl_idx_t * 0x10 + cfg_table_pos], 16);
			memcpy(sram_addr, buf, 4);
			offset = buf[11] << 24 | buf[10] << 16 | buf[9] << 8 | buf[8];
			size = buf[7] << 24 | buf[6] << 16 | buf[5] << 8 | buf[4];

		hx83102j_enable_interrupt(g_chip_info, false);
		/*himax_enter_safe_mode();*/
		himax_sense_off();

			if (himax_sram_write_crc_check(tmp_fw_entry, sram_addr, offset, size) != 0)
				TPD_INFO("%s, Overlay HW crc FAIL\n", __func__);

			send_data[3] = 0x00;
			send_data[2] = 0x00;
			send_data[1] = 0x00;
			send_data[0] = reply;
			count2 = 0;
			do {
				himax_register_write(handshaking_addr, 4, send_data, false);
				msleep(1);
				himax_register_read(handshaking_addr, 4, recv_data, false);
			} while (recv_data[0] != reply && count2++ < 10);
	}

	TPD_DETAIL("%s: overlay request %d times; reply %d times\n", __func__,
			count, count2);

	/* rescue mechanism
*	if (count >= 10) {
*		g_core_fp.fp_0f_operation_dirly();
*		g_core_fp.fp_reload_disable(0);
*		himax_sense_on(0x00);
*		hx83102j_enable_interrupt(g_chip_info, true);
*	}
*/
		ret = 0;
FAIL_OUT2:
	 if (request_fw_headfile != NULL) {
		kfree(request_fw_headfile);
		request_fw_headfile = NULL;
	}
FAIL_OUT1:
	himax_sense_on_by_sys_rst();
	hx83102j_enable_interrupt(g_chip_info, true);
	return ret;
}
#endif
static bool hx_bin_desc_data_get(uint32_t addr, uint8_t *flash_buf)
{
	uint8_t data_sz = 0x10;
	uint32_t i = 0, j = 0;
	uint16_t chk_end = 0;
	uint16_t chk_sum = 0;
	uint32_t map_code = 0;
	unsigned long flash_addr = 0;

	for (i = 0; i < FW_PAGE_SZ; i = i + data_sz) {
		for (j = i; j < (i + data_sz); j++) {
			chk_end |= flash_buf[j];
			chk_sum += flash_buf[j];
		}
		if (!chk_end) { /*1. Check all zero*/
			TPD_INFO("%s: End in %X\n",	__func__, i + addr);
			return false;
		} else if (chk_sum % 0x100) { /*2. Check sum*/
			TPD_DETAIL("%s: chk sum failed in %X\n",	__func__, i + addr);
		} else { /*3. get data*/
			map_code = flash_buf[i] + (flash_buf[i + 1] << 8)
			+ (flash_buf[i + 2] << 16) + (flash_buf[i + 3] << 24);
			flash_addr = flash_buf[i + 4] + (flash_buf[i + 5] << 8)
			+ (flash_buf[i + 6] << 16) + (flash_buf[i + 7] << 24);
			switch (map_code) {
			case FW_CID:
				cid_ver_maj_flash_addr = flash_addr;
				cid_ver_min_flash_addr = flash_addr + 1;
				TPD_DETAIL("%s: CID_VER in %lX\n", __func__,
				cid_ver_maj_flash_addr);
				break;
			case FW_VER:
				fw_ver_maj_flash_addr = flash_addr;
				fw_ver_min_flash_addr = flash_addr + 1;
				TPD_DETAIL("%s: FW_VER in %lX\n", __func__,
				fw_ver_maj_flash_addr);
				break;
			case CFG_VER:
				cfg_ver_maj_flash_addr = flash_addr;
				cfg_ver_min_flash_addr = flash_addr + 1;
				TPD_DETAIL("%s: CFG_VER in = %08lX\n", __func__,
				cfg_ver_maj_flash_addr);
				break;
			case TP_CONFIG_TABLE:
				cfg_table_flash_addr = flash_addr;
				TPD_DETAIL("%s: CONFIG_TABLE in %X\n",
				__func__, cfg_table_flash_addr);
				break;
			}
		}
		chk_end = 0;
		chk_sum = 0;
	}

	return true;
}

static bool hx_mcu_bin_desc_get(unsigned char *fw, uint32_t max_sz)
{
	uint32_t addr_t = 0;
	unsigned char *fw_buf = NULL;
	bool keep_on_flag = false;
	bool g_bin_desc_flag = false;

	do {
		fw_buf = &fw[addr_t];

		/*Check bin is with description table or not*/
		if (!g_bin_desc_flag) {
			if (fw_buf[0x00] == 0x00 && fw_buf[0x01] == 0x00
			&& fw_buf[0x02] == 0x00 && fw_buf[0x03] == 0x00
			&& fw_buf[0x04] == 0x00 && fw_buf[0x05] == 0x00
			&& fw_buf[0x06] == 0x00 && fw_buf[0x07] == 0x00
			&& fw_buf[0x0E] == 0x87)
				g_bin_desc_flag = true;
		}
		if (!g_bin_desc_flag) {
			TPD_INFO("%s: fw_buf[0x00] = %2X, fw_buf[0x0E] = %2X\n",
			__func__, fw_buf[0x00], fw_buf[0x0E]);
			TPD_INFO("%s: No description table\n",	__func__);
			break;
		}

		/*Get related data*/
		keep_on_flag = hx_bin_desc_data_get(addr_t, fw_buf);

		addr_t = addr_t + FW_PAGE_SZ;
	} while (max_sz > addr_t && keep_on_flag);

	return g_bin_desc_flag;
}
bool hx_parse_bin_cfg_data(const struct firmware *fw_entry)
{
	uint32_t cfg_table_pos = cfg_table_flash_addr;
	bool flag_1k_header = false;
	int part_num = 0;
	int i = 0;
	uint8_t buf[16];
	int i_max = 0;
	int i_min = 0;
	uint32_t dsram_base = 0xFFFFFFFF;
	uint32_t dsram_max = 0;

#if defined(HX_CODE_OVERLAY) || defined(HX_ALG_OVERLAY)
	int allovlidx = 0;

#endif

#if defined(HX_ALG_OVERLAY)

	 g_alg_idx_t = 0;
	 g_has_alg_overlay = false;
#endif
#if defined(HX_CODE_OVERLAY)
	uint8_t j = 0;

	if (ovl_idx == NULL) {
		ovl_idx = kzalloc(OVL_SECTION_NUM, GFP_KERNEL);
		if (ovl_idx == NULL) {
			TPD_INFO("%s, ovl_idx alloc failed!\n",
				__func__);
			return -ENOMEM;
		}
	} else {
		memset(ovl_idx, 0, OVL_SECTION_NUM);
	}
#endif

	/*0. check 1k header*/
	if (fw_entry->data[0x00] == 0x00
		&& fw_entry->data[0x01] == 0x00
		&& fw_entry->data[0x02] == 0x00
		&& fw_entry->data[0x03] == 0x00
		&& fw_entry->data[0x04] == 0x00
		&& fw_entry->data[0x05] == 0x00
		&& fw_entry->data[0x06] == 0x00
		&& fw_entry->data[0x07] == 0x00
		&& fw_entry->data[0x0E] == 0x87)
		flag_1k_header = true;
	else
		flag_1k_header = false;

	/*1. get number of partition*/
	part_num = fw_entry->data[cfg_table_pos + 12];


	if (part_num <= 1) {
		TPD_INFO("%s, size of cfg part failed! part_num = %d\n", __func__, part_num);
		return false;
	}
	/*2. initial struct of array*/
	g_zf_info_arr = kzalloc(part_num * sizeof(struct zf_info), GFP_KERNEL);
	if (g_zf_info_arr == NULL) {
		TPD_INFO("%s, Allocate ZF info array failed!\n", __func__);
		return false;
	}

	for (i = 0; i < part_num; i++) {
		/*3. get all partition*/
		TPD_DETAIL("%s, i * 0x10 + cfg_table_pos =%d \n", __func__, i * 0x10 + cfg_table_pos);
		memcpy(buf, &fw_entry->data[i * 0x10 + cfg_table_pos], 16);
		TPD_DETAIL("%s, buf[7]=0x%x buf[6]=0x%x buf[5]=0x%x buf[4]=0x%x \n", __func__, buf[7], buf[6], buf[5], buf[4]);
		memcpy(g_zf_info_arr[i].sram_addr, buf, 4);
		g_zf_info_arr[i].write_size =  buf[7] << 24 | buf[6] << 16
				| buf[5] << 8 | buf[4];
		g_zf_info_arr[i].fw_addr = buf[10] << 16 | buf[9] << 8 | buf[8];
		g_zf_info_arr[i].cfg_addr = g_zf_info_arr[i].sram_addr[0];
		g_zf_info_arr[i].cfg_addr += g_zf_info_arr[i].sram_addr[1] << 8;
		g_zf_info_arr[i].cfg_addr += g_zf_info_arr[i].sram_addr[2] << 16;
		g_zf_info_arr[i].cfg_addr += g_zf_info_arr[i].sram_addr[3] << 24;

#if defined(HX_CODE_OVERLAY)
		/*overlay section*/
		if ((buf[15] == 0x55 && buf[14] == 0x66)
		|| (buf[3] == 0x20 && buf[2] == 0x00
		&& buf[1] == 0x8C && buf[0] == 0xE0)) {
			TPD_INFO("%s: catch overlay section in index %d\n",
				__func__, i);

			/* record index of overlay section */
			allovlidx |= 1 << i;

			if (buf[15] == 0x55 && buf[14] == 0x66) {
				/* current mechanism */
				j = buf[13];
				if (j < OVL_SECTION_NUM)
					ovl_idx[j] = i;
			} else {
				/* previous mechanism */
				if (j < OVL_SECTION_NUM)
					ovl_idx[j++] = i;
			}

			continue;
		}
#endif
#if defined(HX_ALG_OVERLAY)
		if ((buf[15] == 0x77 && buf[14] == 0x88)) {
			TPD_INFO("%s: find alg overlay section in index %d\n",
				__func__, i);
			/* record index of alg overlay section */
			allovlidx |= 1 << i;
			g_alg_idx_t = i;
			g_has_alg_overlay = true;
			continue;
		}

# endif
		/*TPD_INFO("%s, [%d] SRAM addr = %08X\n", __func__, i, g_zf_info_arr[i].cfg_addr);
		TPD_INFO("%s, [%d] fw_addr = %04X!\n", __func__, i, g_zf_info_arr[i].fw_addr);
		TPD_INFO("%s, [%d] write_size = %d!\n", __func__, i, g_zf_info_arr[i].write_size); */
		if (i == 0)
			continue;
		if (dsram_base > g_zf_info_arr[i].cfg_addr) {
			dsram_base = g_zf_info_arr[i].cfg_addr;
			i_min = i;
		} else if (dsram_max < g_zf_info_arr[i].cfg_addr) {
			dsram_max = g_zf_info_arr[i].cfg_addr;
			i_max = i;
		}
	}
	for (i = 0; i < 4; i++)
		hx83102j_nf_sram_min[i] = g_zf_info_arr[i_min].sram_addr[i];
	hx83102j_nf_cfg_sz = (dsram_max - dsram_base) + g_zf_info_arr[i_max].write_size;

	if (hx83102j_nf_cfg_sz % 4 > 0)
		hx83102j_nf_cfg_sz = hx83102j_nf_cfg_sz + (hx83102j_nf_cfg_sz % 4);


	TPD_INFO("%s, hx83102j_nf_cfg_sz = %d!, dsram_base = %X, dsram_max = %X\n", __func__, hx83102j_nf_cfg_sz, dsram_base, dsram_max);

	if (hx83102j_nf_fw_buf == NULL)
		hx83102j_nf_fw_buf = kzalloc(sizeof(unsigned char) * HX64K, GFP_KERNEL);

	for (i = 1; i < part_num; i++) {
		if (g_zf_info_arr[i].cfg_addr % 4 != 0)
			g_zf_info_arr[i].cfg_addr = g_zf_info_arr[i].cfg_addr - (g_zf_info_arr[i].cfg_addr % 4);

#if defined(HX_CODE_OVERLAY)|| defined(HX_ALG_OVERLAY)
		/*overlay section*/
		if (allovlidx & (1 << i)) {
			TPD_INFO("%s: skip overlay section %d\n", __func__, i);
			continue;
		}
#endif
		memcpy(hx83102j_nf_fw_buf + (g_zf_info_arr[i].cfg_addr - dsram_base)
			, (unsigned char *)&fw_entry->data[g_zf_info_arr[i].fw_addr], g_zf_info_arr[i].write_size);
	}
	hx83102j_nf_cfg_crc = himax_mcu_Calculate_crc_with_AP(hx83102j_nf_fw_buf, 0, hx83102j_nf_cfg_sz);
	TPD_INFO("hx83102j_nf_cfg_crc = 0x%x\n", hx83102j_nf_cfg_crc);
	return true;
}

static int hx83102j_nf_zf_part_info(const struct firmware *fw_entry)
{
	bool ret = false;
	bool flag_1k_header = false;
	int retry = 0;
	int crc = -1;
#if defined(HX_CODE_OVERLAY)
	uint8_t tmp_addr[4] = {0xFC, 0x7F, 0x00, 0x10};
	uint8_t send_data[4] = {0};
	uint8_t recv_data[4] = {0};
	uint8_t ovl_idx_t = 0;
	uint8_t gesture_addr[4] = {0};
	uint8_t gesture_data[4] = {0};
#endif

	if (!hx_parse_bin_cfg_data(fw_entry))
		TPD_INFO("%s, Parse cfg from bin failed\n", __func__);
	g_core_fp.fp_sys_reset();
	himax_sense_off();
	/*getnstimeofday(&timeStart);*/
	/* first 64K */

	/*0. check 1k header*/
	if (fw_entry->data[0x00] == 0x00
		&& fw_entry->data[0x01] == 0x00
		&& fw_entry->data[0x02] == 0x00
		&& fw_entry->data[0x03] == 0x00
		&& fw_entry->data[0x04] == 0x00
		&& fw_entry->data[0x05] == 0x00
		&& fw_entry->data[0x06] == 0x00
		&& fw_entry->data[0x07] == 0x00
		&& fw_entry->data[0x0E] == 0x87)
		flag_1k_header = true;
	else
		flag_1k_header = false;

	if (flag_1k_header == true)
		crc = himax_sram_write_crc_check(fw_entry, pzf_op->data_sram_start_addr, HX1K,  g_zf_info_arr[0].write_size);
	else
		crc = himax_sram_write_crc_check(fw_entry, pzf_op->data_sram_start_addr, 0,  g_zf_info_arr[0].write_size);

	ret = (crc == 0) ? true : false;
	if (crc != 0)
		TPD_INFO("size=%d crc Failed! crc = %X",  g_zf_info_arr[0].write_size, crc);
	do {
		himax_mcu_register_write(hx83102j_nf_sram_min, hx83102j_nf_cfg_sz, hx83102j_nf_fw_buf, 0);
		crc = himax_hw_check_crc(hx83102j_nf_sram_min, hx83102j_nf_cfg_sz);
		if (crc != hx83102j_nf_cfg_crc)
			TPD_INFO("Config crc FAIL, HW crc = %X, SW crc = %X, retry time = %d", crc, hx83102j_nf_cfg_crc, retry);
		retry++;
	} while (!ret && retry < 10);

#if defined(HX_CODE_OVERLAY)
	if (private_ts->gesture_enable) {
			gesture_addr[3] = 0x10;
			gesture_addr[2] = 0x00;
			gesture_addr[1] = 0x7F;
			gesture_addr[0] = 0x10;
			gesture_data[3] = 0xA5;
			gesture_data[2] = 0x5A;
			gesture_data[1] = 0xA5;
			gesture_data[0] = 0x5A;
			himax_register_write(gesture_addr, 4, gesture_data, false);
		}
#endif
#if defined(HX_CODE_OVERLAY)
		/* ovl_idx[0] - sorting */
		/* ovl_idx[1] - gesture */
		/* ovl_idx[2] - border  */

if (in_self_test == 1) {
			ovl_idx_t = ovl_idx[0];
			send_data[0] = OVL_SORTING_REPLY;
	} else {
			ovl_idx_t = ovl_idx[2];
			send_data[0] = OVL_BORDER_REPLY;
	}


		if (g_zf_info_arr[ovl_idx_t].write_size == 0) {
			send_data[0] = OVL_FAULT;
			TPD_INFO("%s, WRONG overlay section, plese check FW!\n",
					__func__);
		} else {
			if (himax_sram_write_crc_check(fw_entry,
			g_zf_info_arr[ovl_idx_t].sram_addr,
			g_zf_info_arr[ovl_idx_t].fw_addr,
			g_zf_info_arr[ovl_idx_t].write_size) != 0) {
				send_data[0] = OVL_FAULT;
				TPD_INFO("%s, Overlay HW crc FAIL\n", __func__);
			} else {
				TPD_INFO("%s, Overlay HW crc PASS\n", __func__);
			}
		}

		retry = 0;
		do {
			himax_register_write(tmp_addr, 4, send_data, false);
			himax_register_read(tmp_addr, 4, recv_data, false);
			retry++;
		} while ((send_data[3] != recv_data[3]
				|| send_data[2] != recv_data[2]
				|| send_data[1] != recv_data[1]
				|| send_data[0] != recv_data[0])
				&& retry < HIMAX_REG_RETRY_TIMES);
#endif
#if defined(HX_ALG_OVERLAY)
	if(g_has_alg_overlay)
		alg_overlay(g_alg_idx_t, g_zf_info_arr, fw_entry);
#endif
	g_core_fp.fp_clean_sram_0f(pzf_op->data_mode_switch, 4, 0);
	kfree(g_zf_info_arr);

	return 0;
}

void himax_mcu_firmware_update_0f(const struct firmware *fw_entry)
{
	int retry = 0;
	int crc = -1;
	int ret = 0;
	uint8_t temp_addr[4];
	uint8_t temp_data[4];
	struct firmware *request_fw_headfile = NULL;
	const struct firmware *tmp_fw_entry = NULL;
	bool reload = false;
	TPD_DETAIL("%s, Entering \n", __func__);

fw_reload:
	if (fw_entry == NULL || reload) {
		TPD_INFO("Get FW from headfile\n");
		if (request_fw_headfile == NULL) {
			request_fw_headfile = kzalloc(sizeof(struct firmware), GFP_KERNEL);
		}
		if(request_fw_headfile == NULL) {
			TPD_INFO("%s kzalloc failed!\n", __func__);
			return;
		}
		if (g_chip_info->g_fw_sta) {
			TPD_INFO("request firmware failed, get from g_fw_buf\n");
			request_fw_headfile->size = g_chip_info->g_fw_len;
			request_fw_headfile->data = g_chip_info->g_fw_buf;
			tmp_fw_entry = request_fw_headfile;

		} else {
			TPD_INFO("request firmware failed, get from headfile\n");
			if(g_chip_info->p_firmware_headfile->firmware_data) {
				request_fw_headfile->size = g_chip_info->p_firmware_headfile->firmware_size;
				request_fw_headfile->data = g_chip_info->p_firmware_headfile->firmware_data;
				tmp_fw_entry = request_fw_headfile;
				g_chip_info->using_headfile = true;
			} else {
				TPD_INFO("firmware_data is NULL! exit firmware update!\n");
				if(request_fw_headfile != NULL) {
					kfree(request_fw_headfile);
					request_fw_headfile = NULL;
				}
				return;
			}
		}
	} else {
		tmp_fw_entry = fw_entry;
	}

#if defined(HX_BOOT_UPGRADE)
	ret = request_firmware(&tmp_fw_entry, g_fw_boot_upgrade_name, private_ts->dev);
	TPD_INFO("%s: ---request file %s finished\n", __func__, g_fw_boot_upgrade_name);
	if (ret < 0) {
		TPD_INFO("%s,%d: error code = %d\n", __func__, __LINE__, ret);
	}
#endif

	g_fw_entry = tmp_fw_entry;

	hx_mcu_bin_desc_get((unsigned char *)g_fw_entry->data, HX1K);

	if ((int)tmp_fw_entry->size > HX64K) {
		ret = hx83102j_nf_zf_part_info(tmp_fw_entry);
	} else {
		g_core_fp.fp_sys_reset();
		himax_sense_off();
		/* first 64K */
		do {
			g_core_fp.fp_write_sram_0f(tmp_fw_entry, pzf_op->data_sram_start_addr, 0, HX64K);
			crc = himax_hw_check_crc(pzf_op->data_sram_start_addr, HX64K);

			temp_addr[3] = 0x08;
			temp_addr[2] = 0x00;
			temp_addr[1] = 0xBF;
			temp_addr[0] = 0xFC;
			himax_register_read(temp_addr, 4, temp_data, false);

			TPD_DETAIL("%s, 64K LAST 4 BYTES: data[3] = 0x%02X, data[2] = 0x%02X, data[1] = 0x%02X, data[0] = 0x%02X\n",
							__func__, temp_data[3], temp_data[2], temp_data[1], temp_data[0]);

			if (crc == 0) {
				TPD_DETAIL("%s, HW crc OK in %d time \n", __func__, retry);
				break;
			} else {
				TPD_INFO("%s, HW crc FAIL in %d time !\n", __func__, retry);
			}
			retry++;
		} while (((temp_data[3] == 0 && temp_data[2] == 0 && temp_data[1] == 0 && temp_data[0] == 0 &&
						retry < 80) || (crc != 0 && retry < 30)) && !(g_chip_info->using_headfile && retry < 3));

		if (crc != 0) {
			TPD_INFO("Last time crc Fail!\n");
			if(reload) {
				return;
			} else {
				reload = true;
				goto fw_reload;
			}
		}

		/* if g_poweronof!= 1, it will be clean mode! */
		/*config and setting */
		/*config info*/
		if (g_poweronof == 1) {
			retry = 0;
			do {
				g_core_fp.fp_write_sram_0f(tmp_fw_entry, pzf_op->data_cfg_info, 0xC000, 128);
				crc = himax_hw_check_crc(pzf_op->data_cfg_info, 128);
				if (crc == 0) {
					TPD_DETAIL("%s, config info ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, config info fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (g_chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("config info crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			g_core_fp.fp_clean_sram_0f(pzf_op->data_cfg_info, 128, 2);
		}
		/*FW config*/
		if (g_poweronof == 1) {
			retry = 0;
			do {
				g_core_fp.fp_write_sram_0f(tmp_fw_entry, pzf_op->data_fw_cfg_p1, 0xC100, 528);
				crc = himax_hw_check_crc(pzf_op->data_fw_cfg_p1, 528);
				if (crc == 0) {
					TPD_DETAIL("%s, 1 FW config ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, 1 FW config fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (g_chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("1 FW config crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			g_core_fp.fp_clean_sram_0f(pzf_op->data_fw_cfg_p1, 528, 1);
		}

		if (g_poweronof == 1) {
			retry = 0;
			do {
				g_core_fp.fp_write_sram_0f(tmp_fw_entry, pzf_op->data_fw_cfg_p3, 0xCA00, 128);
				crc = himax_hw_check_crc(pzf_op->data_fw_cfg_p3, 128);
				if (crc == 0) {
					TPD_DETAIL("%s, 3 FW config ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, 3 FW config fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (g_chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("3 FW config crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			g_core_fp.fp_clean_sram_0f(pzf_op->data_fw_cfg_p3, 128, 1);
		}

		/*ADC config*/
		if (g_poweronof == 1) {
			retry = 0;
			do {
				g_core_fp.fp_write_sram_0f(tmp_fw_entry, pzf_op->data_adc_cfg_1, 0xD640, 1200);
				crc = himax_hw_check_crc(pzf_op->data_adc_cfg_1, 1200);
				if (crc == 0) {
					TPD_DETAIL("%s, 1 ADC config ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, 1 ADC config fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (g_chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("1 ADC config crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			g_core_fp.fp_clean_sram_0f(pzf_op->data_adc_cfg_1, 1200, 2);
		}

		if (g_poweronof == 1) {
			retry = 0;
			do {
				g_core_fp.fp_write_sram_0f(tmp_fw_entry, pzf_op->data_adc_cfg_2, 0xD320, 800);
				crc = himax_hw_check_crc(pzf_op->data_adc_cfg_2, 800);
				if (crc == 0) {
					TPD_DETAIL("%s, 2 ADC config ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, 2 ADC config fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (g_chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("2 ADC config crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			g_core_fp.fp_clean_sram_0f(pzf_op->data_adc_cfg_2, 800, 2);
		}

		/*mapping table*/
		if (g_poweronof == 1) {
			retry = 0;
			do {
				g_core_fp.fp_write_sram_0f(tmp_fw_entry, pzf_op->data_map_table, 0xE000, 1536);
				crc = himax_hw_check_crc(pzf_op->data_map_table, 1536);
				if (crc == 0) {
					TPD_DETAIL("%s, mapping table ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, mapping table fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (g_chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("mapping table crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			g_core_fp.fp_clean_sram_0f(pzf_op->data_map_table, 1536, 2);
		}
	}

	g_chip_info->first_download_finished = true;

	/* switch mode*/
	if (g_poweronof == 1) {
		g_core_fp.fp_write_sram_0f(tmp_fw_entry, pzf_op->data_mode_switch, 0xC30C, 4);
	} else {
		g_core_fp.fp_clean_sram_0f(pzf_op->data_mode_switch, 4, 2);
	}

	hx83102j_fw_check(private_ts->chip_data, &private_ts->resolution_info, &private_ts->panel_data);

	if (request_fw_headfile != NULL) {
		kfree(request_fw_headfile);
		request_fw_headfile = NULL;
	}
	TPD_DETAIL("%s, END \n", __func__);
}
int hx_0f_op_file_dirly(char *file_name)
{
	int err = NO_ERR;
	const struct firmware *fw_entry = NULL;


	TPD_INFO("%s, Entering \n", __func__);
	TPD_INFO("file name = %s\n", file_name);
	if (private_ts->fw_update_app_support) {
		err = request_firmware_select(&fw_entry, file_name, private_ts->dev);
	} else {
		err = request_firmware(&fw_entry, file_name, private_ts->dev);
	}
	if (err < 0) {
		TPD_INFO("%s, fail in line%d error code=%d, file maybe fail\n", __func__, __LINE__, err);
		return err;
	}


	if(g_f_0f_updat == 1) {
		TPD_INFO("%s:[Warning]Other thread is updating now!\n", __func__);
		release_firmware(fw_entry);
		err = -1;
		return err;
	} else {
		TPD_INFO("%s:Entering Update Flow!\n", __func__);
		g_f_0f_updat = 1;
	}

	hx83102j_enable_interrupt(g_chip_info, false);

	/* trigger reset */
#ifdef HX_RST_PIN_FUNC
	hx83102j_resetgpio_set(g_chip_info->hw_res, false);
	hx83102j_resetgpio_set(g_chip_info->hw_res, true);
#else
	g_core_fp.fp_sys_reset();
#endif
	g_core_fp.fp_firmware_update_0f(fw_entry);
	release_firmware(fw_entry);

	g_f_0f_updat = 0;
	TPD_INFO("%s, END \n", __func__);
	return err;
}
int himax_mcu_0f_operation_dirly(void)
{
	int err = NO_ERR;

	TPD_DETAIL("%s, Entering \n", __func__);

	if(g_f_0f_updat == 1) {
		TPD_INFO("%s:[Warning]Other thread is updating now!\n", __func__);
		err = -1;
		return err;
	} else {
		TPD_DETAIL("%s:Entering Update Flow!\n", __func__);
		g_f_0f_updat = 1;
	}


	g_core_fp.fp_firmware_update_0f(NULL);


	g_f_0f_updat = 0;
	TPD_DETAIL("%s, END \n", __func__);
	return err;
}

int himax_mcu_0f_operation_test_dirly(char *fw_name)
{
	int err = NO_ERR;
	const struct firmware *fw_entry = NULL;

	TPD_DETAIL("%s, Entering \n", __func__);
	TPD_DETAIL("file name = %s\n", fw_name);
	TPD_INFO("Request TP firmware.\n");
	err = request_firmware(&fw_entry, fw_name, private_ts->dev);
	if (err < 0) {
		TPD_INFO("%s, fail in line%d error code=%d, file maybe fail\n", __func__, __LINE__, err);
		if (fw_entry != NULL) {
			release_firmware(fw_entry);
			fw_entry = NULL;
		}
		return err;
	}

	hx83102j_enable_interrupt(g_chip_info, false);

	g_core_fp.fp_firmware_update_0f(fw_entry);
	release_firmware(fw_entry);

	TPD_DETAIL("%s, END \n", __func__);
	return err;
}

void himax_mcu_0f_operation(struct work_struct *work)
{
	TPD_INFO("file name = %s\n", private_ts->panel_data.fw_name);

	if (g_f_0f_updat == 1) {
		TPD_INFO("%s:[Warning]Other thread is updating now!\n", __func__);
		return;
	} else {
		TPD_INFO("%s:Entering Update Flow!\n", __func__);
		g_f_0f_updat = 1;
	}

	hx83102j_enable_interrupt(g_chip_info, false);
	msleep(1);
	g_core_fp.fp_firmware_update_0f(NULL);

	g_core_fp.fp_reload_disable(0);
	msleep(10);
	himax_read_FW_ver();
	msleep(10);
	himax_sense_on(0x00);
	msleep(10);
	TPD_INFO("%s:End \n", __func__);

#ifdef CONFIG_OPPO_TP_APK
	if(g_chip_info->debug_mode_sta) {
		if(private_ts->apk_op && private_ts->apk_op->apk_debug_set) {
			private_ts->apk_op->apk_debug_set(private_ts->chip_data, true);
		}
	}
#endif

	hx83102j_enable_interrupt(g_chip_info, true);

	g_f_0f_updat = 0;
	TPD_INFO("%s, END \n", __func__);
	return;
}

#ifdef HX_0F_DEBUG
void himax_mcu_read_sram_0f(const struct firmware *fw_entry, uint8_t *addr, int start_index, int read_len)
{
	int total_read_times = 0;
	int max_bus_size = MAX_TRANS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	int address = 0;
	int i = 0, j = 0;
	int not_same = 0;

	uint8_t tmp_addr[4];
	uint8_t *temp_info_data;
	int *not_same_buff;

	TPD_INFO("%s, Entering \n", __func__);

	himax_burst_enable(1);

	total_size = read_len;

	total_size_temp = read_len;
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	if (read_len > MAX_RECVS_SZ) {
		max_bus_size = MAX_RECVS_SZ;
	} else {
		max_bus_size = read_len;
	}
#else
	if (read_len > 2048) {
		max_bus_size = 2048;
	} else {
		max_bus_size = read_len;
	}
#endif
	temp_info_data = kzalloc(sizeof(uint8_t) * total_size, GFP_KERNEL);
	not_same_buff = kzalloc(sizeof(int) * total_size, GFP_KERNEL);


	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];

	TPD_INFO("%s, total size=%d\n", __func__, total_size);

	himax_burst_enable(1);

	if (total_size % max_bus_size == 0) {
		total_read_times = total_size / max_bus_size;
	} else {
		total_read_times = total_size / max_bus_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		if (total_size_temp >= max_bus_size) {
			himax_register_read(tmp_addr, max_bus_size, &temp_info_data[i * max_bus_size], false);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			himax_register_read(tmp_addr, total_size_temp % max_bus_size, &temp_info_data[i * max_bus_size], false);
		}

		address = ((i + 1) * max_bus_size);
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);
		if (tmp_addr[0] < addr[0]) {
			tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF) + 1;
		} else {
			tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF);
		}

		msleep(10);
	}
	TPD_INFO("%s, READ Start \n", __func__);
	TPD_INFO("%s, start_index = %d \n", __func__, start_index);
	j = start_index;
	for (i = 0; i < read_len; i++, j++) {
		if (g_chip_info->g_fw_buf[j] != temp_info_data[i]) {
			not_same++;
			not_same_buff[i] = 1;
		}

		TPD_INFO("0x%2.2X, ", temp_info_data[i]);

		if (i > 0 && i % 16 == 15) {
			printk("\n");
		}
	}
	TPD_INFO("%s, READ END \n", __func__);
	TPD_INFO("%s, Not Same count=%d\n", __func__, not_same);
	if (not_same != 0) {
		j = start_index;
		for (i = 0; i < read_len; i++, j++) {
			if (not_same_buff[i] == 1) {
				TPD_INFO("bin = [%d] 0x%2.2X\n", i, g_chip_info->g_fw_buf[j]);
			}
		}
		for (i = 0; i < read_len; i++, j++) {
			if (not_same_buff[i] == 1) {
				TPD_INFO("sram = [%d] 0x%2.2X \n", i, temp_info_data[i]);
			}
		}
	}
	TPD_INFO("%s, READ END \n", __func__);
	TPD_INFO("%s, Not Same count=%d\n", __func__, not_same);
	TPD_INFO("%s, END \n", __func__);

	kfree(not_same_buff);
	kfree(temp_info_data);
}

void himax_mcu_read_all_sram(uint8_t *addr, int read_len)
{
	int total_read_times = 0;
	int max_bus_size = MAX_RECVS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	int address = 0;
	int i = 0;
	/*
	struct file *fn;
	struct filename *vts_name;
	*/

	uint8_t tmp_addr[4];
	uint8_t *temp_info_data;

	TPD_INFO("%s, Entering \n", __func__);

	himax_burst_enable(1);

	total_size = read_len;

	total_size_temp = read_len;

	temp_info_data = kzalloc(sizeof(uint8_t) * total_size, GFP_KERNEL);


	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];

	TPD_INFO("%s, total size=%d\n", __func__, total_size);

	if (total_size % max_bus_size == 0) {
		total_read_times = total_size / max_bus_size;
	} else {
		total_read_times = total_size / max_bus_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		if (total_size_temp >= max_bus_size) {
			himax_register_read(tmp_addr, max_bus_size, &temp_info_data[i * max_bus_size], false);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			himax_register_read(tmp_addr, total_size_temp % max_bus_size, &temp_info_data[i * max_bus_size], false);
		}

		address = ((i + 1) * max_bus_size);
		tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF);
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);

		msleep(10);
	}
	/*for (i = 0;i < read_len;i++) {
		TPD_INFO("0x%2.2X, ", temp_info_data[i]);

		if (i > 0 && i%16 == 15)
			printk("\n");
	}*/

	/* need modify
	TPD_INFO("Now Write File start!\n");
	vts_name = getname_kernel("/sdcard/dump_dsram.txt");
	fn = file_open_name(vts_name, O_CREAT | O_WRONLY, 0);
	if (!IS_ERR (fn)) {
		TPD_INFO("%s create file and ready to write\n", __func__);
		fn->f_op->write (fn, temp_info_data, read_len*sizeof (uint8_t), &fn->f_pos);
		filp_close (fn, NULL);
	}
	TPD_INFO("Now Write File End!\n");
	*/

	TPD_INFO("%s, END \n", __func__);

	kfree(temp_info_data);
}

void himax_mcu_firmware_read_0f(const struct firmware *fw_entry, int type)
{
	uint8_t tmp_addr[4] = {0};

	TPD_INFO("%s, Entering \n", __func__);
	if (type == 0) { /* first 64K */
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_sram_start_addr, 0, HX64K);
		g_core_fp.fp_read_all_sram(tmp_addr, 0xC000);
	} else { /*last 16k*/
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_cfg_info, 0xC000, 132);
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_fw_cfg, 0xC0FE, 512);
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_adc_cfg_1, 0xD000, 376);
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_adc_cfg_2, 0xD178, 376);
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_adc_cfg_3, 0xD000, 376);
		g_core_fp.fp_read_all_sram(pzf_op->data_sram_clean, HX_32K_SZ);
	}
	TPD_INFO("%s, END \n", __func__);
}

void himax_mcu_0f_operation_check(int type)
{
	TPD_INFO("%s, Entering \n", __func__);

	g_core_fp.fp_firmware_read_0f(NULL, type);

	TPD_INFO("%s, END \n", __func__);
	return;
}
#endif


int hx_0f_init(void)
{
	pzf_op = kzalloc(sizeof(struct zf_operation), GFP_KERNEL);

	g_core_fp.fp_reload_disable = hx_dis_rload_0f;
	g_core_fp.fp_sys_reset = himax_mcu_sys_reset;
	g_core_fp.fp_clean_sram_0f = himax_mcu_clean_sram_0f;
	g_core_fp.fp_write_sram_0f = himax_mcu_write_sram_0f;
	g_core_fp.fp_firmware_update_0f = himax_mcu_firmware_update_0f;
	g_core_fp.fp_0f_operation = himax_mcu_0f_operation;
	g_core_fp.fp_0f_operation_dirly = himax_mcu_0f_operation_dirly;
	g_core_fp.fp_0f_op_file_dirly = hx_0f_op_file_dirly;
#ifdef HX_0F_DEBUG
	g_core_fp.fp_read_sram_0f = himax_mcu_read_sram_0f;
	g_core_fp.fp_read_all_sram = himax_mcu_read_all_sram;
	g_core_fp.fp_firmware_read_0f = himax_mcu_firmware_read_0f;
	g_core_fp.fp_0f_operation_check = himax_mcu_0f_operation_check;
#endif

	himax_in_parse_assign_cmd(ZF_ADDR_DIS_FLASH_RELOAD, pzf_op->addr_dis_flash_reload, sizeof(pzf_op->addr_dis_flash_reload));
	himax_in_parse_assign_cmd(ZF_DATA_DIS_FLASH_RELOAD, pzf_op->data_dis_flash_reload, sizeof(pzf_op->data_dis_flash_reload));
	himax_in_parse_assign_cmd(ZF_ADDR_SYSTEM_RESET, pzf_op->addr_system_reset, sizeof(pzf_op->addr_system_reset));
	himax_in_parse_assign_cmd(ZF_DATA_SYSTEM_RESET, pzf_op->data_system_reset, sizeof(pzf_op->data_system_reset));
	himax_in_parse_assign_cmd(ZF_DATA_SRAM_START_ADDR, pzf_op->data_sram_start_addr, sizeof(pzf_op->data_sram_start_addr));
	himax_in_parse_assign_cmd(ZF_DATA_SRAM_CLEAN, pzf_op->data_sram_clean, sizeof(pzf_op->data_sram_clean));
	himax_in_parse_assign_cmd(ZF_DATA_CFG_INFO, pzf_op->data_cfg_info, sizeof(pzf_op->data_cfg_info));
	himax_in_parse_assign_cmd(ZF_DATA_FW_CFG_P1, pzf_op->data_fw_cfg_p1, sizeof(pzf_op->data_fw_cfg_p1));
	himax_in_parse_assign_cmd(ZF_DATA_FW_CFG_P2, pzf_op->data_fw_cfg_p2, sizeof(pzf_op->data_fw_cfg_p2));
	himax_in_parse_assign_cmd(ZF_DATA_FW_CFG_P3, pzf_op->data_fw_cfg_p3, sizeof(pzf_op->data_fw_cfg_p3));
	himax_in_parse_assign_cmd(ZF_DATA_ADC_CFG_1, pzf_op->data_adc_cfg_1, sizeof(pzf_op->data_adc_cfg_1));
	himax_in_parse_assign_cmd(ZF_DATA_ADC_CFG_2, pzf_op->data_adc_cfg_2, sizeof(pzf_op->data_adc_cfg_2));
	himax_in_parse_assign_cmd(ZF_DATA_ADC_CFG_3, pzf_op->data_adc_cfg_3, sizeof(pzf_op->data_adc_cfg_3));
	himax_in_parse_assign_cmd(ZF_DATA_MAP_TABLE, pzf_op->data_map_table, sizeof(pzf_op->data_map_table));
	himax_in_parse_assign_cmd(ZF_DATA_MODE_SWITCH, pzf_op->data_mode_switch, sizeof(pzf_op->data_mode_switch));

#if defined(HX_ALG_OVERLAY)
	himax_in_parse_assign_cmd(DRIVER_ADDR_FW_DEFINE_2ND_FLASH_RELOAD, pzf_op->addr_fw_define_2nd_flash_reload, sizeof(pzf_op->addr_fw_define_2nd_flash_reload));
	himax_in_parse_assign_cmd(FW_ADDR_RAW_OUT_SEL, pzf_op->addr_raw_out_sel, sizeof(pzf_op->addr_raw_out_sel));
	himax_in_parse_assign_cmd(FW_ADDR_SET_FRAME_ADDR, pzf_op->addr_set_frame_addr, sizeof(pzf_op->addr_set_frame_addr));
	himax_in_parse_assign_cmd(FW_DATA_CLEAR, pzf_op->data_clear, sizeof(pzf_op->data_clear));
	himax_in_parse_assign_cmd(FW_ADDR_SORTING_MODE_EN,
		pzf_op->addr_sorting_mode_en,
		sizeof(pzf_op->addr_sorting_mode_en));
	himax_in_parse_assign_cmd(FLASH_ADDR_SPI200_DATA,
		pzf_op->addr_spi200_data,
		sizeof(pzf_op->addr_spi200_data));
	himax_in_parse_assign_cmd(IC_ADR_AHB_ADDR_BYTE_0,
		pzf_op->addr_ahb_addr_byte_0,
		sizeof(pzf_op->addr_ahb_addr_byte_0));
	himax_in_parse_assign_cmd(IC_ADR_AHB_ACCESS_DIRECTION,
		pzf_op->addr_ahb_access_direction,
		sizeof(pzf_op->addr_ahb_access_direction));
	himax_in_parse_assign_cmd(IC_CMD_AHB_ACCESS_DIRECTION_READ,
		pzf_op->data_ahb_access_direction_read,
		sizeof(pzf_op->data_ahb_access_direction_read));
	himax_in_parse_assign_cmd(IC_ADR_AHB_RDATA_BYTE_0,
		pzf_op->addr_ahb_rdata_byte_0,
		sizeof(pzf_op->addr_ahb_rdata_byte_0));


#endif

	return 0;
}

#endif

bool himax_ic_package_check(void)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[8] = {0};
	uint8_t ret_data = 0x00;
	int i = 0;

#ifdef HX_RST_PIN_FUNC
	hx83102j_resetgpio_set(g_chip_info->hw_res, true);
	hx83102j_resetgpio_set(g_chip_info->hw_res, false);
	hx83102j_resetgpio_set(g_chip_info->hw_res, true);
#else
	himax_mcu_sys_reset();
#endif

	himax_sense_off();


	for (i = 0; i < 5; i++) {
		/* Product ID */
		/* Touch*/
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xD0;
		himax_register_read(tmp_addr, 8, tmp_data, false);


		TPD_INFO("%s: Read driver IC ID: tmp_data[0]=0x%02X, tmp_data[1]=0x%02X, tmp_data[2]=0x%02X, tmp_data[3]=0x%02X \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);


		/*if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x11) && ((tmp_data[1] == 0x2f) || (tmp_data[1] == 0x2f)))*/
		if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x10) && (tmp_data[1] == 0x29)) {
			ic_type = HX_83102J_SERIES_PWON;
			ic_checksum = HX_TP_BIN_CHECKSUM_CRC;
			hx_0f_init();

			fw_ver_maj_flash_addr   = 59397;
			fw_ver_maj_flash_leng   = 1;
			fw_ver_min_flash_addr   = 59398;
			fw_ver_min_flash_leng   = 1;
			cfg_ver_maj_flash_addr = 59648;
			cfg_ver_maj_flash_leng = 1;
			cfg_ver_min_flash_addr = 59649;
			cfg_ver_min_flash_leng = 1;
			cid_ver_maj_flash_addr = 59394;
			cid_ver_maj_flash_leng = 1;
			cid_ver_min_flash_addr = 59395;
			cid_ver_min_flash_leng = 1;

#ifdef HX_AUTO_UPDATE_FW
			g_i_FW_VER = i_CTPM_FW[fw_ver_maj_flash_addr] << 8 | i_CTPM_FW[fw_ver_min_flash_addr];
			g_i_CFG_VER = i_CTPM_FW[cfg_ver_maj_flash_addr] << 8 | i_CTPM_FW[cfg_ver_min_flash_addr];
			g_i_CID_MAJ = i_CTPM_FW[cid_ver_maj_flash_addr];
			g_i_CID_MIN = i_CTPM_FW[cid_ver_min_flash_addr];
#endif
			TPD_INFO("Himax IC package 83102j_in\n");
			ret_data = true;
			break;
		} else {
			ret_data = false;
			TPD_INFO("%s:Read driver ID register Fail:\n", __func__);
		}
	}

	return true;
}


void himax_power_on_init(void)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	TPD_INFO("%s\n", __func__);

	/*RawOut select initial*/
	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x02;
	tmp_addr[1] = 0x04;
	tmp_addr[0] = 0xF4;

	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	himax_register_write(tmp_addr, 4, tmp_data, false);

	/*DSRAM func initial*/
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x07;
	tmp_addr[0] = 0xFC;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	himax_register_write(tmp_addr, 4, tmp_data, false);
#ifndef HX_ZERO_FLASH
	himax_sense_on(0x00);
#endif
}


static void himax_read_FW_ver(void)
{
	uint8_t cmd[4];
	uint8_t data[64];
	uint8_t data2[64];
	int retry = 20;
	int reload_status = 0;
	u8 ver_len = 0;
	char dev_version[MAX_DEVICE_VERSION_LENGTH] = {0};

	himax_sense_on(0);

	while(reload_status == 0) {
		cmd[3] = 0x10;  /*oppo fw id bin address : 0xc014   , 49172    Tp ic address : 0x 10007014*/
		cmd[2] = 0x00;
		cmd[1] = 0x7f;
		cmd[0] = 0x00;
		himax_register_read(cmd, 4, data, false);

		cmd[3] = 0x10;
		cmd[2] = 0x00;
		cmd[1] = 0x72;
		cmd[0] = 0xc0;
		himax_register_read(cmd, 4, data2, false);

		if ((data[1] == 0x3A && data[0] == 0xA3) || (data2[1] == 0x72 && data2[0] == 0xc0)) {
			TPD_INFO("reload OK! \n");
			reload_status = 1;
			break;
		} else if (retry == 0) {
			TPD_INFO("reload 20 times! fail \n");
			return;
		} else {
			retry--;
			msleep(10);
			TPD_INFO("reload fail, delay 10ms retry=%d\n", retry);
		}
	}
	TPD_INFO("%s : data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n", __func__, data[0], data[1], data[2], data[3]);
	TPD_INFO("reload_status=%d\n", reload_status);

	himax_sense_off();

	/*=====================================
		Read FW version : 0x1000_7004  but 05,06 are the real addr for FW Version
	=====================================*/

	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x04;
	himax_register_read(cmd, 4, data, false);


	TPD_INFO("PANEL_VER : %X \n", data[0]);
	TPD_INFO("FW_VER : %X \n", data[1] << 8 | data[2]);

	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x84;
	himax_register_read(cmd, 4, data, false);

	TPD_INFO("CFG_VER : %X \n", data[2] << 8 | data[3]);
	TPD_INFO("TOUCH_VER : %X \n", data[2]);
	TPD_INFO("DISPLAY_VER : %X \n", data[3]);
	snprintf(dev_version, MAX_DEVICE_VERSION_LENGTH, "%02X", data[2]);
	if (private_ts->panel_data.manufacture_info.version) {
		if (private_ts->panel_data.vid_len == 0) {
			strlcpy(&(private_ts->panel_data.manufacture_info.version[12]), dev_version, 3);
		} else {
			ver_len = private_ts->panel_data.vid_len;
			if (ver_len > MAX_DEVICE_VERSION_LENGTH - 4) {
				ver_len = MAX_DEVICE_VERSION_LENGTH - 4;
			}

			strlcpy(&private_ts->panel_data.manufacture_info.version[ver_len],
					dev_version, MAX_DEVICE_VERSION_LENGTH - ver_len);
		}
	}
	TPD_INFO("manufacture_info.version: %s\n", private_ts->panel_data.manufacture_info.version);

	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x00;
	himax_register_read(cmd, 4, data, false);
	TPD_INFO("CID_VER : %X \n", (data[2] << 8 | data[3]));
	return;
}


void himax_read_OPPO_FW_ver(struct chip_data_hx83102j *chip_info)
{
	uint8_t cmd[4];
	uint8_t data[4];
	uint32_t touch_ver = 0;
	u8 ver_len = 0;
	char dev_version[MAX_DEVICE_VERSION_LENGTH] = {0};

	cmd[3] = 0x10;  /*oppo fw id bin address : 0xc014    Tp ic address : 0x 10007014*/
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x14;
	himax_register_read(cmd, 4, data, false);
	if (data[0] == 0xBD && data[1] == 0x12)
		isbd12proj = true;
	if (data[0] == 0xEA && data[1] == 0x00 && (data[2] & 0xF0) == 0x60)
		isea006proj = true;
	chip_info->fw_id = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
	TPD_INFO("%s : data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n", __func__, data[0], data[1], data[2], data[3]);

	cmd[3] = 0x10;  /*oppo fw id bin address : 0xc014    Tp ic address : 0x 10007014*/
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x84;
	himax_register_read(cmd, 4, data, false);
	touch_ver = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
	TPD_INFO("%s :touch_ver = 0x%08X\n", __func__, touch_ver);
	snprintf(dev_version, MAX_DEVICE_VERSION_LENGTH, "%02X", touch_ver>>8);
	if (private_ts->panel_data.manufacture_info.version) {
		if (private_ts->panel_data.vid_len == 0) {
			strlcpy(&(private_ts->panel_data.manufacture_info.version[12]), dev_version, 3);
		} else {
			ver_len = private_ts->panel_data.vid_len;
			if (ver_len > MAX_DEVICE_VERSION_LENGTH - 4) {
				ver_len = MAX_DEVICE_VERSION_LENGTH - 4;
			}
			strlcpy(&private_ts->panel_data.manufacture_info.version[ver_len],
					dev_version, MAX_DEVICE_VERSION_LENGTH - ver_len);
		}
	}
	TPD_INFO("manufacture_info.version: %s\n", private_ts->panel_data.manufacture_info.version);

	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x00;
	himax_register_read(cmd, 4, data, false);
	chip_info->fw_ver = data[2] << 8 | data[3];
	g_touch_ver = touch_ver >> 8;
	TPD_INFO("%s :fw_Ver = 0x%04X touch_ver = 0x%04X\n", __func__, chip_info->fw_ver, g_touch_ver);
	return;
}

uint32_t himax_hw_check_crc(uint8_t *start_addr, int reload_length)
{
	uint32_t result = 0;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int cnt = 0;
	int length = reload_length / 4;

	/*0x8005_0020 <= from, 0x8005_0028 <= 0x0099_length*/
	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x05;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x20;

	himax_flash_write_burst(tmp_addr, start_addr);

	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x05;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x28;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x99;
	tmp_data[1] = (length >> 8);
	tmp_data[0] = length;
	himax_flash_write_burst(tmp_addr, tmp_data);

	cnt = 0;
	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x05;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x00;
	do {
		himax_register_read(tmp_addr, 4, tmp_data, false);

		if ((tmp_data[0] & 0x01) != 0x01) {
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x05;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x18;
			himax_register_read(tmp_addr, 4, tmp_data, false);
			TPD_DETAIL("%s: tmp_data[3]=%X, tmp_data[2]=%X, tmp_data[1]=%X, tmp_data[0]=%X  \n", __func__, tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
			result = ((tmp_data[3] << 24) + (tmp_data[2] << 16) + (tmp_data[1] << 8) + tmp_data[0]);
			break;
		}
	} while (cnt++ < 100);

	return result;
}


bool himax_calculateChecksum(bool change_iref)
{
	uint8_t crc_result = 0;
	uint8_t tmp_data[4];

	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;

	crc_result = himax_hw_check_crc(tmp_data, FW_SIZE_64K);

	msleep(50);

	return !crc_result;
}

int cal_data_len(int raw_cnt_rmd, int HX_MAX_PT, int raw_cnt_max)
{
	int rawdatalen;
	if (raw_cnt_rmd != 0x00) {
		rawdatalen = 128 - ((HX_MAX_PT + raw_cnt_max + 3) * 4) - 1;
	} else {
		rawdatalen = 128 - ((HX_MAX_PT + raw_cnt_max + 2) * 4) - 1;
	}
	return rawdatalen;
}


int himax_report_data_init(int max_touch_point, int tx_num, int rx_num)
{
	if (hx_touch_data->hx_coord_buf != NULL) {
		kfree(hx_touch_data->hx_coord_buf);
	}

	if (hx_touch_data->diag_mutual != NULL) {
		kfree(hx_touch_data->diag_mutual);
	}

	hx_touch_data->event_size = 128;

	hx_touch_data->touch_all_size = 128;

	hx_touch_info_point_cnt = max_touch_point * 4;

	if ((max_touch_point % 4) == 0)
		hx_touch_info_point_cnt += (max_touch_point / 4) * 4;
	else
		hx_touch_info_point_cnt += ((max_touch_point / 4) + 1) * 4;

	hx_touch_data->raw_cnt_max = max_touch_point / 4;
	hx_touch_data->raw_cnt_rmd = max_touch_point % 4;

	if (hx_touch_data->raw_cnt_rmd != 0x00) {
		hx_touch_data->rawdata_size = cal_data_len(hx_touch_data->raw_cnt_rmd, max_touch_point, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (max_touch_point + hx_touch_data->raw_cnt_max + 2) * 4;
	} else {
		hx_touch_data->rawdata_size = cal_data_len(hx_touch_data->raw_cnt_rmd, max_touch_point, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (max_touch_point + hx_touch_data->raw_cnt_max + 1) * 4;
	}
#if defined(HX_ALG_OVERLAY)
		hx_touch_data->touch_info_size += STYLUS_INFO_SZ;
		hx_touch_data->rawdata_size -= STYLUS_INFO_SZ;
#endif
	if ((tx_num * rx_num + tx_num + rx_num) % hx_touch_data->rawdata_size == 0) {
		hx_touch_data->rawdata_frame_size = (tx_num * rx_num + tx_num + rx_num) / hx_touch_data->rawdata_size;
	} else {
		hx_touch_data->rawdata_frame_size = (tx_num * rx_num + tx_num + rx_num) / hx_touch_data->rawdata_size + 1;
	}
	TPD_INFO("%s: rawdata_frame_size = %d ", __func__, hx_touch_data->rawdata_frame_size);
	TPD_INFO("%s: max_touch_point:%d, hx_raw_cnt_max:%d, hx_raw_cnt_rmd:%d, g_hx_rawdata_size:%d, hx_touch_data->touch_info_size:%d\n",
			 __func__, max_touch_point, hx_touch_data->raw_cnt_max, hx_touch_data->raw_cnt_rmd, hx_touch_data->rawdata_size, hx_touch_data->touch_info_size);

	hx_touch_data->hx_coord_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_info_size), GFP_KERNEL);
	if (hx_touch_data->hx_coord_buf == NULL) {
		goto mem_alloc_fail;
	}

	hx_touch_data->diag_mutual = kzalloc(tx_num * rx_num * sizeof(int32_t), GFP_KERNEL);
	if (hx_touch_data->diag_mutual == NULL) {
		goto mem_alloc_fail;
	}


	hx_touch_data->hx_rawdata_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_all_size - hx_touch_data->touch_info_size), GFP_KERNEL);
	if (hx_touch_data->hx_rawdata_buf == NULL) {
		goto mem_alloc_fail;
	}


	hx_touch_data->hx_event_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->event_size), GFP_KERNEL);
	if (hx_touch_data->hx_event_buf == NULL) {
		goto mem_alloc_fail;
	}

	return NO_ERR;

mem_alloc_fail:
	kfree(hx_touch_data->hx_coord_buf);

	kfree(hx_touch_data->hx_rawdata_buf);

	kfree(hx_touch_data->hx_event_buf);


	TPD_INFO("%s: Memory allocate fail!\n", __func__);
	return MEM_ALLOC_FAIL;
}

bool himax_read_event_stack(uint8_t *buf, uint8_t length)
{
	uint8_t cmd[4];

	/*AHB_I2C Burst Read Off*/
	cmd[0] = 0x00;
	if (himax_bus_write(0x11, 1, cmd) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return 0;
	}

	himax_bus_read(0x30, length, buf);

	/*AHB_I2C Burst Read On*/
	cmd[0] = 0x01;
	if (himax_bus_write(0x11, 1, cmd) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return 0;
	}

	return 1;
}

int g_zero_event_count = 0;
int himax_ic_esd_recovery(int hx_esd_event, int hx_zero_event, int length)
{
	if (hx_esd_event == length) {
		goto checksum_fail;
		g_zero_event_count = 0;
	} else if (hx_zero_event == length) {
		g_zero_event_count++;
		TPD_INFO("[HIMAX TP MSG]: ALL Zero event is %d times.\n", g_zero_event_count);
		if (g_zero_event_count > 5) {
			g_zero_event_count = 0;
			TPD_INFO("[HIMAX TP MSG]: ESD event checked - ALL Zero.\n");
			goto checksum_fail;
		}
		goto err_workqueue_out;
	}

checksum_fail:
	return CHECKSUM_FAIL;
err_workqueue_out:
	return WORK_OUT;
}
/*Himax DB Start*/
/*
static int hx83102j_lcd_resetgpio_set(struct hw_resource *hw_res, bool on)
{
	int ret = 0;
		if (gpio_is_valid(hw_res->lcd_reset_gpio)) {
		TPD_DETAIL("Set the lcd_reset_gpio on=%d \n", on);
		ret = gpio_direction_output(hw_res->lcd_reset_gpio, on);
		if (ret) {
			TPD_INFO("Set the lcd_reset_gpio on=%d fail\n", on);
		} else {

		}
		usleep_range(5000, 5100);
		TPD_DETAIL("%s hw_res->lcd_reset_gpio = %d\n", __func__, hw_res->lcd_reset_gpio);
		gpio_free(hw_res->lcd_reset_gpio);
		TPD_INFO("%s: free lcd reset pin\n", __func__);
	}

	return ret;
}*/

/*Himax DB End*/

#ifdef HX_RST_PIN_FUNC
static int hx83102j_resetgpio_set(struct hw_resource *hw_res, bool on)
{
	int ret = 0;
	if (gpio_is_valid(hw_res->reset_gpio)) {
		TPD_DETAIL("Set the reset_gpio on=%d \n", on);
		ret = gpio_direction_output(hw_res->reset_gpio, on);
		if (ret) {
			TPD_INFO("Set the reset_gpio on=%d fail\n", on);
		} else {
			hx_reset_state = on;
		}
		msleep(RESET_TO_NORMAL_TIME);
		TPD_DETAIL("%s hw_res->reset_gpio = %d\n", __func__, hw_res->reset_gpio);
	}

	return ret;
}
#endif

void himax_esd_hw_reset(struct chip_data_hx83102j *chip_info)
{
	int ret = 0;
	int load_fw_times = 10;

	TPD_DETAIL("START_Himax TP: ESD - Reset\n");
	hx_esd_reset_activate = 1;
#ifdef HX_ZERO_FLASH
	mutex_lock(&(g_chip_info->fw_update_lock));
#endif
	hx83102j_enable_interrupt(g_chip_info, false);

	do {
	#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(chip_info->hw_res, true);
		hx83102j_resetgpio_set(chip_info->hw_res, false);
		hx83102j_resetgpio_set(chip_info->hw_res, true);
	#else
		g_core_fp.fp_sys_reset();
	#endif
		TPD_DETAIL("%s: ESD reset finished\n", __func__);

		TPD_DETAIL("It will update fw after esd event in zero flash mode!\n");

		load_fw_times--;
		g_core_fp.fp_0f_operation_dirly();
		ret = g_core_fp.fp_reload_disable(0);
	} while (!ret && load_fw_times > 0);

	if (!load_fw_times) {
		TPD_INFO("%s: load_fw_times over 10 times\n", __func__);
	}

	himax_sense_on(0x00);
	/* need_modify*/
	/* report all leave event
	himax_report_all_leave_event(private_ts);*/

	hx83102j_enable_interrupt(g_chip_info, true);

#ifdef HX_ZERO_FLASH
	mutex_unlock(&(g_chip_info->fw_update_lock));
#endif
}


int himax_checksum_cal(struct chip_data_hx83102j *chip_info, uint8_t *buf, int ts_status)
{
	int hx_eb_event = 0;
	int hx_ec_event = 0;
	int hx_ed_event = 0;
	int hx_ee_event = 0;
	int hx_esd_event = 0;
	int hx_zero_event = 0;
	int shaking_ret = 0;

	uint16_t check_sum_cal = 0;
	int32_t loop_i = 0;
	int length = 0;


	/* Normal */
	if (ts_status == HX_REPORT_COORD) {
		length = hx_touch_data->touch_info_size;
	}

	/* SMWP */
	else if (ts_status == HX_REPORT_SMWP_EVENT) {
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
	} else {
		TPD_INFO("%s, Neither Normal Nor SMWP error!\n", __func__);
	}

	for (loop_i = 0; loop_i < length; loop_i++) {
		check_sum_cal += buf[loop_i];
		/* #ifdef HX_ESD_RECOVERY  */
		if (ts_status == HX_REPORT_COORD || ts_status == HX_REPORT_SMWP_EVENT) {
			/* case 1 ESD recovery flow */
			if (buf[loop_i] == 0xEB) {
				hx_eb_event++;
			} else if (buf[loop_i] == 0xEC) {
				hx_ec_event++;
			} else if (buf[loop_i] == 0xED) {
				hx_ed_event++;
			} else if (buf[loop_i] == 0x00) {/* case 2 ESD recovery flow-Disable */
				hx_zero_event++;
			} else {
				hx_eb_event = 0;
				hx_ec_event = 0;
				hx_ed_event = 0;
				hx_ee_event = 0;
				hx_zero_event = 0;
				g_zero_event_count = 0;
			}

			if (hx_eb_event == length) {
				hx_esd_event = length;
				hx_eb_event_flag++;
				TPD_INFO("[HIMAX TP MSG]: ESD event checked - ALL 0xEB.\n");
			} else if (hx_ec_event == length) {
				hx_esd_event = length;
				hx_ec_event_flag++;
				TPD_INFO("[HIMAX TP MSG]: ESD event checked - ALL 0xEC.\n");
			} else if (hx_ee_event == length) {
				hx_esd_event = length;
				hx_ee_event_flag++;
				TPD_INFO("[HIMAX TP MSG]: ESD event checked - ALL 0xEE.\n");
			} else if (hx_ed_event == length) {
				himax_hx83102j_reload_to_active();
				TPD_INFO("[HIMAX TP MSG]: EXCPT event checked - ALL 0xED.\n");

			} else {
				hx_esd_event = 0;
			}
		}
		/* #endif */
	}

	if (ts_status == HX_REPORT_COORD) {
		if (hx_esd_event == length || hx_ed_event == length) {
			shaking_ret = himax_ic_esd_recovery(hx_esd_event, hx_ed_event, length);
			if (shaking_ret == CHECKSUM_FAIL) {
				himax_esd_hw_reset(chip_info);
				goto checksum_fail;
			} else if (shaking_ret == ERR_WORK_OUT) {
				goto err_workqueue_out;
			} else {
				goto workqueue_out;
			}
		} else if (hx_esd_reset_activate) {
			/* drop 1st interrupts after chip reset */
			hx_esd_reset_activate = 0;
			TPD_INFO("[hx_esd_reset_activate]:%s: Back from reset, ready to serve.\n", __func__);
			goto checksum_fail;
		} else if (hx_hw_reset_activate) {
			/* drop 1st interrupts after chip reset */
			hx_hw_reset_activate = 0;
			TPD_INFO("[hx_hw_reset_activate]:%s: Back from reset, ready to serve.\n", __func__);
			goto ready_to_serve;
		}
	}

	if ((check_sum_cal % 0x100 != 0)) {
		TPD_INFO("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);
		goto workqueue_out;
	}

	/* TPD_INFO("%s:End\n",__func__); */
	return NO_ERR;

ready_to_serve:
	return READY_TO_SERVE;
checksum_fail:
	return CHECKSUM_FAIL;
err_workqueue_out:
	return ERR_WORK_OUT;
workqueue_out:
	return WORK_OUT;
}

void himax_log_touch_data(uint8_t *buf, struct himax_report_data *hx_touch_data)
{
	int loop_i = 0;
	int print_size = 0;

	if (!hx_touch_data->diag_cmd) {
		print_size = hx_touch_data->touch_info_size;
	} else {
		print_size = hx_touch_data->touch_all_size;
	}

	for (loop_i = 0; loop_i < print_size; loop_i++) {
		printk("0x%02X ",  buf[loop_i]);
		if((loop_i + 1) % 8 == 0) {
			printk("\n");
		}
		if (loop_i == (print_size - 1)) {
			printk("\n");
		}
	}
}

void himax_flash_programming(uint8_t *FW_content, int FW_Size)
{
	int page_prog_start = 0;
	int program_length = 48;
	int i = 0, j = 0, k = 0;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t *buring_data;


	buring_data = kzalloc(sizeof(uint8_t) * 256, GFP_KERNEL);
	if (!buring_data) {
		TPD_INFO("%s:[error] buring_data kzalloc fail!\n", __func__);
		goto RET_OUT;
	}
	himax_interface_on();

	/*=====================================
		SPI Transfer Format : 0x8000_0010 ==> 0x0002_0780
	=====================================*/
	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x10;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x02;
	tmp_data[1] = 0x07;
	tmp_data[0] = 0x80;
	himax_flash_write_burst(tmp_addr, tmp_data);

	for (page_prog_start = 0; page_prog_start < FW_Size; page_prog_start = page_prog_start + 256) {
		/*=====================================
			Write Enable : 1. 0x8000_0020 ==> 0x4700_0000
							2. 0x8000_0024 ==> 0x0000_0006
		=====================================*/
		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x20;
		tmp_data[3] = 0x47;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;
		himax_flash_write_burst(tmp_addr, tmp_data);

		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x24;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x06;
		himax_flash_write_burst(tmp_addr, tmp_data);

		/*=================================
			SPI Transfer Control
			Set 256 bytes page write : 0x8000_0020 ==> 0x610F_F000
			Set read start address   : 0x8000_0028 ==> 0x0000_0000
		=================================*/
		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x20;
		tmp_data[3] = 0x61;
		tmp_data[2] = 0x0F;
		tmp_data[1] = 0xF0;
		tmp_data[0] = 0x00;

		himax_flash_write_burst(tmp_addr, tmp_data);

		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x28;


		if (page_prog_start < 0x100) {
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = (uint8_t)page_prog_start;
		} else if (page_prog_start >= 0x100 && page_prog_start < 0x10000) {
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = (uint8_t)(page_prog_start >> 8);
			tmp_data[0] = (uint8_t)page_prog_start;
		} else if (page_prog_start >= 0x10000 && page_prog_start < 0x1000000) {
			tmp_data[3] = 0x00;
			tmp_data[2] = (uint8_t)(page_prog_start >> 16);
			tmp_data[1] = (uint8_t)(page_prog_start >> 8);
			tmp_data[0] = (uint8_t)page_prog_start;
		}

		himax_flash_write_burst(tmp_addr, tmp_data);


		/*=================================
			Send 16 bytes data : 0x8000_002C ==> 16 bytes data
		=================================*/
		buring_data[0] = 0x2C;
		buring_data[1] = 0x00;
		buring_data[2] = 0x00;
		buring_data[3] = 0x80;

		for (i = /*0*/page_prog_start, j = 0; i < 16 + page_prog_start/**/; i++, j++) {
			buring_data[j + 4] = FW_content[i];
		}


		if (himax_bus_write(0x00, 20, buring_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			goto RET_OUT;
		}
		/*=================================
			Write command : 0x8000_0024 ==> 0x0000_0002
		=================================*/
		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x24;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x02;
		himax_flash_write_burst(tmp_addr, tmp_data);

		/*=================================
			Send 240 bytes data : 0x8000_002C ==> 240 bytes data
		=================================*/

		for (j = 0; j < 5; j++) {
			for (i = (page_prog_start + 16 + (j * 48)), k = 0; i < (page_prog_start + 16 + (j * 48)) + program_length; i++, k++) {
				buring_data[k + 4] = FW_content[i];
			}

			if (himax_bus_write(0x00, program_length + 4, buring_data) < 0) {
				TPD_INFO("%s: i2c access fail!\n", __func__);
				goto RET_OUT;
			}
		}

		if (!wait_wip(1)) {
			TPD_INFO("%s:83102j_Flash_Programming Fail\n", __func__);
		}
	}
RET_OUT:
	if(buring_data)
		kfree(buring_data);
	return;
}



int fts_ctpm_fw_upgrade_with_sys_fs_64k(unsigned char *fw, int len, bool change_iref) /* Alice - Un */
{
	return 0;
}


static size_t hx83102j_proc_register_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
	size_t ret = 0;
	uint16_t loop_i;
	int i = 0;
	int times = 0;
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	int max_bus_size = MAX_RECVS_SZ;
#else
	int max_bus_size = 128;
#endif
	uint8_t *data;
	uint32_t now_address;
	char *temp_buf;
	uint8_t now_reg_cmd[4];


	data = kzalloc(sizeof(uint8_t) * 128, GFP_KERNEL);
	if(!data) {
		TPD_INFO("%s: [data]Can't allocate enough data\n", __func__);
		ret = -ENOMEM;
		goto RET_OUT;
	}

	if (!hx_proc_send_flag) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(!temp_buf) {
			TPD_INFO("%s: [tmp_buf]Can't allocate enough data\n", __func__);
			ret = -ENOMEM;
			goto RET_OUT;
		}

		TPD_INFO("himax_register_show: %02X, %02X, %02X, %02X\n", register_command[3], register_command[2], register_command[1], register_command[0]);

		times = (128 % max_bus_size) > 0 ? 128 / max_bus_size + 1 : 128 / max_bus_size;
		for (i = 0 ; i < times; i++) {
			now_address = (register_command[3] << 24) + (register_command[2] << 16)
							+ (register_command[1] << 8) + (register_command[0]);
			now_address += (max_bus_size * i);
			TPD_INFO("Now times %d: reg=0x%08X\n", i, now_address);
			now_reg_cmd[3] = (now_address >> 24) & 0xFF;
			now_reg_cmd[2] = (now_address >> 16) & 0xFF;
			now_reg_cmd[1] = (now_address >> 8) & 0xFF;
			now_reg_cmd[0] = now_address & 0xFF;
			if (i == (times - 1) && i != 0)
				himax_register_read(now_reg_cmd, 128 % max_bus_size, &data[i * max_bus_size], cfg_flag);
			else
				himax_register_read(now_reg_cmd, max_bus_size, &data[i * max_bus_size], cfg_flag);
			now_address = 0;
		}
		ret += snprintf(temp_buf + ret, len - ret, "command:  %02X, %02X, %02X, %02X\n", register_command[3],
							register_command[2], register_command[1], register_command[0]);
		for (loop_i = 0; loop_i < 128; loop_i++) {
			ret += snprintf(temp_buf + ret, len - ret, "0x%2.2X ", data[loop_i]);
			if ((loop_i % 16) == 15) {
				ret += snprintf(temp_buf + ret, len - ret, "\n");
			}
		}
		ret += snprintf(temp_buf + ret, len - ret, "\n");
		if (copy_to_user(buf, temp_buf, len)) {
			TPD_INFO("%s, here:%d\n", __func__, __LINE__);
		}
		kfree(temp_buf);
		hx_proc_send_flag = 1;
	} else {
		hx_proc_send_flag = 0;
	}
	if (data)
		kfree(data);
RET_OUT:
	return ret;
}


static size_t hx83102j_proc_register_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
	char buf[81] = {0};
	char buf_tmp[16];
	uint8_t length = 0;
	unsigned long result = 0;
	uint8_t loop_i = 0;
	uint16_t base = 2;
	char *data_str = NULL;
	uint8_t w_data[20];
	uint8_t x_pos[20];
	uint8_t count = 0;


	if (len >= 80) {
		TPD_INFO("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}
	buf[len] = '\0';

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(w_data, 0x0, sizeof(w_data));
	memset(x_pos, 0x0, sizeof(x_pos));

	TPD_INFO("himax %s \n", buf);

	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' && buf[2] == 'x') {
		length = strlen(buf);


		for (loop_i = 0; loop_i < length; loop_i++) {
			if (buf[loop_i] == 'x') {
				x_pos[count] = loop_i;
				count++;
			}
		}

		data_str = strrchr(buf, 'x');
		TPD_INFO("%s: %s.\n", __func__, data_str);
		length = strlen(data_str + 1) - 1;

		if (buf[0] == 'r') {
			if (buf[3] == 'F' && buf[4] == 'E' && length == 4) {
				length = length - base;
				cfg_flag = true;
				memcpy(buf_tmp, data_str + base + 1, length);
			} else {
				cfg_flag = false;
				memcpy(buf_tmp, data_str + 1, length);
			}

			byte_length = length / 2;
			if (!kstrtoul(buf_tmp, 16, &result)) {
				for (loop_i = 0 ; loop_i < byte_length ; loop_i++) {
					register_command[loop_i] = (uint8_t)(result >> loop_i * 8);
				}
			}
		} else if (buf[0] == 'w') {
			if (buf[3] == 'F' && buf[4] == 'E') {
				cfg_flag = true;
				memcpy(buf_tmp, buf + base + 3, length);
			} else {
				cfg_flag = false;
				memcpy(buf_tmp, buf + 3, length);
			}
			if (count < 3) {
				byte_length = length / 2;
				if (!kstrtoul(buf_tmp, 16, &result)) {
					for (loop_i = 0 ; loop_i < byte_length ; loop_i++) {
						register_command[loop_i] = (uint8_t)(result >> loop_i * 8);
					}
				}
				if (!kstrtoul(data_str + 1, 16, &result)) {
					for (loop_i = 0 ; loop_i < byte_length ; loop_i++) {
						w_data[loop_i] = (uint8_t)(result >> loop_i * 8);
					}
				}
				himax_register_write(register_command, byte_length, w_data, cfg_flag);
			} else {
				byte_length = x_pos[1] - x_pos[0] - 2;
				for (loop_i = 0; loop_i < count; loop_i++) {
					memcpy(buf_tmp, buf + x_pos[loop_i] + 1, byte_length);

					if (!kstrtoul(buf_tmp, 16, &result)) {
						if (loop_i == 0) {
							register_command[loop_i] = (uint8_t)(result);
						} else {
							w_data[loop_i - 1] = (uint8_t)(result);
						}
					}
				}
				byte_length = count - 1;
				himax_register_write(register_command, byte_length, &w_data[0], cfg_flag);
			}
		} else {
			return len;
		}
	}
	return len;
}

void himax_return_event_stack(void)
{
	int retry = 20;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	TPD_INFO("%s:entering\n", __func__);
	do {
		TPD_INFO("%s, now %d times\n!", __func__, retry);
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;
		himax_flash_write_burst(tmp_addr, tmp_data);

		himax_register_read(tmp_addr, 4, tmp_data, false);
		retry--;
	} while ((tmp_data[1] != 0x00 && tmp_data[0] != 0x00) && retry > 0);

	TPD_INFO("%s: End of setting!\n", __func__);
}
/*IC_BASED_END*/

int himax_write_read_reg(uint8_t *tmp_addr, uint8_t *tmp_data, uint8_t hb, uint8_t lb)
{
	int cnt = 0;

	do {
		himax_flash_write_burst(tmp_addr, tmp_data);

		msleep(20);
		himax_register_read(tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s:Now tmp_data[0] = 0x%02X, [1] = 0x%02X, [2] = 0x%02X, [3] = 0x%02X\n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
	} while ((tmp_data[1] != hb && tmp_data[0] != lb) && cnt++ < 100);

	if (cnt >= 99) {
		TPD_INFO("himax_write_read_reg ERR Now register 0x%08X : high byte = 0x%02X, low byte = 0x%02X\n", tmp_addr[3], tmp_data[1], tmp_data[0]);
		return -1;
	}

	TPD_INFO("Now register 0x%08X : high byte = 0x%02X, low byte = 0x%02X\n", tmp_addr[3], tmp_data[1], tmp_data[0]);
	return NO_ERR;
}

void himax_get_DSRAM_data(uint8_t *info_data, uint8_t x_num, uint8_t y_num)
{
	int i = 0;
	unsigned char tmp_addr[4];
	unsigned char tmp_data[4];
	uint8_t max_i2c_size = MAX_RECVS_SZ;
	int m_key_num = 0;
	int total_size = (x_num * y_num + x_num + y_num) * 2 + 4;
	int total_size_temp;
	int mutual_data_size = x_num * y_num * 2;
	int total_read_times = 0;
	int address = 0;
	uint8_t *temp_info_data;
	uint32_t check_sum_cal = 0;
	int fw_run_flag = -1;


	temp_info_data = kzalloc(sizeof(uint8_t) * (total_size + 8), GFP_KERNEL);

	/*1. Read number of MKey R100070E8H to determin data size*/
	m_key_num = 0;

	total_size += m_key_num * 2;

	/* 2. Start DSRAM Rawdata and Wait Data Ready */
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x00;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x5A;
	tmp_data[0] = 0xA5;
	fw_run_flag = himax_write_read_reg(tmp_addr, tmp_data, 0xA5, 0x5A);
	if (fw_run_flag < 0) {
		TPD_INFO("%s Data NOT ready => bypass \n", __func__);
		kfree(temp_info_data);
		return;
	}

	/* 3. Read RawData */
	total_size_temp = total_size;
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x00;

	if (total_size % max_i2c_size == 0) {
		total_read_times = total_size / max_i2c_size;
	} else {
		total_read_times = total_size / max_i2c_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		if (total_size_temp >= max_i2c_size) {
			himax_register_read(tmp_addr, max_i2c_size, &temp_info_data[i * max_i2c_size], false);
			total_size_temp = total_size_temp - max_i2c_size;
		} else {
			himax_register_read(tmp_addr, total_size_temp % max_i2c_size,
								&temp_info_data[i * max_i2c_size], false);
		}

		address = ((i + 1) * max_i2c_size);
		tmp_addr[1] = (uint8_t)((address >> 8) & 0x00FF);
		tmp_addr[0] = (uint8_t)((address) & 0x00FF);
	}

	/* 4. FW stop outputing */

	if (dsram_flag == false) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;
		himax_flash_write_burst(tmp_addr, tmp_data);
	} else {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		tmp_data[3] = 0x11;
		tmp_data[2] = 0x22;
		tmp_data[1] = 0x33;
		tmp_data[0] = 0x44;
		himax_flash_write_burst(tmp_addr, tmp_data);
	}

	/* 5. Data Checksum Check */
	for (i = 2; i < total_size; i = i + 2) { /* 2:PASSWORD NOT included */
		check_sum_cal += (temp_info_data[i + 1] * 256 + temp_info_data[i]);
		printk("0x%2x:0x%4x ", temp_info_data[i], check_sum_cal);
		if (i % 32 == 0)
			printk("\n");
	}

	if (check_sum_cal % 0x10000 != 0) {
		memcpy(info_data, &temp_info_data[4], mutual_data_size * sizeof(uint8_t));
		TPD_INFO("%s check_sum_cal fail=%2x \n", __func__, check_sum_cal);
		kfree(temp_info_data);
		return;
	} else {
		memcpy(info_data, &temp_info_data[4], mutual_data_size * sizeof(uint8_t));
		TPD_INFO("%s checksum PASS \n", __func__);
	}
	kfree(temp_info_data);
}

void himax_ts_diag_func(struct chip_data_hx83102j *chip_info, int32_t *mutual_data)
{
	int i = 0;
	int j = 0;
	unsigned int index = 0;
	int total_size = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num * 2;
	uint8_t *info_data = NULL;
	int32_t new_data;
	/* 1:common dsram,2:100 frame Max,3:N-(N-1)frame */
	int dsram_type = 0;
	char *write_buf = NULL;

	info_data = kcalloc(total_size, sizeof(uint32_t), GFP_KERNEL);
	if (info_data == NULL) {
			TPD_INFO("%s, Failed to allocate memory\n", __func__);
			return;
	}

	write_buf = kcalloc(total_size * 3, sizeof(uint32_t), GFP_KERNEL);
	if (write_buf == NULL) {
			TPD_INFO("%s, Failed to allocate memory\n", __func__);
			return;
	}

	memset(write_buf, 0, total_size * 3);

	dsram_type = g_diag_command / 10;

	TPD_INFO("%s:Entering g_diag_command=%d\n!", __func__, g_diag_command);

	if (dsram_type == 8) {
		dsram_type = 1;
		TPD_INFO("%s Sorting Mode run sram type1 ! \n", __func__);
	}

	himax_burst_enable(1);
	himax_get_DSRAM_data(info_data, chip_info->hw_res->rx_num, chip_info->hw_res->tx_num);

	index = 0;
	for (i = 0; i < chip_info->hw_res->tx_num; i++) {
		for (j = 0; j < chip_info->hw_res->rx_num; j++) {
			new_data = ((int8_t)info_data[index + 1] << 8 | info_data[index]);
			mutual_data[i * chip_info->hw_res->rx_num + j] = new_data;
			index += 2;
		}
	}
}

void diag_parse_raw_data(struct himax_report_data *hx_touch_data, int mul_num, int self_num, uint8_t diag_cmd, int32_t *mutual_data, int32_t *self_data)
{
	int rawdatalen_word;
	int index = 0;
	int temp1, temp2, i;

	if (hx_touch_data->hx_rawdata_buf[0] == 0x3A
		&& hx_touch_data->hx_rawdata_buf[1] == 0xA3
		&& hx_touch_data->hx_rawdata_buf[2] > 0
		&& hx_touch_data->hx_rawdata_buf[3] == diag_cmd) {
		rawdatalen_word = hx_touch_data->rawdata_size / 2;
		index = (hx_touch_data->hx_rawdata_buf[2] - 1) * rawdatalen_word;

		for (i = 0; i < rawdatalen_word; i++) {
			temp1 = index + i;

			if (temp1 < mul_num) {
				mutual_data[index + i] = ((int8_t)hx_touch_data->hx_rawdata_buf[i * 2 + 4 + 1]) * 256 + hx_touch_data->hx_rawdata_buf[i * 2 + 4];
			} else {
				temp1 = i + index;
				temp2 = self_num + mul_num;

				if (temp1 >= temp2) {
					break;
				}
				self_data[i + index - mul_num] = (((int8_t)hx_touch_data->hx_rawdata_buf[i * 2 + 4 + 1]) << 8) | hx_touch_data->hx_rawdata_buf[i * 2 + 4];
			}
		}
	}
}

bool diag_check_sum(struct himax_report_data *hx_touch_data) /*return checksum value  */
{
	uint16_t check_sum_cal = 0;
	int i;

	/*Check 128th byte crc*/
	for (i = 0, check_sum_cal = 0; i < (hx_touch_data->touch_all_size - hx_touch_data->touch_info_size); i = i + 2) {
		check_sum_cal += (hx_touch_data->hx_rawdata_buf[i + 1] * 256 + hx_touch_data->hx_rawdata_buf[i]);
	}
	if (check_sum_cal % 0x10000 != 0) {
		TPD_INFO("%s fail=%2X \n", __func__, check_sum_cal);
		return 0;
	}

	return 1;
}

static size_t hx83102j_proc_diag_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
	char messages[80] = {0};
	uint8_t command[2] = {0x00, 0x00};
	uint8_t receive[1];

	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	/* 0: common, other: dsram*/
	int storage_type = 0;
	/* 1:IIR, 2:DC, 3:Bank, 4:IIR2, 5:IIR2_N, 6:FIR2, 7:Baseline, 8:dump coord */
	int rawdata_type = 0;

	memset(receive, 0x00, sizeof(receive));

	if (len >= 80) {
		TPD_INFO("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	if (messages[1] == 0x0A) {
		g_diag_command = messages[0] - '0';
	} else {
		g_diag_command = (messages[0] - '0') * 10 + (messages[1] - '0');
	}

	storage_type = g_diag_command / 10;
	rawdata_type = g_diag_command % 10;

	TPD_INFO(" messages       = %s\n"
			 " g_diag_command = 0x%x\n"
			 " storage_type   = 0x%x\n"
			 " rawdata_type   = 0x%x\n",
			 messages, g_diag_command, storage_type, rawdata_type);

	if (g_diag_command > 0 && rawdata_type == 0) {
		TPD_INFO("[Himax]g_diag_command = 0x%x, storage_type=%d, rawdata_type=%d! Maybe no support!\n", g_diag_command, storage_type, rawdata_type);
		g_diag_command = 0x00;
	} else {
		TPD_INFO("[Himax]g_diag_command = 0x%x, storage_type=%d, rawdata_type=%d\n", g_diag_command, storage_type, rawdata_type);
	}

	if (storage_type == 0 && rawdata_type > 0 && rawdata_type < 8) {
		TPD_INFO("%s, common\n", __func__);
		if (dsram_flag) {
			/*(1) Clear DSRAM flag*/
			dsram_flag = false;
			/*(2) Enable ISR*/
			enable_irq(chip_info->hx_irq);
			/*(3) FW leave sram and return to event stack*/
			himax_return_event_stack();
		}

		command[0] = g_diag_command;
		himax_diag_register_set(command[0]);
	} else if (storage_type > 0 && storage_type < 8 && rawdata_type > 0 && rawdata_type < 8) {
		TPD_INFO("%s, dsram\n", __func__);

		/*0. set diag flag*/
		if (dsram_flag) {
			/*(1) Clear DSRAM flag*/
			dsram_flag = false;
			/*(2) Enable ISR*/
			enable_irq(chip_info->hx_irq);
			/*(3) FW leave sram and return to event stack*/
			himax_return_event_stack();
		}

		switch(rawdata_type) {
		case 1:
			command[0] = 0x09; /*IIR*/
			break;

		case 2:
			command[0] = 0x0A;/*RAWDATA*/
			break;

		case 3:
			command[0] = 0x08;/*Baseline*/
			break;

		default:
			command[0] = 0x00;
			TPD_INFO("%s: Sram no support this type !\n", __func__);
			break;
		}
		himax_diag_register_set(command[0]);
		TPD_INFO("%s: Start get raw data in DSRAM\n", __func__);
		/*1. Disable ISR*/
		disable_irq(chip_info->hx_irq);

		/*2. Set DSRAM flag*/
		dsram_flag = true;
	} else {
		/*set diag flag*/
		if (dsram_flag) {
			TPD_INFO("return and cancel sram thread!\n");
			/*(1) Clear DSRAM flag*/
			dsram_flag = false;
			/*(2) Enable ISR*/
			enable_irq(chip_info->hx_irq);
			himax_return_event_stack();
		}
		command[0] = 0x00;
		g_diag_command = 0x00;
		himax_diag_register_set(command[0]);
		TPD_INFO("return to normal g_diag_command = 0x%x\n", g_diag_command);
	}
	return len;
}

/*
static size_t hx83102j_proc_diag_read(struct file *file, const char *buff, size_t len, loff_t *pos)
{
	size_t ret = 0;
	#if 0
	size_t ret = 0;
	uint16_t mutual_num;
	uint16_t self_num;
	uint16_t width;
	int dsram_type = 0;
	int data_type = 0;
	int i = 0;
	int j = 0;
	int k = 0;

	struct touchpanel_data *ts = s->private;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	mutual_num = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;
	self_num = chip_info->hw_res->tx_num + chip_info->hw_res->rx_num;
	width = chip_info->hw_res->rx_num;


	s->size = (mutual_num + self_num) * 6 + 256;
	s->buf = kcalloc(s->size, sizeof(char), GFP_KERNEL);
	if (s->buf == NULL) {
		TPD_INFO("[ERROR]%s,%d: Memory allocation falied!\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	memset(s->buf, 0, s->size * sizeof(char));


	dsram_type = g_diag_command / 10;
	data_type = g_diag_command % 10;


	seq_printf(s, "ChannelStart (rx tx) : %4d, %4d\n\n", chip_info->hw_res->rx_num, chip_info->hw_res->tx_num);

		//start to show out the raw data in adb shell
		if ((data_type >= 1 && data_type <= 7)) {
			if (dsram_type > 0)
				himax_ts_diag_func(chip_info, hx_touch_data->diag_mutual);

		for (j = 0; j < chip_info->hw_res->rx_num ; j++) {
			for (i = 0; i < chip_info->hw_res->tx_num; i++) {
				k = ((mutual_num - j) - chip_info->hw_res->rx_num * i) - 1;
				seq_printf(s, "%6d", hx_touch_data->diag_mutual[k]);
			}
			seq_printf(s, " %6d\n", diag_self[j]);
		}

		seq_puts(s, "\n");
		for (i = 0; i < chip_info->hw_res->tx_num; i++) {
			seq_printf(s, "%6d", diag_self[i]);
		}
	}

	seq_puts(s, "\n");
	seq_puts(s, "ChannelEnd");
	seq_puts(s,  "\n");

	// print Mutual/Slef Maximum and Minimum

	for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
		if (hx_touch_data->diag_mutual[i] > g_max_mutual) {
			g_max_mutual = hx_touch_data->diag_mutual[i];
		}
		if (hx_touch_data->diag_mutual[i] < g_min_mutual) {
			g_min_mutual = hx_touch_data->diag_mutual[i];
		}
	}

	for (i = 0; i < (chip_info->hw_res->tx_num + chip_info->hw_res->rx_num); i++) {
		if (diag_self[i] > g_max_self) {
			g_max_self = diag_self[i];
		}
		if (diag_self[i] < g_min_self) {
			g_min_self = diag_self[i];
		}
	}

	seq_printf(s, "Mutual Max:%3d, Min:%3d\n", g_max_mutual, g_min_mutual);
	seq_printf(s, "Self Max:%3d, Min:%3d\n", g_max_self, g_min_self);

	//recovery status after print
	g_max_mutual = 0;
	g_min_mutual = 0xFFFF;
	g_max_self = 0;
	g_min_self = 0xFFFF;

	#endif
	return ret;
}*/

uint8_t himax_read_DD_status(struct chip_data_hx83102j *chip_info, uint8_t *cmd_set, uint8_t *tmp_data)
{
	int cnt = 0;
	uint8_t req_size = cmd_set[0];
	uint8_t cmd_addr[4] = {0xFC, 0x00, 0x00, 0x90};
	uint8_t tmp_addr[4] = {0x80, 0x7F, 0x00, 0x10};

	cmd_set[3] = 0xAA;
	himax_register_write(cmd_addr, 4, cmd_set, 0);

	TPD_INFO("%s: cmd_set[0] = 0x%02X, cmd_set[1] = 0x%02X, cmd_set[2] = 0x%02X, cmd_set[3] = 0x%02X\n", __func__, cmd_set[0], cmd_set[1], cmd_set[2], cmd_set[3]);

	for (cnt = 0; cnt < 100; cnt++) {
		himax_register_read(cmd_addr, 4, tmp_data, false);

		msleep(10);
		if (tmp_data[3] == 0xBB) {
			TPD_INFO("%s Data ready goto moving data\n", __func__);
			break;
		} else if (cnt >= 99) {
			TPD_INFO("%s Data not ready in FW \n", __func__);
			return FW_NOT_READY;
		}
	}
	himax_register_read(tmp_addr, req_size, tmp_data, false);
	return NO_ERR;
}

static size_t hx83102j_proc_DD_debug_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
	int ret = 0;
	uint8_t tmp_data[64];
	uint8_t loop_i = 0;
	char *temp_buf;

	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	if (!hx_proc_send_flag) {
		temp_buf = kzalloc(len, GFP_KERNEL);

		if (mutual_set_flag == 1) {
			if (himax_read_DD_status(chip_info, cmd_set, tmp_data) == NO_ERR) {
				for (loop_i = 0; loop_i < cmd_set[0]; loop_i++) {
					if ((loop_i % 8) == 0) {
						ret += snprintf(temp_buf + ret, len - ret, "0x%02X : ", loop_i);
					}
					ret += snprintf(temp_buf + ret, len - ret, "0x%02X ", tmp_data[loop_i]);
					if ((loop_i % 8) == 7)
						ret += snprintf(temp_buf + ret, len - ret, "\n");
				}
			}
		}

		ret += snprintf(temp_buf + ret, len - ret, "\n");
		if (copy_to_user(buf, temp_buf, len))
			TPD_INFO("%s, here:%d\n", __func__, __LINE__);
		kfree(temp_buf);
		hx_proc_send_flag = 1;
	} else
		hx_proc_send_flag = 0;
	return ret;
}

static size_t hx83102j_proc_DD_debug_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
	uint8_t i = 0;
	uint8_t cnt = 2;
	unsigned long result = 0;
	char buf_tmp[20];
	char buf_tmp2[4];

	if (len >= 20) {
		TPD_INFO("%s: no command exceeds 20 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf_tmp, buff, len)) {
		return -EFAULT;
	}
	memset(buf_tmp2, 0x0, sizeof(buf_tmp2));

	if (buf_tmp[2] == 'x' && buf_tmp[6] == 'x' && buf_tmp[10] == 'x') {
		mutual_set_flag = 1;
		for (i = 3; i < 12; i = i + 4) {
			memcpy(buf_tmp2, buf_tmp + i, 2);
			if (!kstrtoul(buf_tmp2, 16, &result))
				cmd_set[cnt] = (uint8_t)result;
			else
				TPD_INFO("String to oul is fail in cnt = %d, buf_tmp2 = %s", cnt, buf_tmp2);
			cnt--;
		}
		TPD_INFO("cmd_set[2] = %02X, cmd_set[1] = %02X, cmd_set[0] = %02X\n", cmd_set[2], cmd_set[1], cmd_set[0]);
	} else
		mutual_set_flag = 0;

	return len;
}

int himax_read_FW_status(struct chip_data_hx83102j *chip_info, uint8_t *state_addr, uint8_t *tmp_addr)
{
	uint8_t req_size = 0;
	uint8_t status_addr[4] = {0x44, 0x7F, 0x00, 0x10};
	uint8_t cmd_addr[4] = {0xF8, 0x00, 0x00, 0x90};

	if (state_addr[0] == 0x01) {
		state_addr[1] = 0x04;
		state_addr[2] = status_addr[0];
		state_addr[3] = status_addr[1];
		state_addr[4] = status_addr[2];
		state_addr[5] = status_addr[3];
		req_size = 0x04;
		himax_sense_off();
		himax_register_read(status_addr, req_size, tmp_addr, false);
		himax_sense_on(1);
	} else if (state_addr[0] == 0x02) {
		state_addr[1] = 0x30;
		state_addr[2] = cmd_addr[0];
		state_addr[3] = cmd_addr[1];
		state_addr[4] = cmd_addr[2];
		state_addr[5] = cmd_addr[3];
		req_size = 0x30;
		himax_register_read(cmd_addr, req_size, tmp_addr, false);
	}

	return NO_ERR;
}

static size_t hx83102j_proc_FW_debug_read(struct file *file, char *buf,
		size_t len, loff_t *pos)
{
	int ret = 0;
	uint8_t loop_i = 0;
	uint8_t tmp_data[64];
	char *temp_buf;

	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	if (!hx_proc_send_flag) {
		temp_buf = kzalloc(len, GFP_KERNEL);

		cmd_set[0] = 0x01;
		if (himax_read_FW_status(chip_info, cmd_set, tmp_data) == NO_ERR) {
			ret += snprintf(temp_buf + ret, len - ret, "0x%02X%02X%02X%02X :\t", cmd_set[5], cmd_set[4], cmd_set[3], cmd_set[2]);
			for (loop_i = 0; loop_i < cmd_set[1]; loop_i++) {
				ret += snprintf(temp_buf + ret, len - ret, "%5d\t", tmp_data[loop_i]);
			}
			ret += snprintf(temp_buf + ret, len - ret, "\n");
		}
		cmd_set[0] = 0x02;
		if (himax_read_FW_status(chip_info, cmd_set, tmp_data) == NO_ERR) {
			for (loop_i = 0; loop_i < cmd_set[1]; loop_i = loop_i + 2) {
				if ((loop_i % 16) == 0)
					ret += snprintf(temp_buf + ret, len - ret, "0x%02X%02X%02X%02X :\t",
									cmd_set[5], cmd_set[4], cmd_set[3] + (((cmd_set[2] + loop_i) >> 8) & 0xFF), (cmd_set[2] + loop_i) & 0xFF);

				ret += snprintf(temp_buf + ret, len - ret, "%5d\t", tmp_data[loop_i] + (tmp_data[loop_i + 1] << 8));
				if ((loop_i % 16) == 14)
					ret += snprintf(temp_buf + ret, len - ret, "\n");
			}
		}
		ret += snprintf(temp_buf + ret, len - ret, "\n");
		if (copy_to_user(buf, temp_buf, len))
			TPD_INFO("%s, here:%d\n", __func__, __LINE__);
		kfree(temp_buf);
		hx_proc_send_flag = 1;
	} else
		hx_proc_send_flag = 0;
	return ret;
}

static int hx83102j_configuration_init(struct chip_data_hx83102j *chip_info, bool config)
{
	int ret = 0;
	TPD_INFO("%s, configuration init = %d\n", __func__, config);
	return ret;
}

int himax_ic_reset(struct chip_data_hx83102j *chip_info, uint8_t loadconfig, uint8_t int_off)
{
	int ret = 0;
	hx_hw_reset_activate = 1;

	TPD_INFO("%s, status: loadconfig=%d, int_off=%d\n", __func__, loadconfig, int_off);

	if (chip_info->hw_res->reset_gpio) {
		if (int_off) {
			ret = hx83102j_enable_interrupt(chip_info, false);
			if (ret < 0) {
				TPD_INFO("%s: hx83102j enable interrupt failed.\n", __func__);
				return ret;
			}
		}
	#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(chip_info->hw_res, false);
		hx83102j_resetgpio_set(chip_info->hw_res, true);
	#else
		g_core_fp.fp_sys_reset();
	#endif
		if (loadconfig) {
			ret = hx83102j_configuration_init(chip_info, false);
			if (ret < 0) {
				TPD_INFO("%s: hx83102j configuration init failed.\n", __func__);
				return ret;
			}
			ret = hx83102j_configuration_init(chip_info, true);
			if (ret < 0) {
				TPD_INFO("%s: hx83102j configuration init failed.\n", __func__);
				return ret;
			}
		}
		if (int_off) {
			ret = hx83102j_enable_interrupt(chip_info, true);
			if (ret < 0) {
				TPD_INFO("%s: hx83102j enable interrupt failed.\n", __func__);
				return ret;
			}
		}
	}
	return 0;
}

static size_t hx83102j_proc_reset_write(struct file *file, const char *buff,
										size_t len, loff_t *pos)
{
	char buf_tmp[12];
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	if (len >= 12) {
		TPD_INFO("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf_tmp, buff, len)) {
		return -EFAULT;
	}
	if (buf_tmp[0] == '1')
		himax_ic_reset(chip_info, false, false);
	else if (buf_tmp[0] == '2')
		himax_ic_reset(chip_info, false, true);
	else if (buf_tmp[0] == '3')
		himax_ic_reset(chip_info, true, false);
	else if (buf_tmp[0] == '4')
		himax_ic_reset(chip_info, true, true);
	else if ((buf_tmp[0] == 'z') && (buf_tmp[1] == 'r')) {
		himax_mcu_0f_operation_check(0);
		himax_mcu_0f_operation_check(1);
	} else if ((buf_tmp[0] == 'z') && (buf_tmp[1] == 'w'))
		himax_mcu_0f_operation_dirly();
	return len;
}

static size_t hx83102j_proc_sense_on_off_write(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	char buf[80] = {0};


	if (len >= 80) {
		TPD_INFO("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	if (buf[0] == '0') {
		himax_sense_off();
		TPD_INFO("Sense off \n");
	} else if (buf[0] == '1') {
		if (buf[1] == 's') {
			himax_sense_on(0x00);
			TPD_INFO("Sense on re-map on, run sram \n");
		} else {
			himax_sense_on(0x01);
			TPD_INFO("Sense on re-map off, run flash \n");
		}
	} else {
		TPD_INFO("Do nothing \n");
	}
	return len;
}

#ifdef CONFIG_OPPO_TP_APK
static void himax_gesture_debug_mode_set(bool on_off)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	char buf[80] = {0};
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x7F;
	tmp_addr[0] = 0xF8;

	if (on_off) {
		switch_algo = buf[0];
		check_point_format = 1;
		tmp_data[3] = 0xA1;
		tmp_data[2] = 0x1A;
		tmp_data[1] = 0xA1;
		tmp_data[0] = 0x1A;
		himax_register_write(tmp_addr, 4, tmp_data, 0);
		TPD_INFO("%s: Report 40 trajectory coordinate points .\n", __func__);
	}  else {
		switch_algo = 0;
		check_point_format = 0;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;

		himax_register_write(tmp_addr, 4, tmp_data, 0);
		TPD_INFO("%s: close FW enter algorithm switch.\n", __func__);
	}
}


static void himax_debug_mode_set(bool on_off)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	char buf[80] = {0};
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x7F;
	tmp_addr[0] = 0xF8;

	if (on_off) {
		switch_algo = buf[0];
		check_point_format = 0;
		tmp_data[3] = 0xA5;
		tmp_data[2] = 0x5A;
		tmp_data[1] = 0xA5;
		tmp_data[0] = 0x5A;
		himax_register_write(tmp_addr, 4, tmp_data, 0);
		TPD_INFO("%s: open FW enter algorithm switch.\n", __func__);
	}  else {
		switch_algo = 0;
		check_point_format = 0;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;

		himax_register_write(tmp_addr, 4, tmp_data, 0);
		TPD_INFO("%s: close FW enter algorithm switch.\n", __func__);
	}
}

static void himax_debug_sta_judge(struct chip_data_hx83102j *chip_info)
{
	static struct himax_fw_debug_info last_sta;
	struct himax_fw_debug_info sta;

	memcpy(&sta, &hx_touch_data->hx_state_info[3], sizeof(sta));

	if (last_sta.recal0 != sta.recal0) {
		if (sta.recal0) {
			log_buf_write(private_ts, 1);
		} else {
			log_buf_write(private_ts, 2);
		}
	}

	if (last_sta.recal1 != sta.recal1) {
		if (sta.recal1) {
			log_buf_write(private_ts, 4);
		} else {
			log_buf_write(private_ts, 3);
		}
	}

	if (last_sta.paseline != sta.paseline) {
		if (sta.paseline) {
			log_buf_write(private_ts, 5);
		} else {
		}
	}

	if (last_sta.palm != sta.palm) {
		if (sta.palm) {
			log_buf_write(private_ts, 7);
		} else {
			log_buf_write(private_ts, 6);
		}
	}
	if (last_sta.idle != sta.idle) {
		if (sta.idle) {
			log_buf_write(private_ts, 9);
		} else {
			log_buf_write(private_ts, 8);
		}
	}

	if (last_sta.water != sta.water) {
		if (sta.water) {
			log_buf_write(private_ts, 11);
		} else {
			log_buf_write(private_ts, 10);
		}
	}

	if (last_sta.hopping != sta.hopping) {
		if (sta.hopping) {
			log_buf_write(private_ts, 13);
		} else {
			log_buf_write(private_ts, 12);
		}
	}

	if (last_sta.noise != sta.noise) {
		if (sta.noise) {
			log_buf_write(private_ts, 15);
		} else {
			log_buf_write(private_ts, 14);
		}
	}

	if (last_sta.glove != sta.glove) {
		if (sta.glove) {
			log_buf_write(private_ts, 17);
		} else {
			log_buf_write(private_ts, 16);
		}
	}

	if (last_sta.border != sta.border) {
		if (sta.border) {
			log_buf_write(private_ts, 19);
		} else {
			log_buf_write(private_ts, 18);
		}
	}

	if (last_sta.vr != sta.vr) {
		if (sta.vr) {
			log_buf_write(private_ts, 21);
		} else {
			log_buf_write(private_ts, 20);
		}
	}

	if (last_sta.big_small != sta.big_small) {
		if (sta.big_small) {
			log_buf_write(private_ts, 23);
		} else {
			log_buf_write(private_ts, 22);
		}
	}

	if (last_sta.one_block != sta.one_block) {
		if (sta.one_block) {
			log_buf_write(private_ts, 25);
		} else {
			log_buf_write(private_ts, 24);
		}
	}

	if (last_sta.blewing != sta.blewing) {
		if (sta.blewing) {
			log_buf_write(private_ts, 27);
		} else {
			log_buf_write(private_ts, 26);
		}
	}

	if (last_sta.thumb_flying != sta.thumb_flying) {
		if (sta.thumb_flying) {
			log_buf_write(private_ts, 29);
		} else {
			log_buf_write(private_ts, 28);
		}
	}

	if (last_sta.border_extend != sta.border_extend) {
		if (sta.border_extend) {
			log_buf_write(private_ts, 31);
		} else {
			log_buf_write(private_ts, 30);
		}
	}

	memcpy(&last_sta, &sta, sizeof(last_sta));

	if (tp_debug > 0) {
		TPD_INFO("The sta  is = 0x%02X,0x%02X\n",
				 hx_touch_data->hx_state_info[3],
				 hx_touch_data->hx_state_info[4]);
	}

	return;
}


#endif
#if defined(HX_PEN_V2)
#define READ_VAR_BIT(var, nb)			(((var) >> (nb)) & 0x1)
static bool wgp_pen_id_crc(uint8_t *p_id)
{
	uint64_t pen_id, input;
	uint8_t hash_id, devidend;
	uint8_t pol = 0x43;
	int i = 0;

	pen_id = (uint64_t)p_id[0] | ((uint64_t)p_id[1] << 8) |
		((uint64_t)p_id[2] << 16) | ((uint64_t)p_id[3] << 24) |
		((uint64_t)p_id[4] << 32) | ((uint64_t)p_id[5] << 40) |
		((uint64_t)p_id[6] << 48);
	hash_id = p_id[7];


	input = pen_id << 6;
	devidend = input >> (44 + 6);

	for (i = (44 + 6 - 1); i >= 0; i--) {
		if (READ_VAR_BIT(devidend, 6))
			devidend = devidend ^ pol;
		devidend = devidend << 1 | READ_VAR_BIT(input, i);
	}
	if (READ_VAR_BIT(devidend, 6))
		devidend = devidend ^ pol;


	if (devidend == hash_id) {
		return 1;
	}

	return 0;
}
#endif
static int hx83102j_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
	int i, x, y, z = 1, obj_attention = 0;

	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	char *buf;
	uint16_t mutual_num;
	uint16_t self_num;
	int ret = 0;
	int check_sum_cal;
	int ts_status = HX_REPORT_COORD;
	int hx_point_num;
	uint8_t hx_state_info_pos;
	#if defined(HX_ALG_OVERLAY)
	uint8_t p_hover = 0, p_btn = 0, p_btn2 = 0;
	int8_t p_tilt_x = 0, p_tilt_y = 0;
	int p_x = 0, p_y = 0, p_w = 0;
	int base = 0;
#if defined(HX_PEN_V2)
	uint8_t p_id[8];
	uint8_t p_id_en = 0, p_id_sel = 0;
#endif
#endif
	TPD_INFO("%s: enter!MMCDEBUG 555\n", __func__);
	if (!hx_touch_data) {
		TPD_INFO("%s:%d hx_touch_data is NULL\n", __func__, __LINE__);
	}

	if (!hx_touch_data->hx_coord_buf) {
		TPD_INFO("%s:%d hx_touch_data->hx_coord_buf is NULL\n", __func__, __LINE__);
		return 0;
	}

	buf = kzalloc(sizeof(char) * 128, GFP_KERNEL);
	if (!buf) {
		TPD_INFO("%s:%d buf kzalloc error\n", __func__, __LINE__);
		return -ENOMEM;
	}

	himax_burst_enable(0);
	if (g_diag_command)
		ret = himax_read_event_stack(buf, 128);
	else
		ret = himax_read_event_stack(buf, hx_touch_data->touch_info_size);
	if (!ret) {
		TPD_INFO("%s: can't read data from chip in normal!\n", __func__);
		goto checksum_fail;
	}

		himax_log_touch_data(buf, hx_touch_data);


	check_sum_cal = himax_checksum_cal(chip_info, buf, ts_status);
	if (check_sum_cal == CHECKSUM_FAIL) {
		goto checksum_fail;
	} else if (check_sum_cal == ERR_WORK_OUT) {
		goto err_workqueue_out;
	} else if (check_sum_cal == WORK_OUT) {
		goto workqueue_out;
	}


	hx_state_info_pos = hx_touch_data->touch_info_size - 6;
#if defined(HX_ALG_OVERLAY)
	hx_state_info_pos -= STYLUS_INFO_SZ;
	base = hx_touch_data->touch_info_size - STYLUS_INFO_SZ;
#if defined(HX_PEN_V2)
	p_x = hx_touch_data->hx_coord_buf[base] << 8
		| hx_touch_data->hx_coord_buf[base + 1];
	p_y = (hx_touch_data->hx_coord_buf[base + 2] << 8
		| hx_touch_data->hx_coord_buf[base + 3]);
	p_w = (hx_touch_data->hx_coord_buf[base + 4] << 8
		| hx_touch_data->hx_coord_buf[base + 5]);
	p_tilt_x = (int8_t)hx_touch_data->hx_coord_buf[base + 6];
	p_tilt_y = (int8_t)hx_touch_data->hx_coord_buf[base + 7];
	p_hover = hx_touch_data->hx_coord_buf[base + 8] & 0x01;
	p_btn = hx_touch_data->hx_coord_buf[base + 8] & 0x02;
	p_btn2 = hx_touch_data->hx_coord_buf[base + 8] & 0x04;
	p_id_en = hx_touch_data->hx_coord_buf[base + 8] & 0x08;
	if (!p_id_en) {
	} else {
		p_id_sel =
		(hx_touch_data->hx_coord_buf[base + 8] & 0xF0) >> 4;
		p_id[p_id_sel*2] =
		hx_touch_data->hx_coord_buf[base + 9];
		p_id[p_id_sel*2 + 1] =
		hx_touch_data->hx_coord_buf[base + 10];

		if (p_id_sel == 3) {
			ret = wgp_pen_id_crc(p_id);
			if (!ret)
				TPD_INFO("Pen_ID crc not match\n");
		}
	}
#else
	p_x = hx_touch_data->hx_coord_buf[base] << 8
		| hx_touch_data->hx_coord_buf[base + 1];
	p_y = (hx_touch_data->hx_coord_buf[base + 2] << 8
		| hx_touch_data->hx_coord_buf[base + 3]);
	p_w = (hx_touch_data->hx_coord_buf[base + 4] << 8
		| hx_touch_data->hx_coord_buf[base + 5]);
	p_tilt_x = (int8_t)hx_touch_data->hx_coord_buf[base + 6];
	p_hover = hx_touch_data->hx_coord_buf[base + 7];
	p_btn = hx_touch_data->hx_coord_buf[base + 8];
	p_btn2 = hx_touch_data->hx_coord_buf[base + 9];
	p_tilt_y = (int8_t)hx_touch_data->hx_coord_buf[base + 10];
#endif

#endif
	if(ts_status == HX_REPORT_COORD) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0], hx_touch_data->touch_info_size);
		if(buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF) {
			memcpy(hx_touch_data->hx_state_info, &buf[hx_state_info_pos], 5);
#ifdef CONFIG_OPPO_TP_APK
			if (chip_info->debug_mode_sta) {
				himax_debug_sta_judge(chip_info);
			}
#endif
		} else {
			memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));
		}
	}
	if (g_diag_command) {
		mutual_num = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;
		self_num = chip_info->hw_res->tx_num + chip_info->hw_res->rx_num;
		TPD_INFO("hx_touch_data->touch_all_size= %d hx_touch_data->touch_info_size = %d, %d\n", \
				 hx_touch_data->touch_all_size, hx_touch_data->touch_info_size, hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
		memcpy(hx_touch_data->hx_rawdata_buf, &buf[hx_touch_data->touch_info_size], hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
		if (!diag_check_sum(hx_touch_data)) {
			goto err_workqueue_out;
		}
		diag_parse_raw_data(hx_touch_data, mutual_num, self_num, g_diag_command, hx_touch_data->diag_mutual, diag_self);
	}

	if (hx_touch_data->hx_coord_buf[hx_touch_info_point_cnt] == 0xff)
		hx_point_num = 0;
	else
		hx_point_num = hx_touch_data->hx_coord_buf[hx_touch_info_point_cnt] & 0x0f;


	for (i = 0; i < private_ts->max_num; i++) {
		x = hx_touch_data->hx_coord_buf[i * 4] << 8 | hx_touch_data->hx_coord_buf[i * 4 + 1];
		y = (hx_touch_data->hx_coord_buf[i * 4 + 2] << 8 | hx_touch_data->hx_coord_buf[i * 4 + 3]);
		z = hx_touch_data->hx_coord_buf[i + 4 * max_num];
		if(x >= 0 && x <= private_ts->resolution_info.max_x && y >= 0 && y <= private_ts->resolution_info.max_y) {
			points[i].x = x;
			points[i].y = y;
			points[i].width_major = z;
			points[i].touch_major = z;
			points[i].status = 1;
			obj_attention = obj_attention | (0x0001 << i);
		}
	}


checksum_fail:
	if (buf) {
		kfree(buf);
		buf = NULL;
	}
	return obj_attention;
err_workqueue_out:
workqueue_out:
	if (buf) {
		kfree(buf);
		buf = NULL;
	}

	return -EINVAL;
}

static int hx83102j_ftm_process(void *chip_data)
{
#ifdef HX_RST_PIN_FUNC
	hx83102j_resetgpio_set(g_chip_info->hw_res, false);
#endif
	return 0;
}

static int hx83102j_get_vendor(void *chip_data, struct panel_info *panel_data)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	chip_info->tp_type = panel_data->tp_type;
	chip_info->p_tp_fw = &panel_data->tp_fw;
	TPD_INFO("chip_info->tp_type = %d, panel_data->test_limit_name = %s, panel_data->fw_name = %s\n",
			 chip_info->tp_type, panel_data->test_limit_name, panel_data->fw_name);
	return 0;
}


static int hx83102j_get_chip_info(void *chip_data)
{
	return 1;
}

/**
 * hx83102j_get_fw_id -   get device fw id.
 * @chip_info: struct include i2c resource.
 * Return fw version result.
 */
static uint32_t hx83102j_get_fw_id(struct chip_data_hx83102j *chip_info)
{
	uint32_t current_firmware = 0;
	uint8_t cmd[4];
	uint8_t data[64];

	cmd[3] = 0x10;  /* oppo fw id bin address : 0xc014   , 49172    Tp ic address : 0x 10007014*/
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x14;
	himax_register_read(cmd, 4, data, false);

	TPD_DEBUG("%s : data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n", __func__, data[0], data[1], data[2], data[3]);

	current_firmware = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	TPD_INFO("CURRENT_FIRMWARE_ID = 0x%x\n", current_firmware);

	return current_firmware;
}


/*
static void __init get_lcd_vendor(void)
{
	if (strstr(boot_command_line, "1080p_dsi_vdo-1-fps")) {
		g_lcd_vendor = 1;
	} else if (strstr(boot_command_line, "1080p_dsi_vdo-2-fps")) {
		g_lcd_vendor = 2;
	} else if (strstr(boot_command_line, "1080p_dsi_vdo-3-fps")) {
		g_lcd_vendor = 3;
	} else if (strstr(boot_command_line, "1080p_dsi_vdo-7-fps")) {
		g_lcd_vendor = 7;
	} else if (strstr(boot_command_line, "1080p_dsi_vdo-8-fps")) {
		g_lcd_vendor = 8;
	} else if (strstr(boot_command_line, "1080p_dsi_vdo-9-fps")) {
		g_lcd_vendor = 9;
	}
}*/


static fw_check_state hx83102j_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;


	panel_data->tp_fw = hx83102j_get_fw_id(chip_info);


	return FW_NORMAL;
}

static u32 hx83102j_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
	if ((gesture_enable == 1) && is_suspended) {
		return IRQ_GESTURE;
	} else {
		return IRQ_TOUCH;
	}
}
/*
static int hx83102j_reset_for_prepare(void *chip_data)
{
	int ret = -1;
	//int i2c_error_number = 0;
	//struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	TPD_INFO("%s.\n", __func__);
	//hx83102j_resetgpio_set(chip_info->hw_res, true);

	return ret;
}
*/
/*
static void hx83102j_resume_prepare(void *chip_data)
{
	//hx83102j_reset_for_prepare(chip_data);
	#ifdef HX_ZERO_FLASH
	TPD_DETAIL("It will update fw,if there is power-off in suspend!\n");

	g_zero_event_count = 0;

	hx83102j_enable_interrupt(g_chip_info, false);

	// trigger reset
	//hx83102j_resetgpio_set(g_chip_info->hw_res, false);
	//hx83102j_resetgpio_set(g_chip_info->hw_res, true);

	g_core_fp.fp_0f_operation_dirly();
	g_core_fp.fp_reload_disable(0);
	himax_sense_on(0x00);
	// need_modify
	// report all leave event
	//himax_report_all_leave_event(private_ts);

	hx83102j_enable_interrupt(g_chip_info, true);
	#endif
}
*/
static void hx83102j_exit_esd_mode(void *chip_data)
{
	TPD_INFO("exit esd mode ok\n");
	return;
}

/*
 * return success: 0 ; fail : negative
 */
static int hx83102j_reset(void *chip_data)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	int ret = 0;
	int load_fw_times = 10;

	TPD_INFO("%s.\n", __func__);

	if (!chip_info->first_download_finished) {
		TPD_INFO("%s:First download has not finished, don't do reset.\n", __func__);
		return 0;
	}

	g_zero_event_count = 0;


	hx_esd_reset_activate = 0;

#ifdef HX_ZERO_FLASH
	mutex_lock(&(g_chip_info->fw_update_lock));
#endif


	disable_irq_nosync(chip_info->hx_irq);


	do {
		load_fw_times--;
		g_core_fp.fp_0f_operation_dirly();
		ret = g_core_fp.fp_reload_disable(0);
	} while (!ret && load_fw_times > 0);

	if (!load_fw_times) {
		TPD_INFO("%s: load_fw_times over 10 times\n", __func__);
	}
	himax_sense_on(0x00);

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	enable_irq(chip_info->hx_irq);
#endif
#ifdef HX_ZERO_FLASH
	mutex_unlock(&(g_chip_info->fw_update_lock));
#endif

	return ret;
}

void himax_ultra_enter(void)
{
	uint8_t tmp_data[4];
	int rtimes = 0;

	TPD_INFO("%s:entering\n", __func__);

	/* 34 -> 11 */
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:1/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0x11;
		if (himax_bus_write(0x34, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (himax_bus_read(0x34, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x34, correct 0x11 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0x11);

	/* 33 -> 33 */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:2/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0x33;
		if (himax_bus_write(0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (himax_bus_read(0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x33, correct 0x33 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0x33);

	/* 34 -> 22 */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:3/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0x22;
		if (himax_bus_write(0x34, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (himax_bus_read(0x34, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x34, correct 0x22 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0x22);

	/* 33 -> AA */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:4/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0xAA;
		if (himax_bus_write(0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (himax_bus_read(0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x33, correct 0xAA = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0xAA);

	/* 33 -> 33 */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:5/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0x33;
		if (himax_bus_write(0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (himax_bus_read(0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x33, correct 0x33 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0x33);

	/* 33 -> AA */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:6/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0xAA;
		if (himax_bus_write(0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (himax_bus_read(0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x33, correct 0xAA = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0xAA);

	TPD_INFO("%s:END\n", __func__);
}

static int hx83102j_enable_black_gesture(struct chip_data_hx83102j *chip_info, bool enable)
{
	int ret = 0;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);
	static char ovl_done = 0;
	int retry_cnt = 0;
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint8_t back_data[4] = {0};

	TPD_INFO("%s:enable=%d, ts->is_suspended=%d \n", __func__, enable, ts->is_suspended);

	if (ts->is_suspended) {
		/*status in suspend*/
		TPD_INFO("%s: now is in suspend!\n", __func__);
		if (ovl_done == 0) {
			retry_cnt = 0;
			do {
				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x10;
				tmp_data[3] = 0xA5;
				tmp_data[2] = 0x5A;
				tmp_data[1] = 0xA5;
				tmp_data[0] = 0x5A;
				himax_flash_write_burst(tmp_addr, tmp_data);
				back_data[3] = 0xA5;
				back_data[2] = 0x5A;
				back_data[1] = 0xA5;
				back_data[0] = 0x5A;
				himax_register_read(tmp_addr, 4, tmp_data, false);
				TPD_INFO("%s: tmp_data[0] = 0x%02X, retry_cnt=%d \n", __func__, tmp_data[0], retry_cnt);
				retry_cnt++;
			} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1] ||
							tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);

			if (gflagautotest == 1) {
				TPD_INFO("%s:Testing, skip overlay\n", __func__);
			}
#ifdef HX_CODE_OVERLAY
			else
				hx83102j_0f_overlay(2, 1);
#endif
		}
		if (enable) {
			TPD_INFO("%s: now entering SMWP mode!\n", __func__);
			if (ovl_done == 1) {
#ifdef HX_RST_PIN_FUNC
				hx83102j_resetgpio_set(chip_info->hw_res, true);
				hx83102j_resetgpio_set(chip_info->hw_res, false);
				hx83102j_resetgpio_set(chip_info->hw_res, true);
#else
				g_core_fp.fp_sys_reset();
#endif
				 himax_hx83102j_reload_to_active();
			}
#ifdef CONFIG_OPPO_TP_APK
			if (chip_info->debug_gesture_sta) {
					himax_gesture_debug_mode_set(true);
			}
#endif

		} else {
			/* psensor mode*/
			TPD_INFO("%s: now entering utlra mode!\n", __func__);
			retry_cnt = 0;
			do {
				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x10;
				tmp_data[3] = 0x00;
				tmp_data[2] = 0x00;
				tmp_data[1] = 0x00;
				tmp_data[0] = 0x00;
				himax_flash_write_burst(tmp_addr, tmp_data);
				back_data[3] = 0x00;
				back_data[2] = 0x00;
				back_data[1] = 0x00;
				back_data[0] = 0x00;
				himax_register_read(tmp_addr, 4, tmp_data, false);
				TPD_INFO("%s: tmp_data[0] = 0x%02X, retry_cnt=%d \n", __func__, tmp_data[0], retry_cnt);
				retry_cnt++;
			} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1] ||
						tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);
			himax_ultra_enter();
		}
		if (ovl_done == 0)
			ovl_done = 1;
	} else {
		TPD_INFO("%s: Leave suspend and back to normal!\n", __func__);
		/* return back to normal*/
		retry_cnt = 0;
		do {
				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x10;
				tmp_data[3] = 0x00;
				tmp_data[2] = 0x00;
				tmp_data[1] = 0x00;
				tmp_data[0] = 0x00;
				himax_flash_write_burst(tmp_addr, tmp_data);
				back_data[3] = 0x00;
				back_data[2] = 0x00;
				back_data[1] = 0x00;
				back_data[0] = 0x00;
				himax_register_read(tmp_addr, 4, tmp_data, false);
				TPD_INFO("%s: tmp_data[0] = 0x%02X, retry_cnt=%d \n", __func__, tmp_data[0], retry_cnt);
				retry_cnt++;
		} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1]	||
					tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);
		if (!chip_info->first_download_finished) {
			 TPD_INFO("%s:need do overlay.\n", __func__);

			if (gflagautotest == 1) {
				TPD_INFO("%s:Testing, skip overlay\n", __func__);
				gflagautotest = 0;
			}
#if defined(HX_CODE_OVERLAY)
			else
				hx83102j_0f_overlay(3, 1);
#endif
		}
		ovl_done = 0;
	}
	return ret;
}
/*
static int hx83102j_enable_edge_limit(struct chip_data_hx83102j *chip_info, bool enable)
{
	int ret = 0;
	return ret;
}
*/

static int hx83102j_enable_charge_mode(struct chip_data_hx83102j *chip_info, bool enable)
{
	int ret = 0;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	TPD_INFO("%s, charge mode enable = %d\n", __func__, enable);

	/*Enable:0x10007F38 = 0xA55AA55A  */
	if (enable) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x38;
		tmp_data[3] = 0xA5;
		tmp_data[2] = 0x5A;
		tmp_data[1] = 0xA5;
		tmp_data[0] = 0x5A;
		himax_flash_write_burst(tmp_addr, tmp_data);
	} else {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x38;
		tmp_data[3] = 0x77;
		tmp_data[2] = 0x88;
		tmp_data[1] = 0x77;
		tmp_data[0] = 0x88;
		himax_flash_write_burst(tmp_addr, tmp_data);
	}

	return ret;
}

/*on = 1:on   0:off */
static int hx83102j_jitter_switch (struct chip_data_hx83102j *chip_info, bool on)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int rtimes = 0;
	int ret = 0;

	TPD_INFO("%s:entering\n", __func__);

	if (!on) {
		do {
			if (rtimes > 10) {
				TPD_INFO("%s:retry over 10, jitter off failed!\n", __func__);
				TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x5A, 0xA5, 0x5A, 0xA5\n", __func__);
				ret = -1;
				break;
			}

			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0xE0;
			tmp_data[3] = 0xA5;
			tmp_data[2] = 0x5A;
			tmp_data[1] = 0xA5;
			tmp_data[0] = 0x5A;
			himax_flash_write_burst(tmp_addr, tmp_data);

			himax_register_read(tmp_addr, 4, tmp_data, false);

			TPD_INFO("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
					 rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
			rtimes++;
		} while (tmp_data[3] != 0xA5 || tmp_data[2] != 0x5A
				 || tmp_data[1] != 0xA5 || tmp_data[0] != 0x5A);
		TPD_INFO("%s:jitter off success!\n", __func__);
	} else {
		do {
			if (rtimes > 10) {
				TPD_INFO("%s:retry over 10, jitter on failed!\n", __func__);
				TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x00, 0x00, 0x00, 0x00\n", __func__);
				ret = -1;
				break;
			}

			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0xE0;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			himax_flash_write_burst(tmp_addr, tmp_data);

			himax_register_read(tmp_addr, 4, tmp_data, false);

			TPD_INFO("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
					 rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
			rtimes++;
		} while (tmp_data[3] == 0xA5 && tmp_data[2] == 0x5A
				 && tmp_data[1] == 0xA5 && tmp_data[0] == 0x5A);
		TPD_INFO("%s:jitter on success!\n", __func__);
	}
	TPD_INFO("%s:END\n", __func__);
	return ret;
}

static int hx83102j_enable_headset_mode(struct chip_data_hx83102j *chip_info, bool enable)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int rtimes = 0;
	int ret = 0;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	if (ts->headset_pump_support) {
		if (enable) {/* insert headset */
			do {
				if (rtimes > 10) {
					TPD_INFO("%s:insert headset failed!\n", __func__);
					TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x5A, 0xA5, 0x5A, 0xA5\n", __func__);
					ret = -1;
					break;
				}

				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0xE8;
				tmp_data[3] = 0xA5;
				tmp_data[2] = 0x5A;
				tmp_data[1] = 0xA5;
				tmp_data[0] = 0x5A;
				himax_flash_write_burst(tmp_addr, tmp_data);

				himax_register_read(tmp_addr, 4, tmp_data, false);
				TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
						   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
				rtimes++;
			} while (tmp_data[3] != 0xA5 || tmp_data[2] != 0x5A
					 || tmp_data[1] != 0xA5 || tmp_data[0] != 0x5A);

			TPD_INFO("%s:insert headset success!\n", __func__);
		} else {/* remove headset  */
			do {
				if (rtimes > 10) {
					TPD_INFO("%s:remove headset failed!\n", __func__);
					TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x5A, 0xA5, 0x5A, 0xA5\n", __func__);
					ret = -1;
					break;
				}

				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0xE8;
				tmp_data[3] = 0x00;
				tmp_data[2] = 0x00;
				tmp_data[1] = 0x00;
				tmp_data[0] = 0x00;
				himax_flash_write_burst(tmp_addr, tmp_data);

				himax_register_read(tmp_addr, 4, tmp_data, false);
				TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
						   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
				rtimes++;
			} while (tmp_data[3] != 0x00 || tmp_data[2] != 0x00
					 || tmp_data[1] != 0x00 || tmp_data[0] != 0x00);

			TPD_INFO("%s:remove headset success!\n", __func__);
		}
	}
	return ret;
}

/* mode = 0:off   1:normal   2:turn right    3:turn left*/
static int hx83102j_rotative_switch(struct chip_data_hx83102j *chip_info, int mode)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int rtimes = 0;
	int ret = 0;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	TPD_DETAIL("%s:entering\n", __func__);

	if (ts->fw_edge_limit_support) {
		if (mode == 1 || VERTICAL_SCREEN == chip_info->touch_direction) {/* vertical */
			do {
				if (rtimes > 10) {
					TPD_INFO("%s:rotative normal failed!\n", __func__);
					TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x5A, 0xA5, 0x5A, 0xA5\n", __func__);
					ret = -1;
					break;
				}

				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x3C;
				tmp_data[3] = 0xA5;
				tmp_data[2] = 0x5A;
				tmp_data[1] = 0xA5;
				tmp_data[0] = 0x5A;
				himax_flash_write_burst(tmp_addr, tmp_data);

				himax_register_read(tmp_addr, 4, tmp_data, false);
				TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
						   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
				rtimes++;
			} while (tmp_data[3] != 0xA5 || tmp_data[2] != 0x5A
					 || tmp_data[1] != 0xA5 || tmp_data[0] != 0x5A);

			TPD_INFO("%s:rotative normal success!\n", __func__);

		} else {
			rtimes = 0;
			if (LANDSCAPE_SCREEN_270 == chip_info->touch_direction) { /*turn right*/
				do {
					if (rtimes > 10) {
						TPD_INFO("%s:rotative right failed!\n", __func__);
						TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x3A, 0xA3, 0x3A, 0xA3\n", __func__);
						ret = -1;
						break;
					}

					tmp_addr[3] = 0x10;
					tmp_addr[2] = 0x00;
					tmp_addr[1] = 0x7F;
					tmp_addr[0] = 0x3C;
					tmp_data[3] = 0xA3;
					tmp_data[2] = 0x3A;
					tmp_data[1] = 0xA3;
					tmp_data[0] = 0x3A;
					himax_flash_write_burst(tmp_addr, tmp_data);

					himax_register_read(tmp_addr, 4, tmp_data, false);

					TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
							   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
					rtimes++;
				} while (tmp_data[3] != 0xA3 || tmp_data[2] != 0x3A
						 || tmp_data[1] != 0xA3 || tmp_data[0] != 0x3A);

				TPD_INFO("%s:rotative right success!\n", __func__);

			} else if (LANDSCAPE_SCREEN_90 == chip_info->touch_direction) { /*turn left*/
				do {
					if (rtimes > 10) {
						TPD_INFO("%s:rotative left failed!\n", __func__);
						TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x1A, 0xA1, 0x1A, 0xA1\n", __func__);
						ret = -1;
						break;
					}

					tmp_addr[3] = 0x10;
					tmp_addr[2] = 0x00;
					tmp_addr[1] = 0x7F;
					tmp_addr[0] = 0x3C;
					tmp_data[3] = 0xA1;
					tmp_data[2] = 0x1A;
					tmp_data[1] = 0xA1;
					tmp_data[0] = 0x1A;
					himax_flash_write_burst(tmp_addr, tmp_data);

					himax_register_read(tmp_addr, 4, tmp_data, false);

					TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n",
						__func__, rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
					rtimes++;
				} while (tmp_data[3] != 0xA1 || tmp_data[2] != 0x1A
						 || tmp_data[1] != 0xA1 || tmp_data[0] != 0x1A);

				TPD_INFO("%s:rotative left success!\n", __func__);
			}
		}
	} else {
		if (mode) {/*open*/
			do {
				if (rtimes > 10) {
					TPD_INFO("%s:open edge limit failed!\n", __func__);
					TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x5A, 0xA5, 0x5A, 0xA5\n", __func__);
					ret = -1;
					break;
				}

				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x3C;
				tmp_data[3] = 0xA5;
				tmp_data[2] = 0x5A;
				tmp_data[1] = 0xA5;
				tmp_data[0] = 0x5A;
				himax_flash_write_burst(tmp_addr, tmp_data);

				himax_register_read(tmp_addr, 4, tmp_data, false);
				TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
						   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
				rtimes++;
			} while (tmp_data[3] != 0xA5 || tmp_data[2] != 0x5A
					 || tmp_data[1] != 0xA5 || tmp_data[0] != 0x5A);

			TPD_INFO("%s:open edge limit success!\n", __func__);

		} else {/*close*/
			do {
				if (rtimes > 10) {
					TPD_INFO("%s:close edge limit failed!\n", __func__);
					TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x9A, 0xA9, 0x9A, 0xA9\n", __func__);
					ret = -1;
					break;
				}

				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x3C;
				tmp_data[3] = 0xA9;
				tmp_data[2] = 0x9A;
				tmp_data[1] = 0xA9;
				tmp_data[0] = 0x9A;
				himax_flash_write_burst(tmp_addr, tmp_data);

				himax_register_read(tmp_addr, 4, tmp_data, false);
				TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
						   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
				rtimes++;
			} while (tmp_data[3] != 0xA9 || tmp_data[2] != 0x9A
					 || tmp_data[1] != 0xA9 || tmp_data[0] != 0x9A);

			TPD_INFO("%s:close edge limit success!\n", __func__);
		}
	}
	TPD_DETAIL("%s:END\n", __func__);
	return ret;
}

static int hx83102j_mode_switch(void *chip_data, work_mode mode, int flag)
{
	int ret = -1;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	switch(mode) {
	case MODE_NORMAL:
		ret = hx83102j_configuration_init(chip_info, true);
		if (ret < 0) {
			TPD_INFO("%s: hx83102j configuration init failed.\n", __func__);
			return ret;
		}

		break;

	case MODE_SLEEP:
		/*device control: sleep mode*/
		ret = hx83102j_configuration_init(chip_info, false);
		if (ret < 0) {
			TPD_INFO("%s: hx83102j configuration init failed.\n", __func__);
			return ret;
		}

		break;

	case MODE_GESTURE:
		ret = hx83102j_enable_black_gesture(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("%s: hx83102j enable gesture failed.\n", __func__);
			return ret;
		}

		break;

	case MODE_GLOVE:

		break;

	case MODE_EDGE:
		/*ret = hx83102j_enable_edge_limit(chip_info, flag);*/
		ret = hx83102j_rotative_switch(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("%s: hx83102j enable edg & corner limit failed.\n", __func__);
			return ret;
		}

		break;

	case MODE_CHARGE:
		ret = hx83102j_enable_charge_mode(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("%s: enable charge mode : %d failed\n", __func__, flag);
		}
		break;

	case MODE_HEADSET:
		ret = hx83102j_enable_headset_mode(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("%s: enable headset mode : %d failed\n", __func__, flag);
		}
		break;

	case MODE_GAME:
		ret = hx83102j_jitter_switch(chip_info, !flag);
		if (ret < 0) {
			TPD_INFO("%s: enable game mode : %d failed\n", __func__, !flag);
		}
		break;

	default:
		TPD_INFO("%s: Wrong mode.\n", __func__);
	}

	return ret;
}

void himax_set_SMWP_enable(uint8_t SMWP_enable, bool suspended)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t back_data[4];
	uint8_t retry_cnt = 0;

	himax_sense_off();

	/* Enable:0x10007F10 = 0xA55AA55A */
	do {
		if (SMWP_enable) {
			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0x10;
			tmp_data[3] = 0xA5;
			tmp_data[2] = 0x5A;
			tmp_data[1] = 0xA5;
			tmp_data[0] = 0x5A;
			himax_flash_write_burst(tmp_addr, tmp_data);
			back_data[3] = 0XA5;
			back_data[2] = 0X5A;
			back_data[1] = 0XA5;
			back_data[0] = 0X5A;
		} else {
			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0x10;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			himax_flash_write_burst(tmp_addr, tmp_data);
			back_data[3] = 0X00;
			back_data[2] = 0X00;
			back_data[1] = 0X00;
			back_data[0] = 0x00;
		}
		himax_register_read(tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: tmp_data[0]=%d, SMWP_enable=%d, retry_cnt=%d \n", __func__, tmp_data[0], SMWP_enable, retry_cnt);
		retry_cnt++;
	} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1] || tmp_data[0] != back_data[0]) && retry_cnt < 10);

	himax_sense_on(0);
}

int hx_id_match_oppo(int get_id)
{
	int result = 0xff;
	int i = 0;

	for (i = 0; i < HX_GESUTRE_SZ; i++) {
		if (get_id == hx_common_gesture_id[i]) {
			TPD_INFO("Found it, idx=%d, common=0x%02X, oppo=0x%02X\n",
				i, hx_common_gesture_id[i], hx_oppo_gesture_id[i]);
			result = hx_oppo_gesture_id[i];
			break;
		}
	}

	return result;
}

static int hx83102j_get_gesture_info(void *chip_data, struct gesture_info *gesture)
{
	int i = 0;
	int gesture_sign = 0;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	uint8_t *buf;
	int gest_len;
	int check_fc = 0;
	int check_sum_cal;
	int ts_status = HX_REPORT_SMWP_EVENT;


	buf = kzalloc(hx_touch_data->event_size * sizeof(uint8_t), GFP_KERNEL);
	if (!buf) {
		TPD_INFO("%s:%d kzalloc buf error\n", __func__, __LINE__);
		return -1;
	}

	himax_burst_enable(0);
	if (!himax_read_event_stack(buf, hx_touch_data->event_size)) {
		TPD_INFO("%s: can't read data from chip in gesture!\n", __func__);
		kfree(buf);
		return -1;
	}

	for (i = 0; i < 128; i++) {
		if (!i) {
			printk("%s: gesture buf data\n", __func__);
		}
		printk("%02d ", buf[i]);
		if ((i + 1) % 8 == 0) {
			printk("\n");
		}
		if (i == (128 - 1)) {
			printk("\n");
		}
	}

	check_sum_cal = himax_checksum_cal(chip_info, buf, ts_status);
	if (check_sum_cal == CHECKSUM_FAIL) {
		return -1;
	} else if (check_sum_cal == ERR_WORK_OUT) {
		goto err_workqueue_out;
	}

	for (i = 0; i < 4; i++) {
		if (check_fc == 0) {
			if ((buf[0] != 0x00) && ((buf[0] <= 0xFF))) {
				check_fc = 1;
				gesture_sign = buf[i];
			} else {
				check_fc = 0;

				break;
			}
		} else {
			if (buf[i] != gesture_sign) {
				check_fc = 0;

				break;
			}
		}
	}


	if (buf[GEST_PTLG_ID_LEN] != GEST_PTLG_HDR_ID1 ||
		buf[GEST_PTLG_ID_LEN + 1] != GEST_PTLG_HDR_ID2) {
		goto RET_OUT;
	}

	if (buf[GEST_PTLG_ID_LEN] == GEST_PTLG_HDR_ID1 &&
		buf[GEST_PTLG_ID_LEN + 1] == GEST_PTLG_HDR_ID2) {
		gest_len = buf[GEST_PTLG_ID_LEN + 2];
		if (gest_len > 52) {
			gest_len = 52;
		}


		i = 0;
		gest_pt_cnt = 0;

#ifdef CONFIG_OPPO_TP_APK
		if(check_point_format == 0) {
#endif
			while (i < (gest_len + 1) / 2) {
				if (i == 6) {
					gest_pt_x[gest_pt_cnt] = buf[GEST_PTLG_ID_LEN + 4 + i * 2];
				} else {
					gest_pt_x[gest_pt_cnt] = buf[GEST_PTLG_ID_LEN + 4 + i * 2] * private_ts->resolution_info.max_x / 255;
				}
				gest_pt_y[gest_pt_cnt] = buf[GEST_PTLG_ID_LEN + 4 + i * 2 + 1] * private_ts->resolution_info.max_y / 255;
				i++;

				gest_pt_cnt += 1;
			}
#ifdef CONFIG_OPPO_TP_APK
		} else {
			int j = 0;
			int nn;
			int n = 24;
			int m = 26;
			int pt_num;
			gest_pt_cnt = 40;
			if (private_ts->gesture_buf) {
				pt_num = gest_len + buf[126];
				if (pt_num > 104) {
					pt_num = 104;
				}
				private_ts->gesture_buf[0] = gesture_sign;
				private_ts->gesture_buf[1] = buf[127];

				if (private_ts->gesture_buf[0] == 0x07) {
					for(j = 0; j < gest_len * 2; j = j + 2) {
						private_ts->gesture_buf[3 + j] = buf[n];
						private_ts->gesture_buf[3 + j + 1] = buf[n + 1];
						n = n + 4;
					}

					for(nn = 0; nn < (pt_num - gest_len)   * 2 ; nn = nn + 2) {
						private_ts->gesture_buf[3 + j + nn] = buf[m];
						private_ts->gesture_buf[3 + j + nn + 1] = buf[m + 1];
						m = m + 4;
					}
					private_ts->gesture_buf[2] = pt_num;
				} else {
					private_ts->gesture_buf[2] = gest_len;
					memcpy(&private_ts->gesture_buf[3], &buf[24], 80);
				}
			}
		}
#endif
		if (gesture_sign == hx_common_gesture_id[0])
			gest_pt_cnt = 1;
		if (gest_pt_cnt) {
			gesture->gesture_type = hx_id_match_oppo(gesture_sign);/* id */
			gesture->Point_start.x = gest_pt_x[0];/* start x */
			gesture->Point_start.y = gest_pt_y[0];/* start y */
			gesture->Point_end.x = gest_pt_x[1];/* end x */
			gesture->Point_end.y = gest_pt_y[1];/* end y */
			gesture->Point_1st.x = gest_pt_x[2]; /* 1 */
			gesture->Point_1st.y = gest_pt_y[2];
			gesture->Point_2nd.x = gest_pt_x[3];/* 2 */
			gesture->Point_2nd.y = gest_pt_y[3];
			gesture->Point_3rd.x = gest_pt_x[4];/* 3 */
			gesture->Point_3rd.y = gest_pt_y[4];
			gesture->Point_4th.x = gest_pt_x[5];/* 4 */
			gesture->Point_4th.y = gest_pt_y[5];
			gesture->clockwise = gest_pt_x[6]; /*  1, 0 */

			/*for (i = 0; i < 6; i++)
			   TPD_DEBUG("%d [ %d  %d ]\n", i, gest_pt_x[i], gest_pt_y[i]);*/
		}
	}


RET_OUT:
	if (buf) {
		kfree(buf);
	}
	return 0;

err_workqueue_out:

	return -1;
}

static int hx83102j_power_control(void *chip_data, bool enable)
{
	int ret = 0;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	if (true == enable) {
		/*
		ret = tp_powercontrol_2v8(chip_info->hw_res, true);
		if (ret)
			return -1;
		ret = tp_powercontrol_1v8(chip_info->hw_res, true);
		if (ret)
			return -1;
		*/

	#ifdef HX_RST_PIN_FUNC
		ret = hx83102j_resetgpio_set(chip_info->hw_res, true);
		if (ret)
			return -1;
	#endif
	} else {
	#ifdef HX_RST_PIN_FUNC
		ret = hx83102j_resetgpio_set(chip_info->hw_res, false);
		if (ret)
			return -1;
	#endif
	/*
		ret = tp_powercontrol_1v8(chip_info->hw_res, false);
		if (ret)
			return -1;
		ret = tp_powercontrol_2v8(chip_info->hw_res, false);
		if (ret)
			return -1;
	*/
	}
	return ret;
}
/*
static void store_to_file(int fd, char *format, ...)
{
	va_list args;
	char buf[64] = {0};

	va_start(args, format);
	vsnprintf(buf, 64, format, args);
	va_end(args);

	if (fd >= 0) {
		sys_write(fd, buf, strlen(buf));
	}
}
*/

/*
static int hx83102j_int_pin_test(struct seq_file *s, void *chip_data, struct auto_testdata *syna_testdata, char *g_Test_list_log) 
{
	int eint_status, eint_count = 0, read_gpio_num = 10;

	TPD_INFO("%s, step 0: begin to check INT-GND short item\n", __func__);
	while (read_gpio_num--) {
		msleep(5);
		eint_status = gpio_get_value(syna_testdata->irq_gpio);
		if (eint_status == 1)
			eint_count--;
		else
			eint_count++;
		TPD_INFO("%s eint_count = %d  eint_status = %d\n", __func__, eint_count, eint_status);
	}
	TPD_INFO("TP EINT PIN direct short! eint_count = %d\n", eint_count);
	if (eint_count == 10) {
		TPD_INFO("error :  TP EINT PIN direct short!\n");
		seq_printf(s, "TP EINT direct stort\n");
		hx83102j_nf_fail_write_count += snprintf(g_Test_list_log + hx83102j_nf_fail_write_count, 45, "eint_status is low, TP EINT direct stort, \n");

		eint_count = 0;
		return TEST_FAIL;
	}

	return TEST_PASS;
}


static void hx83102j_auto_test(struct seq_file *s, struct touchpanel_data *ts)
{
	int error_count = 0;
	int ret = THP_AFE_INSPECT_OK;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	char *p_node = NULL;
	char *fw_name_test = NULL;
	char *postfix = "_TEST.img";
	uint8_t copy_len = 0;
	char g_file_name_OK[64];
	char g_file_name_NG[64];
	char *g_Test_list_log = NULL;
	char *g_project_test_info_log = NULL;
	char *g_company_info_log = NULL;
	int i = 0;
	in_self_test = 1;
	g_rslt_data_len = 0;
	fw_name_test = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
	if (fw_name_test == NULL) {
		TPD_INFO("fw_name_test kzalloc error!\n");
		goto RET_OUT;
	}

	ret = himax_self_test_data_init(chip_info);
	if (ret != THP_AFE_INSPECT_OK) {
		TPD_INFO("%s:%d criteria error\n", __func__, __LINE__);
		goto RET_OUT;
	}

	g_1kind_raw_size = 5 * chip_info->hw_res->tx_num * chip_info->hw_res->rx_num * 2;
	g_company_info_log = kcalloc(256, sizeof(char), GFP_KERNEL);
	g_Test_list_log = kcalloc(256, sizeof(char), GFP_KERNEL);
	g_project_test_info_log = kcalloc(256, sizeof(char), GFP_KERNEL);
	hx83102j_nf_fail_write_count = 0;
	g_file_path_ok = kcalloc(256, sizeof(char), GFP_KERNEL);
	if (!g_file_path_ok) {
		TPD_INFO("%s:%d g_file_path_ok kzalloc buf error\n", __func__, __LINE__);
		goto RET_OUT;
	}
	g_file_path_ng = kcalloc(256, sizeof(char), GFP_KERNEL);
	if (!g_file_path_ng) {
		TPD_INFO("%s:%d g_file_path_ng kzalloc buf error\n", __func__, __LINE__);
		goto RET_OUT;
	}

	if (g_rslt_data == NULL) {
		TPD_INFO("g_rslt_data is NULL");
		g_rslt_data = kcalloc(g_1kind_raw_size * hx_criteria_item,
							  sizeof(char), GFP_KERNEL);
		if (!g_rslt_data) {
			TPD_INFO("%s:%d g_rslt_data kzalloc buf error\n", __func__, __LINE__);
			goto RET_OUT;
		}
	} else {
		memset(g_rslt_data, 0x00, g_1kind_raw_size * hx_criteria_item *
			   sizeof(char));
	}

	p_node = strstr(private_ts->panel_data.fw_name, ".");
	copy_len = p_node - private_ts->panel_data.fw_name;
	memcpy(fw_name_test, private_ts->panel_data.fw_name, copy_len);
	strlcat(fw_name_test, postfix, MAX_FW_NAME_LENGTH);
	TPD_INFO("%s : p_node=%s, copy_len=%d,postfix:%s, fw_name: %s\n", __func__, p_node, copy_len, postfix, private_ts->panel_data.fw_name);
	TPD_INFO("%s : fw_name_test is %s\n", __func__, fw_name_test);
#ifdef HX_ZERO_FLASH
	mutex_lock(&(g_chip_info->fw_update_lock));
#endif
	himax_mcu_0f_operation_test_dirly(fw_name_test);
	usleep_range(5000, 5100);
	g_core_fp.fp_reload_disable(0);
	usleep_range(5000, 5100);
	himax_sense_on(0x00);
	himax_read_OPPO_FW_ver(chip_info);

	ret = hx83102j_enable_interrupt(chip_info, false);

	error_count += hx83102j_int_pin_test(s, chip_info, syna_testdata, g_Test_list_log);
	error_count += himax_chip_self_test(s, chip_info, g_Test_list_log);
#ifdef HX_ZERO_FLASH
	mutex_unlock(&(g_chip_info->fw_update_lock));
#endif

	if (error_count) {
		snprintf(g_file_path_ng,
				 (int)(strlen(HX_RSLT_OUT_PATH_NG) + strlen(g_file_name_NG) + 1),
				 "%s%s", HX_RSLT_OUT_PATH_NG, g_file_name_NG);
		hx_test_data_pop_out(chip_info, g_Test_list_log, g_company_info_log, g_project_test_info_log, g_rslt_data, g_file_path_ng);

	} else {
		snprintf(g_file_path_ok,
				 (int)(strlen(HX_RSLT_OUT_PATH_OK) + strlen(g_file_name_OK) + 1),
				 "%s%s", HX_RSLT_OUT_PATH_OK, g_file_name_OK);
		hx_test_data_pop_out(chip_info, g_Test_list_log, g_company_info_log, g_project_test_info_log, g_rslt_data, g_file_path_ok);
	}



	seq_printf(s, "%d errors. %s\n", error_count, error_count ? "" : "All test passed.");
	TPD_INFO(" TP auto test %d error(s). %s\n", error_count, error_count ? "" : "All test passed.");

	TPD_INFO(" Now return to normal FW!\n");
	in_self_test = 0;
	himax_mcu_0f_operation_dirly();
	usleep_range(5000, 5100);
	g_core_fp.fp_reload_disable(0);
	usleep_range(5000, 5100);
	himax_sense_on(0x00);
	himax_read_OPPO_FW_ver(chip_info);
	ret = hx83102j_enable_interrupt(chip_info, true);

RET_OUT:
	in_self_test = 0;
	if (fw_name_test) {
		kfree(fw_name_test);
		fw_name_test = NULL;
	}

	if (hx83102j_nf_inspection_criteria != NULL) {
		for (i = 0; i < hx_criteria_size; i++) {
			if (hx83102j_nf_inspection_criteria[i] != NULL) {
				kfree(hx83102j_nf_inspection_criteria[i]);
				hx83102j_nf_inspection_criteria[i] = NULL;
			}
		}
		kfree(hx83102j_nf_inspection_criteria);
		hx83102j_nf_inspection_criteria = NULL;
		TPD_INFO("Now it have free the hx83102j_nf_inspection_criteria!\n");
	} else {
		TPD_INFO("No Need to free hx83102j_nf_inspection_criteria!\n");
	}

	if (hx83102j_nf_inspt_crtra_flag) {
		kfree(hx83102j_nf_inspt_crtra_flag);
		hx83102j_nf_inspt_crtra_flag = NULL;
	}

	if (g_file_path_ok) {
		kfree(g_file_path_ok);
		g_file_path_ok = NULL;
	}
	if (g_file_path_ng) {
		kfree(g_file_path_ng);
		g_file_path_ng = NULL;
	}
	if (g_Test_list_log) {
		kfree(g_Test_list_log);
		g_Test_list_log = NULL;
	}
	if (g_project_test_info_log) {
		kfree(g_project_test_info_log);
		g_project_test_info_log = NULL;
	}
	if (g_company_info_log) {
		kfree(g_company_info_log);
		g_company_info_log = NULL;
	}
}*/

static void hx83102j_read_debug_data(struct seq_file *s, void *chip_data, int debug_data_type)
{
	uint16_t mutual_num;
	uint16_t self_num;
	uint16_t width;
	int i = 0;
	int j = 0;
	int k = 0;
	int32_t *data_mutual_sram;
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};

	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	if (!chip_info)
		return;

	data_mutual_sram = kzalloc(chip_info->hw_res->tx_num * chip_info->hw_res->rx_num * sizeof(int32_t), GFP_KERNEL);
	if (!data_mutual_sram) {
		goto RET_OUT;
	}

	mutual_num = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;
	self_num = chip_info->hw_res->tx_num + chip_info->hw_res->rx_num; /*don't add KEY_COUNT*/
	width = chip_info->hw_res->rx_num;
	seq_printf(s, "ChannelStart (rx tx) : %4d, %4d\n\n", chip_info->hw_res->rx_num, chip_info->hw_res->tx_num);

	/*start to show debug data*/
	switch (debug_data_type) {
	case DEBUG_DATA_BASELINE:
		seq_printf(s, "Baseline data: \n");
		TPD_INFO("Baseline data: \n");
		break;

	case DEBUG_DATA_RAW:
		seq_printf(s, "Raw data: \n");
		TPD_INFO("Raw data: \n");
		break;

	case DEBUG_DATA_DELTA:
		seq_printf(s, "Delta data: \n");
		TPD_INFO("Delta data: \n");
		break;

	case DEBUG_DATA_DOWN:
		seq_printf(s, "Finger down data: \n");
		TPD_INFO("Finger down data: \n");
		break;

	default :
		seq_printf(s, "No this debug datatype \n");
		TPD_INFO("No this debug datatype \n");
		goto RET_OUT;
		break;
	}
	if (debug_data_type == DEBUG_DATA_DOWN) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xF8;

		tmp_data[3] = 0xA5;
		tmp_data[2] = 0x5A;
		tmp_data[1] = 0xA5;
		tmp_data[0] = 0x5A;
		himax_register_write(tmp_addr, 4, tmp_data, 0);
		himax_diag_register_set(DEBUG_DATA_DELTA);
	} else {
		himax_diag_register_set(debug_data_type);
	}
	TPD_INFO("%s: Start get debug data in DSRAM\n", __func__);
	dsram_flag = true;

	himax_ts_diag_func(chip_info, data_mutual_sram);

	for (j = 0; j < chip_info->hw_res->rx_num; j++) {
		for (i = 0; i < chip_info->hw_res->tx_num; i++) {
			k = ((mutual_num - j) - chip_info->hw_res->rx_num * i) - 1;
			seq_printf(s, "%6d", data_mutual_sram[k]);
		}
		seq_printf(s, " %6d\n", diag_self[j]);
	}

	seq_printf(s, "\n");
	for (i = 0; i < chip_info->hw_res->tx_num; i++) {
		seq_printf(s, "%6d", diag_self[i]);
	}
	/*Clear DSRAM flag*/
	himax_diag_register_set(0x00);
	dsram_flag = false;
	himax_return_event_stack();

	seq_printf(s, "\n");
	seq_printf(s, "ChannelEnd");
	seq_printf(s, "\n");

	TPD_INFO("%s, here:%d\n", __func__, __LINE__);

	if (debug_data_type == DEBUG_DATA_DOWN) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xF8;

		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;
		himax_register_write(tmp_addr, 4, tmp_data, 0);
	}

RET_OUT:
	if (data_mutual_sram) {
		kfree(data_mutual_sram);
	}

	return;
}

static void hx83102j_baseline_read(struct seq_file *s, void *chip_data)
{
	hx83102j_read_debug_data(s, chip_data, DEBUG_DATA_BASELINE);
	hx83102j_read_debug_data(s, chip_data, DEBUG_DATA_RAW);
	return;
}

static void hx83102j_delta_read(struct seq_file *s, void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	hx83102j_read_debug_data(s, chip_data, DEBUG_DATA_DELTA);
#ifdef CONFIG_OPPO_TP_APK
	if (chip_info->debug_mode_sta) {
		hx83102j_read_debug_data(s, chip_data, DEBUG_DATA_DOWN);
	}
#endif /*end of CONFIG_OPPO_TP_APK*/
	return;
}

static void hx83102j_main_register_read(struct seq_file *s, void *chip_data)
{
	return;
}

/*Reserved node*/
static void hx83102j_reserve_read(struct seq_file *s, void *chip_data)
{
	return;
}

static fw_update_state hx83102j_fw_update(void *chip_data, const struct firmware *fw, bool force)
{
	uint32_t CURRENT_FIRMWARE_ID = 0, FIRMWARE_ID = 0;
	uint8_t cmd[4];
	uint8_t data[64];
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	const uint8_t *p_fw_id = NULL;

	if (fw) {
		if (chip_info->g_fw_buf) {
			chip_info->g_fw_len = fw->size;
			memcpy(chip_info->g_fw_buf, fw->data, fw->size);
			chip_info->g_fw_sta = true;
		}
	}
	if (fw == NULL) {
		TPD_INFO("fw is NULL\n");
		return FW_NO_NEED_UPDATE;
	}

	p_fw_id = fw->data + 49172;

	if (!chip_info) {
		TPD_INFO("Chip info is NULL\n");
		return 0;
	}

	TPD_INFO("%s is called\n", __func__);



	CURRENT_FIRMWARE_ID = (*p_fw_id << 24) | (*(p_fw_id + 1) << 16) | (*(p_fw_id + 2) << 8) | *(p_fw_id + 3);


	cmd[3] = 0x10;  /*oppo fw id bin address : 0xc014   , 49172    Tp ic address : 0x 10007014*/
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x14;
	himax_register_read(cmd, 4, data, false);
	FIRMWARE_ID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	TPD_INFO("CURRENT TP FIRMWARE ID is 0x%x, FIRMWARE IMAGE ID is 0x%x\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID);


	/*step 3:Get into program mode*/
	/********************get into prog end************/
	/*step 4:flash firmware zone*/
	TPD_INFO("update-----------------firmware ------------------update!\n");

	g_core_fp.fp_firmware_update_0f(fw);
	g_core_fp.fp_reload_disable(0);
	msleep(10);

	himax_read_OPPO_FW_ver(chip_info);
	himax_sense_on(0x00);
	msleep(10);

	enable_irq(chip_info->hx_irq);


	chip_info->first_download_finished = true;
	return FW_UPDATE_SUCCESS;
}


/*
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM

static int hx83102j_get_usb_state(void)
{
	return 0;
}
#else
static int hx83102j_get_usb_state(void)
{
	return 0;
}
#endif
*/

static int hx83102j_reset_gpio_control(void *chip_data, bool enable)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
		TPD_INFO("%s: set reset state %d\n", __func__, enable);
	#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(g_chip_info->hw_res, enable);
	#endif
		TPD_DETAIL("%s: set reset state END\n", __func__);
	}
	return 0;
}

static void hx83102j_set_touch_direction(void *chip_data, uint8_t dir)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	chip_info->touch_direction = dir;
}

static uint8_t hx83102j_get_touch_direction(void *chip_data)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	return chip_info->touch_direction;
}
/*Himax_DB_Test Start*/
int hx83102j_freq_point = 0;
void hx83102j_freq_hop_trigger(void *chip_data)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int rtimes = 0;

	TPD_INFO("send cmd to tigger frequency hopping here!!!\n");
	hx83102j_freq_point = 1 - hx83102j_freq_point;
	if (hx83102j_freq_point) {/*hop to frequency 130K*/
		do {
			if (rtimes > 10) {
				TPD_INFO("%s:frequency hopping failed!\n", __func__);
				TPD_INFO("%s:correct tmp_data[0,1,2,3] = 0x5A,0xA5,0x5A,0xA5\n", __func__);
				break;
			}

			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0xC4;
			tmp_data[3] = 0xA5;
			tmp_data[2] = 0x5A;
			tmp_data[1] = 0xA5;
			tmp_data[0] = 0x5A;
			himax_flash_write_burst(tmp_addr, tmp_data);

			himax_register_read(tmp_addr, 4, tmp_data, false);
			TPD_DETAIL("%s:retry times %d, current tmp_data[0,1,2,3] = 0x%2.2X,0x%2.2X,0x%2.2X,0x%2.2X\n",
				__func__, rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
			rtimes++;
		} while (tmp_data[3] != 0xA5 || tmp_data[2] != 0x5A
				 || tmp_data[1] != 0xA5 || tmp_data[0] != 0x5A);

		if (rtimes <= 10) {
			TPD_INFO("%s:hopping frequency to 130K success!\n", __func__);
		}
	} else {/*hop to frequency 75K*/
		do {
			if (rtimes > 10) {
				TPD_INFO("%s:frequency hopping failed!\n", __func__);
				TPD_INFO("%s:correct tmp_data[0,1,2,3] = 0x3A,0xA3,0x3A,0xA3\n", __func__);
				break;
			}

			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0xC4;
			tmp_data[3] = 0xA3;
			tmp_data[2] = 0x3A;
			tmp_data[1] = 0xA3;
			tmp_data[0] = 0x3A;
			himax_flash_write_burst(tmp_addr, tmp_data);

			himax_register_read(tmp_addr, 4, tmp_data, false);
			TPD_DETAIL("%s:retry times %d, current tmp_data[0,1,2,3] = 0x%2.2X,0x%2.2X,0x%2.2X,0x%2.2X\n",
				__func__, rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
			rtimes++;
		} while (tmp_data[3] != 0xA3 || tmp_data[2] != 0x3A
				 || tmp_data[1] != 0xA3 || tmp_data[0] != 0x3A);

		if (rtimes <= 10) {
			TPD_INFO("%s:hopping frequency to 75K success!\n", __func__);
		}
	}
}
/*Himax_DB_Test End*/

static struct oplus_touchpanel_operations hx83102j_ops = {
	.ftm_process      = hx83102j_ftm_process,
	.get_vendor       = hx83102j_get_vendor,
	.get_chip_info    = hx83102j_get_chip_info,
	.reset            = hx83102j_reset,
	.power_control    = hx83102j_power_control,
	.fw_check         = hx83102j_fw_check,
	.fw_update        = hx83102j_fw_update,
	.trigger_reason   = hx83102j_trigger_reason,
	.get_touch_points = hx83102j_get_touch_points,
	.get_gesture_info = hx83102j_get_gesture_info,
	.mode_switch      = hx83102j_mode_switch,
	.exit_esd_mode    = hx83102j_exit_esd_mode,
	/*.resume_prepare = hx83102j_resume_prepare,*/
	/*.get_usb_state    = hx83102j_get_usb_state,*/
	/*.black_screen_test = hx83102j_black_screen_test,*/
	.reset_gpio_control = hx83102j_reset_gpio_control,
	.set_touch_direction    = hx83102j_set_touch_direction,
	.get_touch_direction    = hx83102j_get_touch_direction,
/*Himax_DB_Test Start*/
	.freq_hop_trigger = hx83102j_freq_hop_trigger,
/*Himax_DB_Test End*/
};

static struct himax_proc_operations hx83102j_proc_ops = {
	/*.auto_test     = hx83102j_auto_test,*/
	.himax_proc_register_write =  hx83102j_proc_register_write,
	.himax_proc_register_read =  hx83102j_proc_register_read,
	.himax_proc_diag_write =  hx83102j_proc_diag_write,
	/*.himax_proc_diag_read =  hx83102j_proc_diag_read,*/
	.himax_proc_DD_debug_read =  hx83102j_proc_DD_debug_read,
	.himax_proc_DD_debug_write =  hx83102j_proc_DD_debug_write,
	.himax_proc_FW_debug_read =  hx83102j_proc_FW_debug_read,
	.himax_proc_reset_write =  hx83102j_proc_reset_write,
	.himax_proc_sense_on_off_write =  hx83102j_proc_sense_on_off_write,
#ifdef HX_ENTER_ALGORITHM_NUMBER
	/*.himax_proc_enter_algorithm_switch_write = himax_enter_algorithm_number_write,*/
	/*.himax_proc_enter_algorithm_switch_read  = himax_enter_algorithm_number_read,*/
#endif
};

static struct engineer_test_operations hx_engineer_test_ops = {
	/*.auto_test = hx_auto_test,
	.black_screen_test = hx_black_screen_test,*/
};

static struct debug_info_proc_operations debug_info_proc_ops = {
	/*.limit_read    = himax_limit_read,*/
	.delta_read    = hx83102j_delta_read,
	.baseline_read = hx83102j_baseline_read,
	.main_register_read = hx83102j_main_register_read,
	.reserve_read = hx83102j_reserve_read,
};

#ifdef CONFIG_OPPO_TP_APK

static void himax_enter_hopping_write(bool on_off)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};


	if (on_off) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xF8;

		tmp_data[3] = 0xA5;
		tmp_data[2] = 0x5A;
		tmp_data[1] = 0xA5;
		tmp_data[0] = 0x5A;
		himax_register_write(tmp_addr, 4, tmp_data, 0);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xC4;

		tmp_data[3] = 0xA1;
		tmp_data[2] = 0x1A;
		tmp_data[1] = 0xA1;
		tmp_data[0] = 0x1A;
		himax_register_write(tmp_addr, 4, tmp_data, 0);
		TPD_INFO("%s: open himax enter hopping write.\n", __func__);
	} else {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xF8;

		tmp_data[3] = 0;
		tmp_data[2] = 0;
		tmp_data[1] = 0;
		tmp_data[0] = 0;
		himax_register_write(tmp_addr, 4, tmp_data, 0);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xC4;

		tmp_data[3] = 0;
		tmp_data[2] = 0;
		tmp_data[1] = 0;
		tmp_data[0] = 0;
		himax_register_write(tmp_addr, 4, tmp_data, 0);

		TPD_INFO("%s: close himax hopping write.\n", __func__);
	}
}


static void himax_apk_game_set(void *chip_data, bool on_off)
{
	hx83102j_mode_switch(chip_data, MODE_GAME, (int)on_off);
}

static bool himax_apk_game_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	return chip_info->lock_point_status;
}

static void himax_apk_debug_set(void *chip_data, bool on_off)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	himax_debug_mode_set(on_off);
	chip_info->debug_mode_sta = on_off;
}

static bool himax_apk_debug_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	return chip_info->debug_mode_sta;
}

static void himax_apk_gesture_debug(void *chip_data, bool on_off)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	chip_info->debug_gesture_sta = on_off;
}

static bool  himax_apk_gesture_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	return chip_info->debug_gesture_sta;
}

static int  himax_apk_gesture_info(void *chip_data, char *buf, int len)
{
	int ret = 0;
	int i;
	int num;
	u8 temp;
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	if(len < 2) {
		return 0;
	}
	buf[0] = 255;

	temp = private_ts->gesture_buf[0];
	if (temp == 0x00) {
		temp = private_ts->gesture_buf[1] | 0x80;
	}
	buf[0] = temp;


	num = private_ts->gesture_buf[2];

	if(num > 40) {
		num = 40;
	}
	ret = 2;

	buf[1] = num;

	for (i = 0; i < num; i++) {
		int x;
		int y;
		x = private_ts->gesture_buf[i * 2 + 3];
		x = x * private_ts->resolution_info.max_x / 255;

		y = private_ts->gesture_buf[i * 2 + 4];
		y = y * private_ts->resolution_info.max_y / 255;




		if (len < i * 4 + 2) {
			break;
		}
		buf[i * 4 + 2] = x & 0xFF;
		buf[i * 4 + 3] = (x >> 8) & 0xFF;
		buf[i * 4 + 4] = y & 0xFF;
		buf[i * 4 + 5] = (y >> 8) & 0xFF;
		ret += 4;
	}

	return ret;
}


static void himax_apk_earphone_set(void *chip_data, bool on_off)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	hx83102j_mode_switch(chip_data, MODE_HEADSET, (int)on_off);
	chip_info->earphone_sta = on_off;
}

static bool himax_apk_earphone_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	return chip_info->earphone_sta;
}

static void himax_apk_charger_set(void *chip_data, bool on_off)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	hx83102j_mode_switch(chip_data, MODE_CHARGE, (int)on_off);
	chip_info->plug_status = on_off;
}

static bool himax_apk_charger_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	return chip_info->plug_status;
}

static void himax_apk_noise_set(void *chip_data, bool on_off)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	himax_enter_hopping_write(on_off);
	chip_info->noise_sta = on_off;
}

static bool himax_apk_noise_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	return chip_info->noise_sta;
}


static int  himax_apk_tp_info_get(void *chip_data, char *buf, int len)
{
	int ret;
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	ret = snprintf(buf, len, "IC:HIMAX%06X\nFW_VER:0x%04X\nCH:%dX%d\n",
				   0x83102J,
				   chip_info->fw_ver,
				   chip_info->hw_res->tx_num,
				   chip_info->hw_res->rx_num);
	if (ret > len) {
		ret = len;
	}

	return ret;
}

static void himax_init_oppo_apk_op(struct touchpanel_data *ts)
{
	ts->apk_op = kzalloc(sizeof(APK_OPERATION), GFP_KERNEL);
	if(ts->apk_op) {
		ts->apk_op->apk_game_set = himax_apk_game_set;
		ts->apk_op->apk_game_get = himax_apk_game_get;
		ts->apk_op->apk_debug_set = himax_apk_debug_set;
		ts->apk_op->apk_debug_get = himax_apk_debug_get;
		/*apk_op->apk_proximity_set = ili_apk_proximity_set;*/
		/*apk_op->apk_proximity_dis = ili_apk_proximity_dis;*/
		ts->apk_op->apk_noise_set = himax_apk_noise_set;
		ts->apk_op->apk_noise_get = himax_apk_noise_get;
		ts->apk_op->apk_gesture_debug = himax_apk_gesture_debug;
		ts->apk_op->apk_gesture_get = himax_apk_gesture_get;
		ts->apk_op->apk_gesture_info = himax_apk_gesture_info;
		ts->apk_op->apk_earphone_set = himax_apk_earphone_set;
		ts->apk_op->apk_earphone_get = himax_apk_earphone_get;
		ts->apk_op->apk_charger_set = himax_apk_charger_set;
		ts->apk_op->apk_charger_get = himax_apk_charger_get;
		ts->apk_op->apk_tp_info_get = himax_apk_tp_info_get;
		/*apk_op->apk_data_type_set = ili_apk_data_type_set;*/
		/*apk_op->apk_rawdata_get = ili_apk_rawdata_get;*/
		/*apk_op->apk_diffdata_get = ili_apk_diffdata_get;*/
		/*apk_op->apk_basedata_get = ili_apk_basedata_get;*/
		/*ts->apk_op->apk_backdata_get = nova_apk_backdata_get;*/
		/*apk_op->apk_debug_info = ili_apk_debug_info;*/

	} else {
		TPD_INFO("Can not kzalloc apk op.\n");
	}
}
#endif /*end of CONFIG_OPPO_TP_APK*/

static ssize_t tp_info_read(struct file *file, char *buf,
							size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	char buf_tmp[1024] = {0};

	if (*pos)
		return 0;

	himax_read_OPPO_FW_ver(g_chip_info);

	ret = snprintf(buf_tmp, sizeof(buf_tmp),
				"[VENDOR] = TXD, [IC] = HX83102J, [FW_VER] = 0x%02X\n", g_touch_ver);
	if (clear_user(buf, len)) {
		TPD_INFO("clear error");
		return -EIO;
	}
	if (copy_to_user(buf, buf_tmp, (len > 1024)?1024:len))
		TPD_INFO("%s,here:%d\n", __func__, __LINE__);

	*pos += ret;
	return ret;
}

#if LINUX_VERSION_CODE>= KERNEL_VERSION(5, 10, 0)

static const struct proc_ops tp_info_ops = {
	.proc_open = simple_open,
	.proc_read = tp_info_read,
};
#else
static const struct file_operations tp_info_ops = {
	.owner = THIS_MODULE,
	.read = tp_info_read,
};
#endif
/*
* Modify and fix by Himax(HX)
* Version: 2020020701
*/
int __maybe_unused hx83102j_tp_probe(struct spi_device *spi)
{
	struct chip_data_hx83102j *chip_info = NULL;
	struct touchpanel_data *ts = NULL;
	struct proc_dir_entry *prEntry_tmp = NULL;
	int ret = -1;

	TPD_INFO("%s  is called\n", __func__);

	/*step1:Alloc chip_info*/
	chip_info = kzalloc(sizeof(struct chip_data_hx83102j), GFP_KERNEL);
	if (chip_info == NULL) {
		TPD_INFO("chip info kzalloc error\n");
		ret = -ENOMEM;
		return ret;
	}
	memset(chip_info, 0, sizeof(*chip_info));
	g_chip_info = chip_info;

	/* allocate himax report data */
	hx_touch_data = kzalloc(sizeof(struct himax_report_data), GFP_KERNEL);
	if (hx_touch_data == NULL) {
		goto err_register_driver;
	}

	/*step2:Alloc common ts*/
	ts = common_touch_data_alloc();
	if (ts == NULL) {
		TPD_INFO("ts kzalloc error\n");
		goto err_register_driver;
	}
	memset(ts, 0, sizeof(*ts));

	chip_info->g_fw_buf = vmalloc(255 * 1024);
	if (chip_info->g_fw_buf == NULL) {
		TPD_INFO("fw buf vmalloc error\n");

		goto err_g_fw_buf;
	}

	/*step3:binding dev for easy operate*/
	chip_info->hx_spi = spi;
	chip_info->syna_ops = &hx83102j_proc_ops;
	ts->debug_info_ops = &debug_info_proc_ops;
	ts->s_client = spi;
	chip_info->hx_irq = spi->irq;
	ts->irq = spi->irq;
	spi_set_drvdata(spi, ts);
	ts->dev = &spi->dev;
	ts->chip_data = chip_info;
	chip_info->hw_res = &ts->hw_res;
#ifdef HX_ZERO_FLASH
	mutex_init(&(chip_info->spi_lock));
	mutex_init(&(chip_info->fw_update_lock));
#endif
	chip_info->touch_direction = VERTICAL_SCREEN;
	chip_info->using_headfile = false;
	chip_info->first_download_finished = false;

	if (ts->s_client->master->flags & SPI_MASTER_HALF_DUPLEX) {
		TPD_INFO("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_spi_setup;
	}
	ts->s_client->bits_per_word = 8;
	ts->s_client->mode = SPI_MODE_3;
	ts->s_client->chip_select = 0;

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	/* new usage of MTK spi API */
	memcpy(&chip_info->hx_spi_mcc, &hx_spi_ctrdata, sizeof(struct mtk_chip_config));
	ts->s_client->controller_data = (void *)&chip_info->hx_spi_mcc;
#else
	/* old usage of MTK spi API */
	/*memcpy(&chip_info->hx_spi_mcc, &hx_spi_ctrdata, sizeof(struct mt_chip_conf));*/
	/*ts->s_client->controller_data = (void *)&chip_info->hx_spi_mcc;*/

	ret = spi_setup(ts->s_client);
	if (ret < 0) {
		TPD_INFO("Failed to perform SPI setup\n");
		goto err_spi_setup;
	}
#endif
	/*chip_info->p_spuri_fp_touch = &(ts->spuri_fp_touch);*/

	/*disable_irq_nosync(chip_info->hx_irq);*/

	/*step4:file_operations callback binding*/
	ts->ts_ops = &hx83102j_ops;
	ts->engineer_ops = &hx_engineer_test_ops;
	private_ts = ts;

#ifdef CONFIG_OPPO_TP_APK
	himax_init_oppo_apk_op(ts);
#endif /* end of CONFIG_OPPO_TP_APK*/

	/*step5:register common touch*/
	ret = register_common_touch_device(ts);
	if (ret < 0) {
		goto err_register_driver;
	}
	disable_irq_nosync(chip_info->hx_irq);
	if (himax_ic_package_check() == false) {
		TPD_INFO("Himax chip doesn NOT EXIST");
		goto err_register_driver;
	}
	chip_info->test_limit_name = ts->panel_data.test_limit_name;
#ifdef HX_ZERO_FLASH
	chip_info->p_firmware_headfile = &ts->panel_data.firmware_headfile;
	g_auto_update_flag = true;
	chip_info->himax_0f_update_wq = create_singlethread_workqueue("HMX_0f_update_reuqest");
	INIT_DELAYED_WORK(&chip_info->work_0f_update, himax_mcu_0f_operation);
	/*queue_delayed_work(chip_info->himax_0f_update_wq, &chip_info->work_0f_update, msecs_to_jiffies(2000));*/
#else
	himax_read_FW_ver();
	himax_calculateChecksum(false);
#endif

	himax_power_on_init();

	/*touch data init*/
	ret = himax_report_data_init(ts->max_num, ts->hw_res.tx_num, ts->hw_res.rx_num);
	if (ret) {
		goto err_register_driver;
	}

	ts->tp_suspend_order = TP_LCD_SUSPEND;
	ts->tp_resume_order = LCD_TP_RESUME;
	ts->skip_suspend_operate = true;
	ts->skip_reset_in_resume = false;

	/*step7:create hx83102j related proc files*/
	himax_create_proc(ts, chip_info->syna_ops);
	irq_en_cnt = 1;
	TPD_INFO("%s, probe normal end\n", __func__);

	prEntry_tmp = proc_create_data("tp_info", 0666, NULL, &tp_info_ops, ts);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc/tp_info proc entry, %d\n", __func__, __LINE__);
	}

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	if (ts->boot_mode == RECOVERY_BOOT) {
#else
	if (ts->boot_mode == MSM_BOOT_MODE__RECOVERY) {
#endif
#ifndef HX_ZERO_FLASH
		enable_irq(chip_info->hx_irq);
#endif
		TPD_INFO("In Recovery mode, no-flash download fw by headfile\n");
		queue_delayed_work(chip_info->himax_0f_update_wq, &chip_info->work_0f_update, msecs_to_jiffies(500));
	}
	if (is_oem_unlocked()) {
		TPD_INFO("Replace system image for cts, download fw by headfile\n");
		queue_delayed_work(chip_info->himax_0f_update_wq, &chip_info->work_0f_update, msecs_to_jiffies(5000));
		}

	if (ts->fw_update_in_probe_with_headfile) {
#ifndef HX_ZERO_FLASH
		enable_irq(chip_info->hx_irq);
#endif
		TPD_INFO("It's fw_update_in_probe_with_headfile\n");
		queue_delayed_work(chip_info->himax_0f_update_wq, &chip_info->work_0f_update, msecs_to_jiffies(5000));
		}

	return 0;
err_spi_setup:
	if (chip_info->g_fw_buf) {
		vfree(chip_info->g_fw_buf);
	}
err_g_fw_buf:
err_register_driver:


#if defined(CONFIG_DRM_MSM)
	if (&ts->fb_notif) {
	}
#elif defined(CONFIG_FB)
	if (&ts->fb_notif) {
		fb_unregister_client(&ts->fb_notif);
	}
#endif/*CONFIG_FB*/

	disable_irq_nosync(chip_info->hx_irq);

	common_touch_data_free(ts);
	ts = NULL;

	if (hx_touch_data) {
		kfree(hx_touch_data);
	}

	if (chip_info) {
		kfree(chip_info);
	}

	ret = -1;

	TPD_INFO("%s, probe error\n", __func__);

	return ret;
}

int __maybe_unused hx83102j_tp_remove(struct spi_device *spi)
{
	struct touchpanel_data *ts = spi_get_drvdata(spi);

	ts->s_client = NULL;
	/* spin_unlock_irq(&ts->spi_lock); */
	spi_set_drvdata(spi, NULL);

	TPD_INFO("%s is called\n", __func__);
	kfree(ts);

	return 0;
}

/*
static int hx83102j_i2c_suspend(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("%s: is called gesture_enable =%d\n", __func__, ts->gesture_enable);
	tp_pm_suspend(ts);

	return 0;
}

static int hx83102j_i2c_resume(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("%s is called\n", __func__);
	tp_pm_resume(ts);

	if (ts->black_gesture_support) {
		 if (ts->gesture_enable == 1) {
			 TPD_INFO("himax_set_SMWP_enable 1\n");
			 himax_set_SMWP_enable(1, ts->is_suspended);
		 }
	 }
	return 0;
}
*/

static const struct spi_device_id tp_id[] = {
#ifdef CONFIG_TOUCHPANEL_MULTI_NOFLASH
	{ "oplus,tp_noflash", 0 },
#else
	{ TPD_DEVICE, 0 },
#endif
	{ }
};

static struct of_device_id tp_match_table[] = {
#ifdef CONFIG_TOUCHPANEL_MULTI_NOFLASH
	{ .compatible = "oplus,tp_noflash", },
#else
	{ .compatible = TPD_DEVICE, },
#endif
	{ },
};

static const struct dev_pm_ops tp_pm_ops = {
#ifdef CONFIG_FB
	/*.suspend = hx83102j_i2c_suspend,*/
	/*.resume = hx83102j_i2c_resume,*/
#endif
};


static struct spi_driver himax_common_driver = {
	.probe      = hx83102j_tp_probe,
	.remove     = hx83102j_tp_remove,
	.id_table   = tp_id,
	.driver = {
		.name = TPD_DEVICE,
		.owner = THIS_MODULE,
		.of_match_table = tp_match_table,
		.pm = &tp_pm_ops,
	},
};

static int __init tp_driver_init(void)
{
	int status = 0;

	TPD_INFO("%s is called\n", __func__);
	if (!tp_judge_ic_match(TPD_DEVICE)) {
		return 0;
	}

	get_oem_verified_boot_state();
	status = spi_register_driver(&himax_common_driver);
	if (status < 0) {
		TPD_INFO("%s, Failed to register SPI driver.\n", __func__);
		return 0;
	}

	return status;
}

/* should never be called */
static void __exit tp_driver_exit(void)
{
	spi_unregister_driver(&himax_common_driver);
	return;
}

module_init(tp_driver_init);
module_exit(tp_driver_exit);

MODULE_DESCRIPTION("Touchscreen Driver");
MODULE_LICENSE("GPL");
