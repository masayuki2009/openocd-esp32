/***************************************************************************
 *   Generic flash driver for Espressif Xtensa chips                       *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

/*
 * Overview
 * --------
 * Like many other flash drivers this one uses special binary program (stub) running on target
 * to perform all operations and communicate to the host. Stub has entry function which accepts
 * variable number of arguments and therefore can handle different flash operation requests.
 * Only the first argument of the stub entry function is mandatory for all operations it must
 * specify the type of flash function to perform (read, write etc.). Actually stub main function
 * is a dispatcher which determines the type of flash operation to perform, retrieves other
 * arguments and calls corresponding handler. In C notation entry function looks like the following:

 * int stub_main(int cmd, ...);

 * In general every flash operation consists of the following steps:
 * 1) Stub is loaded to target.
 * 2) Necessary arguments are prepared and stub's main function is called.
 * 3) Stub does the work and returns the result.

 * Stub Loading
 * ------------
 * To run stub its code and data sections must be loaded to the target. It is done using working area API.
 * But since code and data address spaces are separated in ESP32 it is necessary to have two configured
 * working areas: one in code address space and another one in data space. So driver allocates chunks
 * in respective pools and writes stub sections to them. It is important that the both stub sections reside
 * at the beginning of respective working areas because stub code is linked as ELF and therefore it is
 * position dependent. So target memory for stub code and data must be allocated first.

 * Stub Execution
 * --------------
 * Special wrapping code is used to enter and exit the stub's main function. It prepares register arguments
 * before Windowed ABI call to stub entry and upon return from it executes break command to indicate to OpenOCD
 * that operation is finished.

 * Flash Data Transfers
 * --------------------
 * To transfer data from/to target a buffer should be allocated at ESP32 side. Also during the data transfer
 * target and host must maintain the state of that buffer (read/write pointers etc.). So host needs to check
 * the state of that buffer periodically and write to or read from it (depending on flash operation type).
 * ESP32 does not support access to its memory via JTAG when it is not halted, so accessing target memory would
 * requires halting the CPUs every time the host needs to check if there are incoming data or free space available
 * in the buffer. This fact can slow down flash write/read operations dramatically. To avoid this flash driver and
 * stub use application level tracing module API to transfer the data in 'non-stop' mode.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/sha256.h>
#include <helper/binarybuffer.h>
#include <target/register.h>
#include <target/esp_xtensa_apptrace.h>
#include "esp_xtensa.h"
#include "time_support.h"
#include "contrib/loaders/flash/esp/stub_flasher.h"

#include <zlib.h>

#define ESP_XTENSA_FLASH_MIN_OFFSET      0x1000	/* protect secure boot digest data */
#define ESP_XTENSA_RW_TMO                20000	/* ms */
#define ESP_XTENSA_ERASE_TMO             60000	/* ms */

struct esp_xtensa_rw_args {
	int (*xfer)(struct target *target, uint32_t block_id, uint32_t len, void *priv);
	uint8_t *buffer;
	uint32_t count;
	uint32_t total_count;
	bool connected;
};

struct esp_xtensa_write_state {
	struct esp_xtensa_rw_args rw;
	uint32_t prev_block_id;
	struct working_area *target_buf;
	struct esp_xtensa_flash_bank *esp_xtensa_info;
	struct working_area *stub_wargs_area;
	struct esp_xtensa_stub_flash_write_args stub_wargs;
};

struct esp_xtensa_read_state {
	struct esp_xtensa_rw_args rw;
	uint8_t *rd_buf;
};

struct esp_xtensa_erase_check_args {
	struct working_area *erased_state_buf;
	uint32_t num_sectors;
};

struct esp_xtensa_flash_bp_op_state {
	struct working_area *target_buf;
	struct esp_xtensa_flash_bank *esp_xtensa_info;
};

static int esp_xtensa_compress(const uint8_t *in, uint32_t in_len, uint8_t **out, uint32_t *out_len)
{
	z_stream strm;
	int wbits = -MAX_WBITS;		/*deflate */
	int level = Z_DEFAULT_COMPRESSION;	/*Z_BEST_SPEED; */

	strm.zalloc = Z_NULL;
	strm.zfree = Z_NULL;
	strm.opaque = Z_NULL;

	if (deflateInit2(&strm, level, Z_DEFLATED, wbits, MAX_MEM_LEVEL,
			Z_DEFAULT_STRATEGY) != Z_OK) {
		LOG_ERROR("deflateInit2 error!");
		return ERROR_FAIL;
	}

	strm.avail_out = deflateBound(&strm, (uLong)in_len);

	/* Some compression methods may need a little more space */
	strm.avail_out += 100;

	if (strm.avail_out > INT_MAX) {
		deflateEnd(&strm);
		return ERROR_FAIL;
	}

	*out = (uint8_t *)malloc((int)strm.avail_out);
	if (*out == NULL) {
		LOG_ERROR("out buffer allocation failed!");
		return ERROR_FAIL;
	}
	strm.next_out = *out;
	strm.next_in = (uint8_t *)in;
	strm.avail_in = (uInt)in_len;

	/* always compress in one pass - the return value holds the entire
	 * decompressed data anyway, so there's no reason to do chunked
	 * decompression */
	if (deflate(&strm, Z_FINISH) != Z_STREAM_END) {
		free(*out);
		deflateEnd(&strm);
		LOG_ERROR("not enough output space");
		return ERROR_FAIL;
	}

	deflateEnd(&strm);

	if (strm.total_out > INT_MAX) {
		free(*out);
		LOG_ERROR("too much output");
		return ERROR_FAIL;
	}

	*out_len = strm.total_out;

	LOG_DEBUG("inlen:(%u) outlen:(%u)!", in_len, *out_len);

	return ERROR_OK;
}

static int esp_xtensa_calc_hash(const uint8_t *data, size_t datalen, uint8_t *hash)
{
	if (data == NULL || hash == NULL || datalen == 0)
		return ERROR_FAIL;

	struct tc_sha256_state_struct sha256_state;

	if (tc_sha256_init(&sha256_state) != TC_CRYPTO_SUCCESS) {
		LOG_ERROR("tc_sha256_init failed!");
		return ERROR_FAIL;
	}

	if (tc_sha256_update(&sha256_state, data, datalen) != TC_CRYPTO_SUCCESS) {
		LOG_ERROR("tc_sha256_update failed!");
		return ERROR_FAIL;
	}

	if (tc_sha256_final(hash, &sha256_state) != TC_CRYPTO_SUCCESS) {
		LOG_ERROR("tc_sha256_final failed!");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int esp_xtensa_flasher_image_init(struct xtensa_algo_image *flasher_image,
	const struct esp_xtensa_flasher_stub_config *stub_cfg)
{
	if (!stub_cfg) {
		LOG_ERROR("Invalid stub!");
		return ERROR_FAIL;
	}

	flasher_image->bss_size = stub_cfg->bss_sz;
	memset(&flasher_image->image, 0, sizeof(flasher_image->image));
	int ret = image_open(&flasher_image->image, NULL, "build");
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to create image (%d)!", ret);
		return ret;
	}
	flasher_image->image.start_address_set = 1;
	flasher_image->image.start_address = stub_cfg->entry_addr;
	ret = image_add_section(&flasher_image->image,
		0,
		stub_cfg->code_sz,
		IMAGE_ELF_PHF_EXEC,
		stub_cfg->code);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to create image (%d)!", ret);
		image_close(&flasher_image->image);
		return ret;
	}
	ret = image_add_section(&flasher_image->image, 0, stub_cfg->data_sz, 0, stub_cfg->data);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to create image (%d)!", ret);
		image_close(&flasher_image->image);
		return ret;
	}
	LOG_DEBUG("base=%08x set=%d",
		(unsigned) flasher_image->image.base_address,
		flasher_image->image.base_address_set);
	return ret;
}

int esp_xtensa_flash_init(struct esp_xtensa_flash_bank *esp_xtensa_info, uint32_t sec_sz,
	int (*run_func_image)(struct target *target, struct xtensa_algo_run_data *run,
		struct xtensa_algo_image *image, uint32_t num_args, ...),
	bool (*is_irom_address)(target_addr_t addr),
	bool (*is_drom_address)(target_addr_t addr),
	const struct esp_xtensa_flasher_stub_config *(*get_stub)(struct flash_bank *bank))
{

	esp_xtensa_info->probed = 0;
	esp_xtensa_info->sec_sz = sec_sz;
	esp_xtensa_info->get_stub = get_stub;
	esp_xtensa_info->run_func_image = run_func_image;
	esp_xtensa_info->is_irom_address = is_irom_address;
	esp_xtensa_info->is_drom_address = is_drom_address;
	esp_xtensa_info->hw_flash_base = 0;
	esp_xtensa_info->appimage_flash_base = (uint32_t)-1;
	esp_xtensa_info->compression = 1;	/* enables compression by default */
	esp_xtensa_info->old_cpu_freq = 0;
	return ERROR_OK;
}

int esp_xtensa_protect(struct flash_bank *bank, int set, int first, int last)
{
	return ERROR_FAIL;
}

int esp_xtensa_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

int esp_xtensa_blank_check(struct flash_bank *bank)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct xtensa_algo_image flasher_image;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1300;
	struct mem_param mp;
	init_mem_param(&mp, 3 /*3rd usr arg*/, bank->num_sectors /*size in bytes*/, PARAM_IN);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;

	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		4,
		ESP_XTENSA_STUB_CMD_FLASH_ERASE_CHECK /*cmd*/,
		esp_xtensa_info->hw_flash_base/esp_xtensa_info->sec_sz /*start*/,
		bank->num_sectors /*sectors num*/,
		0 /*address to store sectors' state*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		destroy_mem_param(&mp);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to check erase flash (%d)!", run.ret_code);
		ret = ERROR_FAIL;
	} else {
		for (int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = mp.value[i];
	}
	destroy_mem_param(&mp);
	return ret;
}

static uint32_t esp_xtensa_get_size(struct flash_bank *bank)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	uint32_t size = 0;
	struct xtensa_algo_run_data run;
	struct xtensa_algo_image flasher_image;

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1024;
	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		1,
		ESP_XTENSA_STUB_CMD_FLASH_SIZE);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return 0;
	}
	size = run.ret_code;
	if (size == 0)
		LOG_ERROR("Failed to get flash size!");
	LOG_DEBUG("%s size 0x%x", __func__, size);
	return size;
}

static int esp_xtensa_get_mappings(struct flash_bank *bank,
	struct esp_xtensa_flash_bank *esp_xtensa_info,
	struct esp_xtensa_flash_mapping *flash_map,
	uint32_t appimage_flash_base)
{
	struct xtensa_algo_run_data run;
	struct xtensa_algo_image flasher_image;

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1300;

	struct mem_param mp;
	init_mem_param(&mp,
		2 /*2nd usr arg*/,
		sizeof(struct esp_xtensa_flash_mapping)	/*size in bytes*/,
		PARAM_IN);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;

	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		3 /*args num*/,
		ESP_XTENSA_STUB_CMD_FLASH_MAP_GET /*cmd*/,
		appimage_flash_base,
		0 /*address to store mappings*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		destroy_mem_param(&mp);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to get flash maps (%d)!", run.ret_code);
		if (run.ret_code == ESP_XTENSA_STUB_ERR_INVALID_IMAGE)
			LOG_WARNING(
				"Application image is invalid! Check configured binary flash offset 'appimage_offset'.");
		ret = ERROR_FAIL;
	} else {
		memcpy(flash_map, mp.value, sizeof(struct esp_xtensa_flash_mapping));
		if (flash_map->maps_num == 0)
			LOG_WARNING("Empty flash mapping!");
		for (uint32_t i = 0; i < flash_map->maps_num; i++)
			LOG_INFO("Flash mapping %d: 0x%x -> 0x%x, %d KB",
				i,
				flash_map->maps[i].phy_addr,
				flash_map->maps[i].load_addr,
				flash_map->maps[i].size/1024);
	}
	destroy_mem_param(&mp);
	return ret;
}

int esp_xtensa_erase(struct flash_bank *bank, int first, int last)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct xtensa_algo_image flasher_image;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	assert((0 <= first) && (first <= last) && (last < bank->num_sectors));
	if (esp_xtensa_info->hw_flash_base + first*esp_xtensa_info->sec_sz <
		ESP_XTENSA_FLASH_MIN_OFFSET) {
		LOG_ERROR("Invalid offset!");
		return ERROR_FAIL;
	}

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1024;
	run.tmo = ESP_XTENSA_ERASE_TMO;
	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		3,
		ESP_XTENSA_STUB_CMD_FLASH_ERASE,
		/* cmd */
		esp_xtensa_info->hw_flash_base + first*esp_xtensa_info->sec_sz,
		/* start addr */
		(last-first+1)*esp_xtensa_info->sec_sz);		/* size */
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to erase flash (%d)!", run.ret_code);
		ret = ERROR_FAIL;
	}
	return ret;
}

static int esp_xtensa_rw_do(struct target *target, void *priv)
{
	struct duration algo_time, tmo_time;
	struct esp_xtensa_rw_args *rw = (struct esp_xtensa_rw_args *)priv;
	int retval = ERROR_OK, busy_num = 0;

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start data write time measurement!");
		return ERROR_FAIL;
	}
	while (rw->total_count < rw->count) {
		uint32_t block_id = 0, len = 0;
		LOG_DEBUG("Transfer block on %s", target_name(target));
		retval = esp_xtensa_apptrace_data_len_read(target, &block_id, &len);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace status (%d)!", retval);
			return retval;
		}
		/* transfer block */
		LOG_DEBUG("Transfer block %d, %d bytes", block_id, len);
		retval = rw->xfer(target, block_id, len, rw);
		if (retval == ERROR_WAIT) {
			LOG_DEBUG("Block not ready");
			if (busy_num++ == 0) {
				if (duration_start(&tmo_time) != 0) {
					LOG_ERROR("Failed to start data write time measurement!");
					return ERROR_FAIL;
				}
			} else {
				/* if no transfer check tmo */
				if (duration_measure(&tmo_time) != 0) {
					LOG_ERROR("Failed to stop algo run measurement!");
					return ERROR_FAIL;
				}
				if (1000*duration_elapsed(&tmo_time) > ESP_XTENSA_RW_TMO) {
					LOG_ERROR("Transfer data tmo!");
					return ERROR_WAIT;
				}
			}
		} else if (retval != ERROR_OK) {
			LOG_ERROR("Failed to transfer flash data block (%d)!", retval);
			return retval;
		} else
			busy_num = 0;
		alive_sleep(10);
		if (target->state != TARGET_DEBUG_RUNNING) {
			LOG_ERROR("Algorithm accidentally stopped (%d)!", target->state);
			return ERROR_FAIL;
		}
	}
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop data write measurement!");
		return ERROR_FAIL;
	}
	LOG_INFO("PROF: Data transferred in %g ms @ %g KB/s",
		duration_elapsed(&algo_time)*1000,
		duration_kbps(&algo_time, rw->total_count));

	return ERROR_OK;
}

static int esp_xtensa_write_xfer(struct target *target, uint32_t block_id, uint32_t len, void *priv)
{
	struct esp_xtensa_write_state *state = (struct esp_xtensa_write_state *)priv;
	int retval;

	/* check for target to get connected */
	if (!state->rw.connected) {
		retval =
			esp_xtensa_apptrace_ctrl_reg_read(target, NULL, NULL, &state->rw.connected);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace control reg (%d)!", retval);
			return retval;
		}
		if (!state->rw.connected)
			return ERROR_WAIT;
	}

	if (state->prev_block_id == block_id)
		return ERROR_WAIT;

	uint32_t wr_sz = state->rw.count - state->rw.total_count <
		esp_xtensa_apptrace_usr_block_max_size_get(target) ?
		state->rw.count -
		state->rw.total_count : esp_xtensa_apptrace_usr_block_max_size_get(target);
	retval = esp_xtensa_apptrace_usr_block_write(target,
		block_id,
		state->rw.buffer + state->rw.total_count,
		wr_sz);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write apptrace data (%d)!", retval);
		return retval;
	}
	state->rw.total_count += wr_sz;
	state->prev_block_id = block_id;

	return ERROR_OK;
}

static int esp_xtensa_write_state_init(struct target *target,
	struct xtensa_algo_run_data *run,
	struct esp_xtensa_write_state *state)
{
	struct duration algo_time;

	/* clear control register, stub will set XTENSA_APPTRACE_HOST_CONNECT bit when it will be
	 * ready */
	int ret = esp_xtensa_apptrace_ctrl_reg_write(target,
		0 /*block_id*/,
		0 /*len*/,
		false /*conn*/,
		false /*data*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to clear apptrace ctrl reg (%d)!", ret);
		return ret;
	}

	/* memory buffer */
	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start workarea alloc time measurement!");
		return ERROR_FAIL;
	}
	uint32_t buffer_size = 64*1024;
	while (target_alloc_alt_working_area_try(target, buffer_size,
			&state->target_buf) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size == 0) {
			LOG_ERROR("Failed to alloc target buffer for flash data!");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop workarea alloc measurement!");
		return ERROR_FAIL;
	}
	LOG_DEBUG("PROF: Allocated target buffer %d bytes in %g ms",
		buffer_size,
		duration_elapsed(&algo_time)*1000);

	/* alloc memory for stub flash write arguments in data working area */
	if (target_alloc_alt_working_area(target, sizeof(state->stub_wargs),
			&state->stub_wargs_area) != ERROR_OK) {
		LOG_ERROR("no working area available, can't alloc space for stub flash arguments");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	state->stub_wargs.down_buf_addr = state->target_buf->address;
	state->stub_wargs.down_buf_size = state->target_buf->size;

	ret = target_write_buffer(target, state->stub_wargs_area->address,
		sizeof(state->stub_wargs), (uint8_t *)&state->stub_wargs);
	if (ret != ERROR_OK) {
		LOG_ERROR("Write memory at address " TARGET_ADDR_FMT " failed",
			state->stub_wargs_area->address);
		return ERROR_TARGET_FAILURE;
	}

	buf_set_u32(run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START + 1].value,
		0,
		32,
		state->stub_wargs_area->address);

	return ERROR_OK;
}

static void esp_xtensa_write_state_cleanup(struct target *target,
	struct xtensa_algo_run_data *run,
	struct esp_xtensa_write_state *state)
{
	struct duration algo_time;

	if (!state->target_buf)
		return;
	if (duration_start(&algo_time) != 0)
		LOG_ERROR("Failed to start workarea alloc time measurement!");
	target_free_alt_working_area(target, state->target_buf);
	target_free_alt_working_area(target, state->stub_wargs_area);
	if (duration_measure(&algo_time) != 0)
		LOG_ERROR("Failed to stop data write measurement!");
	LOG_DEBUG("PROF: Workarea freed in %g ms", duration_elapsed(&algo_time)*1000);
}

int esp_xtensa_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct esp_xtensa_write_state wr_state;
	struct xtensa_algo_image flasher_image;
	uint8_t *compressed_buff = NULL;
	uint32_t compressed_len = 0;
	uint32_t stack_size = 1024 + ESP_XTENSA_STUB_UNZIP_BUFF_SIZE;

	if (esp_xtensa_info->hw_flash_base + offset < ESP_XTENSA_FLASH_MIN_OFFSET) {
		LOG_ERROR("Invalid offset!");
		return ERROR_FAIL;
	}
	if (offset & 0x3UL) {
		LOG_ERROR("Unaligned offset!");
		return ERROR_FAIL;
	}
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	if (esp_xtensa_info->compression) {
		struct duration bench;
		duration_start(&bench);
		if (esp_xtensa_compress(buffer, count, &compressed_buff,
				&compressed_len) != ERROR_OK) {
			LOG_ERROR("Compression failed!");
			return ERROR_FAIL;
		}
		duration_measure(&bench);
		LOG_INFO("Compressed %" PRIu32 " bytes to %" PRIu32 " bytes "
			"in %fs",
			count,
			compressed_len,
			duration_elapsed(&bench));

		stack_size += ESP_XTENSA_STUB_IFLATOR_SIZE;
	}

	memset(&run, 0, sizeof(run));
	run.stack_size = stack_size;
	run.usr_func = esp_xtensa_rw_do;
	run.usr_func_arg = &wr_state;
	run.usr_func_init = (xtensa_algo_usr_func_init_t)esp_xtensa_write_state_init;
	run.usr_func_done = (xtensa_algo_usr_func_done_t)esp_xtensa_write_state_cleanup;
	memset(&wr_state, 0, sizeof(struct esp_xtensa_write_state));
	wr_state.rw.buffer = esp_xtensa_info->compression ? compressed_buff : (uint8_t *)buffer;
	wr_state.rw.count = esp_xtensa_info->compression ? compressed_len : count;
	wr_state.rw.xfer = esp_xtensa_write_xfer;
	wr_state.prev_block_id = (uint32_t)-1;
	wr_state.esp_xtensa_info = esp_xtensa_info;
	/* stub flasher arguments */
	wr_state.stub_wargs.size = wr_state.rw.count;
	wr_state.stub_wargs.total_size = count;
	wr_state.stub_wargs.start_addr = esp_xtensa_info->hw_flash_base + offset;
	wr_state.stub_wargs.down_buf_addr = 0;
	wr_state.stub_wargs.down_buf_size = 0;

	ret = esp_xtensa_info->run_func_image(
		bank->target,
		&run,
		&flasher_image,
		2,
		esp_xtensa_info->compression ? ESP_XTENSA_STUB_CMD_FLASH_WRITE_DEFLATED :
		ESP_XTENSA_STUB_CMD_FLASH_WRITE,
		/* cmd */
		0
		/* esp_xtensa_stub_flash_write_args */);

	if (compressed_buff)
		free(compressed_buff);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to write flash (%d)!", run.ret_code);
		ret = ERROR_FAIL;
	}
	return ret;
}

static int esp_xtensa_read_xfer(struct target *target, uint32_t block_id, uint32_t len, void *priv)
{
	struct esp_xtensa_read_state *state = (struct esp_xtensa_read_state *)priv;
	int retval;

	/* check for target to get connected */
	if (!state->rw.connected) {
		retval =
			esp_xtensa_apptrace_ctrl_reg_read(target, NULL, NULL, &state->rw.connected);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace control reg (%d)!", retval);
			return retval;
		}
		if (!state->rw.connected)
			return ERROR_WAIT;
	}

	if (len == 0)
		return ERROR_WAIT;

	retval = esp_xtensa_apptrace_data_read(target, len, state->rd_buf, block_id, 1 /*ack*/);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read apptrace status (%d)!", retval);
		return retval;
	}
	LOG_DEBUG("DATA %d bytes: %x %x %x %x %x %x %x %x", len,
		state->rd_buf[0], state->rd_buf[1], state->rd_buf[2], state->rd_buf[3],
		state->rd_buf[4], state->rd_buf[5], state->rd_buf[6], state->rd_buf[7]);

	uint8_t *ptr = state->rd_buf;
	while (ptr < state->rd_buf + len) {
		uint32_t data_sz = 0;
		ptr = esp_apptrace_usr_block_get(ptr, &data_sz);
		if (data_sz > 0)
			memcpy(state->rw.buffer + state->rw.total_count, ptr, data_sz);
		ptr += data_sz;
		state->rw.total_count += data_sz;
	}

	return ERROR_OK;
}

static int esp_xtensa_read_state_init(struct target *target,
	struct xtensa_algo_run_data *run,
	struct esp_xtensa_read_state *state)
{
	/* clear control register, stub will set XTENSA_APPTRACE_HOST_CONNECT bit when it will be
	 * ready */
	int ret = esp_xtensa_apptrace_ctrl_reg_write(target,
		0 /*block_id*/,
		0 /*len*/,
		false /*conn*/,
		false /*data*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to clear apptrace ctrl reg (%d)!", ret);
		return ret;
	}

	state->rd_buf = malloc(esp_xtensa_apptrace_block_max_size_get(target));
	if (!state->rd_buf) {
		LOG_ERROR("Failed to alloc read buffer!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int esp_xtensa_read(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct esp_xtensa_read_state rd_state;
	struct xtensa_algo_image flasher_image;

	if (offset & 0x3UL) {
		LOG_ERROR("Unaligned offset!");
		return ERROR_FAIL;
	}
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1024;
	run.usr_func_init = (xtensa_algo_usr_func_init_t)esp_xtensa_read_state_init;
	run.usr_func = esp_xtensa_rw_do;
	run.usr_func_arg = &rd_state;
	memset(&rd_state, 0, sizeof(struct esp_xtensa_read_state));
	rd_state.rw.buffer = buffer;
	rd_state.rw.count = count;
	rd_state.rw.xfer = esp_xtensa_read_xfer;

	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		3,
		ESP_XTENSA_STUB_CMD_FLASH_READ,
		/* cmd */
		esp_xtensa_info->hw_flash_base + offset,
		/* start addr */
		count);						/* size */

	free(rd_state.rd_buf);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to read flash (%d)!", run.ret_code);
		ret = ERROR_FAIL;
	}
	return ret;
}

#define BANK_SUBNAME(_b_, _n_)  (strcmp((_b_)->name + strlen((_b_)->name) - strlen(_n_), _n_) == 0)

int esp_xtensa_probe(struct flash_bank *bank)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct esp_xtensa_flash_mapping flash_map = {.maps_num = 0};
	uint32_t irom_base = 0, irom_sz = 0, drom_base = 0, drom_sz = 0, irom_flash_base = 0,
		drom_flash_base = 0;

	esp_xtensa_info->probed = 0;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("Flash size = %d KB @ "TARGET_ADDR_FMT " '%s' - '%s'",
		bank->size/1024,
		bank->base,
		target_name(bank->target),
		target_state_name(bank->target));

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	int ret = esp_xtensa_get_mappings(bank,
		esp_xtensa_info,
		&flash_map,
		esp_xtensa_info->appimage_flash_base);
	if (ret != ERROR_OK || flash_map.maps_num == 0) {
		LOG_WARNING("Failed to get flash mappings (%d)!", ret);
		/* if no DROM/IROM mappings so pretend they are at the end of the HW flash bank and
		 * have zero size to allow correct memory map with non zero RAM region */
		irom_base = drom_base = esp_xtensa_get_size(bank);
	} else {
		for (uint32_t i = 0; i < flash_map.maps_num; i++) {
			if (esp_xtensa_info->is_irom_address(flash_map.maps[i].load_addr)) {
				irom_flash_base = flash_map.maps[i].phy_addr &
					~(esp_xtensa_info->sec_sz-1);
				irom_base = flash_map.maps[i].load_addr &
					~(esp_xtensa_info->sec_sz-1);
				irom_sz = flash_map.maps[i].size;
				if (irom_sz & (esp_xtensa_info->sec_sz-1))
					irom_sz = (irom_sz & ~(esp_xtensa_info->sec_sz-1)) +
						esp_xtensa_info->sec_sz;
			} else if (esp_xtensa_info->is_drom_address(flash_map.maps[i].load_addr)) {
				drom_flash_base = flash_map.maps[i].phy_addr &
					~(esp_xtensa_info->sec_sz-1);
				drom_base = flash_map.maps[i].load_addr &
					~(esp_xtensa_info->sec_sz-1);
				drom_sz = flash_map.maps[i].size;
				if (drom_sz & (esp_xtensa_info->sec_sz-1))
					drom_sz = (drom_sz & ~(esp_xtensa_info->sec_sz-1)) +
						esp_xtensa_info->sec_sz;
			}
		}
	}

	if (BANK_SUBNAME(bank, ".irom")) {
		esp_xtensa_info->hw_flash_base = irom_flash_base;
		bank->base = irom_base;
		bank->size = irom_sz;
	} else if (BANK_SUBNAME(bank, ".drom")) {
		esp_xtensa_info->hw_flash_base = drom_flash_base;
		bank->base = drom_base;
		bank->size = drom_sz;
	} else {
		esp_xtensa_info->hw_flash_base = 0;
		bank->size = esp_xtensa_get_size(bank);
		if (bank->size == 0) {
			LOG_ERROR("Failed to probe flash, size %d KB", bank->size/1024);
			return ERROR_FAIL;
		}
		LOG_INFO("Auto-detected flash bank '%s' size %d KB", bank->name, bank->size/1024);
	}
	LOG_INFO("Using flash bank '%s' size %d KB", bank->name, bank->size/1024);

	if (bank->size) {
		/* Bank size can be 0 for IROM/DROM emulated banks when there is no app in flash */
		bank->num_sectors = bank->size / esp_xtensa_info->sec_sz;
		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
		if (bank->sectors == NULL) {
			LOG_ERROR("Failed to alloc mem for sectors!");
			return ERROR_FAIL;
		}
		for (int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = i*esp_xtensa_info->sec_sz;
			bank->sectors[i].size = esp_xtensa_info->sec_sz;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 0;
		}
	}
	LOG_DEBUG("allocated %d sectors", bank->num_sectors);
	esp_xtensa_info->probed = 1;

	return ERROR_OK;
}

int esp_xtensa_auto_probe(struct flash_bank *bank)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	if (esp_xtensa_info->probed)
		return ERROR_OK;
	return esp_xtensa_probe(bank);
}

static int esp_xtensa_flash_bp_op_state_init(struct target *target,
	struct xtensa_algo_run_data *run,
	struct esp_xtensa_flash_bp_op_state *state)
{
	/* aloocate target buffer for temp storage of flash sections contents when modifying
	 * instruction */
	LOG_DEBUG("SEC_SIZE %d", state->esp_xtensa_info->sec_sz);
	int ret = target_alloc_alt_working_area(target,
		2*(state->esp_xtensa_info->sec_sz),
		&state->target_buf);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to alloc target buffer for insn sectors!");
		return ret;
	}
	buf_set_u32(run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+3].value,
		0,
		32,
		state->target_buf->address);										/*
															 * insn
															 * sectors */

	return ERROR_OK;
}

static void esp_xtensa_flash_bp_op_state_cleanup(struct target *target,
	struct xtensa_algo_run_data *run,
	struct esp_xtensa_flash_bp_op_state *state)
{
	if (!state->target_buf)
		return;
	target_free_alt_working_area(target, state->target_buf);
}

int esp_xtensa_flash_breakpoint_add(struct target *target,
	struct breakpoint *breakpoint,
	struct esp_xtensa_flash_breakpoint *sw_bp)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info;
	struct xtensa_algo_run_data run;
	struct flash_bank *bank;
	struct esp_xtensa_flash_bp_op_state op_state;
	struct mem_param mp;
	struct xtensa_algo_image flasher_image;

	/* flash belongs to root target, so we need to find flash using it instead of core
	 * sub-target */
	int ret = get_flash_bank_by_addr(target, breakpoint->address, true, &bank);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to get flash bank (%d)!", target_name(target), ret);
		return ret;
	}

	/* can set set breakpoints in mapped app regions only */
	if (!BANK_SUBNAME(bank, ".irom")) {
		LOG_ERROR("%s: Can not set BP outside of IROM (BP addr " TARGET_ADDR_FMT ")!",
			target_name(target),
			breakpoint->address);
		return ERROR_FAIL;
	}

	esp_xtensa_info = bank->driver_priv;

	op_state.esp_xtensa_info = esp_xtensa_info;
	LOG_DEBUG("SEC_SIZE %d", esp_xtensa_info->sec_sz);
	memset(&run, 0, sizeof(run));
	run.stack_size = 1300;
	run.usr_func_arg = &op_state;
	run.usr_func_init = (xtensa_algo_usr_func_init_t)esp_xtensa_flash_bp_op_state_init;
	run.usr_func_done = (xtensa_algo_usr_func_done_t)esp_xtensa_flash_bp_op_state_cleanup;

	sw_bp->data.oocd_bp = breakpoint;
	sw_bp->priv = bank;

	ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	init_mem_param(&mp, 2 /*2nd usr arg*/, 3 /*size in bytes*/, PARAM_IN);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;
	uint32_t bp_flash_addr = esp_xtensa_info->hw_flash_base +
		(breakpoint->address - bank->base);
	ret = esp_xtensa_info->run_func_image(target,
		&run,
		&flasher_image,
		4 /*args num*/,
		ESP_XTENSA_STUB_CMD_FLASH_BP_SET /*cmd*/,
		bp_flash_addr /*bp_addr*/,
		0 /*address to store insn*/,
		0 /*address to store insn sectors*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to run flasher stub (%d)!", target_name(target), ret);
		destroy_mem_param(&mp);
		sw_bp->data.oocd_bp = NULL;
		return ret;
	}
	if (run.ret_code == 0) {
		LOG_ERROR("%s: Failed to set bp (%d)!", target_name(target), run.ret_code);
		destroy_mem_param(&mp);
		sw_bp->data.oocd_bp = NULL;
		return ERROR_FAIL;
	}
	sw_bp->data.insn_sz = run.ret_code;
	memcpy(sw_bp->data.insn, mp.value, 3);
	destroy_mem_param(&mp);
	LOG_DEBUG(
		"%s: Placed flash SW breakpoint at " TARGET_ADDR_FMT
		", insn [%02x %02x %02x] %d bytes",
		target_name(target),
		breakpoint->address,
		sw_bp->data.insn[0],
		sw_bp->data.insn[1],
		sw_bp->data.insn[2],
		sw_bp->data.insn_sz);

	return ERROR_OK;
}

int esp_xtensa_flash_breakpoint_remove(struct target *target,
	struct esp_xtensa_flash_breakpoint *sw_bp)
{
	struct flash_bank *bank = (struct flash_bank *)(sw_bp->priv);
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct esp_xtensa_flash_bp_op_state op_state;
	struct mem_param mp;
	struct xtensa_algo_image flasher_image;

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	op_state.esp_xtensa_info = esp_xtensa_info;
	memset(&run, 0, sizeof(run));
	run.stack_size = 1300;
	run.usr_func_arg = &op_state;
	run.usr_func_init = (xtensa_algo_usr_func_init_t)esp_xtensa_flash_bp_op_state_init;
	run.usr_func_done = (xtensa_algo_usr_func_done_t)esp_xtensa_flash_bp_op_state_cleanup;

	init_mem_param(&mp, 2 /*2nd usr arg*/, 3 /*size in bytes*/, PARAM_OUT);
	memcpy(mp.value, sw_bp->data.insn, 3);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;

	uint32_t bp_flash_addr = esp_xtensa_info->hw_flash_base +
		(sw_bp->data.oocd_bp->address - bank->base);
	LOG_DEBUG(
		"%s: Remove flash SW breakpoint at " TARGET_ADDR_FMT
		", insn [%02x %02x %02x] %d bytes",
		target_name(target),
		sw_bp->data.oocd_bp->address,
		sw_bp->data.insn[0],
		sw_bp->data.insn[1],
		sw_bp->data.insn[2],
		sw_bp->data.insn_sz);
	ret = esp_xtensa_info->run_func_image(target,
		&run,
		&flasher_image,
		4 /*args num*/,
		ESP_XTENSA_STUB_CMD_FLASH_BP_CLEAR /*cmd*/,
		bp_flash_addr /*bp_addr*/,
		0 /*address with insn*/,
		0 /*address to store insn sectors*/);
	destroy_mem_param(&mp);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to clear bp (%d)!", run.ret_code);
		return ERROR_FAIL;
	}

	sw_bp->data.oocd_bp = NULL;
	return ret;
}

static int esp_xtensa_flash_calc_hash(struct flash_bank *bank, uint8_t *hash,
	uint32_t offset, uint32_t count)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct xtensa_algo_image flasher_image;

	if (offset & 0x3UL) {
		LOG_ERROR("Unaligned offset!");
		return ERROR_FAIL;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1024 + ESP_XTENSA_STUB_RDWR_BUFF_SIZE;

	struct mem_param mp;
	init_mem_param(&mp,
		3 /*2nd usr arg*/,
		32 /*sha256 hash size in bytes*/,
		PARAM_IN);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;

	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		4 /*args num*/,
		ESP_XTENSA_STUB_CMD_FLASH_CALC_HASH /*cmd*/,
		esp_xtensa_info->hw_flash_base + offset,
		count,
		0 /*address to store hash value*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		destroy_mem_param(&mp);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to get hash value (%d)!", run.ret_code);
		ret = ERROR_FAIL;
	} else
		memcpy(hash, mp.value, 32);
	destroy_mem_param(&mp);
	return ret;
}

static int esp_xtensa_boost_clock_freq(struct flash_bank *bank, int boost)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct xtensa_algo_image flasher_image;
	int new_cpu_freq = -1;	/* set to max level */

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	/* restore */
	if (boost == 0)
		new_cpu_freq = esp_xtensa_info->old_cpu_freq;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1024;
	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		2,
		ESP_XTENSA_STUB_CMD_CLOCK_CONFIGURE,
		new_cpu_freq);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ERROR_FAIL;
	}
	esp_xtensa_info->old_cpu_freq = run.ret_code;
	LOG_DEBUG("%s old_freq (%d) new_freq (%d)",
		__func__,
		esp_xtensa_info->old_cpu_freq,
		new_cpu_freq);
	return ERROR_OK;
}

static int esp_xtensa_target_to_flash_bank(struct target *target,
	struct flash_bank **bank, char *bank_name_suffix, bool probe)
{
	if (target == NULL || bank == NULL)
		return ERROR_FAIL;

	char bank_name[64];
	int retval = snprintf(bank_name,
		sizeof(bank_name),
		"%s.%s",
		target_name(target),
		bank_name_suffix);
	if (retval == sizeof(bank_name)) {
		LOG_ERROR("Failed to build bank name string!");
		return ERROR_FAIL;
	}
	if (probe) {
		retval = get_flash_bank_by_name(bank_name, bank);
		if (retval != ERROR_OK || bank == NULL) {
			LOG_ERROR("Failed to find bank '%s'!",  bank_name);
			return retval;
		}
	} else {/* noprobe */
		*bank = get_flash_bank_by_name_noprobe(bank_name);
		if (*bank == NULL) {
			LOG_ERROR("Failed to find bank '%s'!",  bank_name);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int esp_xtensa_appimage_flash_base_update(struct target *target,
	char *bank_name_suffix,
	uint32_t appimage_flash_base)
{
	struct flash_bank *bank;
	struct esp_xtensa_flash_bank *esp_xtensa_info;

	int retval = esp_xtensa_target_to_flash_bank(target, &bank, bank_name_suffix, false);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	esp_xtensa_info = (struct esp_xtensa_flash_bank *)bank->driver_priv;
	esp_xtensa_info->probed = 0;
	esp_xtensa_info->appimage_flash_base = appimage_flash_base;
	retval = bank->driver->auto_probe(bank);
	return retval;
}

COMMAND_HELPER(esp_xtensa_cmd_appimage_flashoff_do, struct target *target)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Flash offset not specified!");
		return ERROR_FAIL;
	}

	/* update app image base */
	uint32_t appimage_flash_base = strtoul(CMD_ARGV[0], NULL, 16);

	int ret = esp_xtensa_appimage_flash_base_update(target, "irom", appimage_flash_base);
	if (ret != ERROR_OK)
		return ret;
	ret = esp_xtensa_appimage_flash_base_update(target, "drom", appimage_flash_base);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

COMMAND_HANDLER(esp_xtensa_cmd_appimage_flashoff)
{
	return CALL_COMMAND_HANDLER(esp_xtensa_cmd_appimage_flashoff_do,
		get_current_target(CMD_CTX));
}

static int esp_xtensa_set_compression(struct target *target,
	char *bank_name_suffix,
	int compression)
{
	struct flash_bank *bank;
	struct esp_xtensa_flash_bank *esp_xtensa_info;

	int retval = esp_xtensa_target_to_flash_bank(target, &bank, bank_name_suffix, true);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	esp_xtensa_info = (struct esp_xtensa_flash_bank *)bank->driver_priv;
	esp_xtensa_info->compression = compression;
	return ERROR_OK;
}

COMMAND_HELPER(esp_xtensa_cmd_set_compression, struct target *target)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Compression not specified!");
		return ERROR_FAIL;
	}

	int compression = 0;

	if (0 == strcmp("on", CMD_ARGV[0])) {
		LOG_DEBUG("Flash compressed upload is on");
		compression = 1;
	} else if (0 == strcmp("off", CMD_ARGV[0])) {
		LOG_DEBUG("Flash compressed upload is off");
		compression = 0;
	} else {
		LOG_DEBUG("unknown flag");
		return ERROR_FAIL;
	}

	return esp_xtensa_set_compression(target, "flash", compression);
}

COMMAND_HANDLER(esp_xtensa_cmd_compression)
{
	return CALL_COMMAND_HANDLER(esp_xtensa_cmd_set_compression,
		get_current_target(CMD_CTX));
}

static int esp_xtensa_verify_bank_hash(struct target *target,
	uint32_t offset,
	const char *file_name)
{
	uint8_t file_hash[TC_SHA256_DIGEST_SIZE], target_hash[TC_SHA256_DIGEST_SIZE];
	uint8_t *buffer_file;
	struct fileio *fileio;
	size_t filesize, length, read_cnt;
	int differ, retval;
	struct flash_bank *bank;

	retval = esp_xtensa_target_to_flash_bank(target, &bank, "flash", true);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	if (offset > bank->size) {
		LOG_ERROR("Offset 0x%8.8" PRIx32 " is out of range of the flash bank",
			offset);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	retval = fileio_open(&fileio, file_name, FILEIO_READ, FILEIO_BINARY);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not open file");
		return retval;
	}

	retval = fileio_size(fileio, &filesize);
	if (retval != ERROR_OK) {
		fileio_close(fileio);
		return retval;
	}

	length = MIN(filesize, bank->size - offset);

	if (!length) {
		LOG_INFO("Nothing to compare with flash bank");
		fileio_close(fileio);
		return ERROR_OK;
	}

	if (length != filesize)
		LOG_WARNING("File content exceeds flash bank size. Only comparing the "
			"first %zu bytes of the file", length);

	LOG_DEBUG("File size: %zu bank_size: %u offset: %u",
		filesize, bank->size, offset);

	buffer_file = malloc(length);
	if (buffer_file == NULL) {
		LOG_ERROR("Out of memory");
		fileio_close(fileio);
		return ERROR_FAIL;
	}

	retval = fileio_read(fileio, length, buffer_file, &read_cnt);
	fileio_close(fileio);
	if (retval != ERROR_OK || read_cnt != length) {
		LOG_ERROR("File read failure");
		free(buffer_file);
		return retval;
	}

	retval = esp_xtensa_calc_hash(buffer_file, length, file_hash);
	free(buffer_file);
	if (retval != ERROR_OK) {
		LOG_ERROR("File sha256 calculation failure");
		return retval;
	}

	retval = esp_xtensa_flash_calc_hash(bank, target_hash, offset, length);
	if (retval != ERROR_OK) {
		LOG_ERROR("Flash sha256 calculation failure");
		return retval;
	}

	differ = memcmp(file_hash, target_hash, TC_SHA256_DIGEST_SIZE);

	if (differ) {
		LOG_ERROR("**** Verification failure! ****");
		LOG_ERROR("target_hash %x%x%x...%x%x%x",
			target_hash[0], target_hash[1], target_hash[2],
			target_hash[29], target_hash[30], target_hash[31]);
		LOG_ERROR("file_hash: %x%x%x...%x%x%x",
			file_hash[0], file_hash[1], file_hash[2],
			file_hash[29], file_hash[30], file_hash[31]);
	}

	return differ ? ERROR_FAIL : ERROR_OK;
}

COMMAND_HELPER(esp_xtensa_parse_cmd_verify_bank_hash, struct target *target)
{
	if (CMD_ARGC < 2 || CMD_ARGC > 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t offset = 0;

	if (CMD_ARGC > 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], offset);

	return esp_xtensa_verify_bank_hash(target, offset, CMD_ARGV[1]);
}

COMMAND_HANDLER(esp_xtensa_cmd_verify_bank_hash)
{
	return CALL_COMMAND_HANDLER(esp_xtensa_parse_cmd_verify_bank_hash,
		get_current_target(CMD_CTX));
}

COMMAND_HELPER(esp_xtensa_parse_cmd_clock_boost, struct target *target)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Clock boost flag not specified!");
		return ERROR_FAIL;
	}

	int boost = 0;

	if (0 == strcmp("on", CMD_ARGV[0])) {
		LOG_DEBUG("Clock boost is on");
		boost = 1;
	} else if (0 == strcmp("off", CMD_ARGV[0])) {
		LOG_DEBUG("Clock boost is off");
		boost = 0;
	} else {
		LOG_DEBUG("unknown flag");
		return ERROR_FAIL;
	}

	struct flash_bank *bank;
	int retval = esp_xtensa_target_to_flash_bank(target, &bank, "flash", true);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	return esp_xtensa_boost_clock_freq(bank, boost);
}

COMMAND_HANDLER(esp_xtensa_cmd_clock_boost)
{
	return CALL_COMMAND_HANDLER(esp_xtensa_parse_cmd_clock_boost,
		get_current_target(CMD_CTX));
}

const struct command_registration esp_xtensa_exec_flash_command_handlers[] = {
	{
		.name = "appimage_offset",
		.handler = esp_xtensa_cmd_appimage_flashoff,
		.mode = COMMAND_ANY,
		.help =
			"Set offset of application image in flash. Use -1 to debug the first application image from partition table.",
		.usage = "offset",
	},
	{
		.name = "compression",
		.handler = esp_xtensa_cmd_compression,
		.mode = COMMAND_ANY,
		.help =
			"Set compression flag",
		.usage = "['on'|'off']",
	},
	{
		.name = "verify_bank_hash",
		.handler = esp_xtensa_cmd_verify_bank_hash,
		.mode = COMMAND_ANY,
		.help = "Perform a comparison between the file and the contents of the "
			"flash bank using SHA256 hash values. Allow optional offset from beginning of the bank "
			"(defaults to zero).",
		.usage = "bank_id filename [offset]",
	},
	{
		.name = "flash_stub_clock_boost",
		.handler = esp_xtensa_cmd_clock_boost,
		.mode = COMMAND_ANY,
		.help =
			"Set cpu clock freq to the max level. Use 'off' to restore the clock speed",
		.usage = "['on'|'off']",
	},
	COMMAND_REGISTRATION_DONE
};
