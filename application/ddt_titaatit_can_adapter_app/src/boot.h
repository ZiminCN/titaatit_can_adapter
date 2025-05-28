#pragma once
#undef CAN_MODE_NORMAL
#undef CAN_MODE_LOOPBACK
#include <zephyr/arch/arm/aarch32/cortex_m/cmsis.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/can.h>

struct _id {
	uint32_t sw_version;
	uint32_t mcu_id[3];
};

static void cleanup_arm_nvic(void)
{
	/* Allow any pending interrupts to be recognized */
	__ISB();
	__disable_irq();

	/* Disable NVIC interrupts */
	for (uint8_t i = 0; i < ARRAY_SIZE(NVIC->ICER); i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
	}
	/* Clear pending NVIC interrupts */
	for (uint8_t i = 0; i < ARRAY_SIZE(NVIC->ICPR); i++) {
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}
}

struct arm_vector_table {
	uint32_t msp;
	uint32_t reset;
};

__attribute__((unused)) static void jump_to_boot(void)
{
	struct arm_vector_table *vt;

	/*application base address = bootloader base address + bootloader size*/
	vt = (struct arm_vector_table *)(CONFIG_FLASH_BASE_ADDRESS);

	LOG_DBG("vt->reset:0x%08x value:0x%08x", (uint32_t)&vt->reset, vt->reset);

	if (IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)) {
		sys_clock_disable();
	}

	cleanup_arm_nvic(); /* cleanup NVIC registers */
	__set_MSP(vt->msp);

	__set_CONTROL(0x00); /* application will configures core on its own */
	__ISB();

	LOG_DBG("\r\n");

	((void (*)(void))vt->reset)();
}

/*boot_arg*/
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/storage/flash_map.h>

#define BOOT_ARG_MAGIC 0xDEADC0DE

#define BOOT_ARG_FLAG_NORMAL_BOOT 0
#define BOOT_ARG_FLAG_UPDATE_APP  1
#define BOOT_ARG_FLAG_RECOVERY	  2
#define BOOT_ARG_FLAG_NOT_APP	  3
#define BOOT_ARG_FLAG_APP_BAD	  4

static inline const char *boot_flag_str(int flag)
{
	switch (flag) {
	case BOOT_ARG_FLAG_NORMAL_BOOT:
		return "normal boot";
		break;
	case BOOT_ARG_FLAG_UPDATE_APP:
		return "update app";
		break;
	case BOOT_ARG_FLAG_RECOVERY:
		return "recovery mode";
		break;
	case BOOT_ARG_FLAG_NOT_APP:
		return "not find app";
		break;
	case BOOT_ARG_FLAG_APP_BAD:
		return "app boot abnormal";
		break;
	default:
		return "unrecognize flag";
		break;
	}
}

struct boot_arg {
	int magic; // 0xdeadc0de
	short app_boot_cnt;
	short bl_boot_cnt;
	int flag; // BOOT_ARG_FLAG_[...]
	int app_version;
	int app_checksum;
	int app_size;
	int timestamp;
	int cnt; // write cnt of boot_arg
};

#define BOOT_ARG_AREA FIXED_PARTITION_ID(boot_arg_partition)

static void boot_arg_init(struct boot_arg *arg)
{
	arg->app_size = 0;
	arg->app_checksum = 0;
	arg->magic = BOOT_ARG_MAGIC;
	arg->bl_boot_cnt = 1;
	arg->app_version = 0;
	arg->cnt = 0;
	arg->timestamp = 0;
#ifdef CONFIG_BOOTLOADER_DDTBOOT
	arg->flag = BOOT_ARG_FLAG_NORMAL_BOOT;
#else
	arg->flag = BOOT_ARG_FLAG_NOT_APP;
#endif
}

#ifdef CONFIG_BOOTLOADER_DDTBOOT
static void boot_status_update(struct boot_arg *arg)
{
	if (arg->flag == BOOT_ARG_FLAG_NORMAL_BOOT) {
		if (arg->bl_boot_cnt == 0) {
			arg->bl_boot_cnt = 1;
			arg->flag = BOOT_ARG_FLAG_NORMAL_BOOT;
		} else if (arg->bl_boot_cnt == 0) {
			arg->flag = BOOT_ARG_FLAG_RECOVERY;
		} else {
			arg->flag = BOOT_ARG_FLAG_NORMAL_BOOT;
		}
	}
}
#endif

static int boot_area_free_cnt(void)
{
	const struct flash_area *fap;
	struct flash_sector boot_arg_sector;
	struct boot_arg arg;
	int sec_cnt = 1;
	int i;

	flash_area_open(BOOT_ARG_AREA, &fap);

	flash_area_get_sectors(BOOT_ARG_AREA, &sec_cnt, &boot_arg_sector);

	for (i = 0; i < boot_arg_sector.fs_size; i += sizeof(struct boot_arg)) {
		flash_area_read(fap, i, &arg, sizeof(struct boot_arg));
		if (arg.magic == BOOT_ARG_MAGIC) {
			break;
		} else if (arg.magic != 0xFFFFFFFF) {
			return 0; // data err
		}
	}
	flash_area_close(fap);

	return i / sizeof(struct boot_arg);
}

__attribute__((unused)) static void boot_arg_get(struct boot_arg *arg)
{
	const struct flash_area *fap;
	struct flash_sector boot_arg_sector;
	int sec_cnt = 1;

	memset((void *)arg, 0, sizeof(struct boot_arg));

	flash_area_open(BOOT_ARG_AREA, &fap);

	flash_area_get_sectors(BOOT_ARG_AREA, &sec_cnt, &boot_arg_sector);

	for (int i = 0; i < boot_arg_sector.fs_size; i += sizeof(struct boot_arg)) {
		flash_area_read(fap, i, arg, sizeof(struct boot_arg));
		if (arg->magic == BOOT_ARG_MAGIC) {
#ifdef CONFIG_BOOTLOADER_DDTBOOT
			boot_status_update(arg);
#endif
			goto exit;
		} else if (arg->magic != 0xFFFFFFFF) {
			boot_arg_init(arg);
			goto exit;
		}
	}

	arg->flag = BOOT_ARG_FLAG_NOT_APP;
exit:
	flash_area_close(fap);
}

__attribute__((unused)) static void boot_arg_save(struct boot_arg *arg)
{
	const struct flash_area *fap;
	struct flash_sector boot_arg_sector;
	int sec_cnt = 1;
	int free_cnt = boot_area_free_cnt();
	int rst = 0, retry = 3;

	if (arg->magic != BOOT_ARG_MAGIC) {
		return;
	}

	flash_area_open(BOOT_ARG_AREA, &fap);

	flash_area_get_sectors(BOOT_ARG_AREA, &sec_cnt, &boot_arg_sector);

	if (free_cnt == 0) {
		flash_area_erase(fap, 0, boot_arg_sector.fs_size);
		free_cnt = boot_area_free_cnt();
	}

	arg->cnt++;
	do {
		rst = flash_area_write(fap, (free_cnt - 1) * sizeof(struct boot_arg), arg,
				       sizeof(struct boot_arg));
		retry--;
	} while (rst != 0 && retry > 0);

	flash_area_close(fap);
}
