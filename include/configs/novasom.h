/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Wandboard.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <linux/sizes.h>

#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

/*#define MACH_TYPE_WANDBOARD		4412*/
#define MACH_TYPE_WANDBOARD		4415
#define CONFIG_MACH_TYPE		MACH_TYPE_WANDBOARD

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_SYS_GENERIC_BOARD

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_SETEXPR

#define CONFIG_BOOTDELAY		3

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)
#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

/* MMC Configuration */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

/* Ethernet Configuration */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

/* Framebuffer */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE (1 << 20)
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#if defined(CONFIG_MX6DL) || defined(CONFIG_MX6S)
#define CONFIG_DEFAULT_FDT_FILE		"imx6dl-wandboard.dtb"
#elif defined(CONFIG_MX6Q)
#define CONFIG_DEFAULT_FDT_FILE		"imx6q-novasom.dtb"
#endif

#include "novasis_display_device.h"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"serverip=192.168.10.39\0" \
        "Display=" __stringify(DISPLAY_DEVICE) "\0" \
        "ethaddr=00:04:9F:01:F8:A5\0" \
        "ipaddr=192.168.10.69\0" \
        "bootenv=NOVAsom6.params\0" \
	"uimage=NOVAsom6Kernel\0" \
	"fs_image=NOVAsom6Initrd\0" \
	"console=ttymxc0\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"fdt_addr=0x11f00000\0" \
	"fs_addr=0x12800000\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"mmcroot=/dev/mmcblk0p2 rw rootwait mem=1G\0" \
	"ramroot=/dev/ram mem=1G ramdisk_size=440000\0" \
	"default_video=video=mxcfb0:dev=hdmi,1920x1080M@30,if=RGB24\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} root=${mmcroot} ${default_video}\0" \
	"ramargs=setenv bootargs console=${console},${baudrate} root=${ramroot} ${default_video}\0" \
        "loadbootenv=fatload mmc ${mmcdev}:1  ${loadaddr} ${bootenv}\0" \
	"loaduimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${uimage}\0" \
	"loadinitrd=fatload mmc ${mmcdev}:${mmcpart} ${fs_addr} ${fs_image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
        "importbootenv=echo Importing environment ...; env import -t ${loadaddr} ${filesize}\0" \
	"mmcboot=run loadfdt;bootm ${loadaddr} ${fs_addr} ${fdt_addr}\0" \
	"netboot=echo Booting from net ...; " \
		"run ramargs; "\
		"bootp $loadaddr ${serverip}:${uimage} ; tftp $fdt_addr ${serverip}:${fdt_file};tftp $fs_addr ${serverip}:${fs_image};" \
		"bootm ${loadaddr} ${fs_addr} ${fdt_addr}\0" \
        "splashimage_mmc_dev=0\0" \
        "splashimage_mmc_part=1\0" \
        "splashimage=0x10800000\0" \
        "splashpos=m,m\0" \
        "splashimage_file_name=splash.bmp.gz\0"  \
	"final_cmd=FINAL;\0"

#define CONFIG_BOOTCOMMAND \
        "if test -n ${bootenv}; then " \
                "if run loadbootenv; then " \
                        "echo Loaded environment ${bootenv};" \
                        "run importbootenv;" \
                        "run uenvcmd;" \
                "fi;" \
        "fi;" \
	"run mmcboot"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	       16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#endif			       /* __CONFIG_H * */
