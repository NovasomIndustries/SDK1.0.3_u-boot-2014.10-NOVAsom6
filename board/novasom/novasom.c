/*
 * Copyright (C) 2014 Novasis Ingegneria
 *
 * Author: Fil <filippo.visocchi@novasis.it>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>

#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <ipu_pixfmt.h>
#include <mmc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>

extern int get_mac(unsigned char *mac);

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL    (PAD_CTL_PUS_100K_UP |                  \
        PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
        PAD_CTL_ODE | PAD_CTL_SRE_FAST)

//#define USDHC3_CD_GPIO			IMX_GPIO_NR(3, 5)
//GPIO_7__GPIO1_IO07
#define USDHC3_CD_GPIO			IMX_GPIO_NR(1, 7)

#define NOVASOM6_WIFION			IMX_GPIO_NR(3, 9)  /* MX6Q_PAD_EIM_DA9__GPIO3_IO09 */

#define NOVASOM6_LVDS_EN                IMX_GPIO_NR(3, 7)       /* MX6Q_PAD_EIM_DA7__GPIO3_IO07 */
#define NOVASOM6_LVDSBACKLIGHT_EN       IMX_GPIO_NR(3, 8)       /* MX6Q_PAD_EIM_DA8__GPIO3_IO08 */
#define NOVASOM6_LVDSPWM2	        IMX_GPIO_NR(1, 13)       /* MX6Q_PAD_SD2_DAT2__GPIO1_IO13 */
#define NOVASOM6_LVDSPWM3	        IMX_GPIO_NR(1, 12)       /* MX6Q_PAD_SD2_DAT3__GPIO1_IO12 */

int dram_init(void)
{
	gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;

	return 0;
}

iomux_v3_cfg_t const usdhc3_pads[] = {
	/* MicroSD */
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	/* eMMC */
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_ALE__SD4_RESET   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const enet_pads_boot[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* KSZ9031 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* KSZ9031 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const enet_rst_pads[] = {
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define NOVASOM6_ETH_RGMII_RXD0 	IMX_GPIO_NR(6, 25) /* MX6Q_PAD_RGMII_RD0__GPIO_6_25 */
#define NOVASOM6_ETH_RGMII_RXD1 	IMX_GPIO_NR(6, 27) /* MX6Q_PAD_RGMII_RD1__GPIO_6_27 */
#define NOVASOM6_ETH_RGMII_RXD2 	IMX_GPIO_NR(6, 28) /* MX6Q_PAD_RGMII_RD2__GPIO_6_28 */
#define NOVASOM6_ETH_RGMII_RXD3 	IMX_GPIO_NR(6, 29) /* MX6Q_PAD_RGMII_RD3__GPIO_6_29 */
#define NOVASOM6_RGMII_RST      	IMX_GPIO_NR(1, 25) /* MX6Q_PAD_ENET_CRS_DV__GPIO_1_25 */
static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads_boot, ARRAY_SIZE(enet_pads_boot));
        udelay(1000 * 20);
        udelay(1000 * 20);
        udelay(1000 * 20);
        udelay(1000 * 20);
        udelay(1000 * 20);
	gpio_direction_output(NOVASOM6_ETH_RGMII_RXD3, 1);
	gpio_direction_output(NOVASOM6_ETH_RGMII_RXD2, 1);
	gpio_direction_output(NOVASOM6_ETH_RGMII_RXD1, 0);
	gpio_direction_output(NOVASOM6_ETH_RGMII_RXD0, 1);
	/* KSZ9031 PHY Reset */
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	gpio_direction_output(NOVASOM6_RGMII_RST, 0);
        udelay(1000 * 20);
        udelay(1000 * 20);
	/* KSZ9031 PHY Out of Reset */
	gpio_set_value(NOVASOM6_RGMII_RST, 1);
        udelay(1000 * 20);
        udelay(1000 * 20);
        udelay(1000 * 20);
        udelay(1000 * 20);
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wifi_pads[] = {
	MX6_PAD_GPIO_8__XTALOSC_REF_CLK_32K | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_DA9__GPIO3_IO09     | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

/*
static void setup_iomux_wifi(void)
{
	imx_iomux_v3_setup_multiple_pads(wifi_pads, ARRAY_SIZE(wifi_pads));
        gpio_direction_output(NOVASOM6_WIFION, 1);
}
*/

static iomux_v3_cfg_t const usdhc_detect_pads[] = {
        /* eMMC */
        MX6_PAD_GPIO_7__GPIO1_IO07     | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
/*
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
		case USDHC3_BASE_ADDR:
			ret = !gpio_get_value(USDHC3_CD_GPIO);
			break;
		case USDHC4_BASE_ADDR:
			ret = gpio_get_value(USDHC3_CD_GPIO);
			break;
	}
	return ret;
*/
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;

	/*
	 * Following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SOM MicroSD
	 * mmc1                    Carrier board MicroSD
	 */
	imx_iomux_v3_setup_multiple_pads( usdhc_detect_pads, ARRAY_SIZE(usdhc_detect_pads));
	gpio_direction_input(USDHC3_CD_GPIO);
	if ( gpio_get_value(USDHC3_CD_GPIO) == 0 )
	{
		printf("SD Card\n");
		imx_iomux_v3_setup_multiple_pads( usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		usdhc_cfg[0].max_bus_width = 4;
		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
	}
	else
	{
		printf("eMMC\n");
		imx_iomux_v3_setup_multiple_pads( usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
		usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
		usdhc_cfg[1].max_bus_width = 8;
		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
	}
	return status;
}

int board_phy_config(struct phy_device *phydev)
{
        /* adjust KSZ9031 ethernet phy */
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x2);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x4);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0xc002);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x0000);

        phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x2);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x5);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0xc002);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x0000);

        phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x2);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x6);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0xc002);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x0000);

        phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x2);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x8);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0xc002);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x3fff);
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x0);

	phydev->autoneg = 0;
	genphy_config_aneg(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#define NOVASOM6_REDLED         	IMX_GPIO_NR(3, 11)  /* MX6_PAD_EIM_DA11__GPIO_3_11 */
#define NOVASOM6_YELLOWLED      	IMX_GPIO_NR(3, 10)  /* MX6_PAD_EIM_DA10__GPIO_3_10 */
#define NOVASOM6_BLUELED        	IMX_GPIO_NR(2, 28)  /* MX6_PAD_EIM_EB0__GPIO_2_28 */
#define NOVASOM6_AUX_ETH_LED    	IMX_GPIO_NR(5, 2)   /* MX6_PAD_EIM_A25__GPIO5_IO02 */

static iomux_v3_cfg_t const led_pads[] = {
	MX6_PAD_EIM_DA11__GPIO3_IO11    | MUX_PAD_CTRL(NO_PAD_CTRL), /* Red led */
	MX6_PAD_EIM_DA10__GPIO3_IO10    | MUX_PAD_CTRL(NO_PAD_CTRL), /* Yellow led */
	MX6_PAD_EIM_EB0__GPIO2_IO28   	| MUX_PAD_CTRL(NO_PAD_CTRL), /* Blue led */
	MX6_PAD_EIM_A25__GPIO5_IO02   	| MUX_PAD_CTRL(NO_PAD_CTRL), /* Aux eth led */
};

int board_led_init(void)
{
        imx_iomux_v3_setup_multiple_pads(led_pads, ARRAY_SIZE(led_pads));
        gpio_direction_output(NOVASOM6_REDLED, 0);
        gpio_direction_output(NOVASOM6_YELLOWLED, 1);
        gpio_direction_output(NOVASOM6_BLUELED, 1);
        gpio_direction_output(NOVASOM6_AUX_ETH_LED, 1);
	return 0;
}

void backlight_on(void)
{
        gpio_direction_output(NOVASOM6_LVDS_EN, 1);
        gpio_direction_output(NOVASOM6_LVDSBACKLIGHT_EN, 1);
}

void backlight_off(void)
{
        gpio_direction_output(NOVASOM6_LVDS_EN, 1);
        gpio_direction_output(NOVASOM6_LVDSBACKLIGHT_EN, 0);
}

#if defined(CONFIG_VIDEO_IPUV3)

struct display_info_t {
	int     bus;
	int     addr;
	int     pixfmt;
	int     (*detect)(struct display_info_t const *dev);
	void    (*enable)(struct display_info_t const *dev);
	struct  fb_videomode mode;
};

static int detect_hdmi(struct display_info_t const *dev)
{
        struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
        return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

static void enable_hdmi(struct display_info_t const *dev)
{
        gpio_direction_output(NOVASOM6_LVDS_EN, 1);
        gpio_direction_output(NOVASOM6_LVDSBACKLIGHT_EN, 1);
        imx_enable_hdmi_phy();
}

static int detect_i2c(struct display_info_t const *dev)
{
/*
        return ((0 == i2c_set_bus_num(dev->bus)) && (0 == i2c_probe(dev->addr)));
*/
	return 1;
}

static iomux_v3_cfg_t const lvds_signals[] = {
        MX6_PAD_EIM_DA7__GPIO3_IO07     | MUX_PAD_CTRL(NO_PAD_CTRL),
        MX6_PAD_EIM_DA8__GPIO3_IO08     | MUX_PAD_CTRL(NO_PAD_CTRL),
};


static void enable_lvds(struct display_info_t const *dev)
{
struct iomuxc *iomux = (struct iomuxc *) IOMUXC_BASE_ADDR;
	printf("Enable LVDS and Backlight power\n");
        u32 reg = readl(&iomux->gpr[2]);
        //reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
        reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT;
        writel(reg, &iomux->gpr[2]);
        imx_iomux_v3_setup_multiple_pads( lvds_signals, ARRAY_SIZE(lvds_signals));
        gpio_direction_output(NOVASOM6_LVDSBACKLIGHT_EN, 0);
	udelay(1000 * 20); // FILIPPO
        gpio_direction_output(NOVASOM6_LVDS_EN, 1);
	udelay(1000 * 200); // FILIPPO
        gpio_direction_output(NOVASOM6_LVDSBACKLIGHT_EN, 0);
}

static void enable_lvds_800(struct display_info_t const *dev)
{
struct iomuxc *iomux = (struct iomuxc *) IOMUXC_BASE_ADDR;
	printf("Enable LCD and Backlight power\n");
        u32 reg = readl(&iomux->gpr[2]);
        //reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
        reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT;
        writel(reg, &iomux->gpr[2]);
        imx_iomux_v3_setup_multiple_pads( lvds_signals, ARRAY_SIZE(lvds_signals));
	//udelay(1000 * 20); // FILIPPO
        gpio_direction_output(NOVASOM6_LVDS_EN, 1);
	//udelay(1000 * 20); // FILIPPO
        gpio_direction_output(NOVASOM6_LVDSBACKLIGHT_EN, 1);
}

/*
void blank_lvds(void)
{
        imx_iomux_v3_setup_multiple_pads( lvds_signals, ARRAY_SIZE(lvds_signals));
        gpio_direction_output(NOVASOM6_LVDS_EN, 1);
        gpio_direction_output(NOVASOM6_LVDSBACKLIGHT_EN, 0);

}
*/

static iomux_v3_cfg_t const lcd_pads[] = {
        MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
        MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,
        MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,
        MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,
        MX6_PAD_DI0_PIN4__GPIO4_IO20,
        MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00,
        MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01,
        MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02,
        MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03,
        MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04,
        MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05,
        MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06,
        MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07,
        MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08,
        MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09,
        MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10,
        MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11,
        MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12,
        MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13,
        MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14,
        MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15,
        MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16,
        MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17,
        MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18,
        MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19,
        MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20,
        MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21,
        MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22,
        MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23,
};

#define NOVASOM6_DD_LCD_ENABLE          IMX_GPIO_NR(5, 0)       /* EIM_WAIT__GPIO5_IO00 */
#define NOVASOM6_DD_BKL_ENABLE          IMX_GPIO_NR(3, 5)       /* EIM_DA5__GPIO3_IO05 */

static iomux_v3_cfg_t const lcd_enable[] = {
	MX6_PAD_EIM_WAIT__GPIO5_IO00   	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_DA5__GPIO3_IO05   	| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void enable_rgb(struct display_info_t const *dev)
{
int		i;
        imx_iomux_v3_setup_multiple_pads( lcd_enable, ARRAY_SIZE(lcd_enable));
        gpio_direction_output(NOVASOM6_DD_LCD_ENABLE, 0);
        gpio_direction_output(NOVASOM6_DD_BKL_ENABLE, 0);

	for(i=0;i<5;i++)
	        udelay(1000 * 20);
        gpio_set_value(NOVASOM6_DD_LCD_ENABLE, 1);
	for(i=0;i<5;i++)
	        udelay(1000 * 20);
        imx_iomux_v3_setup_multiple_pads( lcd_pads, ARRAY_SIZE(lcd_pads));
	for(i=0;i<5;i++)
	        udelay(1000 * 20);
        gpio_set_value(NOVASOM6_DD_BKL_ENABLE, 1);
}

struct display_info_t const displays[] = 
{
	{
		.bus    = -1,
		.addr   = 0,
		.pixfmt = IPU_PIX_FMT_RGB24,
		.detect = detect_hdmi,
		.enable = enable_hdmi,
		.mode   = 
		{
			.name           = "HDMI",
			.refresh        = 60,
			.xres           = 1024,
			.yres           = 768,
			.pixclock       = 15385,
			.left_margin    = 220,
			.right_margin   = 40,
			.upper_margin   = 21,
			.lower_margin   = 7,
			.hsync_len      = 60,
			.vsync_len      = 10,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
		}
	}, 
	{
		.bus    = 0,
		.addr   = 0x4,
		.pixfmt = IPU_PIX_FMT_RGB666,
		.detect = detect_i2c,
		.enable = enable_lvds,
		.mode   = 
		{
			.name           = "LVDS-640x480",
			.refresh        = 60,
			.xres           = 640,
			.yres           = 480,
			.pixclock       = 39721,
			.left_margin    = 48,
			.right_margin   = 16,
			.upper_margin   = 33,
			.lower_margin   = 10,
			.hsync_len      = 5,
			.vsync_len      = 3,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
		} 
	}, 
	{
		.bus    = 0,
		.addr   = 0x4,
		.pixfmt = IPU_PIX_FMT_RGB666,
		.detect = detect_i2c,
		.enable = enable_lvds_800,
		.mode   = 
		{
			.name           = "LVDS-800x480",
			.refresh        = 60,
			.xres           = 800,
			.yres           = 480,
			.pixclock       = 29850,
			.left_margin    = 124,
			.right_margin   = 124,
			.upper_margin   = 11,
			.lower_margin   = 11,
			.hsync_len      = 8,
			.vsync_len      = 3,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
		} 
	}, 
        {
                .bus    = 0,
                .addr   = 0x4,
                .pixfmt = IPU_PIX_FMT_RGB666,
                .detect = detect_i2c,
                .enable = enable_lvds,
                .mode   =
                {
                        .name           = "LVDS-1024x600",
                        .refresh        = 60,
                        .xres           = 1024,
                        .yres           = 600,
                        .pixclock       = 19417,
                        .left_margin    = 90,
                        .right_margin   = 120,
                        .upper_margin   = 21,
                        .lower_margin   = 7,
                        .hsync_len      = 100,
                        .vsync_len      = 33,
                        .sync           = FB_SYNC_EXT,
                        .vmode          = FB_VMODE_NONINTERLACED
                }
        },

	{
		.bus    = 0,
		.addr   = 0x4,
		.pixfmt = IPU_PIX_FMT_RGB666,
		.detect = detect_i2c,
		.enable = enable_lvds,
		.mode   = 
		{
			.name           = "LVDS-1024x768",
			.refresh        = 60,
			.xres           = 1024,
			.yres           = 768,
			.pixclock       = 19417,
			.left_margin    = 220,
			.right_margin   = 40,
			.upper_margin   = 21,
			.lower_margin   = 7,
			.hsync_len      = 60,
			.vsync_len      = 10,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
		} 
	}, 
        {
                .bus    = 0,
                .addr   = 0x4,
                .pixfmt = IPU_PIX_FMT_RGB666,
                .detect = detect_i2c,
                .enable = enable_lvds,
                .mode   =
                {
                        .name           = "LVDS-1366x768",
                        .refresh        = 50,
                        .xres           = 1366,
                        .yres           = 768,
                        .pixclock       = 19417,
                        .left_margin    = 220,
                        .right_margin   = 40,
                        .upper_margin   = 21,
                        .lower_margin   = 7,
                        .hsync_len      = 60,
                        .vsync_len      = 10,
                        .sync           = FB_SYNC_EXT,
                        .vmode          = FB_VMODE_NONINTERLACED
                }
        },
	{
		.bus    = 2,
		.addr   = 0x48,
		.pixfmt = IPU_PIX_FMT_RGB24,
		.detect = detect_i2c,
		.enable = enable_rgb,
		.mode   = 
		{
			.name           = "LCD-480x272",
			.refresh        = 60,
			.xres           = 480,
			.yres           = 272,
			.pixclock       = 111111,
			.left_margin    = 41,
			.right_margin   = 10,
			.upper_margin   = 4,
			.lower_margin   = 8,
			.hsync_len      = 2,
			.vsync_len      = 4,
			.sync           = 0,
			.vmode          = FB_VMODE_NONINTERLACED
		}
	}, 
};
size_t display_count = ARRAY_SIZE(displays);


int setup_novasom_panel(int index)
{
char const 	*panel = getenv("Display");
struct display_info_t const *dev = displays+index;
int	ret;

	panel = dev->mode.name;
	ret = ipuv3_fb_init(&displays[index].mode, 0, displays[index].pixfmt);
        gpio_direction_output(NOVASOM6_LVDS_EN, 1);
        gpio_direction_output(NOVASOM6_LVDSBACKLIGHT_EN, 0);
	if (!ret)
	{
		displays[index].enable(displays+index);
		printf("Active Display %s (%ux%u@%u) : ",panel, displays[index].mode.xres, displays[index].mode.yres, displays[index].mode.refresh);
		return 1;
	}
	return 0;
}

int	panel_index;
static void setup_display(void)
{
struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
int reg;

	enable_ipu_clock();
	// return;
	imx_setup_hdmi();

        /* Turn on LDB0,IPU,IPU DI0 clocks */
        reg = __raw_readl(&mxc_ccm->CCGR3);
        reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
        writel(reg, &mxc_ccm->CCGR3);

        /* set LDB0, LDB1 clk select to 011/011 */
        reg = readl(&mxc_ccm->cs2cdr);
        reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
        //reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET) |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
        reg |= (4<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET) |(4<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
        writel(reg, &mxc_ccm->cs2cdr);

        reg = readl(&mxc_ccm->chsccdr);
        reg |= (CHSCCDR_CLK_SEL_LDB_DI0 <<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
        writel(reg, &mxc_ccm->chsccdr);

        reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
             |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
             |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
             |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
             |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
             |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
             |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
             |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
             |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
        writel(reg, &iomux->gpr[2]);

        reg = readl(&iomux->gpr[3]);
        reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK |IOMUXC_GPR3_HDMI_MUX_CTL_MASK)) | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
        writel(reg, &iomux->gpr[3]);
/* For GDS Only */
#if defined(CONFIG_GDS_PANELS)
	backlight_off();
#endif

}
#endif /* CONFIG_VIDEO_IPUV3 */

#ifdef CONFIG_SPLASH_SCREEN
extern 	int fat_register_device(block_dev_desc_t *, int);
extern	long file_fat_read(const char *filename, void *buffer, unsigned long maxsize);
int splash_screen_prepare(void)
{
        ulong addr;
        char *s;
        __maybe_unused  int n, err;
        __maybe_unused  ulong offset = 0, size = 0, count = 0;
        __maybe_unused  struct mmc *mmc = NULL;
        __maybe_unused  struct block_dev_desc_t *sata = NULL;
        int partition = 1, mmc_device = 0;

        s = getenv("splashimage");
        if (s != NULL) 
	{
                addr = simple_strtoul(s, NULL, 16);

                if((s = getenv("splashimage_mmc_dev")) != NULL)
                        mmc_device = (int )((*s) & 0x03);
                mmc = find_mmc_device(mmc_device);

                if (!mmc)
		{
                        printf ("Splash : Error Reading MMC.\n");
        		return -1;
                }

                mmc_init(mmc);

                if((s = getenv("splashimage_mmc_part")) != NULL)
                        partition = (int )((*s) & 0x03);

                if((s = getenv("splashimage_file_name")) == NULL) 
			setenv("splashimage_file_name","splash.bmp.gz");
		err = fat_register_device(&mmc->block_dev, partition);
		if (!err) 
		{
			//printf("Splash : %s loading from MMC FAT partition %d on %s\n", s,partition,displays[panel_index].mode.name);
			if (file_fat_read(getenv("splashimage_file_name"), (ulong *)addr, 0) > 0) 
			{
				//printf("Splash : on display %s\n",displays[panel_index].mode.name);
                		if((s = getenv("splashpos")) == NULL) 
					setenv("splashpos" , "0,0");
				return 0;
			}
		}
        }
        return -1;
}
#endif

int board_eth_init(bd_t *bis)
{
int	ret;
	setup_iomux_enet();
	ret = cpu_eth_init(bis);
	return ret;
}

#define NOVASOM6_CSI_RST_B         	IMX_GPIO_NR(1, 4)  /* MX6_PAD_EIM_DA11__GPIO_3_11 */
#define NOVASOM6_CSIPWN		      	IMX_GPIO_NR(4, 5)  /* MX6_PAD_EIM_DA10__GPIO_3_10 */
static iomux_v3_cfg_t const camera_pads[] = {
	MX6_PAD_GPIO_4__GPIO1_IO04    | MUX_PAD_CTRL(NO_PAD_CTRL), /* NOVASOM6_CSI_RST_B */
	MX6_PAD_GPIO_19__GPIO4_IO05    | MUX_PAD_CTRL(NO_PAD_CTRL), /* NOVASOM6_CSIPWN */
};

void camera_init(void)
{
        imx_iomux_v3_setup_multiple_pads(camera_pads, ARRAY_SIZE(camera_pads));
        gpio_direction_output(NOVASOM6_CSI_RST_B, 0);
        gpio_direction_output(NOVASOM6_CSIPWN, 1);
}

#define NOVASOM6_EMMC_RST         	IMX_GPIO_NR(6, 8)  /* MX6_PAD_NANDF_ALE__GPIO6_IO08 */
static iomux_v3_cfg_t const emmc_pads[] = {
	MX6_PAD_NANDF_ALE__GPIO6_IO08    | MUX_PAD_CTRL(NO_PAD_CTRL), /* eMMC reset */
};

void emmc_init(void)
{
        imx_iomux_v3_setup_multiple_pads(emmc_pads, ARRAY_SIZE(emmc_pads));
        gpio_direction_output(NOVASOM6_EMMC_RST,1);
}

#define NOVASOM6_USB_OTG_PWR_EN         IMX_GPIO_NR(2, 5)  /* MX6Q_PAD_NANDF_D5__GPIO_2_5 */
#define NOVASOM6_OTG_IN                 IMX_GPIO_NR(1, 1)  /* MX6Q_PAD_GPIO_1__GPIO_1_1 */
#define NOVASOM6_USB_H1_PWREN           IMX_GPIO_NR(6, 31) /* MX6Q_PAD_EIM_BCLK__GPIO_6_31 */
#define NOVASOM6_USB_HUBRESET           IMX_GPIO_NR(2, 19)  /* MX6Q_PAD_EIM_A19__GPIO_2_19 */

static iomux_v3_cfg_t const usb_pads[] = {
	MX6_PAD_EIM_A19__GPIO2_IO19    | MUX_PAD_CTRL(NO_PAD_CTRL), /* NOVASOM6_USB_HUBRESET */
};

void usb_init(void)
{
        imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));
        gpio_direction_output(NOVASOM6_USB_HUBRESET, 1);
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	board_led_init();
	//camera_init();
	emmc_init();
	usb_init();
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	  MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	  MAKE_CFGVAL(0x40, 0x20, 0x00, 0x00)},
	{NULL,	 0},
};
#endif


unsigned char mac[6];
int board_late_init(void)
{
//int	i;
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	backlight_off();
	return 0;
}


struct i2c_pads_info i2c0_pad_info = {
        .scl = {
                .i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp = IMX_GPIO_NR(5, 27)
        },
        .sda = {
                .i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | MUX_PAD_CTRL(I2C_PAD_CTRL),
                .gp = IMX_GPIO_NR(5, 26)
        }
};

int board_init(void)
{
//char const      *panel = getenv("panel");
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
        setup_i2c(0, 20000, 0x7f, &i2c0_pad_info);
#if defined(CONFIG_VIDEO_IPUV3)
//	if ( strcmp(panel,"HDMI") == 0 )
		setup_display();
#endif
	return 0;
}

int checkboard(void)
{
#ifdef CONFIG_MX6Q
        puts("Board: NOVAsom8 QUAD\n");
#endif
#ifdef CONFIG_MX6DL
        puts("Board: NOVAsom7\n");
#endif
#ifdef CONFIG_MX6S
        puts("Board: NOVAsom6\n");
#endif

	return 0;
}
