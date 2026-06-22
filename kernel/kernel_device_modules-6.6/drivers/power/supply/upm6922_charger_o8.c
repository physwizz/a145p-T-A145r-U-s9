/*
 * UPM6922 battery charging driver
 *
 * Copyright (C) 2023 Unisemipower
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define pr_fmt(fmt) "[upm6922-charger]:%s:" fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>
/*A06_V code for SR-AL7160V-01-92 by xiongxiaoliang at 20240904 start*/
#include <linux/regulator/machine.h>
/*A06_V code for SR-AL7160V-01-92 by xiongxiaoliang at 20240904 end*/
#include <linux/extcon-provider.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

#include "charger_class.h"
#include "mtk_charger.h"

#include "upm6922_charger_o8.h"

#include <linux/phy/phy.h>
 /*A06_V code for SR-AL7160V-01-206  by yexuedong at 20240902 start*/
#include <linux/power/gxy_psy_sysfs.h>
/*A06_V code for SR-AL7160V-01-201 by xiongxiaoliang at 20240911 start*/
#if IS_ENABLED(CONFIG_CHARGER_CP_PPS)
#include "charge_pump/pd_policy_manager.h"
extern struct gxy_cp_info g_cp_charging_lmt;
#endif
/*A06_V code for SR-AL7160V-01-201 by xiongxiaoliang at 20240911 end*/
/**********************************************************
 *
 *   [extern for other module]
 *
 *********************************************************/
#if IS_ENABLED(CONFIG_HQ_PROJECT_O8)
extern void gxy_bat_set_chginfo(enum gxy_bat_chg_info cinfo_data);
#endif
 /*A06_V code for SR-AL7160V-01-206  by yexuedong at 20240902 end*/
#define PHY_MODE_BC11_SET 1
#define PHY_MODE_BC11_CLR 2

struct upm6922_platform_data {
    int ilim;
    int ichg;
    int iprechg;
    int iterm;
    int cv;
    int vlim;
    int boostv;
    int boosti;
    int vac_ovp;
    int statctrl;
};

struct upm6922 {
    struct device *dev;
    struct i2c_client *client;

    int part_no;
    int revision;

    const char *chg_dev_name;

    int irq;
    u32 intr_gpio;

    struct mutex i2c_rw_lock;
    struct mutex lock;

    bool power_good;
    bool vbus_attach;
    u8 chrg_stat;
    u8 vbus_stat;
    u8 chrg_type;

    bool force_dpdm;
    bool chg_config;
    int typec_attached;
    struct upm6922_platform_data *upd;
    bool en_auto_bc12;
    bool en_adc_continuous;

    struct charger_device *chg_dev;

    struct delayed_work psy_dwork;
    struct delayed_work force_detect_dwork;
    struct delayed_work charge_detect_delayed_work;
    struct delayed_work monitor_dwork;
    struct wakeup_source *monitor_ws;

    /*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 start*/
    struct delayed_work hvdcp_done_work;
    bool hvdcp_done;
    /*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 end*/
    int psy_usb_type;

    struct power_supply_desc this_psy_desc;
    struct power_supply *this_psy;

    struct regulator_dev *otg_rdev;
    struct power_supply *inform_psy;
};

enum attach_type {
	ATTACH_TYPE_NONE = 0,
	ATTACH_TYPE_PWR_RDY,
	ATTACH_TYPE_TYPEC,
	ATTACH_TYPE_PD,
	ATTACH_TYPE_PD_SDP,
	ATTACH_TYPE_PD_DCP,
	ATTACH_TYPE_PD_NONSTD,
};

static struct charger_device *s_chg_dev_otg = NULL;

static int __upm6922_read_reg(struct upm6922 *upm, u8 reg, u8 *data)
{
    s32 ret;

    ret = i2c_smbus_read_byte_data(upm->client, reg);
    if (ret < 0) {
        pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
        return ret;
    }

    *data = (u8) ret;

    return 0;
}

static int __upm6922_write_reg(struct upm6922 *upm, int reg, u8 val)
{
    s32 ret;

    ret = i2c_smbus_write_byte_data(upm->client, reg, val);
    if (ret < 0) {
        pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
               val, reg, ret);
        return ret;
    }
    return 0;
}

static int upm6922_read_byte(struct upm6922 *upm, u8 reg, u8 *data)
{
    int ret;

    mutex_lock(&upm->i2c_rw_lock);
    ret = __upm6922_read_reg(upm, reg, data);
    mutex_unlock(&upm->i2c_rw_lock);

    return ret;
}

static int upm6922_write_byte(struct upm6922 *upm, u8 reg, u8 data)
{
    int ret;

    mutex_lock(&upm->i2c_rw_lock);
    ret = __upm6922_write_reg(upm, reg, data);
    mutex_unlock(&upm->i2c_rw_lock);

    if (ret)
        pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

    return ret;
}

static int upm6922_update_bits(struct upm6922 *upm, u8 reg, u8 mask, u8 data)
{
    int ret;
    u8 tmp;

    mutex_lock(&upm->i2c_rw_lock);
    ret = __upm6922_read_reg(upm, reg, &tmp);
    if (ret) {
        pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
        goto out;
    }

    tmp &= ~mask;
    tmp |= data & mask;

    ret = __upm6922_write_reg(upm, reg, tmp);
    if (ret) {
        pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
    }

out:
    mutex_unlock(&upm->i2c_rw_lock);
    return ret;
}

static int __maybe_unused upm6922_enter_hiz_mode(struct upm6922 *upm)
{
    u8 val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;

    pr_info("hiz enter\n");
    return upm6922_update_bits(upm, UPM6922_REG_00, REG00_ENHIZ_MASK, val);
}

static int __maybe_unused upm6922_exit_hiz_mode(struct upm6922 *upm)
{
    u8 val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;

    pr_info("hiz exit\n");
    return upm6922_update_bits(upm, UPM6922_REG_00, REG00_ENHIZ_MASK, val);
}

static int __maybe_unused upm6922_get_hiz_mode(struct upm6922 *upm, bool *en)
{
    int ret = 0;
    u8 val = 0;

    ret = upm6922_read_byte(upm, UPM6922_REG_00, &val);
    if (ret < 0) {
        pr_info("read UPM6922_REG_00 fail\n");
        return ret;
    }

    *en = (val & REG00_ENHIZ_MASK) ? true : false;

    return ret;
}

static int upm6922_set_stat_ctrl(struct upm6922 *upm, int ctrl)
{
    u8 val;

    pr_info("ctrl:%d\n", ctrl);
    val = ctrl;

    return upm6922_update_bits(upm, UPM6922_REG_00, REG00_STAT_CTRL_MASK,
                   val << REG00_STAT_CTRL_SHIFT);
}

static int upm6922_set_input_current_limit(struct upm6922 *upm, int ma)
{
    u8 val;

    pr_info("ma:%d\n", ma);

    if (ma < REG00_IINLIM_MIN) {
        ma = REG00_IINLIM_MIN;
    } else if (ma > REG00_IINLIM_MAX) {
        ma = REG00_IINLIM_MAX;
    }

    val = (ma - REG00_IINLIM_BASE) / REG00_IINLIM_LSB;
    return upm6922_update_bits(upm, UPM6922_REG_00, REG00_IINLIM_MASK,
                   val << REG00_IINLIM_SHIFT);
}

static int upm6922_chg_attach_pre_process(struct charger_device *chg_dev,
                    int attach)
{
    struct upm6922 *upm = charger_get_data(chg_dev);
    upm->typec_attached = attach;
    mutex_lock(&upm->lock);

    switch(attach) {
        case ATTACH_TYPE_TYPEC:
            schedule_delayed_work(&upm->force_detect_dwork, msecs_to_jiffies(150));
            break;
        case ATTACH_TYPE_PD_SDP:
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_USB;
            upm->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
            break;
        case ATTACH_TYPE_PD_DCP:
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
            upm->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
            break;
        case ATTACH_TYPE_PD_NONSTD:
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_USB;
            upm->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
            break;
        case ATTACH_TYPE_NONE:
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
            upm->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
            break;
        default:
            pr_err("using tradtional bc12 flow!\n");
            break;
    }

    /*A14V code for P250108-07220 by chenyulin at 20250117 start*/
    upm->chrg_type = upm->this_psy_desc.type;
    /*A14V code for P250108-07220 by chenyulin at 20250117 end*/
    power_supply_changed(upm->this_psy);

    pr_err("type(%d %d), attach:%d\n",
        upm->this_psy_desc.type, upm->psy_usb_type, attach);
    mutex_unlock(&upm->lock);

    return 0;
}

static int __maybe_unused upm6922_get_input_current_limit(struct upm6922 *upm, int *ma)
{
    u8 val = 0;
    int ret;
    int iindpm;

    ret = upm6922_read_byte(upm, UPM6922_REG_00, &val);
    if (ret < 0){
        pr_err("read UPM6922_REG_00 fail, ret:%d\n", ret);
        return ret;
    }

    iindpm = (val & REG00_IINLIM_MASK) >> REG00_IINLIM_SHIFT;
    iindpm = iindpm * REG00_IINLIM_LSB + REG00_IINLIM_BASE;
    *ma = iindpm;

    return ret;
}

static int __maybe_unused upm6922_reset_watchdog_timer(struct upm6922 *upm)
{
    u8 val = REG01_WDT_RESET << REG01_WDT_RESET_SHIFT;

    pr_info("reset\n");
    return upm6922_update_bits(upm, UPM6922_REG_01, REG01_WDT_RESET_MASK,
                   val);
}

static int __maybe_unused upm6922_enable_otg(struct upm6922 *upm)
{
    u8 val = REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT;

    pr_info("enable otg\n");
    return upm6922_update_bits(upm, UPM6922_REG_01, REG01_OTG_CONFIG_MASK,
                   val);
}

static int __maybe_unused upm6922_disable_otg(struct upm6922 *upm)
{
    u8 val = REG01_OTG_DISABLE << REG01_OTG_CONFIG_SHIFT;

    pr_info("disable otg\n");
    return upm6922_update_bits(upm, UPM6922_REG_01, REG01_OTG_CONFIG_MASK,
                   val);
}

/*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 start*/
static int upm6922_enable_hiz_mode(struct upm6922 *upm)
{
    u8 val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;

    pr_info("%s enter\n", __func__);

    return upm6922_update_bits(upm, UPM6922_REG_00, REG00_ENHIZ_MASK,
                   val);
}

static int upm6922_disable_hiz_mode(struct upm6922 *upm)
{
    u8 val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;

    pr_info("%s enter\n", __func__);
    return upm6922_update_bits(upm, UPM6922_REG_00, REG00_ENHIZ_MASK,
                   val);
}

/*static int upm6922_get_hiz_mode(struct upm6922 *upm, bool *enable)
{
    int ret = 0;
    u8 val = 0;

    ret = upm6922_read_reg(upm, UPM6922_REG_00, &val);
    if (ret < 0) {
        pr_info("[%s] read UPM6922_REG_00 fail\n", __func__);
        return ret;
    }

    val = val & REG00_ENHIZ_MASK;
    if (val) {
        *enable  = true;
    } else {
        *enable  = false;
    }
    dev_info(upm->dev, "%s enter value = %d\n", __func__, *enable);

    return ret;
}*/
/*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 end*/

static int __maybe_unused upm6922_enable_charger(struct upm6922 *upm)
{
    int ret;
    u8 val = REG01_CHG_ENABLE << REG01_CHG_CONFIG_SHIFT;

    pr_info("enable charge\n");
    ret = upm6922_update_bits(upm, UPM6922_REG_01,
                    REG01_CHG_CONFIG_MASK, val);

    return ret;
}

static int __maybe_unused upm6922_disable_charger(struct upm6922 *upm)
{
    int ret;
    u8 val = REG01_CHG_DISABLE << REG01_CHG_CONFIG_SHIFT;

    pr_info("disable charge\n");
    ret = upm6922_update_bits(upm, UPM6922_REG_01,
                    REG01_CHG_CONFIG_MASK, val);

    return ret;
}

static int upm6922_set_boost_current(struct upm6922 *upm, int ma)
{
    u8 val1 = 0;
    u8 val2 = 0;
    int ret = 0;

    pr_info("ma:%d\n", ma);

    if (ma >= 2000) {
        val2 = REG15_BOOST_LIM1_2A;
    } else if (ma >= 1200) {
        val1 = REG02_BOOST_LIM0_1P2A;
        val2 = REG15_BOOST_LIM1_DIS;
    } else {
        val1 = REG02_BOOST_LIM0_0P5A;
        val2 = REG15_BOOST_LIM1_DIS;
    }

    ret = upm6922_update_bits(upm, UPM6922_REG_02, REG02_BOOST_LIM0_MASK,
                   val1 << REG02_BOOST_LIM0_SHIFT);
    if (ret < 0) {
        pr_err("write UPM6922_REG_02 fail, ret:%d\n", ret);
        return ret;
    }

    ret = upm6922_update_bits(upm, UPM6922_REG_15, REG15_BOOST_LIM1_MASK,
                   val2 << REG15_BOOST_LIM1_SHIFT);
    if (ret < 0) {
        pr_err("write UPM6922_REG_02 fail, ret:%d\n", ret);
    }

    return ret;
}

static int upm6922_set_chargecurrent(struct upm6922 *upm, int ma)
{
    u8 ichg;

    pr_info("ma:%d\n", ma);

    if (ma < REG02_ICHG_MIN) {
        ma = REG02_ICHG_MIN;
    } else if (ma > REG02_ICHG_MAX) {
        ma = REG02_ICHG_MAX;
    }

    ichg = (ma - REG02_ICHG_BASE) / REG02_ICHG_LSB;
    return upm6922_update_bits(upm, UPM6922_REG_02, REG02_ICHG_MASK,
                   ichg << REG02_ICHG_SHIFT);
}

static int __maybe_unused upm6922_get_chargecurrent(struct upm6922 *upm, int *ma)
{
    u8 val = 0;
    int ret = 0;

    ret = upm6922_read_byte(upm, UPM6922_REG_02, &val);
    if (ret < 0){
        pr_err("read UPM6922_REG_02 fail, ret:%d\n", ret);
        return ret;
    }

    *ma = ((val & REG02_ICHG_MASK) >> REG02_ICHG_SHIFT) * REG02_ICHG_LSB;

    return ret;
}

static int upm6922_set_prechg_current(struct upm6922 *upm, int ma)
{
    u8 iprechg;

    pr_info("ma:%d\n", ma);

    if (ma < REG03_IPRECHG_MIN) {
        ma = REG03_IPRECHG_MIN;
    } else if (ma < REG03_IPRECHG_MAX) {
        ma = REG03_IPRECHG_MAX;
    }

    iprechg = (ma - REG03_IPRECHG_BASE) / REG03_IPRECHG_LSB;

    return upm6922_update_bits(upm, UPM6922_REG_03, REG03_IPRECHG_MASK,
                   iprechg << REG03_IPRECHG_SHIFT);
}

static int upm6922_set_term_current(struct upm6922 *upm, int ma)
{
    u8 iterm;

    pr_info("ma:%d\n", ma);

    if (ma < REG03_ITERM_MIN) {
        ma = REG03_ITERM_MIN;
    } else if (ma > REG03_ITERM_MAX) {
        ma = REG03_ITERM_MAX;
    }

    iterm = (ma - REG03_ITERM_BASE) / REG03_ITERM_LSB;

    return upm6922_update_bits(upm, UPM6922_REG_03, REG03_ITERM_MASK,
                   iterm << REG03_ITERM_SHIFT);
}

static int upm6922_set_chargevolt(struct upm6922 *upm, int mv)
{
    u8 val1, val2;
    int ret = 0;

    pr_info("mv:%d\n", mv);

    if (mv < REG04_VREG_MIN) {
        mv = REG04_VREG_MIN;
    } else if (mv > REG04_VREG_MAX) {
        mv = REG04_VREG_MAX;
    }

    val1 = (mv - REG04_VREG_BASE) / REG04_VREG_LSB;
    ret = upm6922_update_bits(upm, UPM6922_REG_04, REG04_VREG_MASK,
                   val1 << REG04_VREG_SHIFT);
    if (ret < 0) {
        pr_err("write UPM6922_REG_04 fail, ret:%d\n", ret);
        return ret;
    }

    val2 = ((mv - REG04_VREG_BASE) % REG04_VREG_LSB) / REG0D_VREG_FT_LSB;
    ret = upm6922_update_bits(upm, UPM6922_REG_0D, REG0D_VREG_FT_MASK,
                   val2 << REG0D_VREG_FT_SHIFT);
    if (ret < 0) {
        pr_err("write UPM6922_REG_0D fail, ret:%d\n", ret);
    }

    return ret;
}

static int __maybe_unused upm6922_get_chargevolt(struct upm6922 *upm, int *mv)
{
    u8 val1, val2;
    int cv = 0;
    int ret = 0;

    ret = upm6922_read_byte(upm, UPM6922_REG_04, &val1);
    if (ret < 0){
        pr_err("read UPM6922_REG_04 fail, ret:%d\n", ret);
        return ret;
    }

    ret = upm6922_read_byte(upm, UPM6922_REG_0D, &val2);
    if (ret < 0){
        pr_err("read UPM6922_REG_0D fail, ret:%d\n", ret);
        return ret;
    }

    cv = ((UPM6922_REG_04 & REG04_VREG_MASK) >> REG04_VREG_SHIFT) *
                REG04_VREG_LSB + REG04_VREG_BASE;
    cv += ((val2 & REG0D_VREG_FT_MASK) >> REG0D_VREG_FT_SHIFT) * REG0D_VREG_FT_LSB;

    *mv = cv;

    return ret;
}

/*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 start*/
static int upm6922_enable_port_charging(struct charger_device *chg_dev, bool enable)
{
    struct upm6922 *upm = charger_get_data(chg_dev);
    int ret = 0;

    dev_info(upm->dev, "%s value = %d\n", __func__, enable);

    if (enable) {
        ret = upm6922_disable_hiz_mode(upm);
        /*A06_V code for P250428-09126 by shanxinkai at 20250513 start*/
        if (upm->psy_usb_type == POWER_SUPPLY_USB_TYPE_DCP) {
            upm->hvdcp_done = false;
            schedule_delayed_work(&upm->force_detect_dwork, msecs_to_jiffies(200));
        }
        /*A06_V code for P250428-09126 by shanxinkai at 20250513 end*/
    } else {
        ret = upm6922_enable_hiz_mode(upm);
    }

    return ret;
}

static int upm6922_is_port_charging_enabled(struct charger_device *chg_dev, bool *enable)
{
    struct upm6922 *upm = charger_get_data(chg_dev);
    int ret = 0;
    bool hiz_enable = false;

    dev_info(upm->dev, "%s enter\n", __func__);

    ret = upm6922_get_hiz_mode(upm, &hiz_enable);
    *enable = !hiz_enable;

    return ret;
}
/*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 end*/

static int __maybe_unused upm6922_set_recharge_volt(struct upm6922 *upm, int mv)
{
    u8 val = 0;

    pr_info("mv:%d\n", mv);

    if (mv < 200) {
        val = REG04_VRECHG_100MV;
    } else {
        val = REG04_VRECHG_200MV;
    }

    return upm6922_update_bits(upm, UPM6922_REG_04, REG04_VRECHG_MASK,
                    val << REG04_VRECHG_SHIFT);
}

static int __maybe_unused upm6922_enable_term(struct upm6922 *upm, bool enable)
{
    u8 val;
    int ret;

    pr_info("enable:%d\n", enable);

    if (enable) {
        val = REG05_TERM_ENABLE << REG05_EN_TERM_SHIFT;
    } else {
        val = REG05_TERM_DISABLE << REG05_EN_TERM_SHIFT;
    }

    ret = upm6922_update_bits(upm, UPM6922_REG_05, REG05_EN_TERM_MASK, val);

    return ret;
}

static int __maybe_unused upm6922_set_watchdog_timer(struct upm6922 *upm, u8 seconds)
{
    u8 val = 0;

    pr_info("seconds:%d\n", seconds);

    if (seconds >= 160) {
        val = REG05_WDT_160S;
    } else if (seconds >= 80) {
        val = REG05_WDT_80S;
    } else if (seconds >= 40) {
        val = REG05_WDT_40S;
    } else {
        val = REG05_WDT_DISABLE;
    }

    return upm6922_update_bits(upm, UPM6922_REG_05, REG05_WDT_MASK,
                    val << REG05_WDT_SHIFT);
}

static int upm6922_disable_watchdog_timer(struct upm6922 *upm)
{
    u8 val = REG05_WDT_DISABLE << REG05_WDT_SHIFT;

    pr_info("disable wd\n");

    return upm6922_update_bits(upm, UPM6922_REG_05, REG05_WDT_MASK, val);
}

static int __maybe_unused upm6922_enable_safety_timer(struct upm6922 *upm)
{
    const u8 val = REG05_CHG_TIMER_ENABLE << REG05_EN_TIMER_SHIFT;

    pr_info("enable timer\n");

    return upm6922_update_bits(upm, UPM6922_REG_05, REG05_EN_TIMER_MASK,
                   val);
}

static int __maybe_unused upm6922_disable_safety_timer(struct upm6922 *upm)
{
    const u8 val = REG05_CHG_TIMER_DISABLE << REG05_EN_TIMER_SHIFT;

    pr_info("disable timer\n");

    return upm6922_update_bits(upm, UPM6922_REG_05, REG05_EN_TIMER_MASK,
                   val);
}

static int __maybe_unused upm6922_set_safety_timer(struct upm6922 *upm, u8 hours)
{
    u8 val;

    pr_info("hours:%d\n", hours);

    if (hours > 10) {
        val = REG05_CHG_TIMER_10HOURS;
    } else {
        val = REG05_CHG_TIMER_5HOURS;
    }

    return upm6922_update_bits(upm, UPM6922_REG_05, REG05_CHG_TIMER_MASK,
                   val << REG05_CHG_TIMER_SHIFT);
}

static int upm6922_set_acovp_threshold(struct upm6922 *upm, int mv)
{
    u8 val;

    pr_info("mv:%d\n", mv);

    if (mv >= 14000) {
        val = REG06_OVP_14V;
    } else if (mv >= 11000) {
        val = REG06_OVP_11V;
    } else if (mv >= 6500) {
        val = REG06_OVP_6P5V;
    } else {
        val = REG06_OVP_5P85V;
    }

    return upm6922_update_bits(upm, UPM6922_REG_06, REG06_OVP_MASK,
                   val << REG06_OVP_SHIFT);
}

static int upm6922_set_boost_voltage(struct upm6922 *upm, int mv)
{
    u8 val;

    pr_info("mv:%d\n", mv);

    if (mv >= 5300) {
        val = REG06_BOOSTV_5P3V;
    } else if (mv >= 5150) {
        val = REG06_BOOSTV_5P15V;
    } else if (mv >= 5000) {
        val = REG06_BOOSTV_5V;
    } else {
        val = REG06_BOOSTV_4P85V;
    }

    return upm6922_update_bits(upm, UPM6922_REG_06, REG06_BOOSTV_MASK,
                   val << REG06_BOOSTV_SHIFT);
}

static int upm6922_set_input_volt_limit(struct upm6922 *upm, int mv)
{
    u8 val1, val2;
    int ret = 0;

    pr_info("mv:%d\n", mv);

    if (mv < 3900) {
        val1 = 0;
        val2 = REG0D_VINDPM_OS_3P9V;
    } else if (mv <= 5400) {
        val1 = (mv - 3900) / REG06_VINDPM_LSB;
        val2 = REG0D_VINDPM_OS_3P9V;
    } else if (mv < 5900) {
        val1 = 0xf;
        val2 = REG0D_VINDPM_OS_3P9V;
    } else if (mv < 7500) {
        val1 = (mv - 5900) / REG06_VINDPM_LSB;
        val2 = REG0D_VINDPM_OS_5P9V;
    } else if (mv <= 9000) {
        val1 = (mv - 7500) / REG06_VINDPM_LSB;
        val2 = REG0D_VINDPM_OS_7P5V;
    } else if (mv < 10500) {
        val1 = 0xf;
        val2 = REG0D_VINDPM_OS_7P5V;
    } else if (mv <= 12000) {
        val1 = (mv - 10500) / REG06_VINDPM_LSB;
        val2 = REG0D_VINDPM_OS_10P5V;
    } else {
        val1 = 0xf;
        val2 = REG0D_VINDPM_OS_10P5V;
    }

    ret =  upm6922_update_bits(upm, UPM6922_REG_06, REG06_VINDPM_MASK,
                   val1 << REG06_VINDPM_SHIFT);
    if (ret < 0) {
        pr_err("write UPM6922_REG_06 fail, ret:%d\n", ret);
        return ret;
    }

    ret =  upm6922_update_bits(upm, UPM6922_REG_0D, REG0D_VINDPM_OS_MASK,
                   val2 << REG0D_VINDPM_OS_SHIFT);
    if (ret < 0) {
        pr_err("write UPM6922_REG_0D fail, ret:%d\n", ret);
    }

    return ret;
}


enum upm6922_usbsw {
	USBSW_CHG,
	USBSW_USB,
};

static int upm6922_chg_set_usbsw(struct  upm6922 *upm,
                enum upm6922_usbsw usbsw)
{
    struct phy *phy = NULL;
    int ret = 0;
    int mode = (usbsw == USBSW_CHG) ? PHY_MODE_BC11_SET :
                           PHY_MODE_BC11_CLR;

    pr_err("usbsw=%d\n", usbsw);
    phy = phy_get(upm->dev, "usb2-phy");
    if (IS_ERR_OR_NULL(phy)) {
        pr_err("failed to get usb2-phy\n");
        return -ENODEV;
    }
    ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, mode);
    if (ret)
        pr_err("failed to set phy ext mode\n");
    phy_put(upm->dev, phy);

    return ret;
}

static int upm6922_force_dpdm(struct upm6922 *upm)
{
    int ret = 0;
    const u8 val = REG07_FORCE_DPDM << REG07_FORCE_DPDM_SHIFT;

    pr_info("iindet en\n");

    ret = upm6922_update_bits(upm, UPM6922_REG_07,
            REG07_FORCE_DPDM_MASK, val);

    return ret;
}

static int __maybe_unused upm6922_enable_batfet(struct upm6922 *upm)
{
    const u8 val = REG07_BATFET_ON << REG07_BATFET_DIS_SHIFT;

    pr_info("batfet turn on\n");

    return upm6922_update_bits(upm, UPM6922_REG_07, REG07_BATFET_DIS_MASK,
                   val);
}

static int __maybe_unused upm6922_disable_batfet(struct upm6922 *upm)
{
    const u8 val = REG07_BATFET_OFF << REG07_BATFET_DIS_SHIFT;

    pr_info("batfet turn off\n");

    return upm6922_update_bits(upm, UPM6922_REG_07, REG07_BATFET_DIS_MASK,
                   val);
}

static int __maybe_unused upm6922_en_batfet_delay(struct upm6922 *upm, bool en)
{
    u8 val;

    pr_info("en:%d\n", en);

    if (en) {
        val = REG07_BATFET_DLY_10S;
    } else {
        val = REG07_BATFET_DLY_0S;
    }

    return upm6922_update_bits(upm, UPM6922_REG_07, REG07_BATFET_DLY_MASK,
                   val << REG07_BATFET_DLY_SHIFT);
}

static int __maybe_unused upm6922_batfet_rst_en(struct upm6922 *upm, bool en)
{
    u8 val = 0;

    pr_info("en:%d\n", en);

    if (en) {
        val = REG07_BATFET_RST_ENABLE << REG07_BATFET_RST_EN_SHIFT;
    } else {
        val = REG07_BATFET_RST_DISABLE << REG07_BATFET_RST_EN_SHIFT;
    }

    return upm6922_update_bits(upm, UPM6922_REG_07, REG07_BATFET_RST_EN_MASK,
                   val);
}

static int __maybe_unused upm6922_vdpm_bat_track(struct upm6922 *upm, int mv)
{
    u8 val = 0;

    pr_info("mv:%d\n", mv);

    if (mv >= 300) {
        val = REG07_VDPM_BAT_TRACK_300MV;
    } else if (mv >= 250) {
        val = REG07_VDPM_BAT_TRACK_250MV;
    } else if (mv >= 200) {
        val = REG07_VDPM_BAT_TRACK_200MV;
    } else {
        val = REG07_VDPM_BAT_TRACK_DISABLE;
    }

    return upm6922_update_bits(upm, UPM6922_REG_07, REG07_VDPM_BAT_TRACK_MASK,
                   val << REG07_VDPM_BAT_TRACK_SHIFT);
}

static void upm6922_force_detection_dwork_handler(struct work_struct *work)
{
    int ret = 0;
    struct delayed_work *force_detect_dwork = NULL;
    struct upm6922 *upm = NULL;

    force_detect_dwork = container_of(work, struct delayed_work, work);
    if (force_detect_dwork == NULL) {
        pr_err("Cann't get force_detect_dwork\n");
        return;
    }

    upm = container_of(force_detect_dwork, struct upm6922, force_detect_dwork);
    if (upm == NULL) {
        pr_err("Cann't get upm6922\n");
        return;
    }

    /*A06_V code for AL7160AV-41 by xiongxiaoliang at 20241105 start*/
    if ((upm->power_good == false) && (upm->typec_attached == ATTACH_TYPE_TYPEC)) {
        pr_info("a good VBUS is not attached, wait~~~\n");
        schedule_delayed_work(&upm->force_detect_dwork, msecs_to_jiffies(10));
        return;
    } else if (upm->power_good == false) {
        pr_info("TypeC has been plug out\n");
        return;
    }
    /*A06_V code for AL7160AV-41 by xiongxiaoliang at 20241105 end*/

    msleep(100);
    upm6922_chg_set_usbsw(upm, USBSW_CHG);
    msleep(100);
    ret = upm6922_force_dpdm(upm);
    upm->force_dpdm = true;
    if (ret) {
        pr_err("force dpdm failed(%d)\n", ret);
        return;
    }
}

static int upm6922_get_charging_stat(struct upm6922 *upm, int *stat)
{
    int ret = 0;
    /*A06_V code for AL7160AV-47 by xiongxiaoliang at 20241112 start*/
    u8 reg_val1 = 0;
    u8 chrg_stat = 0;

    ret = upm6922_read_byte(upm, UPM6922_REG_08, &reg_val1);
    if (ret < 0) {
        pr_err("read UPM6922_REG_08 fail, ret:%d\n", ret);
        return ret;
    }

    chrg_stat = (reg_val1 & REG08_CHRG_STAT_MASK) >> REG08_CHRG_STAT_SHIFT;
    if (!upm->chrg_type || (upm->chrg_type == REG08_VBUS_TYPE_OTG)) {
        *stat = POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
        switch (chrg_stat) {
            case REG08_CHRG_STAT_CHGDONE:
                *stat = POWER_SUPPLY_STATUS_FULL;
                break;
            case REG08_CHRG_STAT_PRECHG:
            case REG08_CHRG_STAT_FASTCHG:
                *stat = POWER_SUPPLY_STATUS_CHARGING;
                break;
            case REG08_CHRG_STAT_IDLE:
                if (upm->chg_config) {
                    *stat = POWER_SUPPLY_STATUS_CHARGING;
                } else {
                    *stat = POWER_SUPPLY_STATUS_NOT_CHARGING;
                }
                break;
            default:
                *stat = POWER_SUPPLY_STATUS_UNKNOWN;
                break;
        }
    }

    pr_err("*stat=%d, chrg_type=%d\n", *stat, upm->chrg_type);
    /*A06_V code for AL7160AV-47 by xiongxiaoliang at 20241112 end*/

    return ret;
}

static int upm6922_enable_vindpm_int(struct upm6922 *upm, bool en)
{
    u8 val = 0;

    pr_info("en:%d\n", en);

    if (en) {
        val = REG0A_VINDPM_INT_ENABLE;
    } else {
        val = REG0A_VINDPM_INT_DISABLE;
    }

    return upm6922_update_bits(upm, UPM6922_REG_0A, REG0A_VINDPM_INT_MASK,
                   val << REG0A_VINDPM_INT_SHIFT);
}

static int upm6922_enable_iindpm_int(struct upm6922 *upm, bool en)
{
    u8 val = 0;

    pr_info("en:%d\n", en);

    if (en) {
        val = REG0A_IINDPM_INT_ENABLE;
    } else {
        val = REG0A_IINDPM_INT_DISABLE;
    }

    return upm6922_update_bits(upm, UPM6922_REG_0A, REG0A_IINDPM_INT_MASK,
                   val << REG0A_IINDPM_INT_SHIFT);
}

static int __maybe_unused upm6922_reset_chip(struct upm6922 *upm)
{
    u8 val = REG0B_REG_RESET << REG0B_REG_RESET_SHIFT;

    pr_info("reset chip\n");

    return upm6922_update_bits(upm, UPM6922_REG_0B, REG0B_REG_RESET_MASK, val);

}

static int upm6922_detect_device(struct upm6922 *upm)
{
    int ret;
    u8 data;

    ret = upm6922_read_byte(upm, UPM6922_REG_0B, &data);
    if (!ret) {
        upm->part_no = (data & REG0B_PN_MASK) >> REG0B_PN_SHIFT;
        upm->revision =
            (data & REG0B_DEV_REV_MASK) >> REG0B_DEV_REV_SHIFT;
    }

    return ret;
}

static int __maybe_unused upm6922_enable_dpdm_mux(struct upm6922 *upm, bool en)
{
    const u8 val = en << REG0C_DPDM_LOCK_SHIFT;

    pr_info("en:%d\n", en);

    return upm6922_update_bits(upm, UPM6922_REG_0C, REG0C_DPDM_LOCK_MASK,
                   val);
}

int upm6922_set_dp(struct upm6922 *upm, int dp_stat)
{
    const u8 val = dp_stat << REG0C_DP_MUX_SHIFT;

    pr_info("dp_stat:%d\n", dp_stat);
    /*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 start*/
    schedule_delayed_work(&upm->hvdcp_done_work, msecs_to_jiffies(1500));
    /*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 end*/
    return upm6922_update_bits(upm, UPM6922_REG_0C, REG0C_DP_MUX_MASK,
                   val);
}

static int __maybe_unused upm6922_set_dm(struct upm6922 *upm, int dm_stat)
{
    const u8 val = dm_stat << REG0C_DM_MUX_SHIFT;

    pr_info("dm_stat:%d\n", dm_stat);

    return upm6922_update_bits(upm, UPM6922_REG_0C, REG0C_DM_MUX_MASK,
                   val);
}

static int __maybe_unused upm6922_auto_bc12_en(struct upm6922 *upm, bool en)
{
    pr_info("en:%d\n", en);

    /* not support */

    return 0;
}

static int __maybe_unused upm6922_fast_chg_volt_th(struct upm6922 *upm, int mv)
{
    u8 val = 0;

    pr_info("mv:%d\n", mv);

    if (mv <= 2700) {
        val = REG0D_PRE2FAST_2P7V;
    } else if (mv <= 2800) {
        val = REG0D_PRE2FAST_2P8V;
    } else if (mv <= 2900) {
        val = REG0D_PRE2FAST_2P9V;
    } else {
        val = REG0D_PRE2FAST_3V;
    }

    return upm6922_update_bits(upm, UPM6922_REG_0D, REG0D_V_RRE2FAST_MASK,
                   val << REG0D_V_RRE2FAST_SHIFT);
}

static int __maybe_unused upm6922_get_bat_voltage(struct upm6922 *upm, int *mv)
{
    u8 reg_val = 0;
    int ret = 0;
    int vbat = 0;

    ret = upm6922_read_byte(upm, UPM6922_REG_0E, &reg_val);
    if (ret) {
        pr_err("read UPM6922_REG_0E fail, ret:%d\n", ret);
        return ret;
    }

    reg_val = ((reg_val & REG0E_VBAT_MASK) >> REG0E_VBAT_SHIFT);
    vbat = reg_val * REG0E_VBAT_LSB + REG0E_VBAT_BASE;

    *mv = vbat;

    return ret;
}

static int __maybe_unused upm6922_get_sys_voltage(struct upm6922 *upm, int *mv)
{
    u8 reg_val = 0;
    int ret = 0;
    int vsys = 0;

    ret = upm6922_read_byte(upm, UPM6922_REG_10, &reg_val);
    if (ret) {
        pr_err("read UPM6922_REG_10 fail, ret:%d\n", ret);
        return ret;
    }

    reg_val = ((reg_val & REG10_VSYS_MASK) >> REG10_VSYS_SHIFT);
    vsys = reg_val * REG10_VSYS_LSB + REG10_VSYS_BASE;

    *mv = vsys;

    return ret;
}

static int __maybe_unused upm6922_get_bus_voltage(struct upm6922 *upm, int *mv)
{
    u8 reg_val = 0;
    int ret = 0;
    int vbus = 0;

    ret = upm6922_read_byte(upm, UPM6922_REG_12, &reg_val);
    if (ret) {
        pr_err("read UPM6922_REG_12 fail, ret:%d\n", ret);
        return ret;
    }

    reg_val = ((reg_val & REG12_VBUS_MASK) >> REG12_VBUS_SHIFT);
    vbus = reg_val * REG12_VBUS_LSB + REG12_VBUS_BASE;

    *mv = vbus;

    return ret;
}

static int __maybe_unused upm6922_adc_start(struct upm6922 *upm, bool en)
{
    u8 val = 0;

    pr_info("en:%d\n", en);

    if (en) {
        val = REG15_CONV_START_ACTIVE;
    } else {
        val = REG15_CONV_START_INACTIVE;
    }

    return upm6922_update_bits(upm, UPM6922_REG_15, REG15_CONV_START_MASK,
                   val << REG15_CONV_START_SHIFT);
}

static int upm6922_set_adc_rate(struct upm6922 *upm, u8 mode)
{
    u8 val = 0;

    pr_info("mode:%d\n", mode);

    val = !!mode;
    return upm6922_update_bits(upm, UPM6922_REG_15, REG15_CONV_RATE_MASK,
                   val << REG15_CONV_RATE_SHIFT);
}

void upm6922_dump_regs(struct upm6922 *upm)
{
    int addr;
    u8 val;
    int ret;

    for (addr = 0x0; addr <= 0x16; addr++) {
        ret = upm6922_read_byte(upm, addr, &val);
        if (ret == 0)
            pr_err("Reg[%.2x] = 0x%.2x\n", addr, val);
    }
}

static struct upm6922_platform_data *upm6922_parse_dt(struct device_node *np,
                              struct upm6922 *upm)
{
    int ret;
    struct upm6922_platform_data *pdata;

    pdata = devm_kzalloc(&upm->client->dev, sizeof(struct upm6922_platform_data),
                 GFP_KERNEL);
    if (!pdata)
        return NULL;

    if (of_property_read_string(np, "charger_name", &upm->chg_dev_name) < 0) {
        upm->chg_dev_name = "primary_chg";
        pr_err("no charger name, default primary chg\n");
    }

    upm->intr_gpio = of_get_named_gpio(np, "up,intr_gpio", 0);
    if (upm->intr_gpio < 0) {
        pr_err("no up,intr_gpio, ret:%d\n", upm->intr_gpio);
    }
    pr_err("intr_gpio = %d\n", upm->intr_gpio);

    ret = of_property_read_u32(np, "up,upm6922,iindpm", &pdata->ilim);
    if (ret) {
        pdata->ilim = 2000;
    }

    ret = of_property_read_u32(np, "up,upm6922,ichg", &pdata->ichg);
    if (ret) {
        pdata->ichg = 2000;
        pr_err("Failed to read node of up,upm6922,ichg, default 2000\n");
    }

    ret = of_property_read_u32(np, "up,upm6922,iprechg", &pdata->iprechg);
    if (ret) {
        pdata->iprechg = 180;
        pr_err("Failed to read node of up,upm6922,iprechg, default 180\n");
    }

    ret = of_property_read_u32(np, "up,upm6922,iterm", &pdata->iterm);
    if (ret) {
        pdata->iterm = 180;
        pr_err("Failed to read node of up,upm6922,iterm, default 180\n");
    }

    ret = of_property_read_u32(np, "up,upm6922,vreg", &pdata->cv);
    if (ret) {
        pdata->cv = 4200;
        pr_err("Failed to read node of up,upm6922,vreg, default 4200\n");
    }

    ret = of_property_read_u32(np, "up,upm6922,vindpm", &pdata->vlim);
    if (ret) {
        pdata->vlim = 4700;
        pr_err("Failed to read node of up,upm6922,vindpm, default 4700\n");
    }

    ret = of_property_read_u32(np, "up,upm6922,boostv", &pdata->boostv);
    if (ret) {
        pdata->boostv = 5000;
        pr_err("Failed to read node of up,upm6922,boostv, default 5000\n");
    }

    ret = of_property_read_u32(np, "up,upm6922,boosti", &pdata->boosti);
    if (ret) {
        pdata->boosti = 1200;
        pr_err("Failed to read node of up,upm6922,boosti, default 1200\n");
    }

    ret = of_property_read_u32(np, "up,upm6922,vac-ovp", &pdata->vac_ovp);
    if (ret) {
        pdata->vac_ovp = 6500;
        pr_err("Failed to read node of up,upm6922,vac-ovp, default 6500\n");
    }

    ret = of_property_read_u32(np, "up,upm6922,stat-pin-ctrl", &pdata->statctrl);
    if (ret) {
        pdata->statctrl = 0;
        pr_err("Failed to read node of up,upm6922,stat-pin-ctrl, default 0\n");
    }

    upm->en_auto_bc12 = !of_property_read_bool(np, "up,upm6922,auto-bc12-disable");

    upm->en_adc_continuous = of_property_read_bool(np, "up,upm6922,continuous-adc-enable");

    return pdata;
}

static int upm6922_get_state(struct upm6922 *upm)
{
    u8 chrg_stat = 0;
    u8 chrg_param_0 = 0;
    int ret = 0;
    /*A06_V code for AL7160AV-41 by xiongxiaoliang at 20241105 start*/
    static bool power_good_pre = false;
    /*A06_V code for AL7160AV-41 by xiongxiaoliang at 20241105 end*/

    ret = upm6922_read_byte(upm, UPM6922_REG_08, &chrg_stat);
    if (ret) {
        pr_err("read UPM6922_REG_08 fail\n");
        return ret;
    }

    upm->chrg_stat = (chrg_stat & REG08_CHRG_STAT_MASK) >> REG08_CHRG_STAT_SHIFT;
    upm->power_good = !!(chrg_stat & REG08_PG_STAT_MASK);
    /*A06_V code for AL7160AV-41|AL7160AV-47 by xiongxiaoliang at 20241112 start*/
    pr_err("chrg_type =%d,chrg_stat =%d online = %d power_good_pre=%d\n",
        upm->chrg_type, upm->chrg_stat, upm->power_good, power_good_pre);
    /*A06_V code for AL7160AV-41|AL7160AV-47 by xiongxiaoliang at 20241112 end*/

    /* get vbus state*/
    ret = upm6922_read_byte(upm, UPM6922_REG_0A, &chrg_param_0);
    if (ret) {
        pr_err("read UPM6922_REG_0A fail\n");
        return ret;
    }

    upm->vbus_attach = !!(chrg_param_0 & REG0A_VBUS_GD_MASK);
    pr_err("vbus_attach = %d\n", upm->vbus_attach);
    /*A06_V code for AL7160AV-41 by xiongxiaoliang at 20241105 start*/
    if ((upm->power_good != power_good_pre) && (upm->power_good == true)) {
        power_supply_changed(upm->this_psy);
    }

    power_good_pre = upm->power_good;
    /*A06_V code for AL7160AV-41 by xiongxiaoliang at 20241105 end*/
    return 0;
}

static irqreturn_t upm6922_irq_handler(int irq, void *data)
{
    struct upm6922 *upm = (struct upm6922 *)data;

    pr_err("enter\n");

    schedule_delayed_work(&upm->charge_detect_delayed_work,
        msecs_to_jiffies(UPM6922_WORK_DELAY_TIME));

    return IRQ_HANDLED;
}

static int upm6922_get_chgtype(struct upm6922 *upm, int *type)
{
    int ret = 0;
    u8 chrg_stat = 0;
    u8 chg_type = 0;

    if (upm->typec_attached > ATTACH_TYPE_NONE && upm->power_good) {
        switch(upm->typec_attached) {
            case ATTACH_TYPE_PD_SDP:
            case ATTACH_TYPE_PD_DCP:
            case ATTACH_TYPE_PD_NONSTD:
                pr_err("Attach PD_TYPE, skip bc12 flow!\n");
                return ret;
            default:
                break;
        }
    }

    ret = upm6922_read_byte(upm, UPM6922_REG_08, &chrg_stat);
    if (ret) {
        pr_err("read UPM6922_CHRG_STAT fail\n");
        return ret;
    }

    upm->chrg_type = (chrg_stat & REG08_VBUS_STAT_MASK) >> REG08_VBUS_STAT_SHIFT;
    pr_err("chrg_type = 0x%x\n", upm->chrg_type);
    switch (upm->chrg_type) {
        case REG08_VBUS_TYPE_NONE:
            chg_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_USB;
            pr_info("charger type: none\n");
            break;
        case REG08_VBUS_TYPE_SDP:
            chg_type = POWER_SUPPLY_USB_TYPE_SDP;
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_USB;
            pr_info("charger type: SDP\n");
            break;
        case REG08_VBUS_TYPE_CDP:
            chg_type = POWER_SUPPLY_USB_TYPE_CDP;
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
            pr_info("charger type: CDP\n");
            break;
        case REG08_VBUS_TYPE_DCP:
            chg_type = POWER_SUPPLY_USB_TYPE_DCP;
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
            pr_info("charger type: DCP\n");
            break;
        case REG08_VBUS_TYPE_UNKNOWN:
            chg_type = POWER_SUPPLY_USB_TYPE_SDP;
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_USB;
            pr_info("charger type: unknown adapter\n");
            break;
        case REG08_VBUS_TYPE_NON_STD:
            chg_type = POWER_SUPPLY_USB_TYPE_DCP;
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
            pr_info("charger type: non-std charger\n");
            break;
        default:
            chg_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
            upm->this_psy_desc.type = POWER_SUPPLY_TYPE_USB;
            pr_info("charger type: invalid value\n");
            break;
    }

    if (chg_type == POWER_SUPPLY_USB_TYPE_UNKNOWN) {
        pr_err("unknown type\n");
    } else if (chg_type == POWER_SUPPLY_USB_TYPE_SDP ||
        chg_type == POWER_SUPPLY_USB_TYPE_CDP) {
    } else if (chg_type == POWER_SUPPLY_USB_TYPE_DCP) {

    }

    /*Tab A9 code for SR-AX6739A-01-500 by wenyaqi at 20230515 start*/
    if (upm->this_psy_desc.type == POWER_SUPPLY_TYPE_USB_DCP) {
        ret = upm6922_enable_dpdm_mux(upm, true);
        if (ret) {
            pr_err("Failed to en HVDCP, ret = %d\n", ret);
        }

        ret = upm6922_set_dp(upm, REG0C_DPDM_OUT_0V6V);
        if (ret) {
            pr_err("Failed to set dp 0.6v, ret = %d\n", ret);
        }
    }
    /*Tab A9 code for SR-AX6739A-01-500 by wenyaqi at 20230515 end*/

    *type = chg_type;
    power_supply_changed(upm->this_psy);

    return 0;
}

static int upm6922_ops_dump_register(struct charger_device *chg_dev);
static void charger_detect_work_func(struct work_struct *work)
{
    struct delayed_work *charge_detect_delayed_work = NULL;
    struct upm6922 *upm = NULL;

    int ret = 0;
    bool vbus_attach_pre = false;
    //bool power_good_pre = false;

    pr_notice("enter\n");

    charge_detect_delayed_work = container_of(work, struct delayed_work, work);
    if (charge_detect_delayed_work == NULL) {
        pr_err("Cann't get charge_detect_delayed_work\n");
        return;
    }

    upm = container_of(charge_detect_delayed_work, struct upm6922,
               charge_detect_delayed_work);
    if (upm == NULL) {
        pr_err("Cann't get upm6922\n");
        return;
    }

    vbus_attach_pre = upm->vbus_attach;
    //power_good_pre = upm->power_good;

    mutex_lock(&upm->lock);
    ret = upm6922_get_state(upm);
    mutex_unlock(&upm->lock);

    pr_err("v_pre = %d, v_now = %d, ret = %d\n",
        vbus_attach_pre, upm->vbus_attach, ret);

    if (!vbus_attach_pre && upm->vbus_attach) {
        pr_notice("adapter/usb inserted\n");
        upm->chg_config = true;
    } else if (vbus_attach_pre && !upm->vbus_attach) {
        pr_notice("adapter/usb removed\n");
        upm->chg_config = false;
        cancel_delayed_work_sync(&upm->force_detect_dwork);
        upm->force_dpdm = false;
        /*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 start*/
        cancel_delayed_work_sync(&upm->hvdcp_done_work);
        upm->hvdcp_done = false;
        upm6922_chg_set_usbsw(upm, USBSW_USB);
        /*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 end*/
    }

    /* bc12 done, get charger type */
    if ((upm->force_dpdm == true) && (upm->power_good == true)) {
        pr_notice("start get chgtype!\n");
        upm6922_get_chgtype(upm, &upm->psy_usb_type);
        upm->force_dpdm = false;
        /*A06_V code for SR-AL7160V-01-139 by xiongxiaoliang at 20240906 start*/
        if (upm->psy_usb_type != POWER_SUPPLY_USB_TYPE_DCP) {
            upm6922_chg_set_usbsw(upm, USBSW_USB);
        }
        /*A06_V code for SR-AL7160V-01-139 by xiongxiaoliang at 20240906 end*/
    }

    upm6922_ops_dump_register(upm->chg_dev);
}

/*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 start*/
static void hvdcp_done_work_func(struct work_struct *work)
{
    struct delayed_work *hvdcp_done_work = NULL;
    struct upm6922 *upm = NULL;

    hvdcp_done_work = container_of(work, struct delayed_work, work);
    if (hvdcp_done_work == NULL) {
        pr_err("Cann't get hvdcp_done_work\n");
        return;
    }

    upm = container_of(hvdcp_done_work, struct upm6922,
                      hvdcp_done_work);
    if (upm == NULL) {
        pr_err("Cann't get upm6922\n");
        return;
    }

    upm->hvdcp_done = true;
    power_supply_changed(upm->this_psy);
    pr_info("%s HVDCP end\n", __func__);
}
/*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 end*/

static int upm6922_register_interrupt(struct upm6922 *upm)
{
    int ret = 0;

    if (upm->intr_gpio < 0) {
        pr_err("no intr gpio, skip register interrupt\n");
        return 0;
    }

    ret = devm_gpio_request_one(upm->dev, upm->intr_gpio, GPIOF_DIR_IN,
            devm_kasprintf(upm->dev, GFP_KERNEL,
            "upm6922_intr_gpio.%s", dev_name(upm->dev)));
    if (ret < 0) {
        pr_err("gpio request fail(%d)\n", ret);
        return ret;
    }

    upm->client->irq = gpio_to_irq(upm->intr_gpio);
    if (upm->client->irq < 0) {
        pr_err("gpio2irq fail(%d)\n", upm->client->irq);
        return upm->client->irq;
    }

    ret = devm_request_threaded_irq(upm->dev, upm->client->irq, NULL,
                    upm6922_irq_handler,
                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                    "upm_irq", upm);
    if (ret < 0) {
        pr_err("request thread irq failed:%d\n", ret);
        return ret;
    }

    return 0;
}

static int upm6922_init_device(struct upm6922 *upm)
{
    int ret;

    upm6922_disable_watchdog_timer(upm);

    ret = upm6922_set_input_current_limit(upm, upm->upd->ilim);
    if (ret) {
        pr_err("Failed to set iindpm, ret = %d\n", ret);
    }

    ret = upm6922_set_chargecurrent(upm, upm->upd->ichg);
    if (ret) {
        pr_err("Failed to set ichg, ret = %d\n", ret);
    }

    ret = upm6922_set_prechg_current(upm, upm->upd->iprechg);
    if (ret) {
        pr_err("Failed to set prechg current, ret = %d\n", ret);
    }

    ret = upm6922_set_term_current(upm, upm->upd->iterm);
    if (ret) {
        pr_err("Failed to set termination current, ret = %d\n", ret);
    }

    ret = upm6922_set_chargevolt(upm, upm->upd->cv);
    if (ret) {
        pr_err("Failed to set cv, ret = %d\n", ret);
    }

    ret = upm6922_set_input_volt_limit(upm, upm->upd->vlim);
    if (ret) {
        pr_err("Failed to set vlim, ret = %d\n", ret);
    }

    ret = upm6922_set_boost_voltage(upm, upm->upd->boostv);
    if (ret) {
        pr_err("Failed to set boost voltage, ret = %d\n", ret);
    }

    ret = upm6922_set_boost_current(upm, upm->upd->boosti);
    if (ret) {
        pr_err("Failed to set boost current, ret = %d\n", ret);
    }

    ret = upm6922_set_acovp_threshold(upm, upm->upd->vac_ovp);
    if (ret) {
        pr_err("Failed to set acovp threshold, ret = %d\n", ret);
    }

    ret = upm6922_set_stat_ctrl(upm, upm->upd->statctrl);
    if (ret) {
        pr_err("Failed to set stat pin control mode, ret = %d\n", ret);
    }

    ret = upm6922_enable_vindpm_int(upm, false);
    if (ret) {
        pr_err("Failed to mask vindpm int, ret = %d\n", ret);
    }

    ret = upm6922_enable_iindpm_int(upm, false);
    if (ret) {
        pr_err("Failed to mask iindpm int, ret = %d\n", ret);
    }

    ret = upm6922_set_adc_rate(upm, upm->en_adc_continuous);
    if (ret) {
        pr_err("Failed to set adc rate, ret = %d\n", ret);
    }

    return 0;
}

#ifndef __MTK_CHARGER_TYPE_H__
#define __MTK_CHARGER_TYPE_H__
enum charger_type {
    CHARGER_UNKNOWN = 0,
    STANDARD_HOST,
    CHARGING_HOST,
    NONSTANDARD_CHARGER,
    STANDARD_CHARGER,
    APPLE_2_1A_CHARGER,
    APPLE_1_0A_CHARGER,
    APPLE_0_5A_CHARGER,
    WIRELESS_CHARGER,
};
#endif /* __MTK_CHARGER_TYPE_H__ */

static void upm6922_inform_psy_dwork_handler(struct work_struct *work)
{
	struct upm6922 *upm = container_of(work, struct upm6922,
								psy_dwork.work);
    int ret = 0;
	union power_supply_propval propval;
    int vbus_stat = 0;
    u8 reg_val = 0;

    if (!upm->inform_psy) {
        upm->inform_psy = power_supply_get_by_name("charger");
        if (!upm->inform_psy) {
            pr_err("Couldn't get psy\n");
            mod_delayed_work(system_wq, &upm->psy_dwork,
                    msecs_to_jiffies(2*1000));
            return;
        }
    }

    propval.intval = upm->psy_usb_type ? 1 : 0;
    ret = power_supply_set_property(upm->inform_psy,
                POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0) {
		pr_err("inform power supply online failed:%d\n", ret);
    }

    ret = upm6922_read_byte(upm, UPM6922_REG_08, &reg_val);
    if (ret) {
        pr_err("read UPM6922_REG_08 fail, ret:%d\n", ret);
        return ;
    }

    vbus_stat = (reg_val & REG08_VBUS_STAT_MASK);
    vbus_stat >>= REG08_VBUS_STAT_SHIFT;

    switch (vbus_stat) {
        case REG08_VBUS_TYPE_NONE:
            propval.intval = CHARGER_UNKNOWN;
            break;

        case REG08_VBUS_TYPE_SDP:
            propval.intval = STANDARD_HOST;
            break;

        case REG08_VBUS_TYPE_CDP:
            propval.intval = CHARGING_HOST;
            break;

        case REG08_VBUS_TYPE_DCP:
        case REG08_VBUS_TYPE_NON_STD:
            propval.intval = STANDARD_CHARGER;
            break;

        case REG08_VBUS_TYPE_UNKNOWN:
            propval.intval = NONSTANDARD_CHARGER;
            break;

        default:
            pr_err("is otg? vbus_stat:%d\n", vbus_stat);
            propval.intval = CHARGER_UNKNOWN;
            break;
    }

    ret = power_supply_set_property(upm->inform_psy,
                POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0) {
		pr_err("inform power supply charge type failed:%d\n", ret);
    }
}

static void upm6922_monitor_dwork_handler(struct work_struct *work)
{
    struct upm6922 *upm = container_of(work, struct upm6922,
                                monitor_dwork.work);

    __pm_stay_awake(upm->monitor_ws);

    pr_info("monitor work\n");
    upm6922_dump_regs(upm);

    schedule_delayed_work(&upm->monitor_dwork, msecs_to_jiffies(10*1000));

    __pm_relax(upm->monitor_ws);
}

static ssize_t upm6922_show_registers(struct device *dev, 
                    struct device_attribute *attr, char *buf)
{
    struct upm6922 *upm = dev_get_drvdata(dev);
    u8 addr;
    u8 val;
    u8 tmpbuf[200];
    int len;
    int idx = 0;
    int ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "upm6922 Reg");
    for (addr = 0x0; addr <= 0x16; addr++) {
        ret = upm6922_read_byte(upm, addr, &val);
        if (ret == 0) {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                       "Reg[%.2x] = 0x%.2x\n", addr, val);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t upm6922_store_registers(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
    struct upm6922 *upm = dev_get_drvdata(dev);
    int ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= 0x16) {
        upm6922_write_byte(upm, (unsigned char) reg,
                   (unsigned char) val);
    }

    return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, upm6922_show_registers,
           upm6922_store_registers);

static struct attribute *upm6922_attributes[] = {
    &dev_attr_registers.attr,
    NULL,
};

static const struct attribute_group upm6922_attr_group = {
    .attrs = upm6922_attributes,
};

static enum power_supply_usb_type upm6922_usb_types[] = {
    POWER_SUPPLY_USB_TYPE_UNKNOWN,
    POWER_SUPPLY_USB_TYPE_SDP,
    POWER_SUPPLY_USB_TYPE_CDP,
    POWER_SUPPLY_USB_TYPE_DCP,
};

static enum power_supply_property upm6922_charger_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_USB_TYPE,
    POWER_SUPPLY_PROP_MANUFACTURER,
    POWER_SUPPLY_PROP_TYPE,
    POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
    POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
    POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static int upm6922_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    struct upm6922 *upm = power_supply_get_drvdata(psy);
    int ret = 0;

    val->intval = 0;
    switch (psp) {
        case POWER_SUPPLY_PROP_STATUS:
            ret = upm6922_get_charging_stat(upm, &val->intval);
            break;
        /*A06_V code for SR-AL7160V-01-134 by xiongxiaoliang at 20240830 start*/
        case POWER_SUPPLY_PROP_ONLINE:
            val->intval = upm->vbus_attach | (!!upm->typec_attached);
            if (upm->this_psy_desc.type == POWER_SUPPLY_TYPE_UNKNOWN) {
                val->intval = 0;
            }
            break;
        /*A06_V code for SR-AL7160V-01-134 by xiongxiaoliang at 20240830 end*/
        case POWER_SUPPLY_PROP_USB_TYPE:
            val->intval = upm->psy_usb_type;
            break;
        case POWER_SUPPLY_PROP_MANUFACTURER:
            val->strval = "Unisemipower";
            break;
        case POWER_SUPPLY_PROP_TYPE:
            val->intval = upm->this_psy_desc.type;
            break;
        default:
            break;
    }

    if (ret) {
        pr_err("err: psp:%d, ret:%d", psp, ret);
    }

    return ret;
}

static int upm6922_ops_set_icl(struct charger_device *chg_dev, u32 ua);
static int upm6922_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
    int ret = -EINVAL;

    switch (prop) {
        case POWER_SUPPLY_PROP_ONLINE:
            pr_info("%d\n", val->intval);
            ret = upm6922_chg_attach_pre_process(s_chg_dev_otg, val->intval);
            break;
        case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
            ret = upm6922_ops_set_icl(s_chg_dev_otg, val->intval);
            break;
        default:
            return -EINVAL;
    }

    return ret;
}

static int upm6922_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
    switch (prop) {
        case POWER_SUPPLY_PROP_ONLINE:
        case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
        case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
        case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
        case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
        case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
            return true;
        default:
            return false;
    }
}

static char *upm6922_charger_supplied_to[] = {
    "battery",
    "mtk-master-charger",
};

static struct power_supply_desc upm6922_power_supply_desc = {
    .name = "charger",
    .type = POWER_SUPPLY_TYPE_UNKNOWN,
    .usb_types = upm6922_usb_types,
    .num_usb_types = ARRAY_SIZE(upm6922_usb_types),
    .properties = upm6922_charger_props,
    .num_properties = ARRAY_SIZE(upm6922_charger_props),
    .get_property = upm6922_charger_get_property,
    .set_property = upm6922_charger_set_property,
    .property_is_writeable = upm6922_charger_is_writeable,
};

static int upm6922_charger_psy_register(struct upm6922 *upm)
{

    struct power_supply_config psy_cfg = {
        .drv_data = upm,
        .of_node = upm->dev->of_node,
    };

    psy_cfg.supplied_to = upm6922_charger_supplied_to;
    psy_cfg.num_supplicants = ARRAY_SIZE(upm6922_charger_supplied_to);

    memcpy(&upm->this_psy_desc, &upm6922_power_supply_desc,
        sizeof(upm->this_psy_desc));

    upm->this_psy = devm_power_supply_register(upm->dev,
            &upm->this_psy_desc, &psy_cfg);
    if (IS_ERR(upm->this_psy)) {
        pr_err("failed to register charger_psy\n");
        return PTR_ERR(upm->this_psy);
    }

    pr_err("%s power supply register successfully\n", upm->this_psy_desc.name);

    return 0;
}

/*************************mtk interface start*************************/
static int upm6922_ops_charging(struct charger_device *chg_dev, bool enable)
{
    struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
    int ret = 0;

    if (enable) {
        ret = upm6922_enable_charger(upm);
    } else {
        ret = upm6922_disable_charger(upm);
    }
    /*A06_V code for AL7160AV-47 by xiongxiaoliang at 20241112 start*/
    upm->chg_config = enable;
    /*A06_V code for AL7160AV-47 by xiongxiaoliang at 20241112 end*/

    return ret;
}

static int upm6922_ops_plug_in(struct charger_device *chg_dev)
{
	int ret;

	ret = upm6922_ops_charging(chg_dev, true);
	if (ret) {
		pr_err("Failed to enable charging:%d\n", ret);
    }

	return ret;
}

static int upm6922_ops_plug_out(struct charger_device *chg_dev)
{
	int ret;

	ret = upm6922_ops_charging(chg_dev, false);
	if (ret) {
		pr_err("Failed to disable charging:%d\n", ret);
    }

	return ret;
}

static int upm6922_ops_dump_register(struct charger_device *chg_dev)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);

	upm6922_dump_regs(upm);

	return 0;
}

static int upm6922_ops_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
    int ret = 0;
    u8 val = 0;

	ret = upm6922_read_byte(upm, UPM6922_REG_01, &val);
	if (ret) {
        pr_err("read UPM6922_REG_01 fail, ret:%d\n", ret);
        return ret;
    }

    *en = !!(val & REG01_CHG_CONFIG_MASK);

	return 0;
}

/*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 start*/
static int upm6922_set_shipmode(struct charger_device *chg_dev, bool en)
{
    int ret = 0;
    u8 val = 0;

    struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
    if (en) {
        ret += upm6922_enable_hiz_mode(upm);
        val = REG07_BATFET_OFF << REG07_BATFET_DIS_SHIFT;
        ret += upm6922_update_bits(upm, UPM6922_REG_07, REG07_BATFET_DIS_MASK, val);
    } else {
        val = REG07_BATFET_ON << REG07_BATFET_DIS_SHIFT;
        ret = upm6922_update_bits(upm, UPM6922_REG_07, REG07_BATFET_DIS_MASK, val);
    }
    pr_err(" shipmode %s\n", !ret ? "successfully" : "failed");
    return ret;
}

static int upm6922_get_shipmode(struct charger_device *chg_dev)
{
    int ret = 0;
    u8 val = 0;
    struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);

    msleep(100);
    ret = upm6922_read_byte(upm, UPM6922_REG_07, &val);
    if (ret == 0) {
        pr_err("Reg[%.2x] = 0x%.2x\n", UPM6922_REG_07, val);
    } else {
        pr_err("%s: get shipmode reg fail! \n", __func__);
        return ret;
    }
    ret = (val & REG07_BATFET_DIS_MASK) >> REG07_BATFET_DIS_SHIFT;
    pr_err("%s:shipmode %s\n", __func__, ret ? "enabled" : "disabled");

    return ret;
}

static int upm6922_disable_battfet_rst(struct upm6922 *upm)
{
    int ret = 0;
    u8 val = 0;

    val = REG07_BATFET_RST_DISABLE << REG07_BATFET_RST_EN_SHIFT;
    ret = upm6922_update_bits(upm, UPM6922_REG_07,
                               REG07_BATFET_RST_EN_MASK, val);

    pr_err("disable BATTFET_RST_EN %s\n", !ret ? "successfully" : "failed");

    return ret;
}
/*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 end*/


static int upm6922_ops_get_ichg(struct charger_device *chg_dev, u32 *ua)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
	int ma = 0;
	int ret = 0;

    ret = upm6922_get_chargecurrent(upm, &ma);
    if (ret) {
        pr_err("get ichg fail, ret:%d\n", ret);
        return ret;
    }
    *ua = ma * 1000;

	return ret;
}

static int upm6922_ops_set_ichg(struct charger_device *chg_dev, u32 ua)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);

	return upm6922_set_chargecurrent(upm, ua / 1000);
}

static int upm6922_ops_get_icl(struct charger_device *chg_dev, u32 *ua)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
	int ma = 0;
	int ret = 0;

    ret = upm6922_get_input_current_limit(upm, &ma);
    if (ret) {
        pr_err("get icl fail, ret:%d\n", ret);
        return ret;
    }

    *ua = ma * 1000;

	return ret;
}

static int upm6922_ops_set_icl(struct charger_device *chg_dev, u32 ua)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
    /*A06_V code for SR-AL7160V-01-201 by xiongxiaoliang at 20240911 start*/
    #if IS_ENABLED(CONFIG_CHARGER_CP_PPS)
    if (g_cp_charging_lmt.cp_chg_enable) {
        ua = 200000;
        pr_err("cp open icl curr = %d\n", ua);
    }
    #endif
    /*A06_V code for SR-AL7160V-01-201 by xiongxiaoliang at 20240911 end*/
	return upm6922_set_input_current_limit(upm, ua / 1000);
}

static int upm6922_ops_get_vchg(struct charger_device *chg_dev, u32 *uv)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
	int mv;
	int ret;

	ret = upm6922_get_chargevolt(upm, &mv);
    if (ret) {
        pr_err("get charge voltage fail, ret:%d\n", ret);
        return ret;
    }
    *uv = mv * 1000;

	return ret;
}

static int upm6922_ops_set_vchg(struct charger_device *chg_dev, u32 uv)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);

	return upm6922_set_chargevolt(upm, uv / 1000);
}

static int upm6922_ops_kick_wdt(struct charger_device *chg_dev)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);

	return upm6922_reset_watchdog_timer(upm);
}

static int upm6922_ops_set_ivl(struct charger_device *chg_dev, u32 uv)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);

	return upm6922_set_input_volt_limit(upm, uv / 1000);
}

static int upm6922_ops_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;

	ret = upm6922_read_byte(upm, UPM6922_REG_08, &val);
	if (!ret) {
		val = val & REG08_CHRG_STAT_MASK;
		val = val >> REG08_CHRG_STAT_SHIFT;
		*done = (val == REG08_CHRG_STAT_CHGDONE);
	}

	return ret;
}

static int upm6922_ops_get_min_ichg(struct charger_device *chg_dev, u32 *ua)
{
	*ua = REG02_ICHG_LSB * 1000;

	return 0;
}

static int upm6922_ops_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
	int ret;

	if (en) {
		ret = upm6922_enable_safety_timer(upm);
	} else {
		ret = upm6922_disable_safety_timer(upm);
    }

	return ret;
}

static int upm6922_ops_is_safety_timer_enabled(struct charger_device *chg_dev,
					   bool *en)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = upm6922_read_byte(upm, UPM6922_REG_05, &reg_val);

	if (!ret) {
		*en = !!(reg_val & REG05_EN_TIMER_MASK);
    }

	return ret;
}

static int upm6922_ops_set_otg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);

	if (en) {
		ret = upm6922_enable_otg(upm);
	} else {
		ret = upm6922_disable_otg(upm);
    }

	return ret;
}

static int upm6922_ops_set_boost_ilmt(struct charger_device *chg_dev, u32 ua)
{
	struct upm6922 *upm = dev_get_drvdata(&chg_dev->dev);
	int ret;

	ret = upm6922_set_boost_current(upm, ua / 1000);

	return ret;
}

/*A06_V code for  SR-AL7160V-01-172  by yexuedong at 20240903 start*/
static int upm6922_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
    struct upm6922 *sc = dev_get_drvdata(&chg_dev->dev);
    /*A06_V code for AL7160AV-47 by xiongxiaoliang at 20241112 start*/
    dev_info(sc->dev, "%s, %d\n", __func__, event);
    /*A06_V code for AL7160AV-47 by xiongxiaoliang at 20241112 end*/
    switch (event) {
        case EVENT_FULL:
        case EVENT_RECHARGE:
        case EVENT_DISCHARGE:
            power_supply_changed(sc->this_psy);
            break;
        default:
            break;
    }

    return 0;
}
/*A06_V code for  SR-AL7160V-01-172  by yexuedong at 20240903 end*/
/*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 start*/
static int upm6922_ops_get_hcdcp_status(struct charger_device *chg_dev)
{
    int ret = 0;
    struct upm6922 *upm = charger_get_data(chg_dev);

    if (upm == NULL) {
        pr_info("[%s] fail\n", __func__);
        return ret;
    }

    ret = (upm->hvdcp_done ? 1 : 0);

    return ret;
}
/*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 end*/

/*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 start*/
static int upm6922_hw_init(struct upm6922 *upm)
{
    int ret = 0;

    ret = upm6922_disable_battfet_rst(upm);
    if (ret) {
        goto err_out;
    }
    return 0;

err_out:
    return ret;
}
/*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 start*/

static const struct charger_properties upm6922_chg_props = {
	.alias_name = "upm6922",
};

static struct charger_ops upm6922_chg_ops = {
	/* Normal charging */
	.plug_in = upm6922_ops_plug_in,
	.plug_out = upm6922_ops_plug_out,
	.dump_registers = upm6922_ops_dump_register,
	.enable = upm6922_ops_charging,
	.is_enabled = upm6922_ops_is_charging_enable,
	.get_charging_current = upm6922_ops_get_ichg,
	.set_charging_current = upm6922_ops_set_ichg,
	.get_input_current = upm6922_ops_get_icl,
	.set_input_current = upm6922_ops_set_icl,
	.get_constant_voltage = upm6922_ops_get_vchg,
	.set_constant_voltage = upm6922_ops_set_vchg,
	.kick_wdt = upm6922_ops_kick_wdt,
	.set_mivr = upm6922_ops_set_ivl,
	.is_charging_done = upm6922_ops_is_charging_done,
	.get_min_charging_current = upm6922_ops_get_min_ichg,

	/* Safety timer */
	.enable_safety_timer = upm6922_ops_set_safety_timer,
	.is_safety_timer_enabled = upm6922_ops_is_safety_timer_enabled,

	/* Power path */
	.enable_powerpath = NULL,
	.is_powerpath_enabled = NULL,

	/* OTG */
	.enable_otg = upm6922_ops_set_otg,
	.set_boost_current_limit = upm6922_ops_set_boost_ilmt,
	.enable_discharge = NULL,
    /*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 start*/
    .get_ship_mode = upm6922_get_shipmode,
    .set_ship_mode = upm6922_set_shipmode,
    /*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 end*/

	/* PE+/PE+20 */
	.send_ta_current_pattern = NULL,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
	.enable_cable_drop_comp = NULL,
    /*A06_V code for  SR-AL7160V-01-172  by yexuedong at 20240903 start*/
   /* Event */
    .event = upm6922_do_event,
    /*A06_V code for  SR-AL7160V-01-172  by yexuedong at 20240903 end*/
	/* ADC */
	.get_tchg_adc = NULL,
	/*A06_V code for SR-AL7160V-01-139 by xiongxiaoliang at 20240906 start*/
	.get_hvdcp_status = upm6922_ops_get_hcdcp_status,
	/*A06_V code for SR-AL7160V-01-139 by xiongxiaoliang at 20240906 end*/
    /*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 start*/
    .enable_port_charging = upm6922_enable_port_charging,
    .is_port_charging_enabled= upm6922_is_port_charging_enabled,
    /*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 end*/
};

/*A06_V code for AL7160AV-532 by yexuedong at 20250703 start*/
extern void usbpd_pm_set_otg_txmode(int enable);
/*A06_V code for AL7160AV-532 by yexuedong at 20250703 end*/
static int upm6922_enable_vbus(struct regulator_dev *rdev)
{
	int ret;
	struct upm6922 *upm = rdev_get_drvdata(rdev);

    /*A06_V code for AL7160AV-532 by yexuedong at 20250703 start*/
    usbpd_pm_set_otg_txmode(true);
    /*A06_V code for AL7160AV-532 by yexuedong at 20250703 end*/

    ret = upm6922_enable_otg(upm);
    pr_err("ret = %d\n", ret);

	return 0;
}

static int upm6922_disable_vbus(struct regulator_dev *rdev)
{
    int ret;
    struct upm6922 *upm = rdev_get_drvdata(rdev);

    /*A06_V code for AL7160AV-532 by yexuedong at 20250703 start*/
    usbpd_pm_set_otg_txmode(false);
    /*A06_V code for AL7160AV-532 by yexuedong at 20250703 end*/

    ret = upm6922_disable_otg(upm);
    pr_err("ret = %d\n", ret);

	return 0;
}

static int upm6922_is_enabled_vbus(struct regulator_dev *rdev)
{
    int ret;
	struct upm6922 *upm = rdev_get_drvdata(rdev);
	u8 val = 0;

	ret = upm6922_read_byte(upm, UPM6922_REG_01, &val);
    if (ret < 0)
		return ret;

	return (val & REG01_OTG_CONFIG_MASK) >> REG01_OTG_CONFIG_SHIFT;
}

static const struct regulator_ops upm6922_vbus_ops = {
	.enable = upm6922_enable_vbus,
	.disable = upm6922_disable_vbus,
	.is_enabled = upm6922_is_enabled_vbus,
};

static const struct regulator_desc upm6922_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &upm6922_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

/**************************mtk interface end**************************/

static int upm6922_charger_probe(struct i2c_client *client)
{
    struct upm6922 *upm;
    struct device_node *node = client->dev.of_node;
    int ret = 0;
    __maybe_unused struct regulator_config config = { };

    upm = devm_kzalloc(&client->dev, sizeof(struct upm6922), GFP_KERNEL);
    if (!upm) {
        pr_err("alloc upm6922 struct failed\n");
        return -ENOMEM;
    }

    client->addr = 0x6B;
    upm->dev = &client->dev;
    upm->client = client;

    i2c_set_clientdata(client, upm);
    mutex_init(&upm->i2c_rw_lock);
    mutex_init(&upm->lock);

    ret = upm6922_detect_device(upm);
    if (ret) {
        pr_err("No upm6922 device found!\n");
        ret = -ENODEV;
        goto err_nodev;
    }

    if (upm->part_no != REG0B_PN_UPM6922) {
        pr_err("part no not match, not upm6922!\n");
        ret = -ENODEV;
        goto err_nodev;
    }
    /*A06_V code for SR-AL7160V-01-206  by yexuedong at 20240902 start*/
    #if IS_ENABLED(CONFIG_HQ_PROJECT_O8)
    gxy_bat_set_chginfo(GXY_BAT_CHG_INFO_UPM6922);
    #endif
    /*A06_V code for SR-AL7160V-01-206  by yexuedong at 20240902 end*/
    /*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 start*/
    ret = upm6922_hw_init(upm);
    if (ret) {
        pr_err("Cannot initialize the chip.\n");
        ret = -ENODEV;
        goto err_nodev;
    }
    /*A06_V code for SR-AL7160V-01-164 by yexuedong at 20240910 start*/
    upm->upd = upm6922_parse_dt(node, upm);
    if (!upm->upd) {
        pr_err("No platform data provided.\n");
        ret = -EINVAL;
        goto err_parse_dt;
    }

    INIT_DELAYED_WORK(&upm->psy_dwork, upm6922_inform_psy_dwork_handler);
    INIT_DELAYED_WORK(&upm->force_detect_dwork, upm6922_force_detection_dwork_handler);
    INIT_DELAYED_WORK(&upm->charge_detect_delayed_work, charger_detect_work_func);

    upm->monitor_ws = wakeup_source_register(upm->dev, "upm6922_monitor_ws");
    INIT_DELAYED_WORK(&upm->monitor_dwork, upm6922_monitor_dwork_handler);
    /*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 start*/
    INIT_DELAYED_WORK(&upm->hvdcp_done_work, hvdcp_done_work_func);
    upm->hvdcp_done = false;
    /*Tab A9 code for AX6739A-409 by wenyaqi at 20230530 end*/
    ret = upm6922_init_device(upm);
    if (ret) {
        pr_err("Failed to init device\n");
        ret = -EINVAL;
        goto err_dev_init;
    }
    upm6922_dump_regs(upm);

    upm6922_register_interrupt(upm);

    ret = sysfs_create_group(&upm->dev->kobj, &upm6922_attr_group);
    if (ret)
        pr_err("failed to register sysfs. err: %d\n", ret);

    ret = upm6922_charger_psy_register(upm);
    if (ret) {
        pr_err("failed to register chg psy. err: %d\n", ret);
    }


    upm->chg_dev = charger_device_register(upm->chg_dev_name,
					      &client->dev, upm,
					      &upm6922_chg_ops,
					      &upm6922_chg_props);
	if (IS_ERR_OR_NULL(upm->chg_dev)) {
		ret = PTR_ERR(upm->chg_dev);
		pr_err("register charge device failed, ret:%d\n", ret);
	}

    s_chg_dev_otg = upm->chg_dev;

    /* otg regulator */
	config.dev = upm->dev;
	config.driver_data = upm;
	upm->otg_rdev = devm_regulator_register(upm->dev,
						&upm6922_otg_rdesc, &config);
	/*A06_V code for SR-AL7160V-01-92 by xiongxiaoliang at 20240904 start*/
	upm->otg_rdev->constraints->valid_ops_mask |= REGULATOR_CHANGE_STATUS;
	/*A06_V code for SR-AL7160V-01-92 by xiongxiaoliang at 20240904 end*/
	if (IS_ERR(upm->otg_rdev)) {
		ret = PTR_ERR(upm->otg_rdev);
		pr_err("register otg regulator failed (%d)\n", ret);
	}

    enable_irq_wake(upm->client->irq);
    device_init_wakeup(upm->dev, true);
    /*A06_V code for AL7160AV-15 by lina at 20240904 start*/
    schedule_delayed_work(&upm->charge_detect_delayed_work, msecs_to_jiffies(UPM6922_WORK_DELAY_TIME));
    /*A06_V code for AL7160AV-15 by lina at 20240904 end*/
    pr_err("upm6922 probe successfully, Part Num:%d, Revision:%d\n!",
           upm->part_no, upm->revision);

    return 0;

err_dev_init:
err_parse_dt:
err_nodev:
    mutex_destroy(&upm->i2c_rw_lock);
    devm_kfree(&client->dev, upm);

    return ret;
}

static void upm6922_charger_remove(struct i2c_client *client)
{
    struct upm6922 *upm = i2c_get_clientdata(client);

    sysfs_remove_group(&upm->dev->kobj, &upm6922_attr_group);
    mutex_destroy(&upm->i2c_rw_lock);

}

static void upm6922_charger_shutdown(struct i2c_client *client)
{
    struct upm6922 *upm = i2c_get_clientdata(client);

    upm6922_set_adc_rate(upm, REG15_CONV_RATE_ONE_SHOT);

    pr_info("shutdown\n");
}

static int upm6922_pm_suspend(struct device *dev)
{
    pr_info("suspend\n");

    return 0;
}

static int upm6922_pm_resume(struct device *dev)
{
    pr_info("resume\n");

    return 0;
}

static const struct dev_pm_ops upm6922_pm_ops = {
    .resume = upm6922_pm_resume,
    .suspend = upm6922_pm_suspend,
};

static struct of_device_id upm6922_charger_match_table[] = {
    {.compatible = "up,upm6922_charger",},
    {},
};
MODULE_DEVICE_TABLE(of, upm6922_charger_match_table);

static struct i2c_driver upm6922_charger_driver = {
    .driver = {
        .name = "upm6922-charger",
        .owner = THIS_MODULE,
        .of_match_table = upm6922_charger_match_table,
        .pm = &upm6922_pm_ops,
    },

    .probe = upm6922_charger_probe,
    .remove = upm6922_charger_remove,
    .shutdown = upm6922_charger_shutdown,
};

module_i2c_driver(upm6922_charger_driver);

MODULE_DESCRIPTION("Unisemipower UPM6922 Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("lai.du@unisemipower.com");
