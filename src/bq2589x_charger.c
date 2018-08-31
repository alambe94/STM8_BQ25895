/*
* BQ2589x battery charging driver
*
* Copyright (C) 2013 Texas Instruments
*
* This package is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.

* THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
* WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "bq2589x_reg.h"
#include "stm8l15x.h"
#include "soft_i2c.h"


#define BQ25895_ADDR (0x6A << 1)


int bq2589x_read_byte(u8 *data, u8 reg)
{
  
  if(Soft_I2C_Read_Byte(BQ25895_ADDR,reg,data)==SOFT_I2C_OK)
  {
    return BQ25895_OK;//TI lib uses 1 as failed
  }
  else
  {
    return BQ25895_ERR;//TI lib uses 1 as failed
  }
  
}


int bq2589x_write_byte(u8 reg, u8 data)
{
  
  if(Soft_I2C_Write_Byte(BQ25895_ADDR,reg,&data)==SOFT_I2C_OK)
  {
    return BQ25895_OK;//TI lib uses 1 as failed
  }
  else
  {
    return BQ25895_ERR;//TI lib uses 1 as failed
  }
}


int bq2589x_update_bits(u8 reg, u8 mask, u8 data)
{
  int ret;
  u8 tmp;
  
  ret = bq2589x_read_byte(&tmp, reg);
  
  if (ret)
    return ret;
  
  tmp &= ~mask;
  tmp |= data & mask;
  
  return bq2589x_write_byte(reg, tmp);
}


bq2589x_vbus_type bq2589x_get_vbus_type()
{
  u8 val = 0;
  int ret;
  
  ret = bq2589x_read_byte(&val, BQ2589X_REG_0B);
  if (ret)
    return (BQ2589X_VBUS_UNKNOWN);
  val &= BQ2589X_VBUS_STAT_MASK;
  val >>= BQ2589X_VBUS_STAT_SHIFT;
  
  return (bq2589x_vbus_type)val;
}


int bq2589x_enable_otg()
{
  u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;
  
  return bq2589x_update_bits(BQ2589X_REG_03,
                             BQ2589X_OTG_CONFIG_MASK, val);
  
}

int bq2589x_disable_otg()
{
  u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;
  
  return bq2589x_update_bits(BQ2589X_REG_03,
                             BQ2589X_OTG_CONFIG_MASK, val);
  
}

int bq2589x_set_otg_volt(int volt)
{
  u8 val = 0;
  
  if (volt < BQ2589X_BOOSTV_BASE)
    volt = BQ2589X_BOOSTV_BASE;
  if (volt > BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB)
    volt = BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB;
  
  val = ((volt - BQ2589X_BOOSTV_BASE) / BQ2589X_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;
  
  return bq2589x_update_bits(BQ2589X_REG_0A, BQ2589X_BOOSTV_MASK, val);
  
}

int bq2589x_set_otg_current(int curr)
{
  u8 temp;
  
  if (curr == 500)
    temp = BQ2589X_BOOST_LIM_500MA;
  else if (curr == 700)
    temp = BQ2589X_BOOST_LIM_700MA;
  else if (curr == 1100)
    temp = BQ2589X_BOOST_LIM_1100MA;
  else if (curr == 1600)
    temp = BQ2589X_BOOST_LIM_1600MA;
  else if (curr == 1800)
    temp = BQ2589X_BOOST_LIM_1800MA;
  else if (curr == 2100)
    temp = BQ2589X_BOOST_LIM_2100MA;
  else if (curr == 2400)
    temp = BQ2589X_BOOST_LIM_2400MA;
  else
    temp = BQ2589X_BOOST_LIM_1300MA;
  
  return bq2589x_update_bits(BQ2589X_REG_0A, BQ2589X_BOOST_LIM_MASK, temp << BQ2589X_BOOST_LIM_SHIFT);
}

int bq2589x_enable_charger()
{
  u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;
  return bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
}

int bq2589x_disable_charger()
{
  u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;
  return bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
}


/* interfaces that can be called by other module */
int bq2589x_adc_start(bool oneshot)
{
  u8 val;
  int ret;
  
  ret = bq2589x_read_byte(&val, BQ2589X_REG_02);
  if (ret) 
  {
    return ret;
  }
  
  if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) == BQ2589X_ADC_CONTINUE_ENABLE)
    return BQ25895_OK; /*is doing continuous scan*/
  if (oneshot)
    ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
  else
    ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,  BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
  return ret;
}

int bq2589x_adc_stop()
{
  return bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}


int bq2589x_adc_read_battery_volt()
{
  uint8_t val;
  int volt;
  int ret;
  ret = bq2589x_read_byte(&val, BQ2589X_REG_0E);
  if (ret) {
    return ret;
  } else{
    volt = (BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB) ;
    return volt;
  }
}


int bq2589x_adc_read_sys_volt()
{
  uint8_t val;
  int volt;
  int ret;
  ret = bq2589x_read_byte(&val, BQ2589X_REG_0F);
  if (ret) {
    return ret;
  } else{
    volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB ;
    return volt;
  }
}

int bq2589x_adc_read_vbus_volt()
{
  uint8_t val;
  int volt;
  int ret;
  ret = bq2589x_read_byte(&val, BQ2589X_REG_11);
  if (ret) {
    return ret;
  } else{
    volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
    return volt;
  }
}

int bq2589x_adc_read_temperature()
{
  uint8_t val;
  int temp;
  int ret;
  ret = bq2589x_read_byte(&val, BQ2589X_REG_10);
  if (ret) {
    return ret;
  } else{
    temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
    return temp;
  }
}

int bq2589x_adc_read_charge_current()
{
  uint8_t val;
  int volt;
  int ret;
  ret = bq2589x_read_byte(&val, BQ2589X_REG_12);
  if (ret) {
    return ret;
  } else{
    volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
    return volt;
  }
}

int bq2589x_set_charge_current(int curr)
{
  u8 ichg;
  ichg = (curr - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
  return bq2589x_update_bits(BQ2589X_REG_04, BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);
  
}

int bq2589x_set_term_current(int curr)
{
  u8 iterm;
  
  iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;
  
  return bq2589x_update_bits(BQ2589X_REG_05, BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}


int bq2589x_set_prechg_current(int curr)
{
  u8 iprechg;
  
  iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;
  
  return bq2589x_update_bits(BQ2589X_REG_05, BQ2589X_IPRECHG_MASK, iprechg << BQ2589X_IPRECHG_SHIFT);
}

int bq2589x_set_chargevoltage(int volt)
{
  u8 val;
  
  val = (volt - BQ2589X_VREG_BASE)/BQ2589X_VREG_LSB;
  return bq2589x_update_bits(BQ2589X_REG_06, BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}


int bq2589x_set_input_volt_limit(int volt)
{
  u8 val;
  val = (volt - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;
  return bq2589x_update_bits(BQ2589X_REG_0D, BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
}

int bq2589x_set_input_current_limit(int curr)
{
  u8 val;
  
  val = (curr - BQ2589X_IINLIM_BASE) / BQ2589X_IINLIM_LSB;
  return bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_IINLIM_MASK, val << BQ2589X_IINLIM_SHIFT);
}


int bq2589x_set_vindpm_offset(int offset)
{
  u8 val;
  
  val = (offset - BQ2589X_VINDPMOS_BASE)/BQ2589X_VINDPMOS_LSB;
  return bq2589x_update_bits(BQ2589X_REG_01, BQ2589X_VINDPMOS_MASK, val << BQ2589X_VINDPMOS_SHIFT);
}


int bq2589x_get_charging_status()
{
  u8 val = 0;
  int ret;
  
  ret = bq2589x_read_byte(&val, BQ2589X_REG_0B);
  if (ret) {
    return 0x04; //Error
  }
  val &= BQ2589X_CHRG_STAT_MASK;
  val >>= BQ2589X_CHRG_STAT_SHIFT;
  return val;
}


void bq2589x_set_otg(int enable)
{
  int ret;
  
  if (enable) {
    ret = bq2589x_enable_otg();
    if (ret) {
      return;
    }
  } else{
    ret = bq2589x_disable_otg();
  }
}


int bq2589x_set_watchdog_timer(u8 timeout)
{
  return bq2589x_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK, (u8)((timeout - BQ2589X_WDT_BASE) / BQ2589X_WDT_LSB) << BQ2589X_WDT_SHIFT);
}


int bq2589x_disable_watchdog_timer()
{
  u8 val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;
  
  return bq2589x_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK, val);
}


int bq2589x_reset_watchdog_timer()
{
  u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;
  
  return bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}


int bq2589x_force_dpdm()
{
  int ret;
  u8 val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_FORCE_DPDM_MASK, val);
  if (ret)
    return ret;
  
  //msleep(20);/*TODO: how much time needed to finish dpdm detect?*/
  return BQ25895_OK;
  
}


int bq2589x_reset_chip()
{
  int ret;
  u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
  return ret;
}


int bq2589x_enter_ship_mode()
{
  int ret;
  u8 val = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
  return ret;
  
}


int bq2589x_enter_hiz_mode()
{
  u8 val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;
  
  return bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);
  
}


int bq2589x_exit_hiz_mode()
{
  
  u8 val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;
  
  return bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);
  
}


int bq2589x_get_hiz_mode(u8 *state)
{
  u8 val;
  int ret;
  
  ret = bq2589x_read_byte(&val, BQ2589X_REG_00);
  if (ret)
    return ret;
  *state = (val & BQ2589X_ENHIZ_MASK) >> BQ2589X_ENHIZ_SHIFT;
  
  return BQ25895_OK;
}


int bq2589x_pumpx_enable(int enable)
{
  u8 val;
  int ret;
  
  if (enable)
    val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
  else
    val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_04, BQ2589X_EN_PUMPX_MASK, val);
  
  return ret;
}


int bq2589x_pumpx_increase_volt()
{
  u8 val;
  int ret;
  
  val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK, val);
  
  return ret;
  
}


int bq2589x_pumpx_increase_volt_done()
{
  u8 val;
  int ret;
  
  ret = bq2589x_read_byte(&val, BQ2589X_REG_09);
  if (ret)
    return ret;
  
  if (val & BQ2589X_PUMPX_UP_MASK)
    return BQ25895_ERR;   /* not finished*/
  else
    return BQ25895_OK;   /* pumpx up finished*/
  
}


int bq2589x_pumpx_decrease_volt()
{
  u8 val;
  int ret;
  
  val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_PUMPX_DOWN_MASK, val);
  
  return ret;
  
}


int bq2589x_pumpx_decrease_volt_done()
{
  u8 val;
  int ret;
  
  ret = bq2589x_read_byte(&val, BQ2589X_REG_09);
  if (ret)
    return ret;
  
  if (val & BQ2589X_PUMPX_DOWN_MASK)
    return BQ25895_ERR;   /* not finished*/
  else
    return BQ25895_OK;   /* pumpx down finished*/
  
}


int bq2589x_force_ico()
{
  u8 val;
  int ret;
  
  val = BQ2589X_FORCE_ICO << BQ2589X_FORCE_ICO_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_FORCE_ICO_MASK, val);
  
  return ret;
}


int bq2589x_check_force_ico_done()
{
  u8 val;
  int ret;
  
  ret = bq2589x_read_byte(&val, BQ2589X_REG_14);
  if (ret)
    return ret;
  
  if (val & BQ2589X_ICO_OPTIMIZED_MASK)
    return BQ25895_ERR;  /*finished*/
  else
    return BQ25895_OK;   /* in progress*/
}


int bq2589x_enable_term(bool enable)
{
  u8 val;
  int ret;
  
  if (enable)
    val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
  else
    val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_07, BQ2589X_EN_TERM_MASK, val);
  
  return ret;
}


int bq2589x_enable_auto_dpdm(bool enable)
{
  u8 val;
  int ret;
  
  if (enable)
    val = BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
  else
    val = BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_AUTO_DPDM_EN_MASK, val);
  
  return ret;
  
}


int bq2589x_use_absolute_vindpm(bool enable)
{
  u8 val;
  int ret;
  
  if (enable)
    val = BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT;
  else
    val = BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);
  
  return ret;
  
}


int bq2589x_enable_ico(bool enable)
{
  u8 val;
  int ret;
  
  if (enable)
    val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
  else
    val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;
  
  ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);
  
  return ret;
  
}


int bq2589x_read_idpm_limit()
{
  uint8_t val;
  int curr;
  int ret;
  
  ret = bq2589x_read_byte(&val, BQ2589X_REG_13);
  if (ret) {
    return ret;
  } else{
    curr = BQ2589X_IDPM_LIM_BASE + ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB ;
    return curr;
  }
}


bool bq2589x_is_charge_done()
{
  u8 val;
  
  bq2589x_read_byte(&val, BQ2589X_REG_0B);
  val &= BQ2589X_CHRG_STAT_MASK;
  val >>= BQ2589X_CHRG_STAT_SHIFT;
  
  return (bool)(val == BQ2589X_CHRG_STAT_CHGDONE);
}


int bq2589x_init_device()
{
  int ret;
  
  /*common initialization*/
  
  bq2589x_disable_watchdog_timer();
  
  ret = bq2589x_set_charge_current(2560); //2.5A
  
  return ret;
}


int bq2589x_detect_device(bq2589x_part_no* part_no, int* revision)
{
  u8 data;
  if (bq2589x_read_byte(&data, BQ2589X_REG_14) == 0) 
  {
    *part_no  = (bq2589x_part_no)((data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT);
    *revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
    return BQ25895_OK;
  }
  return BQ25895_ERR;
}

int bq2589x_enable_max_charge(bool enable)
{
  u8 val;
  u8 val1;
  int ret;
  
  if (enable)
  {
    val =  BQ2589X_HVDCP_ENABLE  << BQ2589X_HVDCPEN_SHIFT;
    val1 = BQ2589X_MAXC_ENABLE   << BQ2589X_MAXCEN_SHIFT;
  }
  else
  {
    val =  BQ2589X_HVDCP_DISABLE  << BQ2589X_HVDCPEN_SHIFT;
    val1 = BQ2589X_MAXC_DISABLE << BQ2589X_MAXCEN_SHIFT;
  }
  
  ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_HVDCPEN_MASK, val);
  ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_MAXCEN_MASK, val1);
  
  return ret;
}


