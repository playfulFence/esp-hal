use crate::clock::{PllClock, XtalClock};

const DR_REG_LPPERIPH_BASE: u32 = 0x50120000;
const DR_REG_I2C_ANA_MST_BASE: u32 = DR_REG_LPPERIPH_BASE + 0x4000;

const LPPERI_CLK_EN_REG: u32 = DR_REG_LPPERIPH_BASE + 0x0;
const LPPERI_CK_EN_LP_I2CMST: u32 = 1 << 27;

const I2C_CPLL_OC_DCHGP_LSB: u8 = 4;
const I2C_CPLL_OC_ENB_FCAL_LSB: u8 = 7;
const I2C_CPLL_OC_DLREF_SEL_LSB: u8 = 6;
const I2C_CPLL_OC_DHREF_SEL_LSB: u8 = 4;
const I2C_ANA_MST_CLK160M_REG: u32 = DR_REG_I2C_ANA_MST_BASE + 0x34;
const I2C_ANA_MST_CLK_I2C_MST_SEL_160M: u32 = 1 << 0;

const I2C_CPLL: u8 = 0x67;
const I2C_CPLL_HOSTID: u8 = 0;
const I2C_CPLL_OC_REF_DIV: u8 = 2;
const I2C_CPLL_OC_DIV_7_0: u8 = 3;
const I2C_CPLL_OC_DCUR: u8 = 6;

const I2C_ANA_MST_ANA_CONF1_REG: u32 = DR_REG_I2C_ANA_MST_BASE + 0x1C;
const I2C_ANA_MST_ANA_CONF1: u32 = 0x00FFFFFF;
const I2C_ANA_MST_ANA_CONF1_V: u32 = 0xFFFFFF;
const I2C_ANA_MST_ANA_CONF1_S: u32 = 0;

const I2C_ANA_MST_ANA_CONF2_REG: u32 = DR_REG_I2C_ANA_MST_BASE + 0x20;
const I2C_ANA_MST_ANA_CONF2: u32 = 0x00FFFFFF;
const I2C_ANA_MST_ANA_CONF2_V: u32 = 0xFFFFFF;
const I2C_ANA_MST_ANA_CONF2_S: u32 = 0;

const REGI2C_RTC_SLAVE_ID_V: u8 = 0xFF;
const REGI2C_RTC_SLAVE_ID_S: u8 = 0;
const REGI2C_RTC_ADDR_V: u8 = 0xFF;
const REGI2C_RTC_ADDR_S: u8 = 8;
const REGI2C_RTC_WR_CNTL_V: u8 = 0x1;
const REGI2C_RTC_WR_CNTL_S: u8 = 24;
const REGI2C_RTC_DATA_V: u8 = 0xFF;
const REGI2C_RTC_DATA_S: u8 = 16;
const REGI2C_RTC_BUSY: u32 = 1 << 25;

const I2C_ANA_MST_I2C0_CTRL_REG: u32 = DR_REG_I2C_ANA_MST_BASE + 0x0;

const DR_REG_LPAON_BASE: u32 = 0x50110000;
const DR_REG_PMU_BASE: u32 = DR_REG_LPAON_BASE = 0x5000;
const PMU_IMM_HP_CK_POWER: u32 = DR_REG_PMU_BASE + 0xcc;
const PMU_TIE_HIGH_XPD_CPLL: u32 = 1 << 27;
const PMU_TIE_HIGH_XPD_CPLL_I2C: u32 = 1 << 23;
const PMU_TIE_HIGH_GLOBAL_CPLL_ICG: u32 = 1 << 17;

const DR_REG_HPPERIPH1_BASE: u32 = 0x500C0000;
const DR_REG_HP_SYS_CLKRST_BASE: u32 = DR_REG_HPPERIPH1_BASE + 0x26000;
const HP_SYS_CLKRST_ANA_PLL_CTRL0: u32 = DR_REG_HP_SYS_CLKRST_BASE + 0xbc;
const HP_SYS_CLKRST_REG_CPU_PLL_CAL_STOP: u32 = 1 << 3;

const REGI2C_DIG_REG: u8 = 0x6d;
const REGI2C_CPU_PLL: u8 = 0x67;
const REGI2C_SDIO_PLL: u8 = 0x62;
const REGI2C_BIAS: u8 = 0x6a;
const REGI2C_MSPI: u8 = 0x63;
const REGI2C_SYS_PLL: u8 = 0x66;
const REGI2C_PLLA: u8 = 0x6f;
const REGI2C_SAR_I2C: u8 = 0x69;

const REGI2C_DIG_REG_MST_SEL: u32 = 1 << 10;
const REGI2C_PLL_CPU_MST_SEL: u32 = 1 << 11;
const REGI2C_PLL_SDIO_MST_SEL: u32 = 1 << 6;
const REGI2C_BIAS_MST_SEL: u32 = 1 << 12;
const REGI2C_MSPI_XTAL_MST_SEL: u32 = 1 << 9;
const REGI2C_PLL_SYS_MST_SEL: u32 = 1 << 5;
const REGI2C_PLLA_MST_SEL: u32 = 1 << 8;
const REGI2C_SAR_I2C_MST_SEL: u32 = 1 << 7;

// rtc_clk.c (L125) -> clk_tree_ll.h
pub(crate) fn esp32p4_rtc_cpll_enable() {
    (PMU_IMM_HP_CK_POWER as *mut u32).write_volatile(
        (PMU_IMM_HP_CK_POWER as *mut u32).read_volatile()
            | (PMU_TIE_HIGH_XPD_CPLL | PMU_TIE_HIGH_XPD_CPLL_I2C),
    );

    (PMU_IMM_HP_CK_POWER as *mut u32).write_volatile(
        (PMU_IMM_HP_CK_POWER as *mut u32).read_volatile() | PMU_TIE_HIGH_GLOBAL_CPLL_ICG,
    )
}

// rtc_clk.c (L136)
pub(crate) fn esp32p4_rtc_cpll_configure(_xtal_freq: XtalClock, _cpll_freq: PllClock) {
    // CPLL CALIBRATION START
    (HP_SYS_CLKRST_ANA_PLL_CTRL0_REG as *mut u32).write_volatile(
        (HP_SYS_CLKRST_ANA_PLL_CTRL0_REG as *mut u32).read_volatile()
            | !HP_SYS_CLKRST_REG_CPU_PLL_CAL_STOP,
    );

    // Set configuration
    let oc_div_ref = 0u32;
    let div = 1u32;
    let dcur = 3u32;
    let dchgp = 5u32;
    let enb_fcal = 0u32;

    // Currently only supporting 40MHz XTAL
    assert!(xtal_freq == XtalClock::RtcXtalFreq40M);

    match cpll_freq {
        PllClock::Pll400MHz => {
            div = 6u32;
            dif_ref = 0u32;
        }
        PllClock::Pll360MHz => {
            div = 5u32;
            div_ref = 0u32;
        }
    }

    let i2c_cpll_lref =
        (oc_enb_fcal << I2C_CPLL_OC_ENB_FCAL_LSB) | (dchgp << I2C_CPLL_OC_DCHGP_LSB) | (dif_ref);

    let i2c_cpll_dcur = (1 << I2C_CPLL_OC_DLREF_SEL_LSB) | (3 << I2C_CPLL_OC_DHREF_SEL_LSB) | dcur;

    regi2c_write(
        I2C_CPLL,
        I2C_CPLL_HOSTID,
        I2C_CPLL_OC_REF_DIV,
        i2c_cpll_lref,
    );

    regi2c_write(I2C_CPLL, I2C_CPLL_HOSTID, I2C_CPLL_OC_DIV_7_0, div);

    regi2c_write(I2C_CPLL, I2C_CPLL_HOSTID, I2C_CPLL_OC_DCUR, i2c_cpll_dcur);
}

// esp_rom_regi2c_esp32p4.c (L84)
fn regi2c_enable_block(block: u8) {
    reg_set_bit(LPPERI_CLK_EN_REG, LPPERI_CK_EN_LP_I2CMST);
    set_peri_reg_mask(I2C_ANA_MST_CLK160M_REG, I2C_ANA_MST_CLK_I2C_MST_SEL_160M);

    reg_set_field(
        I2C_ANA_MST_ANA_CONF2_REG,
        I2C_ANA_MST_ANA_CONF2_V,
        I2C_ANA_MST_ANA_CONF2_S,
        0,
    );

    reg_set_field(
        I2C_ANA_MST_ANA_CONF1_REG,
        I2C_ANA_MST_ANA_CONF1_V,
        I2C_ANA_MST_ANA_CONF1_S,
        0,
    );

    match block {
        REGI2C_DIG_REG => {
            reg_set_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_DIG_REG_MST_SEL);
        }
        REGI2C_CPU_PLL => {
            reg_set_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_PLL_CPU_MST_SEL);
        }
        REGI2C_SDIO_PLL => {
            reg_set_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_PLL_SDIO_MST_SEL);
        }
        REGI2C_BIAS => {
            reg_set_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_BIAS_MST_SEL);
        }
        REGI2C_MSPI => {
            reg_set_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_MSPI_XTAL_MST_SEL);
        }
        REGI2C_SYS_PLL => {
            reg_set_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_PLL_SYS_MST_SEL);
        }
        REGI2C_PLLA => {
            reg_set_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_PLLA_MST_SEL);
        }
        REGI2C_SAR_I2C => {
            reg_set_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_SAR_I2C_MST_SEL);
        }
        _ => (),
    }
}

fn regi2c_disable_block(block: u8) {
    match block {
        REGI2C_DIG_REG => {
            reg_clr_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_DIG_REG_MST_SEL);
        }
        REGI2C_CPU_PLL => {
            reg_clr_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_PLL_CPU_MST_SEL);
        }
        REGI2C_SDIO_PLL => {
            reg_clr_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_PLL_SDIO_MST_SEL);
        }
        REGI2C_BIAS => {
            reg_clr_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_BIAS_MST_SEL);
        }
        REGI2C_MSPI => {
            reg_clr_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_MSPI_XTAL_MST_SEL);
        }
        REGI2C_SYS_PLL => {
            reg_clr_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_PLL_SYS_MST_SEL);
        }
        REGI2C_PLLA => {
            reg_clr_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_PLLA_MST_SEL);
        }
        REGI2C_SAR_I2C => {
            reg_clr_bit(I2C_ANA_MST_ANA_CONF2_REG, REGI2C_SAR_I2C_MST_SEL);
        }
        _ => (),
    }
}

pub(crate) fn regi2c_write(block: u8, _host_id: u8, reg_add: u8, data: u8) {
    regi2c_enable_block(block);

    let temp: u32 = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32) << REGI2C_RTC_SLAVE_ID_S as u32)
    | ((reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32)
    | ((0x1 & REGI2C_RTC_WR_CNTL_V as u32) << REGI2C_RTC_WR_CNTL_S as u32) // 0: READ I2C register; 1: Write I2C register;
    | (((data as u32) & REGI2C_RTC_DATA_V as u32) << REGI2C_RTC_DATA_S as u32);
    reg_write(LP_I2C_ANA_MST_I2C0_CTRL_REG, temp);
    while reg_get_bit(I2C_ANA_MST_I2C0_CTRL_REG, REGI2C_RTC_BUSY) != 0 {}

    regi2c_disable_block(block);
}

fn reg_set_field(reg: u32, field_v: u32, field_s: u32, value: u32) {
    (reg as *mut u32).write_volatile(
        ((reg as *mut u32).read_volatile() & !(field_v << field_s))
            | ((value & field_v) << field_s),
    )
}

fn reg_set_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | bit);
    }
}

fn reg_clr_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !bit);
    }
}

fn set_peri_reg_mask(reg: u32, mask: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask);
    }
}
