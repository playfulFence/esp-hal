(function() {var type_impls = {
"esp32c2":[["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-FieldWriter%3C'a,+REG,+WI,+FI%3E\" class=\"impl\"><a class=\"src rightside\" href=\"src/esp32c2/generic.rs.html#322-356\">source</a><a href=\"#impl-FieldWriter%3C'a,+REG,+WI,+FI%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;'a, REG, const WI: u8, FI&gt; <a class=\"type\" href=\"esp32c2/generic/type.FieldWriter.html\" title=\"type esp32c2::generic::FieldWriter\">FieldWriter</a>&lt;'a, REG, WI, FI&gt;<div class=\"where\">where\n    REG: <a class=\"trait\" href=\"esp32c2/generic/trait.Writable.html\" title=\"trait esp32c2::generic::Writable\">Writable</a> + <a class=\"trait\" href=\"esp32c2/generic/trait.RegisterSpec.html\" title=\"trait esp32c2::generic::RegisterSpec\">RegisterSpec</a>,\n    FI: <a class=\"trait\" href=\"esp32c2/generic/trait.FieldSpec.html\" title=\"trait esp32c2::generic::FieldSpec\">FieldSpec</a>,\n    REG::<a class=\"associatedtype\" href=\"esp32c2/generic/trait.RegisterSpec.html#associatedtype.Ux\" title=\"type esp32c2::generic::RegisterSpec::Ux\">Ux</a>: From&lt;FI::<a class=\"associatedtype\" href=\"esp32c2/generic/trait.FieldSpec.html#associatedtype.Ux\" title=\"type esp32c2::generic::FieldSpec::Ux\">Ux</a>&gt;,</div></h3></section></summary><div class=\"impl-items\"><details class=\"toggle\" open><summary><section id=\"associatedconstant.WIDTH\" class=\"associatedconstant\"><a class=\"src rightside\" href=\"src/esp32c2/generic.rs.html#329\">source</a><h4 class=\"code-header\">pub const <a href=\"esp32c2/generic/type.FieldWriter.html#associatedconstant.WIDTH\" class=\"constant\">WIDTH</a>: u8 = WI</h4></section></summary><div class=\"docblock\"><p>Field width</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.width\" class=\"method\"><a class=\"src rightside\" href=\"src/esp32c2/generic.rs.html#332-334\">source</a><h4 class=\"code-header\">pub const fn <a href=\"esp32c2/generic/type.FieldWriter.html#tymethod.width\" class=\"fn\">width</a>(&amp;self) -&gt; u8</h4></section></summary><div class=\"docblock\"><p>Field width</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.offset\" class=\"method\"><a class=\"src rightside\" href=\"src/esp32c2/generic.rs.html#337-339\">source</a><h4 class=\"code-header\">pub const fn <a href=\"esp32c2/generic/type.FieldWriter.html#tymethod.offset\" class=\"fn\">offset</a>(&amp;self) -&gt; u8</h4></section></summary><div class=\"docblock\"><p>Field offset</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.bits\" class=\"method\"><a class=\"src rightside\" href=\"src/esp32c2/generic.rs.html#346-350\">source</a><h4 class=\"code-header\">pub unsafe fn <a href=\"esp32c2/generic/type.FieldWriter.html#tymethod.bits\" class=\"fn\">bits</a>(self, value: FI::<a class=\"associatedtype\" href=\"esp32c2/generic/trait.FieldSpec.html#associatedtype.Ux\" title=\"type esp32c2::generic::FieldSpec::Ux\">Ux</a>) -&gt; &amp;'a mut <a class=\"type\" href=\"esp32c2/generic/type.W.html\" title=\"type esp32c2::generic::W\">W</a>&lt;REG&gt;</h4></section></summary><div class=\"docblock\"><p>Writes raw bits to the field</p>\n<h5 id=\"safety\"><a class=\"doc-anchor\" href=\"#safety\">§</a>Safety</h5>\n<p>Passing incorrect value can cause undefined behaviour. See reference manual</p>\n</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.variant\" class=\"method\"><a class=\"src rightside\" href=\"src/esp32c2/generic.rs.html#353-355\">source</a><h4 class=\"code-header\">pub fn <a href=\"esp32c2/generic/type.FieldWriter.html#tymethod.variant\" class=\"fn\">variant</a>(self, variant: FI) -&gt; &amp;'a mut <a class=\"type\" href=\"esp32c2/generic/type.W.html\" title=\"type esp32c2::generic::W\">W</a>&lt;REG&gt;</h4></section></summary><div class=\"docblock\"><p>Writes <code>variant</code> to the field</p>\n</div></details></div></details>",0,"esp32c2::apb_ctrl::sysclk_conf::PRE_DIV_CNT_W","esp32c2::apb_ctrl::tick_conf::XTAL_TICK_NUM_W","esp32c2::apb_ctrl::tick_conf::CK8M_TICK_NUM_W","esp32c2::apb_ctrl::wifi_bb_cfg::WIFI_BB_CFG_W","esp32c2::apb_ctrl::wifi_bb_cfg_2::WIFI_BB_CFG_2_W","esp32c2::apb_ctrl::wifi_clk_en::WIFI_CLK_EN_W","esp32c2::apb_ctrl::wifi_rst_en::WIFI_RST_W","esp32c2::apb_ctrl::host_inf_sel::PERI_IO_SWAP_W","esp32c2::apb_ctrl::flash_ace0_attr::FLASH_ACE0_ATTR_W","esp32c2::apb_ctrl::flash_ace1_attr::FLASH_ACE1_ATTR_W","esp32c2::apb_ctrl::flash_ace2_attr::FLASH_ACE2_ATTR_W","esp32c2::apb_ctrl::flash_ace3_attr::FLASH_ACE3_ATTR_W","esp32c2::apb_ctrl::flash_ace0_addr::S_W","esp32c2::apb_ctrl::flash_ace1_addr::S_W","esp32c2::apb_ctrl::flash_ace2_addr::S_W","esp32c2::apb_ctrl::flash_ace3_addr::S_W","esp32c2::apb_ctrl::flash_ace0_size::FLASH_ACE0_SIZE_W","esp32c2::apb_ctrl::flash_ace1_size::FLASH_ACE1_SIZE_W","esp32c2::apb_ctrl::flash_ace2_size::FLASH_ACE2_SIZE_W","esp32c2::apb_ctrl::flash_ace3_size::FLASH_ACE3_SIZE_W","esp32c2::apb_ctrl::redcy_sig0::REDCY_SIG0_W","esp32c2::apb_ctrl::redcy_sig1::REDCY_SIG1_W","esp32c2::apb_ctrl::retention_ctrl::RETENTION_LINK_ADDR_W","esp32c2::apb_ctrl::clkgate_force_on::ROM_CLKGATE_FORCE_ON_W","esp32c2::apb_ctrl::clkgate_force_on::SRAM_CLKGATE_FORCE_ON_W","esp32c2::apb_ctrl::mem_power_down::ROM_POWER_DOWN_W","esp32c2::apb_ctrl::mem_power_down::SRAM_POWER_DOWN_W","esp32c2::apb_ctrl::mem_power_up::ROM_POWER_UP_W","esp32c2::apb_ctrl::mem_power_up::SRAM_POWER_UP_W","esp32c2::apb_ctrl::peri_backup_config::PERI_BACKUP_BURST_LIMIT_W","esp32c2::apb_ctrl::peri_backup_config::PERI_BACKUP_TOUT_THRES_W","esp32c2::apb_ctrl::peri_backup_config::PERI_BACKUP_SIZE_W","esp32c2::apb_ctrl::peri_backup_apb_addr::BACKUP_APB_START_ADDR_W","esp32c2::apb_ctrl::peri_backup_mem_addr::BACKUP_MEM_START_ADDR_W","esp32c2::apb_ctrl::date::DATE_W","esp32c2::apb_saradc::ctrl::SARADC_SAR_CLK_DIV_W","esp32c2::apb_saradc::ctrl::SARADC_SAR_PATT_LEN_W","esp32c2::apb_saradc::ctrl::SARADC_XPD_SAR_FORCE_W","esp32c2::apb_saradc::ctrl::SARADC_WAIT_ARB_CYCLE_W","esp32c2::apb_saradc::ctrl2::SARADC_MAX_MEAS_NUM_W","esp32c2::apb_saradc::ctrl2::SARADC_TIMER_TARGET_W","esp32c2::apb_saradc::filter_ctrl1::FILTER_FACTOR1_W","esp32c2::apb_saradc::filter_ctrl1::FILTER_FACTOR0_W","esp32c2::apb_saradc::fsm_wait::SARADC_XPD_WAIT_W","esp32c2::apb_saradc::fsm_wait::SARADC_RSTB_WAIT_W","esp32c2::apb_saradc::fsm_wait::SARADC_STANDBY_WAIT_W","esp32c2::apb_saradc::sar_patt_tab1::SARADC_SAR_PATT_TAB1_W","esp32c2::apb_saradc::sar_patt_tab2::SARADC_SAR_PATT_TAB2_W","esp32c2::apb_saradc::onetime_sample::SARADC_ONETIME_ATTEN_W","esp32c2::apb_saradc::onetime_sample::SARADC_ONETIME_CHANNEL_W","esp32c2::apb_saradc::apb_adc_arb_ctrl::ADC_ARB_APB_PRIORITY_W","esp32c2::apb_saradc::apb_adc_arb_ctrl::ADC_ARB_RTC_PRIORITY_W","esp32c2::apb_saradc::apb_adc_arb_ctrl::ADC_ARB_WIFI_PRIORITY_W","esp32c2::apb_saradc::filter_ctrl0::FILTER_CHANNEL1_W","esp32c2::apb_saradc::filter_ctrl0::FILTER_CHANNEL0_W","esp32c2::apb_saradc::thres0_ctrl::THRES0_CHANNEL_W","esp32c2::apb_saradc::thres0_ctrl::THRES0_HIGH_W","esp32c2::apb_saradc::thres0_ctrl::THRES0_LOW_W","esp32c2::apb_saradc::thres1_ctrl::THRES1_CHANNEL_W","esp32c2::apb_saradc::thres1_ctrl::THRES1_HIGH_W","esp32c2::apb_saradc::thres1_ctrl::THRES1_LOW_W","esp32c2::apb_saradc::dma_conf::APB_ADC_EOF_NUM_W","esp32c2::apb_saradc::apb_adc_clkm_conf::REG_CLKM_DIV_NUM_W","esp32c2::apb_saradc::apb_adc_clkm_conf::REG_CLKM_DIV_B_W","esp32c2::apb_saradc::apb_adc_clkm_conf::REG_CLKM_DIV_A_W","esp32c2::apb_saradc::apb_adc_clkm_conf::REG_CLK_SEL_W","esp32c2::apb_saradc::apb_tsens_ctrl::REG_TSENS_CLK_DIV_W","esp32c2::apb_saradc::apb_tsens_ctrl2::REG_TSENS_XPD_WAIT_W","esp32c2::apb_saradc::apb_tsens_ctrl2::REG_TSENS_XPD_FORCE_W","esp32c2::apb_saradc::cali::CFG_W","esp32c2::apb_saradc::apb_ctrl_date::DATE_W","esp32c2::assist_debug::core_0_sp_min::CORE_0_SP_MIN_W","esp32c2::assist_debug::core_0_sp_max::CORE_0_SP_MAX_W","esp32c2::assist_debug::date::DATE_W","esp32c2::dma::ahb_test::AHB_TESTMODE_W","esp32c2::dma::ahb_test::AHB_TESTADDR_W","esp32c2::dma::date::DATE_W","esp32c2::dma::in_link_ch::INLINK_ADDR_W","esp32c2::dma::in_pri_ch::RX_PRI_W","esp32c2::dma::in_peri_sel_ch::PERI_IN_SEL_W","esp32c2::dma::out_push_ch0::OUTFIFO_WDATA_W","esp32c2::dma::out_link_ch::OUTLINK_ADDR_W","esp32c2::dma::out_pri_ch::TX_PRI_W","esp32c2::dma::out_peri_sel_ch::PERI_OUT_SEL_W","esp32c2::ecc::mult_conf::WORK_MODE_W","esp32c2::ecc::mult_date::DATE_W","esp32c2::efuse::pgm_data0::PGM_DATA_0_W","esp32c2::efuse::pgm_data1::PGM_DATA_1_W","esp32c2::efuse::pgm_data2::PGM_DATA_2_W","esp32c2::efuse::pgm_data3::PGM_DATA_3_W","esp32c2::efuse::pgm_data4::PGM_DATA_4_W","esp32c2::efuse::pgm_data5::PGM_DATA_5_W","esp32c2::efuse::pgm_data6::PGM_DATA_6_W","esp32c2::efuse::pgm_data7::PGM_DATA_7_W","esp32c2::efuse::pgm_check_value0::PGM_RS_DATA_0_W","esp32c2::efuse::pgm_check_value1::PGM_RS_DATA_1_W","esp32c2::efuse::pgm_check_value2::PGM_RS_DATA_2_W","esp32c2::efuse::conf::OP_CODE_W","esp32c2::efuse::cmd::BLK_NUM_W","esp32c2::efuse::dac_conf::DAC_CLK_DIV_W","esp32c2::efuse::dac_conf::DAC_NUM_W","esp32c2::efuse::rd_tim_conf::THR_A_W","esp32c2::efuse::rd_tim_conf::TRD_W","esp32c2::efuse::rd_tim_conf::TSUR_A_W","esp32c2::efuse::rd_tim_conf::READ_INIT_NUM_W","esp32c2::efuse::wr_tim_conf0::THP_A_W","esp32c2::efuse::wr_tim_conf0::TPGM_INACTIVE_W","esp32c2::efuse::wr_tim_conf0::TPGM_W","esp32c2::efuse::wr_tim_conf1::TSUP_A_W","esp32c2::efuse::wr_tim_conf1::PWR_ON_NUM_W","esp32c2::efuse::wr_tim_conf2::PWR_OFF_NUM_W","esp32c2::efuse::date::DATE_W","esp32c2::extmem::icache_sync_addr::ICACHE_SYNC_ADDR_W","esp32c2::extmem::icache_sync_size::ICACHE_SYNC_SIZE_W","esp32c2::extmem::ibus_to_flash_start_vaddr::IBUS_TO_FLASH_START_VADDR_W","esp32c2::extmem::ibus_to_flash_end_vaddr::IBUS_TO_FLASH_END_VADDR_W","esp32c2::extmem::dbus_to_flash_start_vaddr::DBUS_TO_FLASH_START_VADDR_W","esp32c2::extmem::dbus_to_flash_end_vaddr::DBUS_TO_FLASH_END_VADDR_W","esp32c2::extmem::cache_mmu_owner::CACHE_MMU_OWNER_W","esp32c2::extmem::cache_conf_misc::CACHE_MMU_PAGE_SIZE_W","esp32c2::extmem::reg_date::DATE_W","esp32c2::gpio::bt_select::BT_SEL_W","esp32c2::gpio::out::DATA_ORIG_W","esp32c2::gpio::out_w1ts::OUT_W1TS_W","esp32c2::gpio::out_w1tc::OUT_W1TC_W","esp32c2::gpio::sdio_select::SDIO_SEL_W","esp32c2::gpio::enable::DATA_W","esp32c2::gpio::enable_w1ts::ENABLE_W1TS_W","esp32c2::gpio::enable_w1tc::ENABLE_W1TC_W","esp32c2::gpio::status::INTERRUPT_W","esp32c2::gpio::status_w1ts::STATUS_W1TS_W","esp32c2::gpio::status_w1tc::STATUS_W1TC_W","esp32c2::gpio::pin::SYNC2_BYPASS_W","esp32c2::gpio::pin::SYNC1_BYPASS_W","esp32c2::gpio::pin::INT_TYPE_W","esp32c2::gpio::pin::CONFIG_W","esp32c2::gpio::pin::INT_ENA_W","esp32c2::gpio::func_in_sel_cfg::IN_SEL_W","esp32c2::gpio::func_out_sel_cfg::OUT_SEL_W","esp32c2::gpio::reg_date::REG_DATE_W","esp32c2::i2c0::scl_low_period::SCL_LOW_PERIOD_W","esp32c2::i2c0::to::TIME_OUT_VALUE_W","esp32c2::i2c0::fifo_conf::RXFIFO_WM_THRHD_W","esp32c2::i2c0::fifo_conf::TXFIFO_WM_THRHD_W","esp32c2::i2c0::data::FIFO_RDATA_W","esp32c2::i2c0::sda_hold::TIME_W","esp32c2::i2c0::sda_sample::TIME_W","esp32c2::i2c0::scl_high_period::SCL_HIGH_PERIOD_W","esp32c2::i2c0::scl_high_period::SCL_WAIT_HIGH_PERIOD_W","esp32c2::i2c0::scl_start_hold::TIME_W","esp32c2::i2c0::scl_rstart_setup::TIME_W","esp32c2::i2c0::scl_stop_hold::TIME_W","esp32c2::i2c0::scl_stop_setup::TIME_W","esp32c2::i2c0::filter_cfg::SCL_FILTER_THRES_W","esp32c2::i2c0::filter_cfg::SDA_FILTER_THRES_W","esp32c2::i2c0::clk_conf::SCLK_DIV_NUM_W","esp32c2::i2c0::clk_conf::SCLK_DIV_A_W","esp32c2::i2c0::clk_conf::SCLK_DIV_B_W","esp32c2::i2c0::comd::COMMAND_W","esp32c2::i2c0::scl_st_time_out::SCL_ST_TO_I2C_W","esp32c2::i2c0::scl_main_st_time_out::SCL_MAIN_ST_TO_I2C_W","esp32c2::i2c0::scl_sp_conf::SCL_RST_SLV_NUM_W","esp32c2::i2c0::date::DATE_W","esp32c2::interrupt_core0::mac_intr_map::WIFI_MAC_INT_MAP_W","esp32c2::interrupt_core0::wifi_mac_nmi_map::WIFI_MAC_NMI_MAP_W","esp32c2::interrupt_core0::wifi_pwr_int_map::WIFI_PWR_INT_MAP_W","esp32c2::interrupt_core0::wifi_bb_int_map::WIFI_BB_INT_MAP_W","esp32c2::interrupt_core0::bt_mac_int_map::BT_MAC_INT_MAP_W","esp32c2::interrupt_core0::bt_bb_int_map::BT_BB_INT_MAP_W","esp32c2::interrupt_core0::bt_bb_nmi_map::BT_BB_NMI_MAP_W","esp32c2::interrupt_core0::lp_timer_int_map::LP_TIMER_INT_MAP_W","esp32c2::interrupt_core0::coex_int_map::COEX_INT_MAP_W","esp32c2::interrupt_core0::ble_timer_int_map::BLE_TIMER_INT_MAP_W","esp32c2::interrupt_core0::ble_sec_int_map::BLE_SEC_INT_MAP_W","esp32c2::interrupt_core0::i2c_mst_int_map::I2C_MST_INT_MAP_W","esp32c2::interrupt_core0::apb_ctrl_intr_map::APB_CTRL_INTR_MAP_W","esp32c2::interrupt_core0::gpio_interrupt_pro_map::GPIO_INTERRUPT_PRO_MAP_W","esp32c2::interrupt_core0::gpio_interrupt_pro_nmi_map::GPIO_INTERRUPT_PRO_NMI_MAP_W","esp32c2::interrupt_core0::spi_intr_1_map::SPI_INTR_1_MAP_W","esp32c2::interrupt_core0::spi_intr_2_map::SPI_INTR_2_MAP_W","esp32c2::interrupt_core0::uart_intr_map::UART_INTR_MAP_W","esp32c2::interrupt_core0::uart1_intr_map::UART1_INTR_MAP_W","esp32c2::interrupt_core0::ledc_int_map::LEDC_INT_MAP_W","esp32c2::interrupt_core0::efuse_int_map::EFUSE_INT_MAP_W","esp32c2::interrupt_core0::rtc_core_intr_map::RTC_CORE_INTR_MAP_W","esp32c2::interrupt_core0::i2c_ext0_intr_map::I2C_EXT0_INTR_MAP_W","esp32c2::interrupt_core0::tg_t0_int_map::TG_T0_INT_MAP_W","esp32c2::interrupt_core0::tg_wdt_int_map::TG_WDT_INT_MAP_W","esp32c2::interrupt_core0::cache_ia_int_map::CACHE_IA_INT_MAP_W","esp32c2::interrupt_core0::systimer_target0_int_map::SYSTIMER_TARGET0_INT_MAP_W","esp32c2::interrupt_core0::systimer_target1_int_map::SYSTIMER_TARGET1_INT_MAP_W","esp32c2::interrupt_core0::systimer_target2_int_map::SYSTIMER_TARGET2_INT_MAP_W","esp32c2::interrupt_core0::spi_mem_reject_intr_map::SPI_MEM_REJECT_INTR_MAP_W","esp32c2::interrupt_core0::icache_preload_int_map::ICACHE_PRELOAD_INT_MAP_W","esp32c2::interrupt_core0::icache_sync_int_map::ICACHE_SYNC_INT_MAP_W","esp32c2::interrupt_core0::apb_adc_int_map::APB_ADC_INT_MAP_W","esp32c2::interrupt_core0::dma_ch0_int_map::DMA_CH0_INT_MAP_W","esp32c2::interrupt_core0::sha_int_map::SHA_INT_MAP_W","esp32c2::interrupt_core0::ecc_int_map::ECC_INT_MAP_W","esp32c2::interrupt_core0::cpu_intr_from_cpu_0_map::CPU_INTR_FROM_CPU_0_MAP_W","esp32c2::interrupt_core0::cpu_intr_from_cpu_1_map::CPU_INTR_FROM_CPU_1_MAP_W","esp32c2::interrupt_core0::cpu_intr_from_cpu_2_map::CPU_INTR_FROM_CPU_2_MAP_W","esp32c2::interrupt_core0::cpu_intr_from_cpu_3_map::CPU_INTR_FROM_CPU_3_MAP_W","esp32c2::interrupt_core0::assist_debug_intr_map::ASSIST_DEBUG_INTR_MAP_W","esp32c2::interrupt_core0::core_0_pif_pms_monitor_violate_size_intr_map::CORE_0_PIF_PMS_MONITOR_VIOLATE_SIZE_INTR_MAP_W","esp32c2::interrupt_core0::cache_core0_acs_int_map::CACHE_CORE0_ACS_INT_MAP_W","esp32c2::interrupt_core0::cpu_int_enable::CPU_INT_ENABLE_W","esp32c2::interrupt_core0::cpu_int_type::CPU_INT_TYPE_W","esp32c2::interrupt_core0::cpu_int_clear::CPU_INT_CLEAR_W","esp32c2::interrupt_core0::cpu_int_pri_0::CPU_PRI_0_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_1::CPU_PRI_1_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_2::CPU_PRI_2_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_3::CPU_PRI_3_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_4::CPU_PRI_4_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_5::CPU_PRI_5_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_6::CPU_PRI_6_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_7::CPU_PRI_7_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_8::CPU_PRI_8_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_9::CPU_PRI_9_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_10::CPU_PRI_10_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_11::CPU_PRI_11_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_12::CPU_PRI_12_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_13::CPU_PRI_13_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_14::CPU_PRI_14_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_15::CPU_PRI_15_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_16::CPU_PRI_16_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_17::CPU_PRI_17_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_18::CPU_PRI_18_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_19::CPU_PRI_19_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_20::CPU_PRI_20_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_21::CPU_PRI_21_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_22::CPU_PRI_22_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_23::CPU_PRI_23_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_24::CPU_PRI_24_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_25::CPU_PRI_25_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_26::CPU_PRI_26_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_27::CPU_PRI_27_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_28::CPU_PRI_28_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_29::CPU_PRI_29_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_30::CPU_PRI_30_MAP_W","esp32c2::interrupt_core0::cpu_int_pri_31::CPU_PRI_31_MAP_W","esp32c2::interrupt_core0::cpu_int_thresh::CPU_INT_THRESH_W","esp32c2::interrupt_core0::interrupt_reg_date::INTERRUPT_REG_DATE_W","esp32c2::io_mux::pin_ctrl::CLK_OUT1_W","esp32c2::io_mux::pin_ctrl::CLK_OUT2_W","esp32c2::io_mux::pin_ctrl::CLK_OUT3_W","esp32c2::io_mux::gpio::FUN_DRV_W","esp32c2::io_mux::gpio::MCU_SEL_W","esp32c2::io_mux::date::REG_DATE_W","esp32c2::ledc::ch_conf0::TIMER_SEL_W","esp32c2::ledc::ch_conf0::OVF_NUM_W","esp32c2::ledc::ch_hpoint::HPOINT_W","esp32c2::ledc::ch_duty::DUTY_W","esp32c2::ledc::ch_conf1::DUTY_SCALE_W","esp32c2::ledc::ch_conf1::DUTY_CYCLE_W","esp32c2::ledc::ch_conf1::DUTY_NUM_W","esp32c2::ledc::timer_conf::DUTY_RES_W","esp32c2::ledc::timer_conf::CLK_DIV_W","esp32c2::ledc::conf::APB_CLK_SEL_W","esp32c2::ledc::date::LEDC_DATE_W","esp32c2::modem_clkrst::modem_lp_timer_conf::LP_TIMER_CLK_DIV_NUM_W","esp32c2::modem_clkrst::coex_lp_clk_conf::COEX_LPCLK_DIV_NUM_W","esp32c2::modem_clkrst::date::DATE_W","esp32c2::rtc_cntl::options0::SW_STALL_PROCPU_C0_W","esp32c2::rtc_cntl::options0::XTL_EN_WAIT_W","esp32c2::rtc_cntl::options0::XTL_EXT_CTR_SEL_W","esp32c2::rtc_cntl::slp_timer0::SLP_VAL_LO_W","esp32c2::rtc_cntl::slp_timer1::SLP_VAL_HI_W","esp32c2::rtc_cntl::time_low0::TIMER_VALUE0_LOW_W","esp32c2::rtc_cntl::time_high0::TIMER_VALUE0_HIGH_W","esp32c2::rtc_cntl::timer1::CPU_STALL_WAIT_W","esp32c2::rtc_cntl::timer1::CK8M_WAIT_W","esp32c2::rtc_cntl::timer1::XTL_BUF_WAIT_W","esp32c2::rtc_cntl::timer1::PLL_BUF_WAIT_W","esp32c2::rtc_cntl::timer2::MIN_TIME_CK8M_OFF_W","esp32c2::rtc_cntl::timer4::DG_WRAP_WAIT_TIMER_W","esp32c2::rtc_cntl::timer4::DG_WRAP_POWERUP_TIMER_W","esp32c2::rtc_cntl::timer5::MIN_SLP_VAL_W","esp32c2::rtc_cntl::reset_state::RESET_CAUSE_PROCPU_W","esp32c2::rtc_cntl::wakeup_state::WAKEUP_ENA_W","esp32c2::rtc_cntl::store0::SCRATCH0_W","esp32c2::rtc_cntl::store1::SCRATCH1_W","esp32c2::rtc_cntl::store2::SCRATCH2_W","esp32c2::rtc_cntl::store3::SCRATCH3_W","esp32c2::rtc_cntl::slp_reject_conf::SLEEP_REJECT_ENA_W","esp32c2::rtc_cntl::cpu_period_conf::CPUPERIOD_SEL_W","esp32c2::rtc_cntl::clk_conf::CK8M_DIV_W","esp32c2::rtc_cntl::clk_conf::CK8M_DIV_SEL_W","esp32c2::rtc_cntl::clk_conf::CK8M_DFREQ_W","esp32c2::rtc_cntl::clk_conf::ANA_CLK_RTC_SEL_W","esp32c2::rtc_cntl::slow_clk_conf::ANA_CLK_DIV_W","esp32c2::rtc_cntl::bias_conf::DG_VDD_DRV_B_SLP_W","esp32c2::rtc_cntl::bias_conf::DBG_ATTEN_DEEP_SLP_W","esp32c2::rtc_cntl::bias_conf::DBG_ATTEN_MONITOR_W","esp32c2::rtc_cntl::bias_conf::DBG_ATTEN_ACTIVE_W","esp32c2::rtc_cntl::rtc_cntl::SCK_DCAP_W","esp32c2::rtc_cntl::dig_pwc::VDD_SPI_PWR_DRV_W","esp32c2::rtc_cntl::wdtconfig0::WDT_CHIP_RESET_WIDTH_W","esp32c2::rtc_cntl::wdtconfig0::WDT_SYS_RESET_LENGTH_W","esp32c2::rtc_cntl::wdtconfig0::WDT_CPU_RESET_LENGTH_W","esp32c2::rtc_cntl::wdtconfig0::WDT_STG3_W","esp32c2::rtc_cntl::wdtconfig0::WDT_STG2_W","esp32c2::rtc_cntl::wdtconfig0::WDT_STG1_W","esp32c2::rtc_cntl::wdtconfig0::WDT_STG0_W","esp32c2::rtc_cntl::wdtconfig1::WDT_STG0_HOLD_W","esp32c2::rtc_cntl::wdtconfig2::WDT_STG1_HOLD_W","esp32c2::rtc_cntl::wdtconfig3::WDT_STG2_HOLD_W","esp32c2::rtc_cntl::wdtconfig4::WDT_STG3_HOLD_W","esp32c2::rtc_cntl::wdtwprotect::WDT_WKEY_W","esp32c2::rtc_cntl::swd_conf::SWD_SIGNAL_WIDTH_W","esp32c2::rtc_cntl::swd_wprotect::SWD_WKEY_W","esp32c2::rtc_cntl::sw_cpu_stall::SW_STALL_PROCPU_C1_W","esp32c2::rtc_cntl::store4::SCRATCH4_W","esp32c2::rtc_cntl::store5::SCRATCH5_W","esp32c2::rtc_cntl::store6::SCRATCH6_W","esp32c2::rtc_cntl::store7::SCRATCH7_W","esp32c2::rtc_cntl::low_power_st::MAIN_STATE_W","esp32c2::rtc_cntl::diag0::LOW_POWER_DIAG1_W","esp32c2::rtc_cntl::dig_pad_hold::DIG_PAD_HOLD_W","esp32c2::rtc_cntl::brown_out::BROWN_OUT_INT_WAIT_W","esp32c2::rtc_cntl::brown_out::BROWN_OUT_RST_WAIT_W","esp32c2::rtc_cntl::time_low1::TIMER_VALUE1_LOW_W","esp32c2::rtc_cntl::time_high1::TIMER_VALUE1_HIGH_W","esp32c2::rtc_cntl::slp_reject_cause::REJECT_CAUSE_W","esp32c2::rtc_cntl::slp_wakeup_cause::WAKEUP_CAUSE_W","esp32c2::rtc_cntl::ulp_cp_timer_1::ULP_CP_TIMER_SLP_CYCLE_W","esp32c2::rtc_cntl::cntl_retention_ctrl::RETENTION_DONE_WAIT_W","esp32c2::rtc_cntl::cntl_retention_ctrl::RETENTION_CLKOFF_WAIT_W","esp32c2::rtc_cntl::cntl_retention_ctrl::RETENTION_WAIT_W","esp32c2::rtc_cntl::fib_sel::FIB_SEL_W","esp32c2::rtc_cntl::cntl_gpio_wakeup::GPIO_WAKEUP_STATUS_W","esp32c2::rtc_cntl::cntl_gpio_wakeup::GPIO_PIN5_INT_TYPE_W","esp32c2::rtc_cntl::cntl_gpio_wakeup::GPIO_PIN4_INT_TYPE_W","esp32c2::rtc_cntl::cntl_gpio_wakeup::GPIO_PIN3_INT_TYPE_W","esp32c2::rtc_cntl::cntl_gpio_wakeup::GPIO_PIN2_INT_TYPE_W","esp32c2::rtc_cntl::cntl_gpio_wakeup::GPIO_PIN1_INT_TYPE_W","esp32c2::rtc_cntl::cntl_gpio_wakeup::GPIO_PIN0_INT_TYPE_W","esp32c2::rtc_cntl::cntl_dbg_sel::DEBUG_BIT_SEL_W","esp32c2::rtc_cntl::cntl_dbg_sel::DEBUG_SEL0_W","esp32c2::rtc_cntl::cntl_dbg_sel::DEBUG_SEL1_W","esp32c2::rtc_cntl::cntl_dbg_sel::DEBUG_SEL2_W","esp32c2::rtc_cntl::cntl_dbg_sel::DEBUG_SEL3_W","esp32c2::rtc_cntl::cntl_dbg_sel::DEBUG_SEL4_W","esp32c2::rtc_cntl::cntl_dbg_map::GPIO_PIN5_FUN_SEL_W","esp32c2::rtc_cntl::cntl_dbg_map::GPIO_PIN4_FUN_SEL_W","esp32c2::rtc_cntl::cntl_dbg_map::GPIO_PIN3_FUN_SEL_W","esp32c2::rtc_cntl::cntl_dbg_map::GPIO_PIN2_FUN_SEL_W","esp32c2::rtc_cntl::cntl_dbg_map::GPIO_PIN1_FUN_SEL_W","esp32c2::rtc_cntl::cntl_dbg_map::GPIO_PIN0_FUN_SEL_W","esp32c2::rtc_cntl::cntl_sensor_ctrl::SAR2_PWDET_CCT_W","esp32c2::rtc_cntl::cntl_sensor_ctrl::FORCE_XPD_SAR_W","esp32c2::rtc_cntl::cntl_dbg_sar_sel::SAR_DEBUG_SEL_W","esp32c2::rtc_cntl::cntl_date::CNTL_DATE_W","esp32c2::sensitive::rom_table::ROM_TABLE_W","esp32c2::sensitive::internal_sram_usage_1::INTERNAL_SRAM_USAGE_CPU_SRAM_W","esp32c2::sensitive::internal_sram_usage_3::INTERNAL_SRAM_USAGE_MAC_DUMP_SRAM_W","esp32c2::sensitive::sensitive_reg_date::SENSITIVE_REG_DATE_W","esp32c2::sha::mode::MODE_W","esp32c2::sha::t_string::T_STRING_W","esp32c2::sha::t_length::T_LENGTH_W","esp32c2::sha::dma_block_num::DMA_BLOCK_NUM_W","esp32c2::sha::start::START_W","esp32c2::sha::continue_::CONTINUE_W","esp32c2::sha::date::DATE_W","esp32c2::spi0::ctrl1::CLK_MODE_W","esp32c2::spi0::ctrl2::CS_SETUP_TIME_W","esp32c2::spi0::ctrl2::CS_HOLD_TIME_W","esp32c2::spi0::ctrl2::CS_HOLD_DELAY_W","esp32c2::spi0::clock::CLKCNT_L_W","esp32c2::spi0::clock::CLKCNT_H_W","esp32c2::spi0::clock::CLKCNT_N_W","esp32c2::spi0::user1::USR_DUMMY_CYCLELEN_W","esp32c2::spi0::user1::USR_ADDR_BITLEN_W","esp32c2::spi0::user2::USR_COMMAND_VALUE_W","esp32c2::spi0::user2::USR_COMMAND_BITLEN_W","esp32c2::spi0::rd_status::WB_MODE_W","esp32c2::spi0::fsm::CSPI_LOCK_DELAY_TIME_W","esp32c2::spi0::core_clk_sel::SPI01_CLK_SEL_W","esp32c2::spi0::date::DATE_W","esp32c2::spi1::addr::USR_ADDR_VALUE_W","esp32c2::spi1::ctrl1::CLK_MODE_W","esp32c2::spi1::ctrl1::CS_HOLD_DLY_RES_W","esp32c2::spi1::clock::CLKCNT_L_W","esp32c2::spi1::clock::CLKCNT_H_W","esp32c2::spi1::clock::CLKCNT_N_W","esp32c2::spi1::user1::USR_DUMMY_CYCLELEN_W","esp32c2::spi1::user1::USR_ADDR_BITLEN_W","esp32c2::spi1::user2::USR_COMMAND_VALUE_W","esp32c2::spi1::user2::USR_COMMAND_BITLEN_W","esp32c2::spi1::mosi_dlen::USR_MOSI_DBITLEN_W","esp32c2::spi1::miso_dlen::USR_MISO_DBITLEN_W","esp32c2::spi1::rd_status::STATUS_W","esp32c2::spi1::rd_status::WB_MODE_W","esp32c2::spi1::w0::BUF0_W","esp32c2::spi1::w1::BUF1_W","esp32c2::spi1::w2::BUF2_W","esp32c2::spi1::w3::BUF3_W","esp32c2::spi1::w4::BUF4_W","esp32c2::spi1::w5::BUF5_W","esp32c2::spi1::w6::BUF6_W","esp32c2::spi1::w7::BUF7_W","esp32c2::spi1::w8::BUF8_W","esp32c2::spi1::w9::BUF9_W","esp32c2::spi1::w10::BUF10_W","esp32c2::spi1::w11::BUF11_W","esp32c2::spi1::w12::BUF12_W","esp32c2::spi1::w13::BUF13_W","esp32c2::spi1::w14::BUF14_W","esp32c2::spi1::w15::BUF15_W","esp32c2::spi1::flash_waiti_ctrl::WAITI_CMD_W","esp32c2::spi1::flash_waiti_ctrl::WAITI_DUMMY_CYCLELEN_W","esp32c2::spi1::flash_sus_ctrl::PESR_END_MSK_W","esp32c2::spi1::flash_sus_ctrl::SUS_TIMEOUT_CNT_W","esp32c2::spi1::flash_sus_cmd::FLASH_PER_COMMAND_W","esp32c2::spi1::flash_sus_cmd::FLASH_PES_COMMAND_W","esp32c2::spi1::flash_sus_cmd::WAIT_PESR_COMMAND_W","esp32c2::spi1::date::DATE_W","esp32c2::spi2::cmd::CONF_BITLEN_W","esp32c2::spi2::addr::USR_ADDR_VALUE_W","esp32c2::spi2::ctrl::RD_BIT_ORDER_W","esp32c2::spi2::ctrl::WR_BIT_ORDER_W","esp32c2::spi2::clock::CLKCNT_L_W","esp32c2::spi2::clock::CLKCNT_H_W","esp32c2::spi2::clock::CLKCNT_N_W","esp32c2::spi2::clock::CLKDIV_PRE_W","esp32c2::spi2::user1::USR_DUMMY_CYCLELEN_W","esp32c2::spi2::user1::CS_SETUP_TIME_W","esp32c2::spi2::user1::CS_HOLD_TIME_W","esp32c2::spi2::user1::USR_ADDR_BITLEN_W","esp32c2::spi2::user2::USR_COMMAND_VALUE_W","esp32c2::spi2::user2::USR_COMMAND_BITLEN_W","esp32c2::spi2::ms_dlen::MS_DATA_BITLEN_W","esp32c2::spi2::misc::MASTER_CS_POL_W","esp32c2::spi2::w0::BUF0_W","esp32c2::spi2::w1::BUF1_W","esp32c2::spi2::w2::BUF2_W","esp32c2::spi2::w3::BUF3_W","esp32c2::spi2::w4::BUF4_W","esp32c2::spi2::w5::BUF5_W","esp32c2::spi2::w6::BUF6_W","esp32c2::spi2::w7::BUF7_W","esp32c2::spi2::w8::BUF8_W","esp32c2::spi2::w9::BUF9_W","esp32c2::spi2::w10::BUF10_W","esp32c2::spi2::w11::BUF11_W","esp32c2::spi2::w12::BUF12_W","esp32c2::spi2::w13::BUF13_W","esp32c2::spi2::w14::BUF14_W","esp32c2::spi2::w15::BUF15_W","esp32c2::spi2::slave::CLK_MODE_W","esp32c2::spi2::slave::DMA_SEG_MAGIC_VALUE_W","esp32c2::spi2::slave1::SLV_DATA_BITLEN_W","esp32c2::spi2::slave1::SLV_LAST_COMMAND_W","esp32c2::spi2::slave1::SLV_LAST_ADDR_W","esp32c2::spi2::date::DATE_W","esp32c2::system::cpu_per_conf::CPUPERIOD_SEL_W","esp32c2::system::cpu_per_conf::CPU_WAITI_DELAY_NUM_W","esp32c2::system::bt_lpck_div_int::BT_LPCK_DIV_NUM_W","esp32c2::system::bt_lpck_div_frac::BT_LPCK_DIV_B_W","esp32c2::system::bt_lpck_div_frac::BT_LPCK_DIV_A_W","esp32c2::system::rtc_fastmem_config::RTC_MEM_CRC_ADDR_W","esp32c2::system::rtc_fastmem_config::RTC_MEM_CRC_LEN_W","esp32c2::system::sysclk_conf::PRE_DIV_CNT_W","esp32c2::system::sysclk_conf::SOC_CLK_SEL_W","esp32c2::system::mem_pvt::MEM_PATH_LEN_W","esp32c2::system::mem_pvt::MEM_VT_SEL_W","esp32c2::system::comb_pvt_lvt_conf::COMB_PATH_LEN_LVT_W","esp32c2::system::comb_pvt_nvt_conf::COMB_PATH_LEN_NVT_W","esp32c2::system::comb_pvt_hvt_conf::COMB_PATH_LEN_HVT_W","esp32c2::system::reg_date::SYSTEM_REG_DATE_W","esp32c2::systimer::unit0_load_hi::TIMER_UNIT0_LOAD_HI_W","esp32c2::systimer::unit0_load_lo::TIMER_UNIT0_LOAD_LO_W","esp32c2::systimer::unit1_load_hi::TIMER_UNIT1_LOAD_HI_W","esp32c2::systimer::unit1_load_lo::TIMER_UNIT1_LOAD_LO_W","esp32c2::systimer::target0_hi::TIMER_TARGET0_HI_W","esp32c2::systimer::target0_lo::TIMER_TARGET0_LO_W","esp32c2::systimer::target1_hi::TIMER_TARGET1_HI_W","esp32c2::systimer::target1_lo::TIMER_TARGET1_LO_W","esp32c2::systimer::target2_hi::TIMER_TARGET2_HI_W","esp32c2::systimer::target2_lo::TIMER_TARGET2_LO_W","esp32c2::systimer::target0_conf::TARGET0_PERIOD_W","esp32c2::systimer::target1_conf::TARGET1_PERIOD_W","esp32c2::systimer::target2_conf::TARGET2_PERIOD_W","esp32c2::systimer::date::DATE_W","esp32c2::timg0::t0config::DIVIDER_W","esp32c2::timg0::t0alarmlo::ALARM_LO_W","esp32c2::timg0::t0alarmhi::ALARM_HI_W","esp32c2::timg0::t0loadlo::LOAD_LO_W","esp32c2::timg0::t0loadhi::LOAD_HI_W","esp32c2::timg0::t0load::LOAD_W","esp32c2::timg0::wdtconfig0::WDT_SYS_RESET_LENGTH_W","esp32c2::timg0::wdtconfig0::WDT_CPU_RESET_LENGTH_W","esp32c2::timg0::wdtconfig0::WDT_STG3_W","esp32c2::timg0::wdtconfig0::WDT_STG2_W","esp32c2::timg0::wdtconfig0::WDT_STG1_W","esp32c2::timg0::wdtconfig0::WDT_STG0_W","esp32c2::timg0::wdtconfig1::WDT_CLK_PRESCALE_W","esp32c2::timg0::wdtconfig2::WDT_STG0_HOLD_W","esp32c2::timg0::wdtconfig3::WDT_STG1_HOLD_W","esp32c2::timg0::wdtconfig4::WDT_STG2_HOLD_W","esp32c2::timg0::wdtconfig5::WDT_STG3_HOLD_W","esp32c2::timg0::wdtfeed::WDT_FEED_W","esp32c2::timg0::wdtwprotect::WDT_WKEY_W","esp32c2::timg0::rtccalicfg::RTC_CALI_CLK_SEL_W","esp32c2::timg0::rtccalicfg::RTC_CALI_MAX_W","esp32c2::timg0::rtccalicfg2::RTC_CALI_TIMEOUT_RST_CNT_W","esp32c2::timg0::rtccalicfg2::RTC_CALI_TIMEOUT_THRES_W","esp32c2::timg0::ntimers_date::NTIMGS_DATE_W","esp32c2::uart0::fifo::RXFIFO_RD_BYTE_W","esp32c2::uart0::clkdiv::CLKDIV_W","esp32c2::uart0::clkdiv::FRAG_W","esp32c2::uart0::rx_filt::GLITCH_FILT_W","esp32c2::uart0::conf0::BIT_NUM_W","esp32c2::uart0::conf0::STOP_BIT_NUM_W","esp32c2::uart0::conf1::RXFIFO_FULL_THRHD_W","esp32c2::uart0::conf1::TXFIFO_EMPTY_THRHD_W","esp32c2::uart0::sleep_conf::ACTIVE_THRESHOLD_W","esp32c2::uart0::swfc_conf0::XOFF_THRESHOLD_W","esp32c2::uart0::swfc_conf0::XOFF_CHAR_W","esp32c2::uart0::swfc_conf1::XON_THRESHOLD_W","esp32c2::uart0::swfc_conf1::XON_CHAR_W","esp32c2::uart0::txbrk_conf::TX_BRK_NUM_W","esp32c2::uart0::idle_conf::RX_IDLE_THRHD_W","esp32c2::uart0::idle_conf::TX_IDLE_NUM_W","esp32c2::uart0::rs485_conf::RS485_TX_DLY_NUM_W","esp32c2::uart0::at_cmd_precnt::PRE_IDLE_NUM_W","esp32c2::uart0::at_cmd_postcnt::POST_IDLE_NUM_W","esp32c2::uart0::at_cmd_gaptout::RX_GAP_TOUT_W","esp32c2::uart0::at_cmd_char::AT_CMD_CHAR_W","esp32c2::uart0::at_cmd_char::CHAR_NUM_W","esp32c2::uart0::mem_conf::RX_SIZE_W","esp32c2::uart0::mem_conf::TX_SIZE_W","esp32c2::uart0::mem_conf::RX_FLOW_THRHD_W","esp32c2::uart0::mem_conf::RX_TOUT_THRHD_W","esp32c2::uart0::clk_conf::SCLK_DIV_B_W","esp32c2::uart0::clk_conf::SCLK_DIV_A_W","esp32c2::uart0::clk_conf::SCLK_DIV_NUM_W","esp32c2::uart0::clk_conf::SCLK_SEL_W","esp32c2::uart0::date::DATE_W","esp32c2::uart0::id::ID_W","esp32c2::xts_aes::physical_address::PHYSICAL_ADDRESS_W","esp32c2::xts_aes::date::DATE_W"]]
};if (window.register_type_impls) {window.register_type_impls(type_impls);} else {window.pending_type_impls = type_impls;}})()