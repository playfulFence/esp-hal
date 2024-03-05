(function() {var implementors = {
"bitflags":[["impl Debug for <a class=\"struct\" href=\"bitflags/parser/struct.ParseError.html\" title=\"struct bitflags::parser::ParseError\">ParseError</a>"]],
"critical_section":[["impl Debug for <a class=\"struct\" href=\"critical_section/struct.RestoreState.html\" title=\"struct critical_section::RestoreState\">RestoreState</a>"],["impl&lt;'cs&gt; Debug for <a class=\"struct\" href=\"critical_section/struct.CriticalSection.html\" title=\"struct critical_section::CriticalSection\">CriticalSection</a>&lt;'cs&gt;"],["impl&lt;T: Debug&gt; Debug for <a class=\"struct\" href=\"critical_section/struct.Mutex.html\" title=\"struct critical_section::Mutex\">Mutex</a>&lt;T&gt;"]],
"embedded_hal":[["impl Debug for <a class=\"enum\" href=\"embedded_hal/digital/enum.ErrorKind.html\" title=\"enum embedded_hal::digital::ErrorKind\">ErrorKind</a>"],["impl Debug for <a class=\"enum\" href=\"embedded_hal/digital/enum.PinState.html\" title=\"enum embedded_hal::digital::PinState\">PinState</a>"],["impl Debug for <a class=\"enum\" href=\"embedded_hal/i2c/enum.ErrorKind.html\" title=\"enum embedded_hal::i2c::ErrorKind\">ErrorKind</a>"],["impl Debug for <a class=\"enum\" href=\"embedded_hal/i2c/enum.NoAcknowledgeSource.html\" title=\"enum embedded_hal::i2c::NoAcknowledgeSource\">NoAcknowledgeSource</a>"],["impl Debug for <a class=\"enum\" href=\"embedded_hal/pwm/enum.ErrorKind.html\" title=\"enum embedded_hal::pwm::ErrorKind\">ErrorKind</a>"],["impl Debug for <a class=\"enum\" href=\"embedded_hal/spi/enum.ErrorKind.html\" title=\"enum embedded_hal::spi::ErrorKind\">ErrorKind</a>"],["impl Debug for <a class=\"enum\" href=\"embedded_hal/spi/enum.Phase.html\" title=\"enum embedded_hal::spi::Phase\">Phase</a>"],["impl Debug for <a class=\"enum\" href=\"embedded_hal/spi/enum.Polarity.html\" title=\"enum embedded_hal::spi::Polarity\">Polarity</a>"],["impl Debug for <a class=\"struct\" href=\"embedded_hal/spi/struct.Mode.html\" title=\"struct embedded_hal::spi::Mode\">Mode</a>"],["impl&lt;'a&gt; Debug for <a class=\"enum\" href=\"embedded_hal/i2c/enum.Operation.html\" title=\"enum embedded_hal::i2c::Operation\">Operation</a>&lt;'a&gt;"],["impl&lt;'a, Word: Debug + 'static&gt; Debug for <a class=\"enum\" href=\"embedded_hal/spi/enum.Operation.html\" title=\"enum embedded_hal::spi::Operation\">Operation</a>&lt;'a, Word&gt;"]],
"enumset":[["impl&lt;T: <a class=\"trait\" href=\"enumset/trait.EnumSetType.html\" title=\"trait enumset::EnumSetType\">EnumSetType</a> + Debug&gt; Debug for <a class=\"struct\" href=\"enumset/struct.EnumSet.html\" title=\"struct enumset::EnumSet\">EnumSet</a>&lt;T&gt;"],["impl&lt;T: Debug + <a class=\"trait\" href=\"enumset/trait.EnumSetType.html\" title=\"trait enumset::EnumSetType\">EnumSetType</a>&gt; Debug for <a class=\"struct\" href=\"enumset/struct.EnumSetIter.html\" title=\"struct enumset::EnumSetIter\">EnumSetIter</a>&lt;T&gt;<div class=\"where\">where\n    T::Repr: Debug,</div>"]],
"esp32p4":[["impl Debug for <a class=\"enum\" href=\"esp32p4/enum.Interrupt.html\" title=\"enum esp32p4::Interrupt\">Interrupt</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.ADC.html\" title=\"struct esp32p4::ADC\">ADC</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.AES.html\" title=\"struct esp32p4::AES\">AES</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.AHB_DMA.html\" title=\"struct esp32p4::AHB_DMA\">AHB_DMA</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.ASSIST_DEBUG.html\" title=\"struct esp32p4::ASSIST_DEBUG\">ASSIST_DEBUG</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.AXI_DMA.html\" title=\"struct esp32p4::AXI_DMA\">AXI_DMA</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.AXI_ICM.html\" title=\"struct esp32p4::AXI_ICM\">AXI_ICM</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.BITSCRAMBLER.html\" title=\"struct esp32p4::BITSCRAMBLER\">BITSCRAMBLER</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.CACHE.html\" title=\"struct esp32p4::CACHE\">CACHE</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.DMA.html\" title=\"struct esp32p4::DMA\">DMA</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.DS.html\" title=\"struct esp32p4::DS\">DS</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.ECC.html\" title=\"struct esp32p4::ECC\">ECC</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.ECDSA.html\" title=\"struct esp32p4::ECDSA\">ECDSA</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.EFUSE.html\" title=\"struct esp32p4::EFUSE\">EFUSE</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.GPIO.html\" title=\"struct esp32p4::GPIO\">GPIO</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.GPIO_SD.html\" title=\"struct esp32p4::GPIO_SD\">GPIO_SD</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.H264.html\" title=\"struct esp32p4::H264\">H264</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.H264_DMA.html\" title=\"struct esp32p4::H264_DMA\">H264_DMA</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.HMAC.html\" title=\"struct esp32p4::HMAC\">HMAC</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.HP_SYS.html\" title=\"struct esp32p4::HP_SYS\">HP_SYS</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.HP_SYS_CLKRST.html\" title=\"struct esp32p4::HP_SYS_CLKRST\">HP_SYS_CLKRST</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.I2C0.html\" title=\"struct esp32p4::I2C0\">I2C0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.I2C1.html\" title=\"struct esp32p4::I2C1\">I2C1</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.I2S0.html\" title=\"struct esp32p4::I2S0\">I2S0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.I2S1.html\" title=\"struct esp32p4::I2S1\">I2S1</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.I2S2.html\" title=\"struct esp32p4::I2S2\">I2S2</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.I3C_MST.html\" title=\"struct esp32p4::I3C_MST\">I3C_MST</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.I3C_MST_MEM.html\" title=\"struct esp32p4::I3C_MST_MEM\">I3C_MST_MEM</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.I3C_SLV.html\" title=\"struct esp32p4::I3C_SLV\">I3C_SLV</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.INTERRUPT_CORE0.html\" title=\"struct esp32p4::INTERRUPT_CORE0\">INTERRUPT_CORE0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.INTERRUPT_CORE1.html\" title=\"struct esp32p4::INTERRUPT_CORE1\">INTERRUPT_CORE1</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.IO_MUX.html\" title=\"struct esp32p4::IO_MUX\">IO_MUX</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.ISP.html\" title=\"struct esp32p4::ISP\">ISP</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.JPEG.html\" title=\"struct esp32p4::JPEG\">JPEG</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LCD_CAM.html\" title=\"struct esp32p4::LCD_CAM\">LCD_CAM</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LEDC.html\" title=\"struct esp32p4::LEDC\">LEDC</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_ADC.html\" title=\"struct esp32p4::LP_ADC\">LP_ADC</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_ANA_PERI.html\" title=\"struct esp32p4::LP_ANA_PERI\">LP_ANA_PERI</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_AON_CLKRST.html\" title=\"struct esp32p4::LP_AON_CLKRST\">LP_AON_CLKRST</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_GPIO.html\" title=\"struct esp32p4::LP_GPIO\">LP_GPIO</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_HUK.html\" title=\"struct esp32p4::LP_HUK\">LP_HUK</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_I2C0.html\" title=\"struct esp32p4::LP_I2C0\">LP_I2C0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_I2C_ANA_MST.html\" title=\"struct esp32p4::LP_I2C_ANA_MST\">LP_I2C_ANA_MST</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_I2S0.html\" title=\"struct esp32p4::LP_I2S0\">LP_I2S0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_INTR.html\" title=\"struct esp32p4::LP_INTR\">LP_INTR</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_IO_MUX.html\" title=\"struct esp32p4::LP_IO_MUX\">LP_IO_MUX</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_PERI.html\" title=\"struct esp32p4::LP_PERI\">LP_PERI</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_SYS.html\" title=\"struct esp32p4::LP_SYS\">LP_SYS</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_TIMER.html\" title=\"struct esp32p4::LP_TIMER\">LP_TIMER</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_TOUCH.html\" title=\"struct esp32p4::LP_TOUCH\">LP_TOUCH</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_TSENS.html\" title=\"struct esp32p4::LP_TSENS\">LP_TSENS</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_UART.html\" title=\"struct esp32p4::LP_UART\">LP_UART</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.LP_WDT.html\" title=\"struct esp32p4::LP_WDT\">LP_WDT</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.MCPWM0.html\" title=\"struct esp32p4::MCPWM0\">MCPWM0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.MCPWM1.html\" title=\"struct esp32p4::MCPWM1\">MCPWM1</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.MIPI_CSI_BRIDGE.html\" title=\"struct esp32p4::MIPI_CSI_BRIDGE\">MIPI_CSI_BRIDGE</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.MIPI_CSI_HOST.html\" title=\"struct esp32p4::MIPI_CSI_HOST\">MIPI_CSI_HOST</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.MIPI_DSI_BRIDGE.html\" title=\"struct esp32p4::MIPI_DSI_BRIDGE\">MIPI_DSI_BRIDGE</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.MIPI_DSI_HOST.html\" title=\"struct esp32p4::MIPI_DSI_HOST\">MIPI_DSI_HOST</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.PARL_IO.html\" title=\"struct esp32p4::PARL_IO\">PARL_IO</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.PAU.html\" title=\"struct esp32p4::PAU\">PAU</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.PCNT.html\" title=\"struct esp32p4::PCNT\">PCNT</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.PMU.html\" title=\"struct esp32p4::PMU\">PMU</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.PPA.html\" title=\"struct esp32p4::PPA\">PPA</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.PVT.html\" title=\"struct esp32p4::PVT\">PVT</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.RMT.html\" title=\"struct esp32p4::RMT\">RMT</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.RSA.html\" title=\"struct esp32p4::RSA\">RSA</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.SDHOST.html\" title=\"struct esp32p4::SDHOST\">SDHOST</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.SHA.html\" title=\"struct esp32p4::SHA\">SHA</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.SOC_ETM.html\" title=\"struct esp32p4::SOC_ETM\">SOC_ETM</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.SPI0.html\" title=\"struct esp32p4::SPI0\">SPI0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.SPI1.html\" title=\"struct esp32p4::SPI1\">SPI1</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.SPI2.html\" title=\"struct esp32p4::SPI2\">SPI2</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.SPI3.html\" title=\"struct esp32p4::SPI3\">SPI3</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.SYSTIMER.html\" title=\"struct esp32p4::SYSTIMER\">SYSTIMER</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.TIMG0.html\" title=\"struct esp32p4::TIMG0\">TIMG0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.TIMG1.html\" title=\"struct esp32p4::TIMG1\">TIMG1</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.TRACE0.html\" title=\"struct esp32p4::TRACE0\">TRACE0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.TRACE1.html\" title=\"struct esp32p4::TRACE1\">TRACE1</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.TWAI0.html\" title=\"struct esp32p4::TWAI0\">TWAI0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.TWAI1.html\" title=\"struct esp32p4::TWAI1\">TWAI1</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.TWAI2.html\" title=\"struct esp32p4::TWAI2\">TWAI2</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.UART0.html\" title=\"struct esp32p4::UART0\">UART0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.UART1.html\" title=\"struct esp32p4::UART1\">UART1</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.UART2.html\" title=\"struct esp32p4::UART2\">UART2</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.UART3.html\" title=\"struct esp32p4::UART3\">UART3</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.UART4.html\" title=\"struct esp32p4::UART4\">UART4</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.UHCI0.html\" title=\"struct esp32p4::UHCI0\">UHCI0</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.USB_DEVICE.html\" title=\"struct esp32p4::USB_DEVICE\">USB_DEVICE</a>"],["impl Debug for <a class=\"struct\" href=\"esp32p4/struct.USB_WRAP.html\" title=\"struct esp32p4::USB_WRAP\">USB_WRAP</a>"]],
"esp_hal":[["impl Debug for <a class=\"enum\" href=\"esp_hal/clock/enum.CpuClock.html\" title=\"enum esp_hal::clock::CpuClock\">CpuClock</a>"],["impl Debug for <a class=\"enum\" href=\"esp_hal/clock/enum.XtalClock.html\" title=\"enum esp_hal::clock::XtalClock\">XtalClock</a>"],["impl Debug for <a class=\"enum\" href=\"esp_hal/enum.Cpu.html\" title=\"enum esp_hal::Cpu\">Cpu</a>"],["impl Debug for <a class=\"enum\" href=\"esp_hal/interrupt/enum.CpuInterrupt.html\" title=\"enum esp_hal::interrupt::CpuInterrupt\">CpuInterrupt</a>"],["impl Debug for <a class=\"enum\" href=\"esp_hal/interrupt/enum.Error.html\" title=\"enum esp_hal::interrupt::Error\">Error</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.ADC.html\" title=\"struct esp_hal::peripherals::ADC\">ADC</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.AES.html\" title=\"struct esp_hal::peripherals::AES\">AES</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.ASSIST_DEBUG.html\" title=\"struct esp_hal::peripherals::ASSIST_DEBUG\">ASSIST_DEBUG</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.AXI_DMA.html\" title=\"struct esp_hal::peripherals::AXI_DMA\">AXI_DMA</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.AXI_ICM.html\" title=\"struct esp_hal::peripherals::AXI_ICM\">AXI_ICM</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.BITSCRAMBLER.html\" title=\"struct esp_hal::peripherals::BITSCRAMBLER\">BITSCRAMBLER</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.CACHE.html\" title=\"struct esp_hal::peripherals::CACHE\">CACHE</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.DMA.html\" title=\"struct esp_hal::peripherals::DMA\">DMA</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.DS.html\" title=\"struct esp_hal::peripherals::DS\">DS</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.ECC.html\" title=\"struct esp_hal::peripherals::ECC\">ECC</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.ECDSA.html\" title=\"struct esp_hal::peripherals::ECDSA\">ECDSA</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.EFUSE.html\" title=\"struct esp_hal::peripherals::EFUSE\">EFUSE</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.GPIO.html\" title=\"struct esp_hal::peripherals::GPIO\">GPIO</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.GPIO_SD.html\" title=\"struct esp_hal::peripherals::GPIO_SD\">GPIO_SD</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.H264.html\" title=\"struct esp_hal::peripherals::H264\">H264</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.H264_DMA.html\" title=\"struct esp_hal::peripherals::H264_DMA\">H264_DMA</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.HMAC.html\" title=\"struct esp_hal::peripherals::HMAC\">HMAC</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.HP_SYS_CLKRST.html\" title=\"struct esp_hal::peripherals::HP_SYS_CLKRST\">HP_SYS_CLKRST</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.I2C0.html\" title=\"struct esp_hal::peripherals::I2C0\">I2C0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.I2C1.html\" title=\"struct esp_hal::peripherals::I2C1\">I2C1</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.I2S0.html\" title=\"struct esp_hal::peripherals::I2S0\">I2S0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.I2S1.html\" title=\"struct esp_hal::peripherals::I2S1\">I2S1</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.I2S2.html\" title=\"struct esp_hal::peripherals::I2S2\">I2S2</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.I3C_MST.html\" title=\"struct esp_hal::peripherals::I3C_MST\">I3C_MST</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.I3C_MST_MEM.html\" title=\"struct esp_hal::peripherals::I3C_MST_MEM\">I3C_MST_MEM</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.I3C_SLV.html\" title=\"struct esp_hal::peripherals::I3C_SLV\">I3C_SLV</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.INTERRUPT_CORE0.html\" title=\"struct esp_hal::peripherals::INTERRUPT_CORE0\">INTERRUPT_CORE0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.INTERRUPT_CORE1.html\" title=\"struct esp_hal::peripherals::INTERRUPT_CORE1\">INTERRUPT_CORE1</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.IO_MUX.html\" title=\"struct esp_hal::peripherals::IO_MUX\">IO_MUX</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.ISP.html\" title=\"struct esp_hal::peripherals::ISP\">ISP</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.JPEG.html\" title=\"struct esp_hal::peripherals::JPEG\">JPEG</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LCD_CAM.html\" title=\"struct esp_hal::peripherals::LCD_CAM\">LCD_CAM</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LEDC.html\" title=\"struct esp_hal::peripherals::LEDC\">LEDC</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_ADC.html\" title=\"struct esp_hal::peripherals::LP_ADC\">LP_ADC</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_ANA_PERI.html\" title=\"struct esp_hal::peripherals::LP_ANA_PERI\">LP_ANA_PERI</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_AON_CLKRST.html\" title=\"struct esp_hal::peripherals::LP_AON_CLKRST\">LP_AON_CLKRST</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_GPIO.html\" title=\"struct esp_hal::peripherals::LP_GPIO\">LP_GPIO</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_HUK.html\" title=\"struct esp_hal::peripherals::LP_HUK\">LP_HUK</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_I2C0.html\" title=\"struct esp_hal::peripherals::LP_I2C0\">LP_I2C0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_I2C_ANA_MST.html\" title=\"struct esp_hal::peripherals::LP_I2C_ANA_MST\">LP_I2C_ANA_MST</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_I2S0.html\" title=\"struct esp_hal::peripherals::LP_I2S0\">LP_I2S0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_INTR.html\" title=\"struct esp_hal::peripherals::LP_INTR\">LP_INTR</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_IO_MUX.html\" title=\"struct esp_hal::peripherals::LP_IO_MUX\">LP_IO_MUX</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_PERI.html\" title=\"struct esp_hal::peripherals::LP_PERI\">LP_PERI</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_SYS.html\" title=\"struct esp_hal::peripherals::LP_SYS\">LP_SYS</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_TIMER.html\" title=\"struct esp_hal::peripherals::LP_TIMER\">LP_TIMER</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_TOUCH.html\" title=\"struct esp_hal::peripherals::LP_TOUCH\">LP_TOUCH</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_TSENS.html\" title=\"struct esp_hal::peripherals::LP_TSENS\">LP_TSENS</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_UART.html\" title=\"struct esp_hal::peripherals::LP_UART\">LP_UART</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.LP_WDT.html\" title=\"struct esp_hal::peripherals::LP_WDT\">LP_WDT</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.MCPWM0.html\" title=\"struct esp_hal::peripherals::MCPWM0\">MCPWM0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.MCPWM1.html\" title=\"struct esp_hal::peripherals::MCPWM1\">MCPWM1</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.MIPI_CSI_BRIDGE.html\" title=\"struct esp_hal::peripherals::MIPI_CSI_BRIDGE\">MIPI_CSI_BRIDGE</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.MIPI_CSI_HOST.html\" title=\"struct esp_hal::peripherals::MIPI_CSI_HOST\">MIPI_CSI_HOST</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.MIPI_DSI_BRIDGE.html\" title=\"struct esp_hal::peripherals::MIPI_DSI_BRIDGE\">MIPI_DSI_BRIDGE</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.MIPI_DSI_HOST.html\" title=\"struct esp_hal::peripherals::MIPI_DSI_HOST\">MIPI_DSI_HOST</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.PARL_IO.html\" title=\"struct esp_hal::peripherals::PARL_IO\">PARL_IO</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.PAU.html\" title=\"struct esp_hal::peripherals::PAU\">PAU</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.PCNT.html\" title=\"struct esp_hal::peripherals::PCNT\">PCNT</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.PMU.html\" title=\"struct esp_hal::peripherals::PMU\">PMU</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.PPA.html\" title=\"struct esp_hal::peripherals::PPA\">PPA</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.PVT.html\" title=\"struct esp_hal::peripherals::PVT\">PVT</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.RMT.html\" title=\"struct esp_hal::peripherals::RMT\">RMT</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.RSA.html\" title=\"struct esp_hal::peripherals::RSA\">RSA</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.SDHOST.html\" title=\"struct esp_hal::peripherals::SDHOST\">SDHOST</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.SHA.html\" title=\"struct esp_hal::peripherals::SHA\">SHA</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.SOC_ETM.html\" title=\"struct esp_hal::peripherals::SOC_ETM\">SOC_ETM</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.SPI0.html\" title=\"struct esp_hal::peripherals::SPI0\">SPI0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.SPI1.html\" title=\"struct esp_hal::peripherals::SPI1\">SPI1</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.SPI2.html\" title=\"struct esp_hal::peripherals::SPI2\">SPI2</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.SPI3.html\" title=\"struct esp_hal::peripherals::SPI3\">SPI3</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.SYSTEM.html\" title=\"struct esp_hal::peripherals::SYSTEM\">SYSTEM</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.SYSTIMER.html\" title=\"struct esp_hal::peripherals::SYSTIMER\">SYSTIMER</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.TIMG0.html\" title=\"struct esp_hal::peripherals::TIMG0\">TIMG0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.TIMG1.html\" title=\"struct esp_hal::peripherals::TIMG1\">TIMG1</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.TRACE0.html\" title=\"struct esp_hal::peripherals::TRACE0\">TRACE0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.TRACE1.html\" title=\"struct esp_hal::peripherals::TRACE1\">TRACE1</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.TWAI0.html\" title=\"struct esp_hal::peripherals::TWAI0\">TWAI0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.TWAI1.html\" title=\"struct esp_hal::peripherals::TWAI1\">TWAI1</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.TWAI2.html\" title=\"struct esp_hal::peripherals::TWAI2\">TWAI2</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.UART0.html\" title=\"struct esp_hal::peripherals::UART0\">UART0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.UHCI0.html\" title=\"struct esp_hal::peripherals::UHCI0\">UHCI0</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.USB_DEVICE.html\" title=\"struct esp_hal::peripherals::USB_DEVICE\">USB_DEVICE</a>"],["impl Debug for <a class=\"struct\" href=\"esp_hal/peripherals/struct.USB_WRAP.html\" title=\"struct esp_hal::peripherals::USB_WRAP\">USB_WRAP</a>"]],
"esp_riscv_rt":[["impl Debug for <a class=\"struct\" href=\"esp_riscv_rt/struct.TrapFrame.html\" title=\"struct esp_riscv_rt::TrapFrame\">TrapFrame</a>"]],
"fugit":[["impl&lt;T: Debug, const NOM: u32, const DENOM: u32&gt; Debug for <a class=\"struct\" href=\"fugit/struct.Duration.html\" title=\"struct fugit::Duration\">Duration</a>&lt;T, NOM, DENOM&gt;"],["impl&lt;T: Debug, const NOM: u32, const DENOM: u32&gt; Debug for <a class=\"struct\" href=\"fugit/struct.Instant.html\" title=\"struct fugit::Instant\">Instant</a>&lt;T, NOM, DENOM&gt;"],["impl&lt;T: Debug, const NOM: u32, const DENOM: u32&gt; Debug for <a class=\"struct\" href=\"fugit/struct.Rate.html\" title=\"struct fugit::Rate\">Rate</a>&lt;T, NOM, DENOM&gt;"]],
"nb":[["impl&lt;E&gt; Debug for <a class=\"enum\" href=\"nb/enum.Error.html\" title=\"enum nb::Error\">Error</a>&lt;E&gt;<div class=\"where\">where\n    E: Debug,</div>"]],
"portable_atomic":[["impl Debug for <a class=\"struct\" href=\"portable_atomic/struct.AtomicBool.html\" title=\"struct portable_atomic::AtomicBool\">AtomicBool</a>"],["impl Debug for <a class=\"struct\" href=\"portable_atomic/struct.AtomicI16.html\" title=\"struct portable_atomic::AtomicI16\">AtomicI16</a>"],["impl Debug for <a class=\"struct\" href=\"portable_atomic/struct.AtomicI32.html\" title=\"struct portable_atomic::AtomicI32\">AtomicI32</a>"],["impl Debug for <a class=\"struct\" href=\"portable_atomic/struct.AtomicI8.html\" title=\"struct portable_atomic::AtomicI8\">AtomicI8</a>"],["impl Debug for <a class=\"struct\" href=\"portable_atomic/struct.AtomicIsize.html\" title=\"struct portable_atomic::AtomicIsize\">AtomicIsize</a>"],["impl Debug for <a class=\"struct\" href=\"portable_atomic/struct.AtomicU16.html\" title=\"struct portable_atomic::AtomicU16\">AtomicU16</a>"],["impl Debug for <a class=\"struct\" href=\"portable_atomic/struct.AtomicU32.html\" title=\"struct portable_atomic::AtomicU32\">AtomicU32</a>"],["impl Debug for <a class=\"struct\" href=\"portable_atomic/struct.AtomicU8.html\" title=\"struct portable_atomic::AtomicU8\">AtomicU8</a>"],["impl Debug for <a class=\"struct\" href=\"portable_atomic/struct.AtomicUsize.html\" title=\"struct portable_atomic::AtomicUsize\">AtomicUsize</a>"],["impl&lt;T&gt; Debug for <a class=\"struct\" href=\"portable_atomic/struct.AtomicPtr.html\" title=\"struct portable_atomic::AtomicPtr\">AtomicPtr</a>&lt;T&gt;"]],
"rand_core":[["impl Debug for <a class=\"struct\" href=\"rand_core/struct.Error.html\" title=\"struct rand_core::Error\">Error</a>"],["impl&lt;R: <a class=\"trait\" href=\"rand_core/block/trait.BlockRngCore.html\" title=\"trait rand_core::block::BlockRngCore\">BlockRngCore</a> + Debug&gt; Debug for <a class=\"struct\" href=\"rand_core/block/struct.BlockRng.html\" title=\"struct rand_core::block::BlockRng\">BlockRng</a>&lt;R&gt;"],["impl&lt;R: <a class=\"trait\" href=\"rand_core/block/trait.BlockRngCore.html\" title=\"trait rand_core::block::BlockRngCore\">BlockRngCore</a> + Debug&gt; Debug for <a class=\"struct\" href=\"rand_core/block/struct.BlockRng64.html\" title=\"struct rand_core::block::BlockRng64\">BlockRng64</a>&lt;R&gt;"]],
"riscv":[["impl Debug for <a class=\"enum\" href=\"riscv/register/enum.Permission.html\" title=\"enum riscv::register::Permission\">Permission</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/enum.Range.html\" title=\"enum riscv::register::Range\">Range</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/mcause/enum.Exception.html\" title=\"enum riscv::register::mcause::Exception\">Exception</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/mcause/enum.Interrupt.html\" title=\"enum riscv::register::mcause::Interrupt\">Interrupt</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/mcause/enum.Trap.html\" title=\"enum riscv::register::mcause::Trap\">Trap</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/misa/enum.XLEN.html\" title=\"enum riscv::register::misa::XLEN\">XLEN</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/mstatus/enum.Endianness.html\" title=\"enum riscv::register::mstatus::Endianness\">Endianness</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/mstatus/enum.FS.html\" title=\"enum riscv::register::mstatus::FS\">FS</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/mstatus/enum.MPP.html\" title=\"enum riscv::register::mstatus::MPP\">MPP</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/mstatus/enum.SPP.html\" title=\"enum riscv::register::mstatus::SPP\">SPP</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/mstatus/enum.XS.html\" title=\"enum riscv::register::mstatus::XS\">XS</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/mtvec/enum.TrapMode.html\" title=\"enum riscv::register::mtvec::TrapMode\">TrapMode</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/satp/enum.Mode.html\" title=\"enum riscv::register::satp::Mode\">Mode</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/scause/enum.Exception.html\" title=\"enum riscv::register::scause::Exception\">Exception</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/scause/enum.Interrupt.html\" title=\"enum riscv::register::scause::Interrupt\">Interrupt</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/scause/enum.Trap.html\" title=\"enum riscv::register::scause::Trap\">Trap</a>"],["impl Debug for <a class=\"enum\" href=\"riscv/register/sstatus/enum.SPP.html\" title=\"enum riscv::register::sstatus::SPP\">SPP</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/marchid/struct.Marchid.html\" title=\"struct riscv::register::marchid::Marchid\">Marchid</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/mcause/struct.Mcause.html\" title=\"struct riscv::register::mcause::Mcause\">Mcause</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/mcounteren/struct.Mcounteren.html\" title=\"struct riscv::register::mcounteren::Mcounteren\">Mcounteren</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/medeleg/struct.Medeleg.html\" title=\"struct riscv::register::medeleg::Medeleg\">Medeleg</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/mideleg/struct.Mideleg.html\" title=\"struct riscv::register::mideleg::Mideleg\">Mideleg</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/mie/struct.Mie.html\" title=\"struct riscv::register::mie::Mie\">Mie</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/mimpid/struct.Mimpid.html\" title=\"struct riscv::register::mimpid::Mimpid\">Mimpid</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/mip/struct.Mip.html\" title=\"struct riscv::register::mip::Mip\">Mip</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/misa/struct.Misa.html\" title=\"struct riscv::register::misa::Misa\">Misa</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/mstatus/struct.Mstatus.html\" title=\"struct riscv::register::mstatus::Mstatus\">Mstatus</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/mstatush/struct.Mstatush.html\" title=\"struct riscv::register::mstatush::Mstatush\">Mstatush</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/mtvec/struct.Mtvec.html\" title=\"struct riscv::register::mtvec::Mtvec\">Mtvec</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/mvendorid/struct.Mvendorid.html\" title=\"struct riscv::register::mvendorid::Mvendorid\">Mvendorid</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/satp/struct.Satp.html\" title=\"struct riscv::register::satp::Satp\">Satp</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/scounteren/struct.Scounteren.html\" title=\"struct riscv::register::scounteren::Scounteren\">Scounteren</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/sie/struct.Sie.html\" title=\"struct riscv::register::sie::Sie\">Sie</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/sip/struct.Sip.html\" title=\"struct riscv::register::sip::Sip\">Sip</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/sstatus/struct.Sstatus.html\" title=\"struct riscv::register::sstatus::Sstatus\">Sstatus</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/struct.Pmp.html\" title=\"struct riscv::register::Pmp\">Pmp</a>"],["impl Debug for <a class=\"struct\" href=\"riscv/register/stvec/struct.Stvec.html\" title=\"struct riscv::register::stvec::Stvec\">Stvec</a>"]],
"strum":[["impl Debug for <a class=\"enum\" href=\"strum/enum.ParseError.html\" title=\"enum strum::ParseError\">ParseError</a>"]],
"void":[["impl Debug for <a class=\"enum\" href=\"void/enum.Void.html\" title=\"enum void::Void\">Void</a>"]]
};if (window.register_implementors) {window.register_implementors(implementors);} else {window.pending_implementors = implementors;}})()