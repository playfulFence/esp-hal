(function() {var implementors = {
"critical_section":[["impl Clone for <a class=\"struct\" href=\"critical_section/struct.RestoreState.html\" title=\"struct critical_section::RestoreState\">RestoreState</a>"],["impl&lt;'cs&gt; Clone for <a class=\"struct\" href=\"critical_section/struct.CriticalSection.html\" title=\"struct critical_section::CriticalSection\">CriticalSection</a>&lt;'cs&gt;"]],
"embedded_hal":[["impl Clone for <a class=\"enum\" href=\"embedded_hal/digital/enum.ErrorKind.html\" title=\"enum embedded_hal::digital::ErrorKind\">ErrorKind</a>"],["impl Clone for <a class=\"enum\" href=\"embedded_hal/digital/enum.PinState.html\" title=\"enum embedded_hal::digital::PinState\">PinState</a>"],["impl Clone for <a class=\"enum\" href=\"embedded_hal/i2c/enum.ErrorKind.html\" title=\"enum embedded_hal::i2c::ErrorKind\">ErrorKind</a>"],["impl Clone for <a class=\"enum\" href=\"embedded_hal/i2c/enum.NoAcknowledgeSource.html\" title=\"enum embedded_hal::i2c::NoAcknowledgeSource\">NoAcknowledgeSource</a>"],["impl Clone for <a class=\"enum\" href=\"embedded_hal/pwm/enum.ErrorKind.html\" title=\"enum embedded_hal::pwm::ErrorKind\">ErrorKind</a>"],["impl Clone for <a class=\"enum\" href=\"embedded_hal/spi/enum.ErrorKind.html\" title=\"enum embedded_hal::spi::ErrorKind\">ErrorKind</a>"],["impl Clone for <a class=\"enum\" href=\"embedded_hal/spi/enum.Phase.html\" title=\"enum embedded_hal::spi::Phase\">Phase</a>"],["impl Clone for <a class=\"enum\" href=\"embedded_hal/spi/enum.Polarity.html\" title=\"enum embedded_hal::spi::Polarity\">Polarity</a>"],["impl Clone for <a class=\"struct\" href=\"embedded_hal/spi/struct.Mode.html\" title=\"struct embedded_hal::spi::Mode\">Mode</a>"]],
"enumset":[["impl&lt;T: Clone + <a class=\"trait\" href=\"enumset/trait.EnumSetType.html\" title=\"trait enumset::EnumSetType\">EnumSetType</a>&gt; Clone for <a class=\"struct\" href=\"enumset/struct.EnumSet.html\" title=\"struct enumset::EnumSet\">EnumSet</a>&lt;T&gt;<div class=\"where\">where\n    T::Repr: Clone,</div>"],["impl&lt;T: Clone + <a class=\"trait\" href=\"enumset/trait.EnumSetType.html\" title=\"trait enumset::EnumSetType\">EnumSetType</a>&gt; Clone for <a class=\"struct\" href=\"enumset/struct.EnumSetIter.html\" title=\"struct enumset::EnumSetIter\">EnumSetIter</a>&lt;T&gt;<div class=\"where\">where\n    T::Repr: Clone,</div>"]],
"esp32p4":[["impl Clone for <a class=\"enum\" href=\"esp32p4/enum.Interrupt.html\" title=\"enum esp32p4::Interrupt\">Interrupt</a>"]],
"esp_hal":[["impl Clone for <a class=\"enum\" href=\"esp_hal/clock/enum.CpuClock.html\" title=\"enum esp_hal::clock::CpuClock\">CpuClock</a>"],["impl Clone for <a class=\"enum\" href=\"esp_hal/clock/enum.XtalClock.html\" title=\"enum esp_hal::clock::XtalClock\">XtalClock</a>"],["impl Clone for <a class=\"enum\" href=\"esp_hal/enum.Cpu.html\" title=\"enum esp_hal::Cpu\">Cpu</a>"],["impl Clone for <a class=\"enum\" href=\"esp_hal/gpio/enum.Event.html\" title=\"enum esp_hal::gpio::Event\">Event</a>"],["impl Clone for <a class=\"enum\" href=\"esp_hal/gpio/enum.InputSignal.html\" title=\"enum esp_hal::gpio::InputSignal\">InputSignal</a>"],["impl Clone for <a class=\"enum\" href=\"esp_hal/gpio/enum.OutputSignal.html\" title=\"enum esp_hal::gpio::OutputSignal\">OutputSignal</a>"],["impl Clone for <a class=\"enum\" href=\"esp_hal/interrupt/enum.CpuInterrupt.html\" title=\"enum esp_hal::interrupt::CpuInterrupt\">CpuInterrupt</a>"],["impl Clone for <a class=\"enum\" href=\"esp_hal/interrupt/enum.Error.html\" title=\"enum esp_hal::interrupt::Error\">Error</a>"],["impl Clone for <a class=\"struct\" href=\"esp_hal/efuse/struct.EfuseField.html\" title=\"struct esp_hal::efuse::EfuseField\">EfuseField</a>"],["impl Clone for <a class=\"struct\" href=\"esp_hal/interrupt/struct.InterruptControl.html\" title=\"struct esp_hal::interrupt::InterruptControl\">InterruptControl</a>"]],
"esp_riscv_rt":[["impl Clone for <a class=\"struct\" href=\"esp_riscv_rt/struct.TrapFrame.html\" title=\"struct esp_riscv_rt::TrapFrame\">TrapFrame</a>"]],
"fugit":[["impl&lt;T: Clone, const NOM: u32, const DENOM: u32&gt; Clone for <a class=\"struct\" href=\"fugit/struct.Duration.html\" title=\"struct fugit::Duration\">Duration</a>&lt;T, NOM, DENOM&gt;"],["impl&lt;T: Clone, const NOM: u32, const DENOM: u32&gt; Clone for <a class=\"struct\" href=\"fugit/struct.Instant.html\" title=\"struct fugit::Instant\">Instant</a>&lt;T, NOM, DENOM&gt;"],["impl&lt;T: Clone, const NOM: u32, const DENOM: u32&gt; Clone for <a class=\"struct\" href=\"fugit/struct.Rate.html\" title=\"struct fugit::Rate\">Rate</a>&lt;T, NOM, DENOM&gt;"]],
"nb":[["impl&lt;E: Clone&gt; Clone for <a class=\"enum\" href=\"nb/enum.Error.html\" title=\"enum nb::Error\">Error</a>&lt;E&gt;"]],
"rand_core":[["impl&lt;R: Clone + <a class=\"trait\" href=\"rand_core/block/trait.BlockRngCore.html\" title=\"trait rand_core::block::BlockRngCore\">BlockRngCore</a> + ?Sized&gt; Clone for <a class=\"struct\" href=\"rand_core/block/struct.BlockRng.html\" title=\"struct rand_core::block::BlockRng\">BlockRng</a>&lt;R&gt;<div class=\"where\">where\n    R::<a class=\"associatedtype\" href=\"rand_core/block/trait.BlockRngCore.html#associatedtype.Results\" title=\"type rand_core::block::BlockRngCore::Results\">Results</a>: Clone,</div>"],["impl&lt;R: Clone + <a class=\"trait\" href=\"rand_core/block/trait.BlockRngCore.html\" title=\"trait rand_core::block::BlockRngCore\">BlockRngCore</a> + ?Sized&gt; Clone for <a class=\"struct\" href=\"rand_core/block/struct.BlockRng64.html\" title=\"struct rand_core::block::BlockRng64\">BlockRng64</a>&lt;R&gt;<div class=\"where\">where\n    R::<a class=\"associatedtype\" href=\"rand_core/block/trait.BlockRngCore.html#associatedtype.Results\" title=\"type rand_core::block::BlockRngCore::Results\">Results</a>: Clone,</div>"]],
"riscv":[["impl Clone for <a class=\"enum\" href=\"riscv/register/enum.Permission.html\" title=\"enum riscv::register::Permission\">Permission</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/enum.Range.html\" title=\"enum riscv::register::Range\">Range</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/mcause/enum.Exception.html\" title=\"enum riscv::register::mcause::Exception\">Exception</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/mcause/enum.Interrupt.html\" title=\"enum riscv::register::mcause::Interrupt\">Interrupt</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/mcause/enum.Trap.html\" title=\"enum riscv::register::mcause::Trap\">Trap</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/misa/enum.XLEN.html\" title=\"enum riscv::register::misa::XLEN\">XLEN</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/mstatus/enum.Endianness.html\" title=\"enum riscv::register::mstatus::Endianness\">Endianness</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/mstatus/enum.FS.html\" title=\"enum riscv::register::mstatus::FS\">FS</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/mstatus/enum.MPP.html\" title=\"enum riscv::register::mstatus::MPP\">MPP</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/mstatus/enum.SPP.html\" title=\"enum riscv::register::mstatus::SPP\">SPP</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/mstatus/enum.XS.html\" title=\"enum riscv::register::mstatus::XS\">XS</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/mtvec/enum.TrapMode.html\" title=\"enum riscv::register::mtvec::TrapMode\">TrapMode</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/satp/enum.Mode.html\" title=\"enum riscv::register::satp::Mode\">Mode</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/scause/enum.Exception.html\" title=\"enum riscv::register::scause::Exception\">Exception</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/scause/enum.Interrupt.html\" title=\"enum riscv::register::scause::Interrupt\">Interrupt</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/scause/enum.Trap.html\" title=\"enum riscv::register::scause::Trap\">Trap</a>"],["impl Clone for <a class=\"enum\" href=\"riscv/register/sstatus/enum.SPP.html\" title=\"enum riscv::register::sstatus::SPP\">SPP</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/delay/struct.McycleDelay.html\" title=\"struct riscv::delay::McycleDelay\">McycleDelay</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/marchid/struct.Marchid.html\" title=\"struct riscv::register::marchid::Marchid\">Marchid</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/mcause/struct.Mcause.html\" title=\"struct riscv::register::mcause::Mcause\">Mcause</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/mcounteren/struct.Mcounteren.html\" title=\"struct riscv::register::mcounteren::Mcounteren\">Mcounteren</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/medeleg/struct.Medeleg.html\" title=\"struct riscv::register::medeleg::Medeleg\">Medeleg</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/mideleg/struct.Mideleg.html\" title=\"struct riscv::register::mideleg::Mideleg\">Mideleg</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/mie/struct.Mie.html\" title=\"struct riscv::register::mie::Mie\">Mie</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/mimpid/struct.Mimpid.html\" title=\"struct riscv::register::mimpid::Mimpid\">Mimpid</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/mip/struct.Mip.html\" title=\"struct riscv::register::mip::Mip\">Mip</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/misa/struct.Misa.html\" title=\"struct riscv::register::misa::Misa\">Misa</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/mstatus/struct.Mstatus.html\" title=\"struct riscv::register::mstatus::Mstatus\">Mstatus</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/mstatush/struct.Mstatush.html\" title=\"struct riscv::register::mstatush::Mstatush\">Mstatush</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/mtvec/struct.Mtvec.html\" title=\"struct riscv::register::mtvec::Mtvec\">Mtvec</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/mvendorid/struct.Mvendorid.html\" title=\"struct riscv::register::mvendorid::Mvendorid\">Mvendorid</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/satp/struct.Satp.html\" title=\"struct riscv::register::satp::Satp\">Satp</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/scause/struct.Scause.html\" title=\"struct riscv::register::scause::Scause\">Scause</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/scounteren/struct.Scounteren.html\" title=\"struct riscv::register::scounteren::Scounteren\">Scounteren</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/sie/struct.Sie.html\" title=\"struct riscv::register::sie::Sie\">Sie</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/sip/struct.Sip.html\" title=\"struct riscv::register::sip::Sip\">Sip</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/sstatus/struct.Sstatus.html\" title=\"struct riscv::register::sstatus::Sstatus\">Sstatus</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/struct.Pmp.html\" title=\"struct riscv::register::Pmp\">Pmp</a>"],["impl Clone for <a class=\"struct\" href=\"riscv/register/stvec/struct.Stvec.html\" title=\"struct riscv::register::stvec::Stvec\">Stvec</a>"]],
"strum":[["impl Clone for <a class=\"enum\" href=\"strum/enum.ParseError.html\" title=\"enum strum::ParseError\">ParseError</a>"]],
"void":[["impl Clone for <a class=\"enum\" href=\"void/enum.Void.html\" title=\"enum void::Void\">Void</a>"]]
};if (window.register_implementors) {window.register_implementors(implementors);} else {window.pending_implementors = implementors;}})()