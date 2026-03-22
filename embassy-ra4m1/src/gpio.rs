use crate::pac;
use crate::peripherals;
use crate::peripherals::{P100, P110};
use core::convert::Infallible;
use embassy_hal_internal::{Peri, PeripheralType, impl_peripheral};

/// Pull-up configuration for input pins.
///
/// The RA4M1 only supports pull-up resistors.
/// Controlled via PmnPFS.PCR
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    None,
    Up,
}

/// Output drive strength, mapping to PmnPFS.DSCR\[1:0]
///
/// | Variant      | DSCR  | Description                        |
/// |--------------|-------|------------------------------------|
/// | Low          | 0b00  | Low drive (reset default)          |
/// | IicFastMode  | 0b01  | Middle drive for I2C fast-mode     |
/// | Medium       | 0b10  | Middle drive, general purpose      |
///
/// 0b11 is prohibited by the manual and is not representable here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriveStrength {
    Low,
    Medium,
    //IicFastMode,
}

impl DriveStrength {
    const fn dscr_bit(self) -> bool {
        match self {
            DriveStrength::Low => false,
            DriveStrength::Medium => true,
            //DriveStrength::IicFastMode => 0b10,
        }
    }
}

/// Logic level
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    Low,
    High,
}

impl From<bool> for Level {
    fn from(value: bool) -> Self {
        if value { Level::High } else { Level::Low }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> Self {
        level == Level::High
    }
}

#[inline]
fn with_pfs_unlocked(f: impl FnOnce()) {
    critical_section::with(|_| {
        /*
         * Safety: We are the sole writer to PWPR within this critical section.
         * The sequence follows exactly 19.5.1 of the RA4M1 user manual.
         */
        let pmisc = unsafe { &*pac::PMISC::PTR };

        // Step 1: B0WI=0, permit modification of PFSWE
        pmisc.pwpr.write(|w| w.b0wi()._0());

        // Step 2: PFSWE=1, permit modification of PFS registers
        pmisc.pwpr.write(|w| w.pfswe()._1());

        f();

        // Step 3: PFSWE=0, relinquish modification of PFS registers
        pmisc.pwpr.write(|w| w.pfswe()._0());

        // Step 4: B0WI=1, relinquish modification of PWPR
        pmisc.pwpr.write(|w| w.b0wi()._1());
    })
}

pub(crate) trait SealedPin {
    /// Runtime identity: `port_number * 16 + pin_number`
    ///
    /// For P111: 1 * 16 + 11 = 27. This single byte is all the data a pin type carries at runtime;
    /// everything else is register logic.
    fn pin_port(&self) -> u8;

    #[inline]
    fn pin(&self) -> u8 {
        self.pin_port() % 16
    }

    #[inline]
    fn port(&self) -> u8 {
        self.pin_port() / 16
    }

    fn set_high(&self);
    fn set_low(&self);

    /// Read the actual electrical state from PCNTR2.PIDR.
    /// Valid regardless of pin direction or PMR mode.
    fn is_input_high(&self) -> bool;

    /// Read the requested output level from PCNTR1.PODR.
    /// Reflects what we wrote, not the physical pin state.
    fn is_output_high(&self) -> bool;

    fn set_as_input(&self, pull: Pull);
    fn set_as_output(&self, drive_strength: DriveStrength, initial_level: Level);

    /// Return the pin to its reset state: floating input, GPIO mode, no pull.
    fn set_as_disconnected(&self);
}

pub trait Pin: SealedPin + PeripheralType + Into<AnyPin> + Sized + 'static {
    fn pin(&self) -> u8 {
        SealedPin::pin(self)
    }

    fn port(&self) -> u8 {
        SealedPin::port(self)
    }

    /// Type-erase this pin into an `AnyPin`, discarding compile-time identity.
    ///
    /// The runtime port+pin encoding is preserved. The main use case is storing heterogeneous
    /// pins (from different ports) in a single collection or passing a pin to code that doesn't
    /// need to know the specific port.
    fn degrade(self) -> AnyPin {
        AnyPin {
            pin_port: self.pin_port(),
        }
    }
}

/// Marker trait, only implemented for pins that can be outputs
/// Input only pins (P200, P214, P215) do not get this impl
pub trait OutputPin: Pin {}

/// A type-erased pin. Holds only the encoded port+pin byte and dispatches to the correct port
/// peripheral at runtime via a match.
///
/// This is what Flex<'d> actually stores. It allows Flex to be a non-generic type. Without AnyPin,
/// Flex<'d, T: Pin> would infect every type that wraps a Flex with a pin type parameter.
pub struct AnyPin {
    pin_port: u8,
}

impl_peripheral!(AnyPin);

impl Pin for AnyPin {}

impl SealedPin for AnyPin {
    #[inline]
    fn pin_port(&self) -> u8 {
        self.pin_port
    }

    #[inline]
    fn set_high(&self) {
        let pin = SealedPin::pin(self);

        /*
         * PORT2/3/4 are re-exported as port1:: type.
         * PORT5-9 are re-exported as port0:: type.
         * Each branch compiles down to a single word-sized store; the match is the only overhead
         * vs the monomorphised concrete-pin path.
         */
        match SealedPin::port(self) {
            0 => unsafe { &*pac::PORT0::PTR }
                .pcntr3()
                .write(|w| unsafe { w.posr().bits(1 << pin) }),
            1 => unsafe { &*pac::PORT1::PTR }
                .pcntr3()
                .write(|w| unsafe { w.posr().bits(1 << pin) }),
            2 => unsafe { &*pac::PORT2::PTR }
                .pcntr3()
                .write(|w| unsafe { w.posr().bits(1 << pin) }),
            3 => unsafe { &*pac::PORT3::PTR }
                .pcntr3()
                .write(|w| unsafe { w.posr().bits(1 << pin) }),
            4 => unsafe { &*pac::PORT4::PTR }
                .pcntr3()
                .write(|w| unsafe { w.posr().bits(1 << pin) }),
            5 => unsafe { &*pac::PORT5::PTR }
                .pcntr3()
                .write(|w| unsafe { w.posr().bits(1 << pin) }),
            6 => unsafe { &*pac::PORT6::PTR }
                .pcntr3()
                .write(|w| unsafe { w.posr().bits(1 << pin) }),
            7 => unsafe { &*pac::PORT7::PTR }
                .pcntr3()
                .write(|w| unsafe { w.posr().bits(1 << pin) }),
            8 => unsafe { &*pac::PORT8::PTR }
                .pcntr3()
                .write(|w| unsafe { w.posr().bits(1 << pin) }),
            9 => unsafe { &*pac::PORT9::PTR }
                .pcntr3()
                .write(|w| unsafe { w.posr().bits(1 << pin) }),
            _ => unreachable!(),
        }
    }

    #[inline]
    fn set_low(&self) {
        let pin = SealedPin::pin(self);

        match SealedPin::port(self) {
            0 => unsafe { &*pac::PORT0::PTR }
                .pcntr3()
                .write(|w| unsafe { w.porr().bits(1 << pin) }),
            1 => unsafe { &*pac::PORT1::PTR }
                .pcntr3()
                .write(|w| unsafe { w.porr().bits(1 << pin) }),
            2 => unsafe { &*pac::PORT2::PTR }
                .pcntr3()
                .write(|w| unsafe { w.porr().bits(1 << pin) }),
            3 => unsafe { &*pac::PORT3::PTR }
                .pcntr3()
                .write(|w| unsafe { w.porr().bits(1 << pin) }),
            4 => unsafe { &*pac::PORT4::PTR }
                .pcntr3()
                .write(|w| unsafe { w.porr().bits(1 << pin) }),
            5 => unsafe { &*pac::PORT5::PTR }
                .pcntr3()
                .write(|w| unsafe { w.porr().bits(1 << pin) }),
            6 => unsafe { &*pac::PORT6::PTR }
                .pcntr3()
                .write(|w| unsafe { w.porr().bits(1 << pin) }),
            7 => unsafe { &*pac::PORT7::PTR }
                .pcntr3()
                .write(|w| unsafe { w.porr().bits(1 << pin) }),
            8 => unsafe { &*pac::PORT8::PTR }
                .pcntr3()
                .write(|w| unsafe { w.porr().bits(1 << pin) }),
            9 => unsafe { &*pac::PORT9::PTR }
                .pcntr3()
                .write(|w| unsafe { w.porr().bits(1 << pin) }),
            _ => unreachable!(),
        }
    }

    #[inline]
    fn is_input_high(&self) -> bool {
        let pin = SealedPin::pin(self);

        /*
         * All ports expose pidr() returning u16; the type difference between port0:: and port1::
         * doesn't affect this read path.
         */
        let bits = match SealedPin::port(self) {
            0 => unsafe { &*pac::PORT0::PTR }.pcntr2().read().pidr().bits(),
            1 => unsafe { &*pac::PORT1::PTR }.pcntr2().read().pidr().bits(),
            2 => unsafe { &*pac::PORT2::PTR }.pcntr2().read().pidr().bits(),
            3 => unsafe { &*pac::PORT3::PTR }.pcntr2().read().pidr().bits(),
            4 => unsafe { &*pac::PORT4::PTR }.pcntr2().read().pidr().bits(),
            5 => unsafe { &*pac::PORT5::PTR }.pcntr2().read().pidr().bits(),
            6 => unsafe { &*pac::PORT6::PTR }.pcntr2().read().pidr().bits(),
            7 => unsafe { &*pac::PORT7::PTR }.pcntr2().read().pidr().bits(),
            8 => unsafe { &*pac::PORT8::PTR }.pcntr2().read().pidr().bits(),
            9 => unsafe { &*pac::PORT9::PTR }.pcntr2().read().pidr().bits(),
            _ => unreachable!(),
        };
        bits & (1 << pin) != 0
    }

    #[inline]
    fn is_output_high(&self) -> bool {
        let pin = SealedPin::pin(self);
        let bits = match SealedPin::port(self) {
            0 => unsafe { &*pac::PORT0::PTR }.pcntr1().read().podr().bits(),
            1 => unsafe { &*pac::PORT1::PTR }.pcntr1().read().podr().bits(),
            2 => unsafe { &*pac::PORT2::PTR }.pcntr1().read().podr().bits(),
            3 => unsafe { &*pac::PORT3::PTR }.pcntr1().read().podr().bits(),
            4 => unsafe { &*pac::PORT4::PTR }.pcntr1().read().podr().bits(),
            5 => unsafe { &*pac::PORT5::PTR }.pcntr1().read().podr().bits(),
            6 => unsafe { &*pac::PORT6::PTR }.pcntr1().read().podr().bits(),
            7 => unsafe { &*pac::PORT7::PTR }.pcntr1().read().podr().bits(),
            8 => unsafe { &*pac::PORT8::PTR }.pcntr1().read().podr().bits(),
            9 => unsafe { &*pac::PORT9::PTR }.pcntr1().read().podr().bits(),
            _ => unreachable!(),
        };
        bits & (1 << pin) != 0
    }

    fn set_as_input(&self, pull: Pull) {
        let pin = SealedPin::pin(self);

        // Step 1: PDR=0 for this pin. Modify to preserve other pins.
        critical_section::with(|_| {
            let mask = !(1 << pin);
            match SealedPin::port(self) {
                0 => unsafe { &*pac::PORT0::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                1 => unsafe { &*pac::PORT1::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                2 => unsafe { &*pac::PORT2::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                3 => unsafe { &*pac::PORT3::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                4 => unsafe { &*pac::PORT4::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                5 => unsafe { &*pac::PORT5::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                6 => unsafe { &*pac::PORT6::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                7 => unsafe { &*pac::PORT7::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                8 => unsafe { &*pac::PORT8::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                9 => unsafe { &*pac::PORT9::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                _ => unreachable!(),
            }
        });

        /*
         * Step 2: PFS, set PMR=0 (GPIO), PCR per pull argument. This requires the PWPR unlock
         * sequence. AnyPin uses a match to reach the right PFS register. Note: this only covers
         * pins that use P000PFS_SPEC.
         * P108/P110 are deliberately excluded until their API is verified.
         */
        with_pfs_unlocked(|| {
            let pfs = unsafe { &*pac::PFS::PTR };

            /// Helper macro, not a function because we can't abstract over the three (or more, I
            /// might not have seen them all) SPEC types without a trait. This generates identical
            /// code for each arm, letting the compiler resolve the type per-arm.
            macro_rules! configure_pfs_input {
                ($reg:expr) => {
                    $reg.write(|w| {
                        match pull {
                            Pull::None => w.pcr()._0(),
                            Pull::Up => w.pcr()._1(),
                        }
                        .pmr()
                        ._0()
                    })
                };
            }

            match (SealedPin::port(self), SealedPin::pin(self)) {
                (1, 0) => configure_pfs_input!(pfs.p100pfs()[0]),
                (1, 1) => configure_pfs_input!(pfs.p100pfs()[1]),
                (1, 2) => configure_pfs_input!(pfs.p100pfs()[2]),
                (1, 3) => configure_pfs_input!(pfs.p100pfs()[3]),
                (1, 4) => configure_pfs_input!(pfs.p100pfs()[4]),
                (1, 5) => configure_pfs_input!(pfs.p100pfs()[5]),
                (1, 6) => configure_pfs_input!(pfs.p100pfs()[6]),
                (1, 7) => configure_pfs_input!(pfs.p100pfs()[7]),
                (1, 8) => configure_pfs_input!(pfs.p108pfs()),
                (1, 9) => configure_pfs_input!(pfs.p109pfs()),
                (1, 10) => configure_pfs_input!(pfs.p110pfs()),
                (1, 11) => configure_pfs_input!(pfs.p111pfs()),
                _ => unimplemented!("AnyPin PFS not yet implemented for this pin"),
            }
        });
    }

    fn set_as_output(&self, drive: DriveStrength, initial_level: Level) {
        match initial_level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }

        let pin = SealedPin::pin(self);
        // PFS: GPIO mode, drive strength
        with_pfs_unlocked(|| {
            let pfs = unsafe { &*pac::PFS::PTR };
            let dscr = drive.dscr_bit();

            macro_rules! configure_pfs_output {
                ($reg:expr) => {
                    $reg.write(|w| w.pmr()._0().dscr().bit(dscr))
                };
            }

            match (SealedPin::port(self), pin) {
                (1, 0) => configure_pfs_output!(pfs.p100pfs()[0]),
                (1, 1) => configure_pfs_output!(pfs.p100pfs()[1]),
                (1, 2) => configure_pfs_output!(pfs.p100pfs()[2]),
                (1, 3) => configure_pfs_output!(pfs.p100pfs()[3]),
                (1, 4) => configure_pfs_output!(pfs.p100pfs()[4]),
                (1, 5) => configure_pfs_output!(pfs.p100pfs()[5]),
                (1, 6) => configure_pfs_output!(pfs.p100pfs()[6]),
                (1, 7) => configure_pfs_output!(pfs.p100pfs()[7]),
                (1, 8) => configure_pfs_output!(pfs.p108pfs()),
                (1, 9) => configure_pfs_output!(pfs.p109pfs()),
                (1, 10) => configure_pfs_output!(pfs.p110pfs()),
                (1, 11) => configure_pfs_output!(pfs.p111pfs()),
                _ => unimplemented!("AnyPin PFS not yet implemented for this pin"),
            }
        });

        // PDR=1: pin starts driving now
        critical_section::with(|_| {
            let mask = 1 << pin;
            match SealedPin::port(self) {
                0 => unsafe { &*pac::PORT0::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | mask) }),
                1 => unsafe { &*pac::PORT1::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | mask) }),
                2 => unsafe { &*pac::PORT2::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | mask) }),
                3 => unsafe { &*pac::PORT3::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | mask) }),
                4 => unsafe { &*pac::PORT4::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | mask) }),
                5 => unsafe { &*pac::PORT5::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | mask) }),
                6 => unsafe { &*pac::PORT6::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | mask) }),
                7 => unsafe { &*pac::PORT7::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | mask) }),
                8 => unsafe { &*pac::PORT8::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | mask) }),
                9 => unsafe { &*pac::PORT9::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | mask) }),
                _ => unreachable!(),
            }
        });
    }

    fn set_as_disconnected(&self) {
        let pin = SealedPin::pin(self);

        // PDR=0 first
        critical_section::with(|_| {
            let mask = !(1 << pin);
            match SealedPin::port(self) {
                0 => unsafe { &*pac::PORT0::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                1 => unsafe { &*pac::PORT1::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                2 => unsafe { &*pac::PORT2::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                3 => unsafe { &*pac::PORT3::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                4 => unsafe { &*pac::PORT4::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                5 => unsafe { &*pac::PORT5::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                6 => unsafe { &*pac::PORT6::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                7 => unsafe { &*pac::PORT7::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                8 => unsafe { &*pac::PORT8::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                9 => unsafe { &*pac::PORT9::PTR }
                    .pcntr1()
                    .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & mask) }),
                _ => unreachable!(),
            }
        });

        with_pfs_unlocked(|| {
            let pfs = unsafe { &*pac::PFS::PTR };

            macro_rules! reset_pfs {
                ($reg:expr) => {
                    $reg.write(|w| {
                        w.pmr()
                            ._0()
                            .pcr()
                            ._0()
                            .ncodr()
                            ._0()
                            .dscr()
                            .bit(false)
                            .isel()
                            ._0()
                            .asel()
                            ._0()
                            .psel()
                            .variant(0b00000)
                    })
                };
            }

            match (SealedPin::port(self), pin) {
                (1, 0) => reset_pfs!(&pfs.p100pfs()[0]),
                (1, 1) => reset_pfs!(&pfs.p100pfs()[1]),
                (1, 2) => reset_pfs!(&pfs.p100pfs()[2]),
                (1, 3) => reset_pfs!(&pfs.p100pfs()[3]),
                (1, 4) => reset_pfs!(&pfs.p100pfs()[4]),
                (1, 5) => reset_pfs!(&pfs.p100pfs()[5]),
                (1, 6) => reset_pfs!(&pfs.p100pfs()[6]),
                (1, 7) => reset_pfs!(&pfs.p100pfs()[7]),
                (1, 8) => reset_pfs!(pfs.p108pfs()),
                (1, 9) => reset_pfs!(pfs.p109pfs()),
                (1, 10) => reset_pfs!(pfs.p110pfs()),
                (1, 11) => reset_pfs!(pfs.p111pfs()),
                _ => unimplemented!("AnyPin PFS not yet implemented for this pin"),
            }
        });
    }
}

/// This macro is the RA4M1 equivalent of embassy-stm32's `foreach_pin!` expansion. Each invocation
/// produces three impls:
///     1. SealedPin - all register logic, statically dispatched
///     2. Pin - public marker
///     3. From<Pxy> for AnyPin, enables .degrade() and Peri<impl Pin> -> Peri<AnyPin>
///
/// Because $port_periph and $pin_num are compile-time constants, the compiler can resolve every
/// register address and bitmask at compile time. set_high() for P111 should compile to:
///     movs r0, #2048 ; 1 << 11
///     strh r0, \[PORT1_PCNTR3_ADDR]
/// Two instructions, zero branches

macro_rules! impl_pin {
    ($name:ident, $port_periph:ident, $port_num:literal, $pin_num:literal, $pfs_method:ident [ $pfs_idx:literal ]) => {
        impl_pin_inner!(
            $name,
            $port_periph,
            $port_num,
            $pin_num,
            $pfs_method[$pfs_idx]
        );
        impl OutputPin for peripherals::$name {}
    };
    ($name:ident, $port_periph:ident, $port_num:literal, $pin_num:literal, $pfs_method:ident) => {
        impl_pin_inner!($name, $port_periph, $port_num, $pin_num, $pfs_method);
        impl OutputPin for peripherals::$name {}
    };
}

macro_rules! impl_input_only_pin {
    ($name:ident, $port_periph:ident, $port_num:literal, $pin_num:literal, $pfs_method:ident) => {
        impl_pin_inner!($name, $port_periph, $port_num, $pin_num, $pfs_method);
    };
}

macro_rules! impl_pin_inner {
    ($name:ident, $port_periph:ident, $port_num:literal, $pin_num:literal, $pfs_method:ident) => {
        impl_peripheral!($name);

        impl Pin for peripherals::$name {}

        impl From<peripherals::$name> for AnyPin {
            fn from(val: peripherals::$name) -> AnyPin {
                AnyPin {
                    pin_port: SealedPin::pin_port(&val),
                }
            }
        }

        impl SealedPin for peripherals::$name {
            #[inline]
            fn pin_port(&self) -> u8 {
                $port_num * 16 + $pin_num
            }

            #[inline]
            fn set_high(&self) {
                unsafe { &*pac::$port_periph::PTR }
                    .pcntr3()
                    .write(|w| unsafe { w.posr().bits(1 << $pin_num) });
            }

            #[inline]
            fn set_low(&self) {
                unsafe { &*pac::$port_periph::PTR }
                    .pcntr3()
                    .write(|w| unsafe { w.porr().bits(1 << $pin_num) });
            }

            #[inline]
            fn is_input_high(&self) -> bool {
                unsafe { &*pac::$port_periph::PTR }
                    .pcntr2()
                    .read()
                    .pidr()
                    .bits()
                    & (1 << $pin_num)
                    != 0
            }

            #[inline]
            fn is_output_high(&self) -> bool {
                unsafe { &*pac::$port_periph::PTR }
                    .pcntr1()
                    .read()
                    .podr()
                    .bits()
                    & (1 << $pin_num)
                    != 0
            }

            fn set_as_input(&self, pull: Pull) {
                /*
                 * Step 1: PDR=0 for this pin, preserve all other PDR bits. This is read-modify-write
                 * on a shared 16-bit register, so it needs a critical section.
                 */
                critical_section::with(|_| {
                    unsafe { &*pac::$port_periph::PTR }
                        .pcntr1()
                        .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & !(1 << $pin_num)) });
                });

                /*
                 * Step 2: PFS configuration, GPIO mode, pull-up setting. PMR=0 is technically the
                 * reset default, but we set it explicitly to be safe against prior peripheral usage.
                 */
                with_pfs_unlocked(|| {
                    let pfs = unsafe { &*pac::PFS::PTR };
                    pfs.$pfs_method().modify(|_, w| {
                        match pull {
                            Pull::None => w.pcr()._0(),
                            Pull::Up => w.pcr()._1(),
                        }
                        .pmr()
                        ._0()
                    });
                });
            }

            fn set_as_output(&self, drive: DriveStrength, initial_level: Level) {
                /*
                 * Critical ordering: stage the output level BEFORE setting PDR=1.
                 *
                 * If we set PDR=1 first, the pin immediately starts driving whatever PODR happens
                 * to contain, potentially the wrong level. By writing POSR or PORR first, PODR is
                 * already correct by the time PDR=1 takes effect.
                 */
                match initial_level {
                    Level::Low => self.set_low(),
                    Level::High => self.set_high(),
                }

                // PFS: GPIO Mode, drive strength
                with_pfs_unlocked(|| {
                    let pfs = unsafe { &*pac::PFS::PTR };
                    pfs.$pfs_method()
                        .modify(|_, w| w.pmr()._0().dscr().bit(drive.dscr_bit()));
                });

                // PDR=1
                critical_section::with(|_| {
                    unsafe { &*pac::$port_periph::PTR }
                        .pcntr1()
                        .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | (1 << $pin_num)) });
                });
            }

            fn set_as_disconnected(&self) {
                // PDR=0 first: stop driving before reconfiguring PFS
                critical_section::with(|_| {
                    unsafe { &*pac::$port_periph::PTR }
                        .pcntr1()
                        .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & !(1 << $pin_num)) });
                });

                // PFS: reset all fields to power-on defaults
                with_pfs_unlocked(|| {
                    let pfs = unsafe { &*pac::PFS::PTR };

                    pfs.$pfs_method().write(|w| {
                        w.pmr()
                            ._0()
                            .pcr()
                            ._0()
                            .ncodr()
                            ._0()
                            .dscr()
                            .bit(false)
                            .isel()
                            ._0()
                            .asel()
                            ._0()
                            .psel()
                            .variant(0b00000)
                    });
                });
            }
        }
    };
    ($name:ident, $port_periph:ident, $port_num:literal, $pin_num:literal, $pfs_method:ident [ $pfs_idx:literal ]) => {
        impl_peripheral!($name);

        impl Pin for peripherals::$name {}

        impl From<peripherals::$name> for AnyPin {
            fn from(val: peripherals::$name) -> AnyPin {
                AnyPin {
                    pin_port: SealedPin::pin_port(&val),
                }
            }
        }

        impl SealedPin for peripherals::$name {
            #[inline]
            fn pin_port(&self) -> u8 {
                $port_num * 16 + $pin_num
            }

            #[inline]
            fn set_high(&self) {
                unsafe { &*pac::$port_periph::PTR }
                    .pcntr3()
                    .write(|w| unsafe { w.posr().bits(1 << $pin_num) });
            }

            #[inline]
            fn set_low(&self) {
                unsafe { &*pac::$port_periph::PTR }
                    .pcntr3()
                    .write(|w| unsafe { w.porr().bits(1 << $pin_num) });
            }

            #[inline]
            fn is_input_high(&self) -> bool {
                unsafe { &*pac::$port_periph::PTR }
                    .pcntr2()
                    .read()
                    .pidr()
                    .bits()
                    & (1 << $pin_num)
                    != 0
            }

            #[inline]
            fn is_output_high(&self) -> bool {
                unsafe { &*pac::$port_periph::PTR }
                    .pcntr1()
                    .read()
                    .podr()
                    .bits()
                    & (1 << $pin_num)
                    != 0
            }

            fn set_as_input(&self, pull: Pull) {
                /*
                 * Step 1: PDR=0 for this pin, preserve all other PDR bits. This is read-modify-write
                 * on a shared 16-bit register, so it needs a critical section.
                 */
                critical_section::with(|_| {
                    unsafe { &*pac::$port_periph::PTR }
                        .pcntr1()
                        .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & !(1 << $pin_num)) });
                });

                /*
                 * Step 2: PFS configuration, GPIO mode, pull-up setting. PMR=0 is technically the
                 * reset default, but we set it explicitly to be safe against prior peripheral usage.
                 */
                with_pfs_unlocked(|| {
                    let pfs = unsafe { &*pac::PFS::PTR };
                    pfs.$pfs_method()[$pfs_idx].modify(|_, w| {
                        match pull {
                            Pull::None => w.pcr()._0(),
                            Pull::Up => w.pcr()._1(),
                        }
                        .pmr()
                        ._0()
                    });
                });
            }

            fn set_as_output(&self, drive: DriveStrength, initial_level: Level) {
                /*
                 * Critical ordering: stage the output level BEFORE setting PDR=1.
                 *
                 * If we set PDR=1 first, the pin immediately starts driving whatever PODR happens
                 * to contain, potentially the wrong level. By writing POSR or PORR first, PODR is
                 * already correct by the time PDR=1 takes effect.
                 */
                match initial_level {
                    Level::Low => self.set_low(),
                    Level::High => self.set_high(),
                }

                // PFS: GPIO Mode, drive strength
                with_pfs_unlocked(|| {
                    let pfs = unsafe { &*pac::PFS::PTR };
                    pfs.$pfs_method()[$pfs_idx]
                        .modify(|_, w| w.pmr()._0().dscr().bit(drive.dscr_bit()));
                });

                // PDR=1
                critical_section::with(|_| {
                    unsafe { &*pac::$port_periph::PTR }
                        .pcntr1()
                        .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() | (1 << $pin_num)) });
                });
            }

            fn set_as_disconnected(&self) {
                // PDR=0 first: stop driving before reconfiguring PFS
                critical_section::with(|_| {
                    unsafe { &*pac::$port_periph::PTR }
                        .pcntr1()
                        .modify(|r, w| unsafe { w.pdr().bits(r.pdr().bits() & !(1 << $pin_num)) });
                });

                // PFS: reset all fields to power-on defaults
                with_pfs_unlocked(|| {
                    let pfs = unsafe { &*pac::PFS::PTR };

                    pfs.$pfs_method()[$pfs_idx].write(|w| {
                        w.pmr()
                            ._0()
                            .pcr()
                            ._0()
                            .ncodr()
                            ._0()
                            .dscr()
                            .bit(false)
                            .isel()
                            ._0()
                            .asel()
                            ._0()
                            .psel()
                            .variant(0b00000)
                    });
                });
            }
        }
    };
}

impl_pin!(P100, PORT1, 1, 0, p100pfs[0]);
impl_pin!(P110, PORT1, 1, 11, p110pfs);

/// Flex is the foundational wrapper. It holds a type-erased pin and can reconfigure its direction
/// at any time. Input<> and Output<> are just Flex with a constrained construction path; they hold
/// a Flex internally and add no data.
pub struct Flex<'d> {
    pub(crate) pin: Peri<'d, AnyPin>,
}

impl<'d> Flex<'d> {
    /// Wrap a pin as a Flex. The pin is not yet configured, it remains in whatever state it was in
    /// before (typically floating input from reset).
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>) -> Self {
        Self { pin: pin.into() }
    }

    /// Reconfigure as a floating or pulled input.
    #[inline]
    pub fn set_as_input(&mut self, pull: Pull) {
        self.pin.set_as_input(pull);
    }

    /// Reconfigure as a push-pull output with specified drive strength.
    ///
    /// The initial output level should be set via `set_high` or `set_low` after calling this, or
    /// use `Output::new` which handles ordering.
    #[inline]
    pub fn set_as_output(&mut self, drive: DriveStrength) {
        self.pin.set_as_output(drive, Level::Low);
    }

    /// Read the actual electrical state of the pin (PCNTR2.PIDR)
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_input_high()
    }

    /// Read the actual electrical state of the pin (PCNTR2.PIDR)
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Read the current input level as a `Level`
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Read back the last written output level (PCNTR1.PODR)
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_output_high()
    }

    /// Read back the last written output level (PCNTR1.PODR)
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    /// Read the current output level as a `Level`
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    /// Drive the pin high via PCNTR3.POSR (atomic, no read-modify-write)
    #[inline]
    pub fn set_high(&self) {
        self.pin.set_high();
    }

    /// Drive the pin low via PCNTR3.PORR (atomic, no read-modify-write)
    #[inline]
    pub fn set_low(&self) {
        self.pin.set_low();
    }

    /// Set the output level
    #[inline]
    pub fn set_level(&self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    /// Toggle the output level
    #[inline]
    pub fn toggle(&self) {
        /*
         * We read PODR (requested level), not PIDR (the electrical state), so toggle is consistent
         * even when the pin is loaded.
         */
        if self.is_set_low() {
            self.set_high();
        } else {
            self.set_low();
        }
    }
}

impl Drop for Flex<'_> {
    fn drop(&mut self) {
        self.pin.set_as_disconnected();
    }
}

/// A GPIO pin configured as an input.
///
/// When dropped, the pin is returned to its reset state (floating input).
pub struct Input<'d> {
    pub(crate) pin: Flex<'d>,
}

impl<'d> Input<'d> {
    /// Configure a pin as an input with the given pull setting.
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, pull: Pull) -> Self {
        let mut flex = Flex::new(pin);
        flex.set_as_input(pull);
        Self { pin: flex }
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }
}

/// A GPIO pin configured as a push-pull output.
///
/// When dropped, the pin is returned to its reset state (floating input).
pub struct Output<'d> {
    pub(crate) pin: Flex<'d>,
}

impl<'d> Output<'d> {
    /// Configure a pin as an output.
    ///
    /// `initial_level` is driven atomically as the pin enters output mode, there is no
    /// intermediate glitch.
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, initial_level: Level, drive: DriveStrength) -> Self {
        let flex = Flex::new(pin);
        // set_as_output stages the level before enabling the output driver.
        flex.pin.set_as_output(drive, initial_level);
        Self { pin: flex }
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high();
    }
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low();
    }
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level);
    }
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }
}

impl embedded_hal::digital::ErrorType for Input<'_> {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Input<'_> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_low(self))
    }
}

impl embedded_hal::digital::ErrorType for Output<'_> {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for Output<'_> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Self::set_low(self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Self::set_high(self);
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Output<'_> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_set_high(self))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_set_low(self))
    }
}

impl embedded_hal::digital::ErrorType for Flex<'_> {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Flex<'_> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_low(self))
    }
}

impl embedded_hal::digital::OutputPin for Flex<'_> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Self::set_low(self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Self::set_high(self);
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Flex<'_> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_set_high(self))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_set_low(self))
    }
}
