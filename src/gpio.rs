use core::{cell::RefCell, marker::PhantomData};
use super::*;

mutually_exclusive_features::exactly_one_of!("gpio_critical_section", "gpio_atomic");

impl<I2C: i2c::I2c + Debug> embedded_hal::digital::Error for TlaError<I2C> {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
} 

// Typelevel Enum for direction
mod seal {
    pub(super) trait Seal {}
}

#[allow(private_bounds)]
pub trait Mode: seal::Seal {}
pub struct Output {}
impl Mode for Output {}
impl seal::Seal for Output {}
pub struct Input {}
impl Mode for Input {}
impl seal::Seal for Input {}

/// Wrapper allowing the TLA2528's digital outputs to be compatible with the embedded_hal digital
/// traits.
pub struct GpioWrapper<I2C: i2c::I2c, M: Mode> {
    tla: Mutex<Tla2528<I2C>>,
    channel: Channel,
    _mode: PhantomData<M>
}


impl<I2C: i2c::I2c + Debug, M: Mode> embedded_hal::digital::ErrorType for GpioWrapper<I2C, M> {
    type Error = TlaError<I2C>;
}

#[cfg(feature = "gpio_critical_section")]
type Mutex<T> = critical_section::Mutex<RefCell<T>>;
#[cfg(feature = "gpio_critical_section")]
impl<I2C: i2c::I2c> GpioWrapper<I2C, Output> {
    pub fn new(tla: Mutex<Tla2528<I2C>>, channel: Channel) ->  Result<Self, TlaError<I2C>> {
        let tla = Self { tla, channel, _mode: PhantomData };
        critical_section::with(|c| {
            tla.tla.borrow_ref_mut(c).valid_gpo(channel)
        })?;
        Ok(tla)
    }
}
#[cfg(feature = "gpio_critical_section")]
impl<I2C: i2c::I2c + Debug> embedded_hal::digital::OutputPin for GpioWrapper<I2C, Output> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        critical_section::with(|c| {
            self.tla.borrow_ref_mut(c).digital_out(self.channel, false)
        })
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        critical_section::with(|c| {
            self.tla.borrow_ref_mut(c).digital_out(self.channel, true)
        })
    }
}
#[cfg(feature = "gpio_critical_section")]
impl<I2C: i2c::I2c> GpioWrapper<I2C, Input> {
    pub fn new(tla: Mutex<Tla2528<I2C>>, channel: Channel) ->  Result<Self, TlaError<I2C>> {
        let tla = Self { tla, channel, _mode: PhantomData };
        critical_section::with(|c| {
            tla.tla.borrow_ref_mut(c).valid_gpi(channel)
        })?;
        Ok(tla)
    }
}
#[cfg(feature = "gpio_critical_section")]
impl<I2C: i2c::I2c + Debug> embedded_hal::digital::InputPin for GpioWrapper<I2C, Input> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        critical_section::with(|c| {
            self.tla.borrow_ref_mut(c).digital_in(self.channel)
        })
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        critical_section::with(|c| {
            self.tla.borrow_ref_mut(c).digital_in(self.channel)
        }).map(|v| !v)
    }
}


#[cfg(feature = "gpio_atomic")]
compile_error!("Not Yet Implemented");
