use core::cell::RefCell;
use super::*;

mutually_exclusive_features::exactly_one_of!("gpio_critical_section", "gpio_atomic");

impl<I2C: i2c::I2c + Debug> embedded_hal::digital::Error for TlaError<I2C> {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
} 

/// Wrapper allowing the TLA2528's digital outputs to be compatible with the embedded_hal digital
/// traits.
pub struct GpioWrapper<I2C: i2c::I2c> {
    tla: Mutex<Tla2528<I2C>>,
    channel: Channel
}

impl<I2C: i2c::I2c + Debug> embedded_hal::digital::ErrorType for GpioWrapper<I2C> {
    type Error = TlaError<I2C>;
}

#[cfg(feature = "gpio_critical_section")]
type Mutex<T> = critical_section::Mutex<RefCell<T>>;
#[cfg(feature = "gpio_critical_section")]
impl<I2C: i2c::I2c + Debug> embedded_hal::digital::OutputPin for GpioWrapper<I2C> {
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


#[cfg(feature = "gpio_atomic")]
compile_error!("Not Yet Implemented");
