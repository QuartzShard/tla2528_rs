use core::{marker::PhantomData};
#[cfg(feature = "gpio_rtic_arbiter")]
use embedded_hal::i2c::ErrorType;

use super::*;

mutually_exclusive_features::none_or_one_of!("gpio_critical_section", "gpio_atomic", "gpio_rtic_arbiter");

impl<I2C: i2c::ErrorType> embedded_hal::digital::Error for TlaError<I2C> {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
} 

impl<I2C: i2c::ErrorType> embedded_hal::i2c::Error for TlaError<I2C> {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
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

/// Wrapper allowing the TLA2528's digital outputs to be compatible with the `embedded_hal` digital
/// traits.
pub struct GpioWrapper<'a, I2C, M: Mode, IM: super::Mode> {
    tla: &'a Mutex<Tla2528<I2C, IM>>,
    channel: Channel,
    _mode: PhantomData<M>
}

pub trait AsyncInputPinAdapter: embedded_hal_async::i2c::ErrorType {
    fn is_high(&mut self) -> impl Future<Output = Result<bool, Self::Error>>;
    fn is_low(&mut self) -> impl Future<Output = Result<bool, Self::Error>> {
        async move {
            self.is_high().await.map(|p| !p)
        }    
    }
}

pub trait AsyncOutputPinAdapter: embedded_hal_async::i2c::ErrorType {
    fn set_high(&mut self) -> impl Future<Output = Result<bool, Self::Error>>;
    fn set_low(&mut self) -> impl Future<Output = Result<bool, Self::Error>>;
}

impl<I2C: i2c::ErrorType, M: Mode, IM: super::Mode> embedded_hal::digital::ErrorType for GpioWrapper<'_, I2C, M, IM> {
    type Error = TlaError<I2C>;
}
impl<I2C: i2c::ErrorType, M: Mode, IM: super::Mode> embedded_hal::i2c::ErrorType for GpioWrapper<'_, I2C, M, IM> {
    type Error = TlaError<I2C>;
}

#[cfg(feature = "gpio_critical_section")]
use core::cell::RefCell;
#[cfg(feature = "gpio_critical_section")]
type Mutex<T> = critical_section::Mutex<RefCell<T>>;

#[cfg(feature = "gpio_critical_section")]
impl<'a, I2C: i2c::I2c> GpioWrapper<'a, I2C, Output, Sync> {
    pub fn new(tla: &'a Mutex<Tla2528<I2C, Sync>>, channel: Channel) ->  Result<Self, TlaError<I2C>> {
        let tla = Self { tla, channel, _mode: PhantomData };
        critical_section::with(|c| {
            tla.tla.borrow_ref_mut(c).valid_gpo(channel)
        })?;
        Ok(tla)
    }
}
#[cfg(feature = "gpio_critical_section")]
impl<I2C: i2c::I2c> embedded_hal::digital::OutputPin for GpioWrapper<'_, I2C, Output, Sync> {
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
impl<'a, I2C: i2c::I2c> GpioWrapper<'a, I2C, Input, Sync> {
    pub fn new(tla: &'a Mutex<Tla2528<I2C, Sync>>, channel: Channel) ->  Result<Self, TlaError<I2C>> {
        let tla = Self { tla, channel, _mode: PhantomData };
        critical_section::with(|c| {
            tla.tla.borrow_ref_mut(c).valid_gpi(channel)
        })?;
        Ok(tla)
    }
}
#[cfg(feature = "gpio_critical_section")]
impl<I2C: i2c::I2c> embedded_hal::digital::InputPin for GpioWrapper<'_, I2C, Input, Sync> {
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

#[cfg(feature = "gpio_rtic_arbiter")]
type Mutex<T> = rtic_sync::arbiter::Arbiter<T>;


#[cfg(feature = "gpio_rtic_arbiter")]
impl<'a, I2C: embedded_hal_async::i2c::I2c> GpioWrapper<'a, I2C, Output, Async> {
    pub async fn new(tla: &'a Mutex<Tla2528<I2C, Async>>, channel: Channel) ->  Result<Self, TlaError<I2C>> {
        let tla = Self { tla, channel, _mode: PhantomData };
        tla.tla.access().await.valid_gpo(channel).await?;
        Ok(tla)
    }
    pub async fn set_low(&mut self) -> Result<(), <Self as ErrorType>::Error> {
        self.tla.access().await.digital_out(self.channel, false).await
    }

    pub async fn set_high(&mut self) -> Result<(), <Self as ErrorType>::Error> {
        self.tla.access().await.digital_out(self.channel, true).await
    }
}
#[cfg(feature = "gpio_rtic_arbiter")]
impl<'a, I2C: embedded_hal_async::i2c::I2c> GpioWrapper<'a, I2C, Input, Async> {
    pub async fn new(tla: &'a Mutex<Tla2528<I2C, Async>>, channel: Channel) ->  Result<Self, TlaError<I2C>> {
        let tla = Self { tla, channel, _mode: PhantomData };
        tla.tla.access().await.valid_gpi(channel).await?;
        Ok(tla)
    }
    pub async fn is_high(&mut self) -> Result<bool, <Self as ErrorType>::Error> {
        self.tla.access().await.digital_in(self.channel).await
    }

    pub async fn is_low(&mut self) -> Result<bool, <Self as ErrorType>::Error> {
        self.tla.access().await.digital_in(self.channel).await
    }
}


#[cfg(feature = "gpio_atomic")]
compile_error!("Not Yet Implemented");
