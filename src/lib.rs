#![no_std]

use core::fmt::Debug;

use bilge::prelude::*;
use embedded_hal::i2c;

#[cfg(feature = "gpio")]
mod gpio;
#[cfg(feature = "gpio")]
pub use gpio::*;

// Registers
pub const SYSTEM_STATUS: u8 = 0x00; // Reset 0x80
pub const GENERAL_CFG: u8 = 0x01; // Reset 0x00
pub const DATA_CFG: u8 = 0x02; // Reset 0x00
pub const OSR_CFG: u8 = 0x03; // Reset 0x00
pub const OPMODE_CFG: u8 = 0x04; // Reset 0x00
pub const PIN_CFG: u8 = 0x05; // Reset 0x00
pub const GPIO_CONFIG: u8 = 0x07; // Reset 0x00
pub const GPO_DRIVE_CONFIG: u8 = 0x09; // Reset 0x00
pub const GPO_VALUE: u8 = 0x0B; // Reset 0x00
pub const GPI_VALUE: u8 = 0x0D; // Reset 0x00
pub const SEQUENCE_CFG: u8 = 0x10; // Reset 0x00
pub const CHANNEL_SEL: u8 = 0x11; // Reset 0x00
pub const AUTO_SEQ_CH_SEL: u8 = 0x12; // Reset 0x00

// I2C Opcodes
pub enum Opcode {
	SingleRead      = 0x10,
	SingleWrite     = 0x08,
	SetBit          = 0x18,
	ClearBit        = 0x20,
	ReadContinuous  = 0x30,
	WriteContinuous = 0x28,
}

// Channels
#[bitsize(4)]
#[derive(FromBits, Clone, Copy)]
pub enum Channel {
	Ch0,
	Ch1,
	Ch2,
	Ch3,
	Ch4,
	Ch5,
	Ch6,
	Ch7,
	#[fallback]
	Reserved,
}

// Impl

/// Driver for the [tla2528](https://www.ti.com/lit/ds/symlink/tla2528.pdf?ts=1759289066702) I2C ADC
pub struct Tla2528<I2C>
where
	I2C: i2c::I2c,
{
	addr:   u8,
	i2c:    I2C,
	config: Config,
}

impl<I2C: i2c::I2c> Tla2528<I2C> {
	/// Construct and init a new [Tla2528]
	///
	/// # Errors
	///
	/// This function will return an error if the I2C comms error, or the status read-back is
	/// incorrect
	pub fn new(addr: u8, i2c: I2C, config: Config) -> Result<Self, TlaError<I2C>> {
		let mut adc = Self {
			addr,
			i2c,
			config: Config::default(),
		};

		adc.write_config(config)?;

		let status = adc.read_status()?;
		if !status.rsvd() || status.crc_err_fuse() {
			return Err(TlaError::InitFail);
		};

		Ok(adc)
	}

	pub fn read_status(&mut self) -> Result<SystemStatus, TlaError<I2C>> {
		Ok(SystemStatus::from(self.read_reg(SYSTEM_STATUS)?))
	}

	/// Read from a specific ADC channel, once
	pub fn analog_read_manual(&mut self, channel: Channel) -> Result<u16, TlaError<I2C>> {
		let channel = channel as u8;
		if let SeqMode::Automatic = self.config.sequence.seq_mode() {
			return Err(TlaError::WrongMode);
		};
		if self.config.pin.val_0_at(channel as usize) {
			return Err(TlaError::AnalogFromDigital);
		}

		self.write_reg(CHANNEL_SEL, channel)?;
		Ok(self.analog_read()?.0)
	}

	/// Read from all active ADC channels in order
	pub fn analog_read_auto<'buf>(
		&mut self,
		reads: &'buf mut [(u16, Option<Channel>)],
	) -> Result<&'buf [(u16, Option<Channel>)], TlaError<I2C>> {
		if let SeqMode::Manual = self.config.sequence.seq_mode() {
			return Err(TlaError::WrongMode);
		};
		self.set_bit(SEQUENCE_CFG, 0x10)?;
		for &mut (ref mut val, ref mut channel) in reads.iter_mut() {
			(*val, *channel) = self.analog_read()?;
		}
		self.clear_bit(SEQUENCE_CFG, 0x10)?;
		Ok(reads)
	}

	/// Shared impl for reads. Handles different buffer lengths caused by different config
	fn analog_read(&mut self) -> Result<(u16, Option<Channel>), TlaError<I2C>> {
		let channel_append = self.config.data.append_status();
		let averaging = if let OversamplingRatio::_0 = self.config.osr.osr() {
			false
		} else {
			true
		};

		let mut buf = [0u8; 3];

		let buf = match (&channel_append, averaging) {
			(AppendStatus::Yes, true) => &mut buf[..3],
			_ => &mut buf[..2],
		};

		self.i2c
			.read(self.addr, buf)
			.map_err(|e| TlaError::I2c(e))?;

		let read = if averaging {
			(buf[0] as u16) << 8 | buf[1] as u16
		} else {
			(buf[0] as u16) << 4 | (buf[1] as u16) >> 4
		};

		let channel = match (&channel_append, averaging) {
			(AppendStatus::Yes, true) => Some(Channel::from(u4::from(buf[2] >> 4))),
			(AppendStatus::Yes, false) => Some(Channel::from(u4::from(buf[1] & 0x0F))),
			(AppendStatus::No, _) => None,
			(AppendStatus::Invalid, _) => None,
		};
		Ok((read, channel))
	}

	/// Read a specific GPIO
	pub fn digital_in(&mut self, channel: Channel) -> Result<bool, TlaError<I2C>> {
        self.valid_gpi(channel)?;
		let channel = channel as u8;
		Ok((self.read_reg(GPI_VALUE)? & 1 << channel).count_ones() == 1)
	}

    pub fn valid_gpi(&mut self, channel: Channel) -> Result<(), TlaError<I2C>> {
		let channel = channel as usize;
		if !self.config.pin.val_0_at(channel) {
			return Err(TlaError::DigitalFromAnalog);
		};
		if self.config.gpio.val_0_at(channel) {
			return Err(TlaError::WrongGPIODirection);
		};
        Ok(())
    }

	/// Read all GPIO. Pins that are not configured as Digital In will be masked to 0
	pub fn read_gpio(&mut self) -> Result<u8, TlaError<I2C>> {
		let mask = self.config.pin.value & !self.config.gpio.value;
		Ok(self.read_reg(GPI_VALUE)? & mask)
	}

	/// Set a GPIO out
	pub fn digital_out(&mut self, channel: Channel, val: bool) -> Result<(), TlaError<I2C>> {
        self.valid_gpo(channel)?;
		let channel = channel as u8;
		if val {
			self.set_bit(GPO_VALUE, 1 << channel)
		} else {
			self.clear_bit(GPO_VALUE, 1 << channel)
		}
	}

    pub fn valid_gpo(&mut self, channel: Channel) -> Result<(), TlaError<I2C>> {
		let channel = channel as usize;
		if !self.config.pin.val_0_at(channel) {
			return Err(TlaError::DigitalFromAnalog);
		};
		if !self.config.gpio.val_0_at(channel) {
			return Err(TlaError::WrongGPIODirection);
		};
        Ok(())
    }

	/// Write a new value to a register
	fn write_reg(&mut self, addr: u8, val: u8) -> Result<(), TlaError<I2C>> {
		self.i2c
			.write(self.addr, &[Opcode::SingleWrite as u8, addr, val])
			.map_err(|e| TlaError::I2c(e))?;
		Ok(())
	}

	/// Read the contents of a register
	fn read_reg(&mut self, addr: u8) -> Result<u8, TlaError<I2C>> {
		let mut buf = [0u8];
		self.i2c
			.write_read(self.addr, &[Opcode::SingleRead as u8, addr], &mut buf)
			.map_err(|e| TlaError::I2c(e))?;
		Ok(buf[0])
	}

	/// Set bits in a register. Does not modify bits that are set to 0 in the arg
	fn set_bit(&mut self, addr: u8, bitmask: u8) -> Result<(), TlaError<I2C>> {
		self.i2c
			.write(self.addr, &[Opcode::SetBit as u8, addr, bitmask])
			.map_err(|e| TlaError::I2c(e))?;
		Ok(())
	}

	/// Clear bits in a register. Does not modify bits that are set to 0 in the arg
	fn clear_bit(&mut self, addr: u8, bitmask: u8) -> Result<(), TlaError<I2C>> {
		self.i2c
			.write(self.addr, &[Opcode::ClearBit as u8, addr, bitmask])
			.map_err(|e| TlaError::I2c(e))?;
		Ok(())
	}

	/// Write the entire config to the ADC
	pub fn write_config(&mut self, config: Config) -> Result<(), TlaError<I2C>> {
		self.write_general_config(config.general)?;
		self.write_data_config(config.data)?;
		self.write_osr_config(config.osr)?;
		self.write_opmode_config(config.opmode)?;
		self.write_pin_config(config.pin)?;
		self.write_gpio_config(config.gpio)?;
		self.write_gpo_drive_config(config.gpo_drive)?;
		Ok(())
	}

	pub fn write_general_config(&mut self, config: GeneralConfig) -> Result<(), TlaError<I2C>> {
		self.config.general = config;
		self.write_reg(GENERAL_CFG, self.config.general.value)
	}

	pub fn write_data_config(&mut self, config: DataConfig) -> Result<(), TlaError<I2C>> {
		self.config.data = config;
		self.write_reg(DATA_CFG, self.config.data.value)
	}

	pub fn write_osr_config(&mut self, config: OsrConfig) -> Result<(), TlaError<I2C>> {
		self.config.osr = config;
		self.write_reg(OSR_CFG, self.config.osr.value)
	}

	pub fn write_opmode_config(&mut self, config: OpmodeConfig) -> Result<(), TlaError<I2C>> {
		self.config.opmode = config;
		self.write_reg(OPMODE_CFG, self.config.opmode.value)
	}

	pub fn write_pin_config(&mut self, config: PinConfig) -> Result<(), TlaError<I2C>> {
		self.config.pin = config;
		self.write_reg(PIN_CFG, self.config.pin.value)
	}

	pub fn write_gpio_config(&mut self, config: GpioConfig) -> Result<(), TlaError<I2C>> {
		self.config.gpio = config;
		self.write_reg(GPIO_CONFIG, self.config.gpio.value)
	}

	pub fn write_gpo_drive_config(&mut self, config: GpoDriveConfig) -> Result<(), TlaError<I2C>> {
		self.config.gpo_drive = config;
		self.write_reg(GPO_DRIVE_CONFIG, self.config.gpo_drive.value)
	}
}

// Config register bitmaps

#[derive(Default)]
pub struct Config {
	general:   GeneralConfig,
	data:      DataConfig,
	osr:       OsrConfig,
	opmode:    OpmodeConfig,
	pin:       PinConfig,
	gpio:      GpioConfig,
	gpo_drive: GpoDriveConfig,
	sequence:  SequenceConfig,
}

#[bitsize(8)]
#[derive(FromBits)]
pub struct SystemStatus {
	rsvd:         bool,
	seq_status:   bool,
	i2c_speed:    bool,
	reserved:     bool,
	osr_done:     bool,
	crc_err_fuse: bool,
	reserved:     bool,
	bor:          bool,
}

impl Default for SystemStatus {
	fn default() -> Self {
		Self::from(0x80)
	}
}

#[bitsize(8)]
#[derive(Default, FromBits)]
pub struct GeneralConfig {
	reserved: u4,
	cnvst:    bool,
	ch_rst:   bool,
	cal:      bool,
	rst:      bool,
}

#[bitsize(8)]
#[derive(Default, FromBits)]
pub struct DataConfig {
	fix_pat:       bool,
	reserved:      u1,
	append_status: AppendStatus,
	reserved:      u4,
}

#[bitsize(2)]
#[derive(FromBits, Default)]
pub enum AppendStatus {
	#[default]
	No,
	Yes,
	#[fallback]
	Invalid,
}

#[bitsize(8)]
#[derive(Default, FromBits)]
pub struct OsrConfig {
	reserved: u5,
	osr:      OversamplingRatio,
}

#[bitsize(3)]
#[derive(FromBits, Default)]
pub enum OversamplingRatio {
	#[default]
	_0,
	_2,
	_4,
	_8,
	_16,
	_32,
	_64,
	_128,
}

#[bitsize(8)]
#[derive(Default, FromBits)]
pub struct OpmodeConfig {
	reserved: u3,
	osc_sel:  OscSel,
	clk_div:  ClkDiv,
}

#[bitsize(1)]
#[derive(Default, FromBits)]
pub enum OscSel {
	#[default]
	_1MHz,
	_32KHz,
}

#[bitsize(4)]
#[derive(Default, FromBits)]
pub enum ClkDiv {
	#[default]
	_1,
	_1_5,
	_2,
	_3,
	_4,
	_6,
	_8,
	_12,
	_16,
	_24,
	_32,
	_48,
	_64,
	_96,
	_128,
	_192,
}

#[bitsize(8)]
#[derive(Default, FromBits)]
/// 0 for AIN, 1 for GPIO
pub struct PinConfig([bool; 8]);

#[bitsize(8)]
#[derive(Default, FromBits)]
/// 0 for In, 1 for Out
pub struct GpioConfig([bool; 8]);

#[bitsize(8)]
#[derive(Default, FromBits)]
/// 0 for Open-Drain, 1 for Push-Pull
pub struct GpoDriveConfig([bool; 8]);

#[bitsize(8)]
#[derive(FromBits)]
/// Output
pub struct GpoValue([bool; 8]);

#[bitsize(8)]
#[derive(FromBits)]
// Input
pub struct GpiValue([bool; 8]);

#[bitsize(8)]
#[derive(Default, FromBits)]
pub struct SequenceConfig {
	reserved:  u3,
	seq_start: bool,
	reserved:  u2,
	seq_mode:  SeqMode,
}

#[bitsize(2)]
#[derive(Default, FromBits)]
pub enum SeqMode {
	#[default]
	Manual,
	Automatic,
	#[fallback]
	Invalid,
}

#[bitsize(8)]
#[derive(FromBits)]
pub struct ChannelSelect {
	reserved:    u4,
	manual_chid: Channel,
}

#[bitsize(8)]
#[derive(FromBits)]
pub struct AutoSeqChannelSelect([bool; 8]);

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TlaError<I2C: i2c::I2c> {
	I2c(I2C::Error),
	InitFail,
	WrongMode,
	AnalogFromDigital,
	DigitalFromAnalog,
	WrongGPIODirection,
}

