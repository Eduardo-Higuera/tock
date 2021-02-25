// Nothing here yet


#![allow(non_camel_case_types)]

use core::cell::Cell;
use enum_primitive::cast::FromPrimitive;
use enum_primitive::enum_from_primitive;
use kernel::common::cells::{OptionalCell, TakeCell};
use kernel::hil::i2c::{self, Error};
use kernel::hil::sensors;
use kernel::{AppId, Callback, Driver, ReturnCode};

use crate::driver;
use crate::lsm303xx::{
    AccelerometerRegisters, Lsm303AccelDataRate, Lsm303MagnetoDataRate, Lsm303Range, Lsm303Scale,
    CTRL_REG1, CTRL_REG4, RANGE_FACTOR_X_Y, RANGE_FACTOR_Z, SCALE_FACTOR,
};

/// Syscall driver number.
pub const DRIVER_NUM: usize = driver::NUM::Lsm303dlch as usize;

/// Register values
const REGISTER_AUTO_INCREMENT: u8 = 0x80;


pub static mut BUFFER: [u8; 16]= [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; 

enum_from_primitive! {
    enum MagnetometerRegisters {
        CRA_REG_M = 0x60,
        CRB_REG_M = 0x61,
        OUT_X_H_M = 0x68,
        OUT_X_L_M = 0x69,
        OUT_Z_H_M = 0x6A,
        OUT_Z_L_M = 0x6B,
        OUT_Y_H_M = 0x6C,
        OUT_Y_L_M = 0x6D,
    }
}


#[derive(Clone, Copy, PartialEq)]
enum State {
    Idle,
    IsPresent,
    SetPowerMode,
    SetScaleAndResolution,
    ReadAccelerationXYZ,
    SetDataRate,
    SetRange,
    ReadMagnetometerXYZ,
}


pub struct Lsm9ds1<'a> {
	//i2c_accelerometer: &'a dyn i2c::I2CDevice,
	//state: Cell<State>,
        //buffer: TakeCell<'static, [u8]>,
	//callback: OptionalCell<Callback>,
        //nine_dof_client: OptionalCell<&'a dyn sensors::NineDofClient>,

    config_in_progress: Cell<bool>,
    i2c_accelerometer: &'a dyn i2c::I2CDevice,
    i2c_magnetometer: &'a dyn i2c::I2CDevice,
    callback: OptionalCell<Callback>,
    state: Cell<State>,
    accel_scale: Cell<Lsm303Scale>,
    mag_range: Cell<Lsm303Range>,
    accel_high_resolution: Cell<bool>,
    mag_data_rate: Cell<Lsm303MagnetoDataRate>,
    accel_data_rate: Cell<Lsm303AccelDataRate>,
    low_power: Cell<bool>,
    buffer: TakeCell<'static, [u8]>,
    nine_dof_client: OptionalCell<&'a dyn sensors::NineDofClient>,
}

impl<'a> Lsm9ds1<'a> {

    pub fn new(
        i2c_accelerometer: &'a dyn i2c::I2CDevice,
        i2c_magnetometer: &'a dyn i2c::I2CDevice,
        buffer: &'static mut [u8],
    ) -> Lsm9ds1<'a> {
        // setup and return struct
        Lsm9ds1 {
            config_in_progress: Cell::new(false),
            i2c_accelerometer: i2c_accelerometer,
            i2c_magnetometer: i2c_magnetometer,
            callback: OptionalCell::empty(),
            state: Cell::new(State::Idle),
            accel_scale: Cell::new(Lsm303Scale::Scale2G),
            mag_range: Cell::new(Lsm303Range::Range1G),
            accel_high_resolution: Cell::new(false),
            mag_data_rate: Cell::new(Lsm303MagnetoDataRate::DataRate0_75Hz),
            accel_data_rate: Cell::new(Lsm303AccelDataRate::DataRate1Hz),
            low_power: Cell::new(false),
            buffer: TakeCell::new(buffer),
            nine_dof_client: OptionalCell::empty(),
        }
    }

    pub fn configure(
        &self,
        accel_data_rate: Lsm303AccelDataRate,
        low_power: bool,
        accel_scale: Lsm303Scale,
        accel_high_resolution: bool,
        mag_data_rate: Lsm303MagnetoDataRate,
        mag_range: Lsm303Range,
    ) {
        if self.state.get() == State::Idle {
            self.config_in_progress.set(true);
            self.accel_scale.set(accel_scale);
            self.accel_high_resolution.set(accel_high_resolution);
            self.mag_data_rate.set(mag_data_rate);
            self.mag_range.set(mag_range);
            self.accel_data_rate.set(accel_data_rate);
            self.low_power.set(low_power);
            self.set_power_mode(accel_data_rate, low_power);
        }
    }

    fn is_present(&self) {
        self.state.set(State::IsPresent);
        self.buffer.take().map(|buf| {
            // turn on i2c to send commands
            buf[0] = 0x0F;
            self.i2c_magnetometer.enable();
            self.i2c_magnetometer.write_read(buf, 1, 1);
        });
    }

    fn set_power_mode(&self, data_rate: Lsm303AccelDataRate, low_power: bool) {
        if self.state.get() == State::Idle {
            self.state.set(State::SetPowerMode);
            self.buffer.take().map(|buf| {
                buf[0] = AccelerometerRegisters::CTRL_REG1 as u8;
                buf[1] = (CTRL_REG1::ODR.val(data_rate as u8)
                    + CTRL_REG1::LPEN.val(low_power as u8)
                    + CTRL_REG1::ZEN::SET
                    + CTRL_REG1::YEN::SET
                    + CTRL_REG1::XEN::SET)
                    .value;
                self.i2c_accelerometer.enable();
                self.i2c_accelerometer.write(buf, 2);
            });
        }
    }

    fn set_scale_and_resolution(&self, scale: Lsm303Scale, high_resolution: bool) {
        if self.state.get() == State::Idle {
            self.state.set(State::SetScaleAndResolution);
            // TODO move these in completed
            self.accel_scale.set(scale);
            self.accel_high_resolution.set(high_resolution);
            self.buffer.take().map(|buf| {
                buf[0] = AccelerometerRegisters::CTRL_REG4 as u8;
                buf[1] = (CTRL_REG4::FS.val(scale as u8)
                    + CTRL_REG4::HR.val(high_resolution as u8)
                    + CTRL_REG4::BDU::SET)
                    .value;
                self.i2c_accelerometer.enable();
                self.i2c_accelerometer.write(buf, 2);
            });
        }
    }

    fn read_acceleration_xyz(&self) {
        if self.state.get() == State::Idle {
            self.state.set(State::ReadAccelerationXYZ);
            self.buffer.take().map(|buf| {
                buf[0] = AccelerometerRegisters::OUT_X_L_A as u8 | REGISTER_AUTO_INCREMENT;
                self.i2c_accelerometer.enable();
                self.i2c_accelerometer.write_read(buf, 1, 6);
            });
        }
    }

    fn set_magneto_data_rate(&self, data_rate: Lsm303MagnetoDataRate) {
        if self.state.get() == State::Idle {
            self.state.set(State::SetDataRate);
            self.buffer.take().map(|buf| {
                buf[0] = MagnetometerRegisters::CRA_REG_M as u8;
                buf[1] = ((data_rate as u8) << 2) | 1 << 7;
                self.i2c_magnetometer.enable();
                self.i2c_magnetometer.write(buf, 2);
            });
        }
    }

    fn set_range(&self, range: Lsm303Range) {
        if self.state.get() == State::Idle {
            self.state.set(State::SetRange);
            // TODO move these in completed
            self.mag_range.set(range);
            self.buffer.take().map(|buf| {
                buf[0] = MagnetometerRegisters::CRB_REG_M as u8;
                buf[1] = (range as u8) << 5;
                buf[2] = 0;
                self.i2c_magnetometer.enable();
                self.i2c_magnetometer.write(buf, 3);
            });
        }
    }

    fn read_magnetometer_xyz(&self) {
        if self.state.get() == State::Idle {
            self.state.set(State::ReadMagnetometerXYZ);
            self.buffer.take().map(|buf| {
                buf[0] = MagnetometerRegisters::OUT_X_H_M as u8;
                self.i2c_magnetometer.enable();
                self.i2c_magnetometer.write_read(buf, 1, 6);
            });
        }
    }
}



impl i2c::I2CClient for Lsm9ds1<'_> {
   
    fn command_complete(&self, buffer: &'static mut [u8], error: Error) {
        match self.state.get() {
            State::IsPresent => {
                let present = if error == Error::CommandComplete && buffer[0] == 60 {
                    true
                } else {
                    false
                };

                self.callback.map(|callback| {
                    callback.schedule(if present { 1 } else { 0 }, 0, 0);
                });
                self.buffer.replace(buffer);
                self.i2c_magnetometer.disable();
                self.state.set(State::Idle);
            }
            State::SetPowerMode => {
                let set_power = error == Error::CommandComplete;

                self.callback.map(|callback| {
                    callback.schedule(if set_power { 1 } else { 0 }, 0, 0);
                });
                self.buffer.replace(buffer);
                self.i2c_accelerometer.disable();
                self.state.set(State::Idle);
                if self.config_in_progress.get() {
                    self.set_scale_and_resolution(
                        self.accel_scale.get(),
                        self.accel_high_resolution.get(),
                    );
                }
            }
            State::SetScaleAndResolution => {
                let set_scale_and_resolution = error == Error::CommandComplete;

                self.callback.map(|callback| {
                    callback.schedule(if set_scale_and_resolution { 1 } else { 0 }, 0, 0);
                });
                self.buffer.replace(buffer);
                self.i2c_accelerometer.disable();
                self.state.set(State::Idle);
                if self.config_in_progress.get() {
                    self.set_magneto_data_rate(self.mag_data_rate.get());
                }
            }
            State::ReadAccelerationXYZ => {
                let mut x: usize = 0;
                let mut y: usize = 0;
                let mut z: usize = 0;
                let values = if error == Error::CommandComplete {
                    self.nine_dof_client.map(|client| {
                        // compute using only integers
                        let scale_factor = self.accel_scale.get() as usize;
                        x = (((buffer[0] as i16 | ((buffer[1] as i16) << 8)) as i32)
                            * (SCALE_FACTOR[scale_factor] as i32)
                            * 1000
                            / 32768) as usize;
                        y = (((buffer[2] as i16 | ((buffer[3] as i16) << 8)) as i32)
                            * (SCALE_FACTOR[scale_factor] as i32)
                            * 1000
                            / 32768) as usize;
                        z = (((buffer[4] as i16 | ((buffer[5] as i16) << 8)) as i32)
                            * (SCALE_FACTOR[scale_factor] as i32)
                            * 1000
                            / 32768) as usize;
                        client.callback(x, y, z);
                    });

                    x = (buffer[0] as i16 | ((buffer[1] as i16) << 8)) as usize;
                    y = (buffer[2] as i16 | ((buffer[3] as i16) << 8)) as usize;
                    z = (buffer[4] as i16 | ((buffer[5] as i16) << 8)) as usize;
                    true
                } else {
                    self.nine_dof_client.map(|client| {
                        client.callback(0, 0, 0);
                    });
                    false
                };
                if values {
                    self.callback.map(|callback| {
                        callback.schedule(x, y, z);
                    });
                } else {
                    self.callback.map(|callback| {
                        callback.schedule(0, 0, 0);
                    });
                }
                self.buffer.replace(buffer);
                self.i2c_accelerometer.disable();
                self.state.set(State::Idle);
            }
            State::SetDataRate => {
                let set_magneto_data_rate = error == Error::CommandComplete;

                self.callback.map(|callback| {
                    callback.schedule(if set_magneto_data_rate { 1 } else { 0 }, 0, 0);
                });
                self.buffer.replace(buffer);
                self.i2c_magnetometer.disable();
                self.state.set(State::Idle);
                if self.config_in_progress.get() {
                    self.set_range(self.mag_range.get());
                }
            }
            State::SetRange => {
                let set_range = error == Error::CommandComplete;

                self.callback.map(|callback| {
                    callback.schedule(if set_range { 1 } else { 0 }, 0, 0);
                });
                if self.config_in_progress.get() {
                    self.config_in_progress.set(false);
                }
                self.buffer.replace(buffer);
                self.i2c_magnetometer.disable();
                self.state.set(State::Idle);
            }
            State::ReadMagnetometerXYZ => {
                let mut x: usize = 0;
                let mut y: usize = 0;
                let mut z: usize = 0;
                let values = if error == Error::CommandComplete {
                    self.nine_dof_client.map(|client| {
                        // compute using only integers
                        let range = self.mag_range.get() as usize;
                        x = (((buffer[1] as i16 | ((buffer[0] as i16) << 8)) as i32) * 100
                            / RANGE_FACTOR_X_Y[range] as i32) as usize;
                        z = (((buffer[3] as i16 | ((buffer[2] as i16) << 8)) as i32) * 100
                            / RANGE_FACTOR_X_Y[range] as i32) as usize;
                        y = (((buffer[5] as i16 | ((buffer[4] as i16) << 8)) as i32) * 100
                            / RANGE_FACTOR_Z[range] as i32) as usize;
                        client.callback(x, y, z);
                    });

                    x = ((buffer[1] as u16 | ((buffer[0] as u16) << 8)) as i16) as usize;
                    z = ((buffer[3] as u16 | ((buffer[2] as u16) << 8)) as i16) as usize;
                    y = ((buffer[5] as u16 | ((buffer[4] as u16) << 8)) as i16) as usize;
                    true
                } else {
                    self.nine_dof_client.map(|client| {
                        client.callback(0, 0, 0);
                    });
                    false
                };
                if values {
                    self.callback.map(|callback| {
                        callback.schedule(x, y, z);
                    });
                } else {
                    self.callback.map(|callback| {
                        callback.schedule(0, 0, 0);
                    });
                }
                self.buffer.replace(buffer);
                self.i2c_magnetometer.disable();
                self.state.set(State::Idle);
            }
            _ => {
                self.i2c_magnetometer.disable();
                self.i2c_accelerometer.disable();
                self.buffer.replace(buffer);
            }
        }
    }
}



impl Driver for Lsm9ds1<'_> {
    fn command(&self, command_num: usize, data1: usize, data2: usize, _: AppId) -> ReturnCode {
        match command_num {
            0 => ReturnCode::SUCCESS,
            // Check is sensor is correctly connected
            1 => {
                if self.state.get() == State::Idle {
                    self.is_present();
                    ReturnCode::SUCCESS
                } else {
                    ReturnCode::EBUSY
                }
            }
            // Set Accelerometer Power Mode
            2 => {
                if self.state.get() == State::Idle {
                    if let Some(data_rate) = Lsm303AccelDataRate::from_usize(data1) {
                        self.set_power_mode(data_rate, if data2 != 0 { true } else { false });
                        ReturnCode::SUCCESS
                    } else {
                        ReturnCode::EINVAL
                    }
                } else {
                    ReturnCode::EBUSY
                }
            }
            // Set Accelerometer Scale And Resolution
            3 => {
                if self.state.get() == State::Idle {
                    if let Some(scale) = Lsm303Scale::from_usize(data1) {
                        self.set_scale_and_resolution(scale, if data2 != 0 { true } else { false });
                        ReturnCode::SUCCESS
                    } else {
                        ReturnCode::EINVAL
                    }
                } else {
                    ReturnCode::EBUSY
                }
            }
            // Set Magnetometer Temperature Enable and Data Rate
            4 => {
                if self.state.get() == State::Idle {
                    if let Some(data_rate) = Lsm303MagnetoDataRate::from_usize(data1) {
                        self.set_magneto_data_rate(data_rate);
                        ReturnCode::SUCCESS
                    } else {
                        ReturnCode::EINVAL
                    }
                } else {
                    ReturnCode::EBUSY
                }
            }
            // Set Magnetometer Range
            5 => {
                if self.state.get() == State::Idle {
                    if let Some(range) = Lsm303Range::from_usize(data1) {
                        self.set_range(range);
                        ReturnCode::SUCCESS
                    } else {
                        ReturnCode::EINVAL
                    }
                } else {
                    ReturnCode::EBUSY
                }
            }
            // Read Acceleration XYZ
            6 => {
                if self.state.get() == State::Idle {
                    self.read_acceleration_xyz();
                    ReturnCode::SUCCESS
                } else {
                    ReturnCode::EBUSY
                }
            }
            // Read Mangetometer XYZ
            7 => {
                if self.state.get() == State::Idle {
                    self.read_magnetometer_xyz();
                    ReturnCode::SUCCESS
                } else {
                    ReturnCode::EBUSY
                }
            }
            // default
            _ => ReturnCode::ENOSUPPORT,
        }
    }

    fn subscribe(
        &self,
        subscribe_num: usize,
        callback: Option<Callback>,
        _app_id: AppId,
    ) -> ReturnCode {
        match subscribe_num {
            0 /* set the one shot callback */ => {
				self.callback.insert (callback);
				ReturnCode::SUCCESS
			},
            // default
            _ => ReturnCode::ENOSUPPORT,
        }
    }
}



impl<'a> sensors::NineDof<'a> for Lsm9ds1<'a> {
    fn set_client(&self, nine_dof_client: &'a dyn sensors::NineDofClient) {
        self.nine_dof_client.replace(nine_dof_client);
    }

    fn read_accelerometer(&self) -> ReturnCode {
        if self.state.get() == State::Idle {
            self.read_acceleration_xyz();
            ReturnCode::SUCCESS
        } else {
            ReturnCode::EBUSY
        }
    }

    fn read_magnetometer(&self) -> ReturnCode {
        if self.state.get() == State::Idle {
            self.read_magnetometer_xyz();
            ReturnCode::SUCCESS
        } else {
            ReturnCode::EBUSY
        }
    }

}






