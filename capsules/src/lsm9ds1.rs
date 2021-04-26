
#![allow(non_camel_case_types)]

use core::cell::Cell;
use enum_primitive::cast::FromPrimitive;
use enum_primitive::enum_from_primitive;
use kernel::common::cells::{OptionalCell, TakeCell};
use kernel::hil::i2c::{self, Error};
use kernel::hil::sensors;
use kernel::{AppId, Callback, Driver, ReturnCode};

use kernel::debug; 

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
        OUT_X_L_M = 0x28,
        OUT_Z_H_M = 0x6A,
        OUT_Z_L_M = 0x6B,
        OUT_Y_H_M = 0x6C,
        OUT_Y_L_M = 0x6D,
    }
}


#[derive(Clone, Copy, PartialEq)]
enum State {
    Idle,
    ReadAccelerationXYZ,
    ReadMagnetometerXYZ,
    SetupAcceleration,
    SetupMagnetometer, 
    ConfirmSetup, 
}


pub struct Lsm9ds1<'a> {
    accel_setup: Cell<bool>,
    magnet_setup: Cell<bool>,
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
            accel_setup: Cell::new(false), 
            magnet_setup: Cell::new(false), 
            config_in_progress: Cell::new(true),
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

    fn configure(&self) {
        self.state.set(State::SetupAcceleration); 
        self.config_in_progress.set(false);
        self.buffer.take().map(|buf| {
            buf[0] = 0x20 as u8; 
            buf[1] = 0x70 as u8; 
            self.i2c_accelerometer.enable(); 
            self.i2c_accelerometer.write(buf, 2); 
        }); 
    }


    fn read_acceleration_xyz(&self) {
        if self.config_in_progress.get() {
            self.configure(); 
        }else if self.state.get() == State::Idle {
            self.state.set(State::ReadAccelerationXYZ);
            self.buffer.take().map(|buf| {
                buf[0] = AccelerometerRegisters::OUT_X_L_A as u8;  //| REGISTER_AUTO_INCREMENT;
                self.i2c_accelerometer.enable();
                debug!("Reading accelerometer"); 
                self.i2c_accelerometer.write_read(buf, 1, 6);
            });
        }
    }

    
    fn read_magnetometer_xyz(&self) {
        if self.state.get() == State::Idle {
            if self.magnet_setup.get() == false {
                self.state.set(State::SetupMagnetometer); 
                self.magnet_setup.set(true); 
                self.buffer.take().map(|buf| {
                    buf[0] = 0x22 as u8; 
                    buf[1] = 0x00 as u8; 
                    self.i2c_magnetometer.enable(); 
                    self.i2c_magnetometer.write(buf, 2); 
                });
            } else {
                self.state.set(State::ReadMagnetometerXYZ);
                self.buffer.take().map(|buf| {
                    buf[0] = MagnetometerRegisters::OUT_X_L_M as u8 | 0x80;
                    self.i2c_magnetometer.enable();
                    debug!("Reading magnetometer"); 
                    self.i2c_magnetometer.write_read(buf, 1, 6);
                });
            }
        }
    }
}


#[macro_use(debug, static_init)]

impl i2c::I2CClient for Lsm9ds1<'_> {
   
    fn command_complete(&self, buffer: &'static mut [u8], error: Error) {
        debug!("LSM9ds1 driver : I2C operation has error {:?}", error); 
        match self.state.get() {
            State::ReadAccelerationXYZ => {
                let mut x: usize = 0;
                let mut y: usize = 0;
                let mut z: usize = 0;

                let values = if error == Error::CommandComplete {
                    self.nine_dof_client.map(|client| {
                        // compute using only integers
                        // let scale_factor = self.accel_scale.get() as usize;
                        // x = (((buffer[0] as i16 | ((buffer[1] as i16) << 8)) as i32)
                        //     * (SCALE_FACTOR[scale_factor] as i32)
                        //     * 1000
                        //     / 32768) as usize;
                        // y = (((buffer[2] as i16 | ((buffer[3] as i16) << 8)) as i32)
                        //     * (SCALE_FACTOR[scale_factor] as i32)
                        //     * 1000
                        //     / 32768) as usize;
                        // z = (((buffer[4] as i16 | ((buffer[5] as i16) << 8)) as i32)
                        //     * (SCALE_FACTOR[scale_factor] as i32)
                        //     * 1000
                        //     / 32768) as usize;
                        x = (buffer[0] as i16 | ((buffer[1] as i16) << 8)) as usize;
                        y = (buffer[2] as i16 | ((buffer[3] as i16) << 8)) as usize;
                        z = (buffer[4] as i16 | ((buffer[5] as i16) << 8)) as usize;
                        client.callback(x, y, z);
                    });

                    x = (buffer[0] as i16 | ((buffer[1] as i16) << 8)) as usize;
                    y = (buffer[2] as i16 | ((buffer[3] as i16) << 8)) as usize;
                    z = (buffer[4] as i16 | ((buffer[5] as i16) << 8)) as usize;
                    true
                } else {
                    self.nine_dof_client.map(|client| {
                        client.callback(1, 1, 1);
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


            State::ReadMagnetometerXYZ => {
                let mut x: usize = 0;
                let mut y: usize = 0;
                let mut z: usize = 0;
                let values = if error == Error::CommandComplete {
                    self.nine_dof_client.map(|client| {
                        // compute using only integers
                        // let range = self.mag_range.get() as usize;
                        // x = (((buffer[1] as i16 | ((buffer[0] as i16) << 8)) as i32) * 100
                        //     / RANGE_FACTOR_X_Y[range] as i32) as usize;
                        // z = (((buffer[3] as i16 | ((buffer[2] as i16) << 8)) as i32) * 100
                        //     / RANGE_FACTOR_X_Y[range] as i32) as usize;
                        // y = (((buffer[5] as i16 | ((buffer[4] as i16) << 8)) as i32) * 100
                        //     / RANGE_FACTOR_Z[range] as i32) as usize;
                        x = ((buffer[1] as u16 | ((buffer[0] as u16) << 8)) as i16) as usize;
                        z = ((buffer[3] as u16 | ((buffer[2] as u16) << 8)) as i16) as usize;
                        y = ((buffer[5] as u16 | ((buffer[4] as u16) << 8)) as i16) as usize;
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

            State::SetupAcceleration => {
                debug!("In setup acceleration"); 
                self.i2c_accelerometer.disable();
                self.state.set(State::Idle);
                self.buffer.replace(buffer);
                self.read_acceleration_xyz(); 
                
            }
            State::SetupMagnetometer => {
                debug!("In setupt magnet"); 
                self.i2c_magnetometer.disable();
                self.state.set(State::Idle); 
                self.buffer.replace(buffer); 
                self.read_magnetometer_xyz(); 
            }
            State::ConfirmSetup => {
                debug!("In confirm setup");
                self.state.set(State::Idle);
                self.i2c_accelerometer.disable();
                let mut x: u8 = 0;
                x = buffer[0]; 
                debug!("Who am I result: {:?}", x); 
                self.nine_dof_client.map(|client| {
                    client.callback(0, 0, 0);
                });
            }

            _ => {
                self.i2c_magnetometer.disable();
                self.i2c_accelerometer.disable();
                self.buffer.replace(buffer);
            }
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






