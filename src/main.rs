//! Scd30 command-line utility
//!
//! Copyright 2019 Ryan Kurte

use linux_embedded_hal::{Delay, I2cdev};

use clap::Parser;
use humantime::Duration as HumanDuration;
use log::{debug, error, info, warn};
use simplelog::{LevelFilter, TermLogger};

use sensor_scd30::Scd30;

#[derive(Clone, Debug, Parser)]
#[clap(name = "scd30-util")]
/// A Command Line Interface (CLI) for interacting with a local Scd30 environmental sensor over I2C
pub struct Options {
    /// Specify the i2c interface to use to connect to the scd30 device
    #[clap(short = 'd', long, default_value = "/dev/i2c-1", env = "SCD30_I2C")]
    i2c_device: String,

    /// Specify period for taking measurements
    #[clap(short = 'p', long, default_value = "10s")]
    sample_period: HumanDuration,

    /// Delay between sensor poll operations
    #[clap(long, default_value = "100ms")]
    poll_delay: HumanDuration,

    /// Number of allowed I2C errors (per measurement attempt) prior to exiting
    #[clap(long, default_value = "3")]
    allowed_errors: usize,

    /// Enable verbose logging
    #[clap(long, default_value = "info")]
    log_level: LevelFilter,
}

fn main() {
    // Load options
    let opts = Options::parse();

    // Setup logging
    TermLogger::init(opts.log_level, simplelog::Config::default()).unwrap();

    debug!("Connecting to I2C device");
    let i2c = match I2cdev::new(&opts.i2c_device) {
        Ok(v) => v,
        Err(e) => {
            error!("Error opening I2C device '{}': {:?}", &opts.i2c_device, e);
            std::process::exit(-1);
        }
    };

    debug!("Connecting to SCD30");
    let mut sensor = match Scd30::new(i2c, Delay {}) {
        Ok(v) => v,
        Err(e) => {
            error!("Error connecting to SCD30: {:?}", e);
            std::process::exit(-2);
        }
    };

    debug!("Starting sensor polling");
    if let Err(e) = sensor.start_continuous(opts.sample_period.as_secs() as u16) {
        error!("Error starting continuous mode: {:?}", e);
        std::process::exit(-3);
    }

    debug!("Waiting for sensor to initialise");
    std::thread::sleep(*opts.sample_period);

    loop {
        debug!("Starting sensor read cycle");

        let mut ready = false;
        let mut errors = 0;

        // Poll for sensor ready
        for _i in 0..100 {
            match sensor.data_ready() {
                Ok(true) => {
                    ready = true;
                    break;
                }
                Ok(false) => {
                    std::thread::sleep(*opts.poll_delay);
                }
                Err(e) => {
                    warn!("Error polling for sensor ready: {:?}", e);
                    errors += 1;
                }
            };

            if errors > opts.allowed_errors {
                error!("Exceeded maximum allowed I2C errors");
                std::process::exit(-4);
            }
        }

        debug!("Sensor data ready state: {:?}", ready);

        if !ready {
            warn!("Sensor data ready timed-out");
            std::thread::sleep(*opts.sample_period);
            continue;
        }

        // If we're ready, attempt to read the data
        for _i in 0..10 {
            match sensor.read_data() {
                Ok(m) => {
                    info!(
                        "CO2: {:.2} ppm, Temperature: {:.2} C, Humidity: {:.2} %",
                        m.co2, m.temp, m.rh
                    );
                    break;
                }
                Err(e) => {
                    warn!("Error reading sensor data: {:?}", e);
                    errors += 1;
                }
            }

            if errors > opts.allowed_errors {
                error!("Exceeded maximum allowed I2C errors");
                std::process::exit(-5);
            }
        }

        // Wait for enough time for another sensor reading
        std::thread::sleep(*opts.sample_period);
    }
}
