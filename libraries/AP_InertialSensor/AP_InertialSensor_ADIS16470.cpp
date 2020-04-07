/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_ADIS16470.h"


static const float GYRO_SCALE = (0.0174532f / 10);
static const float ACCEL_SCALE = (GRAVITY_MSS / 800);
#define BACKEND_SAMPLE_RATE 1000
/*
  device registers, names follow datasheet conventions, with REGA_
  prefix for accel, and REGG_ prefix for gyro
 */
//  ADIS16470 registers
static constexpr uint8_t DIAG_STAT	= 0x02; // Output, system error flags
static constexpr uint8_t FILT_CTRL	= 0x5C;
static constexpr uint8_t MSC_CTRL	= 0x60;
static constexpr uint8_t DEC_RATE	= 0x64;
static constexpr uint8_t GLOB_CMD	= 0x68;
static constexpr uint8_t PROD_ID	= 0x72;

static constexpr uint16_t PROD_ID_ADIS16470 = 0x4056;	// ADIS16470 Identification, device number

extern const AP_HAL::HAL& hal;

#define int16_val(v)  ((int16_t)(((uint16_t)v[0] << 8) | v[1]))
#define uint16_val(v) (((uint16_t)v[0] << 8) | v[1])

AP_InertialSensor_ADIS16470::AP_InertialSensor_ADIS16470(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                   enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_ADIS16470::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_ADIS16470(imu, std::move(dev), rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_ADIS16470::start()
{
    _gyro_instance = _imu.register_gyro(BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_ADIS16470));
    _accel_instance = _imu.register_accel(BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_ADIS16470));

    set_accel_orientation(_accel_instance, rotation);
    set_gyro_orientation(_gyro_instance, rotation);

    _dev->register_periodic_callback(1000000UL/BACKEND_SAMPLE_RATE, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS16470::_accumulate, void));
}

bool AP_InertialSensor_ADIS16470::init()
{
    _spi_sem = _dev->get_semaphore();
    _spi_sem->take_blocking();
    _dev->set_read_flag(0);

    uint16_t product_id;
    uint8_t tries;

    for(tries = 0; tries < 5; tries++) {
        product_id = read_reg16(PROD_ID);
        if(product_id == PROD_ID_ADIS16470) {
            break;
        }
    }
    
    if(tries == 5 || !reset()) {
        goto failed;
    }

    for(tries = 0; tries < 5; tries++) {
        if(self_test_memory() && self_test_sensor()) {
            break;
        } else {
            reset();
        }
    }

    if(tries == 5) {
        goto failed;
    }

    hal.console->printf("ADIS1670: Found at spi 0x%x\n", _dev->bus_num());
    _spi_sem->give();
    return true;

failed:
    _spi_sem->give();
    return false;
}

bool AP_InertialSensor_ADIS16470::self_test_memory()
{
	// self test (global command bit 4)
	uint8_t value[2] {};

	value[0] = (1 << 4);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);
    hal.scheduler->delay(32);

	// read DIAG_STAT to check result
	uint16_t diag_stat = read_reg16(DIAG_STAT);

	if (diag_stat != 0) {
		return false;
	}

	return true;
}

bool AP_InertialSensor_ADIS16470::self_test_sensor()
{
	// self test (global command bit 2)
	uint8_t value[2] {};

	value[0] = (1 << 2);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);
    hal.scheduler->delay(14);

	// read DIAG_STAT to check result
	uint16_t diag_stat = read_reg16(DIAG_STAT);

	if (diag_stat != 0) {
		return false;
	}
	return true;
}

int AP_InertialSensor_ADIS16470::reset()
{
    uint8_t value[2] {};

	value[0] = (1 << 7);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);
    hal.scheduler->delay(300);
    
	// Miscellaneous Control Register (MSC_CTRL)
	static constexpr uint16_t MSC_CTRL_DEFAULT = 0x00C1;
    write_reg16(MSC_CTRL, MSC_CTRL_DEFAULT);
    hal.scheduler->delay(1);
	// verify
	const uint16_t msc_ctrl = read_reg16(MSC_CTRL);
	if (msc_ctrl != MSC_CTRL_DEFAULT) {
		return false;
	}

    static constexpr uint16_t FILT_CTRL_SETUP = 0x0004; // (disabled: 0x0000, 2 taps: 0x0001, 16 taps: 0x0004)
	write_reg16(FILT_CTRL, FILT_CTRL_SETUP);
	hal.scheduler->delay(1);

    // verify
	const uint16_t filt_ctrl = read_reg16(FILT_CTRL);

    if (filt_ctrl != FILT_CTRL_SETUP) {
		return false;
	}

	// Decimation Filter
	//  set for 1000 samples per second
	static constexpr uint16_t DEC_RATE_SETUP = 0x0001;
	write_reg16(DEC_RATE, DEC_RATE_SETUP);
	hal.scheduler->delay(1);

    // verify
	const uint16_t dec_rate = read_reg16(DEC_RATE);
	if (dec_rate != DEC_RATE_SETUP) {
		return false;
	}
	return true;
}

bool AP_InertialSensor_ADIS16470::update()
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);

    return true;
}

void AP_InertialSensor_ADIS16470::_accumulate()
{
    struct ADISReport {
		uint8_t diag_stat[2];
		uint8_t	gyro_x[2];
		uint8_t	gyro_y[2];
		uint8_t	gyro_z[2];
		uint8_t	accel_x[2];
		uint8_t	accel_y[2];
		uint8_t	accel_z[2];
		uint8_t	temp[2];
		uint8_t	DATA_CNTR[2];
		uint8_t	checksum[2];
	};

    uint8_t cmd[2] {};
    cmd[0] = GLOB_CMD;
    _dev->transfer(cmd, 2, nullptr, 0);

    ADISReport adis_report{0}; 
    _dev->transfer(cmd, 0, (uint8_t *)&adis_report, (sizeof(adis_report) / sizeof(uint8_t)));

    uint8_t *checksum_helper = (uint8_t *)&adis_report.diag_stat;
	uint16_t checksum = 0;

	for (int i = 0; i < 18; i++) {
		checksum += checksum_helper[i];
	}

    if (int16_val(adis_report.checksum) != checksum) {
        _inc_gyro_error_count(_gyro_instance);
        _inc_accel_error_count(_accel_instance);
        return;
    }

    Vector3f accel_data(int16_val(adis_report.accel_y), 
                        int16_val(adis_report.accel_x), 
                        -int16_val(adis_report.accel_z));

    Vector3f gyro_data(int16_val(adis_report.gyro_y), 
                       int16_val(adis_report.gyro_x), 
                       -int16_val(adis_report.gyro_z));

    _temp_filtered = int16_val(adis_report.temp) * 0.1f;

    accel_data *= ACCEL_SCALE;
    gyro_data *= GYRO_SCALE;

    _rotate_and_correct_accel(_accel_instance, accel_data);
    _notify_new_accel_raw_sample(_accel_instance, accel_data);

    _rotate_and_correct_gyro(_gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro_data);

    _publish_temperature(_accel_instance, _temp_filtered);
}

uint16_t AP_InertialSensor_ADIS16470::read_reg16(uint8_t reg)
{
    uint8_t cmd[2] {};

    cmd[0] = reg;
    _dev->transfer(cmd, 2, nullptr, 0);
    hal.scheduler->delay_microseconds_boost(16);
    _dev->transfer(nullptr, 0, cmd, 2);
    hal.scheduler->delay_microseconds_boost(16);
    return uint16_val(cmd);
}

void AP_InertialSensor_ADIS16470::write_reg16(uint8_t reg, uint16_t value)
{
    uint8_t cmd[2] {};

    cmd[0] = reg|0x80;
    cmd[1] =  (0x00ff & value);
    _dev->transfer(cmd, 2, nullptr, 0);
    hal.scheduler->delay_microseconds_boost(16);
    cmd[0] =(reg + 1)|0x80;
    cmd[1] = ((0xff00 & value) >> 8);
    _dev->transfer(cmd, 2, nullptr, 0);
    hal.scheduler->delay_microseconds_boost(16);
}
