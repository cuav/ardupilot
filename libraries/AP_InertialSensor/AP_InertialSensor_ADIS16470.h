/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ADIS16470 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_ADIS16470(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                             enum Rotation rotation);
                             
    /*
      initialise driver
     */
    bool init();

    int reset();
    bool self_test_memory();
    bool self_test_sensor();

    uint16_t read_reg16(uint8_t reg);
    void write_reg16(uint8_t reg, uint16_t value);

    void _accumulate();

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::Semaphore *_spi_sem;

    float _temp_filtered;
    
    uint8_t _accel_instance;
    uint8_t _gyro_instance;
    enum Rotation rotation;
    uint8_t temperature_counter;
};
