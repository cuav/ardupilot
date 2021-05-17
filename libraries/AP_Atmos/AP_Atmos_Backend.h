#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_Atmos.h"


class AP_Atmos_Backend{
public:

    AP_Atmos_Backend(AP_Atmos &frontend, uint8_t instance);
   
    virtual ~AP_Atmos_Backend();

    // return the current humidity in %, if available
    virtual bool get_humidity(float &humidity) = 0;

    // return the current temperature in deg C, if available
    virtual bool get_temperature(float &temperature) = 0;

private:
    AP_Atmos &frontend; 
    uint8_t instance;   
};