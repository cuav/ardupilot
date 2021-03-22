#pragma once

#include "stdio.h"

#ifndef ATMOSPHERE_MAX_SENSORS
#define ATMOSPHERE_MAX_SENSORS  1
#endif

class AP_Atmos_Backend;

class AP_Atmos
{
public:
    friend class AP_Atmos_Backend; 
    
    // constructor
    AP_Atmos();

    void init(void);


   // get temperature if available
    bool get_temperature(float &temperature);

    bool get_humidity(float &humidity); 


private:
    // current primary sensor
    uint8_t primary = 0;

    AP_Atmos_Backend *sensor;    

};




