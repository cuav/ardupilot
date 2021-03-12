#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_Atmos.h"


//这里面提供的是通用的接口，并不只是针对一种传感器
class AP_Atmos_Backend{
public:

    AP_Atmos_Backend(AP_Atmos &frontend, uint8_t instance);
   
    virtual ~AP_Atmos_Backend();

    // return the current humidity in %, if available
    virtual bool get_humidity(float &humidity) = 0;

    // return the current temperature in deg C, if available
    virtual bool get_temperature(float &temperature) = 0;

private:
    AP_Atmos &frontend; //在后台类构造函数中以对象列表的方式被赋值
    uint8_t instance;   //在后台类构造函数中以对象列表的方式被赋值
};