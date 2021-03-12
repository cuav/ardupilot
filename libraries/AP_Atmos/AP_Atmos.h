


#pragma once


#include "stdio.h"
// #include "AP_Atmos_Backend.h"
// #include "AP_Atmos_UAVCAN.h" //不要包含

//给干活类使用
#ifndef ATMOSPHERE_MAX_SENSORS
#define ATMOSPHERE_MAX_SENSORS  1
#endif

class AP_Atmos_Backend;

class AP_Atmos
{
public:
    friend class AP_Atmos_Backend; //将后台类设为友元类
    
    // constructor
    AP_Atmos();

    void init(void);//初始化函数，选择哪一个传感器作为干活类的传感器 ,ArduPlane/system.cpp


   // get temperature if available
    bool get_temperature(float &temperature);

    bool get_humidity(float &humidity); 


private:
    // current primary sensor
    uint8_t primary = 0;

    AP_Atmos_Backend *sensor;     //和后台联系起来

};




