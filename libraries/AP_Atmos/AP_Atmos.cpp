#include "AP_Atmos.h"
#include "AP_Atmos_UAVCAN.h"

//这个被ArduPlane/system.cpp   调用
void AP_Atmos::init(void)  
{

    sensor = AP_Atmos_UAVCAN::probe(*this, 1); 
    printf("-----------------------AP_Atmos::init(void)\n");


}

//构造函数在实例化对象的时候进行初始化
AP_Atmos::AP_Atmos()
{
    
}

//  这样调用 应用层的函数 plane.airspeed.get_humidity(temperature);
// get a humidity reading if possible
bool AP_Atmos::get_humidity(float &humidity)
{
     
    if (sensor) {
        return sensor->get_humidity(humidity);
        printf("AP_Atmos::get_humidity = %f\n",humidity);
    }


    return false;
}

//  这样调用 应用层的函数 plane.airspeed.get_sht31_temperature(temperature);
 bool AP_Atmos::get_temperature(float &temperature)
{
     
    if (sensor) {
        return sensor->get_temperature(temperature);
        printf("AP_Atmos::temperature = %f\n",temperature);
    }

    return false;
}
