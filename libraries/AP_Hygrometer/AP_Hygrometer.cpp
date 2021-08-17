#include "AP_Hygrometer.h"
#include "AP_Hygrometer_UAVCAN.h"


void AP_Hygrometer::init(void)  
{

    sensor = AP_Hygrometer_UAVCAN::probe(*this, 1); 
    printf("-----------------------AP_Hygrometer::init(void)\n");


}


AP_Hygrometer::AP_Hygrometer()
{

}


bool AP_Hygrometer::get_humidity(float &humidity)
{

    if (sensor) {
        return sensor->get_humidity(humidity);
        printf("AP_Hygrometer::get_humidity = %f\n",(float)humidity);
    }


    return false;
}


 bool AP_Hygrometer::get_temperature(float &temperature)
{

    if (sensor) {
        return sensor->get_temperature(temperature);
        printf("AP_Hygrometer::temperature = %f\n",(float)temperature);
    }

    return false;
}

bool AP_Hygrometer::get_id(uint8_t &id)
{

    if (sensor) {
        return sensor->get_id(id);
        printf("AP_Hygrometer::get_id = %d\n",id);
    }


    return false;
}