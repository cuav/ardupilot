#include "AP_Atmos.h"
#include "AP_Atmos_UAVCAN.h"


void AP_Atmos::init(void)  
{

    sensor = AP_Atmos_UAVCAN::probe(*this, 1); 
    printf("-----------------------AP_Atmos::init(void)\n");


}


AP_Atmos::AP_Atmos()
{
    
}


bool AP_Atmos::get_humidity(float &humidity)
{
     
    if (sensor) {
        return sensor->get_humidity(humidity);
        printf("AP_Atmos::get_humidity = %f\n",humidity);
    }


    return false;
}


 bool AP_Atmos::get_temperature(float &temperature)
{
     
    if (sensor) {
        return sensor->get_temperature(temperature);
        printf("AP_Atmos::temperature = %f\n",temperature);
    }

    return false;
}
