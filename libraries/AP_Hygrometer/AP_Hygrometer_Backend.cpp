#include "AP_Hygrometer_Backend.h"
#include "AP_Hygrometer.h"


AP_Hygrometer_Backend::AP_Hygrometer_Backend(AP_Hygrometer &_frontend, uint8_t _instance) :
    frontend(_frontend),
    instance(_instance)
{
}


AP_Hygrometer_Backend::~AP_Hygrometer_Backend(void)
{
} 
