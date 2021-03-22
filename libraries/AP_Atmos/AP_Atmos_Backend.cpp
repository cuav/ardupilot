
#include "AP_Atmos_Backend.h"
#include "AP_Atmos.h"

AP_Atmos_Backend::AP_Atmos_Backend(AP_Atmos &_frontend, uint8_t _instance) :
    frontend(_frontend),
    instance(_instance)
{
}


AP_Atmos_Backend::~AP_Atmos_Backend(void)
{
}