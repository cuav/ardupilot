#pragma once

#include "AP_Atmos_Backend.h" 
#include <AP_UAVCAN/AP_UAVCAN.h>


class AtmosphereCb;
class AP_Atmos_Backend;

class AP_Atmos_UAVCAN : public AP_Atmos_Backend {
public:
   
    // constructor 
    AP_Atmos_UAVCAN(AP_Atmos &_frontend, uint8_t _instance);


    bool get_humidity(float &humidity) override;
    bool get_temperature(float &temperature) override;

    static void handle_atmosphere(AP_UAVCAN* ap_uavcan, uint8_t node_id, const AtmosphereCb &cb);
   
    
    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

    
    static AP_Atmos_Backend* probe(AP_Atmos &_frontend, uint8_t _instance);
    
private:
    float _humidity; 
    float _temperature; 
    uint32_t _last_sample_time_ms; 

   static  AP_Atmos_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id);

    HAL_Semaphore _sem_atmosphere;

     // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_Atmos_UAVCAN *driver;
    } _detected_modules[ATMOSPHERE_MAX_SENSORS]; 


     static HAL_Semaphore _sem_registry;


};