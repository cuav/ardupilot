#include "AP_Atmos_UAVCAN.h"
#include "AP_Atmos.h"
#include <AP_HAL/AP_HAL.h> 
#include <ardupilot/equipment/Sht31.hpp>
#include <AP_CANManager/AP_CANManager.h>


#define LOG_TAG "Atmosphere"

#define C_TO_KELVIN 273.15f

UC_REGISTRY_BINDER(Sht31Cb, ardupilot::equipment::Sht31);

AP_Atmos_UAVCAN::DetectedModules AP_Atmos_UAVCAN::_detected_modules[] = {0};
HAL_Semaphore AP_Atmos_UAVCAN::_sem_registry;

    // constructor
AP_Atmos_UAVCAN::AP_Atmos_UAVCAN(AP_Atmos &_frontend, uint8_t _instance) :
    AP_Atmos_Backend(_frontend, _instance)
{}




void AP_Atmos_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<ardupilot::equipment::Sht31, Sht31Cb> *sht31_listener;
    sht31_listener = new uavcan::Subscriber<ardupilot::equipment::Sht31, Sht31Cb>(*node);

    const int sht31_listener_res = sht31_listener->start(Sht31Cb(ap_uavcan, &handle_sht31)); 
    // printf("AP_Atmos_UAVCAN ------sht31_listener_res = %d\n",sht31_listener_res);
    if (sht31_listener_res < 0) {
        AP_HAL::panic("UAVCAN Sht31 subscriber start problem\n");
    }

}


AP_Atmos_Backend* AP_Atmos_UAVCAN::probe(AP_Atmos &_frontend, uint8_t _instance)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Atmos_UAVCAN* backend = nullptr;

    for (uint8_t i = 0; i < ATMOSPHERE_MAX_SENSORS; i++) {


        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            
            backend = new AP_Atmos_UAVCAN(_frontend, _instance);
            if (backend == nullptr) {
     
                AP::can().log_text(AP_CANManager::LOG_INFO, 
                                      LOG_TAG,
                                      "Failed register UAVCAN Airspeed Node %d on Bus %d\n",
                                      _detected_modules[i].node_id,
                                      _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
    
                _detected_modules[i].driver = backend;
                AP::can().log_text(AP_CANManager::LOG_INFO, 
                                      LOG_TAG,
                                      "Registered UAVCAN Airspeed Node %d on Bus %d\n",
                                      _detected_modules[i].node_id,
                                      _detected_modules[i].ap_uavcan->get_driver_index());
            }
            break;
        }
    }
  
    return backend;
}



AP_Atmos_UAVCAN* AP_Atmos_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < ATMOSPHERE_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan &&
            _detected_modules[i].node_id == node_id ) {
              
            return _detected_modules[i].driver;
        }
    }

    bool detected = false;
    for (uint8_t i = 0; i < ATMOSPHERE_MAX_SENSORS; i++) {
        if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
            
            detected = true;
            break;
        }
    }

    if (!detected) {
        for (uint8_t i = 0; i < ATMOSPHERE_MAX_SENSORS; i++) {
            if (_detected_modules[i].ap_uavcan == nullptr) {
                _detected_modules[i].ap_uavcan = ap_uavcan;
                _detected_modules[i].node_id = node_id;
                break;
            }
        }
    }

    return nullptr;
}


void AP_Atmos_UAVCAN::handle_sht31(AP_UAVCAN* ap_uavcan, uint8_t node_id, const Sht31Cb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Atmos_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);
    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_atmosphere);
        driver->_humidity = cb.msg->humidity;
        // printf("handle_sht31 driver->humidity = %f\n",driver->_humidity);
        if (!isnan(cb.msg->temperature))  { 
            driver->_temperature = cb.msg->temperature - C_TO_KELVIN;
            
        }
        driver->_last_sample_time_ms = AP_HAL::millis();
    }
}

bool AP_Atmos_UAVCAN::get_humidity(float &humidity)
 {
    
    WITH_SEMAPHORE(_sem_atmosphere);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    humidity = _humidity;
    printf("AP_Atmos_UAVCAN::get_humidity = %f\n",humidity);
    return true;

 }


bool AP_Atmos_UAVCAN::get_temperature(float &temperature)
{
   
    WITH_SEMAPHORE(_sem_atmosphere);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    temperature = _temperature;
    printf("AP_Atmos_UAVCAN::get_temperature = %f\n",temperature);
    return true;
}