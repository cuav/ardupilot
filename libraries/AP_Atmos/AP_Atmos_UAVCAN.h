#pragma once

#include "AP_Atmos_Backend.h" //继承一定要包含

#include <AP_UAVCAN/AP_UAVCAN.h>




class Sht31Cb; // call back 使用的是atmos中，具体的传感器的命名

class AP_Atmos_Backend;

class AP_Atmos_UAVCAN : public AP_Atmos_Backend {
public:
   
    // constructor //构造函数在实例化对象的时候进行初始化
    AP_Atmos_UAVCAN(AP_Atmos &_frontend, uint8_t _instance);


   // 继承后台类的接口，获取具体传感器中的数据
   //在前台类的的函数中调用  sensor->get_temperature(temperature)
    bool get_humidity(float &humidity) override;
    bool get_temperature(float &temperature) override;

    //被同类的函数subscribe_msgs调用
    static void handle_sht31(AP_UAVCAN* ap_uavcan, uint8_t node_id, const Sht31Cb &cb);
   
    //libraries/AP_UAVCAN/AP_UAVCAN.cpp 中调用
    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);//订阅消息并获取数据

    //在前台类初始化函数中调用   sensor = AP_Atmos_UAVCAN::probe(*this, 1); 
    static AP_Atmos_Backend* probe(AP_Atmos &_frontend, uint8_t _instance);
    
private:
    float _humidity; // %, 在handle_sht31的时候会用到
    float _temperature; // Celcius 在handle_sht31的时候会用到
    uint32_t _last_sample_time_ms; //在handle_sht31的时候会用到
//给handle_sht31使用
   static  AP_Atmos_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id);

    HAL_Semaphore _sem_atmosphere;

     // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_Atmos_UAVCAN *driver;
    } _detected_modules[ATMOSPHERE_MAX_SENSORS]; //给get_uavcan_backend


     static HAL_Semaphore _sem_registry;


};