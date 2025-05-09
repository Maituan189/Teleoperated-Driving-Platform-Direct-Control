#ifndef CANBUS_H
#define CANBUS_H

#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <string.h> 

#define VEHICLE_CONTROL_ID  0x01
#define GPS_ID              0x02
#define IMU1_ID             0x03
#define IMU2_ID             0x04
#define IMU3_ID             0x05
#define VEHICLE_ID          0x06
#define BATTERY_ID          0x07
#define CONTROLLERS_ID      0x08

class CANBus 
{
public:

// Hàm lấy instance duy nhất của Controller
static CANBus* getInstance();

// Constructor và Destructor
CANBus(const CANBus&) = delete; // Không cho phép sao chép
CANBus& operator=(const CANBus&) = delete; // Không cho phép gán
~CANBus();

uint8_t ID_LIST[6] = {VEHICLE_ID, IMU1_ID, IMU2_ID, IMU3_ID, BATTERY_ID, CONTROLLERS_ID};

typedef struct
{
  uint8_t Throttle_Value = 0;
  uint8_t Brake_Value= 0;
  uint8_t Gear_Shift= 0;
  int16_t Steering_Angle= 0;
  uint8_t Connection_State= 1;
  uint8_t Clutch_Value= 0;
  uint8_t Controller_Mode= 0;
  uint8_t SpeedSetpoint= 0;
} Data_Control;

Data_Control* getDataControl();

typedef struct  
{
  uint16_t RPM;
  uint8_t VehicleSpeed; 
  int16_t SteeringAngle;
  float GPS_X;
  float GPS_Y;
  float Roll_Angle;
  float Pitch_Angle;
  float Yaw_Angle;
  float Yaw_Rate;
  float Current;
  float Battery_Voltage;
  float Temperature;
  float Acc_Z; 
  float Displacement;
  float Error_Controllers;
} FeedBack_Data;
FeedBack_Data* getFeedBack();

void Send_FeedBack_Data(uint8_t ID);
Data_Control Receive_Data_Control();

private: 
CANBus();
FeedBack_Data Generate_FeedBack_Data(uint8_t ID);
static CANBus* instance_canbus;
static Data_Control* instance_data_control;
static FeedBack_Data* instance_feedback_data;
};

#endif // CANBUS_H