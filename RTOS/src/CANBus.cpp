
#include "CANBus.h"

// Khởi tạo con trỏ static
CANBus* CANBus::instance_canbus = nullptr;
CANBus::Data_Control* CANBus::instance_data_control = nullptr;
CANBus::FeedBack_Data* CANBus::instance_feedback_data = nullptr;

// Destructor
CANBus::~CANBus() {
  delete instance_canbus;
  instance_canbus = nullptr;
  delete instance_data_control;
  instance_data_control = nullptr;
  delete instance_feedback_data;
  instance_feedback_data = nullptr;
}

CANBus::CANBus() {
  instance_data_control = new Data_Control();
  instance_feedback_data = new FeedBack_Data();
}

// Singleton Getter
CANBus* CANBus::getInstance() {
  if (!instance_canbus) {
      instance_canbus = new CANBus();
  }
  return instance_canbus;
}

// Getter cho Data_Control
CANBus::Data_Control* CANBus::getDataControl() {
  return instance_data_control;
}

// Getter cho FeedBack_Data
CANBus::FeedBack_Data* CANBus::getFeedBack() {
  return instance_feedback_data;
}

CANBus::Data_Control CANBus:: Receive_Data_Control()
{
  static CAN_frame_t rx_frame;
  static CANBus::Data_Control Data;
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) 
  {
    if (rx_frame.FIR.B.FF == CAN_frame_std) 
    {
      // printf("New standard frame");
      if (rx_frame.FIR.B.RTR != CAN_RTR && rx_frame.MsgID == VEHICLE_CONTROL_ID)
      {
        instance_data_control->Throttle_Value = (uint8_t)rx_frame.data.u8[0];
        instance_data_control->Brake_Value = (uint8_t)rx_frame.data.u8[1];
        instance_data_control->Gear_Shift = (uint8_t)rx_frame.data.u8[2];
        instance_data_control->Steering_Angle = ((int16_t)rx_frame.data.u8[3] | (int16_t)(rx_frame.data.u8[4]<<8));
        instance_data_control->Connection_State = (uint8_t)rx_frame.data.u8[5];
        instance_data_control -> Controller_Mode = (uint8_t)rx_frame.data.u8[6];
        instance_data_control -> SpeedSetpoint = (uint8_t)rx_frame.data.u8[7];
        // printf("SpeedSetpoint: %d\n", instance_data_control -> SpeedSetpoint);
        // printf("Controller_Mode: %d\n", instance_data_control -> Controller_Mode);
        // printf("Vehicle_Speed: %d; Brake_Value: %d; Gear_Shift: %d; Steering_Angle: %d; Connection_State: %d\n", instance_data_control->Throttle_Value, instance_data_control->Brake_Value, instance_data_control->Gear_Shift, instance_data_control->Steering_Angle, instance_data_control->Connection_State);
      }
    }
    else 
    {
      // printf("New extended frame");

    }
  }
  return Data;
}

void CANBus::Send_FeedBack_Data(uint8_t ID)
{
  static CANBus::FeedBack_Data Data;
  static CAN_frame_t Vehicle_Data; 
  static CAN_frame_t Battery_Sensor; 
  static CAN_frame_t IMU1_Sensor; 
  static CAN_frame_t IMU2_Sensor; 
  static CAN_frame_t IMU3_Sensor; 
  static CAN_frame_t Controllers_Data; 
  
  Vehicle_Data.FIR.B.FF = CAN_frame_std;
  Vehicle_Data.MsgID = VEHICLE_ID;
  Vehicle_Data.FIR.B.DLC = 3;

  Battery_Sensor.FIR.B.FF = CAN_frame_std;
  Battery_Sensor.MsgID = BATTERY_ID;
  Battery_Sensor.FIR.B.DLC = 8;

  IMU1_Sensor.FIR.B.FF = CAN_frame_std;
  IMU1_Sensor.MsgID = IMU1_ID;
  IMU1_Sensor.FIR.B.DLC = 8;

  IMU2_Sensor.FIR.B.FF = CAN_frame_std;
  IMU2_Sensor.MsgID = IMU2_ID;
  IMU2_Sensor.FIR.B.DLC = 8;

  IMU3_Sensor.FIR.B.FF = CAN_frame_std;
  IMU3_Sensor.MsgID = IMU3_ID;
  IMU3_Sensor.FIR.B.DLC = 8;

  Controllers_Data.FIR.B.FF = CAN_frame_std;
  Controllers_Data.MsgID = CONTROLLERS_ID;
  Controllers_Data.FIR.B.DLC = 4;

  if (ID == VEHICLE_ID)
  {    
    Vehicle_Data.data.u8[0] = instance_feedback_data->VehicleSpeed;
    Vehicle_Data.data.u8[1] = instance_feedback_data->RPM & (0xFF);
    Vehicle_Data.data.u8[2] = (instance_feedback_data->RPM >> 8) & (0xFF);
    // Vehicle_Data.data.u8[3] = instance_feedback_data->SteeringAngle & (0xFF);
    // Vehicle_Data.data.u8[4] = (instance_feedback_data->SteeringAngle >> 8) & (0xFF);
    ESP32Can.CANWriteFrame(&Vehicle_Data);
    // printf("%02X %02X %02X\n",Vehicle_Data.data.u8[0], Vehicle_Data.data.u8[1], Vehicle_Data.data.u8[2]);
    // printf("VehicleSpeed: %d; SteeringAngle: %d\n",instance_feedback_data->VehicleSpeed, instance_feedback_data->SteeringAngle);
  }
  else if (ID == IMU1_ID)
  {
    memcpy(&IMU1_Sensor.data.u8[0], &instance_feedback_data->Roll_Angle, sizeof(float));
    memcpy(&IMU1_Sensor.data.u8[4], &instance_feedback_data->Pitch_Angle, sizeof(float));
    ESP32Can.CANWriteFrame(&IMU1_Sensor);
    // printf("%02X %02X %02X %02X %02X %02X %02X %02X\n",IMU1_Sensor.data.u8[0], IMU1_Sensor.data.u8[1], IMU1_Sensor.data.u8[2], IMU1_Sensor.data.u8[3], IMU1_Sensor.data.u8[4], IMU1_Sensor.data.u8[5], IMU1_Sensor.data.u8[6], IMU1_Sensor.data.u8[7]);
    // printf("Roll_Angle: %f; Pitch_Angle: %f\n",instance_feedback_data->Roll_Angle, instance_feedback_data->Pitch_Angle);
  }
  else if (ID == IMU2_ID)
  {
    memcpy(&IMU2_Sensor.data.u8[0], &instance_feedback_data->Yaw_Angle, sizeof(float));
    memcpy(&IMU2_Sensor.data.u8[4], &instance_feedback_data->Yaw_Rate, sizeof(float));
    ESP32Can.CANWriteFrame(&IMU2_Sensor);
    // printf("%02X %02X %02X %02X %02X %02X %02X %02X\n",IMU2_Sensor.data.u8[0], IMU2_Sensor.data.u8[1], IMU2_Sensor.data.u8[2], IMU2_Sensor.data.u8[3], IMU2_Sensor.data.u8[4], IMU2_Sensor.data.u8[5], IMU2_Sensor.data.u8[6], IMU2_Sensor.data.u8[7]);
    // printf("Yaw_Angle: %f; Yaw_Rate: %f\n",instance_feedback_data->Yaw_Angle, instance_feedback_data->Yaw_Rate);
  }
  else if (ID == IMU3_ID)
  {
    memcpy(&IMU3_Sensor.data.u8[0], &instance_feedback_data->Temperature, sizeof(float));
    memcpy(&IMU3_Sensor.data.u8[4], &instance_feedback_data->Displacement, sizeof(float));
    ESP32Can.CANWriteFrame(&IMU3_Sensor);
    // printf("%02X %02X %02X %02X %02X %02X %02X %02X\n",IMU2_Sensor.data.u8[0], IMU2_Sensor.data.u8[1], IMU2_Sensor.data.u8[2], IMU2_Sensor.data.u8[3], IMU2_Sensor.data.u8[4], IMU2_Sensor.data.u8[5], IMU2_Sensor.data.u8[6], IMU2_Sensor.data.u8[7]);
    // printf("Yaw_Angle: %f; Yaw_Rate: %f\n",instance_feedback_data->Yaw_Angle, instance_feedback_data->Yaw_Rate);
  }
  else if (ID == BATTERY_ID)
  {
    memcpy(&Battery_Sensor.data.u8[0], &instance_feedback_data->Battery_Voltage, sizeof(float));
    memcpy(&Battery_Sensor.data.u8[4], &instance_feedback_data->Current, sizeof(float));
    ESP32Can.CANWriteFrame(&Battery_Sensor);
    // printf("%02X %02X %02X %02X %02X %02X %02X %02X\n",GPS_Sensor.data.u8[0], GPS_Sensor.data.u8[1], GPS_Sensor.data.u8[2], GPS_Sensor.data.u8[3], GPS_Sensor.data.u8[4], GPS_Sensor.data.u8[5], GPS_Sensor.data.u8[6], GPS_Sensor.data.u8[7]);
    // printf("GPS_X: %f; GPS_Y: %f\n",instance_feedback_data->GPS_X, instance_feedback_data->GPS_Y);
  }
  else if (ID == CONTROLLERS_ID)
  {
    /* */
    memcpy(&Controllers_Data.data.u8[0], &instance_feedback_data->Error_Controllers, sizeof(float));
    ESP32Can.CANWriteFrame(&Controllers_Data);
  }
}

static float random_float(float min, float max) {
    return min + (float)rand() / RAND_MAX * (max - min);
  }
  
CANBus::FeedBack_Data CANBus::Generate_FeedBack_Data(uint8_t ID)
{
  CANBus::FeedBack_Data Data;
  if (ID == VEHICLE_ID)
  {
    Data.VehicleSpeed = (uint8_t)(random_float(0, 255));
    Data.SteeringAngle = (int16_t)(random_float(-400, 400));
  }
  else if (ID == GPS_ID)
  {
    Data.GPS_X = (float)random_float(-400, 400);
    Data.GPS_Y = (float)random_float(-400, 400);
  }
  else if (ID == IMU1_ID)
  {
    Data.Roll_Angle = (float)random_float(-200, 200);
    Data.Pitch_Angle = (float)random_float(-400, 400);
  }
  else if (ID == IMU2_ID)
  {
    Data.Yaw_Angle = (float)random_float(-400, 400);
    Data.Yaw_Rate = (float)random_float(-400, 400);
  }
  else 
  {

  }
  return Data;
}
