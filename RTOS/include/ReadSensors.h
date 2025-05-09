#ifndef READSENSORS_H
#define READSENSORS_H

#include <Arduino.h> 
#include <Wire.h>
#include <stdlib.h> 

#define ADDRESS_IMU 0x68
#define PWR_MGMT_1_REG 0x6B
#define CONFIG_REG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
// Gyroscope Measurements
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
// Accelerometer Measurements
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
// Temperature
#define TEMP_OUT_H  0x41
#define TEMP_OUT_L  0x42
// Measurement Data of Magnetic Sensor
#define MPU_RA_INT_PIN_CFG      0x37
#define CNTL 0x0A
#define MPU9150_RA_MAG_ADDRESS 0x0C
#define HXL 0x03
#define HXH 0x04
#define HYL 0x05
#define HYH 0x06
#define HZL 0x07
#define HZH 0x08
// Sensitivity Adjustment Values
#define ASAX 0x10
#define ASAY 0x11
#define ASAZ 0x12

#define ENCODER_PIN_A  32   
#define ENCODER_PIN_B  33   
#define ENCODER_PPR    200 
#define PULSE_PER_METER  4083
#define BATT_VOL_FEEDBACK 35
#define CURRENT_FEEDBACK 34
#define OFFSET_CURRENT 2.615 // calibration
#define BATT_PIN ADC1_CHANNEL_6  // GPIO35
#define CURRENT_PIN ADC1_CHANNEL_7  // GPIO34
#define OFFSET_VOLTAGE 8.35
// #define OFFSET_VOLTAGE 7.552 
#define ACC_GRAVITY 9.81
#define DELTA_T 0.01
#define CLOCKWISE 1
#define UNCLOCKWISE -1
#define MEDIAN_WINDOW_SIZE 10
class MPU_9150
{
public:
static MPU_9150* getInstance();
// Constructor và Destructor
MPU_9150(const MPU_9150&) = delete; // Không cho phép sao chép
MPU_9150& operator=(const MPU_9150&) = delete; // Không cho phép gán
/*Destructor*/
~MPU_9150();
typedef struct  
{
  float Roll_Angle;
  float Pitch_Angle;
  float Yaw_Angle = 0;
  float Roll_Rate;
  float Pitch_Rate;
  float Yaw_Rate;
  float Acc_Z;
  float Displacement=0;
  float Temperature;
} SensorData;
SensorData* getSensorIMUData();

void getgyros_values(float *gyro_x, float *gyro_y, float *gyro_z);
void getacc_values(float *acc_x, float *acc_y, float *acc_z);
void getmag_values(float *mag_x, float *mag_y, float *mag_z);
void getvibration_values(float *vibr);
void getTemperature(float *temp);

float getAccelerationX();
float getAccelerationY();
float getAccelerationZ();
float getRotationX();
float getRotationY();
float getRotationZ();

void getSensorData(SensorData* data,  float offset_groZ, float offset_accZ );
void KalmanFilter(float* RollAngle, float* PitchAngle, float* YawAngle , float offset_grox, float offset_groy, float offset_groz);
void CommplimentaryFilter(float* data);
void Calibration();
float offsetRateRoll, offsetRatePitch, offsetRateYaw;
float offsetAccX, offsetAccY, offsetAccZ;
private: 
/*Constructor*/
MPU_9150();

float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float KalmanAngleYaw=0, KalmanUncertaintyAngleYaw=2*2;
float Kalman1DOutput[2]={0,0};
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);

static MPU_9150* instance_IMU;
static SensorData* instance_sensorIMU_data;

};

class Vehicle_Engine 
{
  public: 
  static Vehicle_Engine* getInstance();

  // Constructor và Destructor
  Vehicle_Engine(const Vehicle_Engine&) = delete; // Không cho phép sao chép
  Vehicle_Engine& operator=(const Vehicle_Engine&) = delete; // Không cho phép gán
  ~Vehicle_Engine();

  typedef struct
  {
    volatile uint16_t rpm;
    volatile uint16_t speed; // m/s
    volatile int8_t   direction;
    volatile uint32_t pulse_count;
  }Vehicle_Speed;

  Vehicle_Speed* getVehicleSpeed();

  uint16_t getRPM (uint32_t pulse_count);
  uint16_t getSpeed (uint32_t pulse_count);
  int8_t getDirection ();

  private: 
  /*Constructor*/
  Vehicle_Engine();
  /*Destructor*/
  uint16_t calculate_vehicle_speed_from_rpm(uint32_t pulse_count);
  static Vehicle_Engine* instance_engine;
  static Vehicle_Speed* instance_speed;
};

class MedianFilter
{
public:
  static MedianFilter* getInstance();
  // Constructor và Destructor
  MedianFilter(const MedianFilter&) = delete;
  MedianFilter& operator=(const MedianFilter&) = delete; 
  ~MedianFilter();
  typedef struct
  {
    uint16_t data[MEDIAN_WINDOW_SIZE];
    uint8_t index;
    uint8_t count;
  } MedianFilterData;

  MedianFilterData* getMedianFilterData();
  uint16_t MedianFilter_Update(MedianFilterData *filter, uint16_t new_value);
  
  private:
  /*Constructor*/
  MedianFilter();
  /*Destructor*/
  static MedianFilter* instance_median_filter;
  static MedianFilterData* instance_median_filter_data;
};


#endif // READSENSORS_H