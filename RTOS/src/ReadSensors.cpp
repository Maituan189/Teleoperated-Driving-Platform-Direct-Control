#include "ReadSensors.h"

MPU_9150::SensorData* MPU_9150::instance_sensorIMU_data = nullptr;
MPU_9150* MPU_9150::instance_IMU = nullptr;
MPU_9150::MPU_9150()
{
  instance_sensorIMU_data = new SensorData();
}

MPU_9150::~MPU_9150() {
  delete instance_IMU;
  instance_IMU = nullptr;
  delete instance_sensorIMU_data;
  instance_sensorIMU_data = nullptr;
}

MPU_9150* MPU_9150::getInstance() {
  if (!instance_IMU) {
      instance_IMU = new MPU_9150();
  }
  return instance_IMU;
}

MPU_9150::SensorData* MPU_9150::getSensorIMUData() {
  return instance_sensorIMU_data;
}

void MPU_9150::getgyros_values(float *gyro_x, float *gyro_y, float *gyro_z) 
{
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(CONFIG_REG);
  Wire.write(0x05);
  Wire.endTransmission(); 

  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write((uint8_t)GYRO_CONFIG); 
  Wire.write((uint8_t)0x8); 
  Wire.endTransmission(); 

  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write((uint8_t)GYRO_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_IMU,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  
  *gyro_x=(float)GyroX/65.5;
  *gyro_y=(float)GyroY/65.5;
  *gyro_z=(float)GyroZ/65.5;
  // printf("GyroX: %f GyroY: %f GyroZ: %f\n", *gyro_x, *gyro_y, *gyro_z);
}

void MPU_9150::getacc_values(float *acc_x, float *acc_y, float *acc_z) 
{
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(CONFIG_REG);
  Wire.write(0x05);
  Wire.endTransmission(); 

  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(ACCEL_CONFIG); 
  Wire.write((uint8_t)0x10); // 8g = 4096
  // Wire.write((uint8_t)0x8); // 4g = 8192

  Wire.endTransmission();

  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write((uint8_t)ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_IMU,6);
  int16_t Acc_x=Wire.read()<<8 | Wire.read();
  int16_t Acc_y=Wire.read()<<8 | Wire.read();
  int16_t Acc_z=Wire.read()<<8 | Wire.read();

  *acc_x=(float)Acc_x/4096;
  *acc_y=(float)Acc_y/4096;
  *acc_z=(float)Acc_z/4096;

  // printf("Acc_x: %f Acc_y: %f Acc_z: %f\n", *acc_x, *acc_y, *acc_z);
}

void MPU_9150::getvibration_values(float *vibr) 
{
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(CONFIG_REG);
  Wire.write(0x05);
  Wire.endTransmission(); 

  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(ACCEL_CONFIG); 
  Wire.write((uint8_t)0x10); 
  Wire.endTransmission();

  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write((uint8_t)ACCEL_ZOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_IMU,2);
  int16_t vibration=Wire.read()<<8 | Wire.read();
  *vibr=(float)vibration/4096;
}

void MPU_9150::getTemperature(float *temp) 
{
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(CONFIG_REG);
  Wire.write(0x05);
  Wire.endTransmission(); 

  // Wire.beginTransmission(ADDRESS_IMU);
  // Wire.write(ACCEL_CONFIG); 
  // Wire.write((uint8_t)0x10); 
  // Wire.endTransmission();

  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write((uint8_t)TEMP_OUT_H); // TEMP_OUT_H
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_IMU,2);

  int16_t raw_temp =Wire.read()<<8 | Wire.read();

  *temp=(float)(raw_temp / 340) + 35;
}

void MPU_9150::getmag_values(float *mag_x, float *mag_y, float *mag_z)
{ 
  //Set I2C bypass enable pin to true to access magnetometer
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write((uint8_t) MPU_RA_INT_PIN_CFG); 
  Wire.write(0x02); 
  Wire.endTransmission();

  //Enable The Magnetometer
  Wire.beginTransmission(MPU9150_RA_MAG_ADDRESS);
  Wire.write((uint8_t) CNTL); // send address
  Wire.write((uint8_t)0x01); //Single measurement mode
  Wire.endTransmission(); 
  delay(10);// Wait for the measurement to complete

  // Start Read  
  Wire.beginTransmission(MPU9150_RA_MAG_ADDRESS);
  Wire.write((uint8_t)HXL);
  Wire.endTransmission();
  Wire.requestFrom(MPU9150_RA_MAG_ADDRESS,6);
  int16_t Mag_x= Wire.read() | Wire.read()<<8;
  int16_t Mag_y= Wire.read() | Wire.read()<<8;
  int16_t Mag_z= Wire.read() | Wire.read()<<8;

  //Enter FUSE ROM Mode
  Wire.beginTransmission(MPU9150_RA_MAG_ADDRESS);
  Wire.write(0x0A);      // CNTL register
  Wire.write(0x0F);      // Fuse ROM access mode
  Wire.endTransmission();

  // Ajustment Values
  Wire.beginTransmission(MPU9150_RA_MAG_ADDRESS);
  Wire.write((uint8_t)ASAX);
  Wire.endTransmission();
  Wire.requestFrom(MPU9150_RA_MAG_ADDRESS,3);
  int8_t ASAX_Values = Wire.read();
  int8_t ASAY_Values = Wire.read();
  int8_t ASAZ_Values = Wire.read();

  //Exit FUSE ROM Mode
  Wire.beginTransmission(MPU9150_RA_MAG_ADDRESS);
  Wire.write(0x0A);      // CNTL register
  Wire.write(0x00);      // Fuse ROM access mode
  Wire.endTransmission();

  // printf("ASAX: %d ASAY: %d ASAZ: %d\n", ASAX_Values, ASAY_Values, ASAZ_Values);
  *mag_x=(float)Mag_x*((((ASAX_Values - 128)*0.5)/128)+1);
  *mag_y=(float)Mag_y*((((ASAY_Values - 128)*0.5)/128)+1);
  *mag_z=(float)Mag_z*((((ASAZ_Values - 128)*0.5)/128)+1);
  // *mag_x=(float)Mag_x;
  // *mag_y=(float)Mag_y;
  // *mag_z=(float)Mag_z;
  // printf("Mag_x: %f Mag_y: %f Mag_z: %f\n", *mag_x, *mag_y, *mag_z);
}

float MPU_9150::getAccelerationX()
{
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write((uint8_t)ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_IMU,2);
  int16_t Acc_x=Wire.read()<<8 | Wire.read();
  return (float)Acc_x/8192;
}

float MPU_9150::getAccelerationY()
{
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(ACCEL_YOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_IMU,2);
  int16_t Acc_y=Wire.read()<<8 | Wire.read();
  return (float)Acc_y/8192;
}

float MPU_9150::getAccelerationZ()
{
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(ACCEL_ZOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_IMU,2);
  int16_t Acc_z=Wire.read()<<8 | Wire.read();
  return (float)Acc_z/8192;
}

float MPU_9150::getRotationX()
{
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_IMU,2);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  return (float)GyroX/8192;
}

float MPU_9150::getRotationY()
{
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(GYRO_YOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_IMU,2);
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  return (float)GyroY/8192;
}

float MPU_9150::getRotationZ()
{
  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_IMU,2);
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  return (float)GyroZ/8192;
}

void MPU_9150::getSensorData(MPU_9150::SensorData* data, float offset_groZ ,float offset_accZ)
{
  static float mx,my,mz = 0;
  static float ax,ay,az = 0;
  static float Roll_Rate, Pitch_Rate, Yaw_Rate = 0;
  static float vibr = 0;
  static float temp = 0;
  static float vel_z = 0, pos_z = 0;
  getacc_values(&ax, &ay, &az);
  getgyros_values(&Roll_Rate, &Pitch_Rate, &Yaw_Rate);
  getmag_values(&mx, &my, &mz);
  getTemperature(&temp);
  data -> Roll_Angle = (float) (atan2(ay,az) * RAD_TO_DEG);
  data -> Pitch_Angle = (float) (atan2(-ax, sqrt(ay*ay+az*az)) * RAD_TO_DEG);

  float Mx = mx*cos(data ->Pitch_Angle) + mz*sin(data -> Pitch_Angle);
  float My = mx*sin(data -> Roll_Angle)*sin(data -> Pitch_Angle) + my*cos(data -> Roll_Angle) - mz*sin(data -> Roll_Angle)*cos(data->Pitch_Angle);

  // data -> Yaw_Angle = atan2(My, Mx) * RAD_TO_DEG; 
  // data -> Roll_Rate = Roll_Rate * RAD_TO_DEG;
  // data -> Pitch_Rate = Pitch_Rate * RAD_TO_DEG;
  Yaw_Rate -= offset_groZ;
  data -> Yaw_Angle +=  Yaw_Rate * DELTA_T*2;
  az -= offset_accZ ;
  vel_z = az*ACC_GRAVITY * DELTA_T;
  pos_z = ( vel_z*DELTA_T + 0.5* az* ACC_GRAVITY * DELTA_T*DELTA_T)*1000;
  // printf("az: %f vel_z: %f pos_z: %f \n", az*9.81, vel_z ,pos_z);
  data -> Acc_Z = az*ACC_GRAVITY;
  data -> Displacement = pos_z;
  data -> Temperature = temp;

  // printf("Roll_Angle: %f Pitch_Angle: %f Yaw_Angle: %f| Temp: %f\n", data -> Roll_Angle, data -> Pitch_Angle, data -> Yaw_Angle, data ->Temperature);
}

void MPU_9150::Calibration()
{
  for (uint16_t RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    RateCalibrationRoll+= MPU_9150::getRotationX();
    RateCalibrationPitch+= MPU_9150::getRotationY();
    RateCalibrationYaw+= MPU_9150::getRotationZ();
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
}

void MPU_9150::kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
void MPU_9150::KalmanFilter(float* RollAngle, float* PitchAngle, float* YawAngle, float offset_grox, float offset_groy, float offset_groz)
{
  static uint32_t LoopTimer = 0;
  static float Roll_Rate=0, Pitch_Rate=0, Yaw_Rate=0;
  getgyros_values(&Roll_Rate, &Pitch_Rate, &Yaw_Rate);

  Roll_Rate -=   offset_grox;
  Pitch_Rate -=  offset_groy;
  Yaw_Rate -= offset_groz;
  
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, Roll_Rate, *RollAngle);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  *RollAngle = KalmanAngleRoll;

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, Pitch_Rate, *PitchAngle);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  *PitchAngle = KalmanAnglePitch;

  kalman_1d(KalmanAngleYaw, KalmanUncertaintyAngleYaw, Yaw_Rate, *YawAngle);
  KalmanAngleYaw=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  *YawAngle = KalmanAngleYaw;

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}

Vehicle_Engine::Vehicle_Speed* Vehicle_Engine::instance_speed = nullptr; 
Vehicle_Engine* Vehicle_Engine::instance_engine = nullptr;

Vehicle_Engine::Vehicle_Engine() {
  instance_speed = new Vehicle_Speed();
  instance_speed->rpm = 0;
  instance_speed->speed = 0;
  instance_speed->direction = 0;
  instance_speed->pulse_count = 0;
}

Vehicle_Engine::~Vehicle_Engine() {
  delete instance_speed;
  instance_speed = nullptr;
  delete instance_engine;
  instance_engine = nullptr;
}

Vehicle_Engine* Vehicle_Engine::getInstance() {
  if (!instance_engine) {
      instance_engine = new Vehicle_Engine();
  }
  return instance_engine;
}

Vehicle_Engine::Vehicle_Speed* Vehicle_Engine::getVehicleSpeed() {
  return instance_speed;
}

uint16_t Vehicle_Engine::getRPM (uint32_t pulse_count)
{
  static int last_pulse_count = 0;
  static uint32_t rpm = 0;
  int pulse_diff = pulse_count - last_pulse_count;
  last_pulse_count = pulse_count;
  rpm = ((float)pulse_diff / ENCODER_PPR) *60 / 0.01;
  if (rpm > 8000)
  {
    rpm = 8000;
  }
  else if (rpm<0)
  {
    rpm = 0;
  }
  return (uint16_t)rpm;
}

uint16_t Vehicle_Engine::calculate_vehicle_speed_from_rpm(uint32_t pulse_count)
{
  static int last_pulse_count = 0;
  static uint16_t speed = 0;
  int pulse_diff = pulse_count - last_pulse_count;
  last_pulse_count = pulse_count;

  speed = (((float)pulse_diff / PULSE_PER_METER) / 0.01)*3.6*12;

  return (uint16_t)speed;
}

uint16_t Vehicle_Engine::getSpeed (uint32_t pulse_count)
{
  instance_speed->speed = calculate_vehicle_speed_from_rpm(pulse_count);
  return instance_speed->speed;
}

int8_t Vehicle_Engine::getDirection ()
{
  return instance_speed->direction;
}



MedianFilter* MedianFilter::instance_median_filter = nullptr;
MedianFilter::MedianFilterData* MedianFilter::instance_median_filter_data = nullptr;
MedianFilter::MedianFilter()
{
  instance_median_filter_data = new MedianFilterData();
  instance_median_filter_data->index = 0;
  instance_median_filter_data->count = 0;
}
MedianFilter::~MedianFilter() {
  delete instance_median_filter;
  instance_median_filter = nullptr;
  delete instance_median_filter_data;
  instance_median_filter_data = nullptr;
}
MedianFilter* MedianFilter::getInstance() {
  if (!instance_median_filter) {
      instance_median_filter = new MedianFilter();
  }
  return instance_median_filter;
}
MedianFilter::MedianFilterData* MedianFilter::getMedianFilterData() {
  return instance_median_filter_data;
}
uint16_t MedianFilter::MedianFilter_Update(MedianFilterData *filter, uint16_t new_value) {
  if (!filter) return new_value;

  filter->data[filter->index] = new_value;
  filter->index = (filter->index + 1) % MEDIAN_WINDOW_SIZE;

  if (filter->count < MEDIAN_WINDOW_SIZE) 
  {
    filter->count++;
  }
  uint16_t sorted_data[MEDIAN_WINDOW_SIZE] = {0};
  memcpy(sorted_data, filter->data, sizeof(filter->data));
  std::sort(sorted_data, sorted_data + filter->count);
  return sorted_data[filter->count / 2];
}