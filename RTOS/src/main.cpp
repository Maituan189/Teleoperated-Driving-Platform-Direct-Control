#include <Arduino.h>
#include "Controllers.h"
#include "ReadSensors.h"
#include "CANBus.h"
#include <ESP32Servo.h>
#include "driver/mcpwm.h"
#include "driver/rmt.h"
#include "driver/timer.h"
#include "driver/pcnt.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define CORE_0 0
#define CORE_1 1
#define PCNT_UNIT PCNT_UNIT_0 
CAN_device_t CAN_cfg;              

Servo servo;

SemaphoreHandle_t xCANBusReady;
SemaphoreHandle_t xControllersReady;
SemaphoreHandle_t xActuatorReady;
TaskHandle_t xTaskHandle_CANBus_Send;
TaskHandle_t xTaskHandle_CANBus_Receive;
TaskHandle_t xTaskHandle_ControllerCompute;
TimerHandle_t xCANBusTimer_Send;
TimerHandle_t xCANBusTimer_Receive;

Controller* controller = Controller::getInstance();
Controller::Cmd_Control* cmd_control = controller->getCmdControl();
Controller::PIDController* pid_control = controller->getPIDController();

CANBus* canbus = CANBus::getInstance();
CANBus::Data_Control* canbus_data_control = canbus->getDataControl();
CANBus::FeedBack_Data* canbus_feedback_data = canbus->getFeedBack();

Vehicle_Engine *vehicle_engine = Vehicle_Engine::getInstance();
Vehicle_Engine::Vehicle_Speed* vehicle_speed = vehicle_engine->getVehicleSpeed();

MPU_9150 *mpu_9150 = MPU_9150::getInstance();
MPU_9150::SensorData* mpu_9150_data = mpu_9150->getSensorIMUData();

MedianFilter *median_filter = MedianFilter::getInstance();
MedianFilter::MedianFilterData* median_filter_data = median_filter->getMedianFilterData();

QueueHandle_t encoderQueue;

static esp_adc_cal_characteristics_t adc_chars;

void TASK_ProcessEncoder(void *pvParameters) 
{
  int16_t count = 0;
  static int16_t update_count = 0;
  static uint32_t pulse_count = 0;
  static uint32_t rpm = 0;
  const TickType_t xDelay = pdMS_TO_TICKS(10);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static esp_err_t err_msg; 
  static uint16_t temp_rpm = 0;
  while (1) 
  {
    err_msg = pcnt_get_counter_value(PCNT_UNIT, &count);
    if (err_msg == ESP_OK)
    {
      if (update_count != count)
      {
        if (update_count - count > 0)
        {
          vehicle_speed->direction = CLOCKWISE;
        }
        else if (update_count - count < 0)
        {
          vehicle_speed->direction = UNCLOCKWISE;
        }
        pulse_count += abs(count - update_count);
        update_count = count;
      }
    }
    else
    {
      Serial.print(" err_msg: " + err_msg);
    }
    vehicle_speed->pulse_count = pulse_count;
    // vehicle_speed->rpm = vehicle_engine->getRPM(pulse_count);

    temp_rpm = vehicle_engine->getRPM(pulse_count);
    vehicle_speed->rpm = median_filter->MedianFilter_Update(median_filter_data, temp_rpm);
    vehicle_speed->speed = vehicle_engine->getSpeed(pulse_count);
    printf("Pulse Count: %d| RPM: %d | Speed: %d | Direction: %d\n", vehicle_speed->pulse_count, vehicle_speed->rpm, vehicle_speed->speed,vehicle_speed->direction);
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}


void TASK_ReadSensors(void *pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(10);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static int16_t battery_raw = 0;
  static int16_t current_raw = 0;
  static float battery_voltage = 0;
  static float current_value = 0;
  while (1) 
  {
    // printf("Pulse_Count: %d || RPM: %d || Speed: %d || Direction: %d\n", vehicle_speed->pulse_count, vehicle_speed->rpm, vehicle_speed->speed, vehicle_speed->direction);
    canbus_feedback_data-> VehicleSpeed = vehicle_speed->speed;
    canbus_feedback_data-> RPM = vehicle_speed->rpm;
    canbus_feedback_data-> Error_Controllers = 5.5;
    
    // canbus_feedback_data-> Error_Controllers = pid_control->prev_error;
    mpu_9150->getSensorData(mpu_9150_data, mpu_9150->offsetRateYaw, mpu_9150 -> offsetAccZ);
    // printf("Roll_Angle: %f Pitch_Angle: %f Yaw_Angle: %f\n", mpu_9150_data->Roll_Angle, mpu_9150_data->Pitch_Angle, mpu_9150_data->Yaw_Angle);

    // mpu_9150->KalmanFilter(&(mpu_9150_data->Roll_Angle), &(mpu_9150_data->Pitch_Angle), &(mpu_9150_data->Yaw_Angle), mpu_9150->offsetRateRoll, mpu_9150->offsetRatePitch, mpu_9150->offsetRateYaw);
    // printf("Roll_Angle_Filter: %f Pitch_Angle_Filter: %f Yaw_Angle_Filter: %f\n", mpu_9150_data->Roll_Angle, mpu_9150_data->Pitch_Angle, mpu_9150_data->Yaw_Angle);

    canbus_feedback_data->Roll_Angle = mpu_9150_data->Roll_Angle;
    canbus_feedback_data->Pitch_Angle = mpu_9150_data->Pitch_Angle;
    canbus_feedback_data->Yaw_Angle = mpu_9150_data->Yaw_Angle;
    canbus_feedback_data->Yaw_Rate = mpu_9150_data->Yaw_Rate;
    canbus_feedback_data->Acc_Z = mpu_9150_data->Acc_Z;
    canbus_feedback_data->Temperature = mpu_9150_data->Temperature;
    canbus_feedback_data->Displacement = mpu_9150_data->Displacement;
    
    battery_raw = adc1_get_raw(BATT_PIN);
    current_raw = adc1_get_raw(CURRENT_PIN);
 
    battery_voltage = esp_adc_cal_raw_to_voltage(battery_raw, &adc_chars) * OFFSET_VOLTAGE / 1000.0;
    current_value = ((esp_adc_cal_raw_to_voltage(current_raw, &adc_chars)/1000.0) - OFFSET_CURRENT)* 1000 / 60;
    // battery_voltage = esp_adc_cal_raw_to_voltage(battery_raw, &adc_chars);

    // Serial.print("Battery Voltage: ");
    // Serial.println(battery_voltage);

    // Serial.print("Current: ");
    // Serial.println(current_value);

    canbus_feedback_data -> Battery_Voltage = battery_voltage;
    canbus_feedback_data -> Current = current_value;
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void CANBusTimerCallback_Send(TimerHandle_t xTimer) 
{
  // Signal the CANBus task to process data
  xTaskNotifyGive(xTaskHandle_CANBus_Send);
}

void TASK_CANBus_Send(void *pvParameters) {
  static CANBus::Data_Control data_control;
  while (1) {
      // Wait for the timer to signal
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      for (uint8_t i = 0; i < (sizeof(canbus->ID_LIST) / sizeof(canbus->ID_LIST[0])); i++) 
      {
          canbus->Send_FeedBack_Data(canbus->ID_LIST[i]);
      }
      // printf("Steering_Angle: %d ||Throttle_Value: %d  ||Gear_Shift: %d  \n", canbus_data_control->Steering_Angle, canbus_data_control->Throttle_Value, canbus_data_control->Gear_Shift);
  }
}

void CANBusTimerCallback_Receive(TimerHandle_t xTimer) 
{
  // Signal the CANBus task to process data
  xTaskNotifyGive(xTaskHandle_CANBus_Receive);
}

void TASK_CANBus_Receive(void *pvParameters) {
  static CANBus::Data_Control data_control;
  while (1) {
      // Wait for the timer to signal
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      canbus->Receive_Data_Control();
      xSemaphoreGive(xCANBusReady);
      // printf("Steering_Angle: %d ||Throttle_Value: %d  ||Gear_Shift: %d  \n", canbus_data_control->Steering_Angle, canbus_data_control->Throttle_Value, canbus_data_control->Gear_Shift);
  }
}

void TASK_Controllers(void *pvParameters) { 
  static uint8_t mode_control=0;
  static bool entered_cruise = false;
  while (1) {
    if (xSemaphoreTake(xCANBusReady, portMAX_DELAY) == pdTRUE) 
    {
      if (canbus_data_control != nullptr)
      {
        cmd_control -> mode_cmd = controller ->controller_mode(canbus_data_control -> Controller_Mode, vehicle_speed->speed, vehicle_speed->direction, canbus_data_control -> Throttle_Value, canbus_data_control ->Brake_Value, canbus_data_control -> Gear_Shift);
        eTaskState state = eTaskGetState(xTaskHandle_ControllerCompute);
        if (cmd_control -> mode_cmd == SPEED_CONTROL_MODE)
        {    
          // if (state == eSuspended) 
          // {
          //   vTaskResume(xTaskHandle_ControllerCompute);
          // }
          if (!entered_cruise)
          {
            pid_control->pre_throttle = canbus_data_control->Throttle_Value;
            entered_cruise = true;
          }
          cmd_control->throttle_cmd = (uint8_t)pid_control->output;
        }
        else if (cmd_control -> mode_cmd == NORMAL_CONTROL_MODE)
        {
          // if (state != eSuspended) 
          // {
          //   vTaskSuspend(xTaskHandle_ControllerCompute);
          // }
          controller -> PID_Reset(pid_control);
          cmd_control -> throttle_cmd = controller -> control_speed(canbus_data_control -> Throttle_Value, canbus_data_control -> Gear_Shift); 
          pid_control -> pre_throttle = cmd_control -> throttle_cmd;
          entered_cruise = false;
        }
        cmd_control -> steering_angle_cmd = controller -> control_steering_angle(canbus_data_control -> Steering_Angle);
        cmd_control -> state_cmd = controller -> control_direction(canbus_data_control -> Gear_Shift, canbus_data_control -> Connection_State);
        // printf("cmd_control->throttle_cmd Output: %d\n", cmd_control->throttle_cmd);
        // printf("Mode_Controller: %d\n", cmd_control -> mode_cmd);
        // printf("Mode: %d || Steering_Angle: %d || Throttle_Value: %d || Gear_Shift: %d || Connection_State: %d\n", cmd_control -> mode_cmd, cmd_control -> steering_angle_cmd, cmd_control -> throttle_cmd, canbus_data_control -> Gear_Shift, canbus_data_control -> Connection_State);
      }
      else
      {
        /*do not thing*/
      }
      xSemaphoreGive(xControllersReady);
    }
  }
}

void TASK_Actuators(void *pvParameters) {
  static uint8_t angle_position = 0;
  while (1) {
    if (xSemaphoreTake(xControllersReady, portMAX_DELAY) == pdTRUE) 
    {
      if (cmd_control-> state_cmd == BACK_MODE)
      {
        analogWrite(MOTOR_BACK, cmd_control -> throttle_cmd);
        analogWrite(MOTOR_FORWARD, STOP_MODE);
      }
      else if (cmd_control -> state_cmd == FORWARD_MODE)
      {
        analogWrite(MOTOR_BACK, STOP_MODE);
        analogWrite(MOTOR_FORWARD, cmd_control -> throttle_cmd);
      }
      else if (cmd_control-> state_cmd == STOP_MODE)
      {
        analogWrite(MOTOR_BACK, STOP_MODE);
        analogWrite(MOTOR_FORWARD, STOP_MODE);
      }
      else 
      {
        /*do not thing*/
      }
      angle_position = cmd_control -> steering_angle_cmd;
      servo.write(angle_position);
      // printf("Steering_Angle_Control: %d ||Throttle_Value_Control: %d  ||Control_State: %d  \n", angle_position, cmd_control -> throttle_cmd, cmd_control-> state_cmd);
    }
  }
}

void TASK_ControllerCompute(void *pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const uint8_t dt=0.01;
  while (1) 
  {
    eTaskState state = eTaskGetState(xTaskHandle_ControllerCompute);
    // printf("TASK_ControllerCompute State: ");
    // switch (state) 
    // {
    //   case eRunning:   printf("Running\n"); break;
    //   case eReady:     printf("Ready\n"); break;
    //   case eBlocked:   printf("Blocked\n"); break;
    //   case eSuspended: printf("Suspended\n"); break;
    //   case eDeleted:   printf("Deleted\n"); break;
    //   default:         printf("Unknown\n"); break;
    // }
    pid_control->output = controller->PID_Compute(pid_control, canbus_data_control->SpeedSetpoint, vehicle_speed->speed, (float)0.1);
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}
void IMU_calibration()
{
  static float x = 0;
  static float y = 0; 
  static float z = 0;
  static float ax = 0;
  static float ay = 0; 
  static float az = 0;
  static float calib_grox = 0;
  static float calib_groy = 0; 
  static float calib_groz = 0;
  static float calib_accx = 0;
  static float calib_accy = 0; 
  static float calib_accz = 0;
  for (int i=0; i<3000; i++)
  {
    mpu_9150 ->getgyros_values(&x, &y, &z);
    mpu_9150 ->getacc_values(&ax, &ay, &az);
    calib_grox += x;
    calib_groy += y;
    calib_groz += z;
    calib_accx += ax;
    calib_accy += ay;
    calib_accz += az;
    delay(1);

  }
  mpu_9150 -> offsetRateRoll = calib_grox / 3000;
  mpu_9150 -> offsetRatePitch = calib_groy / 3000;
  mpu_9150 -> offsetRateYaw = calib_groz / 3000;
  mpu_9150 -> offsetAccX = calib_accx / 3000;
  mpu_9150 -> offsetAccY = calib_accy / 3000;
  mpu_9150 -> offsetAccZ = calib_accz / 3000;

  printf("offsetRateRoll: %f\n", mpu_9150 -> offsetRateRoll);
  printf("offsetRatePitch: %f\n", mpu_9150 -> offsetRatePitch);
  printf("offsetRateYaw: %f\n", mpu_9150 -> offsetRateYaw);
  printf("offsetAccZ: %f\n", mpu_9150 -> offsetAccZ);

}

void setup() {
  Serial.begin(115200);
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  const int rx_queue_size = 100;       // Receive Queue size
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();

  servo.attach(SERVO_PIN);
  pinMode(MOTOR_BACK, OUTPUT);
  pinMode(MOTOR_FORWARD, OUTPUT);
  pinMode(BATT_VOL_FEEDBACK, INPUT);
  pinMode(CURRENT_FEEDBACK, INPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(ADDRESS_IMU); 
  Wire.write((uint8_t)PWR_MGMT_1_REG);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();

  Wire.beginTransmission(ADDRESS_IMU);
  Wire.write((uint8_t)CONFIG_REG);
  Wire.write(0x05); // DLPF_CFG value  = 5
  
  Wire.endTransmission(); 
  
  IMU_calibration();

  xCANBusReady = xSemaphoreCreateBinary();
  xControllersReady = xSemaphoreCreateBinary();
  xActuatorReady = xSemaphoreCreateBinary();

  // Create the timer (trigger every 5ms)
  xCANBusTimer_Send = xTimerCreate("CANBusTimer_Send", pdMS_TO_TICKS(10), pdTRUE, NULL, CANBusTimerCallback_Send);
  xCANBusTimer_Receive = xTimerCreate("CANBusTimer_Receive", pdMS_TO_TICKS(5), pdTRUE, NULL, CANBusTimerCallback_Receive);
  // Start the timer
  xTimerStart(xCANBusTimer_Send, 0);
  xTimerStart(xCANBusTimer_Receive, 0);
  
  encoderQueue = xQueueCreate(20, sizeof(int[2]));

  xTaskCreatePinnedToCore(TASK_ProcessEncoder, "TASK_ProcessEncoder", 4*2048, NULL, 3, NULL, CORE_0);
  xTaskCreatePinnedToCore(TASK_ReadSensors, "TASK_ReadSensors", 2048, NULL, 2, NULL, CORE_0);
  xTaskCreatePinnedToCore(TASK_CANBus_Send, "TASK_CANBus_Send", 2048, NULL, 2, &xTaskHandle_CANBus_Send, CORE_0);
  xTaskCreatePinnedToCore(TASK_CANBus_Receive, "TASK_CANBus_Receive", 2048, NULL, 3, &xTaskHandle_CANBus_Receive, CORE_1);
  xTaskCreatePinnedToCore(TASK_Controllers, "TASK_Controllers", 2048, NULL, 2, NULL, CORE_1);
  xTaskCreatePinnedToCore(TASK_Actuators, "TASK_Actuators", 2048, NULL, 2, NULL, CORE_1);
  xTaskCreatePinnedToCore(TASK_ControllerCompute, "TASK_ControllerCompute", 2048, NULL, 1, &xTaskHandle_ControllerCompute, CORE_1);
  // vTaskSuspend(xTaskHandle_ControllerCompute);

  xSemaphoreGive(xCANBusReady);

  // Configure the Pulse Counter
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = ENCODER_PIN_A,    // Pin where pulses are input
    .ctrl_gpio_num = ENCODER_PIN_B,     // No control pin (-1 means not used)
    .lctrl_mode = PCNT_MODE_KEEP,       // Control mode: don't change count direction
    .hctrl_mode = PCNT_MODE_REVERSE,    // High control mode: don't change count direction
    .pos_mode = PCNT_COUNT_INC,         // Count up on positive edge
    .neg_mode = PCNT_COUNT_DIS,         // Ignore negative edge
    .counter_h_lim = 16382,             // Maximum count limit
    .counter_l_lim = -16382,            // Minimum count limit
    .unit = PCNT_UNIT_0,                // PCNT unit number
    .channel = PCNT_CHANNEL_0           // PCNT channel (0 or 1)
  };

  pcnt_set_filter_value(PCNT_UNIT, 10);
  pcnt_filter_enable(PCNT_UNIT);
  // Initialize PCNT unit
  pcnt_unit_config(&pcnt_config);
  // Reset the counter
  pcnt_counter_clear(PCNT_UNIT);
  // Start the counter
  pcnt_counter_resume(PCNT_UNIT);

  servo.write(DEFAULT_STEERING_ANGLE);
  analogWrite(MOTOR_BACK, 0);
  analogWrite(MOTOR_FORWARD, 0);

  adc1_config_width(ADC_WIDTH_BIT_12); 
  adc1_config_channel_atten(BATT_PIN, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(CURRENT_PIN, ADC_ATTEN_DB_12);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  controller ->PID_Init(pid_control, 0.1, 0, 0.01);
  
}

void loop() 
{

}

