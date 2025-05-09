#include "Controllers.h"
#include "ReadSensors.h"
#include "CANBus.h"
#include <Arduino.h>

Controller* Controller::instance_controllers = nullptr;
Controller::Cmd_Control* Controller::instance_cmd = nullptr;
Controller::PIDController* Controller::instance_pid = nullptr;
// Hàm lấy instance duy nhất của Controller
Controller* Controller::getInstance() {
  if (instance_controllers == nullptr) {
      instance_controllers = new Controller();
  }
  return instance_controllers;
}

Controller::Controller()
{
  instance_cmd = new Cmd_Control();
  instance_pid = new PIDController();
  instance_cmd -> throttle_cmd = 0;
  instance_cmd -> steering_angle_cmd = 0;
  instance_cmd -> state_cmd = STOP_MODE;
  instance_pid -> integral = 0.0f;
  instance_pid -> prev_error = 0.0f;
  instance_pid -> output = 0.0f;
  instance_pid -> Kp = 0.0f;
  instance_pid -> Ki = 0.0f;
  instance_pid -> Kd = 0.0f;
  instance_pid -> prev_error = 0.0f;
}

Controller::~Controller()
{
  delete instance_controllers;
  instance_controllers = nullptr;
  delete instance_cmd;
  instance_cmd = nullptr;
  delete instance_pid;
  instance_pid = nullptr;
}

uint8_t Controller::control_steering_angle(int16_t raw_angle)
{
  static uint8_t position_angle;
  position_angle = map(raw_angle, -450, 450, 180, 0);
  return position_angle;
}

uint8_t Controller::control_speed(int16_t throttle, uint8_t gear_shift)
{
  static uint8_t speed;
  if (gear_shift == 1 && throttle != 0)
  {
    speed = map(throttle, 0, 100, 0, 40);
  }
  else if (gear_shift == 2 && throttle != 0)
  {
    speed = map(throttle, 0, 100, 0, 70);
  }
  else if (gear_shift == 3 && throttle != 0)
  {
    speed = map(throttle, 0, 100, 0, 160);
  }
  else if (gear_shift == 4 && throttle != 0)
  {
    speed = map(throttle, 0, 100, 0, 200);
  }
  else if (gear_shift == 5 && throttle != 0)
  {
    speed = map(throttle, 0, 100, 0, 230);
  }
  else if (gear_shift == 6 && throttle != 0)
  {
    speed = map(throttle, 0, 100, 0, 255);
  }
  else if (gear_shift == 7 && throttle != 0)
  {
    speed = map(throttle, 0, 100, 0, 120);
  }
  else if (gear_shift == 0 || throttle == 0)
  {
    speed = 0;
  }
  return speed;
}

uint8_t Controller::control_direction(uint8_t gearshift, uint8_t connection_state)
{
  static uint8_t state;
  if (gearshift == 7 && connection_state == 1)
  {
    state = BACK_MODE;
  }
  else if (gearshift == 0 || connection_state == 0)
  {
    state = STOP_MODE;
  }
  else if ( (gearshift >= 1 || gearshift <= 6) && connection_state == 1)
  {
    state = FORWARD_MODE;
  }
  return state;
}

Controller::Cmd_Control* Controller::getCmdControl() {
  return instance_cmd;
}

Controller::PIDController* Controller::getPIDController() {
  return instance_pid;
}

void Controller::PID_Init(PIDController *pid, float Kp, float Ki, float Kd) 
{
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->prev_error = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
}

float Controller::PID_Compute (PIDController *pid, float setpoint, float measured, float dt) 
{
  static float error = 0.0f;
  static float output = 0.0f;
  error = setpoint - measured;
  pid->integral += error * dt;
  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;
  output += pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
  if (abs(output) > 255) 
  {
    output = 255;
  } 
  return output;
}

uint8_t Controller::controller_mode(uint8_t controllersMode, uint8_t current_speed, uint8_t current_direction, uint8_t throttle_value, uint8_t brake_value, uint8_t gearshift_value)
{
  static uint8_t mode;
  if (controllersMode == true && current_speed > MINIMUM_SPEED_SETPOINT && current_direction == FORWARD_MODE && brake_value == NON_BRAKE_VALUE && gearshift_value != GEAR_N && gearshift_value != GEAR_R)
  {
    mode = SPEED_CONTROL_MODE;
  }
  else 
  {
    mode = NORMAL_CONTROL_MODE;
  }
  return mode;
}
void Controller::PID_Reset(PIDController *pid) 
{
  pid->prev_error = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
}