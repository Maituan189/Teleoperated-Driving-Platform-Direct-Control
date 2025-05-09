#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#define SERVO_PIN     13
#define MOTOR_FORWARD 25
#define MOTOR_BACK    26
#define DEFAULT_STEERING_ANGLE 90
#define STOP_MODE 0
#define FORWARD_MODE 1
#define BACK_MODE 2

#define SPEED_CONTROL_MODE 2
#define NORMAL_CONTROL_MODE 1
#define MINIMUM_SPEED_SETPOINT 10
#define NON_THROTTLE_VALUE 0
#define NON_BRAKE_VALUE 0
#define GEAR_N 0
#define GEAR_1 1
#define GEAR_2 2
#define GEAR_3 3
#define GEAR_4 4
#define GEAR_5 5
#define GEAR_6 6
#define GEAR_R 7
#include <Arduino.h> 
class Controller
{
public:
    // Hàm lấy instance duy nhất của Controller
    static Controller* getInstance();

    // Constructor và Destructor
    Controller(const Controller&) = delete; // Không cho phép sao chép
    Controller& operator=(const Controller&) = delete; // Không cho phép gán
    ~Controller();

    typedef struct
    {
        uint8_t throttle_cmd;
        uint8_t steering_angle_cmd;
        uint8_t state_cmd;
        uint8_t mode_cmd;
    } Cmd_Control;

    typedef struct {
        float Kp;       
        float Ki;        
        float Kd;        
        float prev_error; 
        float integral;  
        float output;
        float pre_throttle;
    } PIDController;
    
    // Truy cập dữ liệu điều khiển
    Cmd_Control* getCmdControl();
    PIDController* getPIDController();

    void PID_Init(PIDController *pid, float Kp, float Ki, float Kd);
    float PID_Compute(PIDController *pid, float setpoint, float measured, float dt);
    void PID_Reset(PIDController *pid);
    uint8_t control_steering_angle(int16_t angle);
    uint8_t control_speed(int16_t throttle, uint8_t gear_shift);
    uint8_t control_direction(uint8_t raw_state, uint8_t connection_state);
    uint8_t controller_mode(uint8_t controllersMode, uint8_t current_speed, uint8_t current_direction, uint8_t throttle_value, uint8_t brake_value, uint8_t gearshift_value);
    
    private:
    Controller();
    static Cmd_Control* instance_cmd;
    static PIDController* instance_pid;
    static Controller* instance_controllers;
};


#endif // CONTROLLERS_H