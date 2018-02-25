// UserInclude.h : header file
#ifndef USERINCLUDE_H_
#define USERINCLUDE_H_

// TODO: add user code here
void pid_custom ( int steady_motor_power, long target_sensor_position, unsigned char gyro_analog_port, int tolerance, float Kp, float Ki, float Kd );
void pid_gyro ( int steady_motor_power, long target_sensor_position, unsigned char gyro_analog_port, int tolerance, float Kp, float Ki, float Kd );
void pid_gyro_continuous ( int steady_motor_power, long target_sensor_position, unsigned char gyro_analog_port, int tolerance, float Kp, float Ki, float Kd );
void pid_ultrasonic ( int max_motor_power, long target_sensor_position, unsigned char ultrasonic_interrupt_port, unsigned char ultrasonic_input_port, int tolerance, float Kp, float Ki, float Kd );
void pid_ultrasonic_continuous ( int max_motor_power, long target_sensor_position, unsigned char ultrasonic_interrupt_port, unsigned char ultrasonic_input_port, int tolerance, float Kp, float Ki, float Kd );

#endif // USERINCLUDE_H_








