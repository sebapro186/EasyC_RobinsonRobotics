// pid_ultrasonic.c : implementation file
#include "Main.h"

void pid_ultrasonic ( 
     int max_motor_power,                       //maximum signal to motors   
     long target_sensor_position,               //this is the target sensor value, where we want the robot to be
     unsigned char ultrasonic_interrupt_port,   //this is the digital output port number where we connected the ultrasonic input cable
     unsigned char ultrasonic_input_port,       //this is the digital input port number where we connected the ultrasonic output cable
     int tolerance,                             //this is how much you can deviate from the target to stop
     float Kp,                                  //Proportional constant
     float Ki,                                  //Integral constant
     float Kd                                   //Derivative constant
     )
{
    //Initialize variables
    float error = 0.0; 
    float last_error = 0.0;
    long current_sensor_position = 0;
    int corrected_motor_power=0;
    int new_motor_power = 0; 
    float integral = 0.0; 
    float derivative = 0.0;
    float motor_correction = 0.0;
    long stop_loop = 10;
    char start_countdown = 0;

    while ( stop_loop )
    {
        current_sensor_position = GetUltrasonic ( ultrasonic_interrupt_port , ultrasonic_input_port );
        error =  (float)current_sensor_position - (float)target_sensor_position;
        integral = integral + error;
        derivative = error - last_error;
        motor_correction = (Kp*error) + (Ki*integral) + (Kd*derivative);
        new_motor_power = (int)motor_correction;

        //keep motor power to a value between max_motor_power
        if (new_motor_power > max_motor_power)
        {
            corrected_motor_power = max_motor_power;
        }
        else if (new_motor_power < ( -1 * max_motor_power ))
        {
            corrected_motor_power = ( -1 * max_motor_power );
        }
        else
        {
            corrected_motor_power = new_motor_power;
        }
        forward_backward (corrected_motor_power);
        last_error = error;

        //after the error is less than the tolerance, stop after a few loops
        //we do this to let the pid contlol function to have a chance to correct itself if an overshoot occurs
        //the number of loops is saved in the stop_loop variable
        if ( error <= (float) tolerance )
        {
            start_countdown = 1; //toggles the start_count to one to start decrementing stop_loop on each loop
        }
        if ( start_countdown == 1 )
        {
            --stop_loop; //decrement by one each loop. the while will stop when stop_loop reaches 0
        }

        //Print information to graphic display
        PrintTextToGD ( 0 , 0 , 0 , "current_sensor_position = %d\n" , current_sensor_position ) ;
        PrintTextToGD ( 2 , 0 , 0 , "error = %f\n" , error ) ;
        PrintTextToGD ( 3 , 0 , 0 , "motor_correction = %f\n" , motor_correction ) ;
        PrintTextToGD ( 4 , 0 , 0 , "corrected_motor_power = %d\n" , corrected_motor_power ) ;
        PrintTextToGD ( 5 , 0 , 0 , "Kp = %f\n" , Kp ) ;
        PrintTextToGD ( 6 , 0 , 0 , "Ki = %f\n" , Ki ) ;
        PrintTextToGD ( 7 , 0 , 0 , "Kd = %f\n" , Kd ) ;
    }
    forward_backward (0);
}






