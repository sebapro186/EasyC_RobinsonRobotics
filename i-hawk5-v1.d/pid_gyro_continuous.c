// pid_gyro_continuous.c : implementation file
#include "Main.h"

void pid_gyro_continuous (
int max_motor_power,            //maximum signal to motors must be a positive number less or equal to 127
long target_sensor_position,    //this is the target sensor value, where we want the robot to be
unsigned char gyro_analog_port, //this is the analog port number where we connected the gyro
int tolerance,                  //this is how much you can deviate from the target to stop
float Kp,                       //Proportional constant
float Ki,                       //Integral constant
float Kd                        //Derivative constant
)
{
	//Initialize variables
	float error = 0.0;
	float last_error = 0.0;
	long current_sensor_position = 0;
	long corrected_sensor_position = 0;
	long gyro_drift_correction = 0;
	int corrected_motor_power=0;
	int new_motor_power = 0;
	float integral = 0.0;
	float derivative = 0.0;
	float motor_correction = 0.0;
	unsigned long timer1 = 0;       //used to correct gyro drift every 1 to 2 seconds
	unsigned long prev_timer1 = 0;

	//set timer variables
	timer1 = GetTimer ( 1 );
	prev_timer1 = timer1;

	while ( 1 )
	{
		current_sensor_position = GetGyroAngle (gyro_analog_port);
		corrected_sensor_position = current_sensor_position + gyro_drift_correction;
		error =  (float)target_sensor_position - (float)corrected_sensor_position;
		integral = integral + error;
		derivative = error - last_error;
		motor_correction = (Kp*error) + (Ki*integral) + (Kd*derivative);
		//motor_correction = Kp*error;
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
		rotate (corrected_motor_power);
		last_error = error;

		//correct gyro drift
		//gyro monitoring shows a drift between 1/10 deg/2sec to 2/10 deg/sec
		//selected a correction is 1 gyro unit (1/10 deg) per 1.5 seconds
		timer1 = GetTimer ( 1 ) ;
		if ( (timer1 - prev_timer1) >= 1500 )
		{
			gyro_drift_correction = gyro_drift_correction - 1;
			prev_timer1 = timer1;
		}

		//Print information to graphic display
		PrintTextToGD ( 0 , 0 , 0 , "current_sensor_position = %d\n" , current_sensor_position ) ;
		PrintTextToGD ( 1 , 0 , 0 , "corrected_sensor_position = %d\n" , corrected_sensor_position ) ;
		PrintTextToGD ( 2 , 0 , 0 , "gyro_drift_correction = %d\n" , gyro_drift_correction ) ;
		PrintTextToGD ( 3 , 0 , 0 , "error = %f\n" , error ) ;
		PrintTextToGD ( 4 , 0 , 0 , "motor_correction = %f\n" , motor_correction ) ;
		PrintTextToGD ( 5 , 0 , 0 , "corrected_motor_power = %d\n" , corrected_motor_power ) ;
		PrintTextToGD ( 6 , 0 , 0 , "Kp = %f\n" , Kp ) ;
		PrintTextToGD ( 7 , 0 , 0 , "Ki = %f\n" , Ki ) ;
		PrintTextToGD ( 8 , 0 , 0 , "Kd = %f\n" , Kd ) ;
		PrintTextToGD ( 9 , 0 , 0 , "timer1 = %d\n" , timer1 ) ;
		PrintTextToGD ( 10 , 0 , 0 , "prev_timer1 = %d\n" , prev_timer1 ) ;
	}
	rotate (0);
	//    StopGyro (1);
}
