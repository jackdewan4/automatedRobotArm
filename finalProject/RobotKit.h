#include "Arduino.h"
#include <math.h>
#include <avr/pgmspace.h>
#include "Servo.h"
#include "Generic_robot.h"

// **************************************************************************************
// 									Define Robot Pins
// **************************************************************************************

#define WAIST_PIN 2
#define SHOULDER_PIN 3
#define ELBOW_PIN 4
#define PITCH_PIN 5
#define ROLL_PIN 6
#define GRIPPER_PIN 7

#define SERVO_UPDATE_PERIOD 5

// **************************************************************************************
// 								 Robot Calibration Values
// **************************************************************************************

#define MIN_COMMAND_WAIST 0
#define MAX_COMMAND_WAIST 175   // editted values min + max for waist
#define FORWARD_COMMAND_WAIST 90
#define LEFT_COMMAND_WAIST 180

#define MIN_COMMAND_SHOULDER 55    // editted values min + max for shoulder   
#define MAX_COMMAND_SHOULDER 165
#define HORIZONTAL_COMMAND_SHOULDER 133
#define VERTICAL_COMMAND_SHOULDER 72

#define MIN_COMMAND_ELBOW 90
#define MAX_COMMAND_ELBOW 171
#define STRAIGHT_ARM_COMMAND_ELBOW 17
#define PERPENDICULAR_DOWN_COMMAND_ELBOW 86

#define MIN_COMMAND_PITCH 18
#define MAX_COMMAND_PITCH 180
#define STRAIGHT_ARM_COMMAND_PITCH 90 
#define PERPENDICULAR_DOWN_COMMAND_PITCH 167

#define ZERO_ROLL_POSITION 0
#define PERPENDICULAR_ROLL_POSITION 180

#define GRIPPER_CLOSE_COMMAND 156   
#define GRIPPER_OPEN_COMMAND 67

#define HEIGHT_2_SECOND_MOTOR 9.5 //millimetres
#define LENGTH_ARM_1 2.5 //millimetres
#define LENGTH_ARM_2 10.5 //millimetres
#define SHOULDER_OFFSET 9.6 //millimetres
#define CLAW_LENGTH 13.1 //millimetres

// **************************************************************************************
// 								 KitBot Class Definition
// **************************************************************************************

class KitBot : public GRobot{
	
	public:

    Servo waistMotor;
    Servo shoulderMotor;
    Servo elbowMotor;
    Servo pitchMotor;
    Servo rollMotor;
    Servo gripperMotor;
  
		// Constructor
		KitBot(); 
		
		// Update servo motor commands
		void updateServos(float waistCommand, float shoulderCommand, float elbowCommand, float pitchCommand, 
		float rollCommand, float gripperCommand);
		
		// Enable Robot
		void enableRobot();
		
		// Disable robot
		void disableRobot();

    // Get if the robot is enabled
    bool is_it_enabled();

    // Delays code execution for a fixed amount of time while updating the servos
    void delay_while_updating(long time_in_millis);

    // Delays code execution for a fixed amount of time after the robot has finished moving
    // while updating servos
    void delay_after_finish(long time_in_millis);
	
	private:
		
		// Inverse kinematics function for the robot kit, returns true if command is valid
		bool inverse_kinematics(float x, float y, float z, float R, float P, float Y);
		// Note: Y controls the gripper with 0 being fully open and 100 being fully closed
		
		// Make an enable state and an emergency state
		
		bool _is_enabled;
    bool _estop_triggered;

   // Save time of last update

   long _last_update;
	
};
