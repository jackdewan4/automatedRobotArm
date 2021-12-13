#include "RobotKit.h"

// **************************************************************************************
// 							   	  KitBot Class Functions
// **************************************************************************************

// ***** Constructor *****

KitBot::KitBot():GRobot(){

	// Assign initial position
    
    _x=SHOULDER_OFFSET;
    _y=0;
    _z=LENGTH_ARM_1+LENGTH_ARM_2+CLAW_LENGTH;
    _R=0;
    _P=90;
    _Y=0;
    
	// Assign initial thetas
    
    _theta1=FORWARD_COMMAND_WAIST;
    _theta2=VERTICAL_COMMAND_SHOULDER;
    _theta3=STRAIGHT_ARM_COMMAND_ELBOW;
    _theta4=STRAIGHT_ARM_COMMAND_PITCH;
    _theta5=ZERO_ROLL_POSITION;
    _theta6=GRIPPER_CLOSE_COMMAND;
    
    // Ensure it is not enabled
    
    _is_enabled=false;

    // Ensure no estop is triggered

    _estop_triggered=false;

    // Save time

    _last_update=millis();
	
}

// ***** Servo Motor Functions *****

void KitBot::enableRobot(){

  // Attach servo motors
  waistMotor.attach(WAIST_PIN);
  shoulderMotor.attach(SHOULDER_PIN);
  elbowMotor.attach(ELBOW_PIN);
  pitchMotor.attach(PITCH_PIN);
  rollMotor.attach(ROLL_PIN);
  gripperMotor.attach(GRIPPER_PIN);
  
  if (!_estop_triggered){
	   _is_enabled=true;
  }

}


void KitBot::disableRobot(){

	_is_enabled=false;

}

bool KitBot::is_it_enabled(){
  return _is_enabled;
}

void KitBot::updateServos(float waistCommand, float shoulderCommand, float elbowCommand, float pitchCommand, 
    float rollCommand, float gripperCommand) {

  if(_is_enabled){ // Check if robot is enabled
    
    if ((millis()-_last_update)>SERVO_UPDATE_PERIOD){ // Check if it has been long enough since update

      // Update servo motors
      waistMotor.write(waistCommand);
      shoulderMotor.write(shoulderCommand);
      elbowMotor.write(elbowCommand);
      pitchMotor.write(pitchCommand);
      rollMotor.write(rollCommand);
      gripperMotor.write(gripperCommand);

      _last_update=millis();
      
    }
  }

}

// ***** Inverse kinematics for the robot *****

bool KitBot::inverse_kinematics(float x, float y, float z, float R, float P, float Y){

	/* Check limits for Gripper, Roll and Pitch */

	if (Y>=100.00001||Y<=-0.00001) {

		  return false;
	
	}
	
	if (R>=90.00001||R<=-0.00001) {

		  return false;
		
	}
	
	if (P>=90.00001||P<=-90.00001){

		  return false;
	
	}
    
    // Save old theta values
    float theta1old=_theta1;
    float theta2old=_theta2;
    float theta3old=_theta3;
    float theta4old=_theta4;
    
    // Calculate theta1
    _theta1=atan2(y,x)*R2D;
    
    float pitch_claw_horizontal=CLAW_LENGTH*fcos(int(P));
    float pitch_claw_vertical=CLAW_LENGTH*fsin(int(P));

    Serial.println(pitch_claw_horizontal);
    
    // Determine total x and y considering the shoulder offset and pitch angle
    float x_c=x-(SHOULDER_OFFSET+pitch_claw_horizontal)*fcos(int(_theta1));
    float y_c=y-(SHOULDER_OFFSET+pitch_claw_horizontal)*fsin(int(_theta1));
    
    // Calculate variable D which helps in calculations and in determining if position
    // can be reached
    float D=(x_c*x_c+y_c*y_c+(z-(HEIGHT_2_SECOND_MOTOR+pitch_claw_vertical))*(z-(HEIGHT_2_SECOND_MOTOR+pitch_claw_vertical))-LENGTH_ARM_1*LENGTH_ARM_1-LENGTH_ARM_2*LENGTH_ARM_2)/(2.0*LENGTH_ARM_1*LENGTH_ARM_2);
    //Calculate theta3
    _theta3=atan2(-sqrt(1-D*D),D)*R2D;
        
    //Calculate theta2
    _theta2=atan2(z-(HEIGHT_2_SECOND_MOTOR+pitch_claw_vertical),sqrt(x_c*x_c+y_c*y_c))-atan2(LENGTH_ARM_2*fsin(int(_theta3)),LENGTH_ARM_1+LENGTH_ARM_2*fcos(int(_theta3)));
    _theta2=_theta2*R2D;
        
    //Calculate theta4 to hold position
    _theta4=P-(_theta2)-(_theta3);

    Serial.println(D);
    
    /* Check if position can't be reached (D>1) */
    if (D>1) {

        // If position cannot be reached, hold position
        _theta1=theta1old;
        _theta2=theta2old;
        _theta3=theta3old;
        _theta4=theta4old;
        
        return false;
        
    }
    
    /* Check if theta 4 command exceeds limit and if so, truncate to limit */
    if (abs(_theta4)>90) {
        if (_theta4<0) {
            _theta4=-90;
        } else {
            _theta4=90;
        }
    }
    
    /* Final conversion */
    
    _theta1=map(_theta1,0,90,FORWARD_COMMAND_WAIST,LEFT_COMMAND_WAIST);
    _theta2=map(_theta2,0,90,HORIZONTAL_COMMAND_SHOULDER,VERTICAL_COMMAND_SHOULDER);
    _theta3=map(_theta3,0,-90,STRAIGHT_ARM_COMMAND_ELBOW,PERPENDICULAR_DOWN_COMMAND_ELBOW);
    _theta4=map(_theta4,0,-90,STRAIGHT_ARM_COMMAND_PITCH,PERPENDICULAR_DOWN_COMMAND_PITCH);
    
    /* Check limits */
    
    if(_theta1>MAX_COMMAND_WAIST||_theta1<MIN_COMMAND_WAIST){

    	  _theta1=theta1old;
        _theta2=theta2old;
        _theta3=theta3old;
        _theta4=theta4old;
        
        return false;
    	
    }
    
    if (_theta2>MAX_COMMAND_SHOULDER||_theta2<MIN_COMMAND_SHOULDER){

    	  _theta1=theta1old;
        _theta2=theta2old;
        _theta3=theta3old;
        _theta4=theta4old;
        
        return false;
    
    }
    
    if (_theta3>MAX_COMMAND_ELBOW||_theta3<MIN_COMMAND_ELBOW){

    	  _theta1=theta1old;
        _theta2=theta2old;
        _theta3=theta3old;
        _theta4=theta4old;
        
        return false;
    
    }
    
    if (_theta4>MAX_COMMAND_PITCH||_theta4<MIN_COMMAND_PITCH){

    	  _theta1=theta1old;
        _theta2=theta2old;
        _theta3=theta3old;
        _theta4=theta4old;
        
        return false;
    
    }
    
    /* Set roll and claw */
    
    _theta5=map(R,0,90,ZERO_ROLL_POSITION,PERPENDICULAR_ROLL_POSITION);
    _theta6=map(Y,0,100,GRIPPER_CLOSE_COMMAND,GRIPPER_OPEN_COMMAND);
    
    return true;
    
}

// ***** Delay functions for the robot *****

void KitBot::delay_while_updating(long time_in_millis){

  long start=millis();
  long now=start;

  while ((now-start)<time_in_millis){
    
    int waistCommand=get_theta1_traj(now);
    int shoulderCommand=get_theta2_traj(now);
    int elbowCommand=get_theta3_traj(now);
    int pitchCommand=get_theta4_traj(now);
    int rollCommand=get_theta5_traj(now);
    int gripperCommand=get_theta6_traj(now);

    updateServos(waistCommand, shoulderCommand, elbowCommand, pitchCommand, 
    rollCommand, gripperCommand);

    now=millis();
    
  }
  
}

void KitBot::delay_after_finish(long time_in_millis=0){

  long now=millis();

  while (!motion_completed()){
    
    int waistCommand=get_theta1_traj(now);
    int shoulderCommand=get_theta2_traj(now);
    int elbowCommand=get_theta3_traj(now);
    int pitchCommand=get_theta4_traj(now);
    int rollCommand=get_theta5_traj(now);
    int gripperCommand=get_theta6_traj(now);

    updateServos(waistCommand, shoulderCommand, elbowCommand, pitchCommand, 
    rollCommand, gripperCommand);

    now=millis();
    
  }

  long start=millis();

  while ((now-start)<time_in_millis){
    
    int waistCommand=get_theta1_traj(now);
    int shoulderCommand=get_theta2_traj(now);
    int elbowCommand=get_theta3_traj(now);
    int pitchCommand=get_theta4_traj(now);
    int rollCommand=get_theta5_traj(now);
    int gripperCommand=get_theta6_traj(now);

    updateServos(waistCommand, shoulderCommand, elbowCommand, pitchCommand, 
    rollCommand, gripperCommand);

    now=millis();
    
  }
  
}


    
