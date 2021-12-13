#include "Generic_robot.h"
#include "Arduino.h"
#include <math.h>
#include <avr/pgmspace.h>

// **************************************************************************************
// 							  Constants used in calculations
// **************************************************************************************

// Sine and cosine look up tables

const PROGMEM float ssin[361]={0,-0.017452,-0.034899,-0.052336,-0.069756,-0.087156,-0.10453,-0.12187,-0.13917,-0.15643,-0.17365,-0.19081,-0.20791,-0.22495,-0.24192,-0.25882,-0.27564,-0.29237,-0.30902,-0.32557,-0.34202,-0.35837,-0.37461,-0.39073,-0.40674,-0.42262,-0.43837,-0.45399,-0.46947,-0.48481,-0.5,-0.51504,-0.52992,-0.54464,-0.55919,-0.57358,-0.58779,-0.60182,-0.61566,-0.62932,-0.64279,-0.65606,-0.66913,-0.682,-0.69466,-0.70711,-0.71934,-0.73135,-0.74314,-0.75471,-0.76604,-0.77715,-0.78801,-0.79864,-0.80902,-0.81915,-0.82904,-0.83867,-0.84805,-0.85717,-0.86603,-0.87462,-0.88295,-0.89101,-0.89879,-0.90631,-0.91355,-0.9205,-0.92718,-0.93358,-0.93969,-0.94552,-0.95106,-0.9563,-0.96126,-0.96593,-0.9703,-0.97437,-0.97815,-0.98163,-0.98481,-0.98769,-0.99027,-0.99255,-0.99452,-0.99619,-0.99756,-0.99863,-0.99939,-0.99985,-1,-0.99985,-0.99939,-0.99863,-0.99756,-0.99619,-0.99452,-0.99255,-0.99027,-0.98769,-0.98481,-0.98163,-0.97815,-0.97437,-0.9703,-0.96593,-0.96126,-0.9563,-0.95106,-0.94552,-0.93969,-0.93358,-0.92718,-0.9205,-0.91355,-0.90631,-0.89879,-0.89101,-0.88295,-0.87462,-0.86603,-0.85717,-0.84805,-0.83867,-0.82904,-0.81915,-0.80902,-0.79864,-0.78801,-0.77715,-0.76604,-0.75471,-0.74314,-0.73135,-0.71934,-0.70711,-0.69466,-0.682,-0.66913,-0.65606,-0.64279,-0.62932,-0.61566,-0.60182,-0.58779,-0.57358,-0.55919,-0.54464,-0.52992,-0.51504,-0.5,-0.48481,-0.46947,-0.45399,-0.43837,-0.42262,-0.40674,-0.39073,-0.37461,-0.35837,-0.34202,-0.32557,-0.30902,-0.29237,-0.27564,-0.25882,-0.24192,-0.22495,-0.20791,-0.19081,-0.17365,-0.15643,-0.13917,-0.12187,-0.10453,-0.087156,-0.069756,-0.052336,-0.034899,-0.017452,0,0.017452,0.034899,0.052336,0.069756,0.087156,0.10453,0.12187,0.13917,0.15643,0.17365,0.19081,0.20791,0.22495,0.24192,0.25882,0.27564,0.29237,0.30902,0.32557,0.34202,0.35837,0.37461,0.39073,0.40674,0.42262,0.43837,0.45399,0.46947,0.48481,0.5,0.51504,0.52992,0.54464,0.55919,0.57358,0.58779,0.60182,0.61566,0.62932,0.64279,0.65606,0.66913,0.682,0.69466,0.70711,0.71934,0.73135,0.74314,0.75471,0.76604,0.77715,0.78801,0.79864,0.80902,0.81915,0.82904,0.83867,0.84805,0.85717,0.86603,0.87462,0.88295,0.89101,0.89879,0.90631,0.91355,0.9205,0.92718,0.93358,0.93969,0.94552,0.95106,0.9563,0.96126,0.96593,0.9703,0.97437,0.97815,0.98163,0.98481,0.98769,0.99027,0.99255,0.99452,0.99619,0.99756,0.99863,0.99939,0.99985,1,0.99985,0.99939,0.99863,0.99756,0.99619,0.99452,0.99255,0.99027,0.98769,0.98481,0.98163,0.97815,0.97437,0.9703,0.96593,0.96126,0.9563,0.95106,0.94552,0.93969,0.93358,0.92718,0.9205,0.91355,0.90631,0.89879,0.89101,0.88295,0.87462,0.86603,0.85717,0.84805,0.83867,0.82904,0.81915,0.80902,0.79864,0.78801,0.77715,0.76604,0.75471,0.74314,0.73135,0.71934,0.70711,0.69466,0.682,0.66913,0.65606,0.64279,0.62932,0.61566,0.60182,0.58779,0.57358,0.55919,0.54464,0.52992,0.51504,0.5,0.48481,0.46947,0.45399,0.43837,0.42262,0.40674,0.39073,0.37461,0.35837,0.34202,0.32557,0.30902,0.29237,0.27564,0.25882,0.24192,0.22495,0.20791,0.19081,0.17365,0.15643,0.13917,0.12187,0.10453,0.087156,0.069756,0.052336,0.034899,0.017452,0};

const PROGMEM float ccos[361]={-1,-0.99985,-0.99939,-0.99863,-0.99756,-0.99619,-0.99452,-0.99255,-0.99027,-0.98769,-0.98481,-0.98163,-0.97815,-0.97437,-0.9703,-0.96593,-0.96126,-0.9563,-0.95106,-0.94552,-0.93969,-0.93358,-0.92718,-0.9205,-0.91355,-0.90631,-0.89879,-0.89101,-0.88295,-0.87462,-0.86603,-0.85717,-0.84805,-0.83867,-0.82904,-0.81915,-0.80902,-0.79864,-0.78801,-0.77715,-0.76604,-0.75471,-0.74314,-0.73135,-0.71934,-0.70711,-0.69466,-0.682,-0.66913,-0.65606,-0.64279,-0.62932,-0.61566,-0.60182,-0.58779,-0.57358,-0.55919,-0.54464,-0.52992,-0.51504,-0.5,-0.48481,-0.46947,-0.45399,-0.43837,-0.42262,-0.40674,-0.39073,-0.37461,-0.35837,-0.34202,-0.32557,-0.30902,-0.29237,-0.27564,-0.25882,-0.24192,-0.22495,-0.20791,-0.19081,-0.17365,-0.15643,-0.13917,-0.12187,-0.10453,-0.087156,-0.069756,-0.052336,-0.034899,-0.017452,0,0.017452,0.034899,0.052336,0.069756,0.087156,0.10453,0.12187,0.13917,0.15643,0.17365,0.19081,0.20791,0.22495,0.24192,0.25882,0.27564,0.29237,0.30902,0.32557,0.34202,0.35837,0.37461,0.39073,0.40674,0.42262,0.43837,0.45399,0.46947,0.48481,0.5,0.51504,0.52992,0.54464,0.55919,0.57358,0.58779,0.60182,0.61566,0.62932,0.64279,0.65606,0.66913,0.682,0.69466,0.70711,0.71934,0.73135,0.74314,0.75471,0.76604,0.77715,0.78801,0.79864,0.80902,0.81915,0.82904,0.83867,0.84805,0.85717,0.86603,0.87462,0.88295,0.89101,0.89879,0.90631,0.91355,0.9205,0.92718,0.93358,0.93969,0.94552,0.95106,0.9563,0.96126,0.96593,0.9703,0.97437,0.97815,0.98163,0.98481,0.98769,0.99027,0.99255,0.99452,0.99619,0.99756,0.99863,0.99939,0.99985,1,0.99985,0.99939,0.99863,0.99756,0.99619,0.99452,0.99255,0.99027,0.98769,0.98481,0.98163,0.97815,0.97437,0.9703,0.96593,0.96126,0.9563,0.95106,0.94552,0.93969,0.93358,0.92718,0.9205,0.91355,0.90631,0.89879,0.89101,0.88295,0.87462,0.86603,0.85717,0.84805,0.83867,0.82904,0.81915,0.80902,0.79864,0.78801,0.77715,0.76604,0.75471,0.74314,0.73135,0.71934,0.70711,0.69466,0.682,0.66913,0.65606,0.64279,0.62932,0.61566,0.60182,0.58779,0.57358,0.55919,0.54464,0.52992,0.51504,0.5,0.48481,0.46947,0.45399,0.43837,0.42262,0.40674,0.39073,0.37461,0.35837,0.34202,0.32557,0.30902,0.29237,0.27564,0.25882,0.24192,0.22495,0.20791,0.19081,0.17365,0.15643,0.13917,0.12187,0.10453,0.087156,0.069756,0.052336,0.034899,0.017452,6.1232e-17,-0.017452,-0.034899,-0.052336,-0.069756,-0.087156,-0.10453,-0.12187,-0.13917,-0.15643,-0.17365,-0.19081,-0.20791,-0.22495,-0.24192,-0.25882,-0.27564,-0.29237,-0.30902,-0.32557,-0.34202,-0.35837,-0.37461,-0.39073,-0.40674,-0.42262,-0.43837,-0.45399,-0.46947,-0.48481,-0.5,-0.51504,-0.52992,-0.54464,-0.55919,-0.57358,-0.58779,-0.60182,-0.61566,-0.62932,-0.64279,-0.65606,-0.66913,-0.682,-0.69466,-0.70711,-0.71934,-0.73135,-0.74314,-0.75471,-0.76604,-0.77715,-0.78801,-0.79864,-0.80902,-0.81915,-0.82904,-0.83867,-0.84805,-0.85717,-0.86603,-0.87462,-0.88295,-0.89101,-0.89879,-0.90631,-0.91355,-0.9205,-0.92718,-0.93358,-0.93969,-0.94552,-0.95106,-0.9563,-0.96126,-0.96593,-0.9703,-0.97437,-0.97815,-0.98163,-0.98481,-0.98769,-0.99027,-0.99255,-0.99452,-0.99619,-0.99756,-0.99863,-0.99939,-0.99985,-1};

// **************************************************************************************
// 							  Generic Robot Class Functions
// **************************************************************************************

// ***** Constructor *****

GRobot::GRobot(){
    
    // Assign initial position
    
    _x=0;
    _y=0;
    _z=0;
    _R=0;
    _P=0;
    _Y=0;
    
    // Determine initial thetas
    
    _theta1=0;
    _theta2=0;
    _theta3=0;
    _theta4=0;
    _theta5=0;
    _theta6=0;
    _theta7=0;
    
    // Set initial trajectories
    
    _t_final=0;
    
    _t_blend_1=0;
    _t_blend_2=0;
    _t_blend_3=0;
    _t_blend_4=0;
    _t_blend_5=0;
    _t_blend_6=0;
    _t_blend_7=0;
    
    _v_1=MAX_VELOCITY_1;
    _v_2=MAX_VELOCITY_2;
    _v_3=MAX_VELOCITY_3;
    _v_4=MAX_VELOCITY_4;
    _v_5=MAX_VELOCITY_5;
    _v_6=MAX_VELOCITY_6;
    _v_7=MAX_VELOCITY_7;
    
    _a_1=MAX_ACCELERATION_1;
    _a_2=MAX_ACCELERATION_2;
    _a_3=MAX_ACCELERATION_3;
    _a_4=MAX_ACCELERATION_4;
    _a_5=MAX_ACCELERATION_5;
    _a_6=MAX_ACCELERATION_6;
    _a_7=MAX_ACCELERATION_7;
    
    _offset_d_1=0;
    _offset_d_2=0;
    _offset_d_3=0;
    _offset_d_4=0;
    _offset_d_5=0;
    _offset_d_6=0;
    _offset_d_7=0;
    
    _offset_t=0;

    _motion_complete=false;
    
}

// ***** Robot motion Functions *****

void GRobot::move_xyzRPY(float x, float y, float z, float R, float P, float Y, float t=DEFAULT_TIME){

	bool command_allowed=true;

	float theta1_past=_theta1;
	float theta2_past=_theta2;
	float theta3_past=_theta3;
	float theta4_past=_theta4;
	float theta5_past=_theta5;
	float theta6_past=_theta6;
	float theta7_past=_theta7;

	float blend_1_past=_t_blend_1;
	float blend_2_past=_t_blend_2;
	float blend_3_past=_t_blend_3;
	float blend_4_past=_t_blend_4;
	float blend_5_past=_t_blend_5;
	float blend_6_past=_t_blend_6;
	float blend_7_past=_t_blend_7;

	float v1_past=_v_1;
	float v2_past=_v_2;
	float v3_past=_v_3;
	float v4_past=_v_4;
	float v5_past=_v_5;
	float v6_past=_v_6;
	float v7_past=_v_7;
	
	if (!inverse_kinematics(x,y,z,R,P,Y)) {
	 	
	 	command_allowed=false;
	 	
	}
	
	#ifdef SMOOTH_THETA1
	
	if (command_allowed) {
		if(!get_trajectory_variables(t,(_theta1-theta1_past),_a_1,MAX_VELOCITY_1,_t_blend_1,_v_1)){
			command_allowed=false;
		}
	}
	
	#endif
	
	#ifdef SMOOTH_THETA2
	
	if (command_allowed) {
		if(!get_trajectory_variables(t,(_theta2-theta2_past),_a_2,MAX_VELOCITY_2,_t_blend_2,_v_2)){
			command_allowed=false;
		}
	}
	
	#endif
	
	#ifdef SMOOTH_THETA3
	
	if (command_allowed) {
		if(!get_trajectory_variables(t,(_theta3-theta3_past),_a_3,MAX_VELOCITY_3,_t_blend_3,_v_3)){
			command_allowed=false;
		}
	}
	
	#endif
	
	#ifdef SMOOTH_THETA4
	
	if (command_allowed) {
		if(!get_trajectory_variables(t,(_theta4-theta4_past),_a_4,MAX_VELOCITY_4,_t_blend_4,_v_4)){
			command_allowed=false;
		}
	}
	
	#endif
		
	#ifdef SMOOTH_THETA5
	
	if (command_allowed) {
		if(!get_trajectory_variables(t,(_theta5-theta5_past),_a_5,MAX_VELOCITY_5,_t_blend_5,_v_5)){
			command_allowed=false;
		}
	}
	
	#endif
	
	#ifdef SMOOTH_THETA6
	
	if (command_allowed) {
		if(!get_trajectory_variables(t,(_theta6-theta6_past),_a_6,MAX_VELOCITY_6,_t_blend_6,_v_6)){
			command_allowed=false;
		}
	}
	
	#endif
	
	#ifdef SMOOTH_THETA7
	
	if (command_allowed) {
		if(!get_trajectory_variables(t,(_theta7-theta7_past),_a_7,MAX_VELOCITY_7,_t_blend_7,_v_7)){
			command_allowed=false;
		}
	}
	
	#endif
	
	if (command_allowed){
	
		_x=x;
		_y=y;
		_z=z;
		_R=R;
		_P=P;
		_Y=Y;
		
		_offset_d_1=theta1_past;
		_offset_d_2=theta2_past;
		_offset_d_3=theta3_past;
		_offset_d_4=theta4_past;
		_offset_d_5=theta5_past;
		_offset_d_6=theta6_past;
		_offset_d_7=theta7_past;
		
		_t_final=t;
		
		_offset_t=millis();

   _motion_complete=false;
	
	} else {
	
		_theta1=theta1_past;
		_theta2=theta2_past;
		_theta3=theta3_past;
		_theta4=theta4_past;
		_theta5=theta5_past;
		_theta6=theta6_past;
		_theta7=theta7_past;

    _t_blend_1=blend_1_past;
    _t_blend_2=blend_2_past;
    _t_blend_3=blend_3_past;
    _t_blend_4=blend_4_past;
    _t_blend_5=blend_5_past;
    _t_blend_6=blend_6_past;
    _t_blend_7=blend_7_past;
    
    _v_1=v1_past;
    _v_2=v2_past;
    _v_3=v3_past;
    _v_4=v4_past;
    _v_5=v5_past;
    _v_6=v6_past;
    _v_7=v7_past;
		
	}

}
    
void GRobot::move_xyz(float x, float y, float z, float t=DEFAULT_TIME){

	move_xyzRPY(x,y,z,_R,_P,_Y,t);
	
}

void GRobot::move_RPY(float R, float P, float Y, float t=DEFAULT_TIME){

	move_xyzRPY(_x,_y,_z,R,P,Y,t);

}

void GRobot::move_x(float x, float t=DEFAULT_TIME){

    move_xyzRPY(x,_y,_z,_R,_P,_Y,t);

}

void GRobot::move_y(float y, float t=DEFAULT_TIME){

    move_xyzRPY(_x,y,_z,_R,_P,_Y,t);

}

void GRobot::move_z(float z, float t=DEFAULT_TIME){

    move_xyzRPY(_x,_y,z,_R,_P,_Y,t);

}

void GRobot::move_R(float R, float t=DEFAULT_TIME){

    move_xyzRPY(_x,_y,_z,R,_P,_Y,t);

}

void GRobot::move_P(float P, float t=DEFAULT_TIME){

    move_xyzRPY(_x,_y,_z,_R,P,_Y,t);

}

void GRobot::move_Y(float Y, float t=DEFAULT_TIME){

    move_xyzRPY(_x,_y,_z,_R,_P,Y,t);
}

// ***** Robot relative motion Functions *****

void GRobot::move_xyzRPY_relative(float x, float y, float z, float R, float P, float Y,float t=DEFAULT_TIME){

	move_xyzRPY(_x+x,_y+y,_z+z,_R+R,_P+P,_Y+Y,t);

}
    
void GRobot::move_xyz_relative(float x, float y, float z, float t=DEFAULT_TIME){

	move_xyzRPY(_x+x,_y+y,_z+z,_R,_P,_Y,t);

}

void GRobot::move_RPY_relative(float R, float P, float Y, float t=DEFAULT_TIME){

	move_xyzRPY(_x,_y,_z,_R+R,_P+P,_Y+Y,t);

}

void GRobot::move_x_relative(float x, float t=DEFAULT_TIME){

    move_xyzRPY(_x+x,_y,_z,_R,_P,_Y,t);

}

void GRobot::move_y_relative(float y, float t=DEFAULT_TIME){

    move_xyzRPY(_x,_y+y,_z,_R,_P,_Y,t);

}

void GRobot::move_z_relative(float z, float t=DEFAULT_TIME){

    move_xyzRPY(_x,_y,_z+z,_R,_P,_Y,t);

}

void GRobot::move_R_relative(float R, float t=DEFAULT_TIME){

    move_xyzRPY(_x,_y,_z,_R+R,_P,_Y,t);

}

void GRobot::move_P_relative(float P, float t=DEFAULT_TIME){

    move_xyzRPY(_x,_y,_z,_R,_P+P,_Y,t);

}

void GRobot::move_Y_relative(float Y, float t=DEFAULT_TIME){

    move_xyzRPY(_x,_y,_z,_R,_P,_Y+Y,t);

}

// ***** Robot fast motion Functions *****

void GRobot::move_xyzRPY_fast(float x, float y, float z, float R, float P, float Y){

	float max_time=0;
	float temp_time;

	bool command_allowed=true;

	float theta1_past=_theta1;
	float theta2_past=_theta2;
	float theta3_past=_theta3;
	float theta4_past=_theta4;
	float theta5_past=_theta5;
	float theta6_past=_theta6;
	float theta7_past=_theta7;

  float blend_1_past=_t_blend_1;
  float blend_2_past=_t_blend_2;
  float blend_3_past=_t_blend_3;
  float blend_4_past=_t_blend_4;
  float blend_5_past=_t_blend_5;
  float blend_6_past=_t_blend_6;
  float blend_7_past=_t_blend_7;
    
  float v1_past=_v_1;
  float v2_past=_v_2;
  float v3_past=_v_3;
  float v4_past=_v_4;
  float v5_past=_v_5;
  float v6_past=_v_6;
  float v7_past=_v_7;
	
	if (!inverse_kinematics(x,y,z,R,P,Y)) {
	 	
	 	command_allowed=false;
	 	
	}   
	
	#ifdef SMOOTH_THETA1
	
		temp_time=minimum_time(abs(_theta1-theta1_past),_a_1,MAX_VELOCITY_1);
		if (temp_time>max_time){
			max_time=temp_time;
		}
	
	#endif
	
	#ifdef SMOOTH_THETA2
	
		temp_time=minimum_time(abs(_theta2-theta2_past),_a_2,MAX_VELOCITY_2);
		if (temp_time>max_time){
			max_time=temp_time;
		}
	
	#endif
	
	#ifdef SMOOTH_THETA3
	
		temp_time=minimum_time(abs(_theta3-theta3_past),_a_3,MAX_VELOCITY_3);
		if (temp_time>max_time){
			max_time=temp_time;
		}
	
	#endif
	
	#ifdef SMOOTH_THETA4
	
		temp_time=minimum_time(abs(_theta4-theta4_past),_a_4,MAX_VELOCITY_4);
		if (temp_time>max_time){
			max_time=temp_time;
		}
	
	#endif
	
	#ifdef SMOOTH_THETA5
	
		temp_time=minimum_time(abs(_theta5-theta5_past),_a_5,MAX_VELOCITY_5);
		if (temp_time>max_time){
			max_time=temp_time;
		}
	
	#endif
	
	#ifdef SMOOTH_THETA6
	
		temp_time=minimum_time(abs(_theta6-theta6_past),_a_6,MAX_VELOCITY_6);
		if (temp_time>max_time){
			max_time=temp_time;
		}
	
	#endif
	
	#ifdef SMOOTH_THETA7
	
		temp_time=minimum_time(abs(_theta7-theta7_past),_a_7,MAX_VELOCITY_7);
		if (temp_time>max_time){
			max_time=temp_time;
		}
	
	#endif

  max_time=max_time+EXTRA_TIME_FAST; // Required to ensure that the motion is allowed
  
	#ifdef SMOOTH_THETA1
	
	if (command_allowed) {
		if(!get_trajectory_variables(max_time,(_theta1-theta1_past),_a_1,MAX_VELOCITY_1,_t_blend_1,_v_1)){
			command_allowed=false;
		}
	}
	
	#endif
	
	#ifdef SMOOTH_THETA2
	
	if (command_allowed) {
		if(!get_trajectory_variables(max_time,(_theta2-theta2_past),_a_2,MAX_VELOCITY_2,_t_blend_2,_v_2)){
			command_allowed=false;
		}
	}
	
	#endif
	
	#ifdef SMOOTH_THETA3
	
	if (command_allowed) {
		if(!get_trajectory_variables(max_time,(_theta3-theta3_past),_a_3,MAX_VELOCITY_3,_t_blend_3,_v_3)){
			command_allowed=false;
		}
	}
	
	#endif
	
	#ifdef SMOOTH_THETA4
	
	if (command_allowed) {
		if(!get_trajectory_variables(max_time,(_theta4-theta4_past),_a_4,MAX_VELOCITY_4,_t_blend_4,_v_4)){
			command_allowed=false;
		}
	}
	
	#endif
		
	#ifdef SMOOTH_THETA5
	
	if (command_allowed) {
		if(!get_trajectory_variables(max_time,(_theta5-theta5_past),_a_5,MAX_VELOCITY_5,_t_blend_5,_v_5)){
			command_allowed=false;
		}
	}
	
	#endif
	
	#ifdef SMOOTH_THETA6
	
	if (command_allowed) {
		if(!get_trajectory_variables(max_time,(_theta6-theta6_past),_a_6,MAX_VELOCITY_6,_t_blend_6,_v_6)){
			command_allowed=false;
		}
	}
	
	#endif
	
	#ifdef SMOOTH_THETA7
	
	if (command_allowed) {
		if(!get_trajectory_variables(max_time,(_theta7-theta7_past),_a_7,MAX_VELOCITY_7,_t_blend_7,_v_7)){
			command_allowed=false;
		}
	}
	
	#endif
  
	if (command_allowed){
	
		_x=x;
		_y=y;
		_z=z;
		_R=R;
		_P=P;
		_Y=Y;
		
		_offset_d_1=theta1_past;
		_offset_d_2=theta2_past;
		_offset_d_3=theta3_past;
		_offset_d_4=theta4_past;
		_offset_d_5=theta5_past;
		_offset_d_6=theta6_past;
		_offset_d_7=theta7_past;
		
		_t_final=max_time;
		
		_offset_t=millis();

   _motion_complete=false;
	
	} else {
	
		_theta1=theta1_past;
		_theta2=theta2_past;
		_theta3=theta3_past;
		_theta4=theta4_past;
		_theta5=theta5_past;
		_theta6=theta6_past;
		_theta7=theta7_past;

		_t_blend_1=blend_1_past;
		_t_blend_2=blend_2_past;
		_t_blend_3=blend_3_past;
		_t_blend_4=blend_4_past;
		_t_blend_5=blend_5_past;
		_t_blend_6=blend_6_past;
		_t_blend_7=blend_7_past;
	
		_v_1=v1_past;
		_v_2=v2_past;
		_v_3=v3_past;
		_v_4=v4_past;
		_v_5=v5_past;
		_v_6=v6_past;
		_v_7=v7_past;
    
	}

}
    
void GRobot::move_xyz_fast(float x, float y, float z){

	move_xyzRPY_fast(x,y,z,_R,_P,_Y);
	
}

void GRobot::move_RPY_fast(float R, float P, float Y){

	move_xyzRPY_fast(_x,_y,_z,R,P,Y);

}

void GRobot::move_x_fast(float x){

    move_xyzRPY_fast(x,_y,_z,_R,_P,_Y);

}

void GRobot::move_y_fast(float y){

    move_xyzRPY_fast(_x,y,_z,_R,_P,_Y);

}

void GRobot::move_z_fast(float z){

    move_xyzRPY_fast(_x,_y,z,_R,_P,_Y);

}

void GRobot::move_R_fast(float R){

    move_xyzRPY_fast(_x,_y,_z,R,_P,_Y);

}

void GRobot::move_P_fast(float P){

    move_xyzRPY_fast(_x,_y,_z,_R,P,_Y);

}

void GRobot::move_Y_fast(float Y){

    move_xyzRPY_fast(_x,_y,_z,_R,_P,Y);
}

// ***** Robot fast relative motion Functions *****

void GRobot::move_xyzRPY_relative_fast(float x, float y, float z, float R, float P, float Y){

	move_xyzRPY_fast(_x+x,_y+y,_z+z,_R+R,_P+P,_Y+Y);

}
    
void GRobot::move_xyz_relative_fast(float x, float y, float z){

	move_xyzRPY_fast(_x+x,_y+y,_z+z,_R,_P,_Y);

}

void GRobot::move_RPY_relative_fast(float R, float P, float Y){

	move_xyzRPY_fast(_x,_y,_z,_R+R,_P+P,_Y+Y);

}

void GRobot::move_x_relative_fast(float x){

    move_xyzRPY_fast(_x+x,_y,_z,_R,_P,_Y);

}

void GRobot::move_y_relative_fast(float y){

    move_xyzRPY_fast(_x,_y+y,_z,_R,_P,_Y);

}

void GRobot::move_z_relative_fast(float z){

    move_xyzRPY_fast(_x,_y,_z+z,_R,_P,_Y);

}

void GRobot::move_R_relative_fast(float R){

    move_xyzRPY_fast(_x,_y,_z,_R+R,_P,_Y);

}

void GRobot::move_P_relative_fast(float P){

    move_xyzRPY_fast(_x,_y,_z,_R,_P+P,_Y);

}

void GRobot::move_Y_relative_fast(float Y){

    move_xyzRPY_fast(_x,_y,_z,_R,_P,_Y+Y);

}

// ***** Robot trajectory function *****

bool GRobot::get_trajectory_variables(float final_time, float distance, float acceleration, float max_v, float &blend_time, float &velocity){

	float d=abs(distance);

	float d_max=0;

	if (final_time>(2.0*max_v/acceleration)) {
		d_max=max_v*final_time-max_v*max_v/acceleration;	
	} else {
		d_max=acceleration*final_time*final_time/4.0;
	}
  
	if (d>d_max) {

		return false;
		
	} else {
		
		velocity=0.5*(acceleration*final_time-sqrt(acceleration*acceleration*final_time*final_time-4*d*acceleration));
		blend_time=velocity/acceleration;
    if (d<0.000001){
      velocity=0;
    } else {
		  velocity=velocity*distance/d;
    }
		return true;
		
	}
	
}

// ***** Robot Communication Functions for debugging *****

// Print position of the robot for debugging

String GRobot::print_position(){

    String com="x "+String(_x,DEC);
    com=com+" y "+String(_y,DEC);
    com=com+" z "+String(_z,DEC);
    com=com+" R "+String(_R,DEC);
    com=com+" P "+String(_P,DEC);
    com=com+" Y "+String(_Y,DEC);

    return com;

}

// Print thetas for the sake of debugging

String GRobot::print_thetas(){

    String com="theta1 "+String(_theta1,DEC);
    com=com+" theta2 "+String(_theta2,DEC);
    com=com+" theta3 "+String(_theta3,DEC);
    com=com+" theta4 "+String(_theta4,DEC);
    com=com+" theta5 "+String(_theta5,DEC);
    com=com+" theta6 "+String(_theta6,DEC);
    com=com+" theta7 "+String(_theta7,DEC);

    return com;

}

// ***** Functions to get trajectory values *****

float GRobot::get_theta1_traj(float t){

float out=_theta1;

#ifdef SMOOTH_THETA1
  out=get_traj(t, _offset_t, _t_final, _t_blend_1, _v_1, _a_1, _offset_d_1, _theta1);
  if ((t-_offset_t)>_t_final){
     _motion_complete=true;
  }
#endif

return out;

}

float GRobot::get_theta2_traj(float t){

float out=_theta2;

#ifdef SMOOTH_THETA2
  out=get_traj(t, _offset_t, _t_final, _t_blend_2, _v_2, _a_2, _offset_d_2, _theta2);
  if ((t-_offset_t)>_t_final){
     _motion_complete=true;
  }
#endif

return out;

}

float GRobot::get_theta3_traj(float t){

float out=_theta3;

#ifdef SMOOTH_THETA3
  out=get_traj(t, _offset_t, _t_final, _t_blend_3, _v_3, _a_3, _offset_d_3, _theta3);
  if ((t-_offset_t)>_t_final){
     _motion_complete=true;
  }
#endif

return out;

}
  
float GRobot::get_theta4_traj(float t){

float out=_theta4;

#ifdef SMOOTH_THETA4
  out=get_traj(t, _offset_t, _t_final, _t_blend_4, _v_4, _a_4, _offset_d_4, _theta4);
  if ((t-_offset_t)>_t_final){
     _motion_complete=true;
  }
#endif

return out;

}

float GRobot::get_theta5_traj(float t){

float out=_theta5;

#ifdef SMOOTH_THETA5
  out=get_traj(t, _offset_t, _t_final, _t_blend_5, _v_5, _a_5, _offset_d_5, _theta5);
  if ((t-_offset_t)>_t_final){
     _motion_complete=true;
  }
#endif

return out;

}

float GRobot::get_theta6_traj(float t){

float out=_theta6;

#ifdef SMOOTH_THETA6
  out=get_traj(t, _offset_t, _t_final, _t_blend_6, _v_6, _a_6, _offset_d_6, _theta6);
  if ((t-_offset_t)>_t_final){
     _motion_complete=true;
  }
#endif

return out;

}

float GRobot::get_theta7_traj(float t){

float out=_theta7;

#ifdef SMOOTH_THETA7
  out=get_traj(t, _offset_t, _t_final, _t_blend_7, _v_7, _a_7, _offset_d_7, _theta7);
  if ((t-_offset_t)>_t_final){
     _motion_complete=true;
  }
#endif

return out;

}

// **************************************************************************************
// 							   Additional Useful Functions
// **************************************************************************************

// Fast sine and cosine functions

float fsin(int angle) {
    return pgm_read_float_near(&ssin[angle+180]);
}

float fcos(int angle) {
    return pgm_read_float_near(&ccos[angle+180]);
}

// Elbow function

bool elbow_calculation(float &theta, float &theta_offset, float length_to_reach, float link_length1, float link_length2){

	// Calculate D value that equals cos(theta)
	float D = (length_to_reach*length_to_reach-link_length1*link_length1-link_length2*link_length2)
	/(2*link_length1*link_length2);
	
	// Check if reach is possible
	if (D>=1||D<=-1) {
	
		return false;
	
	}
	
	// Calculate angle and return true
	theta=atan2(sqrt(1-D*D),D)*R2D;
	
	// Calculate offset
	float k1=link_length1+link_length2*fcos(int(theta));
	float k2=link_length2*fsin(int(theta));
	
	theta_offset=atan2(k2,k1)*R2D;
	
	return true;
	
}

// Function to determine minimum movement time

float minimum_time(float distance, float acceleration, float velocity){

	float blend_time=velocity/acceleration;
	float pure_accelerate_distance=acceleration*blend_time*blend_time;
	
	if (distance<pure_accelerate_distance){
		return sqrt(distance/acceleration)*2;
	} else {
		return distance/velocity+velocity/acceleration;	
	}

}

// Function to calculate position along trajectory

float get_traj(float t, long t_offset, float t_f, float t_b, float v, float a, float q0, float qf){

  float dir=qf-q0; // Correction for direction of acceleration

  if (dir!=0) {
    dir=dir/abs(dir);
  }

  a=a*dir;

  float calc=0;
	float time_since=t-t_offset;
  
  if (time_since>t_f){
    
    calc=qf; 
     
  } else if (time_since>(t_f-t_b)) {
    
		calc=qf-a*t_f*t_f*0.5+a*t_f*time_since-a*0.5*time_since*time_since;
   
	} else if (time_since>t_b) {
    
		calc=(qf+q0-v*t_f)*0.5+v*time_since;
    
	} else {
    
		calc=q0+a*time_since*time_since*0.5;
    
	}

  return calc;
	
}
