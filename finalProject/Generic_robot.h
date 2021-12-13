#include "Arduino.h"
#include <math.h>
#include <avr/pgmspace.h>

// radians to degrees
#define R2D 180.0/PI 


// **************************************************************************************
//              Define Trajectory Generation Variables
// **************************************************************************************

#define SMOOTH_THETA1
#define SMOOTH_THETA2
#define SMOOTH_THETA3
#define SMOOTH_THETA4
#define SMOOTH_THETA5
#define SMOOTH_THETA6
// #define SMOOTH_THETA7

#define EXTRA_TIME_FAST 50; // In milliseconds so 
//that fastest time is not at the edge of what is possible

#define MAX_VELOCITY_1 0.04 // units per millisecond
#define MAX_ACCELERATION_1 0.00004 // units per millisecond squared
#define MAX_VELOCITY_2 0.04 // units per millisecond
#define MAX_ACCELERATION_2 0.00004 // units per millisecond squared
#define MAX_VELOCITY_3 0.04 // units per millisecond
#define MAX_ACCELERATION_3 0.00004 // units per millisecond squared
#define MAX_VELOCITY_4 0.04 // units per millisecond
#define MAX_ACCELERATION_4 0.00004 // units per millisecond squared
#define MAX_VELOCITY_5 0.04 // units per millisecond
#define MAX_ACCELERATION_5 0.00004 // units per millisecond squared
#define MAX_VELOCITY_6 0.04 // units per millisecond
#define MAX_ACCELERATION_6 0.0004 // units per millisecond squared
#define MAX_VELOCITY_7 100.0 // units per millisecond
#define MAX_ACCELERATION_7 100.0 // units per millisecond squared

#define DEFAULT_TIME 1000 //Set default motion time to 1 s

// **************************************************************************************
//                 Define Generic Robot Class
// **************************************************************************************

class GRobot {

public:
    
    /* — Declare an instance of the robot — */

    GRobot();

   /* — Functions to print through serial — */

    String print_position(); // Print position of the robot for debugging
    String print_thetas(); // Print thetas for the sake of debugging

    /* — Functions to move the robot - */
        
    void move_xyzRPY(float x, float y, float z, float R, float P, float Y, float t=DEFAULT_TIME);
    
    void move_xyz(float x, float y, float z,float t=DEFAULT_TIME); //General Positioning
    void move_RPY(float R, float P, float Y,float t=DEFAULT_TIME); //General End Effector Orientation
    
    void move_x(float x,float t=DEFAULT_TIME);
    void move_y(float y,float t=DEFAULT_TIME);
    void move_z(float z,float t=DEFAULT_TIME);
    void move_R(float R,float t=DEFAULT_TIME);
    void move_P(float P,float t=DEFAULT_TIME);
    void move_Y(float Y,float t=DEFAULT_TIME);
    
    /* - Functions to move robot relative to current position (adds value to current position) - */
    
    void move_xyzRPY_relative(float x, float y, float z, float R, float P, float Y,float t=DEFAULT_TIME);
    
    void move_xyz_relative(float x, float y, float z,float t=DEFAULT_TIME);
    void move_RPY_relative(float R, float P, float Y,float t=DEFAULT_TIME); 
    
    void move_x_relative(float x,float t=DEFAULT_TIME);
    void move_y_relative(float y,float t=DEFAULT_TIME);
    void move_z_relative(float z,float t=DEFAULT_TIME);
    void move_R_relative(float R,float t=DEFAULT_TIME);
    void move_P_relative(float P,float t=DEFAULT_TIME);
    void move_Y_relative(float Y,float t=DEFAULT_TIME);
    
    /* — Functions to move the robot as quickly as possible - */
        
    void move_xyzRPY_fast(float x, float y, float z, float R, float P, float Y);
    
    void move_xyz_fast(float x, float y, float z); //General Positioning
    void move_RPY_fast(float R, float P, float Y); //General End Effector Orientation
    
    void move_x_fast(float x);
    void move_y_fast(float y);
    void move_z_fast(float z);
    void move_R_fast(float R);
    void move_P_fast(float P);
    void move_Y_fast(float Y);
    
    /* - Functions to move robot relative to current position ASAP (adds value to current position) - */
    
    void move_xyzRPY_relative_fast(float x, float y, float z, float R, float P, float Y);
    
    void move_xyz_relative_fast(float x, float y, float z);
    void move_RPY_relative_fast(float R, float P, float Y); 
    
    void move_x_relative_fast(float x);
    void move_y_relative_fast(float y);
    void move_z_relative_fast(float z);
    void move_R_relative_fast(float R);
    void move_P_relative_fast(float P);
    void move_Y_relative_fast(float Y);
    
    /* - Functions to extract joint angles - */
    
    float get_theta1(){ return _theta1;}
    float get_theta2(){ return _theta2;}
    float get_theta3(){ return _theta3;}
    float get_theta4(){ return _theta4;}
    float get_theta5(){ return _theta5;}
    float get_theta6(){ return _theta6;}
    float get_theta7(){ return _theta7;}
    
    /* - Functions to do trajectory - */
    
    float get_theta1_traj(float t);
    float get_theta2_traj(float t);
    float get_theta3_traj(float t);
    float get_theta4_traj(float t);
    float get_theta5_traj(float t);
    float get_theta6_traj(float t);
    float get_theta7_traj(float t);

    bool motion_completed(){ return _motion_complete;}
    
    protected:
    
    // Declaring position information
    float _x;
    float _y;
    float _z;
    float _R;
    float _P;
    float _Y;
    
    // Declaring joint information
    float _theta1;
    float _theta2;
    float _theta3;
    float _theta4;
    float _theta5;
    float _theta6;
    float _theta7;
    
    // Declaring variables for smooth trajectories
    
    float _t_final;
    
    float _t_blend_1;
    float _t_blend_2;
    float _t_blend_3;
    float _t_blend_4;
    float _t_blend_5;
    float _t_blend_6;
    float _t_blend_7;
    
    float _v_1;
    float _v_2;
    float _v_3;
    float _v_4;
    float _v_5;
    float _v_6;
    float _v_7;
    
    float _a_1;
    float _a_2;
    float _a_3;
    float _a_4;
    float _a_5;
    float _a_6;
    float _a_7;
    
    float _offset_d_1;
    float _offset_d_2;
    float _offset_d_3;
    float _offset_d_4;
    float _offset_d_5;
    float _offset_d_6;
    float _offset_d_7;
    
    long _offset_t;

    bool _motion_complete;
    
    /* - Inverse Kinematics Function Place holder (Does nothing) - */
    
    virtual bool inverse_kinematics(float x, float y, float z, float R, float P, float Y)=0;
    // Returns true if the motion is allowed
    
    /* - Function to calculate trajectory variables - */
    
    bool get_trajectory_variables(float final_time, float distance, float acceleration, float max_v, float &blend_time, float &velocity);
    // Returns true if the motion can be performed in time
    
};

// **************************************************************************************
//                 Additional Useful Functions
// **************************************************************************************

/* - Function to perform the "elbow" calculation seen in some manipulators involving 2
 that must stretch a specified distance - */

bool elbow_calculation(float &theta, float &theta_offset, float length_to_reach, float link_length1, float link_length2);

// Returns true if possible, false if not

/* - Function to calculate minimum time for a motion to occur given the max acceleration and max velocity - */

float minimum_time(float distance, float acceleration, float velocity);

/* - Function to get the trajectory at a given time - */

float get_traj(float t, long t_offset, float t_f, float t_b, float v, float a, float q0, float qf);

/* - Fast Cos and Sin - */

float fsin(int angle);
float fcos(int angle);
