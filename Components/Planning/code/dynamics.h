#include <vector>
#include<math.h>


struct dynamics{

    // Acceleration ================================================
    // Acceleration of individual parts -----------------------------
    // maximum Angular Acceleration on the left drive
    float alpha_left_drive_max=0;
    // maximum Angular Acceleration on the right drive 
    float alpha_right_drive_max=0;
    // maximum Angular Acceleration on the weapon
    float alpha_weapon_max=0;
    float alpha_weapon_min=0;

    // Acceleration of the system ------------------------------------
    // Overall System '''''''''''''''''''''''''''''''''''''''''''''''''
    // Maximum linear acceleration of the system
    float acceleration_max=0;
    float acceleration_min=0;
    // Maximimum angular acceleration of the system in the counter-clockwise direction
    float alpha_min=0;
    // Maximum angular acceleartion of the system in the clockwise direction
    float alpha_max=0;
    // Due to Weapon ```````````````````````````````````````````````````
    float alpha_max_weapon=0;
    float alpha_min_weapon=0;
    // Due to left drive ````````````````````````````````````````````````
    float alpha_max_l_drive=0;
    float alpha_min_l_drive=0;
    // Due to right drive ```````````````````````````````````````````````
    float alpha_max_r_drive=0;
    float alpha_min_r_drive=0;

    // Velocity ========================================================
    // Velocity of individual parts ------------------------------------
    // maximum angular speed on the left drive
    float omega_left_drive_max=0;
    // maximum angular speed on the right drive
    float omega_right_drive_max=0;
    // maximum angular speed on the weapon
    float omega_weapon_max=0;
    float omega_weapon_min=0;

    // Velocity of the system ----------------------------------------
    // Overall System `````````````````````````````````````````````````
    // maximum linear velocity of the robot
    float velocity_max=0;
    float velocity_min=0;
    // maximum angular velocity 
    float omega_max=0;
    // minimum angular velocity 
    float omega_min=0;
    // Due to Weapon `````````````````````````````````````````````````````
    float omega_min_weapon=0;
    float omega_max_weapon=0;
    // Due to left drive `````````````````````````````````````````````````
    float omega_max_l_drive=0;
    float omega_min_l_drive=0;
    // Due to right drive ```````````````````````````````````````````````
    float omega_max_r_drive=0;
    float omega_min_r_drive=0;
    
    
};

struct geometry{
    // Robot Geometry ========================================================================
    // If the robot is not a circle the main body is represented with the following ---------
    float robot_height;
    float robot_width;
    // If the robot is a circle ------------------
    float robot_radius;
    // Weapon Geometry (done from the distance from the center of the robot) =======================================================================
    // x and y cooordinates ----------------------------------------
    float weapon_x;
    float weapon_y;
    // If the weapon is not a horizontal
    float weapon_height;
    float weapon_width;
    // If the weapon is a horizontal
    float weapon_radius;
};


struct RobotModel {
    dynamics robotDynamics;
    geometry robotGeometry;
};

struct Coordinate{
    float x;
    float y;
    float theta;
};

void printDynamics(const dynamics& model);

void printGeometry(const geometry& model);