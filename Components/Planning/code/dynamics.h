#include <vector>
#include<math.h>


struct dynamics{

    // Acceleration ================================================
    // Acceleration of individual parts -----------------------------
    // maximum Angular Acceleration on the left drive
    float alpha_left_drive_max;
    // maximum Angular Acceleration on the right drive 
    float alpha_right_drive_max;
    // maximum Angular Acceleration on the weapon
    float alpha_weapon_max;
    float alpha_weapon_min;

    // Acceleartion of the system ------------------------------------
    // Maximum linear acceleration of the system
    float acceleration_max;
    // Maximimum angular acceleration of the system in the counter-clockwise direction
    float alpha_min;
    // Maximum angular acceleartion of the system in the clockwise direction
    float alpha_max;

    // Velocity ========================================================
    // Velocity of individual parts ------------------------------------
    // maximum angular speed on the left drive
    float omega_left_drive_max;
    // maximum angular speed on the right drive
    float omega_right_drive_max;
    // maximum angular speed on the weapon
    float omega_weapon_max;
    float omega_weapon_min;

    // Velocity of the system ----------------------------------------
    // maximum linear velocity of the robot
    float velocity_max;
    // maximum angular velocity 
    float omega_max;
    // minimum angular velocity 
    float omega_min;
    
    
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

