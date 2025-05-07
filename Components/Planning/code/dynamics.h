#include <vector>
#include<math.h>


struct dynamics{

    // Acceleration ================================================
    // Acceleration of individual parts -----------------------------
    // maximum Angular Acceleration on the left drive
    double alpha_left_drive_max=0;
    // maximum Angular Acceleration on the right drive 
    double alpha_right_drive_max=0;
    // maximum Angular Acceleration on the weapon
    double alpha_weapon_max=0;
    double alpha_weapon_min=0;

    // Acceleration of the system ------------------------------------
    // Overall System '''''''''''''''''''''''''''''''''''''''''''''''''
    // Maximum linear acceleration of the system
    double acceleration_max=0;
    double acceleration_min=0;
    // Maximimum angular acceleration of the system in the counter-clockwise direction
    double alpha_min=0;
    // Maximum angular acceleartion of the system in the clockwise direction
    double alpha_max=0;
    // Due to Weapon ```````````````````````````````````````````````````
    double alpha_max_weapon=0;
    double alpha_min_weapon=0;
    // Due to left drive ````````````````````````````````````````````````
    double alpha_max_l_drive=0;
    double alpha_min_l_drive=0;
    // Due to right drive ```````````````````````````````````````````````
    double alpha_max_r_drive=0;
    double alpha_min_r_drive=0;

    // Velocity ========================================================
    // Velocity of individual parts ------------------------------------
    // maximum angular speed on the left drive
    double omega_left_drive_max=0;
    // maximum angular speed on the right drive
    double omega_right_drive_max=0;
    // maximum angular speed on the weapon
    double omega_weapon_max=0;
    double omega_weapon_min=0;

    // Velocity of the system ----------------------------------------
    // Overall System `````````````````````````````````````````````````
    // maximum linear velocity of the robot
    double velocity_max=0;
    double velocity_min=0;
    // maximum angular velocity 
    double omega_max=0;
    // minimum angular velocity 
    double omega_min=0;
    // Due to Weapon `````````````````````````````````````````````````````
    double omega_min_weapon=0;
    double omega_max_weapon=0;
    // Due to left drive `````````````````````````````````````````````````
    double omega_max_l_drive=0;
    double omega_min_l_drive=0;
    // Due to right drive ```````````````````````````````````````````````
    double omega_max_r_drive=0;
    double omega_min_r_drive=0;
    
    
};

struct geometry{
    // Robot Geometry ========================================================================
    // If the robot is not a circle the main body is represented with the following ---------
    double robot_height;
    double robot_width;
    // If the robot is a circle ------------------
    double robot_radius;
    // Weapon Geometry (done from the distance from the center of the robot) =======================================================================
    // x and y cooordinates ----------------------------------------
    double weapon_x;
    double weapon_y;
    // If the weapon is not a horizontal
    double weapon_height;
    double weapon_width;
    // If the weapon is a horizontal
    double weapon_radius;
};


struct RobotModel {
    dynamics robotDynamics;
    geometry robotGeometry;
};

struct Coordinate{
    double x;
    double y;
    double theta;
};

void printDynamics(const dynamics& model);

void printGeometry(const geometry& model);