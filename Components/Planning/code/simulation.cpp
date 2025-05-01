// OMPL Imprts
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>



// MISC Imprts
#include <math.h>
#include <string>
#include <fstream>
#include <random>
#include <iostream>
#include <filesystem>
#include <limits.h>
#include <sstream>
#include <vector>


// Custom Imports
#include "dynamics.h"
#include "CollisionChecking.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Constant Values
const float friction_coeff = 0.75; //Coefficient of friction between robot wheels and the arena, find for your specific arena

// Finds location of the executable to find file location 
#if defined(_WIN32)
    #include <windows.h>
#elif defined(__linux__)
    #include <unistd.h>
#elif defined(__APPLE__)
    #include <mach-o/dyld.h>
#endif

std::string getExecutableDir() {
    char buffer[1024];
#if defined(_WIN32)
    GetModuleFileNameA(NULL, buffer, sizeof(buffer));
#elif defined(__linux__)
    ssize_t count = readlink("/proc/self/exe", buffer, sizeof(buffer));
    buffer[count] = '\0';
#elif defined(__APPLE__)
    uint32_t size = sizeof(buffer);
    _NSGetExecutablePath(buffer, &size);
#endif

    std::string fullPath(buffer);
    size_t found = fullPath.find_last_of("/\\");
    return fullPath.substr(0, found);
}

// Splits a string by a delimiter
std::vector<float> split(const std::string& str, char delimiter = ' ') {
    std::vector<float> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(std::stof(token));  
    }

    return tokens;
}

// Reads the data and returns a dynamics and a geometry object for the robot to use in modeling itself
RobotModel calcProperties(std::string filePath){

    // List of required data ==============================================

    // Robot Geometry data -------------------------------------
    // True if the robot's body can be best represented as a circle in a 2d plane
    bool isCircle;
    // True if the robot's weapon can be best represented as a circle in a 2d plane
    bool isHorizontal;
    // Contains the data on a robot's geometry
    geometry robotGeometry;

    // Robot Dynamics Data ----------------------------------------
    dynamics robotDynamics;

    // Overall data
    RobotModel RobotModel;

    // Collects the necessary data from the file for the individual components
    std::ifstream inputFile(filePath);  
    std::string line;
    std::vector<float> lineData;
    float Weapon_MOI;
    bool isClockwise;
    bool isBidirectional;
    float Weapon_Power;
    float Weapon_Motor_Max_Omega;
    float Weapon_Reduction;
    float Weapon_Alpha;
    float Weapon_Omega;
    float Drive_Power;
    float Drive_Motor_Max_Omega;
    float Robot_Mass;
    float Robot_MOI;
    // Calculates weapon spinup time
    float spinUpTime;
    float controlChannel;//numerical id for radio channel controlling the robot also functions as an id: 1=Left Drive, 2 = Right Drive, 3 = Weapon Motor
    // Sets values
    robotDynamics.alpha_max_l_drive = 0;
    robotDynamics.alpha_min_l_drive = 0;
    robotDynamics.alpha_max_r_drive = 0;
    robotDynamics.alpha_min_r_drive = 0;
    robotDynamics.omega_max_l_drive = 0;
    robotDynamics.omega_min_l_drive = 0;
    robotDynamics.omega_max_r_drive = 0;
    robotDynamics.omega_min_r_drive = 0;
    robotDynamics.acceleration_max = 0;
    robotDynamics.acceleration_min = 0;
    robotDynamics.velocity_max = 0;
    robotDynamics.velocity_min = 0;

    // Stores data for force and torque calculations
    float force_x;
    float force_y;
    float tempTorque;
    float tan_x;
    float tan_y;
    float x_wheel;
    float y_wheel;
    float abs_force;
    float theta_force;
    float motor_alpha_max;
    float drive_spin_up=0.0;

    float numWheels = 0;
    if (inputFile.is_open()) {
        while (std::getline(inputFile, line)) {
            numWheels = numWheels +1;
        }
        numWheels=numWheels-6;
    }
    inputFile.close();

    int lineCount = 0;
    inputFile.open(filePath);

    if (inputFile.is_open()) {
        while (std::getline(inputFile, line)) {
            lineData = split(line);
            // Reads the first line to find isCircle and isGeometric
            if(lineCount == 0){
                // isCircle, isHorizontal
                isCircle = lineData[0]==1.0;
                isHorizontal = lineData[1]==1.0;
            }
            // Reads the second line to get the robot's geometry
            if(lineCount==1){
                // radius
                if(isCircle){
                    robotGeometry.robot_height=-1;
                    robotGeometry.robot_width=-1;
                    robotGeometry.robot_radius=lineData[0];
                }
                else{
                    // width, height
                    robotGeometry.robot_width=lineData[0];
                    robotGeometry.robot_height=lineData[1];
                    robotGeometry.robot_radius=-1;
                }
                // Checking geometry ======= SUCCESS
                // std::cout<<"\n"<< robotGeometry.robot_height << "\n";
                // std::cout<<"\n"<< robotGeometry.robot_width << "\n";
                // std::cout<<"\n"<< robotGeometry.robot_radius << "\n";

            }

            // Calculates the geometry and robot's weapon and gets data for its dynamics
            if(lineCount==2){
                // GEOMETRIC DATA ==========================================================
                if(isHorizontal){
                    // x y radius isClockwise MOI PWMInputVal
                    robotGeometry.weapon_height = -1;
                    robotGeometry.weapon_width = -1;
                    robotGeometry.weapon_radius = lineData[2];
                    robotGeometry.weapon_x = lineData[0];
                    robotGeometry.weapon_y = lineData[1];
                }
                else{
                    // x y width height isClockwise MOI PWMInputVal
                    robotGeometry.weapon_height = lineData[3];
                    robotGeometry.weapon_width = lineData[2];
                    robotGeometry.weapon_radius = -1;
                    robotGeometry.weapon_x = lineData[0];
                    
                    // Circular robots measure y from the center of the robot
                    if(isCircle){
                        robotGeometry.weapon_y = lineData[1];
                    }
                    // Non-circular robots measure y from the center back of the robot to get the data more easily from CAD
                    else{
                        robotGeometry.weapon_y = lineData[1]-robotGeometry.robot_height/2;
                    }
                }

                // Check SUCCESS
                // std::cout<<"\n"<<robotGeometry.weapon_height << "\n";
                // std::cout<<"\n"<<robotGeometry.weapon_width << "\n";
                // std::cout<<"\n"<<robotGeometry.weapon_radius<< "\n";
                // std::cout<<"\n"<<robotGeometry.weapon_x << "\n";
                // std::cout<<"\n"<<robotGeometry.weapon_y << "\n";


                // DYNAMICS DATA ============================================================================
                if(isHorizontal){
                    // x y radius isClockwise MOI PWMInputVal, isBidirectinal
                    // Stored for later use in dynamics calculations
                    Weapon_MOI = lineData[4];
                    isClockwise = lineData[3] == 1.0;
                    isBidirectional = lineData[7] == 1.0;
                }
                else{
                    // x y width height isClockwise MOI PWMInputVal, isBidirectinal
                    Weapon_MOI = lineData[5];
                    isClockwise = lineData[4]==1.0;
                    isBidirectional = lineData[7]==1.0;

                }

            }
            // Calculates Weapon Dynamics
            if(lineCount==3){
                // Weapon Power, Weapon max RPM, Weapon Gear Reduction (2 = 2 motor rotations for each weapon rotation)
                Weapon_Power = lineData[0];
                Weapon_Motor_Max_Omega = lineData[1]*2*M_PI/60;
                Weapon_Reduction = lineData[2];
               
                // Calculates the maximum absolute angular acceleration of the weapon
                Weapon_Alpha = (Weapon_Power * Weapon_Reduction) / (Weapon_Motor_Max_Omega * Weapon_MOI);
                
                // Calculates the maximum angular speed of the weapon
                Weapon_Omega = Weapon_Motor_Max_Omega/Weapon_Reduction;
                // A bidirectinal weapon can spin in either direction
                if(isBidirectional){
                    robotDynamics.alpha_weapon_max=Weapon_Alpha;
                    robotDynamics.alpha_weapon_min=-Weapon_Alpha;
                    robotDynamics.omega_weapon_max=Weapon_Omega;
                    robotDynamics.omega_weapon_min=-Weapon_Omega;

                }
                else{
                    // Clockwise motion is positive
                    if(isClockwise){
                        robotDynamics.alpha_weapon_max=Weapon_Alpha;
                        robotDynamics.alpha_weapon_min=0;
                        robotDynamics.omega_weapon_max=Weapon_Omega;
                        robotDynamics.omega_weapon_min=0;
                    }
                    // Counterclockwise motion is negative
                    else{
                        robotDynamics.alpha_weapon_max=0;
                        robotDynamics.alpha_weapon_min=-Weapon_Alpha;
                        robotDynamics.omega_weapon_max=0;
                        robotDynamics.omega_weapon_min=-Weapon_Omega;
                    }
                }     
                
                
                // Checking === WORKS
                // std::cout<<"\n"<<robotDynamics.alpha_weapon_max<<"\n";
                // std::cout<<"\n"<<robotDynamics.alpha_weapon_min<<"\n";
                // std::cout<<"\n"<<robotDynamics.omega_weapon_max<<"\n";
                // std::cout<<"\n"<<robotDynamics.omega_weapon_min<<"\n";

            }

            // Gets Data to Calculate Drive Dynamics
            if(lineCount==4){
                // Drive Power, Drive max RPM, Drive Power Reduction (2 = 2*Power), Drive Gear Reduction (2 = 2 motor rotations for each weapon rotation)
                // While Drive Power and Drive Gear reductions should be identical in theory, some drive motors include gearboxes that do not have data on the RPM of the motor without the gearbox
                Drive_Power = lineData[0]*lineData[2];
                Drive_Motor_Max_Omega = lineData[1]/lineData[3];
                Drive_Motor_Max_Omega = Drive_Motor_Max_Omega*2*M_PI/60;
            }

            // Gets the mass and MOI of the entire robot and calculates affect on maximum angular acceleration and velocity per wheel
            if(lineCount==5){
                // Robot Mass, Robot MOI
                Robot_Mass = lineData[0];
                Robot_MOI = lineData[1];
    
                if(isHorizontal==0.0){
                // Calculates the affect of the weapon on the robot's angular acceleration and maximum angular velocity 
                // Torque = MOI * alpha -> alpha_robot = (MOI_Weapon*alpha_weapon)/MOI_Robot
                robotDynamics.alpha_max_weapon = (Weapon_MOI*robotDynamics.alpha_weapon_max)/Robot_MOI;
                robotDynamics.alpha_min_weapon = (Weapon_MOI*robotDynamics.alpha_weapon_min)/Robot_MOI;


                // Calculates the maximum and minimum velocity of the robot due to the weapon spin up
                robotDynamics.omega_max_weapon = (Weapon_MOI * robotDynamics.omega_weapon_max) / Robot_MOI;
                robotDynamics.omega_min_weapon = (Weapon_MOI * robotDynamics.omega_weapon_min) / Robot_MOI;
                }
                else{
                    robotDynamics.alpha_max_weapon = 0;
                    robotDynamics.alpha_min_weapon = 0;


                    // Calculates the maximum and minimum velocity of the robot due to the weapon spin up
                    robotDynamics.omega_max_weapon = 0;
                    robotDynamics.omega_min_weapon = 0;
                }

                // // CHECKING
                // std::cout<<"\n"<<robotDynamics.alpha_max_weapon<<"\n";
                // std::cout<<"\n"<<robotDynamics.alpha_min_weapon<<"\n";
                // std::cout<<"\n"<<robotDynamics.omega_max_weapon<<"\n";
                // std::cout<<"\n"<<robotDynamics.omega_min_weapon<<"\n";

            }

            // Gets the data for each wheel and then calculates the affect of them on the 
            if(lineCount > 5)
            {
                x_wheel = lineData[0];
                // If the robot is a circle
                if(isCircle){
                    y_wheel = lineData[1];
                }
                else{
                    y_wheel = lineData[1] - robotGeometry.robot_height/2;
                }

                // Calculates alpha =================================================
                // (Weapon_Power * Weapon_Reduction) / (Weapon_Motor_Max_Omega * Weapon_MOI);
              

                motor_alpha_max = Drive_Power/(Drive_Motor_Max_Omega*lineData[3]);
                // std::cout<<"\n"<<motor_alpha_max<<"\n";
                // Calculates Force
                force_x = lineData[3]*motor_alpha_max*lineData[2];
                force_y = (1.0/numWheels)*Robot_Mass*motor_alpha_max*lineData[2]*friction_coeff;
                // Finds direction of the tangent of the force for calculating torque
                tan_x = -y_wheel/sqrt(y_wheel*y_wheel + x_wheel*x_wheel);
                tan_y = x_wheel/sqrt(y_wheel*y_wheel + x_wheel*x_wheel);

                // Caclulates the force magnitude
                abs_force = abs(force_x*tan_x + force_y*tan_y);

                // Right drive
                if(lineData[4]==2.0){
                robotDynamics.alpha_min_r_drive = robotDynamics.alpha_min_r_drive - abs(abs_force*sqrt(y_wheel*y_wheel + x_wheel*x_wheel))/Robot_MOI;
                robotDynamics.alpha_max_r_drive = robotDynamics.alpha_max_r_drive + abs(abs_force*sqrt(y_wheel*y_wheel + x_wheel*x_wheel))/Robot_MOI;
                }
                // Left Drive
                else{
                    robotDynamics.alpha_min_l_drive = robotDynamics.alpha_min_l_drive - abs(abs_force*sqrt(y_wheel*y_wheel + x_wheel*x_wheel))/Robot_MOI;
                    robotDynamics.alpha_max_l_drive = robotDynamics.alpha_max_l_drive + abs(abs_force*sqrt(y_wheel*y_wheel + x_wheel*x_wheel))/Robot_MOI;
                    }
                // Calculates the maximum acceleration of the robot (will be averaged later)
                robotDynamics.acceleration_max = robotDynamics.acceleration_max + abs(motor_alpha_max*lineData[2]*friction_coeff);
                robotDynamics.acceleration_min = robotDynamics.acceleration_min - abs(motor_alpha_max*lineData[2]*friction_coeff);

                // Calculates the maximum speed of the robot (will be averaged later)
                robotDynamics.velocity_max =  robotDynamics.velocity_max + abs(lineData[2]*Drive_Motor_Max_Omega);
                robotDynamics.velocity_min =  robotDynamics.velocity_min - abs(lineData[2]*Drive_Motor_Max_Omega);

                // Calculates the maximum angular speed of the robot (will be averaged later)
                robotDynamics.omega_max_l_drive = robotDynamics.omega_max_l_drive + (2*Drive_Motor_Max_Omega*lineData[2])/(x_wheel*2);
                robotDynamics.omega_min_l_drive = robotDynamics.omega_min_l_drive - (2*Drive_Motor_Max_Omega*lineData[2])/(x_wheel*2);
                robotDynamics.omega_max_r_drive = robotDynamics.omega_max_r_drive + (2*Drive_Motor_Max_Omega*lineData[2])/(x_wheel*2);
                robotDynamics.omega_min_r_drive = robotDynamics.omega_min_r_drive - (2*Drive_Motor_Max_Omega*lineData[2])/(x_wheel*2);

                // Checking WORKS
                // std::cout<<"\n";
                // std::cout<<robotDynamics.alpha_min_l_drive <<"\n";
                // std::cout<<robotDynamics.alpha_max_l_drive <<"\n";
                // std::cout<<robotDynamics.alpha_max_r_drive <<"\n";
                // void printGeometry(const geometry& model)std::cout<<robotDynamics.alpha_min_r_drive <<"\n";
                // std::cout<<"\n";



            }

            // std::cout<<"\n" << lineCount <<": Success\n";

            lineCount=lineCount+1;
        }
        inputFile.close();

        // Calculates the maximum and minimum rotational velocity due to the drive
        // left motor
        
        robotDynamics.omega_max_l_drive = robotDynamics.omega_max_l_drive/(lineCount-5);
        robotDynamics.omega_min_l_drive = robotDynamics.omega_min_l_drive/(lineCount-5);

        // right motor
        robotDynamics.omega_max_r_drive = robotDynamics.omega_max_r_drive/(lineCount-5);
        robotDynamics.omega_min_r_drive = robotDynamics.omega_min_r_drive/(lineCount-5);




        // std::cout<<"\n" << "Right Motor Omega" <<": Success\n";


        // Finds the acceleration of the robot
        robotDynamics.acceleration_min = robotDynamics.acceleration_min/(lineCount-5);
        robotDynamics.acceleration_max = robotDynamics.acceleration_max/(lineCount-5);

        // std::cout<<"\n" << "Acceleration" <<": Success\n";


        // Finds the  velocity of the robot
        robotDynamics.velocity_min = robotDynamics.velocity_min/(lineCount-5);
        robotDynamics.velocity_max = robotDynamics.velocity_max/(lineCount-5);

        // Finds the maximum angular acceleration of the system
        robotDynamics.alpha_max = robotDynamics.alpha_max_l_drive+robotDynamics.alpha_max_r_drive+robotDynamics.alpha_max_weapon;
        robotDynamics.alpha_min = robotDynamics.alpha_min_l_drive+robotDynamics.alpha_min_r_drive+robotDynamics.alpha_min_weapon;

        // Track width: distance between left and right wheels
        double track_width = isCircle ? robotGeometry.robot_radius * 2.0 : robotGeometry.robot_width;

        // Compute angular velocity due to differential drive
        robotDynamics.omega_max = ((robotDynamics.velocity_max + robotDynamics.velocity_min) / track_width)+robotDynamics.omega_max_weapon;
        robotDynamics.omega_min = ((robotDynamics.velocity_min - robotDynamics.velocity_max) / track_width)+robotDynamics.omega_min_weapon;



        RobotModel.robotDynamics = robotDynamics;
        RobotModel.robotGeometry = robotGeometry;

    } else {
        std::cerr << "Unable to open file" << std::endl;
        
    }
    printDynamics(robotDynamics);
    printGeometry(robotGeometry);
    return RobotModel;
}


// Main function: Includes robot choice, 
int main(int /* argc */, char ** /* argv */)
{
     // Gets file path for the robot before analysis
     auto exePath = getExecutableDir();
     std::string filePath = exePath.substr(0, exePath.length() - 6);//removes /build
     filePath = filePath + "/robot_data/";

    // Choosing the robot for analysis-add more if desired
    int choice;
    do
    {
        std::cout << "What Robot do you want to make a motion plan for?" << std::endl;
        std::cout << " (1) Very Original" << std::endl;
        std::cout << " (2) Double Stuffed" << std::endl;
        std::cout << " (3) Custom Robot" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 3);

    std::string robot;
    if(choice == 1){
        robot = "Very_Original.txt";
    }
    else{
        if(choice == 2){
            robot ="Double_Stuffed.txt";
        }
        else{
            std::cout<<"Enter the name of the file containing your robot's data" <<std::endl;
            std::cin >>robot;
        }
    }

    // Gets file path to the file with the robot's information on it
    filePath = filePath + robot;

    // gets the data needed for creating a kinodynamic model from the inputted file
    RobotModel model = calcProperties(filePath);


   

    
    return 0;
}
