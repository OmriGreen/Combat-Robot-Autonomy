// OMPL Imprts
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
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


// Custom Imports
#include "CollisionChecking.h"
#include "dynamics.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

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
int calcProperties(std::string filePath){

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


    // Collects the necessary data from the file for the individual components
    std::ifstream inputFile(filePath);  
    std::string line;
    std::vector<float> lineData;
    float MOI;
    bool isClockwise;


    int lineCount = 0;
    if (inputFile.is_open()) {
        while (std::getline(inputFile, line)) {
            valCount = 0; //Counts values in a line
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
                    robotGeometry.robot_height=NULL;
                    robotGeometry.robot_width=NULL;
                    robotGeometry.robot_radius=lineData[0];
                }
                else{
                    // width, height
                    robotGeometry.width=lineData[0];
                    robotGeometry.height=lineData[1];
                    robotGeometry.radius=NULL;
                }
            }

            // Calculates the affect of the robot's weapon on the robot as well as finding the applicable dynamics
            if(lineCount==2){
                // GEOMETRIC DATA ==========================================================
                if(isHorizontal){
                    // x y radius isClockwise MOI PWMInputVal
                    robotGeometry.weapon_height = NULL;
                    robotGeometry.weapon_width = NULL;
                    robotGeometry.weapon_radius = lineData[2];
                    robotGeometry.weapon_x = lineData[0];
                    robotGeometry.weapon_y = lineData[1];
                }
                else{
                    // x y width height isClockwise MOI PWMInputVal
                    robotGeometry.weapon_height = lineData[3];
                    robotGeometry.weapon_width = lineData[2];
                    robotGeometry.weapon_radius = NULL;
                    robotGeometry.weapon_x = lineData[0];
                    
                    // Circular robots measure y from the center of the robot
                    if(isCircle){
                        robotGeometry.weapon_y = lineData[1];
                    }
                    // Non-circular robots measure y from the center back of the robot to get the data more easily from CAD
                    else{
                        robotDynamics.weapon_y = lineData[1]-robotGeometry.robot_height/2;
                    }
                }

                // DYNAMICS DATA ============================================================================
                if(isHorizontal){
                    // x y radius isClockwise MOI PWMInputVal
                    MOI = lineData[4];
                    isClockwise = lineData[3];

                    


                }
                else{
                    // x y width height isClockwise MOI PWMInputVal
                   
                }
            }
            
            lineCount=lineCount+1;
        }
        inputFile.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }

    return 0;
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
    auto robotDynamics = calcProperties(filePath);


   

    
    return 0;
}
