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



int main(int /* argc */, char ** /* argv */)
{
     // Gets file path for the robot before analysis
     auto exePath = getExecutableDir();
     std::string filePath = exePath.substr(0, exePath.length() - 6);//removes /build
     filePath = filePath + "/robot_data/";

    // Choosing the robot for analysis
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
    std::cout<<"\n"<<filePath<<std::endl;

    // gets raw data from the inputted file
    


   

    
    return 0;
}
