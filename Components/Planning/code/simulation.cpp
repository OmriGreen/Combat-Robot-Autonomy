// OMPL Imprts
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/SimpleSetup.h>

// MISC Imprts
#include <math.h>
#include <string>
#include <fstream>
#include <random>
#include <iostream>
#include <filesystem>
#if __cplusplus < 201703L
#include <experimental/filesystem>
namespace std {
    namespace filesystem = experimental::filesystem;
}
#endif
#include <limits.h>
#include <sstream>
#include <vector>

// Custom Imports
#include "CollisionChecking.h"
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Global Variables
RobotModel model;
Obstacles obstacles;
Opponent opponent;
Coordinate start;
Arena arena;

// Constant Values
double dt = 0.01; //time between updates
double friction_coeff = 0.75;

// Finds location of the executable to find file location 
#if defined(_WIN32)
    #include <windows.h>
#elif defined(__linux__)
    #include <unistd.h>
#elif defined(__APPLE__)
    #include <mach-o/dyld.h>
#endif


class RobotProjection : public ob::ProjectionEvaluator
{
public:
    RobotProjection(const ob::StateSpace *space)
      : ob::ProjectionEvaluator(space)
      , model_(model)
      , arena_(arena)
    {
    }

    unsigned int getDimension() const override
    {
        // x, y, θ, v, ω, ω_weapon
        return 6;
    }

    void project(const ob::State *state,
                 Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // 1) Cast to CompoundState
        const auto *compound =
            state->as<ob::CompoundStateSpace::StateType>();

        const auto *se2 = compound->as<ob::SE2StateSpace::StateType>(0);
        double x     = se2->getX();
        double y     = se2->getY();
        double theta = se2->getYaw();


        const auto *rv = compound->as<ob::RealVectorStateSpace::StateType>(1);
        double v       = rv->values[0];
        double omega   = rv->values[1];
        double omega_w = rv->values[2];

        projection.resize(getDimension());
        projection(0) = x       / arena_.width;
        projection(1) = y       / arena_.length;
        projection(2) = theta   / M_PI;
        projection(3) = v       / model_.robotDynamics.velocity_max;

        double wmax = std::max(std::abs(model_.robotDynamics.omega_max),
                               std::abs(model_.robotDynamics.omega_min));
        projection(4) = omega   / wmax;

        double wmmax = std::max(std::abs(model_.robotDynamics.omega_weapon_max),
                                std::abs(model_.robotDynamics.omega_weapon_min));
        projection(5) = omega_w / wmmax;
    }

private:
    RobotModel model_;
    Arena      arena_;
};

void robotODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
            ompl::control::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
    const double *controls = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    double u1 = controls[0]; //Left Drive
    double u2 = controls[1]; //Right Drive
    double u3 = controls[2]; //Weapon Drive

    // updateObstacles(obstacles,arena,dt);
    // Current State (q)
    double x        = q[0];
    double y        = q[1];
    double theta    = q[2];
    double v       = q[3];
    double omega    = q[4];
    double omega_w  = q[5];

    // Current velocity (qdot)
    qdot.resize(6);
    qdot[0] = v*cos(theta); //vX
    qdot[1] = v*sin(theta); //vY
    qdot[2] = omega; //omega
    qdot[3] = model.robotDynamics.acceleration_max*((u1+u2)/2);// a

    // Calculating Alpha of the robot
    if(model.robotDynamics.alpha_max_weapon > 0){
        qdot[4] = u1*model.robotDynamics.alpha_max_l_drive + u2*model.robotDynamics.alpha_max_r_drive  +  u3*model.robotDynamics.alpha_max_weapon;//alpha
    }
    else{
        qdot[4] = u1*model.robotDynamics.alpha_max_l_drive + u2*model.robotDynamics.alpha_max_r_drive  +  u3*model.robotDynamics.alpha_min_weapon;//alpha
    }

    // Calculating alpha of the weapon
    if(model.robotDynamics.alpha_weapon_max > 0){
        qdot[5] = u3*model.robotDynamics.alpha_weapon_max;
    }
    else{
        qdot[5] = u3*model.robotDynamics.alpha_weapon_min;
    }

}

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
std::vector<double> split(const std::string& str, char delimiter = ' ') {
    std::vector<double> tokens;
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
    std::vector<double> lineData;
    double Weapon_MOI;
    bool isClockwise;
    bool isBidirectional;
    double Weapon_Power;
    double Weapon_Motor_Max_Omega;
    double Weapon_Reduction;
    double Weapon_Alpha;
    double Weapon_Omega;
    double Drive_Power;
    double Drive_Motor_Max_Omega;
    double Robot_Mass;
    double Robot_MOI;
    // Calculates weapon spinup time
    double spinUpTime;
    double controlChannel;//numerical id for radio channel controlling the robot also functions as an id: 1=Left Drive, 2 = Right Drive, 3 = Weapon Motor
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
    double force_x;
    double force_y;
    double tempTorque;
    double tan_x;
    double tan_y;
    double x_wheel;
    double y_wheel;
    double abs_force;
    double theta_force;
    double motor_alpha_max;
    double drive_spin_up=0.0;

    double numWheels = 0;
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
    // printDynamics(robotDynamics);
    // printGeometry(robotGeometry);
    return RobotModel;
}

// Plans a path for a robot
void plan(ompl::control::SimpleSetupPtr &ss) {
    auto space = ss->getStateSpace();
    auto si = ss->getSpaceInformation();

    // Ensure the state space and control space are properly set up
    if (!space || !si) {
        std::cerr << "Error: State space or space information is not properly initialized." << std::endl;
        return;
    }

    // Debugging: Check if the state space is a CompoundStateSpace
    auto *compoundSpace = dynamic_cast<ob::CompoundStateSpace *>(space.get());
    if (!compoundSpace) {
        std::cerr << "Error: State space is not a CompoundStateSpace." << std::endl;
        return;
    }

    // Debugging: Check subspaces
    auto cartesianSpace = compoundSpace->as<ob::SE2StateSpace>(0);
    auto velocitySpace = compoundSpace->as<ob::RealVectorStateSpace>(1);

    if (!cartesianSpace || !velocitySpace) {
        std::cerr << "Error: Subspaces are not properly configured." << std::endl;
        return;
    }

    // Debugging: Print subspace information
    std::cout << "CompoundStateSpace has " << compoundSpace->getSubspaceCount() << " subspaces." << std::endl;
    std::cout << "Subspace 0: SE2StateSpace" << std::endl;
    std::cout << "Subspace 1: RealVectorStateSpace (3 dimensions)" << std::endl;

    // Initialize start state
    ob::ScopedState<> startState(space);
    auto *compoundState = static_cast<ob::CompoundStateSpace::StateType *>(startState.get());
    if (!compoundState) {
        std::cerr << "Error: Could not cast startState to CompoundState." << std::endl;
        return;
    }
    auto *startSE2 = compoundState->as<ob::SE2StateSpace::StateType>(0);
    auto *startRealVector = compoundState->as<ob::RealVectorStateSpace::StateType>(1);
    if (!startSE2 || !startRealVector) {
        std::cerr << "Error: Could not cast subspaces from CompoundState." << std::endl;
        return;
    }

    startSE2->setXY(start.x, start.y);
    startSE2->setYaw(start.theta);
    startRealVector->values[0] = 0.0;
    startRealVector->values[1] = 0.0;
    startRealVector->values[2] = 0.0;

    std::cout << "Start state initialized: x=" << start.x << ", y=" << start.y
              << ", theta=" << start.theta << ", v=" << startRealVector->values[0]
              << ", omega=" << startRealVector->values[1]
              << ", omega_weapon=" << startRealVector->values[2] << std::endl;

    try {
        ss->addStartState(startState);
        std::cout << "Start state added successfully." << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Exception while adding start state: " << e.what() << std::endl;
        return;
    }

    // --- Repeat for goal state ---
    ob::ScopedState<> goalState(space);
    auto *goalCompound = static_cast<ob::CompoundStateSpace::StateType *>(goalState.get());
    if (!goalCompound) {
        std::cerr << "Error: Could not cast goalState to CompoundState." << std::endl;
        return;
    }
    auto *goalSE2 = goalCompound->as<ob::SE2StateSpace::StateType>(0);
    auto *goalRealVector = goalCompound->as<ob::RealVectorStateSpace::StateType>(1);
    if (!goalSE2 || !goalRealVector) {
        std::cerr << "Error: Could not cast subspaces from goal CompoundState." << std::endl;
        return;
    }
    goalSE2->setXY(opponent.x, opponent.y);
    goalSE2->setYaw(0.0);
    goalRealVector->values[0] = 0.0;
    goalRealVector->values[1] = 0.0;
    goalRealVector->values[2] = 0.0;
    ss->setGoalState(goalState);

    // Debugging: Check if the planner is properly set
    if (!ss->getPlanner()) {
        std::cerr << "Error: Planner is not set." << std::endl;
        return;
    }

    // Attempt to solve the problem
    ob::PlannerStatus solved = ss->solve(60.0); // 60 seconds timeout

    if (solved && solved == ob::PlannerStatus::EXACT_SOLUTION) {
        std::cout << "Found exact solution:" << std::endl;
        ss->getSolutionPath().printAsMatrix(std::cout);
    } else if (solved && solved == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
        std::cout << "Approximate solution found, but exact solution required. Try increasing solve time or check problem setup." << std::endl;
        ss->getSolutionPath().printAsMatrix(std::cout);
    } else {
        std::cout << "NO SOLUTION FOUND.\nIt is possible that for some environments the constraints render the problem unsolvable. Consider adjusting obstacle placement or increasing solve time." << std::endl;
    }
}

void benchmark(ompl::control::SimpleSetupPtr &ss)
{
    // Ensure start and goal are set before benchmarking
    ss->clear();
    // Set up start state
    auto space = ss->getStateSpace();
    ob::ScopedState<> startState(space);
    auto *compoundState = static_cast<ob::CompoundStateSpace::StateType *>(startState.get());
    auto *startSE2 = compoundState->as<ob::SE2StateSpace::StateType>(0);
    auto *startRealVector = compoundState->as<ob::RealVectorStateSpace::StateType>(1);
    startSE2->setXY(start.x, start.y);
    startSE2->setYaw(start.theta);
    startRealVector->values[0] = 0.0;
    startRealVector->values[1] = 0.0;
    startRealVector->values[2] = 0.0;
    ss->addStartState(startState);

    // Set up goal state
    ob::ScopedState<> goalState(space);
    auto *goalCompound = static_cast<ob::CompoundStateSpace::StateType *>(goalState.get());
    auto *goalSE2 = goalCompound->as<ob::SE2StateSpace::StateType>(0);
    auto *goalRealVector = goalCompound->as<ob::RealVectorStateSpace::StateType>(1);
    goalSE2->setXY(opponent.x, opponent.y);
    goalSE2->setYaw(0.0);
    goalRealVector->values[0] = 0.0;
    goalRealVector->values[1] = 0.0;
    goalRealVector->values[2] = 0.0;
    ss->setGoalState(goalState);

    ompl::tools::Benchmark::Request request;
    request.maxTime = 30.0; // seconds per planner
    request.maxMem = 2048.0; // MB
    request.runCount = 20;
    request.displayProgress = true;
    request.saveConsoleOutput = true;

    ompl::tools::Benchmark b(*ss, "CombatRobotBenchmark");

    // RG-RRT
    auto rg_rrt = std::make_shared<ompl::control::RG_RRT>(ss->getSpaceInformation());
    rg_rrt->setName("RG_RRT");
    b.addPlanner(rg_rrt);

    // RRT
    auto rrt = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
    rrt->setName("RRT");
    b.addPlanner(rrt);

    // KPIECE1
    auto kpiece = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
    kpiece->setName("KPIECE1");

    // Set projection for KPIECE1 (required)
    if (!space->hasDefaultProjection()) {
        space->registerDefaultProjection(std::make_shared<RobotProjection>(space.get()));
    }
    b.addPlanner(kpiece);

    b.benchmark(request);
    b.saveResultsToFile("benchmark_log.log");
}

ompl::control::SimpleSetupPtr createRobot()
{
    // Create a compound state space
    auto stateSpace = std::make_shared<ompl::base::CompoundStateSpace>();

    // Cartesian subspace (x, y, theta)
    auto cartesian = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds cartesianBounds(2);
    cartesianBounds.setLow(0, 0); // x bounds
    cartesianBounds.setHigh(0, arena.width);
    cartesianBounds.setLow(1, 0); // y bounds
    cartesianBounds.setHigh(1, arena.length);
    cartesian->setBounds(cartesianBounds);

    // Velocity subspace (v, omega, omega_weapon)
    auto velocity = std::make_shared<ob::RealVectorStateSpace>(3);
    ob::RealVectorBounds velocityBounds(3);
    velocityBounds.setLow(0, model.robotDynamics.velocity_min); // v bounds
    velocityBounds.setHigh(0, model.robotDynamics.velocity_max);
    velocityBounds.setLow(1, model.robotDynamics.omega_min); // omega bounds
    velocityBounds.setHigh(1, model.robotDynamics.omega_max);
    velocityBounds.setLow(2, model.robotDynamics.omega_weapon_min); // omega_weapon bounds
    velocityBounds.setHigh(2, model.robotDynamics.omega_weapon_max);
    velocity->setBounds(velocityBounds);

    // Add subspaces to the compound state space
    stateSpace->addSubspace(cartesian, 1.0); // Cartesian subspace
    stateSpace->addSubspace(velocity, 0.5);  // Velocity subspace

    // Create a control space
    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 3);
    ob::RealVectorBounds controlBounds(3);
    controlBounds.setLow(0, -1); // left drive
    controlBounds.setHigh(0, 1);
    controlBounds.setLow(1, -1); // right drive
    controlBounds.setHigh(1, 1);
    controlBounds.setLow(2, model.robotDynamics.alpha_weapon_min == model.robotDynamics.alpha_weapon_max ? -1 : 0); // weapon drive
    controlBounds.setHigh(2, 1);
    controlSpace->setBounds(controlBounds);

    // Create a SimpleSetup object
    auto ss = std::make_shared<oc::SimpleSetup>(controlSpace);

    // Set the ODE solver and state propagator
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &robotODE);
    ss->getSpaceInformation()->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
    
    // NEW: Set propagation step size to dt and explicitly set control step counts
    ss->getSpaceInformation()->setPropagationStepSize(dt);
    ss->getSpaceInformation()->setMinMaxControlDuration(dt, dt * 10);

    // Set the state validity checker
    ss->setStateValidityChecker([=](const ob::State *state) {
        const auto *compoundState = state->as<ob::CompoundStateSpace::StateType>();
        if (!compoundState) return false;

        const auto *se2State = compoundState->as<ob::SE2StateSpace::StateType>(0);
        double x = se2State->getX();
        double y = se2State->getY();
        double theta = se2State->getYaw();

        bool valid = checkIfRobotValid(model, opponent, obstacles, x, y, theta, arena);
        if (!valid) {
            // Uncomment for debugging:
            // std::cout << "Invalid state: x=" << x << " y=" << y << " theta=" << theta << std::endl;
        }
        return valid;
    });

    return ss;
}


// Main function: Includes robot choice, 
int main(int /* argc */, char ** /* argv */)
{
     // Gets file path for the robot before analysis
     auto exePath = getExecutableDir();
     std::string filePath = exePath.substr(0, exePath.length() - 6);//removes /build
     filePath = filePath + "/robot_data/";

    // Choosing the robot for analysis-add more if desired
    int choice=1;
    do
    {
        std::cout << "What Robot do you want to make a motion plan for?" << std::endl;
        std::cout << " (1) Very Original" << std::endl;
        std::cout << " (2) Custom Robot" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    std::string robot;
    if(choice == 1){
        robot = "Very_Original.txt";
    }
    else{
        std::cout<<"Enter the name of the file containing your robot's data" <<std::endl;
        std::cin >>robot;
    }

    // Gets file path to the file with the robot's information on it
    filePath = filePath + robot;

    // gets the data needed for creating a kinodynamic model from the inputted file
    model = calcProperties(filePath);

    // Ensure robot.txt is in the build folder (relative to the executable location)
    std::string build_dir = getExecutableDir();
    std::filesystem::copy_file(filePath, build_dir + "/robot.txt", std::filesystem::copy_options::overwrite_existing);

    // Choose the arena for the robot to fight in
    int arenaChoice=1;
    // do
    // {
    //     std::cout << "What Arena do you want the robot to fight in" << std::endl;
    //     std::cout << " (1) 3lb Robot Arena NHRL" << std::endl;
    //     std::cout << " (2) 12lb/30lb Robot Arena NHRL" << std::endl;
    //     std::cout << " (3) Custom Arena" << std::endl;

    //     std::cin >> arenaChoice;
    // } while (arenaChoice < 1 || arenaChoice > 3);
    if(arenaChoice == 1){
        arena.length = 2.438;
        arena.width = 2.438;
    }
    else{
        if(arenaChoice == 2){
            arena.length = 10;
            arena.width = 10;
        }
        else{
            std::cout<<"What width do you want the arena to be?" << std::endl;
            std::cin >> arena.width;
            std::cout<<"What length do you want the arena to be?" << std::endl;
            std::cin >> arena.length;
        }
    }
    int obstacleChoice=3;
    // // Generates Obstacles based on basic inputs
    // do
    // {
    //     std::cout << "What Situation do you want the robot to fight in" << std::endl;
    //     std::cout << " (1) 0 static obstacles, 1 dynamic obstacle" << std::endl;
    //     std::cout << " (2) 1 static obstacle, 0 dynamic obstacles" << std::endl;
    //     std::cout << " (3) 1 static obstacle, 1 dynamic obstacle" << std::endl;
    //     std::cout << " (4) Custom Obstacles" << std::endl;

    //     std::cin >> obstacleChoice;
    // } while (obstacleChoice < 1 || obstacleChoice > 4);
    // Maximum obstacle size is determined by a housebot's size i.e. 0.3 m. max speed is 9m/s
    int numDynamic=0;
    int numStatic=0;
    if(obstacleChoice==1){
      numDynamic = 1;
    }
    else{
        if(obstacleChoice==2){
            numStatic = 1;
        }
        else{
            if(obstacleChoice==3){
                numDynamic = 1;
                numStatic = 1;
            }
            else{
                std::cout<<"How many dynamic obstacles do you want?" << std::endl;
                std::cin >> numDynamic;
                std::cout<<"How many static obstacles do you want?" << std::endl;
                std::cin >> numStatic;
            }
        }
    }
    obstacles = generateObstacles(arena,numStatic,numDynamic);

    opponent = generateOpponent(arena,obstacles);

    start = generateStartingPosition(arena,obstacles,opponent,model);
    std::cout << "Chosen start: x=" << start.x << " y=" << start.y << " theta=" << start.theta << std::endl;

    std::cout << "Static obstacles: " << obstacles.staticObstacles.size() << std::endl;
    for (const auto& o : obstacles.staticObstacles) {
        std::cout << "  Static: x=" << o.x << " y=" << o.y << " r=" << o.radius << std::endl;
    }
    std::cout << "Dynamic obstacles: " << obstacles.dynamicObstacles.size() << std::endl;
    for (const auto& o : obstacles.dynamicObstacles) {
        std::cout << "  Dynamic: x=" << o.x << " y=" << o.y << " r=" << o.radius << std::endl;
    }

    ompl::control::SimpleSetupPtr si = createRobot();

    // --- Set up goal as a region with radius opponent.radius ---
    {
        auto space = si->getStateSpace();
        ob::ScopedState<> goalState(space);
        auto *goalCompound = static_cast<ob::CompoundStateSpace::StateType *>(goalState.get());
        auto *goalSE2 = goalCompound->as<ob::SE2StateSpace::StateType>(0);
        auto *goalRealVector = goalCompound->as<ob::RealVectorStateSpace::StateType>(1);

        goalSE2->setXY(opponent.x, opponent.y);
        goalSE2->setYaw(0.0);
        goalRealVector->values[0] = 0.0;
        goalRealVector->values[1] = 0.0;
        goalRealVector->values[2] = 0.0;

        // Define a goal region class
        class GoalRegionWithRadius : public ob::GoalRegion {
        public:
            GoalRegionWithRadius(const ob::SpaceInformationPtr &si, const ob::ScopedState<> &goal, double radius)
                : ob::GoalRegion(si), goal_(goal), radius_(radius) {
                setThreshold(radius_);
            }
            double distanceGoal(const ob::State *state) const override {
                const auto *compound = state->as<ob::CompoundStateSpace::StateType>();
                const auto *se2 = compound->as<ob::SE2StateSpace::StateType>(0);
                double x = se2->getX();
                double y = se2->getY();
                double gx = goal_[0];
                double gy = goal_[1];
                return std::hypot(x - gx, y - gy);
            }
        private:
            ob::ScopedState<> goal_;
            double radius_;
        };

        auto goalRegion = std::make_shared<GoalRegionWithRadius>(si->getSpaceInformation(), goalState, opponent.radius);
        si->setGoal(goalRegion);
    }

    // Export start and goal region for visualization
    {
        std::ofstream startFile(build_dir + "/start.txt");
        startFile << start.x << " " << start.y << " " << start.theta << std::endl;
        startFile.close();

        std::ofstream goalRegionFile(build_dir + "/goal_region.txt");
        goalRegionFile << opponent.x << " " << opponent.y << " " << opponent.radius << std::endl;
        goalRegionFile.close();
    }

    // Ask user if they want to run benchmark or just plan
    int runBenchmark = 0;
    std::cout << "Choose operation:" << std::endl;
    std::cout << " (1) Run benchmark" << std::endl;
    std::cout << " (2) Run single plan" << std::endl;
    std::cin >> runBenchmark;

    if(runBenchmark == 1) {
        benchmark(si);
    } else {
        // Ask user for planner choice
        int plannerChoice = 1;
        std::cout << "Choose planner:" << std::endl;
        std::cout << " (1) RG-RRT" << std::endl;
        std::cout << " (2) RRT" << std::endl;
        std::cout << " (3) KPIECE1" << std::endl;
        std::cin >> plannerChoice;

        // Set the planner based on user choice
        if (plannerChoice == 1) {
            auto planner = std::make_shared<ompl::control::RG_RRT>(si->getSpaceInformation());
            planner->setGoalBias(0.05);
            si->setPlanner(planner);
        } else if (plannerChoice == 2) {
            auto planner = std::make_shared<ompl::control::RRT>(si->getSpaceInformation());
            planner->setGoalBias(0.05);
            si->setPlanner(planner);
        } else if (plannerChoice == 3) {
            auto planner = std::make_shared<ompl::control::KPIECE1>(si->getSpaceInformation());
            planner->setName("KPIECE1");
            // Set projection for KPIECE1 if not already set
            auto space = si->getStateSpace();
            if (!space->hasDefaultProjection()) {
                space->registerDefaultProjection(std::make_shared<RobotProjection>(space.get()));
            }
            si->setPlanner(planner);
        } else {
            std::cout << "Invalid planner choice, defaulting to RG-RRT." << std::endl;
            auto planner = std::make_shared<ompl::control::RG_RRT>(si->getSpaceInformation());
            planner->setGoalBias(0.05);
            si->setPlanner(planner);
        }
        si->setup();
        plan(si);
    }

    // Output files for visualization (write to build folder)
    {
        std::ofstream obsFile(build_dir + "/obstacles.txt");
        for(const auto& o : obstacles.staticObstacles)
        {
            obsFile << o.x << " " << o.y << " " << (o.radius*2) << " " << (o.radius*2) << std::endl;
        }
        obsFile.close();

        std::ofstream dynObsFile(build_dir + "/dynamic_obstacles.txt");
        for(const auto& o : obstacles.dynamicObstacles)
        {
            dynObsFile << o.x0 << " " << o.y0 << " " << o.x << " " << o.y << " " << o.radius 
                       << " " << o.theta << " " << o.velocity << std::endl;
        }
        dynObsFile.close();

        std::ofstream oppFile(build_dir + "/opponent.txt");
        oppFile << opponent.x0 << " " << opponent.y0 << " " << opponent.x << " " 
                << opponent.y << " " << opponent.radius << " " << opponent.theta << " " 
                << opponent.velocity << std::endl;
        oppFile.close();

        oc::PathControl path = si->getSolutionPath();
        if(path.getStateCount() > 0)
        {
            std::ofstream pathFile(build_dir + "/path.txt");
            for (std::size_t i = 0; i < path.getStateCount(); i++)
            {
                const ob::State* state = path.getState(i);
                auto compound = state->as<ob::CompoundStateSpace::StateType>();
                auto se2 = compound->as<ob::SE2StateSpace::StateType>(0);
                double x = se2->getX();
                double y = se2->getY();
                double theta = se2->getYaw();
                pathFile << x << " " << y << " " << theta << std::endl;
            }
            pathFile.close();
        }
    }
    
    return 0;
}
