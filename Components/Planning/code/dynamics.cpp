#include "dynamics.h"
#include <math.h>
#include <string>
#include <fstream>
#include <random>
#include <iostream>
#include <filesystem>
#include <limits.h>
#include <sstream>

void printDynamics(const dynamics& model) {
    std::cout << "\n========= ROBOT DYNAMICS ===========" << std::endl;

    std::cout << "\n----------- SYSTEM-WIDE PROPERTIES -----------" << std::endl;

    std::cout << "\n--- Acceleration (System) ---" << std::endl;
    std::cout << "Max Linear Acceleration       : " << model.acceleration_max << std::endl;
    std::cout << "Min Linear Acceleration       : " << model.acceleration_min << std::endl;
    std::cout << "Max Angular Acceleration (CW) : " << model.alpha_max << std::endl;
    std::cout << "Max Angular Acceleration (CCW): " << model.alpha_min << std::endl;

    std::cout << "\n--- Acceleration (Components) ---" << std::endl;
    std::cout << "Weapon Alpha Max              : " << model.alpha_weapon_max << std::endl;
    std::cout << "Weapon Alpha Min              : " << model.alpha_weapon_min << std::endl;
    std::cout << "Left Drive Alpha Max          : " << model.alpha_max_l_drive << std::endl;
    std::cout << "Left Drive Alpha Min          : " << model.alpha_min_l_drive << std::endl;
    std::cout << "Right Drive Alpha Max         : " << model.alpha_max_r_drive << std::endl;
    std::cout << "Right Drive Alpha Min         : " << model.alpha_min_r_drive << std::endl;
    std::cout << "Alpha Max Weapon (Overall)    : " << model.alpha_max_weapon << std::endl;
    std::cout << "Alpha Min Weapon (Overall)    : " << model.alpha_min_weapon << std::endl;

    std::cout << "\n--- Velocity (System) ---" << std::endl;
    std::cout << "Max Linear Velocity           : " << model.velocity_max << std::endl;
    std::cout << "Min Linear Velocity           : " << model.velocity_min << std::endl;
    std::cout << "Max Angular Velocity          : " << model.omega_max << std::endl;
    std::cout << "Min Angular Velocity          : " << model.omega_min << std::endl;

    std::cout << "\n--- Velocity (Components) ---" << std::endl;
    std::cout << "Weapon Omega Max              : " << model.omega_weapon_max << std::endl;
    std::cout << "Weapon Omega Min              : " << model.omega_weapon_min << std::endl;

    std::cout << "\n==============================================\n" << std::endl;
}

void printGeometry(const geometry& model) {
    std::cout << "\n=========== ROBOT GEOMETRY ===========\n";

    std::cout << "\n--- Robot Body ---" << std::endl;
    std::cout << "Height             : " << model.robot_height << std::endl;
    std::cout << "Width              : " << model.robot_width << std::endl;
    std::cout << "Radius (if circular): " << model.robot_radius << std::endl;

    std::cout << "\n--- Weapon Geometry (relative to robot center) ---" << std::endl;
    std::cout << "Weapon X Position  : " << model.weapon_x << std::endl;
    std::cout << "Weapon Y Position  : " << model.weapon_y << std::endl;
    std::cout << "Weapon Height      : " << model.weapon_height << std::endl;
    std::cout << "Weapon Width       : " << model.weapon_width << std::endl;
    std::cout << "Weapon Radius      : " << model.weapon_radius << std::endl;

    std::cout << "\n=========================================\n" << std::endl;
}

