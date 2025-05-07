#include <vector>
#include<math.h>
#include "dynamics.h"
#include <random>


// Static obstacles, representing debris in the environment from damage during the fight
// x, y, radius
struct StaticObstacle{
    double x; //x coordinate
    double y; // y coordinate
    double radius; //radius of the object
};

// Dynamic obstacles, representing either housebots, an allied robot, or an opposing multi-bot that is not being targeted
struct DynamicObstacle{
    double x0; //starting x coordinate
    double y0; //starting y coordinate
    double x; //current x coordinate
    double y; //current y coordinate
    double radius; //radius of the object being avoided
    double theta; // angle of the direction of travel of the robot
    double velocity; //velocity of the dynamic obstacle
};

// Goal / Opponent, represents the targeted robot
struct Opponent{
    double x0; //starting x coordinate
    double y0; //starting y coordinate
    double x; //current x coordinate
    double y; //current y coordinate
    double radius; //radius of the opponent being targeted
    double theta; // angle of the direction of travel of the robot
    double velocity; //velocity of the opponent
};

// Represents all obstacles in the arena
struct Obstacles{
    std::vector<StaticObstacle>  staticObstacles;
    std::vector<DynamicObstacle>  dynamicObstacles;
};

// Represents the arena
struct Arena{
    double length;
    double width;
};

// Generates a list of static obstacles; obstacles will not be placed inside the safe zone (default: center of arena, radius 1.0)
std::vector<StaticObstacle> generateStaticObstacles(Arena arena, int numObstacles, double maxRadius, double minRadius = 0.001, double safeZoneX = -1, double safeZoneY = -1, double safeZoneRadius = 1.0);

// Generates a list of dynamic obstacles; obstacles will not be placed inside the safe zone (default: center of arena, radius 1.0)
std::vector<DynamicObstacle> generateDynamicObstacles(Arena arena, int numObstacles, double maxRadius, double maxSpeed, double minRadius = 0.01, double safeZoneX = -1, double safeZoneY = -1, double safeZoneRadius = 1.0);

// Generates the Obstacles object using the static and dynamic generators, passing safe zone parameters
Obstacles generateObstacles(Arena arena, int numStaticObstacles, int numDyamicObstacles, double maxRadius = 0.1, double maxSpeed = 9, double safeZoneX = -1, double safeZoneY = -1, double safeZoneRadius = 1.0);

// Updates Obstacles based on time
Obstacles updateObstacles(Obstacles prevObstacles, Arena arena, double timeInterval=0.01);

// Updates opponent location based on time
Opponent updateOpponent(Opponent prevOpponent, Arena arena, double timeInterval=0.01);

// DETECTING ARENA COLLISIONS ========================================================================

// Checks if a point on a square weapon or robot is not within a arena (True if in a invalid location)
bool checkIfPointInvalidSquare(Arena a,Coordinate c);

// Checks if a point on a round weapon or robot is not within a arena (True is in a invalid location)
bool checkIfPointInvalidRound(Arena a, Coordinate c, double radius);

// Gets all 4 points on a square weapon or robot for testing
std::vector<Coordinate> getCorners(Coordinate c,double theta, double width, double height, double x_w, double y_w);

// Checks is a robot is not within the arena (True means it is in a invalid location)
bool checkPositionInvalid(RobotModel robot, Arena a, double x, double y, double theta);

// DETECTING OBSTACLE COLLISIONS ========================================================================

// Checks if a point in the robot hit an obstacle
bool checkIfPointInObstacles(Obstacles ob, Coordinate c, double r);

// Checks if a robot hit an obstacle
bool checkIfRobotHitObstacle(Obstacles ob, double x, double y, double theta, RobotModel robot);

// DETECTING OPPONENT COLLISIONS ======================================================================

// Checks if a point in the robot hit an opponent
bool checkIfPointInOpponent(Opponent o, Coordinate c, double r);

// Checks if the robot's main body hit an opponent (NOT GOAL SHOULD BE AVOIDED)
bool checkIfBodyInOpponent(Opponent o, double x, double y, double theta, RobotModel robot);

// Checks if the robot's weapon impacted the opponent (VICTORY CONDITION)
bool checkIfWeaponInOpponent(Opponent o, double x, double y, double theta, RobotModel robot);


// TYING EVERYTHING TOGETHER TO FIND INVALID POSITIONS ===========================================
bool checkIfRobotValid(RobotModel robot, Opponent opponent, Obstacles obstacles, double x, double y, double theta, Arena a);

// GENERATES STARTING OPPONENT AND ROBOT POSITIONS =============================================================

// Generates a opponent in a valid position in the arena
Opponent generateOpponent(Arena a, Obstacles ob, double maxSpeed=9, double maxRadius=0.1, double minRadius=0.01);

// Generates a static starting position for the robot
Coordinate generateStartingPosition(Arena a, Obstacles ob, Opponent op, RobotModel robot);



