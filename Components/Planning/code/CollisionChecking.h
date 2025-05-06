#include <vector>
#include<math.h>
#include "dynamics.h"
#include <random>


// Static obstacles, representing debris in the environment from damage during the fight
// x, y, radius
struct StaticObstacle{
    float x; //x coordinate
    float y; // y coordinate
    float radius; //radius of the object
};

// Dynamic obstacles, representing either housebots, an allied robot, or an opposing multi-bot that is not being targeted
struct DynamicObstacle{
    float x0; //starting x coordinate
    float y0; //starting y coordinate
    float x; //current x coordinate
    float y; //current y coordinate
    float radius; //radius of the object being avoided
    float theta; // angle of the direction of travel of the robot
    float velocity; //velocity of the dynamic obstacle
};

// Goal / Opponent, represents the targeted robot
struct Opponent{
    float x0; //starting x coordinate
    float y0; //starting y coordinate
    float x; //current x coordinate
    float y; //current y coordinate
    float radius; //radius of the opponent being targeted
    float theta; // angle of the direction of travel of the robot
    float velocity; //velocity of the opponent
};

// Represents all obstacles in the arena
struct Obstacles{
    std::vector<StaticObstacle>  staticObstacles;
    std::vector<DynamicObstacle>  dynamicObstacles;
};

// Represents the arena
struct Arena{
    float length;
    float width;
};

// Generates a list of static obstacles
std::vector<StaticObstacle> generateStaticObstacles(Arena arena, int numObstacles ,float maxRadius, float minRadius = 0.001);

// Generates a list of dynamic obstacles
std::vector<DynamicObstacle> generateDynamicObstacles(Arena arena, int numObstacles, float maxRadius, float maxSpeed, float minRadius = 0.01);

// Generates the Obstacles object using 2 lists of static and dynamic obstacles respectively
Obstacles generateObstacles(Arena arena, int numStaticObstacles, int numDyamicObstacles, float maxStaticRadius, float maxDynamicRadius, float maxSpeed);

// Updates Obstacles based on time
Obstacles updateObstacles(Obstacles prevObstacles, Arena arena, float timeInterval=0.01);

// Updates opponent location based on time
Opponent updateOpponent(Opponent prevOpponent, Arena arena, float timeInterval=0.01);

// DETECTING ARENA COLLISIONS ========================================================================

// Checks if a point on a square weapon or robot is not within a arena (True if in a invalid location)
bool checkIfPointInvalidSquare(Arena a,Coordinate c);

// Checks if a point on a round weapon or robot is not within a arena (True is in a invalid location)
bool checkIfPointInvalidRound(Arena a, Coordinate c, float radius);

// Gets all 4 points on a square weapon or robot for testing
std::vector<Coordinate> getCorners(Coordinate c,float theta, float width, float height, float x_w, float y_w);

// Checks is a robot is not within the arena (True means it is in a invalid location)
bool checkPositionInvalid(RobotModel robot, Arena a, float x, float y, float theta);

// DETECTING OBSTACLE COLLISIONS ========================================================================

// Checks if a point in the robot hit an obstacle
bool checkIfPointInObstacles(Obstacles ob, Coordinate c, float r);

// Checks if a robot hit an obstacle
bool checkIfRobotHitObstacle(Obstacles ob, float x, float y, float theta, RobotModel robot);

// DETECTING OPPONENT COLLISIONS ======================================================================

// Checks if a point in the robot hit an opponent
bool checkIfPointInOpponent(Opponent o, Coordinate c, float r);

// Checks if the robot's main body hit an opponent (NOT GOAL SHOULD BE AVOIDED)
bool checkIfBodyInOpponent(Opponent o, float x, float y, float theta, RobotModel robot);

// Checks if the robot's weapon impacted the opponent (VICTORY CONDITION)
bool checkIfWeaponInOpponent(Opponent o, float x, float y, float theta, RobotModel robot);


// TYING EVERYTHING TOGETHER TO FIND INVALID POSITIONS ===========================================
bool checkIfRobotValid(RobotModel robot, Opponent opponent, Obstacles obstacles, float x, float y, float theta);

