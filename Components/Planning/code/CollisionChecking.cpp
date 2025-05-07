#include "CollisionChecking.h"
#include <iostream>    // Added for std::cout and std::cerr
#include <ostream>     // Added for std::endl

// Generates a list of static obstacles with safe zone protection
std::vector<StaticObstacle> generateStaticObstacles(Arena arena, int numObstacles, double maxRadius, double minRadius, double safeZoneX, double safeZoneY, double safeZoneRadius){
    std::vector<StaticObstacle> obstacles;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distX(maxRadius, arena.width - maxRadius);
    std::uniform_real_distribution<double> distY(maxRadius, arena.length - maxRadius);
    std::uniform_real_distribution<double> distR(minRadius, maxRadius);

    // Use arena center if safeZone not specified
    if(safeZoneX < 0 || safeZoneY < 0){
        safeZoneX = arena.width/2.0;
        safeZoneY = arena.length/2.0;
    }

    int attempts = 0;
    while(obstacles.size() < static_cast<size_t>(numObstacles) && attempts < numObstacles*10){
        attempts++;
        StaticObstacle o;
        o.radius = distR(gen);
        o.x = distX(gen);
        o.y = distY(gen);

        double dx = o.x - safeZoneX;
        double dy = o.y - safeZoneY;
        double dist = std::sqrt(dx*dx + dy*dy);

        if(dist >= (safeZoneRadius + o.radius)){
            obstacles.push_back(o);
        }
    }
    return obstacles;
}

// Generates a list of dynamic obstacles with safe zone protection
std::vector<DynamicObstacle> generateDynamicObstacles(Arena arena, int numObstacles, double maxRadius, double maxSpeed, double minRadius, double safeZoneX, double safeZoneY, double safeZoneRadius){
    std::vector<DynamicObstacle> obstacles;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distX(0, arena.width);
    std::uniform_real_distribution<double> distY(0, arena.length);
    std::uniform_real_distribution<double> distR(minRadius, maxRadius);
    std::uniform_real_distribution<double> theta(0, 2*M_PI);
    std::uniform_real_distribution<double> velocity(-maxSpeed, maxSpeed);

    // Use arena center if safeZone not specified
    if(safeZoneX < 0 || safeZoneY < 0){
        safeZoneX = arena.width/2.0;
        safeZoneY = arena.length/2.0;
    }

    int attempts = 0;
    while(obstacles.size() < static_cast<size_t>(numObstacles) && attempts < numObstacles*10){
        attempts++;
        DynamicObstacle o;
        o.x = distX(gen);
        o.y = distY(gen);
        o.x0 = o.x;
        o.y0 = o.y;
        o.radius = distR(gen);
        o.theta = theta(gen);
        o.velocity = velocity(gen);

        double dx = o.x - safeZoneX;
        double dy = o.y - safeZoneY;
        double dist = std::sqrt(dx*dx + dy*dy);

        if(dist >= (safeZoneRadius + o.radius)){
            obstacles.push_back(o);
        }
    }
    return obstacles;
}

// Generates obstacles by calling the dynamic and static generators with safe zone params
Obstacles generateObstacles(Arena arena, int numStaticObstacles, int numDyamicObstacles, double maxRadius, double maxSpeed, double safeZoneX, double safeZoneY, double safeZoneRadius){
    Obstacles obstacles;
    obstacles.dynamicObstacles = generateDynamicObstacles(arena, numDyamicObstacles, maxRadius, maxSpeed, 0.01, safeZoneX, safeZoneY, safeZoneRadius);
    obstacles.staticObstacles = generateStaticObstacles(arena, numStaticObstacles, maxRadius, 0.001, safeZoneX, safeZoneY, safeZoneRadius);
    return obstacles;
}

// Updates obstacles over time
Obstacles updateObstacles(Obstacles obstacles, Arena arena, double timeInterval){
    for (auto& o : obstacles.dynamicObstacles) {
        double dx = o.velocity * std::cos(o.theta) * timeInterval;
        double dy = o.velocity * std::sin(o.theta) * timeInterval;

        double nextX = o.x + dx;
        double nextY = o.y + dy;

        double scale = 1.0; // scaling factor for motion if the robot hit a wall

        // Check right wall
        if (nextX + o.radius > arena.width) {
            double maxDist = (arena.width - o.radius) - o.x;
            double proposedDist = dx;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check left wall
        if (nextX - o.radius < 0.0) {
            double maxDist = (0.0 + o.radius) - o.x;
            double proposedDist = dx;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check top wall
        if (nextY + o.radius > arena.length) {
            double maxDist = (arena.length - o.radius) - o.y;
            double proposedDist = dy;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check bottom wall
        if (nextY - o.radius < 0.0) {
            double maxDist = (0.0 + o.radius) - o.y;
            double proposedDist = dy;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Scale dx, dy so the robot stops at a wall
        dx *= scale;
        dy *= scale;

        o.x += dx;
        o.y += dy;

        //The robot stops after hitting a wall
        if (scale < 1.0) {
            o.velocity = 0.0;
        }
    }

    return obstacles;
}

// Updates opponent over time
Opponent updateOpponent(Opponent o, Arena arena, double timeInterval){
    double dx = o.velocity * std::cos(o.theta) * timeInterval;
        double dy = o.velocity * std::sin(o.theta) * timeInterval;

        double nextX = o.x + dx;
        double nextY = o.y + dy;

        double scale = 1.0; // scaling factor for motion if the robot hit a wall

        // Check right wall
        if (nextX + o.radius > arena.width) {
            double maxDist = (arena.width - o.radius) - o.x;
            double proposedDist = dx;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check left wall
        if (nextX - o.radius < 0.0) {
            double maxDist = (0.0 + o.radius) - o.x;
            double proposedDist = dx;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check top wall
        if (nextY + o.radius > arena.length) {
            double maxDist = (arena.length - o.radius) - o.y;
            double proposedDist = dy;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check bottom wall
        if (nextY - o.radius < 0.0) {
            double maxDist = (0.0 + o.radius) - o.y;
            double proposedDist = dy;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Scale dx, dy so the robot stops at a wall
        dx *= scale;
        dy *= scale;

        o.x += dx;
        o.y += dy;

        //The robot stops after hitting a wall
        if (scale < 1.0) {
            o.velocity = 0.0;
        }

        return o;
}

// DETECTS COLLISIONS WITH THE ARENA ===============================================

// Checks if a point on a square robot is not within a arena (True if in a invalid location)
bool checkIfPointInvalidSquare(Arena a, Coordinate c){
    // If the point is outside of the arena return true
    if(a.length <= c.y){
        return true;
    }
    if(0 >= c.y){
        return true;
    }
    if(a.width <= c.x){
        return true;
    }
    if(0 >= c.x){
        return true;
    }
    // If the point is within the arena return false
    return false;


}

// Checks if a point on a round weapon or robot is not within a arena (True is in a invalid location)
bool checkIfPointInvalidRound(Arena a, Coordinate c, double radius){
     // If the point is outside of the arena return true
     if(a.length <= c.y + radius){
        return true;
    }
    if(0 >= c.y - radius){
        return true;
    }
    if(a.width <= c.x + radius){ 
        return true;
    }
    if(0 >= c.x - radius){
        return true;
    }
    // If the point is within the arena return false
    return false;
}

// Gets all the corners of of a square 
std::vector<Coordinate> getCorners(Coordinate c, double width, double height, double x_w, double y_w) {
    std::vector<Coordinate> corners;

    double w = width / 2.0;
    double h = height / 2.0;

    // Define corners in local space (relative to center)
    std::vector<Coordinate> local = {
        {-w + x_w, -h + y_w}, 
        { w + x_w, -h + y_w}, 
        { w + x_w,  h + y_w}, 
        {-w + x_w,  h + y_w}  
    };

    double cos_theta = std::cos(c.theta);
    double sin_theta = std::sin(c.theta);

    // Rotate and translate each corner to world coordinates
    for (const auto& p : local) {
        Coordinate rotated;
        rotated.x = p.x * cos_theta - p.y * sin_theta + c.x;
        rotated.y = p.x * sin_theta + p.y * cos_theta + c.y;
        corners.push_back(rotated);
    }

    return corners;
}

// Returns true if the position is invalid, true otherwise
bool checkPositionInvalid(RobotModel robot, Arena a, double x, double y, double theta){
    geometry r = robot.robotGeometry;
    Coordinate c;
    c.x = x;
    c.y = y;
    c.theta = theta;

     // If a robot is not a circle the y distance is measured from the back middle
     if(r.robot_radius == -1){
        c.y = c.y + r.robot_height/2;
    } 

    // Checking the main body of the robot ========================================
    // If the robot is not a circle
    if(r.robot_radius == -1){
        std::vector<Coordinate> vertices = getCorners(c, r.robot_width, r.robot_height, 0, 0);
        // Checks if every vertice is inside of the arena and if not returns false
        for(int i = 0; i <4; i++){
            if(checkIfPointInvalidSquare(a,vertices[i])){
                return true;
            }
        }
    }
    // If the main body of the robot is a circle
    else{
        if(checkIfPointInvalidRound(a,c,r.robot_radius)){
            return true;
        }
    }

    // Checking the weapon of the robot ==============================================
    // If the robot's weapon is not a horizontal
    if(r.weapon_radius == -1){

        std::vector<Coordinate> vertices = getCorners(c, r.robot_width, r.robot_height, r.weapon_x, r.weapon_y);
        // Checks if every vertice is inside of the arena and if not returns false
        for(int i = 0; i <4; i++){
            if(checkIfPointInvalidSquare(a,vertices[i])){
                return true;
            }
        }
    }
    else{
        // If the robot's weapon is a horizontal
        c.x = c.x + r.weapon_x * cos(theta) - r.weapon_y*sin(theta);
        c.y = c.y + r.weapon_x * sin(theta) + r.weapon_y*cos(theta);
        if(checkIfPointInvalidRound(a,c,r.weapon_radius)){
            return true;
        }

    }

    // Position is valid
    return false;
}

// CHECK COLLISIONS WITH OBSTACLES ==================================================
// Checks if a point is in a obstacle, returns true in an invalid location
bool checkIfPointInObstacles(Obstacles ob, Coordinate c, double r){
    double dist;
    // Checks Dynamic obstacles
    for (auto& o : ob.dynamicObstacles) {
        // Distance betweem edges of the obstacle and the robot or weapoin
        dist = sqrt((o.x-c.x)*(o.x-c.x)-(o.y-c.y)*(o.y-c.y))-o.radius-r;

        // If the distance is less than 0 it means it is intersecting
        if(dist < 0){
            return true;
        }
    }
    // Checks Static Obstacles
    for (auto& o : ob.staticObstacles) {
        // Distance betweem edges of the obstacle and the robot or weapon
        dist = sqrt((o.x-c.x)*(o.x-c.x)-(o.y-c.y)*(o.y-c.y))-o.radius-r;

        // If the distance is less than 0 it means it is intersecting
        if(dist < 0){
            return true;
        }
    }

    return false;
}

// Checks if the robot or weapon hit an obstacle
bool checkIfRobotHitObstacle(Obstacles ob, double x, double y, double theta, RobotModel robot){
    geometry r = robot.robotGeometry;
    Coordinate c;
    c.x = x;
    c.y = y;
    c.theta = theta;

     // If a robot is not a circle the y distance is measured from the back middle
     if(r.robot_radius == -1){
        c.y = c.y + r.robot_height/2;
    } 


     // Checking the main body of the robot ========================================
    // If the robot is not a circle
    if(r.robot_radius == -1){
        std::vector<Coordinate> vertices = getCorners(c, r.robot_width, r.robot_height, 0, 0);
        // Checks if every vertice is inside of the arena and if not returns false
        for(int i = 0; i <4; i++){
            if(checkIfPointInObstacles(ob,vertices[i],0.0)){
                return true;
            }
        }
    }
    // If the main body of the robot is a circle
    else{
        if(checkIfPointInObstacles(ob,c,r.robot_radius)){
            return true;
        }
    }

     // Checking the weapon of the robot ==============================================
    // If the robot's weapon is not a horizontal
    if(r.weapon_radius == -1){

        std::vector<Coordinate> vertices = getCorners(c, r.robot_width, r.robot_height, r.weapon_x, r.weapon_y);
        // Checks if any vertice hit an obstacle and if so returns true
        for(int i = 0; i <4; i++){
            if(checkIfPointInObstacles(ob,vertices[i],0.0)){
                return true;
            }
        }
    }
    else{
        // If the robot's weapon is a horizontal
        c.x = c.x + r.weapon_x * cos(theta) - r.weapon_y*sin(theta);
        c.y = c.y + r.weapon_x * sin(theta) + r.weapon_y*cos(theta);
        if(checkIfPointInObstacles(ob,c,r.weapon_radius)){
            return true;
        }

    }

    // Position is valid
    return false;

}

// CHECK COLLISIONS WITH OPPONENT ===================================================
// Checks if a point in the robot hit an opponent
bool checkIfPointInOpponent(Opponent o, Coordinate c, double r){
    double dist;
    // Distance betweem edges of the obstacle and the robot or weapoin
    dist = sqrt((o.x-c.x)*(o.x-c.x)-(o.y-c.y)*(o.y-c.y))-o.radius-r;

    // If the distance is less than 0 it means it is intersecting
    if(dist < 0){
        return true;
    }

    return false;    
}

// Checks if the robot's main body hit an opponent (NOT GOAL SHOULD BE AVOIDED)
bool checkIfBodyInOpponent(Opponent o, double x, double y, double theta, RobotModel robot){
    geometry r = robot.robotGeometry;
    Coordinate c;
    c.x = x;
    c.y = y;
    c.theta = theta;

     // If a robot is not a circle the y distance is measured from the back middle
     if(r.robot_radius == -1){
        c.y = c.y + r.robot_height/2;
    } 


     // Checking the main body of the robot ========================================
    // If the robot is not a circle
    if(r.robot_radius == -1){
        std::vector<Coordinate> vertices = getCorners(c, r.robot_width, r.robot_height, 0, 0);
        // Checks if every vertice is inside of the arena and if not returns false
        for(int i = 0; i <4; i++){
            if(checkIfPointInOpponent(o,vertices[i],0.0)){
                return true;
            }
        }
    }
    // If the main body of the robot is a circle
    else{
        if(checkIfPointInOpponent(o,c,r.robot_radius)){
            return true;
        }
    }

    // The main body of the robot DID NOT hit the opponent
    return false;
}

// Checks if the robot's weapon impacted the opponent (VICTORY CONDITION)
bool checkIfWeaponInOpponent(Opponent o, double x, double y, double theta, RobotModel robot){
    geometry r = robot.robotGeometry;
    Coordinate c;
    c.x = x;
    c.y = y;
    c.theta = theta;

     // If a robot is not a circle the y distance is measured from the back middle
     if(r.robot_radius == -1){
        c.y = c.y + r.robot_height/2;
    } 

     // Checking the weapon of the robot ==============================================
    // If the robot's weapon is not a horizontal
    if(r.weapon_radius == -1){

        std::vector<Coordinate> vertices = getCorners(c, r.robot_width, r.robot_height, r.weapon_x, r.weapon_y);
        // Checks if any vertice hit an obstacle and if so returns true
        for(int i = 0; i <4; i++){
            if(checkIfPointInOpponent(o,vertices[i],0.0)){
                return true;
            }
        }
    }
    else{
        // If the robot's weapon is a horizontal
        c.x = c.x + r.weapon_x * cos(theta) - r.weapon_y*sin(theta);
        c.y = c.y + r.weapon_x * sin(theta) + r.weapon_y*cos(theta);
        if(checkIfPointInOpponent(o,c,r.weapon_radius)){
            return true;
        }

    }

    // Robot didn't hit opponent with its weapon
    return false;
}

// FINAL COLLISION DETECTION CODE ================================
bool checkIfRobotValid(RobotModel robot, Opponent opponent, Obstacles obstacles, double x, double y, double theta, Arena arena){
    // Checks if the robot is not within the arena and if so the robot is in an invalid position
    if(checkPositionInvalid(robot, arena, x, y, theta)){
        return false;
    }
    // Checks if the robot hit an obstacle and if so the robot is in an invalid position
    if(checkIfRobotHitObstacle(obstacles, x, y, theta, robot)){
        return false;
    }
    // If a robot's weapon hit an opponent then the position is ALWAYS valid (Opponent WILL be knocked back)
    if(checkIfWeaponInOpponent(opponent, x, y, theta, robot)){
        return true;
    }
    else{
        // If the robot's weapon DID NOT hit opponent but its body hit the opponent then the position is NOT valid
        if(checkIfBodyInOpponent(opponent, x, y, theta, robot)){
            return false;
        }
        else{
            return true;
        }
    }
}

// GENERATES STARTING OPPONENT AND ROBOT POSITIONS =============================================================

// Generates a opponent in a valid position in the arena
Opponent generateOpponent(Arena a, Obstacles ob, double maxSpeed, double maxRadius, double minRadius){
    Coordinate c;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distX(0, a.width);
    std::uniform_real_distribution<double> distY(0, a.length);
    std::uniform_real_distribution<double> distR(minRadius, maxRadius);
    std::uniform_real_distribution<double> genTheta(0, 2*M_PI);
    std::uniform_real_distribution<double> genVelocity(-maxSpeed, maxSpeed);

    // randomly generates a position for an opponent and if valid returns it
    bool invalid = true;
    double x;
    double y;
    double theta;
    double radius;
    double velocity;

    // To avoid overlap with start, optionally pass a forbidden region (center, radius)
    // For now, just avoid the arena center (used by start by default)
    double forbidden_x = a.width / 2.0;
    double forbidden_y = a.length / 2.0;
    double forbidden_radius = 0.5; // minimum separation

    while(invalid){
        // generates a randomized position and trajectory for an opponent
        x = distX(gen);
        y = distY(gen);
        theta = genTheta(gen);
        radius = distR(gen);
        velocity = genVelocity(gen);

        // Checks if the opponent's position is valid
        c.theta = theta;
        c.x = x;
        c.y = y;
        // Checks if the point is within an obstacle cannot be true initially
        if(checkIfPointInObstacles(ob,c,radius) == false){
            // Checks if the point is outside of the arena cannot be true initially
            if(checkIfPointInvalidRound(a,c,radius) == false){
                // Ensure not too close to forbidden region (start)
                double dx = x - forbidden_x;
                double dy = y - forbidden_y;
                double dist = std::sqrt(dx*dx + dy*dy);
                if(dist > forbidden_radius + radius) {
                    invalid = false;
                }
            }
        }
    }

    // Generates opponent
    Opponent o;
    o.radius = radius;
    o.theta = theta;
    o.velocity = velocity;
    o.x0 = x;
    o.x = x;
    o.y = y;
    o.y0 = y;

    return o;
}

// Generates a static starting position for the robot
Coordinate generateStartingPosition(Arena a, Obstacles ob, Opponent op, RobotModel robot){
    Coordinate c;
    // Try using the arena center with zero orientation as initial state
    c.x = a.width / 2.0;
    c.y = a.length / 2.0;
    c.theta = 0.0;
    // Ensure start is not at the opponent's position
    if ((std::abs(c.x - op.x) > 1e-3 || std::abs(c.y - op.y) > 1e-3) &&
        checkIfRobotValid(robot, op, ob, c.x, c.y, c.theta, a)) {
        std::cout << "Using center of arena as start state." << std::endl;
        return c;
    }
    // Fallback to random sampling if the center is invalid or coincides with opponent
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distX(0, a.width);
    std::uniform_real_distribution<double> distY(0, a.length);
    std::uniform_real_distribution<double> genTheta(0, 2*M_PI);
    bool invalid = true;
    int attempts = 0;
    while(invalid){
        c.x = distX(gen);
        c.y = distY(gen);
        c.theta = genTheta(gen);
        // Ensure start is not at the opponent's position
        if ((std::abs(c.x - op.x) > 1e-3 || std::abs(c.y - op.y) > 1e-3) &&
            checkIfRobotValid(robot, op, ob, c.x, c.y, c.theta, a)) {
            invalid = false;
        }
        attempts++;
        if (attempts > 10000) {
            std::cerr << "Fallback sampling exceeded attempts. Approximate solution found but exact solution required. Returning last candidate. Consider increasing solve time or adjusting problem setup." << std::endl;
            break;
        }
    }
    return c;
}
