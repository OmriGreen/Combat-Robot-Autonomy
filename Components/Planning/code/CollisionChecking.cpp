#include "CollisionChecking.h"

// Generates a list of static obstacles
std::vector<StaticObstacle> generateStaticObstacles(Arena arena, int numObstacles ,float maxRadius, float minRadius){
    std::vector<StaticObstacle> obstacles;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distX(maxRadius, arena.width - maxRadius);
    std::uniform_real_distribution<float> distY(maxRadius, arena.length - maxRadius);
    std::uniform_real_distribution<float> distR(minRadius, maxRadius);

    for (int i = 0; i < numObstacles; i++) {
        StaticObstacle o;
        o.x = distX(gen);
        o.y = distY(gen);
        o.radius = distR(gen);
        obstacles.push_back(o);
    }
    return obstacles;
}

// Generates a list of dynamic obstacles
std::vector<DynamicObstacle> generateDynamicObstacles(Arena arena, int numObstacles, float maxRadius, float maxSpeed, float minRadius){
    std::vector<DynamicObstacle> obstacles;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distX(0, arena.width);
    std::uniform_real_distribution<float> distY(0, arena.length);
    std::uniform_real_distribution<float> distR(minRadius, maxRadius);
    std::uniform_real_distribution<float> theta(0, 2*M_PI);
    std::uniform_real_distribution<float> velocity(-maxSpeed, maxSpeed);

    for (int i = 0; i < numObstacles; i++) {
        DynamicObstacle o;
        o.x = distX(gen);
        o.y = distY(gen);
        o.x0 = o.x;
        o.y0 = o.y;
        o.theta = theta(gen);
        o.radius = distR(gen);
        o.velocity = velocity(gen);
        obstacles.push_back(o);
    }
    return obstacles;

}

// Utilizes the generateDynamicObstacles and generateStaticObstacles objects to generate obstacles for the robot based on variables
Obstacles generateObstacles(Arena arena, int numStaticObstacles, int numDyamicObstacles, float maxRadius, float maxSpeed){
    Obstacles obstacles;
    obstacles.dynamicObstacles = generateDynamicObstacles(arena, numDyamicObstacles, maxRadius, maxSpeed);
    obstacles.staticObstacles = generateStaticObstacles(arena,numStaticObstacles, maxRadius);
    return obstacles;
}

// Updates obstacles over time
Obstacles updateObstacles(Obstacles obstacles, Arena arena, float timeInterval){
    for (auto& o : obstacles.dynamicObstacles) {
        float dx = o.velocity * std::cos(o.theta) * timeInterval;
        float dy = o.velocity * std::sin(o.theta) * timeInterval;

        float nextX = o.x + dx;
        float nextY = o.y + dy;

        float scale = 1.0; // scaling factor for motion if the robot hit a wall

        // Check right wall
        if (nextX + o.radius > arena.width) {
            float maxDist = (arena.width - o.radius) - o.x;
            float proposedDist = dx;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check left wall
        if (nextX - o.radius < 0.0) {
            float maxDist = (0.0 + o.radius) - o.x;
            float proposedDist = dx;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check top wall
        if (nextY + o.radius > arena.length) {
            float maxDist = (arena.length - o.radius) - o.y;
            float proposedDist = dy;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check bottom wall
        if (nextY - o.radius < 0.0) {
            float maxDist = (0.0 + o.radius) - o.y;
            float proposedDist = dy;
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
Opponent updateOpponent(Opponent o, Arena arena, float timeInterval){
    float dx = o.velocity * std::cos(o.theta) * timeInterval;
        float dy = o.velocity * std::sin(o.theta) * timeInterval;

        float nextX = o.x + dx;
        float nextY = o.y + dy;

        float scale = 1.0; // scaling factor for motion if the robot hit a wall

        // Check right wall
        if (nextX + o.radius > arena.width) {
            float maxDist = (arena.width - o.radius) - o.x;
            float proposedDist = dx;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check left wall
        if (nextX - o.radius < 0.0) {
            float maxDist = (0.0 + o.radius) - o.x;
            float proposedDist = dx;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check top wall
        if (nextY + o.radius > arena.length) {
            float maxDist = (arena.length - o.radius) - o.y;
            float proposedDist = dy;
            if (proposedDist != 0.0) scale = std::min(scale, maxDist / proposedDist);
        }

        // Check bottom wall
        if (nextY - o.radius < 0.0) {
            float maxDist = (0.0 + o.radius) - o.y;
            float proposedDist = dy;
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
bool checkIfPointInvalidRound(Arena a, Coordinate c, float radius){
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
std::vector<Coordinate> getCorners(Coordinate c, float width, float height, float x_w, float y_w) {
    std::vector<Coordinate> corners;

    float w = width / 2.0;
    float h = height / 2.0;

    // Define corners in local space (relative to center)
    std::vector<Coordinate> local = {
        {-w + x_w, -h + y_w}, 
        { w + x_w, -h + y_w}, 
        { w + x_w,  h + y_w}, 
        {-w + x_w,  h + y_w}  
    };

    float cos_theta = std::cos(c.theta);
    float sin_theta = std::sin(c.theta);

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
bool checkPositionInvalid(RobotModel robot, Arena a, float x, float y, float theta){
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
bool checkIfPointInObstacles(Obstacles ob, Coordinate c, float r){
    float dist;
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
bool checkIfRobotHitObstacle(Obstacles ob, float x, float y, float theta, RobotModel robot){
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
            if(checkIfPointInObstacles(ob,vertices[i],0)){
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
            if(checkIfPointInObstacles(ob,vertices[i],0)){
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
bool checkIfPointInOpponent(Opponent o, Coordinate c, float r){
    float dist;
    // Distance betweem edges of the obstacle and the robot or weapoin
    dist = sqrt((o.x-c.x)*(o.x-c.x)-(o.y-c.y)*(o.y-c.y))-o.radius-r;

    // If the distance is less than 0 it means it is intersecting
    if(dist < 0){
        return true;
    }

    return false;    
}

// Checks if the robot's main body hit an opponent (NOT GOAL SHOULD BE AVOIDED)
bool checkIfBodyInOpponent(Opponent o, float x, float y, float theta, RobotModel robot){
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
            if(checkIfPointInOpponent(o,vertices[i],0)){
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
bool checkIfWeaponInOpponent(Opponent o, float x, float y, float theta, RobotModel robot){
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
            if(checkIfPointInOpponent(o,vertices[i],0)){
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
bool checkIfRobotValid(RobotModel robot, Opponent opponent, Obstacles obstacles, float x, float y, float theta, Arena arena){
    // Checks if the robot is not within the arena and if so the robot is in a invalid position
    if(checkPositionInvalid(robot,arena,x,y,theta)){
        return false;
    }

    // Checks if the robot hit a obstacle and if so the robot is in an invalid position
    if(checkIfRobotHitObstacle(obstacles, x, y, theta, robot)){
        return false;
    }

    // If a robot's weapon hit an opponent and its body hit the opponent the position is ALWAYS valid (Opponent WILL be knowcked back)
    if(checkIfWeaponInOpponent(opponent,x,y,theta,robot)){
        return true;
    }
    else{
        // If a robot's weapon DID NOT hit an opponent and it has hit the opponent the position is NOT valid (Likely means hit by the opponent)
        if(checkIfBodyInOpponent(opponent,x,y,theta,robot)){
            return false;
        }
        else{
            return true;
        }
    }
}

// GENERATES STARTING OPPONENT AND ROBOT POSITIONS =============================================================

// Generates a opponent in a valid position in the arena
Opponent generateOpponent(Arena a, Obstacles ob, float maxSpeed, float maxRadius, float minRadius){
    Coordinate c;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distX(0, a.width);
    std::uniform_real_distribution<float> distY(0, a.length);
    std::uniform_real_distribution<float> distR(minRadius, maxRadius);
    std::uniform_real_distribution<float> genTheta(0, 2*M_PI);
    std::uniform_real_distribution<float> genVelocity(-maxSpeed, maxSpeed);

    // randomly generates a position for an opponent and if valid returns it
    bool invalid = true;
    float x;
    float y;
    float theta;
    float radius;
    float velocity;

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
                // Valid location therefore
                invalid = false;
            }
        }

    }

    // Generates opponent
    Opponent o;
    o.radius = radius;
    o.theta = theta;
    o.velocity = velocity;
    o.x0 = 0;
    o.x = x;
    o.y = y;
    o.y0 = y;

    return o;

}

// Generates a static starting position for the robot
Coordinate generateStartingPosition(Arena a, Obstacles ob, Opponent op, RobotModel robot){
    float maxSpeed = robot.robotDynamics.velocity_max;
    Coordinate c;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distX(0, a.width);
    std::uniform_real_distribution<float> distY(0, a.length);
    std::uniform_real_distribution<float> genTheta(0, 2*M_PI);

    // Assumes position is invalid initially
    bool invalid = true;
    while(invalid){
        // Generates a position
        c.x = distX(gen);
        c.y = distY(gen);
        c.theta = genTheta(gen);

        // Checks if the position is valid
        if(checkIfRobotValid(robot,op,ob,c.x,c.y,c.theta, a)){
            invalid = false;
        }
    }

    return c;


}
