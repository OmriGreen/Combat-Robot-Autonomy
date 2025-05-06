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

// Returns true if the position is valid, false otherwise
bool checkPositionValid(RobotModel robot, Arena a, float x, float y, float theta){
    geometry r = robot.robotGeometry;
    Coordinate c;
    c.x = x;
    c.y = y;
    c.theta = theta;

    // Checking the main body of the robot ========================================
    // If the robot is not a circle
    if(r.robot_radius == -1){
        std::vector<Coordinate> vertices = getCorners(c, r.robot_width, r.robot_height, 0, 0);
        // Checks if every vertice is inside of the arena and if not returns false
        for(int i = 0; i <4; i++){
            if(checkIfPointInvalidSquare(a,vertices[i])){
                return false;
            }
        }
    }
    // If the main body of the robot is a circle
    else{
        if(checkIfPointInvalidRound(a,c,r.robot_radius)){
            return false;
        }
    }

    // Checking the weapon of the robot ==============================================
    
}



