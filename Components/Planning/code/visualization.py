import matplotlib
import argparse
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import os 
import sys
from matplotlib import patches
import math
import numpy as np

# Gets a list of obstacle/s from the file
def get_obstacles_from_file(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            obstacles = []
            
            for line in lines:
                obstacle = line.split()
                obstacles.append([float(obstacle[i]) for i in range(len(obstacle))])
        obstacles = list(filter(lambda x: x, obstacles))
        return obstacles
    
    else:
        print("Obstacle file not found - Please provide the right path")
        sys.exit(0)

#  Gets a list of opponent/s from the file
def get_opponents_from_file(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            opponents = []
            
            for line in lines:
                opponent = line.split()
                opponents.append([float(opponent[i]) for i in range(len(opponent))])
        opponents = list(filter(lambda x: x, opponents))
        return opponents
    
    else:
        print("Oppoenent file not found - Please provide the right path")
        sys.exit(0)

#  Gets a list of housebot/s from the file
def get_housebots_from_file(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            housebots = []
            
            for line in lines:
                housebot = line.split()
                housebots.append([float(housebot[i]) for i in range(len(housebot))])
        housebots = list(filter(lambda x: x, housebots))
        return housebots
    
    else:
        print("Housebot file not found - Please provide the right path")
        sys.exit(0)

#  Gets a data for the robot
def get_robot_from_file(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            data = f.readlines()
            robot_data = {}
            robot_data["config_data"] = data[0].split()
            robot_data["robot_geometry"] = data[1].split()
            robot_data["weapon_geometry"] = data[2].split()
            robot_data["Wheel_geometry"] = []
            for line in data[3:]:
                print(line)
            print(robot_data)

        return robot_data
    
    else:
        print("Robot file not found - Please provide the right path")
        sys.exit(0)

# STILL NEEDS TO BE IMPLEMENTED AFTER EVERYTHING ELSE IS DONE
def get_path_from_file(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            path = []
            print("NOT IMPLEMENTED YET, COMPLETE CODE TO UNDERSTAND HOW DATA IS INPUTTED")
            # for l in lines:
            #     configuration = l.split()
            #     path.append([float(ci) for ci in configuration])

        path = list(filter(lambda x: x, path))
        return path
    else:
        print("Path file not found - Please provide the right path")
        sys.exit(0)

def get_corners(pose, robot_size):
    x, y, theta = pose
    corners = [(x + robot_size/2 * math.cos(theta) - robot_size/2 * math.sin(theta), y + robot_size/2 * math.sin(theta) + robot_size/2 * math.cos(theta)), \
                (x - robot_size/2 * math.cos(theta) - robot_size/2 * math.sin(theta), y - robot_size/2 * math.sin(theta) + robot_size/2 * math.cos(theta)), \
                (x - robot_size/2 * math.cos(theta) + robot_size/2 * math.sin(theta), y - robot_size/2 * math.sin(theta) - robot_size/2 * math.cos(theta)), \
                (x + robot_size/2 * math.cos(theta) + robot_size/2 * math.sin(theta), y + robot_size/2 * math.sin(theta) - robot_size/2 * math.cos(theta))]
    return corners

def set_plot_properties(ax):
    ax.grid(True)
    ax.set_aspect('equal', 'box')
    ax.set_facecolor('#f0f0f0')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_title('Environment and Path Visualization')
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')

def draw_environment(ax, obstacles):
    for obstacle in obstacles:
        ax.add_patch(patches.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3], fill=True, color='black'))

def plot_environment_and_path(obstacles, path):
    _, ax = plt.subplots()
    draw_environment(ax, obstacles)

    for i in range(len(path) - 1):
        x_values = [path[i][0], path[i + 1][0]]
        y_values = [path[i][1], path[i + 1][1]]
        ax.plot(x_values, y_values, 'b-', linewidth=1)

    for config in path:
       corners = get_corners(config[:3], 0.4)
       polygon = patches.Polygon(corners, fill=False, color='green')
       ax.add_patch(polygon)

    set_plot_properties(ax)
    plt.savefig('path_visualization.png', dpi=300)
    plt.close()

def animate_environment_and_path(obstacles, path):
    fig, ax = plt.subplots()

    draw_environment(ax, obstacles)
    line, = ax.plot([], [], 'g-', linewidth=1, label='Path')
    point, = ax.plot([], [], 'go', markersize=2, label='Pose')
    corners = get_corners(path[0][:3], 1)
    polygon = patches.Polygon(corners, fill=False, color='green', label='Robot')
    ax.add_patch(polygon)
    set_plot_properties(ax)

    def update(frame):
        config = path[frame] 
        if frame > 0:
            x_values = [path[frame-1][0], config[0]]
            y_values = [path[frame-1][1], config[1]]
            ax.plot(x_values, y_values, 'b-', linewidth=1)
        corners = get_corners(config[:3], 1)
        polygon.set_xy(corners)
        return line, point, polygon

    ani = FuncAnimation(fig, update, frames=len(path), blit=True, repeat=False)
    ani.save('path_visualization.gif', writer='imagemagick', fps=30)
    plt.close()

def main():
    parser = argparse.ArgumentParser(description='Visualize the path of a combat robot / battlebot in an environment with obstacles, a housebot, and an opponent. It also includes data on robot dynamics \n All data is in m or Newton/m')
    parser.add_argument('--obstacles', type=str, default='obstacles', help='Name of the obstacles file')
    parser.add_argument('--opponents', type=str, default='opponents', help='Name of the opponent file') 
    parser.add_argument('--housebots', type=str, default='housebots', help='Name of the housebot file') 
    parser.add_argument('--robot', type=str, default='robot', help='Name of the robot file') 
    parser.add_argument('--path', type=str, default='path', help='Name of the path file') 

    args = parser.parse_args()

    print("***" * 19 + f"\n Visualising the trajectory of {args.robot} on path {args.path} on the way to impact {args.opponents} while avoiding {args.housebots} and obstacles {args.obstacles}\n" + "***" * 19)
    print("***" * 19 + f"\nThe assumed PWMInputs are 1: Left Drive, 2: Right Drive, 3: Weapon Input\n" + "***" * 19)
    print("Instructions Obstacles: \n1. Please ensure that the obstacles text file should contain the obstacle data in the format: x y radius")
    print("2. Instructions Housebot & Opponents: \n2. Please ensure that the housebot/s and oponent/s text files should contain the robot trajectory data in the format: x y theta radius velocity")
    print("3. Instructions Robot: \n3. Please ensure that the robot text files contain the robot geometry data with the first line being: isCircle, isHorizontal")
    print("3. Instructions Robot: \n3.1.1. isCircle refers to the robots geometry, if true the next line should contain the robot's radius in the format: radius")
    print("3. Instructions Robot: \n3.1.2. isCircle refers to the robots geometry,  if false the next line should contain data on the robot's main body in the format: width height")
    print("3. Instructions Robot: \n3.2.1. isHorizontal refers to the robots weapon,  if false the following lines should contain data on the robot's weapon in the format all data is from the back center if isCircle if false, from the center if isCircle is true: x y width height isClockwise MOI PWMInputVal")
    print("3. Instructions Robot: \n3.2.2. isHorizontal refers to the robots weapon,  if true the following lines should contain data on the robot's weapon in the format: x y radius isClockwise MOI PWMInputVal")
    print("3. Instructions Robot: \n3.3. Following Line refers to weapon motor , while not used in visualization | format: motor_wattage max_RPM gear_reduction")
    print("3. Instructions Robot: \n3.4. Following Line refers to drive motor wattage, while not used in visualization, very important to planner")
    print("3. Instructions Robot: \n3.5. Following Lines refers to the weight and MOI of the robot in kg / N\m| format: mass MOI")

    print("3. Instructions Robot: \n3.6. Following Lines refer to wheel locations, they are formated: x y MOI radius PWMInputVal and are related to the location based on isCircle")

    print("4. The path text file should specify the the poses of the robot lines \n")

    # Paths to each file
    obs_path = os.path.join(os.getcwd(), args.obstacles)
    opp_path = os.path.join(os.getcwd(), args.opponents)
    house_path = os.path.join(os.getcwd(), args.housebots)
    robot_path = os.path.join(os.getcwd(), args.robot)
    path_path = os.path.join(os.getcwd(), args.path)

    obstacles = get_obstacles_from_file(obs_path)
    opponents = get_opponents_from_file(opp_path)
    housebots = get_opponents_from_file(house_path)
    robot = get_robot_from_file(robot_path)
    path = get_path_from_file(path_path)

    print("Obstacle, Opponents, Housebots, Robot, and Path data loaded successfully")

    # plot_environment_and_path(obstacles, path)
    # animate_environment_and_path(obstacles, path)

if __name__ == "__main__":
    main()
