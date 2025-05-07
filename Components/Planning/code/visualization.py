import matplotlib
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

# Modified get_path_from_file: now parses each non-empty line into a configuration (assumed format: x y theta [...])
def get_path_from_file(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            path = []
            for line in lines:
                if line.strip():
                    configuration = [float(ci) for ci in line.split()]
                    path.append(configuration)
        path = list(filter(lambda x: x, path))
        return path
    else:
        print("Path file not found - Please provide the right path")
        sys.exit(0)

def get_start_from_file(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            line = f.readline()
            return [float(x) for x in line.split()]
    else:
        print("Start file not found - Please provide the right path")
        sys.exit(0)

def get_goal_region_from_file(filepath):
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            line = f.readline()
            return [float(x) for x in line.split()]
    else:
        print("Goal region file not found - Please provide the right path")
        sys.exit(0)

def get_corners(pose, robot_size):
    x, y, theta = pose
    corners = [(x + robot_size/2 * math.cos(theta) - robot_size/2 * math.sin(theta), y + robot_size/2 * math.sin(theta) + robot_size/2 * math.cos(theta)), \
                (x - robot_size/2 * math.cos(theta) - robot_size/2 * math.sin(theta), y - robot_size/2 * math.sin(theta) + robot_size/2 * math.cos(theta)), \
                (x - robot_size/2 * math.cos(theta) + robot_size/2 * math.sin(theta), y - robot_size/2 * math.sin(theta) - robot_size/2 * math.cos(theta)), \
                (x + robot_size/2 * math.cos(theta) + robot_size/2 * math.sin(theta), y + robot_size/2 * math.sin(theta) - robot_size/2 * math.cos(theta))]
    return corners

def set_plot_properties(ax, path):
    ax.grid(True)
    ax.set_aspect('equal', 'box')
    ax.set_facecolor('#f0f0f0')
    # Set limits based on path extents with a margin
    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        margin = 1.0
        min_x, max_x = min(xs) - margin, max(xs) + margin
        min_y, max_y = min(ys) - margin, max(ys) + margin
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
    ax.set_title('Environment and Path Visualization')
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')

def draw_environment(ax, obstacles):
    # Draw static obstacles as black circles
    for obstacle in obstacles:
        if len(obstacle) == 4:
            # Rectangle fallback (legacy)
            ax.add_patch(patches.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3], fill=True, color='black'))
        elif len(obstacle) == 3:
            # Circle: x, y, radius
            ax.add_patch(patches.Circle((obstacle[0], obstacle[1]), obstacle[2], fill=True, color='black', alpha=0.7))
        # else: ignore

    # Draw dynamic obstacles if present
    build_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build")
    dyn_obs_path = os.path.join(build_dir, "dynamic_obstacles.txt")
    if os.path.exists(dyn_obs_path):
        with open(dyn_obs_path, 'r') as f:
            for line in f:
                vals = line.strip().split()
                if len(vals) >= 5:
                    x = float(vals[2])
                    y = float(vals[3])
                    radius = float(vals[4])
                    ax.add_patch(patches.Circle((x, y), radius, fill=True, color='orange', alpha=0.5, label='Dynamic Obstacle'))
    # Remove duplicate legend entries
    handles, labels = ax.get_legend_handles_labels()
    seen = set()
    new_handles, new_labels = [], []
    for h, l in zip(handles, labels):
        if l not in seen:
            new_handles.append(h)
            new_labels.append(l)
            seen.add(l)
    if new_labels:
        ax.legend(new_handles, new_labels)

def get_robot_shape(robot_data):
    # Returns (shape_type, size_tuple)
    # shape_type: "circle" or "rectangle"
    # size_tuple: (radius,) or (width, height)
    geom = robot_data.get("robot_geometry", [])
    # Defensive: check length and parse as needed
    try:
        if len(geom) >= 3 and float(geom[2]) != -1:
            return "circle", (float(geom[2]),)
        elif len(geom) >= 2:
            return "rectangle", (float(geom[0]), float(geom[1]))
        else:
            # fallback: small square
            return "rectangle", (0.4, 0.4)
    except (ValueError, IndexError):
        return "rectangle", (0.4, 0.4)

def get_robot_pose(config):
    # Returns (x, y, theta)
    return config[0], config[1], config[2]

def get_rectangle_corners(x, y, theta, width, height):
    # Returns corners of rectangle centered at (x, y) with orientation theta
    w = width / 2.0
    h = height / 2.0
    corners_local = np.array([
        [-w, -h],
        [ w, -h],
        [ w,  h],
        [-w,  h]
    ])
    rot = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    corners_world = np.dot(corners_local, rot.T) + np.array([x, y])
    return corners_world

def plot_environment_and_path(obstacles, path, start=None, goal_region=None, robot_data=None):
    _, ax = plt.subplots()
    draw_environment(ax, obstacles)

    # Draw path
    for i in range(len(path) - 1):
        x_values = [path[i][0], path[i + 1][0]]
        y_values = [path[i][1], path[i + 1][1]]
        ax.plot(x_values, y_values, 'b-', linewidth=1)

    # Draw robot along path
    if robot_data is not None:
        shape_type, size = get_robot_shape(robot_data)
        for config in path:
            x, y, theta = get_robot_pose(config)
            if shape_type == "circle":
                circle = patches.Circle((x, y), size[0], fill=False, color='green')
                ax.add_patch(circle)
            else:
                corners = get_rectangle_corners(x, y, theta, size[0], size[1])
                polygon = patches.Polygon(corners, fill=False, color='green')
                ax.add_patch(polygon)
    else:
        # fallback: draw as small rectangles
        for config in path:
            corners = get_corners(config[:3], 0.4)
            polygon = patches.Polygon(corners, fill=False, color='green')
            ax.add_patch(polygon)

    # Draw start point and robot at start
    if start is not None and robot_data is not None:
        shape_type, size = get_robot_shape(robot_data)
        x, y, theta = start
        ax.plot(x, y, marker='o', color='red', markersize=8, label='Start')
        if shape_type == "circle":
            circle = patches.Circle((x, y), size[0], fill=False, color='red', linestyle='--', linewidth=2)
            ax.add_patch(circle)
        else:
            corners = get_rectangle_corners(x, y, theta, size[0], size[1])
            polygon = patches.Polygon(corners, fill=False, color='red', linestyle='--', linewidth=2)
            ax.add_patch(polygon)
        ax.legend()

    # Draw goal region at the final robot position, with radius = opponent size * 2
    if path and goal_region is not None:
        final_x, final_y = path[-1][0], path[-1][1]
        # Use the opponent's radius (from goal_region[2]), but double it for visibility
        goal_radius = goal_region[2] * 2
        goal_circle = patches.Circle((final_x, final_y), goal_radius, fill=False, color='magenta', linestyle='--', linewidth=2, label='Goal Region')
        ax.add_patch(goal_circle)
        ax.legend()

    set_plot_properties(ax, path)
    plt.savefig('path_visualization.png', dpi=300)
    plt.close()

def animate_environment_and_path(obstacles, path, start=None, goal_region=None, robot_data=None):
    fig, ax = plt.subplots()

    draw_environment(ax, obstacles)
    line, = ax.plot([], [], 'g-', linewidth=1, label='Path')
    point, = ax.plot([], [], 'go', markersize=2, label='Pose')

    # Draw start point and robot at start
    if start is not None and robot_data is not None:
        shape_type, size = get_robot_shape(robot_data)
        x, y, theta = start
        ax.plot(x, y, marker='o', color='red', markersize=8, label='Start')
        if shape_type == "circle":
            circle = patches.Circle((x, y), size[0], fill=False, color='red', linestyle='--', linewidth=2)
            ax.add_patch(circle)
        else:
            corners = get_rectangle_corners(x, y, theta, size[0], size[1])
            polygon_start = patches.Polygon(corners, fill=False, color='red', linestyle='--', linewidth=2)
            ax.add_patch(polygon_start)

    # Draw goal region at the final robot position, with radius = opponent size * 2
    if path and goal_region is not None:
        final_x, final_y = path[-1][0], path[-1][1]
        goal_radius = goal_region[2] * 2
        goal_circle = patches.Circle((final_x, final_y), goal_radius, fill=False, color='magenta', linestyle='--', linewidth=2, label='Goal Region')
        ax.add_patch(goal_circle)

    # Draw robot at each frame
    if robot_data is not None:
        shape_type, size = get_robot_shape(robot_data)
        if shape_type == "circle":
            robot_patch = patches.Circle((path[0][0], path[0][1]), size[0], fill=False, color='green', label='Robot')
        else:
            corners = get_rectangle_corners(path[0][0], path[0][1], path[0][2], size[0], size[1])
            robot_patch = patches.Polygon(corners, fill=False, color='green', label='Robot')
        ax.add_patch(robot_patch)
    else:
        corners = get_corners(path[0][:3], 1)
        robot_patch = patches.Polygon(corners, fill=False, color='green', label='Robot')
        ax.add_patch(robot_patch)

    set_plot_properties(ax, path)

    def update(frame):
        config = path[frame]
        if frame > 0:
            x_values = [path[frame-1][0], config[0]]
            y_values = [path[frame-1][1], config[1]]
            ax.plot(x_values, y_values, 'b-', linewidth=1)
        if robot_data is not None:
            shape_type, size = get_robot_shape(robot_data)
            if shape_type == "circle":
                robot_patch.center = (config[0], config[1])
            else:
                corners = get_rectangle_corners(config[0], config[1], config[2], size[0], size[1])
                robot_patch.set_xy(corners)
        else:
            corners = get_corners(config[:3], 1)
            robot_patch.set_xy(corners)
        return line, point, robot_patch

    ani = FuncAnimation(fig, update, frames=len(path), blit=True, repeat=False)
    ani.save('path_visualization.gif', writer='imagemagick', fps=30)
    plt.close()

def main():
    cwd = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build")
    obs_path = os.path.join(cwd, "obstacles.txt")
    opp_path = os.path.join(cwd, "opponent.txt")
    house_path = os.path.join(cwd, "dynamic_obstacles.txt")
    robot_path = os.path.join(cwd, "robot.txt")
    path_path = os.path.join(cwd, "path.txt")
    start_path = os.path.join(cwd, "start.txt")
    goal_region_path = os.path.join(cwd, "goal_region.txt")

    obstacles = get_obstacles_from_file(obs_path)
    opponents = get_opponents_from_file(opp_path)
    housebots = get_housebots_from_file(house_path)
    robot = get_robot_from_file(robot_path)
    path = get_path_from_file(path_path)
    start = get_start_from_file(start_path)
    goal_region = get_goal_region_from_file(goal_region_path)

    print("Files loaded successfully from build folder")
    # Generate image and animation of the environment following the path
    plot_environment_and_path(obstacles, path, start, goal_region, robot)
    animate_environment_and_path(obstacles, path, start, goal_region, robot)

if __name__ == "__main__":
    main()
