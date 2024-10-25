import math
import time
import matplotlib
import random
import matplotlib.pyplot as plt 

# Global Variables
TRACK_WIDTH = 0.16
TIME_END = 10 

K_ORIENTATION = 0
K_POSITION = 0

GOAL = (0, 0, 0)
MAX_LINEAR_VELOCITY = 0
MAX_ANGULAR_VELOCITY = 0

# pose class
class Pose:
    x = 0
    y = 0
    theta = 0
    # initialize our Pose 
    # def __init__(self):
    #     self.x = 0
    #     self.y = 0
    #     self.theta = 0
    def get_pose(self): 
        return (self.x, self.y, self.theta)

    def __str__(self):
        return str(self.x) + " " + str(self.y) + " " + str(self.theta)


# forward kinematics class
class ForwardKinematics:
    def forward_kinematics(pose, v_left, v_right, dt):
        v = (v_left + v_right) / 2
        thetadot = (v_right - v_left) / TRACK_WIDTH

        pose.x += v * math.cos(pose.theta) * dt
        pose.y += v * math.sin(pose.theta) * dt
        pose.theta += thetadot * dt

        return pose



# position control class
class PositionControl: 
    def __init__(self):
        self.xs = []
        self.ys = []
        self.thetas = []

    def position_control(self, pose, goal):
        self.xs.append(pose.get_pose()[0])
        self.ys.append(pose.get_pose()[1])
        self.thetas.append(pose.get_pose()[2])

        d = math.dist(pose.get_pose(), goal)
        # if d is small then return 0, 0
        # what do we consider small
        if (d < 0.5):
            return (0,0)

        v = K_POSITION * d  
        v = min(v, MAX_LINEAR_VELOCITY)

        # Not sure 
        dx = goal[0] - pose.x
        dy = goal[1] - pose.y
        angle_rad = math.atan2(dy, dx)
        angle_to_goal = angle_rad
        # angle_to_goal = math.degrees((pose.x, pose.y), (GOAL[0], GOAL[1]))
        theta_error = angle_to_goal - pose.theta
        theta_dot = K_ORIENTATION * theta_error
        theta_dot = min(theta_dot, MAX_ANGULAR_VELOCITY)

        v_left = v - theta_dot * TRACK_WIDTH / 2
        v_right = v + theta_dot * TRACK_WIDTH / 2

        return v_left, v_right


def plot_graph(position, subplot):
    
    plot = plt.plot(position.xs, position.ys)

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Current Goal: "  + str(GOAL))
    plt.figtext(0, 1, "Max Linear Velocity: " + str(MAX_LINEAR_VELOCITY))
    plt.figtext(0, 2, "Max Angular Velocity: " + str(MAX_ANGULAR_VELOCITY))
    plt.figtext(0, 3, "K Position: " + str(K_POSITION))
    plt.figtext(0, 4, "K Orientation: " + str(K_ORIENTATION))

    

    return plot


def main():
    fig, axs = plt.subplots(2, 5)
    
    for i in range(10):
        initial_t = time.time()
        t = time.time()
        dt = 0.25
        v_left = 0.2
        v_right = 0.25
        pose = Pose()
        position = PositionControl()
        
        while t < initial_t + TIME_END:
            pose = ForwardKinematics.forward_kinematics(pose, v_left, v_right, dt)
            print(pose)
            v_left, v_right = position.position_control(pose, GOAL)
            t = time.time()

            interval = t + dt
            while t < interval:
                time.sleep(0.05)
                t = time.time()

            t = t + dt

        plot_graph(position, axs[i])
        
        K_ORIENTATION = random.uniform(0, 4)
        K_POSITION = random.uniform(0, 4)

        GOAL = (random.uniform(-3, 3), random.uniform(-3, 3), random.uniform(-3, 3))
        MAX_LINEAR_VELOCITY = random.uniform(0, 2)
        MAX_ANGULAR_VELOCITY = random.uniform(0, 2)


if __name__ == '__main__':
    main()