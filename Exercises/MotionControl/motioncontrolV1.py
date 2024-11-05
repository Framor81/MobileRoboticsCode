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

    def reset(self):
        self.x = 0
        self.y = 0
        self.theta = 0

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

        d = math.dist(pose.get_pose()[:2], goal[:2])
        # if d is small then return 0, 0
        # what do we consider small
        if (d < 0.25):
            return 0, 0

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
    
    subplot.plot(position.xs, position.ys)
    subplot.plot(GOAL[0], GOAL[1], marker="x", markersize=20)


    subplot.set_title("Goal: "  + str(GOAL))
    subplot.set_xlabel("Max Linear Velocity: " + str(MAX_LINEAR_VELOCITY) + "\n"
                       + "Max Angular Velocity: " + str(MAX_ANGULAR_VELOCITY) + "\n"
                       + "K Position: " + str(K_POSITION) + "\n" 
                       + "K Orientation: " + str(K_ORIENTATION), ha = "center", fontsize = 8)


def main():
    global K_ORIENTATION, K_POSITION, GOAL, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY  # Declare global variables

    fig, axs = plt.subplots(2, 5, figsize = (15, 5))
    fig.tight_layout(h_pad = 5)
    axs = axs.flatten()
    plt.subplots_adjust(hspace=1, top=0.9, bottom=0.2, left=0.1, right=0.9)
    
    for i in range(10):
        t = 0
        dt = 0.5
        v_left = 0.2
        v_right = 0.25
        pose = Pose()
        position = PositionControl()
        
        while t < TIME_END:
            print(pose)
            pose = ForwardKinematics.forward_kinematics(pose, v_left, v_right, dt)
            v_left, v_right = position.position_control(pose, GOAL)

            t += dt

        plot_graph(position, axs[i])
        
        K_ORIENTATION = round(random.uniform(0, 4), 2)
        K_POSITION = round(random.uniform(0, 4), 2)

        GOAL = (round(random.uniform(-3, 3), 2), 
                round(random.uniform(-3, 3),2), 
                round(random.uniform(-2 * math.pi, 2 * math.pi), 2))
        MAX_LINEAR_VELOCITY = round(random.uniform(0, 2), 2)
        MAX_ANGULAR_VELOCITY = round(random.uniform(0, 2), 2)

        pose.reset()

    plt.savefig('./motioncontrol.png')
    plt.show()


if __name__ == '__main__':
    main()
