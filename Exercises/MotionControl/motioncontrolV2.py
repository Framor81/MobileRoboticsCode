import math
import time
import matplotlib
import random
import matplotlib.pyplot as plt 

# Global Variables
TRACK_WIDTH = 0.16
TIME_END = 10 

K_ORIENTATION = 3
K_POSITION = 3

GOAL = (3, 3, 0)
MAX_LINEAR_VELOCITY = 1
MAX_ANGULAR_VELOCITY = 1

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


def main():
    global SD
    fig, axs = plt.subplots(3, 4, figsize = (15, 5))
    fig.tight_layout(h_pad = 5)
    axs = axs.flatten()
    plt.subplots_adjust(hspace=1, top=0.9, bottom=0.2, left=0.1, right=0.9)

    for j in range(3):
        SD = [0.5, 1, 1.5]
        args = [(0, 0), (round(random.normalvariate(sigma=SD[j]), 2), 0), (0, round(random.normalvariate(sigma=SD[j]), 2)), (round(random.normalvariate(sigma=SD[j]), 2), round(random.normalvariate(sigma=SD[j]), 2))]
        print(args)

        for i in range(4):
            t = 0
            dt = 0.5
            v_left = 0.2
            v_right = 0.25
            pose = Pose()
            position = PositionControl()
            
            while t < TIME_END:
                # print(pose)
                pose = ForwardKinematics.forward_kinematics(pose, v_left, v_right, dt)
                v_left, v_right = position.position_control(pose, GOAL)
                v_left += args[i][0]
                v_right += args[i][1]

                t += dt

            plot_graph(position, axs[4 * j + i])
            axs[4 * j + i].set_xlabel("sd = " + str(SD) + "\n"
                                      + "left noise = " + str(args[i][0]) + "\n"
                                      + "right noise = " + str(args[i][1]) + "\n", fontsize = 8)

            pose.reset()

    plt.savefig('./noisymotioncontrol.png')
    plt.show()


if __name__ == '__main__':
    main()
