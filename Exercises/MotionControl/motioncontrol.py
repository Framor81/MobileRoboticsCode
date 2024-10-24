import math
import time

# Global Variables
TRACK_WIDTH = 0.16
TIME_END = 10 

K_ORIENTATION = 1.5
K_POSITION = 2

GOAL = (2, 2, 0)
MAX_LINEAR_VELOCITY = 0.5
MAX_ANGULAR_VELOCITY = 1.5

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
    def position_control(pose, GOAL):
        d = math.dist(pose.get_pose(), GOAL)
        # if d is small then return 0, 0
        # what do we consider small
        if (d < 0.5):
            return (0,0)

        v = K_POSITION * d  
        v = min(v, MAX_LINEAR_VELOCITY)

        # Not sure 
        dx = GOAL[0] - pose.x
        dy = GOAL[1] - pose.y
        angle_rad = math.atan2(dy, dx)
        angle_to_goal = angle_rad
        # angle_to_goal = math.degrees((pose.x, pose.y), (GOAL[0], GOAL[1]))
        theta_error = angle_to_goal - pose.theta
        theta_dot = K_ORIENTATION * theta_error
        theta_dot = min(theta_dot, MAX_ANGULAR_VELOCITY)

        v_left = v - theta_dot * TRACK_WIDTH / 2
        v_right = v + theta_dot * TRACK_WIDTH / 2

        return v_left, v_right


def main():
    initial_t = time.time()
    t = time.time()
    dt = 0.25
    v_left = 0.2
    v_right = 0.25
    pose = Pose()
    
    while t < initial_t + TIME_END:
        pose = ForwardKinematics.forward_kinematics(pose, v_left, v_right, dt)
        print(pose)
        v_left, v_right = PositionControl.position_control(pose, GOAL)
        t = time.time()

        interval = t + dt
        while t < interval:
            time.sleep(0.05)
            t = time.time()

        t = t + dt


if __name__ == '__main__':
    main()