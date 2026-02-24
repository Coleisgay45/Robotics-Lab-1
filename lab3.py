"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np

pose_x = 0
pose_y = 0
pose_theta = 0

# create the Robot instance.
robot = Supervisor()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1257 # ePuck wheel speed in m/s
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 0
vR = 0

# Initialize gps and compass for odometry
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

# TODO: Find waypoints to navigate around the arena while avoiding obstacles

# (Add some more waypoints, this is jutt one I chose out) - Stephen

waypoints = [[-0.064705, -0.414838, 0.0199956]]
# Index indicating which waypoint the robot is reaching next
index = 0

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")

state = 'turn_drive_turn_control'

rho_thresh = 0.03 # metrics are in meters
angle_thresh = 0.08 # radians


# this will be used for the proportional controller state
k_rho = 2
k_alpha = 6
k_eta = 2

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:
    # Set the position of the marker
    marker.setSFVec3f([waypoints[index][0], waypoints[index][1], 0.01])
    
    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # Read pose_x, pose_y, pose_theta from gps and compass
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    pose_theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])
    
    # TODO: controller
    xr = pose_x
    yr = pose_y
    theta = pose_theta
    
    xg, yg, thetag = waypoints[index]
    
    position_error = np.sqrt((xg - xr)**2+(yg - yr)**2)
    bearing_error = np.atan2(yg - yr, xg - xr) - theta
    heading_error = thetag - theta
    # ________________________________________________________________________________________
    # Everything above this line is what we did in class and I fixed up some hard coded stuff
    
    # Same thing as above just as greek symbols
    rho = np.sqrt((xg - xr)**2+(yg - yr)**2)
    alpha = np.atan2(yg - yr, xg - xr) - theta
    eta = thetag - theta
    
    # Velocities of left and right
    vL = 0
    vR = 0
    
    #turn_drive_turn_control state
    if state == "turn_drive_turn_control":
        if(abs(alpha) > angle_thresh and rho > rho_thresh):
            w = 2 * alpha
            vL = -w
            vR = w
            
            
            # This code moves the robot to the waypoint
            
        elif rho > rho_thresh:
            v = 3
            vL = v
            vR = v
            
            # if you guys are confused the thresh is just like
            # okay its close enough to this error so it will just still
            # work, so its close enough to still be solved.
            
        elif abs(eta) > angle_thresh:
            w = 2 * eta
            vL = -w
            vR = w
        else:
            vL = 0
            vR = 0
            index = (index + 1)% len(waypoints)
        
    elif state == "proportional_controller":
        xR_dot = k_rho * rho
        
    
    print(position_error, bearing_error, heading_error)
    print("Current pose: [%5f, %5f, %5f]" % (xr, yr, theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
