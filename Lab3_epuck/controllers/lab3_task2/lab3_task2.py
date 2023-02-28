"""
Matias Cinera - U 6931_8506
CAP-6626
Instructor: Dr.Alfredo Weitzenfeld 
Ta:         Chance Hamilton
Assigment:  lab3_task2 controller.
"""
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math

#######################################################
# Creates Robot
#######################################################
robot = Robot()
#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())
#######################################################
# Gets Robots Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/distancesensor
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)

#######################################################
# Gets Robots Lidar Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/lidar
#######################################################
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar_horizontal_res = lidar.getHorizontalResolution()
lidar_num_layers = lidar.getNumberOfLayers()
lidar_min_dist = lidar.getMinRange()
lidar_max_dist = lidar.getMaxRange()

print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res, "\nLidar Image Number of Layers: ", lidar_num_layers)
print("Lidar Range: [",lidar_min_dist," ,", lidar_max_dist,'] in meters')

#######################################################
# Gets Robots Camera
# Documentation:
#  https://cyberbotics.com/doc/reference/camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#######################################################
# Gets Robots Motors
# Documentation:
#  https://cyberbotics.com/doc/reference/motor
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
#######################################################
# Gets Robot's the position sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/positionsensor
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

#######################################################
# Gets Robot's IMU sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/inertialunit
#######################################################
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# global variables
distBtwWhe = 2.28
dmid = distBtwWhe/2
w_dia = 1.6
w_r = w_dia/2
pi = math.pi
half_of_robot = 0.037*39.3701 

# get left & right pid
def getDistanceSensor():
    toIn = 39.3701
    return [leftDistanceSensor.getValue()*toIn, rightDistanceSensor.getValue()*toIn]

def getLidar():
    image = lidar.getRangeImage()
    toIn = 39.3701
    return [image[270]*toIn - half_of_robot, image[90]*toIn - half_of_robot]

# set speed to motors
def setSpeedIPS(vl, vr):
    vl /= w_r
    vr /= w_r
    leftMotor.setVelocity(vl)
    rightMotor.setVelocity(vr)

# saturation fnc
def v_saturation(v, max):
    if math.isinf(v):
        return max
    if v > max:
        return max
    if v < -max:
        return -max
    return v

# return the distance in inches from the front pid
def front_pid():
    return frontDistanceSensor.getValue()*39.3701

def front_lidar():
    image = lidar.getRangeImage()
    return image[0]*39.3701

def printSensors():
    pids = getDistanceSensor()
    lids = getLidar()
    print(f'Distance:\t\tFront: {front_pid():.2f}\tLeft: {pids[0]:.2f}\tRight: {pids[1]:.2f}\n')
    print(f'Lidar:\t\tFront: {front_lidar():.2f}\tLeft: {lids[0]:.2f}\tRight: {lids[1]:.2f}\n')
# assume angle is in radians
def rotationInPlace(direction, angle, in_v):
    s = angle*dmid
    time = s/in_v
    v = in_v/w_r # input must be less than 6.28
    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        printSensors()
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break 
        if direction == "left":
            leftMotor.setVelocity(-v)
            rightMotor.setVelocity(v)
        else:
            leftMotor.setVelocity(v)
            rightMotor.setVelocity(-v)

def wallFollow(wall, fpid, k):
    pids = getDistanceSensor()
    left_pid = pids[0]
    right_pid = pids[1]
    
    dist_to_wall = 3
    v = v_saturation(fpid, 4)
    error = (v - 2.5)  # target distance to wall = 2.5 inches
    # error = (v - 2.5)*0.8  # target distance to wall = 2.5 inches
    if wall == 'right':    
        if fpid > 3:
            if right_pid < dist_to_wall:    # too close to target wall
                setSpeedIPS(v-abs(error)*k, v)
            elif right_pid > dist_to_wall:  # too far to target wall
                setSpeedIPS(v, v-abs(error)*k)
            elif left_pid < dist_to_wall:   # too close to opposite wall
                setSpeedIPS(v, v-abs(error)*k)
        else:
            setSpeedIPS(fpid, fpid)
    elif wall == 'left':
        if fpid > 3:

            if left_pid < dist_to_wall:    # too close to target wall
                setSpeedIPS(v, v-abs(error)*k)
            elif left_pid > dist_to_wall:  # too far to target wall
                setSpeedIPS(v-abs(error)*k, v)
            elif right_pid < dist_to_wall: # too close to opposite wall
                setSpeedIPS(v-abs(error)*k, v)
        else:
            setSpeedIPS(fpid, fpid)

def wallFollowLidar(wall, flid, k):
    lids = getLidar()
    left_lid = lids[0]
    right_lid = lids[1]
    dist_to_wall = 3
    v = v_saturation(flid, 4)
    error = (v - 2.5)  # target distance to wall = 2.5 inches
    # error = (v - 2.5)*0.8  # target distance to wall = 2.5 inches
    if wall == 'right':    
        if flid > 3:
            
            if right_lid < dist_to_wall:    # too close to target wall
                setSpeedIPS(v-abs(error)*k, v)
            elif right_lid > dist_to_wall:  # too far to target wall
                setSpeedIPS(v, v-abs(error)*k)
            elif left_lid < dist_to_wall:   # too close to opposite wall
                setSpeedIPS(v, v-abs(error)*k)
        else:
            setSpeedIPS(v, v)
    elif wall == 'left':
        if flid > 3:
            if left_lid < dist_to_wall:    # too close to target wall
                setSpeedIPS(v, v-abs(error)*k)
            elif left_lid > dist_to_wall:  # too far to target wall
                setSpeedIPS(v-abs(error)*k, v)
            elif right_lid < dist_to_wall: # too close to opposite wall
                setSpeedIPS(v-abs(error)*k, v)
        else:
            setSpeedIPS(v, v)

# Main loop:
# perform simulation steps until Webots is stopping the controller
kps_vals = [0.1, 0.5, 1.0, 2.0, 2.5, 5.0]
while robot.step(timestep) != -1:
    printSensors()
    fpid = front_pid()
    wall = 'left'
    if fpid < 2.5:  # to close to wall, rotate 45 deg away from it
        if wall == 'left':
            rotationInPlace('right', pi/4, 0.9)
        elif wall == 'right':
            rotationInPlace('left', pi/4, 0.9)
    else:   # else follow wall
        # pids
        # wallFollow(wall, fpid, kps_vals[2])

        # lidar
        wallFollowLidar(wall, front_lidar() - half_of_robot, kps_vals[2])