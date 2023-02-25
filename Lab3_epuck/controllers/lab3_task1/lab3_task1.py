"""
Matias Cinera - U 6931_8506
CAP-6626
Instructor: Dr.Alfredo Weitzenfeld 
Ta:         Chance Hamilton
Assigment:  lab3_task1 controller.
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

################## - GETTERS & HELPER - ######################
def getPositionSensors():
    return leftposition_sensor.getValue(), rightposition_sensor.getValue()

def imu_cleaner(imu_reading):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*math.pi
    return math.degrees(rad_out)

def imu_rad():
    return math.radians(imu_cleaner(imu.getRollPitchYaw()[2]))

# Transform inches to rad - max wheel speed is 5.024 in/
def inToRad(v):
    return v/w_r


def straightMotionD(d):
    v = 6.28        # use max speed
    time = d/5.024  # 5.024 = v*r ==> max linear speed
    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break 
        leftMotor.setVelocity(v)
        rightMotor.setVelocity(v)

# angle must be provided in radians
def rotationInPlace(direction, angle):
    # quarter of a circle = 2pi/4  == pi/2
    # for this self rotation ICC R = d_mid
    s = angle*dmid
    in_v = 0.8
    time = s/in_v

    v = inToRad(in_v) # input must be less than 6.28

    s_time = robot.getTime()
    while robot.step(timestep) != -1:
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

def circularMotion(direction, R):
    vr = 2
    omega = vr/(R+dmid)
    vl = omega*(R-dmid)
    v = (vr+vl)/2
    s = (pi/2) * R
    time = s/v
 
    vl_rad = inToRad(vl)
    vr_rad = inToRad(vr) 

    if direction == "right":
        vl_rad = inToRad(vr)
        vr_rad = inToRad(vl)

    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
        leftMotor.setVelocity(vl_rad)
        rightMotor.setVelocity(vr_rad)

# Main loop:
# perform simulation steps until Webots is stopping the controller
def front_pid(goal, k):

    front_dist = frontDistanceSensor.getValue()
    d_inches = front_dist*39.3701
    print(d_inches)
    error = d_inches - goal 

    tolerance = (goal*1.01) - (goal*0.99)
    if tolerance < 0.1:
        tolerance = 0.1

    if abs(error) < tolerance:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        return
 
    # max speed is 5.024 inches --> 2pi * 0.8 (w radius)
    v = k*error
    motor_v = v/w_r
    print(f'error: {error:.5f}, v: {v:.5f}, tolerance: {tolerance:.5f}')

    if abs(motor_v) > 6.28:
        leftMotor.setVelocity(6.28)
        rightMotor.setVelocity(6.28)
    else:
        if v < 0.3:
            v = 0.3
        leftMotor.setVelocity(motor_v)
        rightMotor.setVelocity(motor_v)


# print("\tRight:\t", right_dist, "\t|", full_range_image[90])
# print("\tRear:\t", rear_dist, "\t|", full_range_image[180])
kps_vals = [0.1, 0.5, 1.0, 2.0, 2.5, 5.0]
while robot.step(timestep) != -1:
    front_pid(5, 1)
    # Read the sensors:
    # # Getting full Range Image from Lidar returns a list of 1800 distances = 5 layers X 360 distances
    # full_range_image = lidar.getRangeImage()
    # # print size of Range Image
    # print('#################################################################')
    # print("Lidar's Full Range Image Size: ", len(full_range_image))
    # # Compare Distance Sensors to Lidar Ranges
    # front_dist = frontDistanceSensor.getValue()
    # right_dist = rightDistanceSensor.getValue()
    # rear_dist = rearDistanceSensor.getValue()
    # left_dist = leftDistanceSensor.getValue()

    # print("Distance Sensor vs Lidar")
    # print("\tFront:\t", front_dist, "\t|", full_range_image[0])
    # print("\tRight:\t", right_dist, "\t|", full_range_image[90])
    # print("\tRear:\t", rear_dist, "\t|", full_range_image[180])
    # print("\tLeft:\t", left_dist, "\t|", full_range_image[270])

    # # Enter here functions to send actuator commands, like:
    # leftMotor.setVelocity(6)
    # rightMotor.setVelocity(6)

    # if full_range_image[0] < .07:

    #     leftMotor.setVelocity(0)
    #     rightMotor.setVelocity(0)
    #     break
# Enter here exit cleanup code.
