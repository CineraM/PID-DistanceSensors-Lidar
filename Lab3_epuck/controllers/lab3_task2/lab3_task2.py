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

def setSpeedIPS(vl, vr):
    vlr = vl/w_r
    if vlr > 6.28: vlr = 6.28
    if vlr < -6.28: vlr = -6.28

    vrr = vr/w_r
    if vrr > 6.28: vrr = 6.28
    if vrr < -6.28: vrr = -6.28

    leftMotor.setVelocity(vlr)
    rightMotor.setVelocity(vrr)

def getDistanceSensor():
    toIn = 39.3701
    return [leftDistanceSensor.getValue()*toIn, rightDistanceSensor.getValue()*toIn, frontDistanceSensor.getValue()*toIn]

def wallFollow(wall, targetDistance, kp):
    v=4
    # true for left
    if wall:
        error = getDistanceSensor()[0] - targetDistance # target distance
        print(error)
        if error < 0:
            setSpeedIPS(v-abs(error)*kp, v) # turn away from right wall
        else:
            setSpeedIPS(v, v-abs(error)*kp) # turwn towards right wall
    else:
        error = getDistanceSensor()[1] - targetDistance # target distance
        print(error)
        if error < 0:
            setSpeedIPS(v, v-abs(error)*kp) # turn away from left wall
        else:
            setSpeedIPS(v-abs(error)*kp, v) # turwn towards left wall



# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    d = getDistanceSensor()
    if d[1]>d[0]:
        wallFollow(False, 2.5, .1)
    else:
        wallFollow(True, 2.5, .1)
    # # Read the sensors:
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
