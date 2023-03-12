"""
Matias Cinera - U 6931_8506
CAP-6626
Instructor: Dr.Alfredo Weitzenfeld 
Ta:         Chance Hamilton
Assigment:  lab2_task1 controller.
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
# Gets Robots Camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)
#######################################################
# Gets Robots Motors
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
#######################################################
# Gets Robot's the position sensors
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)
imu = robot.getDevice('inertial unit')
imu.enable(timestep)
#######################################################

# global variables
distBtwWhe = 2.28
dmid = distBtwWhe/2
w_dia = 1.6
w_r = w_dia/2
pi = math.pi
indx = 0
poseTimes = []
target_time = 999

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

################## - POSITION FUNCTIONS - ######################
# Return the distance in in/s based on the R and L Wheel speed
def distAfterTask(vl, vr):
    vl = round(vl, 8)
    vr = round(vr, 8)
    
    if vl == vr:  
        return 0.032*vl, 0.032*vr
    if vl == -vr or math.isnan(vl):
        return 0, 0
    else:
        # circular motions turns
        # vl = ( (cur_l-prev_l) /0.032) * wheel_radius
        v = (vr+vl)/2
        S = v*0.032
        x = S*math.cos(imu.getRollPitchYaw()[2])
        y = S*math.sin(imu.getRollPitchYaw()[2])
        return abs(x), abs(y)

# Position variables
cur_position = [-5,-15, 0] 
prev_l, prev_r = getPositionSensors()
signs = [0, 0]

def robotPose():
    print(f'Time: {robot.getTime():.2f}s - Robot Position: [{cur_position[0]:.2f}, {cur_position[1]:.2f}, {cur_position[2]:.2f}]')
    print("------------------------------------------------------")

def updateRobotPose():
    global prev_l, prev_r, target_time, indx
    cur_l, cur_r = getPositionSensors()
    # 0.032 = time step
    vl = (cur_l-prev_l)/0.032
    vr = (cur_r-prev_r)/0.032
    
    x, y = distAfterTask(vl*w_r, vr*w_r)

    if signs[0] == 1:
        cur_position[0] += x
    elif signs[0] == -1:
        cur_position[0] -= x
    
    # print(f'x: {x}, y: {y}')
    if signs[1] == 1:
        cur_position[1] += y
    elif signs[1] == -1:
        cur_position[1] -= y
    prev_l = cur_l
    prev_r = cur_r
    
    cur_position[2] = imu_cleaner(imu.getRollPitchYaw()[2])

    if robot.getTime() >= target_time:
        robotPose()
        indx+=1
        try:
            target_time = poseTimes[indx]
        except:
            target_time = 9999

##################################################################    
    
################## - MOTION FUNCTIONS - ######################
# move in straing line at max speed = 5.024 in/s
def straightMotionD(d):
    v = 6.28        # use max speed
    time = d/5.024  # 5.024 = v*r ==> max linear speed
    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            updateRobotPose()
            break 
        leftMotor.setVelocity(v)
        rightMotor.setVelocity(v)
        updateRobotPose()
        
 
def rotationInPlace(direction):
    # quarter of a circle = 2pi/4  == pi/2
    # for this self rotation ICC R = d_mid
    s = pi*dmid / 2
    # s = v * t
    in_v = 0.4
    time = s/in_v
    # time = 15
    # in_v = s/time
    v = inToRad(in_v) # input must be less than 6.28

    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            updateRobotPose()
            break 
        if direction == "left":
            leftMotor.setVelocity(-v)
            rightMotor.setVelocity(v)
        else:
            leftMotor.setVelocity(v)
            rightMotor.setVelocity(-v)
        updateRobotPose()

def circularMotion(direction):
    vr = 2
    R = 5   # radius of the circle, hardcoded for this task.
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
            updateRobotPose()
            break 
        leftMotor.setVelocity(vl_rad)
        rightMotor.setVelocity(vr_rad)
        updateRobotPose()
##################################################################

def getRobotCommands(A):
    cur_pose = "east"   # default starting position for this lab
    commands = []
    prev = []
    for i in range(len(A)):
        if i == 0:
            prev = A[i]
            continue
        # horizontal movement
        if prev[0] != A[i][0] and prev[1] == A[i][1]:
            # left to right
            if prev[0] < A[i][0]: # looking east 2
                if cur_pose == "north":
                    commands.append([0, 0, 0])
                elif cur_pose == "south":
                    commands.append([-1, 0, 0]) 
                cur_pose = "east"
                commands.append([abs(prev[0] - A[i][0]), 1, 0])           
            # right to left
            else: # looking west
                if cur_pose == "north":
                    commands.append([-1, 0, 0])
                elif cur_pose == "south":
                    commands.append([0, 0, 0]) 
                cur_pose = "west"
                commands.append([abs(prev[0] - A[i][0]), -1, 0])            
        # vertical movement
        elif prev[0] == A[i][0] and prev[1] != A[i][1]:
            # going up
            if prev[1] < A[i][1]:   
                if cur_pose == "east":
                    commands.append([-1, 0, 0]) 
                elif cur_pose == "west":
                    commands.append([0, 0, 0]) 
                cur_pose = "north"
                commands.append([abs(prev[1] - A[i][1]), 0, 1])          
            # going down
            else:                   
                if cur_pose == "east":
                    commands.append([0, 0, 0])
                elif cur_pose == "west":
                    commands.append([-1, 0, 0])
                cur_pose = "south"          
                commands.append([abs(prev[1] - A[i][1]), 0, -1]) 
        # 1/4 circle moves - hardcoded moves that are not straight lines 
        elif prev[0] != A[i][0] and prev[1] != A[i][1]:
            if cur_pose == "west":
                commands.append([-3, -1, -1])
                cur_pose = "south"
            elif cur_pose == "east":
                commands.append([-2, 1, -1])
                cur_pose = "south"
            elif cur_pose == "south":
                commands.append([-2, -1, -1])
                cur_pose = "west"
        prev = A[i]

    return commands
    
# commands = [20, -1, 30, -1, 25, -3, 5, -1, 15, -2, -2, 10, -3, 5] --> commands should be this
# commands = [command, x-change, y-change], x and y changes can either be -1, 0 or 1.
def robotWayPointNav(A, T=[]):
    global poseTimes, target_time, indx
    poseTimes = T
    target_time = T[indx]
    commands = getRobotCommands(A)
    for it in commands:
        global signs
        signs = [it[1], it[2]]
        ctype = it[0]
        if ctype == -1:   # rotate in place left
            rotationInPlace("left")
        elif ctype == 0:  # rotate in place right
            rotationInPlace("right")
        elif ctype == -3: # quarter of a circle left
            circularMotion("left")
        elif ctype == -2: # quarter of a circle right
            circularMotion("right")
        else:       # else its a straight motion
            straightMotionD(ctype)


# get the initial position of the robot 
def main():
    A = [[-5,-15],[15,-15],[15,15], [-10,15] ,[-15,10] ,[-15,5], [0,5] ,[5, 0] , [0,-5], [-10,-5] ,[-15,-10] ,[-15,-15]]
    robotWayPointNav(A, [1, 2, 3, 5, 10, 15, 20, 23, 30])
    # spin :)
    while robot.step(timestep) != -1:
        leftMotor.setVelocity(3)
        rightMotor.setVelocity(-3)
        
if __name__ == "__main__":
    main()