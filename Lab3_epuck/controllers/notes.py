
'''
max or min nfc()

if more than 4
    return 4


saturation function
front straig pid v():


wallfollow():

    error = front distance sensor - inches from wall (2.5??)


    setSpeedIPS(v-abs(error)*kp, v)

    if > 3
        too close

        too far

        too close to other wall

    else goldy lock
        return max speed for vl and vr


wall follow():
    calculate front pid distance, get speed

# 2.5 target distance
# 3.5 for wall 45deg turn

'''
def getDistanceSensor():
    return []
def setSpeedIPS():
    pass

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