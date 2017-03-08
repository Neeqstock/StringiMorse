#general utility
halfPhi = 1.57
Phi = 3.14

class Position:
    x = 0.0
    y = 0.0
    yaw = 0.0

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

def approx(inputVal: float, testVal: float, approx: float = 0.0025) -> bool:
    lowerBound = testVal - approx
    upperBound = testVal + approx
    if inputVal > lowerBound and inputVal < upperBound:
        return True
    else:
        return False

#prints
def print_pos(pos):
    print("X = %s, Y = %s and yaw =%s" % (pos.get()['x'], pos.get()['y'], pos.get()['yaw']))

    # only displaying useful information

def print_position(pos: Position):
    print("X = %s, Y = %s and yaw =%s" % (pos.x, pos.y, pos.yaw))

    # only displaying useful information

def print_prox(proxi):
    print("target = %s" % proxi.get()['near_objects']['target'])

def print_ir(ir):
    print("left = %s, center = %s, right = %s" % (ir.get()['range_list'][20], ir.get()['range_list'][10], ir.get()['range_list'][0]))

    # range 20 is the leftmost, 0 the rightmost

#basic motion
def move(motion, v, w=0.0):
    motion.publish({"v": v, "w": w})

def stop(motion):
    motion.publish({"v": 0, "w": 0})

def turn90Right(pos, motion):
    stop(motion)
    yaw = pos.get()['yaw']
    resultYaw = yaw - halfPhi
    if resultYaw < - Phi:
        resultYaw = resultYaw + 2*Phi
    move(motion, 0, -0.4)
    while not approx(pos.get()['yaw'],resultYaw):
        pass
    stop(motion)

def turn90Left(pos, motion):
    yaw = pos.get()['yaw']
    resultyaw = yaw + halfPhi
    if resultyaw > Phi:
        resultyaw = resultyaw - 2*Phi
    move(motion, 0, 0.4)
    while not approx(pos.get()['yaw'],resultyaw):
        pass
    stop(motion)

def turn180(pos, motion):
    stop(motion)
    yaw = pos.get()['yaw']
    resultYaw = yaw - Phi
    if resultYaw < - Phi:
        resultYaw = resultYaw + 2*Phi
    move(motion, 0, -0.4)
    while not approx(pos.get()['yaw'],resultYaw):
        pass
    stop(motion)

# data from sensors
def getIr(ir):
    return ir.get()['range_list'][10]

def getProximity(prox):
    return prox.get()['near_objects']['target']

def getPose(pos):
    position = Position(pos.get()['x'], pos.get()['y'], pos.get()['yaw'])
    return position
