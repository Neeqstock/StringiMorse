#general utility
import time

from libs import Position
import math

from libs.Coordinates import Coordinates
from maths.triangl import findVertex

halfPhi = 1.57
Phi = 3.14
Pi = 3.1415

minBFr = 1.3
maxBFr = 1.9
midBFr = 1.65
correctProp = 1.4
rotBound = 1.5

class SimpleController:
    __pos = None
    __motion = None
    __prox = None
    __ir1 = None
    __ir2 = None
    __ir3 = None
    __ir4 = None

    __target = None

    __rotationSpeed = 0.4
    __motionSpeed = 1

    __triangDistance = 1.4

    def __init__(self, pos, motion, prox, ir1, ir2, ir3, ir4):
        self.__pos = pos
        self.__motion = motion
        self.__prox = prox
        self.__ir1 = ir1
        self.__ir2 = ir2
        self.__ir3 = ir3
        self.__ir4 = ir4

    # MATHS UTILITIES
    def approx(self, inputVal: float, testVal: float, approx: float = 0.03) -> bool:
        lowerBound = testVal - approx
        upperBound = testVal + approx
        if inputVal > lowerBound and inputVal < upperBound:
            return True
        else:
            return False

    def getDistance(self, Point1: Coordinates, Point2: Coordinates):
        x = Point2.x - Point1.x
        y = Point2.y - Point1.y
        return (x ** 2 + y ** 2) ** 0.5

    def evalBFrange(self, ir):  # Returns 0 if under, 1 if in, 2 if over
        if self.getIrMedium(ir) < maxBFr and self.getIrMedium(ir) > minBFr:
            return 1
        elif self.getIrMedium(ir) < minBFr:
            return 0
        elif self.getIrMedium(ir) > maxBFr:
            return 2

    def getBFerr(self, ir):
        return midBFr - self.getIrMinimum(ir)

    # SENSOR PRINT
    def print_pos(self):
        print("X = %s, Y = %s and yaw =%s" % (self.__pos.get()['x'], self.__pos.get()['y'], self.__pos.get()['yaw']))

        # only displaying useful information

    def print_position(self):
        print("X = %s, Y = %s and yaw =%s" % (self.__pos.x, self.__pos.y, self.__pos.yaw))

        # only displaying useful information

    def print_prox(self):
        print("target = %s" % self.__prox.get()['near_objects']['target'])

    def print_ir(self, ir):
        print("left = %s, center = %s, right = %s" % (ir.get()['range_list'][20], ir.get()['range_list'][10], ir.get()['range_list'][0]))
        # range 20 is the leftmost, 0 the rightmost

    def printAllIrsMinimum(self):
        print("Center IR: ", self.getMinimum(self.getCIr()))
        print("Left   IR: ", self.getMinimum(self.getLIr()))
        print("Right  IR: ", self.getMinimum(self.getRIr()))
        print("Back   IR: ", self.getMinimum(self.getBIr()))
        print("-------------------")

    # GET SENSORS
    def getCIr(self):
        return self.__ir1
    def getLIr(self):
        return self.__ir2
    def getRIr(self):
        return self.__ir3
    def getBIr(self):
        return self.__ir4

    def getCmin(self):
        self.getMinimum(self.getCIr())

    def getLmin(self):
        self.getMinimum(self.getLIr())

    def getRmin(self):
        self.getMinimum(self.getRIr())

    def getBmin(self):
        self.getMinimum(self.getCIr())

    def getProximity(self, prox):
        return prox.get()['near_objects']['target']

    def getPosition(self):
        position = Position(self.__pos.get()['x'], self.__pos.get()['y'], self.__pos.get()['yaw'])
        return position

    # SENSORS ELABORATION
    def getIrMedium(self, ir):
        return ir.get()['range_list'][10]

    def getIrMinimum(self, ir):
        rangeList = ir.get()['range_list']
        min = rangeList[0]
        for value in rangeList:
            if value < min:
                min = value
        return float(min)

    # BASIC MOTION
    def stopWhenReached(self, coordinates: Position, approximation: float = 0.5):
        while(not self.approx(self.__pos.get()['x'], coordinates.getX(), approximation)) and (not self.approx(self.__pos.get()['y'], coordinates.getY(), approximation)):
            pass
        self.stop()

    def move(self, v, w=0.0):
        self.__motion.publish({"v": v, "w": w})

    def moveFWD(self):
        self.__motion.publish({"v": self.__motionSpeed, "w": 0})

    def moveBWD(self):
        self.__motion.publish({"v": -self.__motionSpeed, "w": 0})

    def stop(self):
        self.__motion.publish({"v": 0, "w": 0})

    # BASIC ROTATION
    def rotateCW(self):
        self.__motion.publish({"v": 0, "w": -self.__rotationSpeed})

    def rotateCCW(self):
        self.__motion.publish({"v": 0, "w": self.__rotationSpeed})

    def rotate90Right(self):
        self.stop()
        yaw = self.__pos.get()['yaw']
        resultYaw = yaw - halfPhi
        if resultYaw < - Phi:
            resultYaw = resultYaw + 2*Phi
        self.move(0, - self.__rotationSpeed)
        while not self.approx(self.__pos.get()['yaw'], resultYaw):
            pass
        self.stop()

    def rotate90Left(self):
        yaw = self.__pos.get()['yaw']
        resultyaw = yaw + halfPhi
        if resultyaw > Phi:
            resultyaw = resultyaw - 2*Phi
        self.move(0, self.__rotationSpeed)
        while not self.approx(self.__pos.get()['yaw'], resultyaw):
            pass
        self.stop()

    def rotate180(self):
        self.stop()
        yaw = self.__pos.get()['yaw']
        resultYaw = yaw - Phi
        if resultYaw < - Phi:
            resultYaw = resultYaw + 2*Phi
            self.move(0, -self.__rotationSpeed)
        while not self.approx(self.__pos.get()['yaw'], resultYaw):
            pass
        self.stop()

    def rotateTowardsPoint(self, coordinates: Coordinates):
        xCat = coordinates.x - self.__pos.get()['x']
        selfPosition = Coordinates(self.__pos.get()['x'], self.__pos.get()['y'])
        Hyp = self.getDistance(selfPosition, coordinates)
        angle = math.acos(xCat/Hyp)
        if (coordinates.y - selfPosition.y) < 0:
            angle = -angle
        self.rotateInTheMoreConvenient(angle)

    def rotateInTheMoreConvenient(self, finalYaw):
        actualYaw = self.__pos.get()['yaw']
        diff = actualYaw - finalYaw

        if diff < -Pi or (diff > 0 and diff < Pi):

            self.rotateCW()
        else:

            self.rotateCCW()
        while not self.approx(self.__pos.get()['yaw'], finalYaw):
            pass
        self.stop()

    def rotateTowardsTarget(self):
        if self.__target is None:
            print("No target has been calculated yet! You first need to triangulate it.")
        else:
            self.rotateTowardsPoint(self.__target)

    # TARGET LOCATION
    def triangulateTarget(self) -> Coordinates:
        cPose = self.__pos.get()
        cDist = self.__prox.get()['near_objects']['target']

        # Turn the robot up
        self.rotateInTheMoreConvenient(Pi/2)

        # Reach PY
        self.moveFWD()
        time.sleep(self.__triangDistance)
        self.stop()

        yPose = self.__pos.get()
        yDist = self.__prox.get()['near_objects']['target']

        # Get back to PC
        self.moveBWD()
        time.sleep(self.__triangDistance)
        self.stop()

        # Reach PX
        self.rotateInTheMoreConvenient(0)
        self.moveFWD()
        time.sleep(self.__triangDistance)
        self.stop()

        xPose = self.__pos.get()
        xDist = self.__prox.get()['near_objects']['target']

        # Damn calculations
        xPoint = (xPose['x'], xPose['y'])
        yPoint = (yPose['x'], yPose['y'])
        cPoint = (cPose['x'], cPose['y'])
        side = abs(xPoint[0] - cPoint[0])

        targetCoordinates = findVertex(cPoint, cDist, xDist, yDist, side)

        print("xPoint: ", xPoint[0], xPoint[1])
        print("yPoint: ", yPoint[0], yPoint[1])
        print("cPoint: ", cPoint[0], cPoint[1])
        print("oldTrilaterator: ", targetCoordinates)

        self.__target = Coordinates(targetCoordinates[0], targetCoordinates[1])

        return self.__target

    def getTarget(self) -> Coordinates:
        if self.__target is None:
            print("No target has been calculated yet! You first need to triangulate it.")
            return None
        else:
            return self.__target

    # COMPLEX MOTION: BOUNDARY FOLLOWING

    def alg_moveIntoBF(self): # Move until reached an obstacle and then go into BF mode
        self.moveUntilObstacleInRange()
        if self.evalBFrange(self.getLIr()) == 1:
            self.unlimLeftBoundaryFollow()
        elif self.evalBFrange(self.getRIr()) == 1:
            self.leftBoundaryFollow()

    def leftBoundaryFollow(self):
        while True:
            if self.evalBFrange(self.getLIr()) == 1:
                err = self.getBFerr(self.getLIr())
                self.move(0.7, -err * correctProp)
            elif self.evalBFrange(self.getLIr()) == 0:
                while self.getIrMedium(self.getLIr()) < midBFr:
                    self.rotateCW()
            elif self.evalBFrange(self.getLIr()) == 2:
                while self.getIrMedium(self.getLIr()) > midBFr:
                    self.rotateCCW()

    def unlimLeftBoundaryFollow(self):
        while True: # exit condition here! TODO
            if self.evalBFrange(self.getCIr()) == 1 or self.evalBFrange(self.getCIr()) == 0:
                while self.evalBFrange(self.getCIr()) == 1 or self.evalBFrange(self.getCIr()) == 0:
                    self.move(0.2, -0.6)
            else:
                if self.evalBFrange(self.getLIr()) == 0:
                    while self.getIrMedium(self.getLIr()) < midBFr:
                        self.rotateCW()
                elif self.evalBFrange(self.getLIr()) == 1:
                    err = self.getBFerr(self.getLIr())
                    if err > 1: err = rotBound
                    if err < -1: err = -rotBound
                    self.move(0.7, -err * correctProp)
                elif self.evalBFrange(self.getLIr()) == 2:
                    err = self.getBFerr(self.getLIr())
                    if err > 1: err = 1
                    if err < -1: err = 1
                    self.move(0.5, -err*0.8)

    def moveUntilObstacleInRange(self):
        while self.evalBFrange(self.getLIr()) != 1 and self.evalBFrange(self.getRIr()) != 1:
            self.moveFWD()



