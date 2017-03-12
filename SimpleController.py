#general utility
import time

from libs import Position
import math

from libs.Coordinates import Coordinates
from maths.triangl import findVertex
from maths.triangl import findVertex2

halfPhi = 1.57
Phi = 3.14
Pi = 3.1415

# secure distance from wall when turning on itself (Left, medium) =1.2563
# secure distance from wall when turning on itself (right, medium) = 1.2414
# secure distance from wall when turning on itself (center, medium) = 0.7755

strettoiaMode = 1.5

minBFr = 1.5
maxBFr = 1.8        # tested: 1.9
midBFr = 1.65
correctProp = 0.9
rotBound = 1
centerSecure = 1.8  # tested: 1.8 # security distance from which the robot starts turning if there's an obstacle in front of him
arrivalApprox = 1 # defines the radius in which the robot considers to be arrived to the target
angleApprox = 0.1   # approximation for isInRect

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

    __insertionAngle = None     # Condizione per verificare se mi trovo sulla retta
    __hitPoint = None
    __iAmInHitPoint = False
    __hitPointDistance = None
    __hitPointRadius = 1.5      # ball radius of the hitpoint

    __strettoia = 0             # ocio alle strettoie!
    __strettoiaLimit = 0.3
    __troppoStretto = 0.6

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

    def getRelativeAngles(self, coordinates: Coordinates) -> float(2):
        xCat = coordinates.x - self.__pos.get()['x']
        selfPosition = Coordinates(self.__pos.get()['x'], self.__pos.get()['y'])
        Hyp = self.getDistance(selfPosition, coordinates)
        try:
            angle = math.acos(xCat / Hyp)
        except:
            angle = 4
        if (coordinates.y - selfPosition.y) < 0:
            angle = -angle
        if angle > 0:
            return (angle - Pi, angle)
        else:
            return (angle, angle + Pi)

    def evalBFrange(self, ir):  # Returns 0 if under, 1 if in, 2 if over
        if self.getIrMedium(ir) < maxBFr - self.__strettoia and self.getIrMedium(ir) > minBFr - self.__strettoia:
            return 1
        elif self.getIrMedium(ir) < minBFr - self.__strettoia:
            return 0
        elif self.getIrMedium(ir) > maxBFr - self.__strettoia:
            return 2

    def getBFerr(self, ir):
        return midBFr - self.__strettoia - self.getIrMinimum(ir)

    # SENSOR PRINT
    def print_pos(self):
        print("X = %s, Y = %s and yaw =%s" % (self.__pos.get()['x'], self.__pos.get()['y'], self.__pos.get()['yaw']))

        # only displaying useful information

    def print_position(self):
        print("X = %s, Y = %s and yaw =%s" % (self.__pos.get()['x'], self.__pos.get()['y'], self.__pos.get()['yaw']))

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

    def getCL(self):
        return self.__ir1.get()['range_list'][0]

    def getCR(self):
        return self.__ir1.get()['range_list'][20]

    def getProximity(self, prox):
        return prox.get()['near_objects']['target']

    def getPosition(self):
        position = Position(self.__pos.get()['x'], self.__pos.get()['y'], self.__pos.get()['yaw'])
        return position

    def getPositionCoordinates(self) -> Coordinates:
        position = Coordinates(self.__pos.get()['x'], self.__pos.get()['y'])
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

    def isInRect(self) -> bool:
        angles = self.getRelativeAngles(self.__target)
        if (self.approx(self.__insertionAngle, angles[0], angleApprox)) or (self.approx(self.__insertionAngle, angles[1], angleApprox)):
            return True
        return False

    def amIArrived(self) -> bool:
        position = self.getPositionCoordinates()
        if (self.getDistance(position, self.__target) < arrivalApprox):
            return True
        else:
            return False

    # BASIC MOTION
    def stopWhenReached(self, coordinates: Position, approximation: float = 0.5):
        while(not self.approx(self.__pos.get()['x'], coordinates.getX(), approximation)) and (not self.approx(self.__pos.get()['y'], coordinates.getY(), approximation)):
            pass
        self.stop()

    def move(self, v, w=0.0):
        self.__motion.publish({"v": v, "w": w})

    def moveFWD(self, speed = __motionSpeed):
        self.__motion.publish({"v": speed, "w": 0})

    def moveBWD(self):
        self.__motion.publish({"v": -self.__motionSpeed, "w": 0})

    def stop(self):
        self.__motion.publish({"v": 0, "w": 0})

    # BASIC ROTATION
    def rotateCW(self, w = __rotationSpeed):
        self.__motion.publish({"v": 0, "w": -w})

    def rotateCCW(self, w = __rotationSpeed):
        self.__motion.publish({"v": 0, "w": w})

    def rotateInTheMoreConvenient(self, finalYaw, w=__rotationSpeed, approx=0.03):
        actualYaw = self.__pos.get()['yaw']
        diff = actualYaw - finalYaw
        if diff < -Pi or (diff > 0 and diff < Pi):
            self.rotateCW(w)
        else:
            self.rotateCCW(w)
        while not self.approx(self.__pos.get()['yaw'], finalYaw, approx):
            pass
        self.stop()

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
            self.move(0, -1)
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

    def rotateTowardsTarget(self):
        if self.__target is None:
            print("No target has been calculated yet! You first need to triangulate it.")
        else:
            self.rotateTowardsPoint(self.__target)

    # TARGET LOCATION
    def triangulateTarget(self) -> Coordinates:
        self.checkTrilaterationSpace()
        cPose = self.__pos.get()
        cDist = self.__prox.get()['near_objects']['target']
        print("cx = %s cy = %s" % (cPose['x'], cPose['y']))

        # Turn the robot up
        self.rotateInTheMoreConvenient(Pi/2, 0.1, 0.005)

        # Reach PY
        self.moveFWD()
        time.sleep(self.__triangDistance)
        self.stop()

        yPose = self.__pos.get()
        yDist = self.__prox.get()['near_objects']['target']
        print("Yx = %s Yy = %s" % (yPose['x'], yPose['y']))

        # Get back to PC
        self.moveBWD()
        time.sleep(self.__triangDistance)
        self.stop()

        # Reach PX
        self.rotateInTheMoreConvenient(0, 0.1, 0.005)
        self.moveFWD()

        time.sleep(self.__triangDistance)
        self.stop()

        xPose = self.__pos.get()
        xDist = self.__prox.get()['near_objects']['target']
        print("Xx = %s Xy = %s" % (xPose['x'], xPose['y']))

        # Damn calculations
        xPoint = (xPose['x'], xPose['y'])
        yPoint = (yPose['x'], yPose['y'])
        cPoint = (cPose['x'], cPose['y'])
        side = abs(xPoint[0] - cPoint[0])

        targetCoordinates = findVertex(cPoint, cDist, xDist, yDist, side)
        self.__target = Coordinates(targetCoordinates[0], targetCoordinates[1])
        print("Tx = %s Ty = %s" % (targetCoordinates[0], targetCoordinates[1]))

        self.moveBWD()
        time.sleep(self.__triangDistance)
        self.stop()
        return self.__target

    def triangulateTarget2(self) -> Coordinates:
        self.checkTrilaterationSpace()
        cPose = self.__pos.get()
        cDist = self.__prox.get()['near_objects']['target']
        print("cx = %s cy = %s" % (cPose['x'], cPose['y']))

        # Turn the robot up
        self.rotateInTheMoreConvenient(Pi / 2, 0.1, 0.005)

        # Reach PY
        self.moveFWD()
        time.sleep(self.__triangDistance)
        self.stop()

        yPose = self.__pos.get()
        yDist = self.__prox.get()['near_objects']['target']
        print("Yx = %s Yy = %s" % (yPose['x'], yPose['y']))

        # Get back to PC
        self.moveBWD()
        time.sleep(self.__triangDistance)
        self.stop()

        # Reach PX
        self.rotateInTheMoreConvenient(0, 0.1, 0.005)
        self.moveFWD()

        time.sleep(self.__triangDistance)
        self.stop()

        xPose = self.__pos.get()
        xDist = self.__prox.get()['near_objects']['target']
        print("Xx = %s Xy = %s" % (xPose['x'], xPose['y']))

        # Damn calculations
        xPoint = (xPose['x'], xPose['y'])
        yPoint = (yPose['x'], yPose['y'])
        cPoint = (cPose['x'], cPose['y'])
        side = abs(xPoint[0] - cPoint[0])

        targetCoordinates = findVertex2(cPoint, yPoint,xPoint, cDist, xDist, yDist, side)
        self.__target = Coordinates(targetCoordinates[0], targetCoordinates[1])
        print("Tx = %s Ty = %s" % (targetCoordinates[0], targetCoordinates[1]))

        self.moveBWD()
        time.sleep(self.__triangDistance)
        self.stop()
        return self.__target

    def goal(self):
        print(time.time(), "Goal!!! I am arrived.")
        self.stop()
        while True:
            pass

    # Nuove di Tom
    def evalIrRange(self, ir, range: float):  # Returns 0 if no obj, 1 if obj
        if self.getIrMedium(ir) < range:
            return 1
        else:
            return 0

    def checkTrilaterationSpace(self):
        self.rotateInTheMoreConvenient(Pi / 2, w=0.4)
        self.checkFrontalSpace()
        self.rotateInTheMoreConvenient(0, w=0.4)
        self.checkFrontalSpace()

    def checkFrontalSpace(self):
        if self.evalIrRange(self.getCIr(), 2) == 1:
            print("no space front")
            enoughSpace = self.getIrMedium(self.getBIr()) - self.getIrMedium(self.getCIr())
            if enoughSpace > 0:
                self.moveBWD()
                time.sleep(enoughSpace)
                self.stop()
                return 0
            else:
                print("can't trilaterate")
                return 1

    def getTarget(self) -> Coordinates:
        if self.__target is None:
            print("No target has been calculated yet! You first need to triangulate it.")
            return None
        else:
            return self.__target

    # COMPLEX MOTION: BOUNDARY FOLLOWING
    def alg_moveIntoBF(self): # Move until reached an obstacle and then go into BF mode
        while True:
            # self.moveUntilObstacleInRange()
            if (self.__ir1.get()['range_list'][20] < maxBFr) or (self.getIrMedium(self.__ir2) < 2):
                print("left BF!")
                self.leftBoundaryFollow()
            elif (self.__ir1.get()['range_list'][0] < maxBFr) or (self.getIrMedium(self.__ir3) < 2):
                print("right BF!")
                self.rightBoundaryFollow()
            self.moveFWD()

    def old_LeftBoundaryFollow(self):
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

    def leftBoundaryFollow(self):
        while True: # exit condition here! TODO
            if self.getIrMedium(self.getCIr()) < centerSecure:
                print("if")
                while self.getIrMedium(self.getCIr()) < centerSecure:
                    self.move(0.1, -0.6)
            else:
                print("else")
                if self.evalBFrange(self.getLIr()) == 0:
                    while self.getIrMedium(self.getLIr()) < midBFr:
                        self.rotateCW()
                elif self.evalBFrange(self.getLIr()) == 1:
                    err = self.getBFerr(self.getLIr())
                    if err > 1: err = rotBound
                    if err < -1: err = -rotBound
                    self.move(1-abs(err)*1.6, -err * correctProp)
                elif self.evalBFrange(self.getLIr()) == 2:
                    err = self.getBFerr(self.getLIr())
                    if err > 1: err = 1
                    if err < -1: err = 1
                    self.move(0.5, -err*0.8)

    def rightBoundaryFollow(self):
        while True: # exit condition here! TODO
            if self.getIrMedium(self.getCIr()) < centerSecure:
                print("if")
                while self.getIrMedium(self.getCIr()) < centerSecure:
                    self.move(0.1, 0.6)
            else:
                print("else")
                if self.evalBFrange(self.getRIr()) == 0:
                    while self.getIrMedium(self.getRIr()) < midBFr:
                        self.rotateCCW()
                elif self.evalBFrange(self.getRIr()) == 1:
                    err = self.getBFerr(self.getRIr())
                    if err > 1: err = rotBound
                    if err < -1: err = -rotBound
                    self.move(1-abs(err)*1.6, err * correctProp)
                elif self.evalBFrange(self.getRIr()) == 2:
                    err = self.getBFerr(self.getRIr())
                    if err > 1: err = 1
                    if err < -1: err = 1
                    self.move(0.5, err*0.8)

    def moveUntilObstacleInRange(self):                 # DEPRECATA!
        while self.evalBFrange(self.getLIr()) != 1 and self.evalBFrange(self.getRIr()) != 1:
            self.moveFWD()

    def alg_bug2(self): # Move until reached an obstacle and then go into BF mode
        print(time.time(), ": triangulating target.")
        self.triangulateTarget2()                        # 1: TRIANGOLA!
        print(time.time(), ": target triangulated. Target pos = ", self.__target.x, self.__target.y)

        self.rotateTowardsTarget()                      # GIRATI VERSO L'OBBIETTIVO
        print(time.time(), ": i'm now directed towards target.")

        self.__insertionAngle = self.__pos.get()['yaw'] # REGISTRA L'INSERTION ANGLE
        print(time.time(), ": the insertion angle is ", self.__insertionAngle)

        while True:
            if self.amIArrived():                 # 2: VERIFICO SE SONO ARRIVATO...
                self.goal()
                                                        # VERIFICO SE ENTRARE IN BF
            if self.__ir1.get()['range_list'][10] < maxBFr:             # center hit!
                if self.__ir1.get()['range_list'][20] < self.__ir1.get()['range_list'][0]:
                    print(time.time(), ": obstacle, center hit! Entering LEFT boundary following.")
                    self.leftBoundaryFollowWithCondition()
                else:
                    print(time.time(), ": obstacle, center hit! Entering RIGHT boundary following.")
                    self.rightBoundaryFollowWithCondition()
            elif(self.__ir1.get()['range_list'][20] < maxBFr) or (self.getIrMedium(self.__ir2) < 2): # left hit!
                print(time.time(), ": obstacle, left hit! Entering LEFT boundary following.")
                self.leftBoundaryFollowWithCondition()

            elif (self.__ir1.get()['range_list'][0] < maxBFr) or (self.getIrMedium(self.__ir3) < 2): # right hit
                print(time.time(), ": obstacle, right hit! Entering RIGHT boundary following.")
                self.rightBoundaryFollowWithCondition()
            else:
                self.moveFWD(0.6)                              # ALTRIMENTI CONTINUO AD ANDARE...

        print(time.time(), ": algorithm stopped. I hope I just reached the target! :)")

    def leftBoundaryFollowWithCondition(self):
        # REGISTRO L'HITPOINT
        self.__hitPoint = self.getPositionCoordinates()
        self.__iAmInHitPoint = True
        self.__hitPointDistance = self.__prox.get()['near_objects']['target']
        print(time.time(), ": new hitpoint registered x = ", self.__hitPoint.x," y = ", self.__hitPoint.y, " distance = ", self.__hitPointDistance)

        while True:
            if self.amIArrived():
                self.goal()
            # CONDIZIONE DI USCITA
            if (self.__iAmInHitPoint == False) and (self.isInRect()):  # CHE CONDIZIONE DI USCITA!
                if (self.__prox.get()['near_objects']['target'] < self.__hitPointDistance):
                    break
                print(time.time(), " i was on the rect, but i was more far than the hitpoint.")

            # VERIFICONE PER DICHIARARE DI ESSERMI ALLONTANATO DALL'HITPOINT
            if self.__iAmInHitPoint == True:
                if (self.getDistance(self.getPositionCoordinates(), self.__hitPoint)) > self.__hitPointRadius:
                    self.__iAmInHitPoint = False
                    print("Just got out of the hitpoint...")

            if self.getIrMedium(self.getCIr()) < centerSecure:  # centrosbatt mode
                print("centrosbatt mode")
                self.move(0, -0.4)
            elif self.getIrMedium(self.getRIr()) < strettoiaMode:   # strettoia mode
                if (self.getIrMedium(self.getLIr()) < self.__troppoStretto) and (self.getIrMedium(self.getRIr()) < self.__troppoStretto):
                    print("azz, tropp strett! merd")
                    self.rotate180()
                err = self.getBFerr(self.getLIr()) - (2 - self.getIrMedium(self.getRIr()))
                print("strettoia")
                if err > rotBound: err = rotBound
                if err < -rotBound: err = -rotBound
                self.move(1 - abs(err)*0.5, -err * correctProp)
            elif self.evalBFrange(self.getLIr()) == 0:
                print("trop vicin")
                while self.getIrMedium(self.getLIr()) < midBFr:
                    self.rotateCW(0.4)
            elif self.evalBFrange(self.getLIr()) == 1:
                err = self.getBFerr(self.getLIr())
                if err > rotBound: err = rotBound
                if err < -rotBound: err = -rotBound
                self.move(1 - abs(err) * 1.2, -err * correctProp)
            elif self.evalBFrange(self.getLIr()) == 2:
                err = self.getBFerr(self.getLIr())
                print("trop lontan")
                if err > 1: err = 1
                if err < -1: err = 1
                self.move(0.5, -err * 0.9)

        self.rotateTowardsTarget()  # EXIT FROM BF: ROTATE TOWARDS TARGET

    def rightBoundaryFollowWithCondition(self):
        # REGISTRO L'HITPOINT
        self.__hitPoint = self.getPositionCoordinates()
        self.__iAmInHitPoint = True
        self.__hitPointDistance = self.__prox.get()['near_objects']['target']
        print(time.time(), ": new hitpoint registered x = ", self.__hitPoint.x," y = ", self.__hitPoint.y, " distance = ", self.__hitPointDistance)

        while True:
            if self.amIArrived():
                self.goal()
            # CONDIZIONE DI USCITA
            if (self.__iAmInHitPoint == False) and (self.isInRect()):  # CHE CONDIZIONE DI USCITA!
                if (self.__prox.get()['near_objects']['target'] < self.__hitPointDistance):
                    break
                print(time.time(), " i was on the rect, but i was more far than the hitpoint.")

            # VERIFICONE PER DICHIARARE DI ESSERMI ALLONTANATO DALL'HITPOINT
            if (self.__iAmInHitPoint == True):
                if (self.getDistance(self.getPositionCoordinates(), self.__hitPoint)) > self.__hitPointRadius:
                    self.__iAmInHitPoint = False
                    print("Just got out of the hitpoint...")

            if self.getIrMedium(self.getCIr()) < centerSecure: # centrosbatt mode
                print("centrosbatt mode")
                self.move(0, 0.4)
            elif self.getIrMedium(self.getLIr()) < strettoiaMode:   # strettoia mode
                if (self.getIrMedium(self.getLIr()) < self.__troppoStretto) and (self.getIrMedium(self.getRIr()) < self.__troppoStretto):
                    print("azz, tropp strett! merd")
                    self.rotate180()
                err = self.getBFerr(self.getRIr()) - (2 - self.getIrMedium(self.getLIr()))
                print("strettoia")
                if err > rotBound: err = rotBound
                if err < -rotBound: err = -rotBound
                self.move(1 - abs(err)*0.5, err * correctProp)
            elif self.evalBFrange(self.getRIr()) == 0:
                print("trop vicin")
                while self.getIrMedium(self.getRIr()) < midBFr:
                    self.rotateCCW(0.4)
            elif self.evalBFrange(self.getRIr()) == 1:
                err = self.getBFerr(self.getRIr())
                if err > rotBound: err = rotBound
                if err < -rotBound: err = -rotBound
                self.move(1 - abs(err) * 1.2, err * correctProp)
            elif self.evalBFrange(self.getRIr()) == 2:
                print("trop lontan")
                err = self.getBFerr(self.getRIr())
                if err > 1: err = 1
                if err < -1: err = 1
                self.move(0.5, err * 0.9)

        self.rotateTowardsTarget()  # EXIT FROM BF: ROTATE TOWARDS TARGET



