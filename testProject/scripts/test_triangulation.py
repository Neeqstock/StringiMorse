import sys

from libs.SimpleController import SimpleController

try:
    from pymorse import Morse
except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

#triangDistance = 1.4
#rotationSpeed = 0.25

#piGreco = 3.14159
#piLower = 3.1
#piHigher = 3.2

#radUp = 1.57
#radDown = -1.57
#radRight = 0
#radLeft = 3.14

with Morse() as simu:

    # Definition of the sensors and the actuator
    pose = simu.robot.pose                  # pose sensor
    ir1 = simu.robot.ir1                    # ir sensor #1
    ir2 = simu.robot.ir2                    # ir sensor #2
    ir3 = simu.robot.ir3                    # ir sensor #3
    ir4 = simu.robot.ir4                    # ir sensor #4
    prox = simu.robot.prox                  # proximity sensor
    motion = simu.robot.motion              # motion speed actuator

    # Get sensor measurements and control the actuator
    pose.get()                              # get from pose sensor
    ir1.get()                               # get from ir sensor #1
    ir2.get()                               # get from ir sensor #2
    ir3.get()                               # get from ir sensor #3
    ir4.get()                               # get from ir sensor #4
    prox.get()                              # get from proximity sensor
    motion.publish({"v": 0, "w": 0})        # set robot linear and angular speed

    #cPose = pose.get()
    #cDist = prox.get()['near_objects']['target']
    #
    ## Turn the robot up
    #while (not approx(pose.get()['yaw'],radUp)):
    #    motion.publish({"v": 0, "w": rotationSpeed})
    #motion.publish({"v": 0, "w": 0})
    #
    ## Reach PY
    #motion.publish({"v": 1, "w": 0})
    #time.sleep(triangDistance)
    #motion.publish({"v": 0, "w": 0})
    #
    #yPose = pose.get()
    #yDist = prox.get()['near_objects']['target']
    #
    ## Get back to PC
    #motion.publish({"v": -1, "w": 0})
    #time.sleep(triangDistance)
    #motion.publish({"v": 0, "w": 0})
    #
    ## Reach PX
    #while (not approx(pose.get()['yaw'],radRight)):
    #    motion.publish({"v": 0, "w": -rotationSpeed})
    #motion.publish({"v": 1, "w": 0})
    #time.sleep(triangDistance)
    #motion.publish({"v": 0, "w": 0})
    #
    #xPose = pose.get()
    #xDist = prox.get()['near_objects']['target']
    #
    ## Damn calculations
    #xPoint = (xPose['x'], xPose['y'])
    #yPoint = (yPose['x'], yPose['y'])
    #cPoint = (cPose['x'], cPose['y'])
    #side = abs(xPoint[0] - cPoint[0])
    #
    #targetCoordinates = findVertex(cPoint, cDist, xDist, yDist, side)
    #targetNewCoordinates = triLaterate(cPoint, xPoint, yPoint, cDist, xDist, yDist)
    #targetAgainNew = triBoh(cPoint, xPoint, yPoint, cDist, xDist, yDist)
    #targetArgh = triArgh(xPoint, yPoint, cDist, xDist, yDist)
    #
    #print("xPoint: ", xPoint[0], xPoint[1])
    #print("yPoint: ", yPoint[0], yPoint[1])
    #print("cPoint: ", cPoint[0], cPoint[1])
    #print("oldTrilaterator: ", targetCoordinates)
    #print("newTrilaterator: ", targetNewCoordinates)
    #print("letsHope: ", targetAgainNew)
    #print("...: ", targetArgh)
    #
    controller = SimpleController(pose, motion, prox, ir1, ir2, ir3, ir4)

    #controller.triangulateTarget()
    controller.triangulateTarget2()