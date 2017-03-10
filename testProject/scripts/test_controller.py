import sys

import time

from libs.SimpleController import SimpleController

try:
    from pymorse import Morse
except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

piGreco = 3.14159

with Morse() as simu:

    # Definition of the sensors and the actuator
    pose = simu.robot.pose                  # pose sensor
    ir1 = simu.robot.ir1                    # ir sensor #1
    ir2 = simu.robot.ir2                    # ir sensor #2
    ir3 = simu.robot.ir3                    # ir sensor #3
    ir4 = simu.robot.ir4                    # ir sensor #4
    prox = simu.robot.prox                  # proximity sensor
    motion = simu.robot.motion              # motion speed actuator

    controller = SimpleController(pose, motion, prox, ir1, ir2, ir3, ir4)

    # VARIOUS TESTS
    #controller.rotateInTheMoreConvenient(-1)
    #time.sleep(1)
    # controller.triangulateTarget()
    # controller.moveBWD()
    # time.sleep(1)
    # controller.stop()
    # print("Target position: ", controller.getTarget().x, controller.getTarget().y)
    # time.sleep(1)
    # controller.rotateTowardsTarget()

    # while True:
    #     time.sleep(1)
    #     controller.printAllIrsMinimum()

    controller.alg_moveIntoBF()