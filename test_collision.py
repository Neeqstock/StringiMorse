import sys

from libs.SimpleController import SimpleController

try:
    from pymorse import Morse
except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

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

    controller = SimpleController(pose, motion, prox, ir1, ir2, ir3, ir4)

    #controller.rotateInTheMoreConvenient(0)

    #while True:
    #    x = controller.getIrMedium(controller.getCIr())
    #    print(x)

    controller.alg_moveIntoBF()