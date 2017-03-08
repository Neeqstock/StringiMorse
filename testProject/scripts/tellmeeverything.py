import sys
import time

try:
    from pymorse import Morse
except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

piGreco = 3.14159
piLower = 3.1
piHigher = 3.2

with Morse() as simu:

    # Definition of the sensors and the actuator
    pose = simu.robot.pose                  # pose sensor
    ir1 = simu.robot.ir1                    # ir sensor #1
    ir2 = simu.robot.ir2                    # ir sensor #2
    ir3 = simu.robot.ir3                    # ir sensor #3
    ir4 = simu.robot.ir4                    # ir sensor #4
    prox = simu.robot.prox                    # proximity sensor
    motion = simu.robot.motion              # motion speed actuator

    while(True):
        print(pose.get()['yaw'])
        time.sleep(0.4)