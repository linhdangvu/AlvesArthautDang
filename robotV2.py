# à lancer sur le robot
import serial
from time import sleep
import os
import signal, sys
from rplidar import RPLidar,RPLidarException
from time import sleep
from os import popen
from astar import runLidarWithRobotV2

LIDAR_PORT = '/dev/ttyUSB0'

#### comenter
#/dev/ttyACM0
robotPort = os.popen('ls /dev/ttyACM*').read().strip()
print("Found serial : " + robotPort)

def openSerial():
    return serial.Serial(robotPort)

# def sendInstruction(char, ser):
#     ser.write(char.encode())

ser = openSerial()

# instructionRobot(path, ser)
#### /comenter

while(True):
    print('starting...')
    sleep(1)
    try :
        lidar = RPLidar(LIDAR_PORT)

        point = []
        for i, scan in enumerate(lidar.iter_scans()):
            for qual, angle, dist in scan:
                # print("{};{}".format(angle, dist))
                point.append((angle, dist))
            break

        runLidarWithRobotV2(point, ser)

        # for (i, val) in enumerate(lidar.iter_measurments()):
        #     _, qual, angle, dist = val
        #     print(qual, angle, dist)
        #     break
        # break
    except KeyboardInterrupt:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        break
    except SystemExit:
        break
    except:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        try:
            print(str(exc_type) + '\t' + str(exc_value))
        except KeyboardInterrupt:
            lidar = RPLidar(LIDAR_PORT)
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()

# for i in range(10):
#     sleep(1/(i+1))
#     sendInstruction('z', ser)
#     # To do dans cette partie


ser.close()

