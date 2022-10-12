import signal, sys
from rplidar import RPLidar,RPLidarException
from time import sleep
# import csv
# import math
from os import popen
# import numpy as np
# import matplotlib.pyplot as plt

# from astar import displayMatrix, displayPath,verify, astar, convertToMatrix, displayCSV, dataLidar
from astar import astar, convertToMatrix, dataLidar


LIDAR_PORT = '/dev/ttyUSB0'

# pathLidar = []

while(True):
    print('starting...')
    sleep(2)
    try :
        lidar = RPLidar(LIDAR_PORT)

        point = []
        for i, scan in enumerate(lidar.iter_scans()):
            for qual, angle, dist in scan:
                # print("{};{}".format(angle, dist))
                point.append((angle, dist))
            break
        
        # transform Lidar to matrix and create A star
        pathLidar = dataLidar(point)
        # displayCSV(pathLidar)
        matrix0 = convertToMatrix(pathLidar)
        print(matrix0)
        start = (0, 5)
        end = (9, 5)
        path = astar(matrix0, start, end) ### import to robot
        # displayMatrix(matrix0)
        # displayPath(matrix0, path)
        # print(verify(matrix0, start, end, path))
        # for (i, val) in enumerate(lidar.iter_measurments()):
        #     _, qual, angle, dist = val
        #     print(qual, angle, dissqst)
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

