#!/usr/bin/env python3

from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM
from breezyslam.sensors import Laser
from mines import MinesLaser, Rover, load_data
from progressbar import ProgressBar
from pgm_utils import pgm_save

from sys import argv, exit, stdout
import sys
import time
import shutil, glob
import os
import itertools

import lcm
from lidarlcm import transmitter

import threading
from threading import Lock
from queue import LifoQueue

# Map size, scale
MAP_SIZE_PIXELS          = 800
MAP_SIZE_METERS          =  32


def my_handler(channel, data):
    msg = transmitter.decode(data)

    mutex.acquire()

    odometry = (msg.row[0], msg.row[1], msg.row[2])
    threadList = list(msg.row[24:-1])
    odoQ.put(odometry)
    lcmQ.put(threadList)

    mutex.release()

def stream():
    try:
        while(True):
            if(keyPressed != 's'):
                lc.handle()
            else:
                break
    except KeyboardInterrupt:
        pass

#Will terminate threads is character s is pressed in terminal
def stopdetect():
    global keyPressed
    keyPressed = input()

def main():

    # Bozo filter for input args
    if len(argv) < 3:
        print('Usage:   %s <dataset> <use_odometry> [random_seed]' % argv[0])
        print('Example: %s exp2 1 9999' % argv[0])
        exit(1)

    # Grab input args
    dataset = argv[1]
    use_odometry  =  True if int(argv[2]) else False
    seed =  int(argv[3]) if len(argv) > 3 else 0

    # Build a robot model if we want odometry
    robot = Rover() if use_odometry else None
    lidarobj = Laser(360, 12, 360, 8000)

    # Create a CoreSLAM object with laser params and robot object
    slam = RMHC_SLAM(lidarobj, MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=seed) \
           if seed \
           else Deterministic_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Start with an empty trajectory of positions
    trajectory = []
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    suffix = 1

    while(True):

        if(use_odometry):

            mutex.acquire()

            mainLCMQ = lcmQ
            mainODOQ = odoQ

            # Clear Queues once copied from thread into main for next batch of data
            lcmQ.queue.clear()
            odoQ.queue.clear()

            mutex.release()

            velocities = robot.computePoseChange(mainODOQ.get())
            slam.update(mainLCMQ.get(), velocities)
            x_mm, y_mm, theta_degrees = slam.getpos()

            x_pix = mm2pix(x_mm)
            y_pix = mm2pix(y_mm)

            trajectory.append((y_pix, x_pix))
            slam.getmap(mapbytes)

            trajLen = len(trajectory)

            for i in range(trajLen):
                if(i == (trajLen-1)):
                    mapbytes[trajectory[i][0]*MAP_SIZE_PIXELS + trajectory[i][1]] = 0
                else:
                    mapbytes[trajectory[i][0]*MAP_SIZE_PIXELS + trajectory[i][1]] = 120

            filename = dataset + str(suffix)
            pgm_save('%s.pgm' % filename, mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
            suffix += 1

            if(keyPressed == 's'):

                #Wrap up last map using leftover data
                pgm_save('%s.pgm' % filename, mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))

                '''
                This will take all the maps generated and place them into pgmbagfolder
                For this to work, make sure your destination directory has a folder called pgmbagfolder
                Change the directory:

                /home/Shaurya98/rplidar_workspace/src/mapping/BreezySLAM/examples
                and
                /home/Shaurya98/rplidar_workspace/src/mapping/BreezySLAM/examples/pgmbagfolder

                With your own destination directory. It it recommended to put pgmbagfolder under the examples
                directory
                '''

                os.chdir("/home/pi/rplidar_workspace/src/mapping/BreezySLAM/examples/pgmbagfolder")
                for pgm_file in glob.iglob('*.pgm'):
                    os.remove(pgm_file)
                print("\nEmptied pgmbagfolder")

                os.chdir("/home/pi/rplidar_workspace/src/mapping/BreezySLAM/examples")

                for pgm_file in glob.iglob('*.pgm'):
                    shutil.copy2(pgm_file, "/home/pi/rplidar_workspace/src/mapping/BreezySLAM/examples/pgmbagfolder")
                    os.remove(pgm_file)

                print("\nFiles recorded and sent to pgmbagfolder")

                #Terminate threads before exiting main()
                thread1.join()
                thread2.join()
                thread3.join()

                break

def mm2pix(mm):
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))

def animate():
    for c in itertools.cycle(['| | ', '/ | ', '- | ', '\\ | ']):
        if(keyPressed != 's'):
            sys.stdout.write('\rGenerating maps - press \'s\' and enter to exit ' + c)
            sys.stdout.flush()
            time.sleep(0.1)
        else:
            break

##############Script Starts Here##############
#For threading
keyPressed = ''
lcmQ = LifoQueue()
odoQ = LifoQueue()
mainLCMQ = LifoQueue()
mainODOQ = LifoQueue()

#For connecting to lidarscript.cpp
lc = lcm.LCM()
subscription = lc.subscribe("PIPE", my_handler)

#Thread obj
thread1 = threading.Thread(target=stream)
thread2 = threading.Thread(target=stopdetect)
thread3 = threading.Thread(target=animate)

#Thread locking obj
mutex = threading.Lock()

thread1.start()
thread2.start()
thread3.start()

#Give time for the LCM connection and to allow Queues to populate
time.sleep(2)

main()
