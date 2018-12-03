# rplidar_workspace

The rplidar_workspace repository contains source code for connecting your Raspberry Pi 3 with the RPLIDAR A2M8 Lidar and Zumo 32U4 robot for simulateneous localization and mapping, or SLAM. The repository makes use of BreezySLAM, an open source SLAM library and LCM, a set of specific libraries written for communication and marshalling.

**NOTE: Whenever connecting the A2M8 and/or 32U4, it is necessary to give the serial port adapter files read and write permissions**
```
sudo chmod 666 /dev/ttyUSB0 /dev/ttyACM0
```

## Setting up prerequisites

Clone this repository to your **/home** folder

If not done already, install the **libglib2.0-dev** library
```
sudo apt-get install libglib2.0-dev
```

#### LCM

Use this link to set up [lcm](http://lcm-proj.github.io/) on your Raspberry Pi 3

1. Go to **Downloads** and install **lcm-1.3.1.zip** file under **v1.3.1**
2. Go to your **/Downloads** directory in a terminal and enter the following
```
unzip lcm-1.3.1.zip
cd lcm-1.3.1
./configure
make
sudo make install
sudo ldconfig
cd lcm-python
sudo python3 setup.py install
```

**The lcm library should now be installed on your system**

#### BreezySLAM

The library is within the repository, but it must be installed on your system.

1. Go to the **rplidar_workspace/src/mapping/BreezySLAM/python/** directory in a terminal. There should be a **setup.py** file located in this directory. Run the python script:
```
sudo python3 setup.py install
```

## Getting Started
**NOTE: Along with giving the serial port adapter files read and write permission, you must also clear the current buffer in the serial port for the 32U4. There is a python script written in rplidar_workspace/src to help reset the buffer.**
```
python3 ~/rplidar_workspace/src/robotbufferreset.py
```
The script should output a stream of integer values. These values are compass and encoder values sent straight from the robot and are exactly the same values that are used throughout the repository. The script also contains the ability to control the robot which is used to confirm a successful open connection with the port. To control the robot, enter these characters into the terminal.
```
f -> forward
b -> backward
s -> stop
```
To exit the program, press ctrl+c to terminate the script.

1. In a terminal, go to your **rplidar_workspace/src/** directory and look for a file labelled **lidartomap.lcm**. Enter the following command to compile the file and auto-generate the appropriate folder:
```
lcm-gen -x lidartomap.lcm
lcm-gen -p lidartomap.lcm
```
There should now be a **lidarlcm** folder in the current directory. Copy this folder to the **rplidar_workspace/src/mapping/BreezySLAM/examples** folder

It is now time to turn get everything up and running

2. Open two terminals. The first terminal should be in the **rplidar_workspace/src/** directory, and the second should be in the **rplidar_workspace/src/mapping/BreezySLAM/examples/**. 
   - In the first terminal, type:
   ```
   ./compile.sh
   ./compile_and_execute.sh
   ```
   - In the second terminal, type:
   ```
   make pytest
   ```

**The output should be similar to this**

![](https://github.com/schan-2040/rplidar_workspace/blob/master/RevisedFinalPiSC.png)

**To exit, press ctrl+c on the first terminal and 's' on the second terminal. The A2M8 should stop spinning and the images generated will be relocated to the pgmbagfolder in rplidar_workspace/src/mapping/BreezySLAM/examples/**
