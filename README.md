# A-ROS-Driver-for-USB-COM
## Description
This is a ROS driver for serial port that is capable of reading, writing, and parsing messages from a USB serial port. 
I have implemented a message fragment merging function. With this driver, it is easy to connect various conversion modules, such as the 232_USB or CAN_USB modules.

## How to Use it
### 1. Install ROS Seriel and Granting permissions to a USB device.
    sudo apt install ros-noetic-serial
    ls /dev/ttyUSB*
    sudo chmod 777 /dev/ttyUSB0

### 2.Change the config file
change the config/systemConfig.yaml

### 3.Roslaunch
    roslaunch com_pkg COM.launch
