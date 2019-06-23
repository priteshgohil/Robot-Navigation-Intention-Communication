# sdp_ss2019_P6_IntentionComm
Repository containing the install executable file with the structure of the whole software. Specifically for the project Intention Communication

## Install

Installation of dependencies:
- Install ROS kinetic (follow [instructions](http://wiki.ros.org/kinetic/Installation))
- ropod_teleop
- ropod_ros_msgs
- ./install_software_cp_ps4

This command will install the proper dependencies and will create a folder in your home folder ~/workspace_SPD_intention_comm
in such folder the required packages will be builded.
Additionaly the proper udev rule for a connection with the Circuit Playground boaard will be created inside your folder /etc/udev/rules.d/

The Circuit Playground board should be connected to any available usb port 
The program code to be flashed in the circuit playground can be find inside /utilities/Arduino/arduinointerface

In order to make a connection to the ps4 controller you may install the repository ds4drv

```
cd Downloads && sudo apt install git -y && git clone https://github.com/chrippa/ds4drv.git && cd ds4drv && sudo python setup.py install
```

In case the ds4drv is already installed you may require to run:
```
sudo ds4rdv
```

As a final step under the folder /utilities it is available a script to connect to the ps4 controller and use it to send omnidirectional commands
```
cd /utilities && ./Connect_to_ps4_joypad
```
