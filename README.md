# Robot Navigation Intention Communication
To visualize the movement and special actions of the ROPOD (RObotic POD). This project can easily be modifed for another robot. We use Adafruit classic playground board for this project. It is integrated with the ROBOT's OS using USB. Robot's navigation is sent to the Adafruite arduino board using serial communication (UART). Protocol design and detailed document can be found [presentation](documents/SDP_presentation.pdf) or in [docs](intention_communication/docs/).

## Project Directories
- [documents/](documents): Kickoff and final presentation of this project
- [intention_communication/](intention_communication/): ROS package that listens to robot's velocity topic, create message framework for Adafruit board and send message to Adafruite board using serial communication. 
- [progress/](progress/): Weekly progress report
- [utilities/](utilities/): Arduino C language code to parse the received message and display action. 

## Install

Installation of dependencies:
- Install ROS kinetic (follow [instructions](http://wiki.ros.org/kinetic/Installation))
- ropod_teleop
- ropod_ros_msgs
- ./install_software_cp_ps4
Note: install_software_cp_ps4 script is written for this specific project and specific robot. 

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

## Project Overview
<img src="https://github.com/priteshgohil/Robot-Navigation-Intention-Communication/blob/master/documents/Images/Overview.png" alt="Overview" width="600">
<img src="https://github.com/priteshgohil/Robot-Navigation-Intention-Communication/blob/master/documents/Images/Motion_identificatoin.png" alt="Motion Identificatoin" width="600">

## Demo
The robot is moving in a 2D plane therefore only x and y coordinates are considered. ROS velocity topic of the robot is subscribed and the navigation direction is calculated for LEDs in Adafruit Board.

<img src="https://github.com/priteshgohil/Robot-Navigation-Intention-Communication/blob/master/documents/Images/Motion1.png" alt="Intention Display1" width="600">
<img src="https://github.com/priteshgohil/Robot-Navigation-Intention-Communication/blob/master/documents/Images/Motion2.png" alt="Intention Display2" width="600">


## Authors
* Pritesh Gohil [@priteshgohil](https://github.com/priteshgohil)
* Eduardo Cervantes [@eduardcd](https://github.com/eduardcd)
