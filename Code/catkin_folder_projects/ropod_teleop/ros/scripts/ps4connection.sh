#!/bin/bash

sudo ds4drv
sudo jstest /dev/input/js1
ls -l /dev/input/js1
sudo chmod a+rw /dev/input/js1
roscore
rosparam set joy_node/dev "/dev/input/js1"
