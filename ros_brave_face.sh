#!/bin/bash
#--------------------------------------------------------------------
#バックグラウンド実行用のスクリプト
#--------------------------------------------------------------------

#変数の設定
SCRIPTDIR=/home/ubuntu/catkin_ws/src/ros_brave_face/
LOGDIR=$SCRIPTDIR/log
ENVFILE=/home/ubuntu/catkin_ws/devel/setup.bash
ENVFILE_ROS=/opt/ros/melodic/setup.bash

#FTDI Timer Setting 1msec
sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

#実行
if [ -f ${ENVFILE} ]; then
    #環境変数読み込み
    echo "Loading ROS Env..."
    source $ENVFILE_ROS
    source $ENVFILE
    if [ -d ${LOGDIR} ]; then
        echo "ROS Launching..."
        #roslaunch実行
        exec roslaunch ros_brave_face face_brave.launch  >> ${LOGDIR}/ros_brave_face.log 2>&1
    else
        echo "There is no ${LOGDIR}"
    fi
else
    echo "There is no ${ENVFILE}"
fi
