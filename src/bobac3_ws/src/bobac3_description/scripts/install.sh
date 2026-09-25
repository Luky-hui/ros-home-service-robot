#!/bin/bash
UBUNTU_VERSION="$(lsb_release -sr)"
GAZEBO_VERSION="$(gazebo -v|grep version)"
UBUNTU18="18"
GAZEBO9="9"
UBUNTU20="20"
GAZEBO11="11"
TARGET_UBUNTU_VER=0
TARGET_GAZEBO_VER=0
if [[ $UBUNTU_VERSION == *"$UBUNTU18"* ]]; then
  TARGET_UBUNTU_VER=18
elif [[ $UBUNTU_VERSION == *"$UBUNTU20"* ]]; then
  TARGET_UBUNTU_VER=20
else
  echo "Error: sorry, your system not support, only support[ubuntu18, ubuntu20]">&2
  exit 0
fi
if [[ $GAZEBO_VERSION == *"$GAZEBO9"* ]]; then
  TARGET_GAZEBO_VER=9
elif [[ $GAZEBO_VERSION == *"$GAZEBO11"* ]]; then
  TARGET_GAZEBO_VER=11
else
  echo "Error: sorry, your gazebo not support, only support[gazebo9, gazebo11]">&2
  exit 0
fi
GAZEBO_PLUGIN_FILE=contact_plugin_${TARGET_UBUNTU_VER}-04_g${TARGET_GAZEBO_VER}
cd gazebo_plugins/${GAZEBO_PLUGIN_FILE}
source ./install_plugin.sh
echo "export LASER_TYPE="CSPC"" >> ~/.bashrc
cd ../../models
cp ./* -rf ~/.gazebo/models
echo "成功" 
