#!/bin/bash
sudo cp ./libChargeContact.so /usr/lib/x86_64-linux-gnu/gazebo-11/plugins
sudo cp ./ChargeContactPlugin.hh /usr/include/gazebo-11/gazebo/plugins
echo "install contact plugin success"
