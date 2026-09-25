#!/bin/bash
sudo cp ./libChargeContact.so /usr/lib/x86_64-linux-gnu/gazebo-9/plugins
sudo cp ./ChargeContactPlugin.hh /usr/include/gazebo-9/gazebo/plugins
echo "install contact plugin success"
