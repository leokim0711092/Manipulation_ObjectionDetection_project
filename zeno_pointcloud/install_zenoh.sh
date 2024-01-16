#!/bin/bash

echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null 
sudo apt update 
sudo apt install zenoh-bridge-ros2dds