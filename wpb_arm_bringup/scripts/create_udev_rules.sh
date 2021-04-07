#!/bin/bash

echo "***************"
echo "remap the device serial port(ttyUSBX) to ch34x"
echo "start copy ch34x.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpb_arm_bringup`/scripts/ch34x.rules  /etc/udev/rules.d

echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
