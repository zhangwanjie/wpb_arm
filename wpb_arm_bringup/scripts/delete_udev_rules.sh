#!/bin/bash

echo "***************"
echo "delete the remap device serial port to ch34x"
sudo rm   /etc/udev/rules.d/ch34x.rules
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
