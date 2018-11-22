#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  arduino"
echo "sudo rm   /etc/udev/rules.d/arduino.rules"
sudo rm   /etc/udev/rules.d/arduino.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
