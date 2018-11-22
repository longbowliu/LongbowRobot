#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  arduino"
echo "arduino usb connection as /dev/arduino , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy arduino.rules to  /etc/udev/rules.d/"
echo "`rospack find ros_arduino_python`/scripts/arduino.rules"
sudo cp `rospack find ros_arduino_python`/scripts/arduino.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
