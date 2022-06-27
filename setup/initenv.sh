#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{devnum}=="4", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules

echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ardupilot"' >/etc/udev/rules.d/ardupilot-usb.rules

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{devnum}=="3", MODE:="0666", GROUP:="dialout",  SYMLINK+="ardupilot"' >/etc/udev/rules.d/ardupilot.rules

service udev reload
sleep 2
service udev restart

