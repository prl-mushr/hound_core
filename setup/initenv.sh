#!/bin/bash
echo  'KERNEL=="ttyUSB*", KERNELS=="1-1.3:1.0", ATTRS{idVendor}=="10c4", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules

echo  'KERNEL=="ttyUSB*", KERNELS=="1-1.4:1.0", ATTRS{idVendor}=="10c4", MODE:="0666", GROUP:="dialout",  SYMLINK+="ardupilot"' >/etc/udev/rules.d/ardupilot.rules

service udev reload
sleep 2
service udev restart

