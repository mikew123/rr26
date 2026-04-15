#!/bin/bash

echo "hubreset.sh"

sudo usbreset "USB2.0 HUB"
sleep 2.0
n=$(usbreset | grep -e 'OpenMV' -e 'CP2102N' | wc | awk -F ' ' '{print $1}')

while [ $n -ne 2 ]; do
  echo "Lidar or Camera not ready n = $n - reset the USB HUB"
  sudo usbreset "USB2.0 HUB"
  sleep 2.0
  n=$(usbreset | grep -e 'OpenMV' -e 'CP2102N' | wc | awk -F ' ' '{print $1}')
done
