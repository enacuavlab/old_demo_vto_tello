#!/bin/bash

cp /sbin/dhclient /usr/sbin/dhclinet >/dev/null

ifconfig $WIFI_DEV down
ifconfig $WIFI_DEV up

while [[ "`iw dev $WIFI_DEV scan 2>/dev/null | grep $DRONE_AP | wc -l`" != 1 ]];do sleep 0.1;done
iw dev $WIFI_DEV connect $DRONE_AP
/usr/sbin/dhclinet $WIFI_DEV > /dev/null 2>&1
pkill dhclient
WIFI_IP=$(ip a | grep $WIFI_DEV | pcregrep -o1 'inet ([0-9]+.[0-9]+.[0-9]+.[0-9]+)')

/usr/bin/socat udp-recv:11111,bind=192.168.10.2 udp-sendto:172.17.0.1:$VID_PORT &>/dev/null &

while true;
do 
  while [[ $(cat /sys/class/net/$WIFI_DEV/carrier) = 1 ]];do sleep 0.1;done
  while [[ "`iw dev $WIFI_DEV scan 2>/dev/null | grep $DRONE_AP | wc -l`" != 1 ]];do sleep 0.1;done
  iw dev $WIFI_DEV connect $DRONE_AP
  /usr/bin/killall /usr/sbin/dhclinet > /dev/null 2>&1
  /usr/sbin/dhclinet $WIFI_DEV > /dev/null 2>&1
  /usr/bin/killall /usr/bin/python3 > /dev/null 2>&1
  /minitest.py &> /dev/null
done
