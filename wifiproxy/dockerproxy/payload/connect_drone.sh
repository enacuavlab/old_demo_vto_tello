#!/bin/bash

cp /sbin/dhclient /usr/sbin/dhclinet >/dev/null

ifconfig $WIFI_DEV down
ifconfig $WIFI_DEV up

while true
do
  while [[ "`iw dev $WIFI_DEV scan 2>/dev/null | grep $DRONE_AP | wc -l`" != 1 ]];do sleep 0.1;done

  iw dev $WIFI_DEV connect $DRONE_AP
  /usr/sbin/dhclinet $WIFI_DEV > /dev/null 2>&1
  pkill dhclinet

  WIFI_IP=$(ip a | grep $WIFI_DEV | pcregrep -o1 'inet ([0-9]+.[0-9]+.[0-9]+.[0-9]+)')
  /usr/bin/socat udp-recv:11111,bind=$WIFI_IP udp-sendto:172.17.0.1:$VID_PORT &>/dev/null &
  /minitest.py $WIFI_IP &> /dev/null &

  while [[ $(cat /sys/class/net/$WIFI_DEV/carrier) = 1 ]];do sleep 0.1;done

  pkill socat > /dev/null 2>&1
  pkill minitest.py > /dev/null 2>&1
done
