#!/bin/bash

cp /sbin/dhclient /usr/sbin/dhclinet >/dev/null

while true;
do
  ifconfig $WIFI_DEV up
  while [[ "`iw dev $WIFI_DEV scan 2>/dev/null | grep $DRONE_AP | wc -l`" != 1 ]];do sleep 0.1;done

  iw dev $WIFI_DEV connect $DRONE_AP
  /usr/sbin/dhclinet $WIFI_DEV > /dev/null 2>&1
  pkill dhclinet

  WIFI_IP=$(ip a | grep $WIFI_DEV | pcregrep -o1 'inet ([0-9]+.[0-9]+.[0-9]+.[0-9]+)')
  /usr/bin/socat udp-recv:11111,bind=$WIFI_IP udp-sendto:172.17.0.1:$VID_PORT &>/dev/null &

  DOCKER_IP=$(ip a | grep eth0 | pcregrep -o1 'inet ([0-9]+.[0-9]+.[0-9]+.[0-9]+)')
  sleep 2
  /minitest.py $DOCKER_IP $CMD_PORT &> /dev/null &

#  while timeout 0.5 ping -c 1 -n 192.168.10.1 &> /dev/null; do sleep 0.5; done
  REACHEABLE=0;
  while [ $REACHEABLE -ne "1" ];
    do ping -q -c 1 -w 2 192.168.10.1 &> /dev/null; REACHEABLE=$?; sleep 1;
  done


  pkill socat > /dev/null 2>&1
  pkill minitest.py > /dev/null 2>&1

  ifconfig $WIFI_DEV down
done
