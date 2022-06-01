#!/usr/bin/python3
import queue
import socket
import threading
import time
import docker
import subprocess
import sys
import re


def set_connection(name):
    res = subprocess.run(
      ['docker','exec',name,'/sbin/ifconfig wlx3c7c3fa9c1e4 down'], capture_output=True, text=True
    )
    res = subprocess.run(
      ['docker','exec',name,'/sbin/ifconfig wlx3c7c3fa9c1e4 up'], capture_output=True, text=True
    )
    res = subprocess.run(
      ['docker','exec',name,'/sbin/iw dev wlx3c7c3fa9c1e4 connect TELLO-F0B594'], capture_output=True, text=True
    )
    res = subprocess.run(
      ['docker','exec',name,'/usr/bin/socat udp-recv:11111,bind=192.168.10.2 udp-sendto:172.17.0.1:11113 &'],capture_output=True, text=True
    )
    res = subprocess.run(
      ['docker','exec',name,'/usr/bin/socat udp-listen:8889,bind=172.17.0.2 udp-sendto:192.168.10.1:8889 &'],capture_output=True, text=True
    )


def check_connection(name):
  ret=False
  res1 = subprocess.run(
    ['docker','exec',name,'/bin/bash','-c','iwconfig | grep '+name], capture_output=True, text=True
  )
  if(len(res1.stdout)>0):
    wifidev = res1.stdout.split()[0]
    res2 = subprocess.run(
      ['docker','exec',name,'/bin/bash','-c','ip a | grep '+wifidev], capture_output=True, text=True
    )
    tmp=res2.stdout
    left="inet"
    if (left in tmp):
      ipwifi=(tmp[1+tmp.index(left)+len(left):tmp.index("/")])
      if (re.match(r'^(?:[0-9]{1,3}\.){3}[0-9]{1,3}$', ipwifi)):
        res3 = subprocess.run(
          ['docker','exec',name,'/bin/ping','-c 1','-W 1','192.168.10.1'], capture_output=True, text=True
        )
        tmp=res3.stdout
        if ("100% packet loss" not in tmp):ret=True
  return(ret)


def startup_sequence(commands):
  time.sleep(5)
  commands.put('command')
  commands.put('streamon')
  commands.put('downvision 0')


def monitor_connection(name,commands):
  prev_state=False
  while True:
    res = subprocess.run(
      ['docker','exec',name,'/bin/ping','-c 1','-W 1','192.168.10.1'], capture_output=True, text=True
    )
    tmp=res.stdout
    if ("100% packet loss" in tmp):curr_state=False
    else: curr_state=True
    if ((prev_state != curr_state) and curr_state): startup_sequence(commands)
    prev_state = curr_state


def monitor_battery(sock,commands):
  prev_state=False
  while True: 
    msg='battery?'
    if (msg not in commands.queue): commands.put(msg)
    try:
      data, server = sock.recvfrom(1518)
      tmp=data.decode(encoding="utf-8")
      if(tmp.count('\n')==1):
        bat=tmp[:-1]
        print(bat)
        time.sleep(1)
    except Exception:
      print ('\nExit . . .\n')
      break


def mission_sequence(commands):
  time.sleep(5)
  commands.put('takeoff')
  time.sleep(8)
  #commands.put('up 100')
  #time.sleep(6)
  #commands.put('cw 180')
  #time.sleep(6)
  #commands.put('ccw 180')
  #time.sleep(5)
  commands.put('land')


if __name__ == '__main__':
  commands = queue.Queue()
  commands.put('command')

  if(len(sys.argv)==2):
    if(sys.argv[1] == '?'):
      for i in  docker.DockerClient().containers.list():
        print(i.name+" created")
        if(check_connection(i.name)): print(i.name+" connected")
    else:
      for i in  docker.DockerClient().containers.list():
        if(sys.argv[1] == i.name):

           set_connection(i.name)

#          if(check_connection(i.name)): 
#            print(i.name+" created & connected")
#            tello_add = (docker.DockerClient().containers.get(i.name).attrs['NetworkSettings']['IPAddress'], 8889)
#            print(tello_add)
#            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#            pingThread = threading.Thread(target=monitor_connection,args=(i.name,commands))
#            battThread = threading.Thread(target=monitor_battery,args=(sock,commands))
#            missThread = threading.Thread(target=mission_sequence,args=[commands])
#            pingThread.start()
#            battThread.start()
#            missThread.start()
#            
#            try:
#              while True:
#
#                while not commands.empty():
#                  print(list(commands.queue))
#                  msg=commands.get()
#                  print("Sending <"+msg+">")
#                  sock.sendto(msg.encode(encoding="utf-8"),tello_add)
#
#                time.sleep(0.5)
#            except KeyboardInterrupt:
#              print("KeyboardInterrupt")
#              missThread.join()
#              battThread.join()
#              pingThread.join()
#              sock.close()
#              exit()
