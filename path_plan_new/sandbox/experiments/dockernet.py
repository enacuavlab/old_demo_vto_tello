#!/usr/bin/python3


import docker
import subprocess


#------------------------------------------------------------------------------
#
# This function is used to get IP and PORT from drone configured as hotspot, and connected through docker
# These drones provide different IP and PORT to be guided and delivering video
#
#------------------------------------------------------------------------------
def getdockeraddr(name):
  ret = False
  docker_ip = None
  cmd_port = None

  for i in  docker.DockerClient().containers.list():
    if(name == i.name):
      res = subprocess.run(
        ['docker','exec',i.name,'/bin/ping','-c 1','-W 1','192.168.10.1'], capture_output=True, text=True
      )
      if ("100% packet loss" in res.stdout):break
      res = subprocess.run(
        ['docker','exec',i.name,'/usr/bin/env'], capture_output=True, text=True
      )
      tmp=res.stdout
      left="CMD_PORT="
      if (left in tmp):
        cmd_port=int((tmp[tmp.index(left)+len(left):]).split()[0])
        docker_ip=(docker.DockerClient().containers.get(i.name).attrs['NetworkSettings']['IPAddress'])
        ret = True

  return ret,(docker_ip,cmd_port)
