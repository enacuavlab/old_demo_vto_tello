#!/usr/bin/python3

import subprocess

tellos_routeur = {61:'TELLO-ED433E',62:'TELLO-ED4317',63:'TELLO-ED42A3',64:'TELLO-ED4381',65:'TELLO-F0B594',66:'TELLO-99CE21'}
tellos_docker = {60:'TELLO-ED4310',67:'TELLO-99CE5A',68:'TELLO-99CE4E'}
#tellos_docker = {67:'TELLO-99CE5A',68:'TELLO-99CE4E'}

#------------------------------------------------------------------------------
def initNetDrone(selected):
  telloDic = {}
  cpt=0
  for i in selected:
    if i in tellos_routeur:
      addr = ('192.168.1.'+str(i),8889)
      p = subprocess.Popen(["ping", "-q", "-c", "1", addr[0]], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      if p.wait() == 0:
        telloDic[i] = (tellos_routeur[i],addr)
        cpt = cpt + 1

    elif i in tellos_docker:
      telloDic[i] = tellos_docker[i]
      from dockernet import getdockeraddr   # Should run even without docker installed,
      ret,addr = getdockeraddr(telloDic[i]) # for non docker tellos
      if ret:
        telloDic[i] = (tellos_docker[i],addr)
        cpt = cpt + 1

  ret = True if cpt == len(selected) else False
  return(ret,telloDic)
