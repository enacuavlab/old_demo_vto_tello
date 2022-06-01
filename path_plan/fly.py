#!/usr/bin/python3

import docker
import time
import pygame
from djitellopy import TelloSwarm


def main():
  pygame.display.init()
  pygame.joystick.init()
  joystick_nr = pygame.joystick.get_count()
  controller = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
  for j in range(pygame.joystick.get_count()): controller[j].init()

  #ac_list = [['60', '60', docker.DockerClient().containers.get('TELLO-ED4310').attrs['NetworkSettings']['IPAddress']],]
  ac_list = [['59', '59', docker.DockerClient().containers.get('TELLO-F0B594').attrs['NetworkSettings']['IPAddress']],
             ['60', '60', docker.DockerClient().containers.get('TELLO-ED4310').attrs['NetworkSettings']['IPAddress']]]

  ip_list = [_[2] for _ in ac_list]
  swarm = TelloSwarm.fromIps(ip_list)
  id_list = [_[1] for _ in ac_list]
  for i,id in enumerate(id_list): swarm.tellos[i].set_ac_id(id)

  swarm.connect(wait_for_state=False)
  time.sleep(5)
  swarm.takeoff()
  time.sleep(10)
  swarm.land()

if __name__=="__main__":
    main()
