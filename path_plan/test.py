#!/usr/bin/python3
from natnet2python import Natnet2python
from natnet2python import Vehicle
import time

def main():

  ac_list = [['TELLO-ED4310',60,0,0,0],]
  ac_id_list = [[_[1]] for _ in ac_list]
  print(ac_id_list)

  vehicles = []
  for i in ac_list:
    vehicles.append(Vehicle(str(i[1])))
  for i in vehicles:
    print(i.ac_id)

  #ac_list = [['63', '63', '192.168.1.63'],]
  ac_list = [['60', '60', '192.168.1.60'],]
  ac_id_list = [[_[1], _[1]] for _ in ac_list]
  #ac_id_list.append(['888', '888']) # Add a moving target
  #vehicles = [Vehicle('60'),]


  voliere = Natnet2python(ac_id_list, vehicles, freq=40)

  try:
    voliere.run()
    while True:
      time.sleep(0.5)

  except (KeyboardInterrupt, SystemExit):
    print("Shutting down natnet interfaces...")
    voliere.stop()

if __name__=="__main__":
    main()
