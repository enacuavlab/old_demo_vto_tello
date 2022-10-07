#!/usr/bin/python3
import socket
import threading
import time

#ap ssid pass
#ap pprz_router pprzpprz

host = ''
port = 9000
locaddr = (host,port)

tello_address = ('192.168.10.1', 8889)

#------------------------------------------------------------------------------
def recv():
  count = 0
  while True:
    try:
      data, server = sock.recvfrom(1518)
      print(data.decode(encoding="utf-8"))
    except Exception:
      print ('\nExit . . .\n')
      break

#------------------------------------------------------------------------------
if __name__=="__main__":

  sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.bind(locaddr)

  recvThread = threading.Thread(target=recv)
  recvThread.start()

  time.sleep(2.0)
  sock.sendto('command'.encode(encoding="utf-8"),tello_address)

  loop = True
  while loop:
    try:
      msg = input("")
      print("sending ["+msg+"]")
      sent = sock.sendto(msg.encode(encoding="utf-8"),tello_address)

    except KeyboardInterrupt:
      print("\nWe are interrupting the program\n")
      loop=False
      sock.close()
      print("mainloop stopped")
