#!/usr/bin/python3
import queue

def main():
  commands = queue.Queue()
  commands.put(('command',56))
  commands.put(('command',54))
  while not commands.empty():
    vtupple=commands.get()
    print(vtupple[1])

if __name__=="__main__":
    main()
