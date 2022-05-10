#!/usr/bin/env python 
# Echo client program
import socket
import sys
HOST = "192.168.0.100" # The UR IP address
PORT = 30002 # UR secondary client
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
#f = open ("Grip.script", "rb")   #Robotiq Gripper
#f = open ("setzero.script", "rb")  #Robotiq FT sensor
path = '/home/hydran/catkin_ws/src/soft_robotics/soft_robotics_description/scripts/'
def callback(data):
    
    script=''
    if(data == 'open1'):
        script = path + 'open1.script'
    elif(data == 'close'):
        script = path + 'close.script'
    if(data == 'open2'):
        script = path + 'open2.script'
    else:
        print("Invalid argument!")
    f = open (script, "rb")   #Robotiq Gripper
    l = f.read(2024)
    while (l):
        s.send(l)
        print(l)
        l = f.read(2024)
    f.close()
    print(str(data))

    # spin() simply keeps python from exiting until this node is stopped
if __name__ == '__main__':
    callback('open')

