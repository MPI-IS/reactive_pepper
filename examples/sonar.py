import math,time,random
import pepper_interface

IP = "192.168.0.147"
PORT = 9559
simulation = False

with pepper_interface.get(IP,PORT,simulation) as pepper:

    time.sleep(1.0)

    values,time_stamp = pepper.sonar.get()

    print
    print "Front"
    print values["Front"]
    print
    print "Back"
    print values["Back"]
    print
