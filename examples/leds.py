import math,time,random
import pepper_interface

IP = "192.168.0.147"
PORT = 9559

simulation = False

with pepper_interface.get(IP,PORT,simulation) as pepper:

    pepper.leds.set("blue","EYES",duration=1.0)
    pepper.leds.set("green","CHEST",duration=1.0)
    
    time.sleep(2.0)

    pepper.leds.set("red","ALL",duration=1.0)

    time.sleep(2.0)

    pepper.leds.set("playful","ALL",duration=1.0)

    time.sleep(2.0)
    
    for _ in range(3):
        pepper.leds.blink("playful_pink","playful",duration=0.01)
        time.sleep(2.0)
    
    pepper.leds.set("playful","ALL")

