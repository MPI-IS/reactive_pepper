import math,time
import pepper_interface



IP = "192.168.0.147"
PORT = 9559
simulation = False

#This example shows how to play music using pepper
#The tracks have to to be transferred to pepper's internal memory using sftp
#The path of the folder /home/nao/playful_music


with pepper_interface.get(IP,PORT,simulation) as pepper:

    time_start = time.time()
    song = "wheels_on_the_bus"
    pepper.music.load_music("/home/nao/playful_music/" + song + ".mp3")

    executed = False

    while time.time() - time_start < 15:

        if not executed:
            pepper.music.play_music()
            executed = True
        
        time.sleep(0.01)
      
    pepper.music.stop_music()
    pepper.music.unload_music()


            
