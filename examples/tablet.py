import pepper_interface

IP = "192.168.0.147"
PORT = 9559

simulation = False

url = "http://vincentberenz.is.tuebingen.mpg.de/export/playful_playful.png"

with pepper_interface.get(IP,PORT,simulation) as pepper:

    pepper.tablet.set_background("#FFFFFF")
    pepper.tablet.set_image(url)
