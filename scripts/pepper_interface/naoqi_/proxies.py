import naoqi


class Proxies(object):

    
    def __init__(self,IP,PORT,
                 simulation,
                 face_detection,
                 face_detection_subscription=None,
                 face_detection_period=None):

        self.posture = naoqi.ALProxy("ALRobotPosture", IP, PORT)
        self.memory = naoqi.ALProxy("ALMemory", IP, PORT)
        self.motion = naoqi.ALProxy("ALMotion", IP, PORT)
        self.battery = naoqi.ALProxy("ALBattery", IP, PORT)
        self.tts = naoqi.ALProxy("ALTextToSpeech", IP, PORT)
        self.leds = naoqi.ALProxy("ALLeds", IP, PORT)


        # does not work on naoqi 2.4.3 (works on 2.5)
        try :
            self.tablet = naoqi.ALProxy("ALTabletService",IP,PORT)
        except Exception as e :
            print
            print "pepper interface: failed to connect to tablet proxy: "+str(e)
            print "(expected if you are using naoqi version < 2.5)"
            print
            pass
            
        self.tracker = naoqi.ALProxy("ALTracker",IP,PORT)
        
        if simulation:

      	    self.face_detection = None
      	    self.camera = None
            self.audio = None
            
        else:
            
            self.audio = naoqi.ALProxy("ALAudioPlayer",IP,PORT)
            self.camera = naoqi.ALProxy("ALVideoDevice", IP, PORT)

            if face_detection:
                self.face_detection = naoqi.ALProxy("ALFaceDetection", IP, PORT)
                self.face_detection.subscribe(face_detection_subscription,
                                              face_detection_period,
                                              0.0)
            
        self._face_detection_subscription = face_detection_subscription

        
    def stop(self):
        try :
            self.face_detection.unsubscribe(self._face_detection_subscription)
        except :
            pass

        
    def __del__(self):
        self.stop()
