import qi
import naoqi
import argparse
import sys
import time

IP = "192.168.0.147"
PORT = 9559


def main(session):
    """
    This example shows how to use ALTracker to track an object with trackEvent api.
    This example is only a subscriber. You need to create another script to raise the tracked event.
    Your events should follow this structure :
        EventNameInfo {
          TargetPositionInFrameWorld,
          TimeStamp,
          EffectorId,
          HeadThreshold (optional)
          }
    All details are available in ALTracker API Documentation.
    """

    # Set target to track.
    eventName = "ALTracker/BlobDetected"

    
    # Get the services ALTracker, ALMotion and ALRobotPosture.

    #motion_service = session.service("ALMotion")
    motion_service = naoqi.ALProxy("ALMotion", IP, PORT)
    posture_service = session.service("ALRobotPosture")
    tracker_service = session.service("ALTracker")
    memory_service = session.service("ALMemory")

    memory_service.declareEvent(eventName)
    
    # First, wake up.
    motion_service.wakeUp()

    print
    print "Head"
    print motion_service.getPosition("Head",2,False)
    print

    motion_service.post.moveToward(0.0,0.0,0.4,[])
    
    fractionMaxSpeed = 0.8

    # Go to posture stand
    posture_service.goToPosture("StandInit", fractionMaxSpeed)


    # set mode
    mode = "Head"
    tracker_service.setMode(mode)
    
    # Then, start tracker.
    tracker_service.trackEvent(eventName)

    print "ALTracker successfully started."
    print "Use Ctrl+c to stop this script."

    position = [0.0 for _ in range(6)]
    position[0] = 2.0
    position[1] = 0.0
    position[2] = 1.0
    
    try:
        while True:

            #print memory_service.getEventHistory(eventName)[0]
            #print tracker_service.getAvailableModes()
            #print tracker_service.getActiveTarget()
            #print tracker_service.getRegisteredTargets()
            #print tracker_service.getTargetPosition(2)
            #print tracker_service.isActive()
            
            #position[1]=y
            #y = -1.0 * y
            
            time_stamp = qi.clockNow()
        
            al_value = [position,
                        [time.time(),time_stamp],
                        0]

            print al_value

            memory_service.raiseEvent(eventName,al_value)

            motion_service.post.moveToward(0.0,0.0,0.6,[])
            
            time.sleep(2)
            
    except KeyboardInterrupt:
        print
        print "Interrupted by user"
        print "Stopping..."

    # Stop tracker, go to posture Sit.
    tracker_service.stopTracker()
    tracker_service.unregisterAllTargets()
    #posture_service.goToPosture("Sit", fractionMaxSpeed)
    #motion_service.rest()

    motion_service.post.moveToward(0.0,0.0,0.0,[])
    
    print "ALTracker stopped."


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.0.147",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)
