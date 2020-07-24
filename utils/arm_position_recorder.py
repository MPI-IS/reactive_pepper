import numpy as np
import naoqi, rospy, time, sys, termios, tty

#This program can be used to record the arm positions to be used
#in interactive dance motions, run the script by changing the IP
#Press 0 to record the joint angles in a text file 
 
IP = "192.168.0.147"
PORT = 9559

motion = naoqi.ALProxy("ALMotion", IP, PORT)
motion.wakeUp()

left_arm_tags= ['LShoulderPitch','LShoulderRoll','LElbowYaw','LElbowRoll','LWristYaw']
right_arm_tags= ['RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll','RWristYaw']
both_arms = ['LShoulderPitch','LShoulderRoll','LElbowYaw','LElbowRoll','LWristYaw','RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll','RWristYaw']

left = False


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


if left:
    names = 'LArm'
    tags = left_arm_tags
else:
    names = 'RArm'
    tags = right_arm_tags

tags = both_arms
names = ['LArm', 'RArm']

stiffnessLists  = 0.0
timeLists  = 1.0

motion.stiffnessInterpolation(names, stiffnessLists, timeLists)

time.sleep(1)

stiffnesses_return =motion.getStiffnesses(names)
joints_free = 1

for i in stiffnesses_return:
    if i != 0.0:
        joints_free = 0

start_time = time.time();
values = motion.getAngles("Body", True)

if (joints_free):
    while  time.time() - start_time <10:
        char = getch()
        
        if char == 'c':
            sys.exit() 

        elif char == '0':
            #file = open('joint_angles.txt','a') 
            values = motion.getAngles(tags, True)
            joint_values = {j:v for j,v in zip(tags,values)}
            print repr(joint_values)
            #file.write(str(joint_values)+'\n')
            #file.close()

