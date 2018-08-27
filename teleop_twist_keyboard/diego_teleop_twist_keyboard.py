#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from ros_arduino_msgs.srv import *
from math import pi as PI, degrees, radians
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist, Servo or sensor!
---------------------------
Moving around:
   u    i    o
   j    k    l

, : up (+z)
. : down (-z)

----------------------------
Left arm servo control:
+   1   2   3   4   5   6
-   q   w   e   r   t   y  
----------------------------
Right arm servo control:
+   a   s   d   f   g   h
-   z   x   c   v   b   n 

p : init the servo

CTRL-C to quit
"""

armServos={
            	'1':(0,1),
	        '2':(1,1),
	        '3':(2,1),
	        '4':(3,1),
	        '5':(4,1),
	        '6':(5,1),
            	'7':(6,1),
            	'8':(7,1),
            	'9':(8,1),
	        'q':(0,0),
	        'w':(1,0),
	        'e':(2,0),
	        'r':(3,0),
	        't':(4,0),
	        'y':(5,0),
	        'u':(6,0),
            	'i':(7,0),
            	'o':(8,0),
	      }
	      
armServoValues=[90,90,90,90,90,90,90,90,90,90]



def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

       
def servoWrite(servoNum, value):
        rospy.wait_for_service('/arduino/servo_write')
	try:
	    servo_write=rospy.ServiceProxy('/arduino/servo_write',ServoWrite)
	    servo_write(servoNum,value)
	    print servoNum
            print value
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('teleop_twist_keyboard')

	try:
		print msg
        
		servoWrite(0, armServoValues[0])
		servoWrite(1, armServoValues[1])
		servoWrite(2, armServoValues[2])
		servoWrite(3, armServoValues[3])
		servoWrite(4, armServoValues[4])
		servoWrite(5, armServoValues[5])
		servoWrite(6, armServoValues[6])
		servoWrite(7, armServoValues[7])
		servoWrite(8, armServoValues[8])

		while(1):
			key = getKey()
			print key
			if key in armServos.keys():  
			    if(armServos[key][1]==0):
			        armServoValues[armServos[key][0]]=armServoValues[armServos[key][0]]-5
			        if armServoValues[armServos[key][0]]<=0:
			           armServoValues[armServos[key][0]]=0
			    else:
			        armServoValues[armServos[key][0]]=armServoValues[armServos[key][0]]+5
			        if armServoValues[armServos[key][0]]>=180:
			           armServoValues[armServos[key][0]]=180
			    print armServoValues[armServos[key][0]]
			    servoWrite(armServos[key][0], armServoValues[armServos[key][0]])
			
	except BaseException,e:
		print e

	finally:
		print "Atention something wrong!!!!!"

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


