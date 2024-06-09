#!/usr/bin/env python3
from _future_ import print_function
import rospy
from geometry_msgs.msg import Twist
from turtle_pub.srv import myfinalservices
from DrawCircle.srv import TeleportAbsolute

    
def teleport_turtle(x, y):
    try:
        teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        teleport(x, y, 0)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def turtle_controller(radius):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # Control rate: 10 Hz
    while not rospy.is_shutdown():
        msg = Twist()
        rospy.sleep(1)
        msg.linear.x = radius 
        pub.publish(msg)
        rospy.sleep(2)
        msg.linear.x= 0
        msg.angular.z =1.57
        pub.publish(msg)
        rospy.sleep(2)
        msg.linear.x = 2*3.14*radius
        msg.angular.z = 2*3.14
        pub.publish(msg)
        rospy.sleep()
        rate.sleep()

def teleporting_now(req):
     teleport_turtle(req.x ,req.y)
     turtle_controller(req.radius)

def teleporting_server():
       rospy.init_node('add_two_ints_server')
       s = rospy.Service('MERI_SERVICE', myfinalservices, teleporting_now)
       print("READY FOR THE INPUT FROM USER")
       rospy.spin()
   
if _name_ == "_main_":
   teleporting_server()
