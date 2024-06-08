#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportAbsoluteRequest
from DrawCircle.srv import DrawCircle, DrawCircleResponse

def draw_circle(req):
    rospy.wait_for_service('turtle1/teleport_absolute')
    try:
        teleport_service = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        teleport_request = TeleportAbsoluteRequest()
        teleport_request.x = req.x
        teleport_request.y = req.y
        teleport_request.theta = 0
        teleport_service(teleport_request)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return DrawCircleResponse(success=False)
    
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    
    move_cmd = Twist()
    move_cmd.linear.x = req.radius
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rospy.sleep(1)
    
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 1.0
    pub.publish(move_cmd)
    
    circle_duration = 2 * 3.14159 * req.radius / 1.0
    rospy.sleep(circle_duration)
    
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    
    return DrawCircleResponse(success=True)

def draw_circle_server():
    rospy.init_node('draw_circle_server')
    s = rospy.Service('draw_circle', DrawCircle, draw_circle)
    rospy.loginfo("Ready to draw circle.")
    rospy.spin()

if __name__ == "__main__":
    draw_circle_server()
