# TurtleBot Draw Circle Service

## Overview
This package allows a TurtleBot to teleport to a given location and draw a circle of a specified radius using a custom ROS service.

## Folder Structure

## Steps to Run

1. **Set Up the Workspace**:
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    ```

2. **Clone the Repository and Navigate to Your Directory**:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/yourusername/yourrepository.git
    cd ROS_Y23/RollNo/src/turtle_pub
    ```

3. **Build the Package**:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

4. **Run the Turtlesim Node**:
    ```bash
    rosrun turtlesim turtlesim_node
    ```

5. **Run the Service Server Node**:
    ```bash
    rosrun turtle_pub draw_circle_server.py
    ```

6. **Call the Service**:
    ```bash
    rosservice call /draw_circle "x: 5.0 y: 5.0 radius: 2.0"
    ```

## Procedure Followed

1. **Created a Catkin Workspace**:
    - Set up a Catkin workspace to organize ROS packages.
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    ```

2. **Created a Package Named `turtle_pub`**:
    - Used `catkin_create_pkg` to create a new package.
    ```bash
    cd ~/catkin_ws/src
    catkin_create_pkg turtle_pub std_msgs rospy roscpp
    ```

3. **Created a Service Definition `DrawCircle.srv`**:
    - Defined the service to take inputs `x`, `y`, and `radius`.
    ```plaintext
    float32 x
    float32 y
    float32 radius
    ---
    bool success
    ```

4. **Wrote the `draw_circle_server.py` Script**:
    - Implemented the logic for teleporting the TurtleBot and making it draw a circle.
    ```python
    #!/usr/bin/env python3

    import rospy
    from geometry_msgs.msg import Twist
    from turtlesim.srv import TeleportAbsolute, TeleportAbsoluteRequest
    from turtle_pub.srv import DrawCircle, DrawCircleResponse

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
    ```

5. **Tested the Service**:
    - Ensured that the service correctly teleports the TurtleBot and makes it draw a circle.

## Notes
- Ensure that you have sourced your workspace before running any ROS commands:
  ```bash
  source ~/catkin_ws/devel/setup.bash
