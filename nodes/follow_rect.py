#!/usr/bin/env python3

"""
 The follow_rect node publishes a custom velocity message into a geometry_msgs/Twist
 and publish to the robot's cmd_vel in order to drive in a rectangle figure

PUBLISHERS:
  pub_vel (geometry_msgs/Twist) - the linear and angular velocity of the robot
  path_pub (nav_msgs/Path) - the path (x,y) of the robot

SUBSCRIBERS:
  sub_odom (nav_msgs/Odometry) - the linear and angular velocity of the robot

PARAMETERS:
  width - The width of the figure eight
  height - The height of the figure eight
  period - The amount of time it takes to complete the figure eight
  ~pub_freq - The frequency at which to publish the messages (a private parameter)
  linear_velocity - The linear velocity of the robot
  angular_velocity - The angular velocity of the robot
"""

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path, Odometry
from std_srvs.srv import Empty, EmptyResponse


class FollowRect():
    """ Publish a geometry_msgs/Twist of the linear and angular
    velocity to the robot's cmd_vel at a fixed rate 
    """
    def __init__(self):
        self.W = rospy.get_param("width")                   # initializing the width of the rectangular figure
        self.H = rospy.get_param("height")                  # initializing the height of the rectangular figure
        self.T = rospy.get_param("period")                  # initializing the amount of time it takes to complete one round
        self.R = rospy.get_param("pub_freq")                # initializing the frequency at which to publish the messages
        self.V = rospy.get_param("linear_velocity")         # initializing the linear velocity of the robot
        self.Omega = rospy.get_param("angular_velocity")    # initializing the angular velocity of the robot

        self.sub_odom = rospy.Subscriber("/diff/odom", Odometry, self.position_callback)
        self.pub_vel = rospy.Publisher("/diff/cmd_vel", Twist, queue_size = 10)
        self.path_pub = rospy.Publisher("path", Path, queue_size = 10)

        self.rate = rospy.Rate(self.R)
        self.x = 0
        self.y = 0
        self.x_init = -3
        self.y_init = -3
        self.angle = 0
        self.path = Path()
        self.path.poses = []


    def position_callback(self, data):
        """ Callback function for /diff/odom.
        Subscribes to /diff/odom, to get the position (x, y, angle) of the robot. 
        The subscriber updates the variables x, y and angle.
        
        Args:
          data from the subscricber
        """
        self.x = data.pose.pose.position.x - self.x_init
        self.y = data.pose.pose.position.y - self.y_init
        self.angle = data.pose.pose.orientation.z


    def twist_pub(self, x, w):
        """ Create a twist suitable for the cmd_vel
        Args: None
        Returns:
            Twist - a 2D twist object corresponding to linear/angular velocity
        """
        return Twist(linear = Vector3(x = x, y = 0, z = 0),
                    angular = Vector3(x = 0, y = 0, z = w))


    def move(self, x, w):
        """ Move the robot by publish a twist messages to the cmd_vel
        """
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "world"
        rospy.Subscriber("/diff/odom", Odometry, self.position_callback)
        self.path.poses.append(PoseStamped(header = self.path.header, 
                                        pose = Pose(
                                            position = Point(self.x, self.y, 0), 
                                            orientation = Quaternion(0, 0, 0, 1))))
        self.path_pub.publish(self.path)
        twist = self.twist_pub(x, w)
        self.pub_vel.publish(twist)
        self.rate.sleep()



    def run(self):
        """ Run the node
        """
        rospy.logdebug(f"Run Message")
        while not rospy.is_shutdown() and self.x < self.W:
            rospy.logdebug(f"1. angle = {self.angle}, x = {self.x}, y = {self.y}")
            self.move(self.V, 0)
        self.move(0, 0)
        while not rospy.is_shutdown() and self.angle < 0.65:
            rospy.logdebug(f"2. angle = {self.angle}, x = {self.x}, y = {self.y}")
            self.move(0, self.Omega)
        self.move(0, 0)
        while not rospy.is_shutdown() and self.y < self.H:
            rospy.logdebug(f"3. angle = {self.angle}, x = {self.x}, y = {self.y}")
            self.move(self.V, 0)
        self.move(0, 0)
        while not rospy.is_shutdown() and self.angle < 0.998:
            rospy.logdebug(f"4. angle = {self.angle}, x = {self.x}, y = {self.y}")
            self.move(0, self.Omega)
        self.move(0, 0)
        while not rospy.is_shutdown() and self.x > 0:
            rospy.logdebug(f"5. angle = {self.angle}, x = {self.x}, y = {self.y}")
            self.move(self.V, 0)
        self.move(0, 0)   
        while not rospy.is_shutdown() and self.angle > 0.75:
            rospy.logdebug(f"6. angle = {self.angle}, x = {self.x}, y = {self.y}")
            self.move(0, self.Omega)
        self.move(0, 0)    
        while not rospy.is_shutdown() and self.y > 0:
            rospy.logdebug(f"7. angle = {self.angle}, x = {self.x}, y = {self.y}")
            self.move(self.V, 0)
        self.move(0, 0)    
        while not rospy.is_shutdown() and self.angle > 0.05:
            rospy.logdebug(f"8. angle = {self.angle}, x = {self.x}, y = {self.y}")
            self.move(0, self.Omega)
        self.move(0, 0) 
        rospy.logdebug(f"Finised")


def main():
    """ The main() function. """
    rospy.init_node('follow_rect', log_level = rospy.DEBUG)
    rect = FollowRect()
    rect.run()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass