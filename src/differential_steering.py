#!/usr/bin/python3

import rospy
import tf
import os
import math

from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from tf.transformations import quaternion_from_euler

WHEEL_RADIUS = 0.0335
WHEELBASE = 0.1
TICKS_RESOLUTION = 144

class DifferentialSteering:

    def __init__(self, robot_name):
        self.robot_name = robot_name
        rospy.init_node('differential_steering', anonymous=True)
        
        # create publisher for wheel commands
        wheel_command_topic = '/' + robot_name + '/wheels_driver_node/wheels_cmd'
        self.wheel_command_publisher = rospy.Publisher(wheel_command_topic, WheelsCmdStamped, queue_size = 10)
        
        # subscribe wheel tick messages
        left_wheel_tick_topic = '/' + robot_name + '/left_wheel_encoder_driver_node/tick'
        right_wheel_tick_topic = '/' + robot_name + '/right_wheel_encoder_driver_node/tick'
        rospy.Subscriber(left_wheel_tick_topic, WheelEncoderStamped, self.callback_left_wheel_tick)
        rospy.Subscriber(right_wheel_tick_topic, WheelEncoderStamped, self.callback_right_wheel_tick)

        self.transform_broadcaster = tf.TransformBroadcaster()

        # internal pose state (to track pose in memory)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # tick storage
        self.left_ticks = None
        self.right_ticks = None

        # set the robot at pose (0,0,0)
        self.publish_transform(x=0,y=0,theta=0)
       

    def callback_left_wheel_tick(self, data):
        self.left_ticks = data.data
    
    def callback_right_wheel_tick(self, data):
        self.right_ticks = data.data

    def update_pose(self, delta_left, delta_right):
        # convert number of ticks to real distance
        # distance = (ticks/ticks per res) * circumference
        d_left = (delta_left / TICKS_RESOLUTION) * 2 * math.pi * WHEEL_RADIUS
        d_right = (delta_right / TICKS_RESOLUTION) * 2 * math.pi * WHEEL_RADIUS

        # apply differential drive formulas
        V = (d_right + d_left) / 2
        w = (d_right - d_left) / WHEELBASE

        self.x += V * math.cos(self.theta)
        self.y += V * math.sin(self.theta)
        self.theta += w

    def run(self):
            while self.left_ticks is None or self.right_ticks is None:
                rospy.sleep(0.05)

            # store starting tick value
            prev_left = self.left_ticks
            prev_right = self.right_ticks

            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                curr_left = self.left_ticks
                curr_right = self.right_ticks

                delta_left = curr_left - prev_left
                delta_right = curr_right - prev_right

                if delta_left != 0 or delta_right != 0:
                    self.update_pose(delta_left, delta_right)
                    self.publish_transform(self.x, self.y, self.theta)
                    rospy.loginfo(f"x={self.x:.3f} y={self.y:.3f} θ={math.degrees(self.theta):.1f} degrees")
        
                prev_left = curr_left
                prev_right = curr_right

                rate.sleep()

    def publish_transform(self, x, y, theta):
        """publishes a transform between the map frame and the robot_name/base frame
                Parameters:
                    x = relative x position
                    y = relative y position
                    theta = relative rotation around z-axis
                    robot_name = name of duckiebot
        """
        q = quaternion_from_euler(0, 0 , theta)
        self.transform_broadcaster.sendTransform((x, y, 0.0), q, rospy.Time.now(),  self.robot_name + "/base", "map")


if __name__ == '__main__':
    vehicle_name = os.getenv('VEHICLE_NAME', 'pi')
    ds = DifferentialSteering(vehicle_name)
    ds.run()
