#!/usr/bin/python3

import rospy
import os
import math

from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from std_msgs.msg import Header

TICKS_RESOLUTION = 144
WHEEL_RADIUS = 0.0335
WHEELBASE = 0.07 #changed so it makes exactly one turn

# length each wheel must travel: pi * wheelbase
# resolution = arc_length / (2pi * wheel_radius)
# ticks needed = resolution * TICKS_RESOLUTION
ROTATION_TICKS = int((math.pi * WHEELBASE) / (2 * math.pi * WHEEL_RADIUS) * TICKS_RESOLUTION)

class RotateInPlace:

    def __init__(self, robot_name):
        # initialize node with a name that is unique
        rospy.init_node('rotate_in_place', anonymous=True)

        # create publisher (wheel command)
        topic_cmd = '/' + robot_name + '/wheels_driver_node/wheels_cmd'
        self.publisher = rospy.Publisher(topic_cmd, WheelsCmdStamped, queue_size=10)

        # tick counters
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_ticks_start = None
        self.right_ticks_start = None

        # subscribe to encoder ticks
        topic_left = '/' + robot_name + '/left_wheel_encoder_driver_node/tick'
        rospy.Subscriber(topic_left, WheelEncoderStamped, self.left_encoder_cb)
        topic_right = '/' + robot_name + '/right_wheel_encoder_driver_node/tick'
        rospy.Subscriber(topic_right, WheelEncoderStamped, self.right_encoder_cb)

        self.command = WheelsCmdStamped()
        self.command.header = Header()

        rospy.sleep(2.0) # wait for connections

    def left_encoder_cb(self, msg):
        self.left_ticks = msg.data
        if self.left_ticks_start is None:
            self.left_ticks_start = msg.data

    def right_encoder_cb(self, msg):
        self.right_ticks = msg.data
        if self.right_ticks_start is None:
            self.right_ticks_start = msg.data

    def set_wheels(self, vel_left, vel_right):
        self.command.vel_left = vel_left
        self.command.vel_right = vel_right
        self.command.header.stamp = rospy.Time.now()
        self.publisher.publish(self.command)

    def run(self):
        rospy.loginfo(f"Rotating in place.")

        # Wait until encoder start values set
        while self.left_ticks_start is None or self.right_ticks_start is None:
            rospy.sleep(0.05)

        rate = rospy.Rate(20) # 20 Hz control loop

        # Spin: left wheel forward, right wheel backward
        while not rospy.is_shutdown():
            left_traveled  = abs(self.left_ticks  - self.left_ticks_start)
            right_traveled = abs(self.right_ticks - self.right_ticks_start)

            rospy.loginfo(f"Ticks left: {left_traveled}, right: {right_traveled}")

            if left_traveled >= ROTATION_TICKS and right_traveled >= ROTATION_TICKS:
                break

            # slow down right wheel
            if right_traveled > left_traveled:
                vel_left  = 0.2
                vel_right = -0.2 * (left_traveled / right_traveled)
            elif left_traveled > right_traveled:
                vel_left  = 0.2 * (right_traveled / left_traveled)
                vel_right = -0.2
            else:
                vel_left  = 0.2
                vel_right = -0.2

            self.set_wheels(vel_left, vel_right)
            rate.sleep()

        self.set_wheels(0.0, 0.0)
        rospy.loginfo("Rotation complete!")

if __name__ == '__main__':
    vehicle_name = os.getenv('VEHICLE_NAME', 'pi')
    robot = RotateInPlace(vehicle_name)
    robot.run()

    

         
