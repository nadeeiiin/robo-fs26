#!/usr/bin/env python3
import rospy
import os
import math
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
from std_msgs.msg import Header

TICKS_RESOLUTION = 144
WHEEL_RADIUS = 0.0335
WHEELBASE = 0.1

class RotateInPlace:
    def __init__(self, robot_name):
        rospy.init_node('rotate_in_place', anonymous=True)

        topic_cmd = '/' + robot_name + '/car_cmd_switch_node/cmd'
        self.publisher = rospy.Publisher(topic_cmd, Twist2DStamped, queue_size=10)

        self.left_ticks = 0
        self.right_ticks = 0
        self.left_ticks_start = None
        self.right_ticks_start = None

        topic_left  = '/' + robot_name + '/left_wheel_encoder_driver_node/tick'
        topic_right = '/' + robot_name + '/right_wheel_encoder_driver_node/tick'
        rospy.Subscriber(topic_left,  WheelEncoderStamped, self.left_encoder_cb)
        rospy.Subscriber(topic_right, WheelEncoderStamped, self.right_encoder_cb)

        self.command = Twist2DStamped()
        self.command.header = Header()
        rospy.sleep(2.0)

    def left_encoder_cb(self, msg):
        self.left_ticks = msg.data
        if self.left_ticks_start is None:
            self.left_ticks_start = msg.data

    def right_encoder_cb(self, msg):
        self.right_ticks = msg.data
        if self.right_ticks_start is None:
            self.right_ticks_start = msg.data

    def set_command(self, v, omega):
        self.command.v = v
        self.command.omega = omega
        self.command.header.stamp = rospy.Time.now()
        self.publisher.publish(self.command)

    def run(self):
        omega = 4.0
        rotation_time = 4
        rospy.loginfo(f"Rotating in place for {rotation_time:.2f} seconds")

        self.set_command(v=0.0, omega=omega)
        rospy.sleep(rotation_time)
        self.set_command(v=0.0, omega=0.0)

        rospy.loginfo("Rotation complete!")

if __name__ == '__main__':
    vehicle_name = os.getenv('VEHICLE_NAME', 'pi')
    robot = RotateInPlace(vehicle_name)
    robot.run()