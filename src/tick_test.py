#!/usr/bin/env python3
import rospy
import os
import math
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

WHEEL_RADIUS = 0.0335
DISTANCE_METERS = 1.3
TICKS_RESOLUTION = 144
TICKS_TO_DRIVE = 60
REPETITIONS = 1

class TickTest:
    def __init__(self, robot_name):
        rospy.init_node('tick_test')
        self.left_ticks = None
        self.right_ticks = None
        self.pub = rospy.Publisher('/' + robot_name + '/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/' + robot_name + '/left_wheel_encoder_driver_node/tick', WheelEncoderStamped, self.cb_left)
        rospy.Subscriber('/' + robot_name + '/right_wheel_encoder_driver_node/tick', WheelEncoderStamped, self.cb_right)
        rospy.sleep(2.0)

    def cb_left(self, msg):
        self.left_ticks = msg.data

    def cb_right(self, msg):
        self.right_ticks = msg.data

    def set_wheels(self, vel_l, vel_r):
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = vel_l
        cmd.vel_right = vel_r
        self.pub.publish(cmd)

    def to_ticks(self, distance):
        return int(distance / (2 * math.pi * WHEEL_RADIUS) * TICKS_RESOLUTION)


    def drive_ticks(self, ticks, vel_l, vel_r):
        while self.left_ticks is None or self.right_ticks is None:
            rospy.sleep(0.05)
        
        self.set_wheels(vel_l, vel_r)

        init_l, init_r = self.left_ticks, self.right_ticks
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.left_ticks != init_l or self.right_ticks != init_r:
                break
            rate.sleep()

        start_l, start_r = self.left_ticks, self.right_ticks
        rospy.loginfo(f"Ziel: {ticks} ticks")

        while not rospy.is_shutdown():
            delta = (abs(self.left_ticks - start_l) + abs(self.right_ticks - start_r)) / 2
            if delta >= ticks:
                break
            self.set_wheels(vel_l, vel_r)
            rate.sleep()

        self.set_wheels(0.0, 0.0)
        rospy.sleep(3.0)

    def run(self):
        ticks = self.to_ticks(DISTANCE_METERS)
        rospy.loginfo(f"Distanz: {DISTANCE_METERS}m = {ticks} ticks")
        rospy.loginfo(f"START: left={self.left_ticks}, right={self.right_ticks}")
        for i in range(REPETITIONS):
            rospy.loginfo(f"Durchlauf {i+1}/{REPETITIONS}")
            self.drive_ticks(ticks, 0.1, 0.1)
            rospy.loginfo(f"Nach Durchlauf {i+1}: left={self.left_ticks}, right={self.right_ticks}")
        rospy.loginfo(f"ENDE: left={self.left_ticks}, right={self.right_ticks}")

if __name__ == '__main__':
    robot = TickTest(os.getenv('VEHICLE_NAME', 'pi'))
    robot.run()