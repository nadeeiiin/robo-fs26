#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from std_msgs.msg import Header

class WheelTest:
    def __init__(self, robot_name):
        rospy.init_node('wheel_test', anonymous=True)

        topic_cmd = '/' + robot_name + '/wheels_driver_node/wheels_cmd'
        self.publisher = rospy.Publisher(topic_cmd, WheelsCmdStamped, queue_size=10)

        self.left_ticks = 0
        self.right_ticks = 0

        topic_left = '/' + robot_name + '/left_wheel_encoder_driver_node/tick'
        topic_right = '/' + robot_name + '/right_wheel_encoder_driver_node/tick'
        rospy.Subscriber(topic_left,  WheelEncoderStamped, lambda msg: setattr(self, 'left_ticks',  msg.data))
        rospy.Subscriber(topic_right, WheelEncoderStamped, lambda msg: setattr(self, 'right_ticks', msg.data))

        self.command = WheelsCmdStamped()
        self.command.header = Header()
        rospy.sleep(2.0)

    def set_wheels(self, vel_left, vel_right):
        self.command.vel_left = vel_left
        self.command.vel_right = vel_right
        self.command.header.stamp = rospy.Time.now()
        self.publisher.publish(self.command)

    def run(self):
        # record start
        left_start  = self.left_ticks
        right_start = self.right_ticks

        # drive straight for 1 second
        self.set_wheels(0.3, -0.3)
        rospy.sleep(1.0)
        self.set_wheels(0.0, 0.0)

        left_traveled  = abs(self.left_ticks  - left_start)
        right_traveled = abs(self.right_ticks - right_start)

        #rospy.loginfo(f"Left ticks:  {left_traveled}")
        #rospy.loginfo(f"Right ticks: {right_traveled}")
        #rospy.loginfo(f"Difference:  {abs(left_traveled - right_traveled)}")
        #rospy.loginfo(f"Ratio right/left: {right_traveled / left_traveled:.2f}")

if __name__ == '__main__':
    vehicle_name = os.getenv('VEHICLE_NAME', 'pi')
    robot = WheelTest(vehicle_name)
    robot.run()