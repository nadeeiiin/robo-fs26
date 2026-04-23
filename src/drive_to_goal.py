#!/usr/bin/env python3
import rospy
import os
import math
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from std_msgs.msg import Header

WHEEL_RADIUS = 0.0335
WHEELBASE = 0.048 # angepasst da viel zu stark gedreht
TICKS_RESOLUTION = 144

class DriveToGoal:
    def __init__(self, robot_name):
        rospy.init_node('drive_to_goal')
        self.left_ticks = None
        self.right_ticks = None

        self.pub = rospy.Publisher('/' + robot_name + '/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/' + robot_name + '/left_wheel_encoder_driver_node/tick',  WheelEncoderStamped, self.cb_left)
        rospy.Subscriber('/' + robot_name + '/right_wheel_encoder_driver_node/tick', WheelEncoderStamped, self.cb_right)
        rospy.sleep(2.0)

    def cb_left(self, msg):
        self.left_ticks = -msg.data

    def cb_right(self, msg):
        self.right_ticks = msg.data

    def set_wheels(self, vel_left, vel_right):
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = vel_left
        cmd.vel_right = vel_right
        self.pub.publish(cmd)

    def drive_ticks(self, ticks, vel_l, vel_r):
        while self.left_ticks is None or self.right_ticks is None:
            rospy.sleep(0.05)
        
        # Räder starten
        self.set_wheels(vel_l, vel_r)
        
        # Warten bis sich die Encoder bewegen
        init_l, init_r = self.left_ticks, self.right_ticks
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.left_ticks != init_l or self.right_ticks != init_r:
                break  # erster Tick angekommen
            rate.sleep()
        
        # Startwert setzen
        start_l, start_r = self.left_ticks, self.right_ticks
        
        while not rospy.is_shutdown():
            delta = (abs(self.left_ticks - start_l) + abs(self.right_ticks - start_r)) / 2
            if delta >= ticks:
                break
            rate.sleep()

        delta_l = abs(self.left_ticks - start_l)
        delta_r = abs(self.right_ticks - start_r)
        rospy.loginfo(f"Effektiv: left={delta_l}, right={delta_r}, durchschnitt={(delta_l+delta_r)/2:.1f}")
    
        
        self.set_wheels(0.0, 0.0)
        rospy.sleep(3.0)

    def to_ticks(self, distance):
        return int(distance / (2* math.pi * WHEEL_RADIUS) * TICKS_RESOLUTION)

    def rotate(self, angle):
        ticks = self.to_ticks(abs(angle) * WHEELBASE / 2)
        rospy.loginfo(f"Rotation: {math.degrees(angle):.1f} Grad, theoretische Ticks={ticks}")
        self.drive_ticks(ticks, -0.2 if angle > 0 else 0.2, 0.2 if angle > 0 else -0.2)

    def run(self, goal_x, goal_y):
        angle = math.atan2(goal_y, goal_x)
        distance = math.sqrt(goal_x ** 2 + goal_y ** 2)

        self.rotate(angle)
        self.drive_ticks(self.to_ticks(distance), 0.3, 0.3)
        rospy.loginfo("done!")

if __name__ == '__main__':
    robot = DriveToGoal(os.getenv('VEHICLE_NAME', 'pi'))
    robot.run(goal_x = 0.24, goal_y = 0.29)
    #robot.rotate(math.pi / 2) # Rotation optisch testen
