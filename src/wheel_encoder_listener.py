import rospy
from duckietown_msgs.msg import WheelEncoderStamped

def callback(msg):
    rospy.loginfo(f"Ticks: {msg.data}")
    rospy.loginfo(f"Resolution: {msg.resolution}")
    rospy.loginfo(f"Type:" {msg.type})

def listener():
    rospy.init_node('wheel_encoder_listener', anonymous=True)

    rospy.Subscriber("/pi/left_wheel_encoder_driver_node/tick", WheelEncoderStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()