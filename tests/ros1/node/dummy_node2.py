import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("dummy_node2", disable_rosout=True)
    rospy.Subscriber("/topic", String)
    rospy.spin()


if __name__ == "__main__":
    main()
