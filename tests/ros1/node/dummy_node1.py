import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool


def callback(msg):
    pass


def main():
    rospy.init_node("dummy_node1", disable_rosout=True)
    rospy.Publisher("topic", String, queue_size=1)
    pub = rospy.Publisher("pub", Int32, queue_size=1)
    rospy.Subscriber("sub", Image, callback)
    rospy.Subscriber("sub_any", rospy.AnyMsg, callback)
    rospy.Service("server", SetBool, callback)
    rospy.ServiceProxy("client", SetBool)

    def timer(_e):
        msg = Int32()
        msg.data = 42
        pub.publish(msg)

    rospy.Timer(rospy.Duration(0.01), timer)
    rospy.spin()


if __name__ == "__main__":
    main()
