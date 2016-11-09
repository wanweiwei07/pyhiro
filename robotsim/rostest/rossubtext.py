import rospy
from std_msgs.msg import String

def callback(message):
    rospy.loginfo("I heard %s", message.data)

rospy.init_node("listener")
pub = rospy.Subscriber("chatter", String, callback)
rospy.spin()