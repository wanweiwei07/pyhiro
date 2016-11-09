import rospy
from geometry_msgs.msg import Twist

rospy.init_node("vel_publisher")
pub = rospy.Publisher("/base_controller/command", Twist, queue_size = 10)
while not rospy.is_shutdown():
    vel = Twist()
    vel.linear.x = 0.5
    print vel
    pub.publish(vel)