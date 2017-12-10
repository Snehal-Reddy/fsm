import rospy
from geometry import Vector2D
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import GoToPoint
from kubs import kubs
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)


kub = kubs.kubs(0,pub)
g_fsm = GoToPoint.GoToPoint(kub)
g_fsm.spin()