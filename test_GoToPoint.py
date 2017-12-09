from kubs import *
from run import *
import rospy
sys.path.append('./../../plays_py/scripts/utils/')
from geometry import Vector2D
from krssg_ssl_msgs.msg import point_2d
from config import *

BOT_ID = 0
pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)
kub = kubs.kubs(BOT_ID, pub)
start_time = rospy.Time.now()
start_time = 1.0*start.secs + 1.0*start.nsecs/pow(10,9)
GOAL_POINT = point_2d()
GOAL_POINT.x = 1000
GOAL_POINT.y = 1200
REPLANNED = 0
homePos = None
awayPos = None

def reset():
	global start_time
	start_time = rospy.Time.now()
	start_time = 1.0*start.secs + 1.0*start.nsecs/pow(10,9)

def BS_callback(data):
	global homePos, REPLANNED
	global awayPos
	homePos = data.homePos
	awayPos = data.awayPos
	t = rospy.Time.now()
	t = t.secs + 1.0*t.nsecs/pow(10,9)

	[vx, vy, vw, REPLANNED] = Get_Vel(start_time, t, BOT_ID, GOAL_POINT, homePos, awayPos)	#vx, vy, vw, replanned
	
	if(REPLANNED):
		reset()
		REPLANNED = 0

	kub.move(vx, vy, vw)
	kub.execute()

if __name__ == "__main__":
	rospy.init_node('node_new',anonymous=False)
	pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)	
	rospy.Subscriber('/belief_state', BeliefState, BS_Callback, queue_size=1000)