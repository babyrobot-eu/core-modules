import rospy
from std_msgs.msg import String
   
pub = rospy.Publisher('spk_action_rec', String)
rospy.init_node('test')
rospy.sleep(1)
pub.publish('0 %s %s h1 0.6 h2 0.5' % (10,20))
rospy.sleep(6)
pub.publish('1 %s %s h1 0.4 h2 0.6' % (12,20))
rospy.sleep(0.5)
pub.publish('0 %s %s h1 0.6 h2 0.4' % (26,36))
#pub.publish('1 %s %s h1 0.6 h2 0.4' % (31,36))
#rospy.sleep(0.2)
#rospy.sleep(2)
#pub.publish('1 %s %s h1 0.3 h2 0.7' % (37,47))
#rospy.sleep(1)
#pub.publish('0 %s %s h1 0.6 h2 0.4' % (37,47))
#rospy.sleep(0.05)
#pub.publish('0 %s %s h1 0.6 h2 0.4' % (47,50))
