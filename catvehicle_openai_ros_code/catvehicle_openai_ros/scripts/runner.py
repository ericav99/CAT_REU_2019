#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
from std_srvs.srv import Empty

def pauseSim():
    rospy.logdebug("PAUSING START")
    rospy.wait_for_service('/gazebo/pause_physics')
    try:
        pause()
    except rospy.ServiceException as e:
        print ("/gazebo/pause_physics service call failed")
         
    rospy.logdebug("PAUSING FINISH")
        
def unpauseSim():
    rospy.logdebug("UNPAUSING START")
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause()
    except rospy.ServiceException as e:
        print ("/gazebo/unpause_physics service call failed")
    
    rospy.logdebug("UNPAUSING FiNISH")

def runner():
    pub = rospy.Publisher('/catvehicle/cmd_vel', Twist, queue_size=1)
    rospy.init_node('runner', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        spd = Twist()
        spd.linear.x = 2
        rospy.loginfo(spd)
        pub.publish(spd)
        rate.sleep()
        #time.sleep(1)
        pauseSim()
        time.sleep(2)
        unpauseSim()

if __name__ == '__main__':
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    try:
        runner()
    except rospy.ROSInterruptException:
        pass
