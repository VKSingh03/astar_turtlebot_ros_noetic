#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def publisher():
    t = Twist()
    pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
    rospy.init_node('astar_publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        t.angular.z = 0.2
        t.linear.x = 1
        # data_str = "Pubishing Commands to Topics"
        # rospy.loginfo(data_str)
        pub_move.publish(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass