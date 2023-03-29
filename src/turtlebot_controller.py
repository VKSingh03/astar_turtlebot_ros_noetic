#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

def publisher():
    t = Twist()
    # pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
    rospy.init_node('astar_publisher', anonymous=True)
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     t.angular.z = 0.2
    #     t.linear.x = 1
    #     pub_move.publish(t)
    #     rate.sleep()

    list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
    req = ListControllersRequest()
    resp = list_controllers(req)
    left_wheel_controller = ''
    right_wheel_controller = ''
    for controller in resp.controller:
        if controller.name == 'left_wheel':
            left_wheel_controller = controller.name
        elif controller.name == 'right_wheel':
            right_wheel_controller = controller.name

    # Switch the wheel controllers to RPM mode
    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    req = SwitchControllerRequest()
    req.start_controllers = [left_wheel_controller, right_wheel_controller]
    req.stop_controllers = []
    req.strictness = 2 # BEST_EFFORT
    switch_controller(req)

    left_wheel_pub = rospy.Publisher('/left_wheel/command', Float64, queue_size=10)
    right_wheel_pub = rospy.Publisher('/right_wheel/command', Float64, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        left_wheel_rpm = 10.0 # set the left wheel RPM to 10.0
        right_wheel_rpm = 20.0 # set the right wheel RPM to 20.0
        left_wheel_pub.publish(left_wheel_rpm)
        right_wheel_pub.publish(right_wheel_rpm)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass