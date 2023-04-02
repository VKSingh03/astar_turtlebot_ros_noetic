#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
import math

def publisher(states):
    t = Twist()
    pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
    rospy.init_node('astar_publisher', anonymous=True)
    rate = rospy.Rate(1)
    wheel_radius= 0.033
    wheel_distance= 0.16
    theta = 60
    x_vel = 0
    y_vel = 0
    ang_vel = 0
    # states = [(5,5), (10,5), (5,5), (5, 20), (5,5), (5,5), (10,5)]

    # states = [(0,0), (40,40), (40,50),( 50,40), (40,50), (50,40), (40,50), (50,40), (40,50), (50,40), (40,50), (50,40), (40,50), (50,40)]
    # while not rospy.is_shutdown():
    for i in range(10):
        rate.sleep()
    for left_rpm,right_rpm in states:

        # left_velocity = left_rpm * wheel_radius * 2 * 3.14 / 60
        # right_velocity = right_rpm * wheel_radius * 2 * 3.14 / 60

        # # Compute the robot's linear and angular velocities
        # linear_velocity = (left_velocity + right_velocity) / 2
        # angular_velocity = (right_velocity - left_velocity) / wheel_distance
        
        ang_vel =(wheel_radius/wheel_distance)*(right_rpm - left_rpm)

        x_vel = 0.5*wheel_radius*(right_rpm+left_rpm)*math.cos(theta)
        y_vel = 0.5*wheel_radius*(right_rpm+left_rpm)*math.sin(theta)
        for i in range(10):
            theta = theta + ang_vel*0.1
        theta = round(theta)%360
        data_str = "Pubishing Commands to Topics, " + str(x_vel) + " , "+ str(ang_vel)+ ","+str(theta)
        rospy.loginfo(data_str)
        t.angular.z = ang_vel
        t.linear.x = x_vel
        t.linear.y = y_vel
        pub_move.publish(t)
        rate.sleep()

#     list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
#     req = ListControllersRequest()
#     resp = list_controllers(req)
#     left_wheel_controller = ''
#     right_wheel_controller = ''
#     for controller in resp.controller:
#         if controller.name == 'left_wheel':
#             left_wheel_controller = controller.name
#         elif controller.name == 'right_wheel':
#             right_wheel_controller = controller.name

#     # Switch the wheel controllers to RPM mode
#     switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
#     req = SwitchControllerRequest()
#     req.start_controllers = [left_wheel_controller, right_wheel_controller]
#     req.stop_controllers = []
#     req.strictness = 2 # BEST_EFFORT
#     switch_controller(req)

#     left_wheel_pub = rospy.Publisher('/left_wheel/command', Float64, queue_size=10)
#     right_wheel_pub = rospy.Publisher('/right_wheel/command', Float64, queue_size=10)
#     rate = rospy.Rate(10) # 10 Hz

#     while not rospy.is_shutdown():
#         left_wheel_rpm = 10.0 # set the left wheel RPM to 10.0
#         right_wheel_rpm = 20.0 # set the right wheel RPM to 20.0
#         left_wheel_pub.publish(left_wheel_rpm)
#         right_wheel_pub.publish(right_wheel_rpm)
#         rate.sleep()


if __name__ == '__main__':
    read = False
    values = []
    # rospy.loginfo("Waiting for path to be generated")
    while (read == False):
        try:
            with open('actions.txt', 'r') as f:
                # Read the lines of the file
                lines = f.readlines()
                lines = [line.strip() for line in lines]
                for line in lines:
                    # Split the line into values
                    values_list = line.split()
                    # Convert the values to integers and store them as a tuple
                    values_tuple = (int(values_list[0]), int(values_list[1]))
                    # Append the tuple to the list of values
                    values.append(values_tuple)
                read = True
        except:
            continue
    
    # print(values)
    # exit(0)
    # rospy.loginfo("Path generated: Moving Robot")
    # Remove any newline characters from the lines
    # lines = [line.strip() for line in lines]
    
    # # Create an empty list to store the values
    # values = [(0,0), (40,40), (40,50),( 50,40), (40,50), (50,40), (40,50), (50,40), (40,50), (50,40), (40,50), (50,40), (40,50), (50,40), (0,0)]

    # # Loop through the lines of the file
    # for line in lines:
    #     # Split the line into values
    #     values_list = line.split()
    #     # Convert the values to integers and store them as a tuple
    #     values_tuple = (int(values_list[0]), int(values_list[1]))
    #     # Append the tuple to the list of values
    #     values.append(values_tuple)
    try:
        publisher(values)
    except rospy.ROSInterruptException:
        pass