#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
import math
import os
import rospkg

class Robot:

    def __init__(self):
        rospy.init_node('astar_publisher', anonymous=True)
        self.t = Twist()
        self.pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
        rospy.Subscriber("startflag", Bool, self.callback)
        self.rate = rospy.Rate(1)
        self.rate_ = 1
        self.wheel_radius= 0.033
        self.wheel_distance= 0.16
        self.flag=False
        

    def publisher(self,states):
        
        theta = 0
        x_vel = 0
        y_vel = 0
        ang_vel = 0
        if self.flag==True:

            for i in range(5):
                self.rate.sleep()

            for right_rpm,left_rpm in states:

                # left_velocity = left_rpm * wheel_radius * 2 * 3.14 / 60
                # right_velocity = right_rpm * wheel_radius * 2 * 3.14 / 60

                # # Compute the robot's linear and angular velocities
                # x_vel = (left_velocity + right_velocity) / 2
                # ang_vel = (right_velocity - left_velocity) / wheel_distance
                
                ang_vel =(self.wheel_radius/self.wheel_distance)*(right_rpm - left_rpm)
                x_vel = 0.5*self.wheel_radius*(right_rpm+left_rpm)*math.cos(theta*3.14/180)
                y_vel = 0.5*self.wheel_radius*(right_rpm+left_rpm)*math.sin(theta*3.14/180)
                x_robot = (x_vel**2 + y_vel**2)**0.5

                for i in range(10):
                    theta = theta + ang_vel*0.1
                theta = theta%(math.pi)
                # data_str = "Pubishing Commands to Topics, " + str(x_vel) + " , "+ str(ang_vel)+ ","+str(theta)
                # rospy.loginfo(data_str)
                print(str(x_robot), " , ", str(ang_vel), ","+str(theta), ",", left_rpm, right_rpm)
                self.t.angular.z = self.rate_*ang_vel/1.6
                self.t.linear.x = x_robot/7.0
                self.pub_move.publish(t)
                self.rate.sleep()

    def callback(self,msg):
        
        if msg.data==True:
            self.flag=True
        else:
            self.flag=False


if __name__ == '__main__':

    
    r=Robot()
    read = False
    states = []
    while(r.flag==False):
        # rospy.loginfo("Waiting for path to be generated")
        pass
    while (read == False):
        try:
            # print('Inside try')
            rp = rospkg.RosPack()
            package_path = rp.get_path("astar_turtlebot")
            print(package_path)
            filepath = os.path.join(package_path,'src/actions.txt')
            print(filepath)
            with open(filepath, 'r') as f:
                # print('Inside while')
                # Read the lines of the file
                lines = f.readlines()
                lines = [line.strip() for line in lines]
                for line in lines:
                    values_list = line.split()
                    values_tuple = (int(values_list[0]), int(values_list[1]))
                    states.append(values_tuple)
                read = True
        except:
            continue
    states.append((0,0))
    
    try:
        r.publisher(states)
    except rospy.ROSInterruptException:
        pass