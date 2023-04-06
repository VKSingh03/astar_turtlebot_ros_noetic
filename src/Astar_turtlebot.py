# Project 3 
# Submission By: Shantanu Parab (sparab), Vineet Singh (vsingh03)

# GITHUB Link: https://github.com/VKSingh03/Astar-on-turtlebot.git


# Imports
import math
import numpy as np
import time
from queue import PriorityQueue
import cv2
import argparse
import matplotlib.pyplot as plt

# Creating Node Class
class Astar:
    def __init__(self,width,height,scale,start,goal,robot_clear,obj_clear,step_size):
        
        # Get the video dimensions and FPS
        self.result = cv2.VideoWriter("Astar02.avi", cv2.VideoWriter_fourcc(*'MJPG'), 600, (width, height))
        
        # Define constants
        self.START=start
        self.GOAL=goal
        self.SCALE=scale
        self.WIDTH=width
        self.HEIGHT=height
        self.L=step_size
        self.frame_info=[]
        self.ROBOT=robot_clear
        self.OBJ=obj_clear

        # Define Colors
        self.background=(255,255,255)
        self.obst_color=(0,0,0)
        self.clear_color= (12, 253, 240)
        self.grid_color=(233, 226, 230, 0.2)
        self.start_color=(132, 222, 15, 0.8)
        self.goal_color=(254, 0, 0, 0.8)
        self.explored_color= (64, 188, 237)
        self.track_color= (16, 141, 16)
        self.robot_color= (62, 169, 119)
        self.robot_clear_color= (255, 254, 255)

        # Define Grid Size
        self.grid_size=2

        self.H_PX=self.WIDTH*self.SCALE/self.grid_size
        self.V_PX=self.HEIGHT*self.SCALE/self.grid_size

        self.HEX=self.hexagon(125,300,50)
        self.HEX_CLR=self.hexagon(125,300,50+(self.OBJ))
        self.HEX_ROBO_CLR=self.hexagon(125,300,50+(self.OBJ+self.ROBOT))
        # Triangle
        p1=(25-self.OBJ, 460-self.OBJ)
        p2=(25-self.OBJ, 460+self.OBJ)
        p3=(125,510+self.OBJ)
        p4=(225+self.OBJ, 460+self.OBJ)
        p5=(225+self.OBJ,460-self.OBJ)
        self.TRI_CLR = [p1, p2, p3, p4, p5]
        
        p1=(25-(self.OBJ+self.ROBOT), 460-(self.OBJ+self.ROBOT))
        p2=(25-(self.OBJ+self.ROBOT), 460+(self.OBJ+self.ROBOT))
        p3=(125,510+(self.OBJ+self.ROBOT))
        p4=(225+(self.OBJ+self.ROBOT), 460+(self.OBJ+self.ROBOT))
        p5=(225+(self.OBJ+self.ROBOT),460-(self.OBJ+self.ROBOT))
        self.TRI_ROBO_CLR = [p1, p2, p3, p4, p5]
        
        p1=(25,460)
        p2=(225, 460)
        p3=(125,510)
        self.TRI=[p1,p2,p3]
    
    def line_equation(self,p,p1,p2):
        try:
            m=(p2[1]-p1[1])/(p2[0]-p1[0])
            value=p[1]-p1[1]-(m*p[0])+(m*p1[0])
        except:
            value=p[0]-p1[0]
            
        return value
    
    def hexagon(self,cx, cy, sl):
            # Calculate the points of the hexagon
            center_x = cx
            center_y = cy
            side_length = sl
            angle_deg = 60
            angle_rad = math.radians(angle_deg)
            angle_deg = 60
            angle_rad = math.radians(angle_deg)
            points = []
            for i in range(6):
                x = round(center_x + side_length * math.cos(angle_rad * i))
                y = round(center_y + side_length * math.sin(angle_rad * i))
                points.append((x, y))
            return points
    
    def obstacle_detect_triangle(self,p,v1,v2,v3):
        h1=self.line_equation(p,v2,v3)>=0 if self.line_equation(v1,v2,v3)>0 else self.line_equation(p,v2,v3)<=0
        h2=self.line_equation(p,v1,v3)>=0 if self.line_equation(v2,v1,v3)>0 else self.line_equation(p,v1,v3)<=0
        h3=self.line_equation(p,v1,v2)>=0 if self.line_equation(v3,v1,v2)>0 else self.line_equation(p,v1,v2)<=0
        print(h1,h2,h3)
        return h1 and h2 and h3

    def obstacle_detect_rectangle(self,point,x,y,width,height):
        if point[0]>=x and point[1]>=y and point[0]<=(x+width) and point[1]<=(y+height):
            return True
        else:
            return False
    
    def obstacle_detect_boundary(self,point,x,y,width,height):
        if point[0]<=x or point[1]<=y or point[0]>=(x+width) or point[1]>=(y+height):
            return True
        else:
            return False
        
    def obstacle_detect_polygon(self,p,points):
        points=points+points[:3]
        for i in range(len(points)-3):
            v1,v2,v3=points[i],points[i+1],points[i+2]
            h=self.line_equation(p,v1,v2)>=0 if self.line_equation(v3,v1,v2)>0 else self.line_equation(p,v1,v2)<=0
            if not h:
                return False
        return True
        
    def obstacle_detect_circle(self,p, yc, xc, r):
        if ((p[1] - xc) ** 2 + (p[0] - yc) ** 2)<=r**2:
            return True
        return False
    
    def check_obstacle(self,p):
        h1=self.obstacle_detect_rectangle(p,0,250,125,15)
        h2=self.obstacle_detect_rectangle(p,75,150,125,15)
        # h3=self.obstacle_detect_polygon(p,self.HEX)
        # h4=self.obstacle_detect_polygon(p,self.TRI)
        h5=self.obstacle_detect_circle(p,110,400,50)

        return (h1 or h2 or h5 )
    
    def check_clearance(self,p):
        h1=self.obstacle_detect_rectangle(p,0,250-self.OBJ,125+self.OBJ,15+2*self.OBJ)
        h2=self.obstacle_detect_rectangle(p,75-self.OBJ,150-self.OBJ,125+self.OBJ,15+2*self.OBJ)
        # h3=self.obstacle_detect_polygon(p,self.HEX_CLR)
        # h4=self.obstacle_detect_polygon(p,self.TRI_CLR)
        h5= self.obstacle_detect_boundary(p,self.OBJ,self.OBJ,self.HEIGHT-2*self.OBJ,self.WIDTH-2*self.OBJ)
        h6=self.obstacle_detect_circle(p,110,400,50+self.OBJ)
        
        return (h1 or h2 or h5 or h6)
    
    def check_robot(self,p):
        h1=self.obstacle_detect_rectangle(p,0,250-(self.OBJ+self.ROBOT),125+(self.OBJ+self.ROBOT),15+2*(self.OBJ+self.ROBOT))
        h2=self.obstacle_detect_rectangle(p,75-(self.OBJ+self.ROBOT),150-(self.OBJ+self.ROBOT),125+(self.OBJ+self.ROBOT),15+2*(self.OBJ+self.ROBOT))
        # h3=self.obstacle_detect_polygon(p,self.HEX_ROBO_CLR)
        # h4=self.obstacle_detect_polygon(p,self.TRI_ROBO_CLR)
        h5= self.obstacle_detect_boundary(p,(self.OBJ+self.ROBOT),(self.OBJ+self.ROBOT),self.HEIGHT-2*(self.OBJ+self.ROBOT),self.WIDTH-2*(self.OBJ+self.ROBOT))
        h6=self.obstacle_detect_circle(p,110,400,50+self.OBJ+self.ROBOT)
        return (h1 or h2 or h5 or h6)

    def make_obstacle_space(self):
    
        grid=np.ones((self.HEIGHT,self.WIDTH,3),np.uint8)
        for i in range(self.HEIGHT):
            for j in  range(self.WIDTH):
                    if self.check_robot((i,j)):
                        grid[i][j]=np.array(self.robot_clear_color)
                        if self.check_clearance((i,j)):
                            grid[i][j]=np.array(self.clear_color)
                            if self.check_obstacle((i,j)):
                                grid[i][j]=np.array(self.obst_color)
                    else:
                        grid[i][j]=np.array(self.background)
        
        return grid

    def game(self):
        self.running=True
        if self.START[0]>=0 and self.START[0]<self.HEIGHT and self.START[1]>=0 and self.START[1]<self.WIDTH:
            if self.check_robot(self.START):
                print("Invalid Start")
                self.running=False
            else:
                print("Valid Start")
        else:
            self.running=False
            print(" Start Out of Bounds")

        if self.GOAL[0]>=0 and self.GOAL[0]<self.HEIGHT and self.GOAL[1]>=0 and self.GOAL[1]<self.WIDTH:
            if self.check_robot(self.GOAL):
                print("Invalid Goal")
                self.running=False
            else:
                print("Valid Goal")
        else:
            self.running=False
            print(" Goal Out of Bounds")
        if self.running:
            start_time = time.time()
            self.img = self.make_obstacle_space()
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"\nGrid formation Time: {elapsed_time:.2f} seconds")
            self.t,self.c= self.astar()
            if self.t is None:
                print("No Path to track")
                return
            self.video()
            self.img=self.back_track(self.t,self.c,self.img)
            for i in range(3000):
                flip_img = cv2.flip(self.img, 0)
                self.result.write(flip_img)
            flip_img = cv2.flip(self.img, 0)
            cv2.imshow("Out", flip_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def back_track(self,tracker,current,img):
        g_node=current
        points=[]
        points.append(current[6])
        actions=[]
        actions.append(current[7])
        while not g_node[3]==0:
            for c in tracker:
                if c[3] == g_node[2]:
                    points.append(c[6])
                    actions.append(c[7])
                    g_node=c
        print("Complete")
        points.reverse()
        actions.reverse()
        with open('actions.txt', 'w') as file:
        # Write each sublist to the file, with elements separated by a space
            for row in actions:
                line = ' '.join(str(elem) for elem in row) + '\n'
                file.write(line)
        path=[]
        for p in points:
            path+=p
        for i in range(len(path) - 1):
            img=cv2.line(img, path[i], path[i+1], self.track_color, thickness=2)
        return img
    
        
    def action_set(self,current,action):
        t=0
        r= 0.038*10
        L= 0.354*10
        dt=0.1
        xg=current[0]
        yg=current[1]
        thetag=np.deg2rad(current[2])
        del_xn,del_yn=0,0
        D=0
        track=[(yg,xg)]
        while t<1:
            t=t+dt
            del_xn += 0.5*r*sum(action)*math.cos(thetag)*dt
            del_yn += 0.5*r*sum(action)*math.sin(thetag)*dt
            track.append((int(yg+del_yn),int(xg+del_xn)))
            thetag+=(r/L)*(action[1]-action[0])*dt
            D=D+math.sqrt(math.pow(0.5*r*sum(action)*math.cos(thetag)*dt,2)+math.pow(0.5*r*sum(action)*math.sin(thetag)*dt,2))
        thetag=np.rad2deg(thetag)
        xg=round(xg+del_xn)
        yg=round(yg+del_yn)
        thetag=round(thetag)%360
        if xg>=0 and yg>=0 and xg<self.HEIGHT and yg<self.WIDTH:
#       Cost,CostToGoal,Parent,Idx,State,CostToCome,Track
                if self.check_robot((xg,yg)):
                    return [-1,None,None,None,(xg,yg,thetag),None,None,action],D,track
                else:
                    return [np.Inf,np.Inf,None,None,(xg,yg,thetag),np.Inf,None,action],D,track
        else:
            return None,None,None
    
    def draw_vector(self,current,neighbor):
        points=neighbor[6]
        cv2.circle(self.img, (neighbor[4][1],neighbor[4][0]), 1, (0,0,240), -1)
        for i in range(len(points) - 1):
            self.img=cv2.line(self.img, points[i], points[i+1], self.robot_color, thickness=1)
       
    def cost_to_goal(self,x,y):
        return math.sqrt((self.GOAL[0]-x)**2 + (self.GOAL[1]-y)**2)
    
    def check_goal(self, current):
        # Calculate the distance between the point and the center of the circle using the Pythagorean theorem
        distance = ((current[0] - self.GOAL[0]) ** 2 + (current[1] - self.GOAL[1]) ** 2) ** 0.5
        # If the distance is less than or equal to the radius of the circle, the point lies within the circle
        if distance <= 7.5:
            return True
        else:
            return False

    def check_dup(self,current, close_list):
        for node in close_list:
            if ((current[0] - node[0]) ** 2 + (current[1] - node[1]) ** 2)<=16:
                return True
        return False
    
    
    def video(self):
        for current,neighbor in self.frame_info:
            self.draw_vector(current,neighbor)
            flip_img = cv2.flip(self.img, 0)
            for i in range(10):
                self.result.write(flip_img)
                 
    def astar(self):
        start_time = time.time()
        idx=1
        # If weight>1 then Algorithm runs as weighted AStar
        weight = 2
        cv2.circle(self.img, (self.START[1],self.START[0]), 7, (0,0,240), 2)
        cv2.circle(self.img, (self.GOAL[1],self.GOAL[0]), 10, (255,0,0), 2)
        start_cost_to_goal=math.sqrt((self.GOAL[0]-self.START[0])**2 + (self.GOAL[1]-self.START[1])**2)
        start=(start_cost_to_goal,start_cost_to_goal,0,0,self.START,0,[(self.START[1],self.START[0])],[0,0])
        goal=(float('inf'),0,None,None,self.GOAL,None)
        # Left Wheel
        RPM1=int(input("Enter RPM1 (Left wheel RPM): "))
        # Right Wheel
        RPM2=int(input("Enter RPM2 (Right wheel RPM): "))
#         RPM1,RPM2=30,40
        actions=[[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]]
        # Create an open list
        open_list = PriorityQueue()
        # Create a close list
        close_list=[]
        # Create a tracker for path node with compatible data type
        tracker=[]
        current=start
        open_list.put(start)
        self.visited= np.zeros((self.HEIGHT, self.WIDTH, 360))
        self.visited[self.START[0],self.START[1],self.START[2]]=start_cost_to_goal
        while open_list.queue and not self.check_goal(current[4]) :
            current=open_list.get()
            close_list.append(current[4])
            tracker.append(current)
            if self.check_goal(current[4]):
                    break
            else:
                for a in actions:
                    neighbor,cost_of_action,track=self.action_set(current[4],a)
                    
                    if not neighbor==None:    
                            if not self.check_dup(neighbor[4],close_list)  and not neighbor[0]==-1:
                                self.frame_info.append([current,neighbor])
#                                 Parent
                                neighbor[2]=current[3]
#                                 Cost to Come
                                neighbor[5]=current[5]+cost_of_action
#                                 Cost
                                neighbor[0]=neighbor[5]+self.cost_to_goal(neighbor[4][0],neighbor[4][1])*weight
                                neighbor[1]=self.cost_to_goal(neighbor[4][0],neighbor[4][1])
                                idx=idx+1
                                neighbor[3]=idx
                                neighbor[6]=track
                                # self.draw_vector(current,neighbor)
                                # flip_img = cv2.flip(self.img, 0)
                                # cv2.imshow('Frame',flip_img)
                                # cv2.waitKey(1)
                                open_list.put(tuple(neighbor))
                                self.visited[neighbor[4][0],neighbor[4][1]]=neighbor[0]

                            else:
                                if self.visited[neighbor[4][0],neighbor[4][1],neighbor[4][2]]>current[5]+cost_of_action:
                                    neighbor[2]=current[3]
                                    neighbor[5]=current[5]+cost_of_action
                                    neighbor[6]=track
                                    neighbor[0]=neighbor[5]+self.cost_to_goal(neighbor[4][0],neighbor[4][1])*weight
                                    self.visited[neighbor[4][0],neighbor[4][1],neighbor[4][2]]=neighbor[0]

        if not open_list.queue:
            print("Goal Not Found")
            return None, None
        
        cv2.destroyAllWindows()
        tracker.append(current)   
#         print("Tracker: ",tracker)
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"\nExecution time of algorithm: {elapsed_time:.2f} seconds")
        return tracker,current
    

if __name__ == "__main__":

    # Parameters to accept start and goal
    parser = argparse.ArgumentParser()
    # print("Enter start and goal points as per ROS map coordinates: ")
    parser.add_argument("--InitState",nargs='+', type=float, help = 'Initial state for the matrix')
    parser.add_argument("--GoalState",nargs='+', type=float, help = 'Goal state for the matrix')
    Args = parser.parse_args()
    initial_point = Args.InitState
    goal_point = Args.GoalState
    init_deg = initial_point[2]
    goal_deg = goal_point[2]
    # Converting inputs from list to tuple and integer 
    start=(int(initial_point[1]*100+100),int(initial_point[0]*100+50),int(init_deg))
    goal=(int(goal_point[1]*100+100),int(goal_point[0]*100+50),int(goal_deg))
    # print(start, ", ", goal)
    robot_clear = int(input("Enter Robot Clearance: "))
    object_clear = int(input("Enter Object Clearance: "))
    # step_size = int(input('Enter Step Size: '))
    step_size = 10

    #Creating an instance of A*
    map_width = 600
    map_height = 200
    d_algo = Astar(map_width,map_height,1,start,goal,robot_clear,object_clear,step_size)

    # Call the game method
    d_algo.game()