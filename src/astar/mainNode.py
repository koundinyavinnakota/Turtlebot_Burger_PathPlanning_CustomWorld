#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import geometry_msgs.msg

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import math

from queue import PriorityQueue

import cmap
from collections import defaultdict
import cv2
import time



def isInObstacle(point):
    if 100 > point[0] and point[0] > 0 and   point[1] < 100 and point[1]> 0:
        if np.array_equal(cmap.gmap[99-int(point[1]),int(point[0]),0], 255):
            return True
    elif 100 < point[0] or point[0] < 0 or   point[1] > 100 or point[1]< 0:
        return True
    else:

        return False


def cuvrePoints(current_point,UL,UR):
    Xi = current_point[0]
    Yi = current_point[1]
    Thetai = current_point[2]
    t = 0

    r = 3.3
    L = 16
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180
    D = 0
    Xs = []
    Ys = []

    while t<1:
        Xs.append(Xn)
        Ys.append(Yn)
        t = t + dt
        Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Xn += Delta_Xn
        Yn += Delta_Yn
        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
    Thetan = 180 * (Thetan) / 3.14

    return Xs, Ys


def cost(current_point,UL,UR):
    Xi = current_point[0]
    Yi = current_point[1]
    Thetai = current_point[2]
    t = 0

    r = 0.33
    L = 1.6
    r = 3.3
    L = 16
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180
    D = 0
    Xs = []
    Ys = []

    while t<1:
        Xs.append(Xn)
        Ys.append(Yn)
        t = t + dt
        Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Xn += Delta_Xn
        Yn += Delta_Yn
        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
    Thetan = 180 * (Thetan) / 3.14
    
    return [Xn, Yn, Thetan, D, 0]

def getNeighbours(current_point):
    outcomes = []
    for action in actions:
        new_point = cost(current_point,action[0], action[1])
        new_point[4] = actions.index(action)

        if isInObstacle(new_point):
            continue
        else:
            outcomes.append(new_point)

    return outcomes

def costToGo(point, goal_point):
    #Computation of distance between two points
    dist = np.linalg.norm(np.array([point[0],point[1]]-np.array([goal_point[0],goal_point[1]])))

    return dist

def thresh(pt):
    t = 0.5
    return int(pt[0]/t), int(pt[1]/t), int(pt[2]/t)

def plotCurves(points):

    for point in points:
        for action in actions:
            Xs, Ys = cuvrePoints(point, action[0], action[1])
            if isInObstacle([Xs[-1],Ys[-1]]):
                    continue
            else:
                for i in range(1, len(Xs)):
                    cv2.line(cmap.gmap, (int(Xs[i]), 99 - int(Ys[i])), (int(Xs[i-1]), 99 - int(Ys[i-1])), [255, 0, 0], 2)
                    cv2.imshow("Exploration",cv2.resize(cmap.gmap,(500,500)))
                    cv2.waitKey(1)
            # if cv2.waitKey(0) & 0xFF == ord('q'):
            #     break

    
def backTrack(dict, final_point, start_point, image):

    path = [(round(final_point[0],0),round(final_point[1],0), round(final_point[2],0),final_point[4])]
    
    i = (round(final_point[0],0),round(final_point[1],0), round(final_point[2],0),final_point[4])

    dict_keys = list(dict.keys())

    dict_values = list(dict.values())



    reached_start_point = False

    while not reached_start_point:
        for children in dict_values:
            for a in children:

            
                if i[0]==a[0] and i[1]==a[1] and i[2]==a[2]:
                    
                    v = i
                    image[99-int(i[1]),int(i[0])] = (255,0,255)
                    j = dict_values.index(children)
                    path.append(dict_keys[j])
                    i = dict_keys[j]
                    cv2.line(image,(int(v[0]),int(99-v[1])),(int(i[0]),int(99-i[1])),(255,0,255),1)
            if (i[0] == start_point[0]) and (i[1] == start_point[1]):
                return path[::-1]

def takeInput():

    start = input("Input Staring Position in format: x,y,theta\n")
    x_start,y_start,theta_s = int(start.split(',')[0]), int(start.split(',')[1]),int(start.split(',')[2])



    # #Checking if the entered input is valid
    while isInObstacle([x_start,y_start]):
        print("Start point is in obstacle space. Please enter valid point.")
        start = input("Input Staring Position in format: x,y,theta\n")
        x_start,y_start,theta_s = int(start.split(',')[0]), int(start.split(',')[1]),int(start.split(',')[2])

    goal = input("Enter the Goal Position in format: x,y\n")
    x_goal,y_goal = int(goal.split(',')[0]), int(goal.split(',')[1])
        
    while isInObstacle([x_goal,y_goal]) :
        print("Goal point is in obstacle space. Please enter valid point")
        goal = input("Enter the Goal Position in format: x,y\n")
        x_goal,y_goal = int(goal.split(',')[0]), int(goal.split(',')[1])
        
    start_point = [x_start,y_start,theta_s,0,0]
    goal_point = [x_goal,y_goal]

    return start_point, goal_point

def plot_map(points):
        fig, ax = plt.subplots()
        ax.set(xlim=(0, 100), ylim=(0, 100))

        c1 = plt.Circle((20, 80), 10, fill=True)
        c2 = plt.Circle((20, 20), 10, fill=True)
        currentAxis = plt.gca()
        currentAxis.add_patch(Rectangle((2.5, 42.5), 15, 15, fill=True, alpha=1))
        currentAxis.add_patch(Rectangle((37.5, 42.5), 25, 15, fill=True, alpha=1))
        currentAxis.add_patch(Rectangle((72.5, 20), 15, 20, fill=True, alpha=1))
        currentAxis.add_patch(Rectangle((0, 0), 100, 100, fill=None, alpha=1))
       
        ax.add_artist(c1)
        ax.add_artist(c2)
        ax.set_aspect('equal')

        plt.grid()

        for point in points:

            for action in actions:
                Xs, Ys = cuvrePoints(point, action[0], action[1])
                if isInObstacle([Xs[-1],Ys[-1]]):
                    continue
                else:
                    plt.plot(Xs, Ys, color="green")
            plt.pause(0.001)



def velocity_inputs(UL, UR, radius, wheel_distance):
    r = radius
    L = wheel_distance
    rpm_UL = UL*2*math.pi/60
    rpm_UR = UR*2*math.pi/60
    # rpm_UR = UR
    # rpm_UL = UL
    theta_dot = (r / L) * (rpm_UR - rpm_UL) 
    vel_mag = (r / 2) * (rpm_UL + rpm_UR)
    return vel_mag, theta_dot

def move_turtlebot(path, radius, wheel_distance):
    
    msg = geometry_msgs.msg.Twist()
    rospy.init_node('move_tt', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=100)
    msg.linear.x = 0.0
    msg.angular.z = 0.0


    pub.publish(msg)
    rpms = []
    
    for i in range (len(path)):
        rpms.append(actions[path[i][3]])
        rospy.loginfo(rpms)
    c = 0
    # r = rospy.Rate(25)
    for rpm in rpms:
        print("action:", rpm)
        while not rospy.is_shutdown():
            if c == 101:
                msg.linear.x = 0
                msg.angular.z = 0
                pub.publish(msg)
                break
            else:
                vel, th = velocity_inputs(rpm[0],rpm[1], radius, wheel_distance)
                msg.angular.y = 0
                msg.angular.x = 0      
                msg.angular.z =  th/0.9
                msg.linear.x = vel/3.8
                msg.linear.y = 0
                msg.linear.z = 0
                pub.publish(msg)
                time.sleep(0.09)
                c += 1
                
                # r.sleep()
        c = 0


if __name__=='__main__':

    global actions

    x,y = input("Enter the two RPM values: rpm1, rpm2: ").split(',')
    x= float(x)
    y = float(y)
    actions = [[0,x],[x,0],[x,x],[0,y],[y,0],[y,y],[x,y],[y,x]]

    
    c2c = np.full((200,200,720), np.inf)  

    totalCost = np.full((200,200,720),np.inf)


    start_point, end_point = takeInput()
    print("Computing path....")
    

    c2c[thresh(start_point)] = 0




    totalCost[thresh(start_point)] = c2c[thresh(start_point)] +costToGo(list(thresh(start_point)),end_point)

    x = totalCost[thresh(start_point)]

    open = PriorityQueue()

    open.put((x,start_point))

    parent = defaultdict(list)

    visited = np.full((200,200,720),-1)

    explored = []

    while True:

        _, current_point = open.get()
        explored.append(current_point)
        
        # print(current_point)


        if (end_point[0])-5 <= current_point[0] <= (end_point[0])+5 and (end_point[1])-5 <= current_point[1] <= (end_point[1])+5 :
            print("goal reached")

            plotCurves(explored)
            # print("current point at goal:", current_point)

            path = backTrack(parent,current_point, start_point,cmap.gmap)

            print("Path: ",path)
            cv2.imshow("Result ",cv2.resize(cmap.gmap, (500,500)))
            # cv2.waitKey(0)

            if cv2.waitKey(0) & 0xFF == ord('q'):
                break

            break
            
        if visited[thresh(current_point)] != -1 :

            continue

        else:

            new_points = getNeighbours(current_point)

            for point in new_points:
                pt = [point[0],point[1], point[2]]
                newC2C = c2c[thresh(current_point)] + point[3]

                if newC2C < c2c[thresh(pt)]:

                    c2c[thresh(pt)] = newC2C

                    totalCost[thresh(pt)] = c2c[thresh(pt)] + costToGo(point, end_point)


                    parent[(round(current_point[0],0), round(current_point[1],0), round(current_point[2],0),current_point[4])].append((round(point[0],0),round(point[1],0),round(point[2],0),point[4]))
                    
                open.put((totalCost[thresh(pt)], point))

                visited[thresh(current_point)] = 1
    
    try:

        radius = 3.3
        wheel_distance = 16
        move_turtlebot(path,radius,wheel_distance)
    except rospy.ROSInterruptException:
        pass
    

    
