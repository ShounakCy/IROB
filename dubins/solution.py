#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Shounak Chakraborty
# {07581534}
# {shounakc@kth.se}

from dubins import *
from math import *

dt = 0.01

class Node(object):

    
    linear_resolution = 0.3
    angular_resolution = 2.0 * pi / 6.0

    def __init__(self, car, x, y, theta, G_cost = 0.0, parent_index = None):
        self.car = car
        self.x = x
        self.y = y
        self.theta = theta
        self.G_cost = G_cost #1
        self.parent_index = parent_index #2

        
        self.H_cost = 0.0 
        self.F_cost = 0.0  

        
        self.time = 0 
        self.phi = 0.2 

    def cost(self):
        #expected ideal cost(distance) to target
        self.H_cost = sqrt((pow((self.x - self.car.xt),2) + pow((self.y - self.car.yt),2)))
        self.F_cost = self.G_cost + self.H_cost
    
    def verify_distance_goal(self):
        verify_distance = sqrt((pow((self.x - self.car.xt),2) + pow((self.y - self.car.yt),2)))
        print("verify_distance", verify_distance)
        if verify_distance <= 1.5:
            return True
        else: 
            return False


    
def calc_grid_index(node):
    # lineat resolution = 0.2
    x_index = int(node.x / node.linear_resolution) 
    y_index = int(node.y / node.linear_resolution)
    # angular resolution = 2 pi / 6 (6 states / grid)
    theta_pos = node.theta % (2.0 * pi)
    theta_index = int(theta_pos / Node.angular_resolution)
    return (x_index, y_index, theta_index)

def verify_node(car, x, y):
    if x > car.xub:
        return False
    if x < car.xlb:
        return False
    if y > car.yub:
        return False
    if y < car.ylb:
        return False
    for ob in car.obs:
        if sqrt(pow((x - ob[0]),2) + pow((y - ob[1]),2)) < (ob[2] + 0.1): 
            return False
    return True

def solution(car):

    ''' <<< write your code below >>> '''
    
    #list of turning angle
    controls=[] 
    
    times=[]
    #initialize start node, initialize cost function
    theta0 = 0.0
    nstart = Node(car, car.x0, car.y0, theta0) 
    nstart.cost()
    print("start position: (", nstart.x, ", ", nstart.y, ")")
    print("goal position: (", car.xt, ", ", car.yt, ")")
    #initial open set, closed set:
    open_set, closed_set = dict() , dict()
    
    #add start node into openset
    open_set[calc_grid_index(nstart)] = nstart
    c = 1
    #while loop to explore the path when openset is not empty
    while len(open_set) != 0:
       

        #choose the node with smallest cost value from open set
        current_index = min(open_set, key=lambda i: open_set[i].F_cost)
        current_node = open_set[current_index]     

        #delete the current node from open set
        del open_set[current_index]
       
        #add current node into closed set
        closed_set[current_index] = current_node
       
        #check if current node reach the goal or run too many loops
        #if current_node.verify_distance_goal() :
        print("current_node.verify_distance_goal()", current_node.verify_distance_goal())
        if current_node.verify_distance_goal() == True:
                print("Find goal")
                print("current_node x, y = ", current_node.x, ", ", current_node.y)
                print("\n")
                times.insert(0, current_node.time * dt)
                controls.insert(0, current_node.phi)
                while True:
                    if current_node.parent_index == None:
                        # control needs to be poped out if current_node is the start node
                        controls.pop(0)
                        break
                    else:
                        current_node = current_node.parent_index
                        times.insert(0, current_node.time * dt)
                        controls.insert(0, current_node.phi)
                        
                break

        else :
            
            next_nodes(car, current_node, open_set, closed_set)
        c += 1
        print("c = ", c)
                

    ''' <<< write your code below >>> '''
    
    #print(controls)
    #print(times)
    return controls, times

def next_nodes(car, current_node, open_set, closed_set, steps = 45):
    # three steering directions
    steering_angle = [-pi/4.0, 0.0, pi/4.0]
    # generate primitive three next nodes
    for steer in steering_angle: #for all δ do
        #k = -1
        xn = current_node.x
        yn = current_node.y
        thetan = current_node.theta

        # n′ ← succeeding state of n using μ(nθ , δ)
        for i in range(steps):
            k = i 
            xn, yn, thetan = step(car, xn, yn, thetan, steer)
            #check if next step nodes are within boundaries and not near obstacles
            if not verify_node(car, xn, yn):
                k = "break"
                break
      
        # n′ ← succeeding state of n using μ(nθ , δ): initialize neighbor node 
        new_node = Node(car, xn, yn, thetan)
        new_node_index = calc_grid_index(new_node)
        # print("new_node_index = ", new_node_index)
        # if n′ ∈/ C then
        if not new_node_index in closed_set:
            # Check if new_node in obstacle/collision:
                        
            if k == "break":
                #C ← C ∪ {n′}
                closed_set[new_node_index] = new_node 

            #elseif ∃ n∈O: n = n′ then
            elif (k == (steps - 1)) and (new_node_index in open_set):
                #compute new costs g′
                n_G_cost = current_node.G_cost + dt * steps #(distance/timestep = 0.01, velocity = 1)
                new_node.cost() #initialize cost_H and cost
                new_node.parent_index = current_node
                new_node.time = current_node.time + steps
                new_node.phi = steer
                #if g′ < g value of existing node in O then replace existing node in O with n′
                if new_node.F_cost < open_set[new_node_index].F_cost:
                    open_set[new_node_index] = new_node
            #else
            elif k == (steps - 1):
                # O ← O ∪ {n′}
                n_G_cost = current_node.G_cost + dt * steps 
                new_node.cost() #initialize cost_H and cost
                new_node.parent_index = current_node
                new_node.time = current_node.time + steps
                new_node.phi = steer
                open_set[new_node_index] = new_node

            

    return True