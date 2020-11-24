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
    #    print("verify_distance", verify_distance)
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
    #print("start position: (", nstart.x, ", ", nstart.y, ")")
    #print("goal position: (", car.xt, ", ", car.yt, ")")
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
    #    print("current_node.verify_distance_goal()", current_node.verify_distance_goal())
        if current_node.verify_distance_goal() == True:
              #  print("Find goal")
               # print("current_node x, y = ", current_node.x, ", ", current_node.y)
                #print("\n")
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
        #print("c = ", c)
                

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
    

"""
##############################################3 BREADTH-FIRST-SEARCH ##########
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from dubins import *
car=Car()
dt=0.01
def collisions(x,y,car):
    for obs in car.obs:
        d = math.sqrt((x - obs[0])**2 + (y - obs[1])**2)
        if d <= obs[2] + 0.1:
            return True
    return False
def replace_array(a):
    new = []
    for i in a:
        new.append(i)
    return new
def outbounds(x,y,car):

    if (car.xlb <= x <= car.xub and car.ylb <= y <= car.yub ):
        return False
    return True
def exact_position211(x,y,phi,theta,car,controls,times,thresh):

    cost=0
    for i in range(100 if phi == 0 else 157):
        x,y,theta=step(car,x,y,theta,phi)
        while theta >= math.pi:
            theta -= 2*math.pi
        while theta <= -2*math.pi:
            theta += math.pi  
        controls.append(phi)
        times.append(times[-1] + dt)
        if collisions(x,y,car) or outbounds(x,y,car):
            return False, 0, 0, 0, controls, times, 999999
        if math.sqrt((x - car.xt)**2 + (y - car.yt)**2) <= thresh:
            return True, x, y, theta, controls, times, 0
        cost = math.sqrt((x - car.xt)**2 + (y - car.yt)**2)
    return True, x, y, theta, controls, times, cost



def mapping123(car, path, visited):
    threshold = 0.2
    queue = [[car.x0,car.y0,0,[],[0],math.sqrt((car.x0 - car.xt)**2 + (car.y0 - car.yt)**2)]]
    queue1 = []
    while len(queue) > 0:
        x,y,theta,controls,times,_ = queue.pop(0)
        if math.sqrt((x - car.xt)**2 + (y - car.yt)**2) <= threshold:
            return controls, times
        visited.append([round(x,1), round(y,1)])
        for phi in [-math.pi/4, 0, math.pi/4]:
            useable, x1, y1, theta1, controls1, times1, cost = exact_position211(x,y,phi,theta,car,replace_array(controls),replace_array(times),threshold)
            if useable and not [round(x1,1), round(y1,1), round(theta1,1)] in queue1:
                path.append(phi)
                queue1.append([round(x1,1), round(y1,1), round(theta1,1)])
                queue.append([x1, y1, theta1, controls1, times1, cost])
            queue.sort(key=lambda x: x[5])
    return [],[0]


def solution(car):

    
    
    controls=[0]
    times=[0,1]
    controls, times = mapping123(car, [], [])
    return controls, times

"""
"""
########################################### RRT ##################################3

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Alexander Wallen Kiessling}
# {19970408-0754}
# {akie@kth.se}

# Course DD2410
# Task: Assignment 3
# Info: This assignment concerns planning the path of a Dubin's car.
# The algorithm chosen for this problem is RRT.

# SETUP
# ---------------------

# Import packages
from dubins import *
import numpy as np

# DEFINITIONS
# ---------------------

class Tree(object):
    '''
    Tree structure, vertices are a custom object.
    Iterates to find a path between a starting
    point and a goal point, adhering to certain
    constraints.
    '''
    def __init__(self, car):
        # Constructor
        self.car = car
        self.vertices = [Vertex(self.car.x0,self.car.y0)] #starting point
        self.forward_limit = 20
        self.goal_tolerance = 1
        self.bias = 15

    def build_tree(self):
        while True:
            # Get random point on map (with goal bias, returning goal sometimes)
            random = self.random_state()

            # Find nearest point from tree using indexing
            nearest_index = self.get_nearest(random)
            nearest = self.vertices[nearest_index]

            # Calculate appropriate driving angle
            phi = self.select_input(nearest, random)

            #Initialize data structures and flags
            collision = False
            forward_drive = 0
            x_array = []
            y_array = []
            theta_array = []

            #set current coordinate and heading values
            x_current = nearest.x
            y_current = nearest.y
            theta_current = nearest.theta

            #drive with given steering
            while (not collision) and (forward_drive < self.forward_limit):

                #step a distance correlating to dt time in driving direction
                x_current, y_current, theta_current = step(self.car, x_current, y_current, theta_current, phi)

                #append to arrays if successful drive
                x_array.append(x_current)
                y_array.append(y_current)
                theta_array.append(theta_current)

                #increase counter
                forward_drive = forward_drive + 1

                #check collision status with boundary or obstacle
                if not self.valid_check(x_current, y_current):
                    collision = True
                    forward_drive = forward_drive-1

            #add drive path to tree (if path exists)
            if forward_drive != 0:
                new = Vertex(x_array[forward_drive-1], y_array[forward_drive-1], theta_array[forward_drive-1], phi, forward_drive*0.01 + nearest.dt, nearest_index)
                self.vertices.append(new)

                #check if new vertex point is in goal zone
                distance_x = new.x - self.car.xt
                distance_y = new.y - self.car.yt
                distance = (distance_x**2 + distance_y**2)
                if distance < self.goal_tolerance:
                    break

        #Initialize arrays
        controls = []
        times = []
        last = len(self.vertices) - 1

        #Trace path along vertices, with timestamps
        while self.vertices[last].parent != None:
            vertex = self.vertices[last]
            controls.insert(0,vertex.phi)
            times.insert(0,vertex.dt)
            last = vertex.parent

        times.insert(0,0)
        return controls, times

    def get_nearest(self, random):
        distance_list = [(vertex.x - random[0]) ** 2 + (vertex.y - random[1])** 2 for vertex in self.vertices]
        nearest_vertex = distance_list.index(min(distance_list))
        return nearest_vertex

    def random_state(self):
        if np.random.randint(0, 100) > self.bias:
            return [np.random.uniform(self.car.xlb, self.car.xub), np.random.uniform(self.car.ylb, self.car.yub)]
        return [self.car.xt, self.car.yt]

    def select_input(self, nearest, random):

        #Calculate input angle phi using random and nearest vectors
        n_vector = np.array([np.cos(nearest.theta), np.sin(nearest.theta), 0])
        r_vector = np.array([random[0] - nearest.x, random[1] - nearest.y, 0])
        v_product = np.cross(n_vector, r_vector)
        cos_phi = n_vector.dot(r_vector)/(np.linalg.norm(n_vector)*np.linalg.norm(r_vector))
        steering_angle = np.arccos(cos_phi)

        #Check that phi does not exceed limits
        if v_product[2] < 0:
            steering_angle = steering_angle*-1
            if steering_angle < -1*np.pi/4:
                steering_angle = -1*np.pi/4
        if steering_angle > np.pi/4:
            steering_angle = np.pi/4
        return steering_angle

    def valid_check(self, x, y):

        #Check that coordinates are not inside obstacles
        for (obstacle_x, obstacle_y, obstacle_r) in self.car.obs:
            distance_x = obstacle_x - x
            distance_y = obstacle_y - y
            distance = (distance_x**2 + distance_y**2)
            if distance <= obstacle_r**2 + 0.1:
                return False

        #Check that coordinates are not outside bounds
        if not (self.car.xlb <= x <= self.car.xub - 0.1) or not (self.car.ylb + 0.1 <= y <= self.car.yub - 0.1):
            return False
        return True

class Vertex():
    '''
    Data storage object, stores the
    details of each visited position
    for tree, such that a path can
    be constructed.
    '''
    def __init__(self, x, y, theta = 0, phi = 0, dt = 0, parent = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.dt = dt
        self.parent = parent


def solution(car):
    '''
    Solution function, called by solver method
    for path integration
    '''
    #initialize tree with start vertex
    path_planning = Tree(car)

    #build tree to find feasible path
    controls, times = path_planning.build_tree()

    return controls, times

"""