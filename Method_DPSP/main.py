# -*- coding: utf-8 -*-
"""
Created on Sun Feb 16 19:47:25 2020

@author: jipengda
"""

"""
It is expected to be size 10 x 10, or whose grids is between 80 and 140 grids
"""
from docplex.mp.model import Model
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import obtain_labels_data
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import Data
import time
import random
import math
import numpy as np

#------------------------------------------------------------------------------
# def functions
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
# SUPPLEMENTARY FUNCTIONS
#------------------------------------------------------------------------------
def check_obstacle(obstacles, lastNode, newNode, colNumber, rowNumber):
    sideLength = 1
    flag=1 # initialize flag=1
    for obstacle in obstacles:
        x3 = obstacle % colNumber
        y3 = obstacle // colNumber
        r_xmin = x3 - sideLength/2
        r_xmax = x3 + sideLength/2
        r_ymin = y3 - sideLength/2
        r_ymax = y3 + sideLength/2
        if RSIntersection(r_xmin, r_xmax, r_ymin, r_ymax, lastNode, newNode, colNumber, rowNumber) == 1:
            flag = 0 #flag is variable to replace directly return False of True because we need to test all obstacles
        else:
            flag = 1
        if flag == 0:#once flag change to 0, means one obstacle is hit, thus new node can not be added
            return False
    return True# This means all obstacles are not hit, new node can be considered to be added
"""
 * if [x1, x2] and [x3, x4] (x4 maybe smaller than x3) has interrection or has not，if yes: return 1， if no: return 0
"""


def IntervalOverlap(x1, x2, x3, x4):
    t = 0    
    if x3 > x4:
       t = x3
       x3 = x4	   
       x4 = t
    if x3 >= x2 or x4 <= x1:
        return 0
    else:
        return 1
    
"""
 * judge rectangular r and line segment AB has intersection or not，if yes: return 1，if no: return 0
"""
def RSIntersection(r_xmin,r_xmax,r_ymin, r_ymax, nodeA, nodeB, colNumber, rowNumber):
    A_x = nodeA % colNumber
    A_y = nodeA // colNumber
    B_x = nodeB % colNumber
    B_y = nodeB // colNumber
    if (A_y == B_y):# line segement is parallel to x axis//线段平行于x轴
        if A_y <= r_ymax and A_y >= r_ymin:
            return IntervalOverlap(r_xmin, r_xmax, A_x,B_x)
        else:
            return 0

	# Echange point A and point B to let point B has bigger y coordinate//AB两点交换，让B点的y坐标最大

    # Exchange node A and node B, let B's y value is bigger
    t = 0
    if A_y > B_y:
       t = A_y
       A_y = B_y
       B_y = t
       t= A_x
       A_x = B_x
       B_x=t
	
    # In line segment//xianduan AB, to find point C and D
    # Two points secure a line: (x-x1)/(x2-x1)=(y-y1)/(y2-y1)
    k = (B_x - A_x)/(B_y - A_y)
    if A_y < r_ymin:
       D_y = r_ymin
       D_x = k*(D_y - A_y) + A_x
    else:
       D_y=A_y
       D_x=A_x
    if B_y > r_ymax:
       C_y = r_ymax
       C_x = k*(C_y-A_y) + A_x
    else:
       C_y = B_y
       C_x = B_x
    if C_y >= D_y: # y axis has overlap
       return IntervalOverlap(r_xmin, r_xmax,D_x, C_x)
    else:
       return 0

agentNumber = 10
colNumber=4
rowNumber=4
Battery_capacity_constraint = 11.0 # 6.0 is test
nodesNumber=rowNumber * colNumber
sideLength=1
departurePoint=0
obstacles=[4,8,9,12,13] # no obstacle
Nodes = [i for i in range(nodesNumber) if i not in obstacles and i!=departurePoint]
NodesAndDeparturePoint=[i for i in range(nodesNumber)]
coord_x=[]
coord_y=[]
coord_x=Data.create_coord_x(colNumber,rowNumber)
coord_y=Data.create_coord_y(colNumber,rowNumber)
D=Data.create_D(nodesNumber, coord_x, coord_y)
radians_to_degrees=180/(math.pi)
Nodes=[i for i in range(nodesNumber) if i not in obstacles and i!= departurePoint]
NodesAndDeparturePoint = Nodes + [departurePoint]
edges=[(i,j) for i in NodesAndDeparturePoint for j in NodesAndDeparturePoint]
arcs=[(i,j,k) for i in NodesAndDeparturePoint for j in NodesAndDeparturePoint for k in NodesAndDeparturePoint]
distance_lambda = 0.1164
c={(i,j):0 for i,j in edges}
q={(i,j,k):0 for i,j,k in arcs}
distance={(i,j):0 for i,j in edges}
for i,j in edges:
    distanceValue=np.hypot(coord_x[i]-coord_x[j],coord_y[i]-coord_y[j]) # it is wrong, it does not consider the obstacle between nodes.
    distance[(i,j)]=distanceValue
    distance_cost = distance_lambda * distanceValue
    c[(i,j)] = distance_cost
    

for o,p in edges:
    View = check_obstacle(obstacles, o, p, colNumber, rowNumber)
    if View == 0:
        c[(o,p)] = math.inf
    else:
        pass
    
seq=[-10,-9,-8,-7,-6,-5,-4,-3-2,-1,0,1,2,3,4,5,6,7,8,9,10]
fixed_turn_gamma=0.0173
turn_factor=0.0001 
random.seed(10)   
for i,j,k in arcs:
    turn_gamma = fixed_turn_gamma + random.choice(seq) * turn_factor
    theta_radians=math.pi-np.arccos(round((distance[i,j]**2+distance[j,k]**2-distance[i,k]**2)/(2*distance[i,j]*distance[j,k]),2))
    theta_degrees=theta_radians*radians_to_degrees
    turning_cost=turn_gamma*theta_degrees
    q[(i,j,k)]=turning_cost
    a=math.isnan(turning_cost)
    if a is True:
        turning_cost=0
    else:
        pass
    q[(i,j,k)]=turning_cost

# but how to pass them to obtain_all_qualified_labels functions
# now we have c, q, and d

start_time = time.time()
all_qualified_labels=obtain_labels_data.obtain_all_qualified_labels(q,distance,D, coord_x, coord_y, nodesNumber, Battery_capacity_constraint, departurePoint, obstacles) #add obstacle(m,n)
label_number = len(all_qualified_labels)
C=[0] * label_number
for l in range(label_number):
    C[l] = all_qualified_labels[l][-1][0]
a = [[0 for i in range(nodesNumber - 1)] for j in range(label_number)]
labels = [l for l in range(label_number)]

for i in range(label_number):
    for j in range(1, nodesNumber):
        if j in all_qualified_labels[i]:
            a[i][j-1]=1

# An arc flow model for the basic EECPP
model = Model("EECPP PROBLEM")
x = model.binary_var_list(labels, name = 'X') # determine if the label is selected(x=1) or not(x=0)

Z = model.sum( (C[l] * x[l]) for l in labels )
model.minimize(Z)

model.add_constraint(model.sum( x[l] for l in labels )<= agentNumber, ctname = 'agent number limit')

for i in Nodes:
    model.add_constraint(model.sum( a[l][i-1] * x[l] for l in labels) == 1, ctname = 'Node %d covered'%i) 
model.parameters.timelimit=3600

solution = model.solve(log_output = True)
print("--- %s seconds ---" % (time.time() - start_time))
width = 2*colNumber - 2
height = 2*rowNumber - 2
plt.figure(figsize=(width, height))
plt.xlabel("Coordinate X")
plt.ylabel("Coordinate Y")
plt.title("Solution of DP/SP for Multi-agent EECPP Problem")

for n in NodesAndDeparturePoint:
    if n!=departurePoint:
        plt.scatter(x=coord_x[n],y=coord_y[n],color='blue')
        currentAxis=plt.gca()
        rect=patches.Rectangle( (coord_x[n]-1/2*sideLength,coord_y[n]-1/2*sideLength),sideLength,sideLength,linewidth=1,edgecolor='k',facecolor='none' )
        currentAxis.add_patch(rect)
    else:
        plt.scatter(x=coord_x[n],y=coord_y[n],color='green')
        currentAxis=plt.gca()
        rect=patches.Rectangle( (coord_x[n]-1/2*sideLength,coord_y[n]-1/2*sideLength),sideLength,sideLength,linewidth=1,edgecolor='k',facecolor='none' )
        currentAxis.add_patch(rect)

#for obstacles, first plot in red to mark
for obstacle in obstacles:
    # use x0,y0 to be short for coord_x[obstacle],coord_y[obstacle]
    x0=coord_x[obstacle]
    y0=coord_y[obstacle]
    plt.scatter(x=x0,y=y0,color='red')
    #second, pathes.Rectangle
    currentAxis=plt.gca()
    rect=patches.Rectangle( (x0-1/2*sideLength,y0-1/2*sideLength),sideLength,sideLength,linewidth=1,edgecolor='k',facecolor='grey' )
    currentAxis.add_patch(rect)
#end
    
sets=[]
agent1_set=[]
agent2_set=[]
agent3_set=[]
agent4_set=[]
agent5_set=[]
agent6_set=[]
agent7_set=[]
agent8_set=[]
agent9_set=[]
agent10_set=[]
for l in labels:
    if x[l].solution_value > 0.9:
        sets.append(all_qualified_labels[l])
agent1_set=sets[0][:-1]
if len(sets) >=2:
    agent2_set=sets[1][:-1]
if len(sets)>=3:
    agent3_set=sets[2][:-1]
if len(sets)>=4:
    agent4_set=sets[3][:-1]
if len(sets)>=5:
    agent5_set=sets[4][:-1]
if len(sets)>=6:
    agent6_set=sets[5][:-1]
if len(sets)>=7:
    agent7_set=sets[6][:-1]
if len(sets)>=8:
    agent8_set=sets[7][:-1]
if len(sets)>=9:
    agent9_set=sets[8][:-1]
if len(sets)>=10:
    agent10_set=sets[9][:-1]
length = len(agent1_set)
for i in range(length-1):
    start = agent1_set[i]
    end   = agent1_set[i+1]
    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='blue')

length = len(agent2_set)
for i in range(length-1):
    start = agent2_set[i]
    end   = agent2_set[i+1]
    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='orange')

length = len(agent3_set)
for i in range(length-1):
    start = agent3_set[i]
    end   = agent3_set[i+1]
    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='green')
    
length = len(agent4_set)
for i in range(length-1):
    start = agent4_set[i]
    end   = agent4_set[i+1]
    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='red')

length = len(agent5_set)
for i in range(length-1):
    start = agent5_set[i]
    end   = agent5_set[i+1]
    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='purple')

length = len(agent6_set)
for i in range(length-1):
    start = agent6_set[i]
    end   = agent6_set[i+1]
    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='brown')

length = len(agent7_set)
for i in range(length-1):
    start = agent7_set[i]
    end   = agent7_set[i+1]
    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='pink')
    
length = len(agent8_set)
for i in range(length-1):
    start = agent8_set[i]
    end   = agent8_set[i+1]
    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='gray')
    
length = len(agent9_set)
for i in range(length-1):
    start = agent9_set[i]
    end   = agent9_set[i+1]
    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='olive')

length = len(agent10_set)
for i in range(length-1):
    start = agent10_set[i]
    end   = agent10_set[i+1]
    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='cyan')
plt.savefig("figure.pdf")
plt.show() 
solution.display()