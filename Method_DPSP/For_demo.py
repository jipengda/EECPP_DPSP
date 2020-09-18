# -*- coding: utf-8 -*-
"""
Created on Thu Mar 12 21:54:56 2020

@author: Jipeng
"""

# For draw a map 4 x 4, and 5 is obstacle , as a demo

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

agentNumber = 10
colNumber=4
rowNumber=4
Battery_capacity_constraint = 5.0
nodesNumber=rowNumber * colNumber
sideLength=1
departurePoint=0
obstacles=[5]
Nodes = [i for i in range(nodesNumber) if i not in obstacles and i!=departurePoint]
#obstacles=[9,10,14,
#           18,21,22,
#           29,30,
#           33,34,38,
#           41,42,
#           49,50,
#           56,57,58,
#           64,65,66]
#obstacles=[30,34,35,40,44,45,54,55,64,65,72,73,74,75,82,83,84]
NodesAndDeparturePoint=[i for i in range(nodesNumber)]
coord_x=[]
coord_y=[]
coord_x=Data.create_coord_x(colNumber,rowNumber)
coord_y=Data.create_coord_y(colNumber,rowNumber)
D=Data.create_D(nodesNumber, coord_x, coord_y)
# change
#Nodes = [i for i in range(nodesNumber) if i != departurePoint and i not in obstacles ]
# end
#start_time = time.time()
#all_qualified_labels=obtain_labels_data.obtain_all_qualified_labels(D, coord_x, coord_y, nodesNumber, Battery_capacity_constraint, departurePoint, obstacles) #add obstacle(m,n)
#label_number = len(all_qualified_labels)
#C=[0] * label_number
#for l in range(label_number):
#    C[l] = all_qualified_labels[l][-1][0]
#a = [[0 for i in range(nodesNumber - 1)] for j in range(label_number)]
#labels = [l for l in range(label_number)]
#
#for i in range(label_number):
#    for j in range(1, nodesNumber):
#        if j in all_qualified_labels[i]:
#            a[i][j-1]=1
#
## An arc flow model for the basic EECPP
#model = Model("EECPP PROBLEM")
#x = model.binary_var_list(labels, name = 'X') # determine if the label is selected(x=1) or not(x=0)
#
#Z = model.sum( (C[l] * x[l]) for l in labels )
#model.minimize(Z)
#
#model.add_constraint(model.sum( x[l] for l in labels )<= agentNumber, ctname = 'agent number limit')
#
#for i in Nodes:
#    model.add_constraint(model.sum( a[l][i-1] * x[l] for l in labels) == 1, ctname = 'Node %d covered'%i) 
#model.parameters.timelimit=3600

#solution = model.solve(log_output = True)
#print("--- %s seconds ---" % (time.time() - start_time))
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
    
#sets=[]
#agent1_set=[]
#agent2_set=[]
#agent3_set=[]
#agent4_set=[]
#agent5_set=[]
#agent6_set=[]
#agent7_set=[]
#agent8_set=[]
#agent9_set=[]
#agent10_set=[]
#for l in labels:
#    if x[l].solution_value > 0.9:
#        sets.append(all_qualified_labels[l])
#agent1_set=sets[0][:-1]
#if len(sets) >=2:
#    agent2_set=sets[1][:-1]
#if len(sets)>=3:
#    agent3_set=sets[2][:-1]
#if len(sets)>=4:
#    agent4_set=sets[3][:-1]
#if len(sets)>=5:
#    agent5_set=sets[4][:-1]
#if len(sets)>=6:
#    agent6_set=sets[5][:-1]
#if len(sets)>=7:
#    agent7_set=sets[6][:-1]
#if len(sets)>=8:
#    agent8_set=sets[7][:-1]
#if len(sets)>=9:
#    agent9_set=sets[8][:-1]
#if len(sets)>=10:
#    agent10_set=sets[9][:-1]
#length = len(agent1_set)
#for i in range(length-1):
#    start = agent1_set[i]
#    end   = agent1_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='blue')
#
#length = len(agent2_set)
#for i in range(length-1):
#    start = agent2_set[i]
#    end   = agent2_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='orange')
#
#length = len(agent3_set)
#for i in range(length-1):
#    start = agent3_set[i]
#    end   = agent3_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='green')
#    
#length = len(agent4_set)
#for i in range(length-1):
#    start = agent4_set[i]
#    end   = agent4_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='red')
#
#length = len(agent5_set)
#for i in range(length-1):
#    start = agent5_set[i]
#    end   = agent5_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='purple')
#
#length = len(agent6_set)
#for i in range(length-1):
#    start = agent6_set[i]
#    end   = agent6_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='brown')
#
#length = len(agent7_set)
#for i in range(length-1):
#    start = agent7_set[i]
#    end   = agent7_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='pink')
#    
#length = len(agent8_set)
#for i in range(length-1):
#    start = agent8_set[i]
#    end   = agent8_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='gray')
#    
#length = len(agent9_set)
#for i in range(length-1):
#    start = agent9_set[i]
#    end   = agent9_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='olive')
#
#length = len(agent10_set)
#for i in range(length-1):
#    start = agent10_set[i]
#    end   = agent10_set[i+1]
#    plt.plot([coord_x[start], coord_x[end]],[coord_y[start], coord_y[end]], color='cyan')
plt.savefig("figure.pdf")
plt.show() 