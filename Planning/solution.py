#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Akash Singh
# akashsin@kth.se


import numpy as np
import time
#import matplotlib.pyplot as plt

class Node():

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = 0
        self.parent = None


def closestPoint(wayPointlist, randPoint):
    dist = [(node.x - randPoint[0])**2 + (node.y-randPoint[1])**2 for node in wayPointlist]
    closest = dist.index(min(dist))
    #print("Closest : {}".format(closest))

    return closest

def ObstacleCheck(newPosition, car):
    i=len(car.obs)
    if ((car.xlb+0.2) < newPosition[0] < (car.xub-0.2) and (car.ylb+0.2) < newPosition[1] < (car.yub-0.2)):
        for (x,y,r) in car.obs:
            if car.obs[0][2] == 0.2:
                r = r*3.0
            else :
                r = r*1.5
            if (newPosition[0]-x)**2 + (newPosition[1]-y)**2 <= r:
                #print("Node lies near obstacle. Obstacle is :{}".format(car.obs[-i]))
                return False  #near obstacle
            i=i-1

        #print("A free point")
        return True  #free point

    else:
        #print("Car hit the wall")
        return False



def CollisionCheck(newPosition, prevNode, obslist):
    i=len(obslist)
    A = newPosition[1] - prevNode.y
    B = prevNode.x - newPosition[0]
    C = -1.0*A * newPosition[0] + B * newPosition[1]*-1.0
    D = np.sqrt(A**2 + B**2)
    #print("A : {}, B : {}, C:{}, Sq. sum of coeff :{}".format(A,B,C,D))

    for (x, y, r) in obslist:
        perp_dist = (abs(A * x + B * y + C) / D)
        if B==0:
            m = x
        else :
            m = ((B*(A*y-B*x-(A/B)*C))/(D**2))
        if A==0:
            n = y
        else :
            n = ((A*(A*y-B*x-(B/A)*C))/(D**2))
        #print("Distance from obs:{}, Radius of obs :{}, foot_of_perp :{},{}".format(perp_dist,r,m,n))
        if perp_dist <= (r + 5.0):
            if (prevNode.x < m < newPosition[0] and prevNode.y < n < newPosition[1]):
                print("Collision on the path with obstacle : {}".format(obslist[-i]))
                return False
        i=i-1
    #print("Collision free path")
    return True

def getClosestNodeIndex(xl, yl, target_x, target_y, index):

    #dist = [(x-target_x)**2 + (y - target_y)**2 for (x,y) in zip(xl,yl)]
    #closestNode = dist.index(min(dist))
    #min_dist = min(dist)
    #print("Closest point to {},{} is {},{} at index {} at distance of {}".format(target_x,target_y, xl[closestNode], yl[closestNode], closestNode, min_dist))

    #return closestNode

    closestDist=0.8
    #for (i, x) in enumerate(xl):
    #    y = yl[i]
    if (np.square(xl-target_x) + np.square(yl-target_y)) < np.square(closestDist):
        print("Closest point to {},{} is {},{} at index {}".format(target_x,target_y, xl, yl, index))
        return index-1,True

    #print("Not enough Samples")
    return None, False

def tracebackPath(waypoints, lastindex):
    x_p = []
    y_p = []
    indices = []
    while(waypoints[lastindex].parent is not None):
        #print("current point: {} {}, next index : {} ".format(waypoints[lastindex].x, waypoints[lastindex].y, waypoints[lastindex].parent))
        x_p.append(waypoints[lastindex].x)
        y_p.append(waypoints[lastindex].y)
        indices.append(lastindex)
        lastindex = waypoints[lastindex].parent

    #print(len(x_p), len(y_p), len(indices))
    return x_p, y_p, indices



def DrawGraph(rnd,waypoints,car):
    """
    Draw Graph
    """
    plt.clf()
    if rnd is not None:
        plt.plot(rnd[0], rnd[1], "^k")
    for node in waypoints:
        if node.parent is not None:
            plt.plot([node.x, waypoints[node.parent].x], [
                     node.y, waypoints[node.parent].y], "-g")

    for (ox, oy, size) in car.obs:
        plt.plot(ox, oy, "ok", ms=30*size)

    plt.plot(car.x0, car.y0, "xr")
    plt.plot(car.xt,car.yt, "xr")
    plt.axis([car.xlb, car.xub, car.ylb, car.yub])
    plt.grid(True)
    plt.pause(0.00001)


def solution(car):

    # initial conditions
    x, y = car.x0, car.y0
    theta = 0
    phi = 0.1
    t = 0

    n = 100
    # numerical integration
    xl, yl, thetal, phil, tl = [x], [y], [theta], [], [t]

    #RRT
    start_node = Node(x,y,theta)
    wayPoints = [start_node]
    lastIndex = [None, False]
    delay = 48
    close_time = time.time() + delay

    while lastIndex[1] is False:
        if time.time() > close_time:
        #if len(wayPoints)==10000:
            break

        randPoint = [np.random.uniform(car.xlb, car.xub), np.random.uniform(car.ylb,car.yub), np.random.uniform(-np.pi/4, np.pi/4)]
        #print("Random points : {}".format(randPoint))

        nearestPoint = closestPoint(wayPoints, randPoint)


        #print("CLosest Point :{}, {}, Last waypoint :{}, {}, ListLength :{}".format(wayPoints[nearestPoint].x, wayPoints[nearestPoint].y, wayPoints[-1].x, wayPoints[-1].y, len(wayPoints)))

        newPosition = [[0, 0, 0]]
        newPosition[0] = car.step(wayPoints[nearestPoint].x, wayPoints[nearestPoint].y, wayPoints[nearestPoint].theta,
                                  randPoint[2])

        #taking 10 steps
        if car.obs[0][2] == 0.2 :
            steps = 150
        else:
            steps = 80

        i = 1
        while i < steps+1 :
            #print("i : {}, prev x:{}, prev y:{}, prev theta:{}".format(i, newPosition[i-1][0],newPosition[i-1][1],newPosition[i-1][2]))
            newPosition.append(car.step(newPosition[-1][0], newPosition[-1][1], newPosition[-1][2], randPoint[2]))
            i=i+1

        #print("NewPosition : {}".format(newPosition[-1]))


        #obstacle check for newPosition, if newPosition or path lies inside area of obs
        if ObstacleCheck(newPosition[-1], car):
            #if CollisionCheck(newPosition[-1], wayPoints[nearestPoint], car.obs):
            newNode = Node(newPosition[-1][0], newPosition[-1][1], newPosition[-1][2])
            newNode.parent = nearestPoint
            wayPoints.append(newNode)
            #print("New node found : {}, tl : {}".format(wayPoints.index(newNode), tl[-1]))

            xl.append(newPosition[-1][0])
            yl.append(newPosition[-1][1])
            thetal.append(newPosition[-1][2])
            phil.append(randPoint[2])
            tl.append(tl[-1] + car.dt)
            #print(len(wayPoints))
            #print(len(phil))
            lastIndex = getClosestNodeIndex(newPosition[-1][0], newPosition[-1][1], car.xt, car.yt, len(wayPoints))
            #print("Check xl entry ({},{}) and waypoint entry ({},{})".format(xl[-1], yl[-1], wayPoints[-1].x,wayPoints[-1].y))

        #DrawGraph(randPoint, wayPoints, car)
                #return phil, tl
    #generate track from the list collected

    if lastIndex[1] == False:
        print("NO path found in this sampling")
        return [0], [0,0.01]

    path = tracebackPath(wayPoints, lastIndex[0])
    #print(path[2][0],path[2][-1])

    #plt.plot(path[0], path[1], "k")
    #plt.pause(2)

    steeringlist = []
    times = [0]

    #print(phil[lastIndex[0]-1])
    for i in path[2] :
        for n in range(0,steps):
            steeringlist.append(phil[i-1])
            times.append(times[-1]+car.dt)

    controls = list(reversed(steeringlist))
    #print(times)

    #print("Number of sampled points {}".format(len(xl)))
    #print(xl)
    #print(yl)

    return controls, times
