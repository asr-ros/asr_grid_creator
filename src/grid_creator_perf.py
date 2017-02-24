#!/usr/bin/env python

'''
Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import sys
import rospy
import next_best_view
import subprocess, os, signal
import xml.etree.cElementTree as ET
import math
import rospkg
import datetime


from next_best_view.srv import IsPositionReachable,GetDistance
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def calcDist(x,y):
    """
    returns the distance between two points using the appropriate service call
    """
    return  math.sqrt(math.pow(x.x-y.x,2)+math.pow(x.y-y.y,2))


def two_opt_with_cycle(orderedPoseListe):
    """
    returns an optimised list from an orderedpose using the 2-opt algo with creating a Hamilton cycle
    """
    improve = True #we iterate in the list as long as we find improvements
    while improve == True:
        improve = False
        for i in range(len(orderedPoseListe)):
            for j in range(i+2,len(orderedPoseListe)):
                if i==len(orderedPoseListe)-1:
                    if calcDist(orderedPoseListe[i],orderedPoseListe[0])+calcDist(orderedPoseListe[j],orderedPoseListe[j+1]) > calcDist(orderedPoseListe[i],orderedPoseListe[j])+calcDist(orderedPoseListe[0],orderedPoseListe[j+1]):
                        orderedPoseListe[0], orderedPoseListe[j] = orderedPoseListe[j] , orderedPoseListe[0]
                        improve = True
                elif j==len(orderedPoseListe)-1:
                    if calcDist(orderedPoseListe[i],orderedPoseListe[i+1])+calcDist(orderedPoseListe[j],orderedPoseListe[0]) > calcDist(orderedPoseListe[i],orderedPoseListe[j])+calcDist(orderedPoseListe[i+1],orderedPoseListe[0]):
                        orderedPoseListe[i+1], orderedPoseListe[j] = orderedPoseListe[j] , orderedPoseListe[i+1]
                        improve = True
                else:
                    if calcDist(orderedPoseListe[i],orderedPoseListe[i+1])+calcDist(orderedPoseListe[j],orderedPoseListe[j+1]) > calcDist(orderedPoseListe[i],orderedPoseListe[j])+calcDist(orderedPoseListe[i+1],orderedPoseListe[j+1]):
                        orderedPoseListe[i+1], orderedPoseListe[j] = orderedPoseListe[j] , orderedPoseListe[i+1]
                        improve = True
    return orderedPoseListe

def nearest_neighbour(PoseListe):
    """
    returns an ordered list from pose using the nearest neighbout algo
    """
    orderedPoseListe = []
    distance = 100000
    startX = rospy.get_param("startX")
    startY = rospy.get_param("startY")
    for pose in PoseListe:
        if(math.sqrt(math.pow((startX-pose.x),2)+math.pow((startY-pose.y),2)) < distance):
            distance = math.sqrt(math.pow((startX-pose.x),2)+math.pow((startY-pose.y),2))
            if len(orderedPoseListe)>0:
                del orderedPoseListe[0]
            orderedPoseListe.append(pose)
    for idx,pose in enumerate(PoseListe):
        if pose == orderedPoseListe[0]:
            del PoseListe[idx]
            break
    while len(PoseListe)>0:
        distance = float(10000.0)
        ref = -1
        for index in range(len(PoseListe)):
            dist = calcDist(orderedPoseListe[-1],PoseListe[index])
            if dist<distance:
                ref = index
                distance = dist
        orderedPoseListe.append(PoseListe[ref])
        del PoseListe[ref]
    return orderedPoseListe


if __name__ == "__main__":
    rospy.init_node('grid_creator', anonymous=True)
    PoseListe = []
    for size in range(20,2000,20):
        for elt in range(0,size):
            PoseListe.append(Point(*[random.uniform(0, 100),random.uniform(0, 100),0]))
        print(str(PoseListe.size)+",")
        start = time()
        orderedPoseListe = nearest_neighbour(PoseListe)
        print(str(time()-start)+",")
        #2-opt algorithm
        start = time()
        orderedPoseListe = two_opt_with_cycle(orderedPoseListe)
        print(str(time()-start)+";")























