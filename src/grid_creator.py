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
import subprocess, os, signal
import xml.etree.cElementTree as ET
import math
import rospkg
import datetime


from asr_robot_model_services.srv import GetDistance, IsPositionAllowed
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class GridCreator():

    def __init__(self):
        rospy.init_node('grid_creator', anonymous=True)
        self.Xmax = rospy.get_param("width")
        self.Ymax = rospy.get_param("height")
        self.GridType = rospy.get_param("GridType")
        ExitPath = rospy.get_param("ExitXML")

        StepSizeMin = rospy.get_param("StepSizeMin")
        StepSizeMax = rospy.get_param("StepSizeMax")
        StepSizeStep = rospy.get_param("StepSizeStep")

        HtransMin = rospy.get_param("HtransMin")
        HtransMax = rospy.get_param("HtransMax")
        HtransStep = rospy.get_param("HtransStep")

        VtransMin = rospy.get_param("VtransMin")
        VtransMax = rospy.get_param("VtransMax")
        VtransStep = rospy.get_param("VtransStep")

        OffsetMin = rospy.get_param("OffsetMin")
        OffsetMax = rospy.get_param("OffsetMax")
        OffsetStep = rospy.get_param("OffsetStep")


        bestPoseListe = []
        bestMarkerArray = MarkerArray()
        bestK = 0

        bestStepSize = 0.0
        bestHtrans = 0.0
        bestVtrans = 0.0
        bestOffset = 0.0

        rospy.loginfo("begin of grid creations")
        for currentStepSize in self.frange(StepSizeMin, StepSizeMax, StepSizeStep):
            self.StepSize = currentStepSize
            for currentHtrans in self.frange(HtransMin, HtransMax, HtransStep):
                self.Htrans = currentHtrans
                for currentVtrans in self.frange(VtransMin, VtransMax, VtransStep):
                    self.Vtrans = currentVtrans
                    for currentOffset in self.frange(OffsetMin, OffsetMax, OffsetStep):
                        #beginn = rospy.get_time()
                        self.Offset = currentOffset

                        self.PoseListe = []
                        self.markerArray = MarkerArray()
                        self.k = 0 #marker id
                        self.createGrid()
                        if len(self.PoseListe) >= len(bestPoseListe):
                            bestPoseListe = self.PoseListe
                            bestMarkerArray = self.markerArray
                            bestK = self.k
                            bestStepSize = self.StepSize
                            bestHtrans = self.Htrans
                            bestVtrans = self.Vtrans
                            bestOffset = self.Offset
                        #rospy.loginfo('Iteration took ' + str(rospy.get_time() - beginn)+ 's to finish')

        rospy.loginfo("end of grid creations with number of best poses: " + str(len(bestPoseListe)))

        rospy.loginfo("bestStepSize: " + str(bestStepSize))
        rospy.loginfo("bestHtrans: " + str(bestHtrans))
        rospy.loginfo("bestVtrans: " + str(bestVtrans))
        rospy.loginfo("bestOffset: " + str(bestOffset))

        #rospy.loginfo("Grid created")

        #rospy.loginfo("Begin nearest Neighbour")
        #Nearest Neighbour
        orderedPoseListe = self.nearest_neighbour(bestPoseListe)
        #rospy.loginfo("End nearest Neighbour")

        #rospy.loginfo("Begin 2-Opt")
        #2-opt algorithm
        if rospy.get_param("HamiltonCycle") == True:
           orderedPoseListe = self.two_opt_with_cycle(orderedPoseListe)
        else:
           orderedPoseListe = self.two_opt_without_cycle(orderedPoseListe)
        #rospy.loginfo("End 2-Opt")

        #Save the ordered pose list in the defined xml file
        root = ET.Element("Posen")
        for elem in orderedPoseListe:
           ET.SubElement(root, "Pose", X=str(elem.x), Y=str(elem.y), Z="0", QX="0", QY="0", QZ="0", QW="0")
        tree = ET.ElementTree(root)
        tree.write(ExitPath)
        rospy.loginfo("config.xml created")

        text = 1
        cpoints = []
        for child in orderedPoseListe:
            cpoints.append(Point(*[float(child.x),float(child.y),0]))

        b=0
        for point in cpoints:
            marker = self.getMarker(point,bestK,'GridPoints',0,0.7,1)
            bestMarkerArray.markers.append(marker)
            bestK += 1
            b += 0.01
            marker2 = self.getMarker(point,bestK,'NumPoints',1,0,0)
            marker2.type = 9
            marker2.scale.z = 0.5
            marker2.text = str(text)
            bestMarkerArray.markers.append(marker2)
            bestK += 1
            text += 1


        p=0
        for j in range(len(cpoints)-1):
            points = []
            points.append(Point(*[float(cpoints[j].x),float(cpoints[j].y),0]))
            points.append(Point(*[float(cpoints[j+1].x),float(cpoints[j+1].y),0]))
            marker = self.getMarkerArrow(points,bestK,"Arrows",p,1-p,0)
            bestMarkerArray.markers.append(marker)
            bestK += 1
            p += 0.02

        pub = rospy.Publisher('direct_search/grid_visualization', MarkerArray, queue_size=100)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            pub.publish(bestMarkerArray)
            rate.sleep()


    def createGrid(self):
        #rospy.loginfo("Begin from grid creation")
        #Quad grid creation
        if self.GridType == "Quad":
            i = 0
            while i<self.Xmax:
                j=0
                test = 0 # 0 is not reachable, 1 is reachable
                while j< self.Ymax:
                    TempPose = self.transformPoint(Point(*[i,j,0]),self.Offset,self.Htrans,self.Vtrans)
                    if self.isRobotPositionAllowed(TempPose):
                        self.PoseListe.append(TempPose)
                        test = 1
                    else:
                        marker = self.getMarker(TempPose,self.k,'UnreachablePoints',1,0.5,0)
                        self.markerArray.markers.append(marker)
                        self.k += 1
                        test = 0

                    if test == 1:
                        marker = self.drawQuad(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,0,0,1,self.k,"QuadDrawReach")
                        self.k+=1
                        markerColor = self.colorQuad(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,0,0,1,self.k,"ColorQuadReach")
                        self.markerArray.markers.append(markerColor)
                    else:
                        marker = self.drawQuad(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,1,0,0,self.k,"QuadDrawUnreach")
                        marker.color.a = 0.2
                        self.k+=1
                        markerColor = self.colorQuad(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,1,0,0,self.k,"ColorQuadUnreach")
                        self.markerArray.markers.append(markerColor)

                    self.markerArray.markers.append(marker)
                    self.k+=1
                    j+=self.StepSize
                i+=self.StepSize


        #Hexagonal grid creation
        #The hexagonal grid is created in 2 steps; Each step create one out of two line of hexagon, because of the offset
        elif self.GridType == "Hex":
            i=0
            while i < self.Xmax:
                j=0
                test = 0 # 0 is not reachable, 1 is reachable
                while j < self.Ymax:
                    TempPose = self.transformPoint(Point(*[i,j,0]),self.Offset,self.Htrans,self.Vtrans)
                    if self.isRobotPositionAllowed(TempPose):
                        self.PoseListe.append(TempPose)
                        test = 1
                    else:
                        marker = self.getMarker(TempPose,self.k,'UnreachablePoints',1,0.5,0)
                        self.markerArray.markers.append(marker)
                        self.k += 1
                        test = 0

                    if test == 1 :
                        marker = self.drawHexagon(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,0,0,1,self.k,"HexDrawReach")
                        self.k+=1
                        markerColor = self.colorHexagon(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,0,0,1,self.k,"ColorHexReach")
                        self.markerArray.markers.append(markerColor)
                    else:
                        marker = self.drawHexagon(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,1,0,0,self.k,"HexDrawUnreach")
                        marker.color.a = 0.2
                        self.k+=1
                        markerColor = self.colorHexagon(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,1,0,0,self.k,"ColorHexUnreach")
                        self.markerArray.markers.append(markerColor)
                    self.markerArray.markers.append(marker)
                    self.k += 1

                    j += 3*self.StepSize
                i+= math.sqrt(3)*self.StepSize

            i = math.sqrt(3)*self.StepSize/2
            while i < self.Xmax:
                test = 0 # 0 is not reachable, 1 is reachable
                j = 1.5*self.StepSize
                while j < self.Ymax:
                    TempPose = self.transformPoint(Point(*[i,j,0]),self.Offset,self.Htrans,self.Vtrans)
                    if self.isRobotPositionAllowed(TempPose):
                        self.PoseListe.append(TempPose)
                        test = 1
                    else:
                        marker = self.getMarker(TempPose,self.k,'UnreachablePoints',1,0.5,0)
                        self.markerArray.markers.append(marker)
                        self.k += 1
                        test = 0

                    if test == 1 :
                        marker = self.drawHexagon(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,0,0,1,self.k,"HexDrawReach")
                        self.k+=1
                        markerColor = self.colorHexagon(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,0,0,1,self.k,"ColorHexReach")
                        self.markerArray.markers.append(markerColor)
                    else:
                        marker = self.drawHexagon(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,1,0,0,self.k,"HexDrawUnreach")
                        marker.color.a = 0.2
                        self.k+=1
                        markerColor = self.colorHexagon(i,j,self.Offset,self.Htrans,self.Vtrans,self.StepSize,1,0,0,self.k,"ColorHexUnreach")
                        self.markerArray.markers.append(markerColor)
                    self.markerArray.markers.append(marker)
                    self.k += 1

                    j += 3*self.StepSize
                i+= math.sqrt(3)*self.StepSize


    def frange(self, start, stop, step):
         i = start
         while i <= stop:
             yield i
             i += step


    def isRobotPositionAllowed(self, robotPosition):
        """
        returns if robotPosition is allowed using the appropriate service call
        """
        rospy.wait_for_service('/asr_robot_model_services/IsPositionAllowed')
        try:
            isReachableHandler = rospy.ServiceProxy('/asr_robot_model_services/IsPositionAllowed', IsPositionAllowed)
            ret = isReachableHandler(robotPosition)
            return ret.isAllowed
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

    def calcDist(self, x,y):
        """
        returns the distance between two points using the appropriate service call
        """
        rospy.wait_for_service('/asr_robot_model_services/GetDistance')
        try:
            GetDistanceHandler = rospy.ServiceProxy('/asr_robot_model_services/GetDistance', GetDistance)
            ret = GetDistanceHandler(x, y)
            return ret.distance
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def getMarker(self, pose,k,n_s,r,g,b):
        """
        returns a sphere marker
        """
        color = ColorRGBA(*[r,g,b,1])
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = n_s
        marker.id = k
        marker.type = 2
        marker.action = 0
        marker.pose.position = pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = color
        return marker

    def getMarkerLine(self, points,k,n_s,r,g,b):
        """
        returns a line marker
        """
        marker = Marker()
        color = ColorRGBA(*[r,g,b,1])
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = n_s
        marker.id = k
        marker.type = 4
        marker.action = 0
        marker.points = points
        marker.pose.position = Point(*[0,0,0])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color = color
        return marker

    def getMarkerArrow(self, points,k,n_s,r,g,b):
        """
        returns an arrow marker
        """
        marker = Marker()
        color = ColorRGBA(*[r,g,b,1])
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = n_s
        marker.id = k
        marker.type = 0
        marker.action = 0
        marker.points=points
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0
        marker.color = color
        return marker

    def transformPoint(self, point,Offset,Htrans,Vtrans):
        """
        returns a transformed point using Offset rotation and (Htrans,Vtrans) translation
        """
        ret = Point()
        ret.x = float(point.x)*math.cos(Offset) - math.sin(Offset)*float(point.y) + Htrans
        ret.y = float(point.x)*math.sin(Offset) + math.cos(Offset)*float(point.y) + Vtrans
        ret.z = 0
        return ret

    def drawHexagon(self, i,j,Offset,Htrans,Vtrans,StepSize,r,g,b,k,ns):
        """
        returns a list of line marker to draw the Hexagonal grid
        """
        points = []
        xplus = float(i)+StepSize*math.sqrt(3.0)/2
        xminus = float(i)-StepSize*math.sqrt(3.0)/2
        yplus = float(j)+float((StepSize/2.0))
        yminus = float(j)-float((StepSize/2.0))
        ypp = float(j)+float(StepSize)
        ymm = float(j)-float(StepSize)
        points.append(self.transformPoint(Point(*[i,ypp,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xplus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xplus,yminus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,ymm,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xminus,yminus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xminus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,ypp,0]),Offset,Htrans,Vtrans))
        return self.getMarkerLine(points,k,ns,r,g,b)

    def colorHexagon(self, i,j,Offset,Htrans,Vtrans,StepSize,r,g,b,k,ns):
        """
        returns a list of triangle marker to color the Hexagonal grid
        """
        points = []
        xplus = float(i)+StepSize*math.sqrt(3.0)/2
        xminus = float(i)-StepSize*math.sqrt(3.0)/2
        yplus = float(j)+float((StepSize/2.0))
        yminus = float(j)-float((StepSize/2.0))
        ypp = float(j)+float(StepSize)
        ymm = float(j)-float(StepSize)

        #We use the points list to store all the triangle used for coloring the grid cells.
        #Color depends on reachability. An hexagon is divided into 6 triangles
        points.append(self.transformPoint(Point(*[i,ypp,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xplus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,j,0]),Offset,Htrans,Vtrans))

        points.append(self.transformPoint(Point(*[xplus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,j,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xplus,yminus,0]),Offset,Htrans,Vtrans))

        points.append(self.transformPoint(Point(*[i,j,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xplus,yminus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,ymm,0]),Offset,Htrans,Vtrans))

        points.append(self.transformPoint(Point(*[i,j,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xminus,yminus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,ymm,0]),Offset,Htrans,Vtrans))

        points.append(self.transformPoint(Point(*[i,j,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xminus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xminus,yminus,0]),Offset,Htrans,Vtrans))

        points.append(self.transformPoint(Point(*[i,j,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,ypp,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xminus,yplus,0]),Offset,Htrans,Vtrans))

        marker = Marker()
        color = ColorRGBA(*[r,g,b,1])
        colorM = ColorRGBA(*[r,g,b,0.5])
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = ns
        marker.id = k
        marker.type = 11
        marker.action = 0
        marker.points=points
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color = color
        for point in points:
            marker.colors.append(colorM)
        return marker

    def drawQuad(self, i,j,Offset,Htrans,Vtrans,StepSize,r,g,b,k,ns):
        """
        returns a list of line marker to draw the quad grid
        """
        points = []
        xplus = float(i)+float((StepSize/2.0))
        xminus = float(i)-float((StepSize/2.0))
        yplus = float(j)+float((StepSize/2.0))
        yminus = float(j)-float((StepSize/2.0))
        points.append(self.transformPoint(Point(*[xminus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xplus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xplus,yminus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xminus,yminus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xminus,yplus,0]),Offset,Htrans,Vtrans))
        return self.getMarkerLine(points,k,ns,r,g,b)

    def colorQuad(self, i,j,Offset,Htrans,Vtrans,StepSize,r,g,b,k,ns):
        """
        returns a list of triangle marker to color the quad grid
        """
        points = []
        xplus = float(i)+float((StepSize/2.0))
        xminus = float(i)-float((StepSize/2.0))
        yplus = float(j)+float((StepSize/2.0))
        yminus = float(j)-float((StepSize/2.0))

        #We use the points list to store all the triangle used for coloring the grid cells.
        #Color depends on reachability. A quad is divided into 4 triangles
        points.append(self.transformPoint(Point(*[xminus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xplus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,j,0]),Offset,Htrans,Vtrans))

        points.append(self.transformPoint(Point(*[xplus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xplus,yminus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,j,0]),Offset,Htrans,Vtrans))

        points.append(self.transformPoint(Point(*[xplus,yminus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xminus,yminus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,j,0]),Offset,Htrans,Vtrans))

        points.append(self.transformPoint(Point(*[xminus,yminus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[xminus,yplus,0]),Offset,Htrans,Vtrans))
        points.append(self.transformPoint(Point(*[i,j,0]),Offset,Htrans,Vtrans))

        marker = Marker()
        color = ColorRGBA(*[r,g,b,0.5])
        colorM = ColorRGBA(*[r,g,b,0.5])
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = ns
        marker.id = k
        marker.type = 11
        marker.action = 0
        marker.points=points
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color = color
        for point in points:
            marker.colors.append(colorM)
        return marker

    def two_opt_with_cycle(self, orderedPoseListe):
        """
        returns an optimised list from an orderedpose using the 2-opt algo with creating a Hamilton cycle
        """
        improve = True #we iterate in the list as long as we find improvements
        while improve == True:
            improve = False
            for i in range(len(orderedPoseListe)):
                for j in range(i+2,len(orderedPoseListe)):
                    if i==len(orderedPoseListe)-1:
                        if self.calcDist(orderedPoseListe[i],orderedPoseListe[0])+self.calcDist(orderedPoseListe[j],orderedPoseListe[j+1]) > self.calcDist(orderedPoseListe[i],orderedPoseListe[j])+self.calcDist(orderedPoseListe[0],orderedPoseListe[j+1]):
                            orderedPoseListe[0], orderedPoseListe[j] = orderedPoseListe[j] , orderedPoseListe[0]
                            improve = True
                    elif j==len(orderedPoseListe)-1:
                        if self.calcDist(orderedPoseListe[i],orderedPoseListe[i+1])+self.calcDist(orderedPoseListe[j],orderedPoseListe[0]) > self.calcDist(orderedPoseListe[i],orderedPoseListe[j])+self.calcDist(orderedPoseListe[i+1],orderedPoseListe[0]):
                            orderedPoseListe[i+1], orderedPoseListe[j] = orderedPoseListe[j] , orderedPoseListe[i+1]
                            improve = True
                    else:
                        if self.calcDist(orderedPoseListe[i],orderedPoseListe[i+1])+self.calcDist(orderedPoseListe[j],orderedPoseListe[j+1]) > self.calcDist(orderedPoseListe[i],orderedPoseListe[j])+self.calcDist(orderedPoseListe[i+1],orderedPoseListe[j+1]):
                            orderedPoseListe[i+1], orderedPoseListe[j] = orderedPoseListe[j] , orderedPoseListe[i+1]
                            improve = True
        return orderedPoseListe

    def two_opt_without_cycle(self, orderedPoseListe):
        """
        returns an optimised list from an orderedpose using the 2-opt algo without creating a Hamilton cycle
        """
        improve = True #we iterate in the list as long as we find improvements
        while improve == True:
            improve = False
            for i in range(len(orderedPoseListe)-1):
                for j in range(i+2,len(orderedPoseListe)-1):
                    if self.calcDist(orderedPoseListe[i],orderedPoseListe[i+1])+self.calcDist(orderedPoseListe[j],orderedPoseListe[j+1]) > self.calcDist(orderedPoseListe[i],orderedPoseListe[j])+self.calcDist(orderedPoseListe[i+1],orderedPoseListe[j+1]):
                        orderedPoseListe[i+1], orderedPoseListe[j] = orderedPoseListe[j] , orderedPoseListe[i+1]
                        improve = True
        return orderedPoseListe

    def nearest_neighbour(self, PoseListe):
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
                dist = self.calcDist(orderedPoseListe[-1],PoseListe[index])
                if dist<distance:
                    ref = index
                    distance = dist
            orderedPoseListe.append(PoseListe[ref])
            del PoseListe[ref]
        return orderedPoseListe


if __name__ == "__main__":
    gridCreator = GridCreator()
























