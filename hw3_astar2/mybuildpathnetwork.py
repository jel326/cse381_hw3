'''
 * Copyright (c) 2014, 2015 Entertainment Intelligence Lab, Georgia Institute of Technology.
 * Originally developed by Mark Riedl.
 * Last edited by Mark Riedl 05/2015
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
'''

import sys, pygame, math, numpy, random, time, copy, operator
from pygame.locals import *

from constants import *
from utils import *
from core import *

# Creates the path network as a list of lines between all path nodes that are traversable by the agent.
def myBuildPathNetwork(pathnodes, world, agent = None):
	lines = []
	### YOUR CODE GOES BELOW HERE ###
	def calcDistance(p1, p2):
		return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

	def colinear(p1, p2, p3):
		#Check if the points are colinear for forming triangles
		area = abs((p2[0]-p1[0]) * (p3[1]-p1[1]) - (p3[0]-p1[0]) * (p2[1]-p1[1]))	#colinear formula for traingle points. used gen-ai to help verify the points and indexes are correct
		return area < 0.001		

	def getCentroid(poly):
		x = sum([p[0] for p in poly]) / len(poly)
		y = sum(p[1] for p in poly) / len(poly)
		return (x,y)
 
	allPoints = list(worldPoints)
	mapWidth, mapHeight = world.getDimensions()
	worldCorners = [(0,0), (mapWidth, 0), (mapWidth, mapHeight), (0, mapHeight)]
 
	for corner in worldCorners:			#add corner points with obstacle points
		if corner not in allPoints:
			allPoints.append(corner)
   
	#Create triangles
	poly = []
 
	for i, point1 in enumerate(allPoints):
		distance = []
		#find points that are closest to point1 for connection
		for j, point2 in enumerate(allPoints):
			if i != j:
				dist = calcDistance(point1, point2)
				distance.append((dist, j, point2))
		distance.sort()		#sort distances from point

		#form triangle with closest points
		for k in range(len(distance)):
			for m in range(k+1, len(distance)):
				point2 = distance[k][2]
				point3 = distance[m][2]
				if colinear(point1, point2, point3):
					continue
				#check to make sure the traingels dont collide with obstacles by checking the centroid
				triEdges = [(point1, point2), (point2, point3), (point3, point1)]
				valid = True
				#get the centroid point to see if its inside any obstacles
				centroid = getCentroid([point1, point2, point3])
				for obstacle in worldObstacles:
					if pointInsidePolygonPoints(centroid, obstacle.getPoints()):
						valid = False
						break
				
				if valid:
					triangle = [point1, point2, point3]
					poly.append(triangle)

	#remove duplicates
	uniquePoly = []
	for p in poly:
		if p not in uniquePoly:
			uniquePoly.append(p)
	poly = uniquePoly
 
	#create nodes at centroids 
	nodes = []
	for p in poly:
		nodes.append(getCentroid(p))
  
	#Create edges between adjacent polygons
	edges = []
	for i, poly1 in enumerate(poly):
		for j, poly2 in enumerate(poly):
			if i < j and polygonsAdjacent(poly1, poly2):
				edges.append((getCentroid(poly1), getCentroid(poly2)))
	### YOUR CODE GOES ABOVE HERE ###
	return lines
