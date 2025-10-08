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

import sys, pygame, math, numpy, random, time, copy
from pygame.locals import * 

from constants import *
from utils import *
from core import *



###############################
### AStarNavigator2
###
### Creates a path node network and implements the A* algorithm to create a path to the given destination.
			
class AStarNavigator2(PathNetworkNavigator):

				
	### Finds the shortest path from the source to the destination using A*.
	### self: the navigator object
	### source: the place the agent is starting from (i.e., its current location)
	### dest: the place the agent is told to go to
	def computePath(self, source, dest):
		self.setPath(None)
		### Make sure the next and dist matrices exist
		if self.agent != None and self.world != None: 
			self.source = source
			self.destination = dest
			### Step 1: If the agent has a clear path from the source to dest, then go straight there.
			### Determine if there are no obstacles between source and destination (hint: cast rays against world.getLines(), check for clearance).
			### Tell the agent to move to dest
			if clearShot(source, dest, self.world.getLinesWithoutBorders(), self.world.getPoints(), self.agent):
				self.agent.moveToTarget(dest)
			else:
				### Step 2: If there is an obstacle, create the path that will move around the obstacles.
				### Find the path nodes closest to source and destination.
				start = getOnPathNetwork(source, self.pathnodes, self.world.getLinesWithoutBorders(), self.agent)
				end = getOnPathNetwork(dest, self.pathnodes, self.world.getLinesWithoutBorders(), self.agent)
				if start != None and end != None:
					### Remove edges from the path network that intersect gates
					newnetwork = unobstructedNetwork(self.pathnetwork, self.world.getGates(), self.world)
					closedlist = []
					### Create the path by traversing the pathnode network until the path node closest to the destination is reached
					path, closedlist = astar(start, end, newnetwork)
					if path is not None and len(path) > 0:
						### Determine whether shortcuts are available
						path = shortcutPath(source, dest, path, self.world, self.agent)
						### Store the path by calling self.setPath()
						self.setPath(path)
						if self.path is not None and len(self.path) > 0:
							### Tell the agent to move to the first node in the path (and pop the first node off the path)
							first = self.path.pop(0)
							self.agent.moveToTarget(first)
		return None
		
	### Called when the agent gets to a node in the path.
	### self: the navigator object
	def checkpoint(self):
		myCheckpoint(self)
		return None

	### This function gets called by the agent to figure out if some shortcuts can be taken when traversing the path.
	### This function should update the path and return True if the path was updated.
	def smooth(self):
		return mySmooth(self)

	def update(self, delta):
		myUpdate(self, delta)


### Removes any edge in the path network that intersects a worldLine (which should include gates).
def unobstructedNetwork(network, worldLines, world):
	newnetwork = []
	for l in network:
		hit = rayTraceWorld(l[0], l[1], worldLines)
		if hit == None:
			newnetwork.append(l)
	return newnetwork

#checking if two lines from x1 to x2 and y1 to y2 
def linesIntersect(x1, x2, y1, y2):
	#calcuating distance
	dx1 = x2[0] - x1[0] 
	dy1 = x2[1] - x1[1]
	dx2 = y2[0] - y1[0]
	dy2 = y2[1] - y1[1]

	#getting determinant
	det = dx1 * dy2 - dy1 * dx2
	if det == 0:
		return False 		#lines are parallel
	dx3 = x1[0] - y1[0]
	dy3 = x1[1] - y1[1]
	t1 = (dx2 * dy3 - dy2 * dx3) / det
	t2 = (dx1 * dy3 - dy1 * dx3) / det
 
	#now check if the intersection point is within both line segments
	if t1 >= 0 and t1 <= 1 and t2 >= 0 and t2 <= 1:
		return True
	return False


### Returns true if the agent can get from p1 to p2 directly without running into an obstacle.
### p1: the current location of the agent
### p2: the destination of the agent
### worldLines: all the lines in the world
### agent: the Agent object
def clearShot(p1, p2, worldLines, worldPoints, agent):
    ### YOUR CODE GOES BELOW HERE ###

	#check the lines in the world to see if there is anythign that is blocking the path
	for line in worldLines:
     	start = line[0]
		end = line[1]

		#no intersection is found which means path is good ot go
		if not linesIntersect(p1, p2, start, end):
			return True
  
    ### YOUR CODE GOES ABOVE HERE ###
	return False

### Given a location, find the closest pathnode that the agent can get to without collision
### agent: the agent
### location: the location to check from (typically where the agent is starting from or where the agent wants to go to) as an (x, y) point
### pathnodes: a list of pathnodes, where each pathnode is an (x, y) point
### world: pointer to the world
def getOnPathNetwork(location, pathnodes, worldLines, agent):
	node = None
	### YOUR CODE GOES BELOW HERE ###

	### YOUR CODE GOES ABOVE HERE ###
	return node



### Implement the a-star algorithm
### Given:
### Init: a pathnode (x, y) that is part of the pathnode network
### goal: a pathnode (x, y) that is part of the pathnode network
### network: the pathnode network
### Return two values: 
### 1. the path, which is a list of states that are connected in the path network
### 2. the closed list, the list of pathnodes visited during the search process
def astar(init, goal, network):
	path = []
	open = []
	closed = []
	### YOUR CODE GOES BELOW HERE ###
 
	#Helper function to get fCost
	def getFCost(node, fList):
		for n, cost in fList:
			if n == node:
				return cost

	#Helper function to get parent (similar to other get helper functions)
	def getParent(node, pList):
		for n, parent in pList:
			if n == node:
				return parent

	def getGCost(node, gList):
		for n, cost in gList:
			if n == node:
				return cost

	#udpate the parent for a node in the parent list with a parent node called 'parentNode'
	def updateParent(node, parentNode, pList):
		for i in range(len(pList)):
			if pList[i][0] == node:
				pList[i] = (node, parentNode)
				return
		pList.append((node, parentNode))
  
	#update the gCost for a node inside the gList with a new cost named 'cost'
	def updateGCost(node, cost, gList):
		for i in range(len(gList)):
			if gList[i][0] == node:
				gList[i] = (node, cost)
				return
		gList.append((node, cost))
  
	#update the fCost for a node inside the fList with a new cost named 'cost'
	def updateFCost(node, cost, fList):
		for i in range(len(fList)):
			if fList[i][0] == node:
				fList[i] = (node, cost)
				return
		fList.append((node, cost))
	
	#initalize variables to track the costs 
	goalCost = []
	fCost = []
	parent = []
	
	open.append(init)
	while len(open) > 0:			#start with the lowest distance cost in list
		current = open[0]
		lowestF = getFCost(current, fCost)
		for node in open:
			fValue = getFCost(node, fCost)
			if fValue < lowestF:
				current = node		#find the lowest F Cost using linear search 
				lowestF = fValue
		if current == goal:
			#follow parent
			pathNode = current
			while pathNode != None:
				path.append(pathNode)
				pathNode = getParent(pathNode, parent)
			path.reverse()
			break

		#after reconstructing path, move the current from open to closed
		open.remove(current)
		closed.append(current)
  
		#Get your neighbors
		neighbors = []
		for edge in network:
			if edge[0] == current:
				neighbors.append(edge[1])	#append neighbor
			elif edge[1] == current:
				neighbors.append(edge[0])
		#Now check teh neighbors if its closed or not
		for neighbor in neighbors:
			if neighbor in closed:
				continue
			
			g = getGCost(current, goalCost)
			tentativeG = g + distance(current, neighbor)		#find a tentative g cost to its neighbor
			neighborG = getGCost(neighbor, goalCost)
			openNeighbor = neighbor in open

			if not openNeighbor or tentativeG < neighborG:
				#now we have to update the parent 
				updateParent(neighbor, current, parent)
				updateGCost(neighbor, tentativeG, goalCost)
				updateFCost(neighbor, tentativeG + distance(neighbor, goal), fCost)
    
				if not openNeighbor:
					open.append(neighbor)
		
	
 
	### YOUR CODE GOES ABOVE HERE ###
	return path, closed




def myUpdate(nav, delta):
	### YOUR CODE GOES BELOW HERE ###
	
	### YOUR CODE GOES ABOVE HERE ###
	return None




def myCheckpoint(nav):
	### YOUR CODE GOES BELOW HERE ###
	
	### YOUR CODE GOES ABOVE HERE ###
	return None







### This function optimizes the given path and returns a new path
### source: the current position of the agent
### dest: the desired destination of the agent
### path: the path previously computed by the A* algorithm
### world: pointer to the world
def shortcutPath(source, dest, path, world, agent):
	path = copy.deepcopy(path)
	### YOUR CODE GOES BELOW HERE ###
	
	### YOUR CODE GOES BELOW HERE ###
	return path


### This function changes the move target of the agent if there is an opportunity to walk a shorter path.
### This function should call nav.agent.moveToTarget() if an opportunity exists and may also need to modify nav.path.
### nav: the navigator object
### This function returns True if the moveTarget and/or path is modified and False otherwise
def mySmooth(nav):
	### YOUR CODE GOES BELOW HERE ###
	
	### YOUR CODE GOES ABOVE HERE ###
	return False

