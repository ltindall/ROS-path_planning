#! /usr/bin/env python
"""
Author: Lucas Tindall
"""

# astar implementation needs to go here

class Node(): 
    def __init__(self,x,y,h,obstacle): 
	self.x = x
	self.y = y
	self.visited = False
	self.h = h  
	self.obstacle = obstacle 

class astar(): 
    def __init__(self, config): 
	self.config = config 
	map_size = self.config['map_size'] 
	start = self.config['start'] 
	goal = self.config['goal'] 

	nodes = [[Node(0,0,0,True) for y in range(map_size[1])] for x in range(map_size[0])] 
	for i in range(map_size[0]): 
	    for j in range(map_size[1]):
		coordinate = [i,j] 
		heuristic = abs(i-goal[0]) + abs(j-goal[1])
		if (coordinate in self.config['walls'] 
			or coordinate in self.config['pits']): 
		    #nodes[i][j] = Node(i,j,heuristic,True)
		    nodes[i][j].x = i 
		    nodes[i][j].y = j 
		    nodes[i][j].obstacle = True 
		    nodes[i][j].h = heuristic
		else: 
		    #nodes[i][j] = Node(i,j,heuristic,False)
		    nodes[i][j].x = i 
		    nodes[i][j].y = j 
		    nodes[i][j].obstacle = False
		    nodes[i][j].h = heuristic

	# for all of the neighbors of the visiting node
	# compute the f value (h is precalculated, g is +1 of current node) 
	# and set parent 

	openList = []
	nodes[start[0]][start[1]].g = 0
	openList.append(nodes[start[0]][start[1]])
	
	closedList = [] 
	foundGoal = False
	while(len(openList) > 0 and foundGoal == False ):
	    visiting = openList.pop(0)
	    closedList.append(visiting)
	    validNeighbors = []
	    #check left
	    if(visiting.y - 1 >= 0 and nodes[visiting.x][visiting.y-1].obstacle == False
		and nodes[visiting.x][visiting.y-1] not in closedList): 
		validNeighbors.append(nodes[visiting.x][visiting.y-1]) 

	    #check right
	    if(visiting.y+1 < map_size[1] and nodes[visiting.x][visiting.y+1].obstacle == False
		and nodes[visiting.x][visiting.y+1] not in closedList): 
		validNeighbors.append(nodes[visiting.x][visiting.y+1]) 

	    #check up 
	    if(visiting.x-1 >= 0 and nodes[visiting.x-1][visiting.y].obstacle == False
		and nodes[visiting.x-1][visiting.y] not in closedList): 
		validNeighbors.append(nodes[visiting.x-1][visiting.y])

	    #check down
	    if(visiting.x+1 < map_size[0] and nodes[visiting.x+1][visiting.y].obstacle == False
		and nodes[visiting.x+1][visiting.y] not in closedList): 
		validNeighbors.append(nodes[visiting.x+1][visiting.y])

	    for neighbor in validNeighbors: 
		if(neighbor.x == goal[0] and neighbor.y == goal[1]): 
		    foundGoal = True
		    break
	    for neighbor in validNeighbors: 
		neighbor.g = visiting.g + 1
		neighbor.f = neighbor.g + neighbor.h 
		neighbor.parent = visiting
		openList.append(neighbor)
	    
	    openList = sorted(openList, key=lambda node: node.f)

        reversePath = []
	reversePath.append(nodes[goal[0]][goal[1]])
	notStart = True
	i = 0 
	while(notStart): 
	    reversePath.append(reversePath[i].parent)
	    if(reversePath[i].parent.x == start[0] and reversePath[i].parent.y == start[1]): 
		notStart = False
		break
	    i += 1 
	self.path = [] 
	for node in reversePath: 
	    self.path.insert(0, [node.x,node.y])
	
	
