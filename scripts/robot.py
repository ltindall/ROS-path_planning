#!/usr/bin/env python

#robot.py implementation goes here

import rospy 
from read_config import read_config 
from cse_190_assi_3.msg import AStarPath, PolicyList
from astar import astar
from mdp import mdp
from std_msgs.msg import Bool 

class robot(): 
    def __init__(self): 
	self.config = read_config()
	rospy.init_node("robot") 
	
	self.path_publisher = rospy.Publisher(
	    "/results/path_list", 
            AStarPath, 
	    queue_size = 50
	)
	self.complete_publisher = rospy.Publisher(
	    "/map_node/sim_complete", 
	    Bool, 
	    queue_size = 10
	)
	self.policy_publisher = rospy.Publisher(
	    "/results/policy_list", 
	    PolicyList, 
	    queue_size = 10
	) 	     
	astarPath = astar(self.config)
	path = astarPath.path

	rospy.sleep(5)	
	for point in path: 
	    print point
	    self.path_publisher.publish(point)
	
	rospy.sleep(3)
	
	map_size = self.config['map_size']
        rows = map_size[0]
        columns = map_size[1]
        goal = self.config['goal']
        walls = self.config['walls']
        pits = self.config['pits']
        move_list = self.config['move_list']
	

	oldV = [[0 for x in range(columns)] for y in range(rows)]

        # initialize V values 
        #oldV = [[0 for x in range(columns)] for y in range(rows)]              
        for i in range(rows):
            for j in range(columns):
                tempGrid = [i,j]
                if tempGrid == goal:
                    oldV[i][j] = self.config['reward_for_reaching_goal']
                elif tempGrid in walls:
                    oldV[i][j] = self.config['reward_for_hitting_wall']
                elif tempGrid in pits:
                    oldV[i][j] = self.config['reward_for_falling_in_pit']
                else:
                    #this is redundant 
                    oldV[i][j] = 0

	stoppingSum = 0
	firstThreshold = False
	secondThreshold = False
	firstX = 0
	secondX = -100
	for x in range(self.config['max_iterations']):
 	    mdpPolicy = mdp(self.config,oldV)
	    policies = mdpPolicy.policies
	    #policies = ["WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "S", "S", "S", "W", "PIT", "S", "PIT", "S", "WALL", "E", "E", "S", "W", "W", "W", "S", "E", "S", "WALL", "WALL", "E", "S", "W", "WALL", "PIT", "S", "PIT", "S", "WALL", "WALL", "E", "S", "W", "WALL", "WALL", "S", "WALL", "S", "WALL", "WALL", "E", "S", "W", "WALL", "PIT", "S", "PIT", "S", "WALL", "WALL", "E", "E", "E", "E", "S", "S", "S", "E", "GOAL", "WALL", "E", "N", "W", "WALL", "E", "E", "E", "N", "WALL", "WALL", "N", "N", "N", "E", "N", "N", "N", "N", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL", "WALL"]
	    self.policy_publisher.publish(policies)
            for i in range(rows):
                for j in range(columns):
	       	    stoppingSum += abs(oldV[i][j]-mdpPolicy.newV[i][j])
	    if(stoppingSum < self.config['threshold_difference']): 
		secondX = x 
	    if(secondX - 1 == firstX ):
		break
	    if(stoppingSum < self.config['threshold_difference'] ):
		firstX = x 
		firstThreshold = True
		
	    oldV = mdpPolicy.newV
	
	rospy.sleep(5)	


	self.complete_publisher.publish(True)
	rospy.sleep(3)
	rospy.signal_shutdown("sim complete")
		    

if __name__ == '__main__':
    robo = robot()		
 
