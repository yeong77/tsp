#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May  9 00:16:30 2021

@author: wellpast
"""
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ant_tsp_12MAY import *
from time import sleep
from std_msgs.msg import String

class ROS_tsp:
	def callback(self, m):
		rospy.loginfo("subscribe to /multi_classification/true")
		rospy.signal_shutdown("END tsp")

	def __init__(self):
		self.multi = rospy.Subscriber("/multi_classification/true", String, self.callback)
		self.tp_number = rospy.get_param("/ROS_Ant_tsp/tp_number")
		self.cities = rospy.get_param("/ROS_Ant_tsp/cities")
		self._colony_size = 10
		self._steps = 50
		self.cities = []
		self._nodes= []
		self.solution_points = []
		self.acs = []
		self.elitist = []
		self.x = []
		self.y = []


	def tsp_problem(self):
		self.tp_number = rospy.get_param("/ROS_Ant_tsp/tp_number") 
		print("\n*\n*\n***** the number of points to go : ", self.tp_number, "*****", "\n\n")
    
		self.cities = rospy.get_param("/ROS_Ant_tsp/cities")        
		print("Destination points to go are as follows:", self.cities)
		print("\n\n\n**************TRAVELLING SALESMAN PROBLEM**************")
		self._nodes= []
		self._nodes = self.cities

		self.acs = SolveTSPUsingACO(mode='ACS', colony_size=self._colony_size, steps=self._steps, nodes=self._nodes)
		self.acs.run()
		self.acs.plot()
		self.elitist = SolveTSPUsingACO(mode='Elitist', colony_size=self._colony_size, steps=self._steps, nodes=self._nodes)
		self.elitist.run()
		self.elitist.plot()
		self.max_min = SolveTSPUsingACO(mode='MaxMin', colony_size=self._colony_size, steps=self._steps, nodes=self._nodes)
		self.max_min.run()
		self.max_min.plot()  


	def movebase_client(self):
		client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		client.wait_for_server()
		target = MoveBaseGoal()
		target.target_pose.header.frame_id = "map"
		target.target_pose.pose.position.x = self.x 
		target.target_pose.pose.position.y = self.y
		target.target_pose.pose.orientation.w = 1.0
		client.send_goal(target)
		wait = client.wait_for_result()
		if not wait:
			rospy.signal_shutdown("No Action Service!")
		else:
			return client.get_result()
        

	def dumdum(self):
		self.solution_points = []
		self.solution_points = self.max_min.array 
		print("\n*\n*\nAnt Colony - Solution set of TSP points respectively : ", self.solution_points)

		for i in range(self.tp_number+1):
			print("{}. destination coordinates x={} y={}".format(i, self.solution_points[i][0], self.solution_points[i][1]))

		for k in range(self.tp_number+1):
			if k == 0:
				print("Robot is moving...")
			else:
				print("\nThe new route is being set.\n")
				sleep(2)
				self.x = self.solution_points[k][0] 
				self.y = self.solution_points[k][1]
				print("Location points of the target: x=", self.x,"y=", self.y)

	
				if __name__ == '__main__':
					try:
						result = self.movebase_client()
						if result:
							rospy.loginfo("Reached at the target point.!")
							x_guncel = self.x
							y_guncel = self.y
						else:
							rospy.loginfo("Going to target point......")
                		         
					except rospy.ROSInterruptException:
						pass


	def main(self):
		while not rospy.is_shutdown():
			self.tsp_problem()
			self.dumdum()				

if __name__ == '__main__':
	rospy.init_node("ROS_tsp", anonymous = False)
	ROS_tsp().main()
	#rospy.spin()


