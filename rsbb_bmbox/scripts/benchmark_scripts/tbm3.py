#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rsbb_bmbox.BenchmarkObjects import BaseBenchmarkObject, GoalObject, ManualOperationObject


class BenchmarkObject (BaseBenchmarkObject):
	
	benchmark_code = "HCFGAC"
	
	def execute(self):
		
		##########################################
		#                 GOAL i                 #
		##########################################
		
		goal = GoalObject({}, 15.0)
		
		print "request_goal"
		self.request_goal(goal)
		
		print "wait_goal_result"
		self.wait_goal_result()
		
		##########################################
		#    CHECK RESULT i AND UPDATE SCORE     #
		##########################################
		
		if goal.has_timed_out():
			print "GOAL TIMEOUT"
		elif goal.has_been_completed():
			print "GOAL COMPLETED:"
			result = goal.get_result()
			print "result:\n", result
		else:
			print "GOAL NOT COMPLETED"
		
		self.save_and_publish_score()
		
		if not self.is_benchmark_running():
			print self.get_end_description()
			
			if self.has_benchmark_timed_out():
				print "BENCHMARK TIMEOUT"
			elif self.has_benchmark_been_stopped():
				print "BENCHMARK STOPPED"
			else:
				print "BENCHMARK ABORTED"
			
