#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, yaml

from rsbb_bmbox.BenchmarkObjects import BaseBenchmarkObject, GoalObject, ManualOperationObject


class BenchmarkObject (BaseBenchmarkObject):
	
#T	def get_benchmark_code(self, hello): return "STB"
#T	def get_benchmark_code(self): return 5
	
#T	def get_benchmark_code(self): return "STB"
#T	def execute_(self):
#T	def execute(self, hello):
	
	benchmark_code = "STB"
	
	def execute(self):
		
		N = 2
		i = 1
		execution_time = rospy.Duration(0.0)
		
		
		print "params:\n", self.params
				
		print "referee_score:\n", self.referee_score
		
		##########################################
		#            MANUAL OPERATION            #
		##########################################
		
		manual_operation_first = ManualOperationObject("First Manual Operation")
		
		self.start_manual_operation(manual_operation_first)
		self.wait_manual_operation()
		
		##########################################
		#     CHECK RESULT AND UPDATE SCORE      #
		##########################################
		
		if manual_operation_first.has_been_completed():
			print "First Manual Operation result: %s" % manual_operation_first.get_result()
			self.score["first_manual_operation"] = manual_operation_first.get_result()
			self.save_and_publish_score()
		else:
			print "First Manual Operation NOT EXECUTED"
			self.score["first_manual_operation"] = "not executed"
			self.save_and_publish_score()
		
		if not self.is_benchmark_running():
			if self.has_benchmark_timed_out():
				print "BENCHMARK TIMEOUT"
				return
			elif self.has_benchmark_been_stopped():
				print "BENCHMARK STOPPED"
				return
			else:
				print "BENCHMARK ABORTED"
				return
		
		
		while self.is_benchmark_running() and i <= N:
			
			##########################################
			#                 GOAL i                 #
			##########################################
			
			goal = GoalObject({"goal": "GOAL %d"%i, "details": 4}, 15.0)
			
			self.request_goal(goal)
			start_time = rospy.Time.now()
			
			print "wait_goal_result"
			
			self.wait_goal_result()
			end_time = rospy.Time.now()
			
#T			while not goal.has_been_completed() and not goal.has_timed_out(): rate.sleep(); self.complete_goal()
			
			execution_time += end_time - start_time
			rospy.loginfo("Execution time - %f" % execution_time.to_sec())
			
			
			##########################################
			#    CHECK RESULT i AND UPDATE SCORE     #
			##########################################
			
			self.score["goal_%i"%i] = {}
			self.score["goal_%i"%i]["timeout"] = goal.has_timed_out()
			self.score["goal_%i"%i]["completed"] = goal.has_been_completed()
			
			if goal.has_timed_out():
				print "GOAL TIMEOUT"
			elif goal.has_been_completed():
				print "GOAL COMPLETED:"
				result = goal.get_result()
				print "result:\n", result
				self.score["goal_%i"%i]["result"] = result
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
			
			i += 1
		
		##########################################
		#            MANUAL OPERATION            #
		##########################################
		
		manual_operation_last = ManualOperationObject("Last Manual Operation")
		
		self.request_manual_operation(manual_operation_last)
		
		
		##########################################
		#     CHECK RESULT AND UPDATE SCORE      #
		##########################################
		
		if manual_operation_last.has_been_completed():
			print "Last Manual Operation result: %s" % manual_operation_last.get_result()
			self.score["last_manual_operation"] = manual_operation_last.get_result()
			self.save_and_publish_score()
		else:
			print "Last Manual Operation NOT EXECUTED"
			self.score["last_manual_operation"] = "not executed"
			self.save_and_publish_score()
		
		if not self.is_benchmark_running():
			if self.has_benchmark_timed_out():
				print "BENCHMARK TIMEOUT"
			elif self.has_benchmark_been_stopped():
				print "BENCHMARK STOPPED"
			else:
				print "BENCHMARK ABORTED"
		
		
		##########################################
		#            UPDATE SCORE                #
		##########################################
		
		self.score["execution_time"] = execution_time.to_sec()
		self.save_and_publish_score()

