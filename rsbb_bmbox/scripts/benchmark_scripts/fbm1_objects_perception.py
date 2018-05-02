#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, yaml, tf, random
from math import pi, sin, cos, atan2, fabs, sqrt, pow
from geometry_msgs.msg import Pose2D
from os import path

from rsbb_bmbox.BenchmarkObjects import BaseBenchmarkObject, GoalObject, ManualOperationObject

def Pose2D_from_tf_concatenation(benchmark_object, new_target_frame, translation, orientation, original_target_frame, source_frame, tf_broadcaster, tf_listener, max_failed_attempts=None, timeout=1.0):
	
	attempts = 0
	pose_acquired = False
	pose2D = None
	
	while not pose_acquired and benchmark_object.is_benchmark_running():
		if max_failed_attempts != None and attempts >= max_failed_attempts:
			break
		
		try:
			now = rospy.Time.now()
			
			# testbed_origin_to_marker transform
			tf_listener.waitForTransform(source_frame, original_target_frame, now, rospy.Duration(timeout))
			
			# broadcast the robot frame relative to the marker
			tf_broadcaster.sendTransform(translation, orientation, now, new_target_frame, original_target_frame)
			
			# receive the robot transform and compute the robot pose
			tf_listener.waitForTransform(source_frame, new_target_frame, now, rospy.Duration(timeout))
			( (x, y, _), q ) = tf_listener.lookupTransform(source_frame, new_target_frame, rospy.Time(0))
			(_, _, theta) = tf.transformations.euler_from_quaternion(q)
			
			pose2D = Pose2D(x, y, theta)
			pose_acquired = True
		
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception), Argument:
			attempts += 1
			pose_acquired = False
			rospy.logwarn("Mocap pose could not be acquired")
			rospy.loginfo(Argument)
	
	return pose2D


class BenchmarkObject (BaseBenchmarkObject):
	
	benchmark_code = "HOPF"
	
	def execute(self):
		
		# Benchmark runs
		N = 10
		
		# Ground truth accuracy
		TRANS_ACCURACY = 0.0 # meters
		ROT_ACCURACY = 0.0 # degrees
		
		# init tf
		tf_listener = tf.TransformListener()
		tf_broadcaster = tf.TransformBroadcaster()
		
		# Get items
		items_transform_path = path.join( path.join(rospy.get_param("~resources_directory"), "objects_informations" ), "items_with_pose.yaml" )
		try:
			with open(items_transform_path, 'r') as items_transform_file:
				items = yaml.load(items_transform_file)["items"]
		except (EnvironmentError):
			rospy.logerr("Could not open items pose file [%s]" % (items_transform_path))
			return
		
		# Variables to compute score
		i = 0
		class_positives = 0.0
		instance_positives = 0.0
		pose_score_sum = 0.0
		execution_time = rospy.Duration(0)
		
		# Maximum pose error
		max_pose_error = sqrt(pow(float(self.params['table_size']['x']), 2) + pow(float(self.params['table_size']['y']), 2))
		
		# Start the benchmark
		while self.is_benchmark_running() and (i < N):
			i += 1
			
			# Pick a random item and remove it from the list
			item = random.choice(items)
			items.remove(item)
			self.score["goal_%i"%i] = {}
			
			##########################################
			#            MANUAL OPERATION            #
			##########################################
			
			manual_operation_1 = ManualOperationObject("Place the positioner and the %s (id: %d, instance: %s) on the table" % (item['description'], item['id'], item['instance']))
			
			self.request_manual_operation(manual_operation_1)
			
			
			##########################################
			#     CHECK RESULT AND UPDATE SCORE      #
			##########################################
			
			if manual_operation_1.has_been_completed():
				print "First Manual Operation result: %s" % manual_operation_1.get_result()
				self.score["goal_%i"%i]["manual_operation_1"] = manual_operation_1.get_result()
			else:
				print "First Manual Operation NOT EXECUTED"
				self.score["goal_%i"%i]["manual_operation_1"] = "not executed"
			
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
			
#			now = rospy.Time.now()
			
			##########################################
			#   ACQUIRE OBJECT POSITION WITH MOCAP   #
			##########################################
			
			acquired_object_pose = None
			acquired_object_pose = Pose2D_from_tf_concatenation(self, "/item", item['trans'], item['rot'], "/positioner_origin", "/table_origin", tf_broadcaster, tf_listener, max_failed_attempts=3)
			
			
			# If the position could not be acquired, try again by requesting to move the positioner
			while acquired_object_pose == None:
				rospy.logwarn("The position of the positioner could not be acquired")
				
				manual_operation_3 = ManualOperationObject("The position of the positioner could not be acquired!\nPlace in a different position the positioner and the %s (id: %d, instance: %s) on the table" % (item['description'], item['id'], item['instance']))
				
				self.request_manual_operation(manual_operation_3)
				
				
				##########################################
				#     CHECK RESULT AND UPDATE SCORE      #
				##########################################
				
				if manual_operation_3.has_been_completed():
					print "First Manual Operation result: %s" % manual_operation_3.get_result()
					self.score["goal_%i"%i]["manual_operation_3"] = manual_operation_3.get_result()
				else:
					print "First Manual Operation NOT EXECUTED"
					self.score["goal_%i"%i]["manual_operation_3"] = "not executed"
				
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
				
				acquired_object_pose = Pose2D_from_tf_concatenation(self, "/item", item['trans'], item['rot'], "/positioner_origin", "/table_origin", tf_broadcaster, tf_listener, max_failed_attempts=3)
				
			
			
			##########################################
			#            MANUAL OPERATION            #
			##########################################
			
			manual_operation_2 = ManualOperationObject("Remove the positioner")
			
			self.request_manual_operation(manual_operation_2)
			
			
			##########################################
			#     CHECK RESULT AND UPDATE SCORE      #
			##########################################
			
			if manual_operation_2.has_been_completed():
				print "First Manual Operation result: %s" % manual_operation_2.get_result()
				self.score["goal_%i"%i]["manual_operation_2"] = manual_operation_2.get_result()
			else:
				print "First Manual Operation NOT EXECUTED"
				self.score["goal_%i"%i]["manual_operation_2"] = "not executed"
			
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
			
			
			##########################################
			#                 GOAL i                 #
			##########################################
			
			goal = GoalObject({}, self.params['goal_timeout'])
			
			self.request_goal(goal)
			start_time = rospy.Time.now()
			
			print "wait_goal_result"
			
			self.wait_goal_result()
			end_time = rospy.Time.now()
			
			segment_time = end_time - start_time
			
			execution_time += segment_time
			rospy.loginfo("Execution time - %f" % execution_time.to_sec())
			
			
			##########################################
			#    CHECK RESULT i AND UPDATE SCORE     #
			##########################################
			
			self.score["goal_%i"%i]["timeout"] = goal.has_timed_out()
			self.score["goal_%i"%i]["completed"] = goal.has_been_completed()
			self.score["goal_%i"%i]["segment_time"] = segment_time.to_sec()
			
			if goal.has_timed_out():
				print "GOAL TIMEOUT"
			elif goal.has_been_completed():
				print "GOAL COMPLETED"
				print "RESULT:  %s" % (goal.get_result())
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
			
			
			if goal.has_been_completed():
				
				result = goal.get_result()
				
				result_object_pose = Pose2D(result['x'], result['y'], result['theta'])
				
				rospy.loginfo("Received result - X: %.2f \t Y: %.2f \t W: %.2f rad \t Class: %s \t Instance: %s"
					% (result_object_pose.x, result_object_pose.y, result_object_pose.theta, result['class'], result['instance']))
				
				# Evaluate position error
				tf_broadcaster.sendTransform([result_object_pose.x, result_object_pose.y, 0], tf.transformations.quaternion_from_euler(0, 0, result_object_pose.theta), rospy.Time.now(), "result_%d" % i, "origin")
				
				# Evaluate class
				is_class_correct = (result['class'] == item['class'])
				if (is_class_correct):
					class_positives += 1
				
				# Evaluate instance
				is_instance_correct = (result['instance'] == item['instance'])
				if (is_instance_correct):
					instance_positives += 1
				
				x_err = result_object_pose.x - acquired_object_pose.x
				y_err = result_object_pose.y - acquired_object_pose.y
				w_err = (result_object_pose.theta - acquired_object_pose.theta)
				
				if w_err > pi: w_err = w_err - 2*pi
				if w_err < -pi: w_err = w_err + 2*pi
				if abs(x_err) < TRANS_ACCURACY : x_err = 0.0
				if abs(y_err) < TRANS_ACCURACY : y_err = 0.0
				if abs(w_err) < ROT_ACCURACY : w_err = 0.0
				
				pose_err_trans = sqrt(pow(x_err, 2) + pow(y_err, 2))
#				if pose_err_trans > max_pose_error : pose_err_trans = 1.0 # corrected with version from Lisbon 2017 Aug event
				
				pose_err_rot = abs(w_err) / pi
				if pose_err_rot > 1 : pose_err_rot = 1.0
				
				pose_score = 1 - (pose_err_trans / 2) - (pose_err_rot / 2)
				pose_score_sum += pose_score
				
				# Update the score with the result of the current goal
				self.score["goal_%i"%i]['item_acquired'] = {}
				self.score["goal_%i"%i]['item_acquired']['pose'] = [acquired_object_pose.x, acquired_object_pose.y, acquired_object_pose.theta]
				self.score["goal_%i"%i]['item_acquired']['pose_units'] = ['m', 'm', 'rad']
				self.score["goal_%i"%i]['item_acquired']['class'] = item['class']
				self.score["goal_%i"%i]['item_acquired']['instance'] = item['instance']
				
				self.score["goal_%i"%i]['item_received'] = {}
				self.score["goal_%i"%i]['item_received']['pose'] = [result_object_pose.x, result_object_pose.y, result_object_pose.theta]
				self.score["goal_%i"%i]['item_received']['pose_units'] = ['m', 'm', 'rad']
				self.score["goal_%i"%i]['item_received']['class'] = result['class']
				self.score["goal_%i"%i]['item_received']['instance'] = result['instance']
				
				self.score["goal_%i"%i]['partial_score'] = {}
				self.score["goal_%i"%i]['partial_score']['pose_score'] = pose_score
				self.score["goal_%i"%i]['partial_score']['pose_error_translation_score'] = pose_err_trans
				self.score["goal_%i"%i]['partial_score']['pose_error_translation_score_unit'] = 'pure number'
				self.score["goal_%i"%i]['partial_score']['pose_error_rotation_score'] = pose_err_rot
				self.score["goal_%i"%i]['partial_score']['pose_error_rotation_score_unit'] = 'pure number'
				self.score["goal_%i"%i]['partial_score']['pose_error'] = [x_err, y_err, w_err]
				self.score["goal_%i"%i]['partial_score']['pose_error_units'] = ['m', 'm', 'rad']
				self.score["goal_%i"%i]['partial_score']['is_class_correct'] = is_class_correct
				self.score["goal_%i"%i]['partial_score']['is_instance_correct'] = is_instance_correct
				
				self.save_and_publish_score()
			
			# Update the global score with the result of the current goal (whether the goal was completed or not)
			self.score['class_accuracy'] = class_positives / i
			self.score['instance_accuracy'] = instance_positives / i
			self.score['pose_score'] = pose_score_sum / i
			self.score['execution_time'] = execution_time.to_sec()
			
			self.save_and_publish_score()
			
			
				
