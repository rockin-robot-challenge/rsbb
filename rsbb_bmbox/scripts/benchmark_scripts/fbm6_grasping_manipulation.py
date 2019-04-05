#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, yaml, tf, random
from math import pi, sin, cos, atan2, fabs, sqrt, pow
from geometry_msgs.msg import Pose2D
from os import path

from rsbb_bmbox.BenchmarkObjects import BaseBenchmarkObject, GoalObject, ManualOperationObject

object_grasped = False
object_height_reference = 0.0
tf_listener = None

def Pose2D_from_tf_concatenation(benchmark_object, new_target_frame, translation, orientation, original_target_frame, source_frame, tf_broadcaster, tf_listener, max_failed_attempts=None, timeout=1.0):
	global object_height_reference
	
	attempts = 0
	pose_acquired = False
	pose2D = None
	height = None
	
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
			( (x, y, height), q ) = tf_listener.lookupTransform(source_frame, new_target_frame, rospy.Time(0))
			(_, _, theta) = tf.transformations.euler_from_quaternion(q)
			
			pose2D = Pose2D(x, y, theta)
			pose_acquired = True
		
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception), Argument:
			attempts += 1
			pose_acquired = False
			rospy.logwarn("Mocap pose could not be acquired")
			rospy.loginfo(Argument)
	
	return (pose2D, height)

class BenchmarkObject (BaseBenchmarkObject):
	
	benchmark_code = "HGMF"
	
	def execute(self):
		global tf_listener
		global object_height_reference
		global object_grasped

        # Load the list of objects from the config file
		objects = self.params["objects"]

		# Num. of requested goals: equal to the num. of objects
		N = len(objects)
		
		# Calculate range where objects can be placed: 15cm margin from the table edges
		table_size = self.params["table_size"]
		min_x = 0.15
		min_y = 0.15
		max_x = table_size['x'] - 0.15
		max_y = table_size['y'] - 0.15

		# init tf
		tf_broadcaster = tf.TransformBroadcaster()
		tf_listener = tf.TransformListener()
		
		# Variables to compute score
		i = 0
		objects_grasped_sum = 0
		goals_completed_sum = 0
		distance_error_sum = 0
		angle_error_sum = 0
		execution_time = rospy.Duration(0)
		
		# Start the benchmark
		while self.is_benchmark_running() and (i < N):
			# Pick a random object and remove it from the list
			obj = random.choice(objects)
			objects.remove(obj)
			self.score["goal_%i" % i] = {}
			object_grasped = False

			# Pick a random target position for the object
			target_x = random.uniform(min_x, max_x)
			target_y = random.uniform(min_y, max_y)
			
			##########################################
			#            MANUAL OPERATION            #
			##########################################
			
			manual_operation_1 = ManualOperationObject("Place the %s on the table, with MoCap markers attached." % obj['description'])
			
			self.request_manual_operation(manual_operation_1)
			
			##########################################
			#     CHECK RESULT AND UPDATE SCORE      #
			##########################################
			
			if manual_operation_1.has_been_completed():
				print "First Manual Operation result: %s" % manual_operation_1.get_result()
			else:
				print "First Manual Operation NOT EXECUTED"
			
			if not self.is_benchmark_running():
				if self.has_benchmark_timed_out():
					print "BENCHMARK TIMEOUT"
				elif self.has_benchmark_been_stopped():
					print "BENCHMARK STOPPED"
				else:
					print "BENCHMARK ABORTED"
				return

			## Fist manual operation finished, the object should be trackable on the table
			# Check if we can get the object's pose, and if it's not too close to the target position
			object_pose_mocap = None
			# We assume the markerset is perfectly aligned with the object - if it's not, add a markerset-acquiring step and use the results here
			(object_pose_mocap, object_height_reference) = Pose2D_from_tf_concatenation(self, "/object", (0, 0, 0), (0, 0, 0, 1), "/actor_markerset", "/table_origin", tf_broadcaster, tf_listener, max_failed_attempts=3)
			
			while object_pose_mocap == None:
				rospy.logwarn("The position of the object markerset could not be acquired - asking RefOp to retry")
				manual_operation_2 = ManualOperationObject("Could not get the %s's pose. Make sure it's inside the designated area with a markerset attached." % obj['description'])

				self.request_manual_operation(manual_operation_2)
			
				if manual_operation_2.has_been_completed():
					print "Second Manual Operation result: %s" % manual_operation_2.get_result()
				else:
					print "Second Manual Operation NOT EXECUTED"

				if not self.is_benchmark_running():
					if self.has_benchmark_timed_out():
						print "BENCHMARK TIMEOUT"
					elif self.has_benchmark_been_stopped():
						print "BENCHMARK STOPPED"
					else:
						print "BENCHMARK ABORTED"
					return
				
				(object_pose_mocap, object_height_reference) = Pose2D_from_tf_concatenation(self, "/object", (0, 0, 0), (0, 0, 0, 1), "/actor_markerset", "/table_origin", tf_broadcaster, tf_listener, max_failed_attempts=3)
			
			# If target location is too close to the object, pick a different one
			distance_from_target = sqrt((object_pose_mocap.x - target_x)**2 + (object_pose_mocap.y - target_y)**2)
			while distance_from_target < 0.05:
				target_x = random.uniform(min_x, max_x)
				target_y = random.uniform(min_y, max_y)
				distance_from_target = sqrt((object_pose_mocap.x - target_x)**2 + (object_pose_mocap.y - target_y)**2)

			#########################################################
			#   OBJECT IS IN THE TABLE, SEND THE GOAL TO THE ROBOT  #
			#########################################################
			
			##########################################
			#                 GOAL i                 #
			##########################################
			
			goal = GoalObject([target_x, target_y, obj['id']], self.params['goal_timeout'])
			
			self.request_goal(goal)
			start_time = rospy.Time.now()
			
			# Start a timer: 10x / second, acquire the object pose and check if it has lifted from the table.
			timer = rospy.Timer(rospy.Duration(0.1), timer_callback)

			print "wait_goal_result"

			self.wait_goal_result()
			end_time = rospy.Time.now()

			timer.shutdown()
			
			segment_time = end_time - start_time
			
			execution_time += segment_time

			self.score["goal_%i" % i]["segment_time [s]"] = segment_time.to_sec()
			self.save_and_publish_score()
			rospy.loginfo("segment_time: %f" % segment_time.to_sec())
			
			##########################################
			#    CHECK RESULT i AND UPDATE SCORE     #
			##########################################
			
			self.score["goal_%i" % i]["timeout"] = goal.has_timed_out()
			self.score["goal_%i"% i ]["completed"] = goal.has_been_completed()
			
			if goal.has_timed_out():
				print "GOAL TIMEOUT"
			elif goal.has_been_completed():
				print "GOAL COMPLETED"
				print "RESULT: %s" % (goal.get_result())
			else:
				print "GOAL NOT COMPLETED"
			
			if goal.has_been_completed():
				goals_completed_sum += 1

				if object_grasped:
					objects_grasped_sum += 1

				# Acquire object position from actor_markerset, which is the ground truth
				object_pose_mocap = None
				object_pose_mocap = Pose2D_from_tf_concatenation(self, "/object", (0, 0, 0), (0, 0, 0, 1), "/actor_markerset", "/table_origin", tf_broadcaster, tf_listener, max_failed_attempts=3)[0]
				if object_pose_mocap == None:
					# Robot has finished the goal, but the pose couldn't be acquired from MoCap: restart the goal
					rospy.logwarn("The position of the person markerset could not be acquired - restarting current goal")

					execution_time -= segment_time
					objects.append(obj)

					continue

				x_err = target_x - object_pose_mocap.x
				y_err = target_y - object_pose_mocap.y
				distance_error = sqrt( x_err**2 + y_err**2 )

				distance_error_sum += distance_error

				# TODO no orientation error for now: should it be added?
				
				# Update the score with the result of the current goal
				self.score["goal_%i" % i]['Object grasped'] = object_grasped
				self.score["goal_%i" % i]['Distance error'] = distance_error
				self.score["goal_%i" % i]['Target object position'] = '(%.2f, %.2f)' % (target_x, target_y)
				self.score["goal_%i" % i]['Measured object position'] = '(%.2f, %.2f)' % (object_pose_mocap.x, object_pose_mocap.y)
				self.score["goal_%i" % i]['Object type'] = '%s (id %d)' % (obj['description'], obj['id'])

				# Update overall score
				self.score["Objects grasped"] = str(objects_grasped_sum) + " out of " + str(N)
				self.score['Avg distance error'] = distance_error_sum / goals_completed_sum
				
			else:
				self.score["goal_%i" % i]['result'] = 'Not completed'
			
			# Update execution time whether the goal was completed or not
			self.score['execution_time'] = execution_time.to_sec()
			
			self.save_and_publish_score()

			if not self.is_benchmark_running():
				if self.has_benchmark_timed_out():
					print "BENCHMARK TIMEOUT"
				elif self.has_benchmark_been_stopped():
					print "BENCHMARK STOPPED"
				else:
					print "BENCHMARK ABORTED"
				return

			i += 1

def timer_callback(event):
	global object_grasped

	try:
		# check if Z increased by more than 3cm. If it did, set object_grasped to True
		now = rospy.Time.now()
		tf_listener.waitForTransform('/table_origin', '/actor_markerset', now, rospy.Duration(0.1))
		( (_, _, current_object_height), _ ) = tf_listener.lookupTransform('/table_origin', '/actor_markerset', rospy.Time(0))

		if current_object_height > (object_height_reference + 0.03):
			object_grasped = True

	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception), _:
		return