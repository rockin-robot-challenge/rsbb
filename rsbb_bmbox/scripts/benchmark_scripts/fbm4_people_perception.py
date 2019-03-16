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
	
	benchmark_code = "HPPF"
	
	def execute(self):
        # Load the list of people's names from the config file
		people = self.params["people"]

		# Num. of requested goals: equal to the num. of people
		N = len(people)
		
		# init tf
		tf_listener = tf.TransformListener()
		tf_broadcaster = tf.TransformBroadcaster()
		
		# Variables to compute score
		i = 0
		detected_sum = 0
		recognized_sum = 0
		position_error_sum = 0
		execution_time = rospy.Duration(0)
		
		# Start the benchmark
		while self.is_benchmark_running() and (i < N):
			# Pick a random person and remove it from the list
			person = random.choice(people)
			people.remove(person)
			self.score["goal_%i" % i] = {}
			
			##########################################
			#            MANUAL OPERATION            #
			##########################################
			
			manual_operation_1 = ManualOperationObject("%s should enter the target area wearing a cap with MoCap markers, and place him/herself in an arbitrary position." % person)
			
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

			## Fist manual operation finished, the person should be trackable inside the area
			# Check if we can get the person's pose - although we don't use the pose now, only after the robot's end_execute signal
			person_pose_mocap = None
			person_pose_mocap = Pose2D_from_tf_concatenation(self, "/person", (0, 0, 0), (0, 0, 0, 1), "/actor_markerset", "/testbed_origin", tf_broadcaster, tf_listener, max_failed_attempts=3)
			while person_pose_mocap == None:
				rospy.logwarn("The position of the person markerset could not be acquired - asking RefOp to retry")
				manual_operation_2 = ManualOperationObject("Could not get %s's pose. Make sure (s)he is inside the designated area and wearing the markerset cap." % person)

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
				
				person_pose_mocap = Pose2D_from_tf_concatenation(self, "/person", (0, 0, 0), (0, 0, 0, 1), "/actor_markerset", "/testbed_origin", tf_broadcaster, tf_listener, max_failed_attempts=3)
			
			#########################################################
			#   PERSON IS IN THE AREA, SEND THE GOAL TO THE ROBOT   #
			#########################################################
			
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

				# Acquire person position from actor_markerset, which is the ground truth
				person_pose_mocap = None
				person_pose_mocap = Pose2D_from_tf_concatenation(self, "/person", (0, 0, 0), (0, 0, 0, 1), "/actor_markerset", "/testbed_origin", tf_broadcaster, tf_listener, max_failed_attempts=3)
				if person_pose_mocap == None:
					# Robot has finished the goal, but the pose couldn't be acquired from MoCap: restart the goal
					rospy.logwarn("The position of the person markerset could not be acquired - restarting current goal")

					execution_time -= segment_time
					people.append(person)

					continue

				detected_sum += 1
				
				result = goal.get_result()

				rospy.loginfo("Received result - X: %.2f \t Y: %.2f \t Theta: %.2f \t Person name: %s"
					% (result['x'], result['y'], result['theta'], result['person_name']))

				# Compare person name
				recognized = (result['person_name'] == person)
				if recognized:
					recognized_sum += 1
				
				# Calculate position error
				result_person_pose = Pose2D(result['x'], result['y'], result['theta'])

				x_err = result_person_pose.x - person_pose_mocap.x
				y_err = result_person_pose.y - person_pose_mocap.y
				position_error = sqrt( x_err**2 + y_err**2 )

				position_error_sum += position_error

				# TODO no orientation error for now: should it be added?
				
				# Update the score with the result of the current goal
				self.score["goal_%i" % i]['detected'] = True
				self.score["goal_%i" % i]['recognized'] = recognized
				self.score["goal_%i" % i]['position_error'] = position_error

				# Update overall score
				self.score['Recognized persons'] = recognized_sum
				self.score['% Recognized persons'] = str(round(recognized_sum / float(N) * 100, 2)) + '%'
				self.score['Distance error'] = position_error_sum / detected_sum
				
			else:
				self.score["goal_%i" % i]['detected'] = False
			
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
