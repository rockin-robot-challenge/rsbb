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
	
	benchmark_code = "HPFF"
	
	def execute(self):
		# Maximum consecutive times where pose-acquisition fails
		MAX_CONSECUTIVE_MISSING_POSES = 20
		
		# Load acceptable distance range from config
		MIN_ACCEPTABLE_DISTANCE = self.params['min_acceptable_following_distance']
		MAX_ACCEPTABLE_DISTANCE = self.params['max_acceptable_following_distance']

		# Read markerset_to_robot transform from the transform file
		team_name = self.get_benchmark_team()
		transform_path = path.join( path.join(rospy.get_param("~resources_directory"), "transforms" ), "transform-%s.yaml" % team_name )
		
		with open(transform_path, 'r') as transform_file:
			markerset_to_robot_transform = yaml.load(transform_file)["markerset_to_robot_transform"]
		
		print "team_name: ", team_name, "\ttransform_path: ", transform_path, "\tmarkerset_to_robot_transform: ", markerset_to_robot_transform

		# init tf
		tf_listener = tf.TransformListener()
		tf_broadcaster = tf.TransformBroadcaster()
		
		# Variables to compute score
		distance_samples = 0
		distance_samples_within_tolerance = 0
		distance_covered_while_following = 0
		consecutive_missing_poses = 0
		last_absolute_robot_pose = None
		
		# Start the benchmark
		while self.is_benchmark_running():
			self.score = {}

			## Send request for robot to start following
			goal = GoalObject({}, self.params['goal_timeout'])
			self.request_goal(goal)

			## Start tracking pose of human and robot.
			benchmark_start_time = rospy.Time.now()

			while (not goal.has_timed_out()) and (not goal._executed):
				sample_start_time = rospy.Time.now()

				## Acquire robot_markerset in relation to actor_markerset
				robot_to_human_pose = None
				robot_to_human_pose = Pose2D_from_tf_concatenation(self, "/robot", markerset_to_robot_transform[0], markerset_to_robot_transform[1], "/robot_markerset", "/actor_markerset", tf_broadcaster, tf_listener, max_failed_attempts=1, timeout=0.2)

				absolute_robot_pose = None
				absolute_robot_pose = Pose2D_from_tf_concatenation(self, "/robot", markerset_to_robot_transform[0], markerset_to_robot_transform[1], "/robot_markerset", "/testbed_origin", tf_broadcaster, tf_listener, max_failed_attempts=1, timeout=0.2)
				
				## Update score
				if robot_to_human_pose != None:
					rospy.loginfo('robot_to_human_pose: (%f, %f)' % (robot_to_human_pose.x, robot_to_human_pose.y))

					consecutive_missing_poses = 0
					distance_samples += 1

					# Calculate distance
					distance = sqrt( robot_to_human_pose.x**2 + robot_to_human_pose.y**2 )
					rospy.loginfo('Current following distance: %f' % distance)

					if (distance >= MIN_ACCEPTABLE_DISTANCE and distance <= MAX_ACCEPTABLE_DISTANCE):
						distance_samples_within_tolerance += 1

						# Update distance_covered_while_following
						if last_absolute_robot_pose != None and absolute_robot_pose != None:
							x_dist = absolute_robot_pose.x - last_absolute_robot_pose.x
							y_dist = absolute_robot_pose.y - last_absolute_robot_pose.y
							dist = sqrt( x_dist**2 + y_dist**2 )
							distance_covered_while_following += dist

				else:
					consecutive_missing_poses += 1
					if consecutive_missing_poses > MAX_CONSECUTIVE_MISSING_POSES:
						print "Robot and person poses missing for too much time. Benchmark failed."
						rospy.logerr("Robot and person poses missing for too much time. Benchmark failed.")
						self.abort_benchmark(message='Benchmark failed!')
						# TODO possibly later add a pause_benchmark service to core and call it here instead of aborting (see commented code below)
						return

						# Below can't be done, because a manual operation can't be started while executing a goal
						# while robot_to_human_pose == None:
						# 	manual_operation = ManualOperationObject("Failed to acquire poses for too long. Please make sure the robot and person are trackable by the MoCap system.")
						# 	self.request_manual_operation(manual_operation)

						# 	robot_to_human_pose = None
						# 	robot_to_human_pose = Pose2D_from_tf_concatenation(self, "/robot", markerset_to_robot_transform[0], markerset_to_robot_transform[1], "/robot_markerset", "/actor_markerset", tf_broadcaster, tf_listener, max_failed_attempts=1, timeout=0.2)

						# 	absolute_robot_pose = None
						# 	absolute_robot_pose = Pose2D_from_tf_concatenation(self, "/robot", markerset_to_robot_transform[0], markerset_to_robot_transform[1], "/robot_markerset", "/testbed_origin", tf_broadcaster, tf_listener, max_failed_attempts=1, timeout=0.2)

				if absolute_robot_pose != None:
					last_absolute_robot_pose = absolute_robot_pose

				## Sleep for the remaining amount of time to 0.5s (position should be sampled every 0.5s)
				current_time = rospy.Time.now()
				elapsed_time = current_time - sample_start_time
				remaining_time = 0.5 - elapsed_time.to_sec()
				if remaining_time > 0:
					rospy.sleep(remaining_time)

				if not self.is_benchmark_running():
					if self.has_benchmark_timed_out():
						print "BENCHMARK TIMEOUT"
					elif self.has_benchmark_been_stopped():
						print "BENCHMARK STOPPED"
					else:
						print "BENCHMARK ABORTED"
					return

			# Check if benchmark duration was within 15 seconds of the goal_timeout
			benchmark_end_time = rospy.Time.now()
			benchmark_duration = (benchmark_end_time - benchmark_start_time).to_sec()
			target_duration = self.params['goal_timeout']
			if benchmark_duration < (target_duration-15) or benchmark_duration > (target_duration+15):
				rospy.logerr('Benchmark failed: finished prematurely or took too long')
				self.score['Result'] = 'Benchmark failed: finished prematurely or took too long'
				self.save_and_publish_score()
				self.abort_benchmark()
				return

			# Calculate score statistics and print.
			if distance_samples > 0:
				self.score['Acceptable distance accuracy'] = distance_samples_within_tolerance / float(distance_samples)
				self.score['Distance covered while following'] = distance_covered_while_following
			
			self.save_and_publish_score()

			# Since the goal hasn't finished, simulate clicking 'Stop' on the GUI
			self.abort_benchmark()
			return