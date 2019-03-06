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
		MAX_CONSECUTIVE_MISSING_POSES = 20 # TODO remove this mechanism
		
		# Load acceptable distance from config
		MIN_FOLLOWING_DISTANCE = self.params['min_following_distance']
		MAX_FOLLOWING_DISTANCE = self.params['max_following_distance']
		DESIRED_FOLLOWING_DISTANCE = self.params['desired_following_distance']

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
		mocap_success_sum = 0
		mocap_failure_sum = 0
		deviation_from_desired_distance_sum = 0
		distance_covered_while_following = 0
		last_absolute_robot_pose = None
		
		# Start the benchmark
		while self.is_benchmark_running():
			self.score = {}

			## Check if both person and robot's poses can be acquired
			robot_to_human_pose = None
			robot_to_human_pose = Pose2D_from_tf_concatenation(self, "/robot", markerset_to_robot_transform[0], markerset_to_robot_transform[1], "/robot_markerset", "/actor_markerset", tf_broadcaster, tf_listener, max_failed_attempts=3, timeout=0.5)
			while robot_to_human_pose == None:
				manual_operation = ManualOperationObject("Failed to acquire pose of human or robot. Make sure they have detectable markersets.")
				self.request_manual_operation(manual_operation)
				robot_to_human_pose = Pose2D_from_tf_concatenation(self, "/robot", markerset_to_robot_transform[0], markerset_to_robot_transform[1], "/robot_markerset", "/actor_markerset", tf_broadcaster, tf_listener, max_failed_attempts=3, timeout=0.5)

			## Send request for robot to start following
			goal = GoalObject({}, self.params['goal_timeout'])
			self.request_goal(goal)

			## Start tracking pose of human and robot.
			benchmark_start_time = rospy.Time.now()

			# Sleep for 8 seconds so that the robot has time to position itself at the desired following distance.
			rospy.sleep(8.0)

			while (not goal.has_timed_out()) and (not goal._executed):
				sample_start_time = rospy.Time.now()
				
				## Acquire robot_markerset in relation to actor_markerset
				robot_to_human_pose = None
				robot_to_human_pose = Pose2D_from_tf_concatenation(self, "/robot", markerset_to_robot_transform[0], markerset_to_robot_transform[1], "/robot_markerset", "/actor_markerset", tf_broadcaster, tf_listener, max_failed_attempts=1, timeout=0.2)

				absolute_robot_pose = None
				absolute_robot_pose = Pose2D_from_tf_concatenation(self, "/robot", markerset_to_robot_transform[0], markerset_to_robot_transform[1], "/robot_markerset", "/testbed_origin", tf_broadcaster, tf_listener, max_failed_attempts=1, timeout=0.2)
				
				## Update score
				if robot_to_human_pose != None:
					mocap_success_sum += 1

					rospy.loginfo('robot_to_human_pose: (%f, %f)' % (robot_to_human_pose.x, robot_to_human_pose.y))

					# Calculate distance
					distance = sqrt( robot_to_human_pose.x**2 + robot_to_human_pose.y**2 )
					rospy.loginfo('Current following distance: %f' % distance)

					deviation_from_desired_distance = abs(distance - DESIRED_FOLLOWING_DISTANCE)
					deviation_from_desired_distance_sum += deviation_from_desired_distance

					if (distance >= MIN_FOLLOWING_DISTANCE and distance <= MAX_FOLLOWING_DISTANCE):
						# Update distance_covered_while_following
						if last_absolute_robot_pose != None and absolute_robot_pose != None:
							x_dist = absolute_robot_pose.x - last_absolute_robot_pose.x
							y_dist = absolute_robot_pose.y - last_absolute_robot_pose.y
							dist = sqrt( x_dist**2 + y_dist**2 )
							distance_covered_while_following += dist

				else:
					mocap_failure_sum += 1

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
				self.abort_benchmark(message='Benchmark failed! Finished prematurely or took too long')
				return

			# Calculate score statistics and print.
			if mocap_success_sum > 0:
				self.score['Average deviation from desired distance'] = deviation_from_desired_distance_sum / float(mocap_success_sum)				
			self.score['Distance covered while following'] = distance_covered_while_following

			percent_mocap_failures = mocap_failure_sum / float(mocap_success_sum + mocap_failure_sum) * 100
			self.score['Benchmark reliability'] = str(round(100 - percent_mocap_failures, 1)) + '%'

			if percent_mocap_failures > 50:
				self.score['WARNING'] = ('%.1f%% of the time, mocap pose acquiring failed. Consider repeating the benchmark.' % percent_mocap_failures)
			
			self.save_and_publish_score()

			# Since the goal hasn't finished, simulate clicking 'Stop' on the GUI
			self.abort_benchmark()
			return