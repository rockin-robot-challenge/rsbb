#!/usr/bin/env python
# -*- coding: utf-8 -*-

from os import path
from time import strftime
from datetime import date, datetime
from dateutil.tz import tzlocal

import rospy, yaml
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

#####################################################################################
###    NOTE    																	  ###
#####################################################################################
#																					#
#	It is advisable to set the smoothing to 100 in Motive (as high as possible)		#
#	before acquiring the markerset-robot transform.									#
#	The appropriate smoothing for each configuration/markerset is also described	#
#	in the documentation "The RoCKIn Benchmarking System".							#
#																					#
#####################################################################################










if __name__ == '__main__':
	rospy.init_node('robot_markerset_transform_acquirer', anonymous=True)
	
	raw_team_name = raw_input("Insert the name of the team (without spaces): ")
	team_name = raw_team_name.replace(" ", "")
	
	transform_path = path.join( path.join(rospy.get_param("~resources_directory"), "transforms" ), "transform-%s.yaml" % team_name )
	
	transform_output = {}
	transform_output['team_name'] = team_name
	
	print "\n\n\n", "team_name", team_name
	print "\n\n\n", "transform_path", transform_path
	print "\n\n\n"
		
	# init tf and subscribers
	tf_listener = tf.TransformListener()
	
	# Acquire the markerset_robot transform
	try:
		
		print "waiting to receive tf data"
		
		rospy.sleep(rospy.Duration(1.0))
		
		print "waitForTransform"
		
		tf_listener.waitForTransform("/testbed_origin", "/robot_markerset", rospy.Time(0), rospy.Duration(1.0))
		
		now = rospy.Time.now()
		
		print "acquiring markerset_to_robot_transform"
		
		markerset_to_robot_transform = tf_listener.lookupTransform("/robot_markerset", "/testbed_origin", rospy.Time(0))
		
		print "acquiring robot_to_markerset_transform"
		
		robot_to_markerset_transform = tf_listener.lookupTransform("/testbed_origin", "/robot_markerset", rospy.Time(0))
		
		print "/robot_markerset to /testbed_origin", markerset_to_robot_transform
		
		print "/testbed_origin to /robot_markerset", robot_to_markerset_transform
		
		transform_output['transform_timestamp'] = datetime.now(tzlocal()).strftime("%Y-%m-%d_%H:%M:%S_%Z(%z)")
		transform_output['robot_to_markerset_transform'] = robot_to_markerset_transform
		transform_output['markerset_to_robot_transform'] = markerset_to_robot_transform
		
		
		print "\n\n\n", "transform_output", transform_output
		print "\n\n\n"
		
		with open(transform_path, 'w') as transform_file:
			transform_file.write(yaml.dump(transform_output))
		
	except (tf.Exception):#tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), Argument:
		rospy.logerr("Markerset position could not be acquired!")
#		raise
	
	rospy.signal_shutdown("")

