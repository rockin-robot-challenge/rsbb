#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
#import os
#import getopt
#import threading
from time import gmtime, strftime
from os import path
from time import strftime
from datetime import date, datetime
from dateutil.tz import tzlocal

import rospy, yaml
import tf
#import math
#import geometry_msgs.msg
#import roslib

#origin = {'trans': [0, 0, 0], 'rot': [0, 0, 0, 1]}
#object_id = None

class colors:
	INPUT = '\033[96m'
	INFO = '\033[92m'
	WARN = '\033[93m'
	FAIL = '\033[91m'
	END = '\033[0m'
	
def main():
#	try:
#		opts, args = getopt.getopt(sys.argv[1:], "hi:")
#	except getopt.GetoptError:
#		print "Usage: " + sys.argv[0] + " -i <id>"
#		sys.exit()
#	for opt, arg in opts:
#		if opt == '-h':
#			print "Usage: " + sys.argv[0] + " -i <id>"
#			sys.exit()
#		elif opt == '-i':
#			object_id = arg

	rospy.init_node("acquire_objects")
	
	tf_listener = tf.TransformListener()
#	br = tf.TransformBroadcaster()
	
	items_list_path = path.join( path.join(rospy.get_param("~resources_directory"), "objects_informations" ), "items_list.yaml" )
	items_transform_output_path = path.join( path.join(rospy.get_param("~resources_directory"), "objects_informations" ), "items_with_pose.yaml" )

	print "\n\n\n", "items_list_path", items_list_path
	print "\n\n\n", "items_transform_output_path", items_transform_output_path
	print "\n\n\n"	
	
	with open(items_list_path, 'r') as items_list_file:
		items_list = yaml.load(items_list_file)['items']
		
	print "\n\n\n",  "items_list: ", items_list
	print "\n\n\n"	
	
	items_transform_output = {'items': []}
	
	# wait to be sure that the tf buffer contains enough data
	rospy.sleep(rospy.Duration(1.0))
	
	for item in items_list:
		
		if rospy.is_shutdown(): break
		
		try:
			a = raw_input(colors.INPUT + "Place item '%s' (id: '%d', instance: '%s') against the positioner and press ENTER" % (item['description'], item['id'], item['instance']) + colors.END)
			
			if rospy.is_shutdown(): break
			
			tf_listener.waitForTransform("positioner_origin", "table_origin", rospy.Time(0), rospy.Duration(1.0))
#			now = rospy.Time.now()
			(ref_trans, ref_rot) = tf_listener.lookupTransform("positioner_origin", "table_origin", rospy.Time(0))
			ref_euler = tf.transformations.euler_from_quaternion(ref_rot)
			
			print colors.INFO + "Item acquired" + colors.END
			print "X: %2.5f\tY: %2.5f\tZ: %2.5f\tW: %3.2f rad\tW: %3.2f °" % (ref_trans[0], ref_trans[1], ref_trans[2], ref_euler[2], ref_euler[2] * 57.2957795)
			
		except (tf.Exception):#tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print colors.WARN + "Positioner not visible" + colors.END
			raise
		
		items_transform_output['items'].append({'trans': ref_trans, 'rot': ref_rot, 'id': item['id'], 'class': item['class'], 'instance': item['instance'], 'description': item['description']})
	
	if rospy.is_shutdown(): return
	
	confirm = ''
	while confirm != 'y':
		confirm = raw_input(colors.INPUT + "Confirm overwriting the output file 'objects_informations/items_with_pose.yaml' by typing 'y', to abort press Ctrl+C then ENTER (all measurements will be lost)\n" + colors.END)
		if rospy.is_shutdown(): return
	
	with open(items_transform_output_path, 'w') as items_transform_output_file:
		items_transform_output_file.write(yaml.dump(items_transform_output))
		rospy.loginfo("\n\nMeasurements successfully written to '%s'!\n\n" % (items_transform_output_path))
	
if __name__ == '__main__':
	main()

