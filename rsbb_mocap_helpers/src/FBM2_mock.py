#!/usr/bin/env python  
import rospy
import tf

rospy.init_node('FBM2_mock')

rospy.loginfo('\nFBM2 helper: emitting tf testbed_origin->robot_markerset @120Hz where the robot is at (0,0) aligned with the origin.')

# Emit transform continuously: robot_markerset to testbed_origin
tf_broadcaster = tf.TransformBroadcaster()

def tf_callback(event):
    tf_broadcaster.sendTransform(
        (0, 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "robot_markerset",
        "testbed_origin"
    )

rospy.Timer(rospy.Duration(1.0/120.0), tf_callback) # 120Hz, like MoCap

rospy.spin()
