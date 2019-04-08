#!/usr/bin/env python
import rospy
import tf

rospy.init_node('FBM5_mock')

rospy.loginfo('''
FBM5 helper: Emitting 2 tfs: testbed_origin->actor_markerset and testbed_origin->robot_markerset at 120Hz.
actor_markerset (x,y) = (0.5, 0.5). robot_markerset (x,y) = (1, 1)
''')

# Emit transforms continuously: robot_markerset and actor_markerset to testbed_origin
tf_broadcaster = tf.TransformBroadcaster()

def tf_callback(event):

    tf_broadcaster.sendTransform(
        (0.5, 0.5, 1),
        tf.transformations.quaternion_from_euler(0, 0, 0.8),
        rospy.Time.now(),
        "actor_markerset",
        "testbed_origin"
    )

    tf_broadcaster.sendTransform(
        (1, 1, 1),
        tf.transformations.quaternion_from_euler(0, 0, 0.8),
        rospy.Time.now(),
        "robot_markerset",
        "testbed_origin"
    )

rospy.Timer(rospy.Duration(1.0/120.0), tf_callback)  # 120Hz, like MoCap

rospy.spin()
