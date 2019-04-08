#!/usr/bin/env python
import rospy
import tf

rospy.init_node('FBM6_mock')

rospy.loginfo('''
FBM6 helper: Emitting 2 tfs: testbed_origin->table_origin and table_origin->actor_markerset at 120Hz.
table_origin (x,y) = (1, 1) in relation to testbed. actor_markerset (x,y) = (0.5, 0.5) in relation to table.
''')

# Emit transforms
tf_broadcaster = tf.TransformBroadcaster()

def tf_callback(event):

    tf_broadcaster.sendTransform(
        (1, 1, 1),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "table_origin",
        "testbed_origin"
    )

    tf_broadcaster.sendTransform(
        (0.5, 0.5, 0.1),
        tf.transformations.quaternion_from_euler(0, 0, 0.8),
        rospy.Time.now(),
        "actor_markerset",
        "table_origin"
    )

rospy.Timer(rospy.Duration(1.0/120.0), tf_callback)  # 120Hz, like MoCap

rospy.spin()
