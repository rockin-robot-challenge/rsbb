#!/usr/bin/env python
import rospy
import tf

rospy.init_node('FBM4_mock')

rospy.loginfo('\nFBM4 helper: emitting tf testbed_origin -> actor_markerset where X:0.2, Y:0.2, Theta:0.8 at 120Hz.')

# Emit transform continuously: actor_markerset to testbed_origin
tf_broadcaster = tf.TransformBroadcaster()

def tf_callback(event):

    tf_broadcaster.sendTransform(
        (0.2, 0.2, 1),
        tf.transformations.quaternion_from_euler(0, 0, 0.8),
        rospy.Time.now(),
        "actor_markerset",
        "testbed_origin"
    )

rospy.Timer(rospy.Duration(1.0/120.0), tf_callback)  # 120Hz, like MoCap

rospy.spin()
