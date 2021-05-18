#!/usr/bin/python

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
import numpy as np
import geometry_msgs.msg


def main():
    rospy.init_node('tf_reader')
    pub = rospy.Publisher('/end_effector_pose', geometry_msgs.msg.Pose2D, queue_size=1)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world_frame', 'arm_link_5', rospy.Time())
            tf = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            tf = list(np.around(np.array(tf), 3))
            quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            (roll, pitch, yaw) = euler_from_quaternion(quat)
            rot = [roll, pitch, yaw]
            rot = list(np.around(np.array(rot), 2))
            ee_pose = geometry_msgs.msg.Pose2D()
            ee_pose.x = tf[0]
            ee_pose.y = tf[1]
            ee_pose.theta = rot[2]
            pub.publish(ee_pose)
            rate.sleep()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
if __name__ == '__main__':
    main()


# rosrun tf static_transform_publisher  0.4 1.0 0 0 3.141 0 arm_link_0 world_frame 10
