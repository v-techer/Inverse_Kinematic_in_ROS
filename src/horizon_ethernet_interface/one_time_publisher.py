#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped

pubTopic = ''
rosNodeName = 'one_time_publisher'

if __name__ == '__main__':	
	
	rospy.init_node(rosNodeName, anonymous=True)
	
	pubMAP_ODOM = tf2_ros.TransformBroadcaster()
	pubAR_POSE_ESTIM = rospy.Publisher('horizon/ar_pose_estim',PoseWithCovarianceStamped, queue_size = 1)
	
	t_msg = PoseWithCovarianceStamped()
	t_msg.header.stamp = rospy.Time.now()
	t_msg.header.frame_id = 'map'
	t_msg.pose.pose.position.x = 1.10	
	t_msg.pose.pose.position.y = 8.5
	t_msg.pose.pose.position.z = 0
	t_msg.pose.pose.orientation.x = 0
	t_msg.pose.pose.orientation.y = 0
	t_msg.pose.pose.orientation.z = 0
	t_msg.pose.pose.orientation.w = 1
	t_msg.pose.covariance =  [	5.0,0.0,0.0,0.0,0.0,0.0,
								0.0,5.0,0.0,0.0,0.0,0.0,
								0.0,0.0,0.0,0.0,0.0,0.0,
								0.0,0.0,0.0,0.0,0.0,0.0,
								0.0,0.0,0.0,0.0,0.0,0.0,
								0.0,0.0,0.0,0.0,0.0,0.0]
	
	rate = rospy.Rate(1.0)
	cnt = 0
	
	while not rospy.is_shutdown():
		#t.header.stamp = rospy.Time.now()
		t_msg.header.stamp = rospy.Time.now()
		pubAR_POSE_ESTIM.publish(t_msg)	
		#pubMAP_ODOM.sendTransform(t)
		rate.sleep()
		cnt += 1
		if cnt == 5:
			break
	
