#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess
import roslaunch

rosNodeName = 'markerMergeChecker'
last = rospy.Time(0)

def callBackMergedMarker(data):
	last = data.header.stamp.nsecs
	return

if __name__ == '__main__':	
	
	rospy.init_node(rosNodeName, anonymous=True)
	
	subMarker = rospy.Subscriber('horizon/ar_pose_estim', PoseWithCovarianceStamped, callBackMergedMarker) 
	
	rate = rospy.Rate(1.0)
	cnt = 0
	
	while not rospy.is_shutdown():
		#if last callback is more than a second in the past
		if ((rospy.Time.now() - last).to_sec() >= 1):
			#send SIGKILL, because the node has certainly hung up
			subprocess.call(["rosnode","kill","marius_frost_ar_rover_pose"])
			#wait for a bit
			rospy.sleep(1)
			print("Kill it")
			#restart node
			node = roslaunch.core.Node("zed_wrapper","marius_frost_rover_pose_ar")
			launch = roslaunch.scriptapi.ROSLaunch()
			launch.start()
			process = launch.launch(node)
			#give it a second
			rospy.sleep(1)
			print("And bring it back to life")
			#reset stuff
			last = rospy.Time.now()
		rate.sleep()
	
