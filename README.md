ISIR Custom tracker:
====================

Kinect pack that tracks humain body movements and publish transforms on /tf topic:
	/tf/head
	/tf/neck
	/tf/torso
	/tf/right_shoulder
	/tf/right_elbow
	/tf/right_hand
	/tf/left_shoulder
	/tf/left_elbow
	/tf/left_hand
	/tf/right_hip
	/tf/right_knee
	/tf/right_ankle
	/tf/left_hip
	/tf/left_knee
	/tf/left_ankle

Install:
---------
	copy the pack on your ros workspace
	delete build/CMakeCache.txt
	build the pack:
		rosmake tracker

Examples:
---------
	to run the pack :
		roslaunch openni_roslaunch openni_roslaunch
		rosrun tracker tracker

	to visualize the transforms on rviz:
		rqt
		change in :
			Global Frame >> Fixed Frame >> /camera_depth_optical_frame
			Add >> TF

	to check the /tf topic
		rostopic echo /tf

