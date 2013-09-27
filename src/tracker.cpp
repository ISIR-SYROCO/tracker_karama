/*!
 * \file tracker.cpp
 * \brief kinect package that tracks human motion 
 * \author Copyright (c) 2008, Willow Garage, Inc. All rights reserved.
 * \version 0.2 modified by Karama SRITI
 * \date June 14th 2013
 */
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#define USR_NUMBER 1

using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

//----------------------------------------> commence la detection et demande la calibration pour un nouvel utilisateur 
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) 
{
	//----------------------------------------------------
	XnUserID my_nId = 1;// my_nId = 1 -> au regard du programme l'utilisateur aura toujours le id = 1
	ROS_INFO("New User %d changed to User %d",nId,my_nId);
	//----------------------------------------------------

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, my_nId);//
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(my_nId, TRUE);//
}
//----------------------------------------> affiche le message "Lost User n" sur le terminal  
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) 
{
	//----------------------------------------------------
	XnUserID my_nId = 1;
	//----------------------------------------------------

	ROS_INFO("Lost user %d changed to User %d",nId, my_nId);//
}
//----------------------------------------> affiche le message "calibration started for user n" 
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) 
{
	//----------------------------------------------------
	XnUserID my_nId = 1;
	//----------------------------------------------------

	ROS_INFO("Calibration started for user %d changet to User %d", nId, my_nId);//
}
//----------------------------------------> calibration  
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) 
{
	//----------------------------------------------------
	XnUserID my_nId = 1;
	//----------------------------------------------------

	if (bSuccess) 
	{
		ROS_INFO("Calibration complete, start tracking user %d changed to user %d",nId, my_nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(my_nId);
	}
	else 
	{
		ROS_INFO("Calibration failed for user %d changet to user %d",nId, my_nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, my_nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(my_nId, TRUE);
	}
}
//----------------------------------------> demande calibration ?
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) 
{
	//----------------------------------------------------
	XnUserID my_nId = 1;
	//----------------------------------------------------

	ROS_INFO("Pose %s detected for user %d changed to user %d", strPose, nId, my_nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(my_nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(my_nId, TRUE);
}
//----------------------------------------> publication des reperes lies au skelette
void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) 
{
	static tf::TransformBroadcaster br;

	XnSkeletonJointPosition joint_position;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
	double x = -joint_position.position.X / 1000.0;
	double y = joint_position.position.Y / 1000.0;
	double z = joint_position.position.Z / 1000.0;

	XnSkeletonJointOrientation joint_orientation;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

	XnFloat* m = joint_orientation.orientation.elements;
	KDL::Rotation rotation(m[0], m[1], m[2],
			       m[3], m[4], m[5],
			       m[6], m[7], m[8]);
	double qx, qy, qz, qw;
	rotation.GetQuaternion(qx, qy, qz, qw);

	char child_frame_no[128];
	snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, z));
	transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

	// #4994
	tf::Transform change_frame;
	change_frame.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion frame_rotation;
	frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
	change_frame.setRotation(frame_rotation);

	transform = change_frame * transform;

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}
//---------------------------------------------------------------------------------------------
void publishTransforms(const std::string& frame_id) 
{
    	XnUserID users[USR_NUMBER];		//USR_NUMBER est fixe a 1, car on ne concidere qu'un seule personne a la fois
    	XnUInt16 users_count = USR_NUMBER;
    	g_UserGenerator.GetUsers(users, users_count);

    	for (int i = 0; i < users_count; ++i) 
	{
        	XnUserID user = users[i];
        	if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            	continue;


        	publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
		publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
		publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

		publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
		publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
		publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

		publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
		publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
		publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

		publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
		publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
		publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

		publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
		publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
		publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
    	}
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{												\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));				\
		return nRetVal;										\
	}
//---------------------------------------------------------------------------------------------
int main(int argc, char **argv) 
{
	ros::init(argc, argv, "tracker");
	ros::NodeHandle nh;

	string configFilename = ros::package::getPath("tracker") + "/tracker.xml";
	XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
	CHECK_RC(nRetVal, "InitFromXml");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
		
	if (nRetVal != XN_STATUS_OK) 
	{
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) 
	{
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

	XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) 
	{
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) 
		{
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(30);

        
        ros::NodeHandle pnh("~");
        string frame_id("/camera_link"); //camera_depth_optical_frame "openni_depth_frame" a la base le nom du repere de base  de la kinect a ete modifie, mois je prefere le fixer a camera_link, et ainsi garder la meme nomination pour tous les programmes  camera_link
        pnh.getParam("camera_frame_id", frame_id);
                
	while (ros::ok()) 
	{
		g_Context.WaitAndUpdateAll();
		publishTransforms(frame_id);
		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
