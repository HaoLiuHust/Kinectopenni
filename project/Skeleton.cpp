#define _CRT_SECURE_NO_WARNINGS
#include "Skeleton.h"

skeletondrawer::skeletondrawer(xn::UserGenerator& s,xn::DepthGenerator& depthgenerator):
	mUser(s),mSC(s.GetSkeletonCap()),m_depthgenerator(depthgenerator)
{
	//add bones
	bones.push_back(make_pair(XN_SKEL_HEAD, XN_SKEL_NECK));
	bones.push_back(make_pair(XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER));
	bones.push_back(make_pair(XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW));
	bones.push_back(make_pair(XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND));
	bones.push_back(make_pair(XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER));
	bones.push_back(make_pair(XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW));
	bones.push_back(make_pair(XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND));
	bones.push_back(make_pair(XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO));
	bones.push_back(make_pair(XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO));
	bones.push_back(make_pair(XN_SKEL_TORSO, XN_SKEL_LEFT_HIP));
	bones.push_back(make_pair(XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP));
	bones.push_back(make_pair(XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE));
	bones.push_back(make_pair(XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT));
	bones.push_back(make_pair(XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE));
	bones.push_back(make_pair(XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT));	

	ajoints[0] = XN_SKEL_HEAD;
	ajoints[1] = XN_SKEL_NECK;
	ajoints[2] = XN_SKEL_LEFT_SHOULDER;
	ajoints[3] = XN_SKEL_LEFT_ELBOW;
	ajoints[4] = XN_SKEL_LEFT_HAND;
	ajoints[5] = XN_SKEL_RIGHT_SHOULDER;
	ajoints[6] = XN_SKEL_RIGHT_ELBOW;
	ajoints[7] = XN_SKEL_RIGHT_HAND;
	ajoints[8] = XN_SKEL_TORSO;
	ajoints[9] = XN_SKEL_LEFT_HIP;
	ajoints[10] = XN_SKEL_LEFT_KNEE;
	ajoints[11] = XN_SKEL_LEFT_FOOT;
	ajoints[12] = XN_SKEL_RIGHT_HIP;
	ajoints[13] = XN_SKEL_RIGHT_KNEE;
	ajoints[14] = XN_SKEL_RIGHT_FOOT;

}

void skeletondrawer::updateSkeleton(XnUserID userid)
{
	joints.clear();
	m_jointspos.clear();
	if (mSC.IsTracking(userid))
	{
		for (int i = 0; i < 15;++i)
		{
			XnSkeletonJointPosition pos;
			mSC.GetSkeletonJointPosition(userid, this->ajoints[i], pos);
			joints[ajoints[i]] = pos;
		}
	}
}

void skeletondrawer::drawbone(XnSkeletonJoint joint1, XnSkeletonJoint joint2,Mat& img)
{
	XnSkeletonJointPosition pos1 = joints[joint1];
	XnSkeletonJointPosition pos2 = joints[joint2];
	if (pos1.fConfidence<0.5||pos2.fConfidence<0.5)
	{
		return;
	}
	XnPoint3D pt[2];
	pt[0] = m_jointspos[joint1];
	pt[1] = m_jointspos[joint2];

	Point2d cpoint[2] = { {pt[0].X,pt[0].Y},{pt[1].X,pt[1].Y} };
	line(img, cpoint[0], cpoint[1], Scalar(0, 0, 255),1);
}

void skeletondrawer::drawjoints(Mat& img)
{
	for (int i = 0; i < 15;++i)
	{
		XnSkeletonJointPosition pos = joints[ajoints[i]];
		if (pos.fConfidence>=0.5)
		{
			XnPoint3D pt = pos.position;
			m_depthgenerator.ConvertRealWorldToProjective(1, &pt, &pt);
			m_jointspos[ajoints[i]] = pt;

			Point2d cpt = { pt.X,pt.Y };
			circle(img, cpt, 5, Scalar(0, 255, 0),-1);
		}
	}


}
void skeletondrawer::drawSkeleton(XnUserID userid,Mat& img)
{
	if (mSC.IsTracking(userid))
	{
		xn::SceneMetaData Smd;
		mUser.GetUserPixels(userid, Smd);
		const XnLabel* pLabels = Smd.Data();
		Mat mask1(img.size(), CV_16UC1, (void*)pLabels);
		Mat mask2;
		mask1.convertTo(mask2, CV_8U);
		img.setTo(Scalar(255, 0, 0), mask2);
		drawjoints(img);
		for (int i = 0; i < bones.size(); ++i)
		{
			drawbone(bones[i].first, bones[i].second, img);
		}
	}
	
}
