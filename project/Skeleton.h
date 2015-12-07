#ifndef SKELETON_H
#define SKELETON_H
#include <iostream>
#include<vector>
#include<map>
#include<XnCppWrapper.h>
#include"cv.h"
#include"highgui.h"
#include"cxcore.h"

using namespace std;
using namespace cv;
class skeletondrawer
{
public:
	skeletondrawer(xn::UserGenerator& s,xn::DepthGenerator& depthgenerator);
	void updateSkeleton(XnUserID userid);
	void drawSkeleton(XnUserID userid,Mat& img);
private:
	void drawbone(XnSkeletonJoint joint1, XnSkeletonJoint joint2,Mat& img);
	void drawjoints(Mat& img);
private:
	xn::UserGenerator mUser;
	xn::SkeletonCapability mSC;
	xn::DepthGenerator m_depthgenerator;
	vector<pair<XnSkeletonJoint, XnSkeletonJoint> > bones;
	map<XnSkeletonJoint, XnSkeletonJointPosition> joints;
	map<XnSkeletonJoint, XnPoint3D> m_jointspos;
	XnSkeletonJoint ajoints[15];
};

#endif
