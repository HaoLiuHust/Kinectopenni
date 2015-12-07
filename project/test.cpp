#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include<iostream>
#include<string>
#include<XnCppWrapper.h>
#include<time.h>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "kinect-motors.h"
#include "depthfilterMeta.h"
#include "Skeleton.h"
using namespace cv;
using namespace std;
using namespace xn;

//Skeleton Part
void XN_CALLBACK_TYPE NewUser(UserGenerator& generator, XnUserID user, void* pCookie)
{
	cout << "New user identified: " << user << endl;
	generator.GetSkeletonCap().RequestCalibration(user, true);
}

void XN_CALLBACK_TYPE CalibrationEnd(SkeletonCapability& skeleton, XnUserID user, XnCalibrationStatus eStatus, void* pCookie)
{
	cout << "Calibration complete for user " << user << ",";
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		cout << "Success" << endl;
		skeleton.StartTracking(user);
	}
	else
	{
		cout << "Failure" << endl;
		skeleton.RequestCalibration(user, true);
	}
}


void transformDepthMD(DepthMetaData& depthMD)
{
	DepthMap& depthMap = depthMD.WritableDepthMap();

	Mat depthMat(depthMD.FullYRes(), depthMD.FullXRes(), CV_16UC1, depthMD.WritableData());

	Mat filterdMat;
	filterDepthMeta(depthMat, filterdMat);
	//filterKalman(filterdMat, filterdMat, params);
	//fiterDepthAverageMeta(filterdMat, filterdMat);
	/*Mat depthshow;
	depthMat.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
	Mat depthfilterd(depthMat.size(),CV_8U);
	cv::bilateralFilter(depthshow, depthfilterd, 5, 1, 1);*/
	for (int j = 0; j < depthMat.rows; ++j)
	{
		ushort* data = filterdMat.ptr<ushort>(j);
		for (int i = 0; i < depthMat.cols; ++i)
		{
			depthMap(i, j) = data[i];
		}
	}
}

void transformDepthMD(DepthMetaData& depthMD,Kalmanpar& params)
{
	DepthMap& depthMap = depthMD.WritableDepthMap();

	Mat depthMat(depthMD.FullYRes(), depthMD.FullXRes(), CV_16UC1, depthMD.WritableData());

	Mat filterdMat;
	filterDepthMeta(depthMat, filterdMat);
	//filterKalman(filterdMat, filterdMat,params);
	fiterDepthAverageMeta(filterdMat, filterdMat);
	/*Mat depthshow;
	depthMat.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
	Mat depthfilterd(depthMat.size(),CV_8U);
	cv::bilateralFilter(depthshow, depthfilterd, 5, 1, 1);*/
	for (int j = 0; j < depthMat.rows; ++j)
	{
		ushort* data = filterdMat.ptr<ushort>(j);
		for (int i = 0; i < depthMat.cols;++i)
		{
			depthMap(i, j) = data[i];
		}
	}
}

//处理深度图
void modifyrecordfile(const string& input, const string& output)
{
	XnStatus nRetVal = XN_STATUS_OK;
	xn::Context m_Context;
	m_Context.Init();

	xn::Player player;
	m_Context.OpenFileRecording(input.c_str(),player);

	xn::DepthGenerator depthGenerator;
	m_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);

	xn::MockDepthGenerator mockDepth;
	mockDepth.CreateBasedOn(depthGenerator);

	xn::Recorder recorder;
	recorder.Create(m_Context);
	recorder.SetDestination(XN_RECORD_MEDIUM_FILE, output.c_str());
	recorder.AddNodeToRecording(mockDepth);

	player.SetRepeat(false);
	XnUInt32 nFrames = 0;
	player.GetNumFrames(depthGenerator.GetName(), nFrames);

	m_Context.StartGeneratingAll();
	DepthMetaData depthData;

	XnMapOutputMode Mode;
	depthGenerator.GetMapOutputMode(Mode);
	Mat depthimg(Mode.nYRes, Mode.nXRes, CV_16UC1);
	Mat depthshow;

	while (depthGenerator.WaitAndUpdateData()!=XN_STATUS_EOF)
	{
		depthGenerator.GetMetaData(depthData);
		depthData.MakeDataWritable();
		memcpy(depthimg.data, (void*)(depthData.Data()), depthData.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		imshow("depth", depthshow);
		transformDepthMD(depthData);
		mockDepth.SetData(depthData);
		memcpy(depthimg.data, (void*)(depthData.Data()), depthData.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		imshow("depthmodify", depthshow);
		recorder.Record();
		cout << "Record frameID: " << depthData.FrameID() << endl;
		if (cv::waitKey(1)=='q')
		{
			break;
		}
	}
	mockDepth.Release();
	depthGenerator.Release();
	m_Context.StopGeneratingAll();
	recorder.Release();
	player.Release();
	m_Context.Release();
}
void checkerror(XnStatus eResult, const string& sStatus)
{
	if (eResult!=XN_STATUS_OK)
	{
		cerr<<sStatus << "Error:" << xnGetStatusString(eResult) << endl;
	}
}

void Record(const string& filename,xn::Recorder& mRecorder,xn::Context& m_Context,xn::Generator& m_generator)
{	
	mRecorder.Create(m_Context);
	mRecorder.SetDestination(XN_RECORD_MEDIUM_FILE, filename.c_str());
	mRecorder.AddNodeToRecording(m_generator);	
}

void stopRecord(xn::Recorder& mRecorder)
{
	if (mRecorder.GetHandle()!=NULL)
		mRecorder.Release();
}

void playBack(const string& filename)
{
	XnStatus eResult = XN_STATUS_OK;

	//INITIAL CONTEXT
	xn::Context mContext;
	xn::Player player;
	eResult = mContext.Init();
	checkerror(eResult, "initial error");
	mContext.OpenFileRecording(filename.c_str(),player);
	
	//set map mode
	XnMapOutputMode mapMode;
	mapMode.nFPS = 30;
	mapMode.nXRes = 640;
	mapMode.nYRes = 480;

	//create generator
	xn::DepthGenerator depthGenerator;
	eResult=mContext.FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);
	//eResult = depthGenerator.Create(mContext);
	checkerror(eResult, "create depthgenerator");
	depthGenerator.SetMapOutputMode(mapMode);

	player.SetRepeat(false);
	//start generate data
	
	xn::MockDepthGenerator mockDepth;
	mockDepth.CreateBasedOn(depthGenerator, "mock-depth");

	//Skeleton Part
	Query xQuery;
	xQuery.AddNeededNode("mock-depth");
	UserGenerator xUser;
	xUser.Create(mContext, &xQuery);
	XnCallbackHandle hUserCB;
	xUser.RegisterUserCallbacks(NewUser, NULL, NULL, hUserCB);
	SkeletonCapability mSc = xUser.GetSkeletonCap();
	mSc.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	XnCallbackHandle hCalibCB;
	mSc.RegisterToCalibrationComplete(CalibrationEnd, &xUser, hCalibCB);
	mContext.StartGeneratingAll();
	
	DepthMetaData depthData;

	Mat depthimg(mapMode.nYRes, mapMode.nXRes, CV_16UC1);
	Mat depthshow;

	int imgsize = depthimg.rows*depthimg.step[0] / depthimg.elemSize();
	Kalmanpar params(imgsize);
	skeletondrawer bodydrawer(xUser, depthGenerator);

	//VideoWriter vwriter1("original.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25.0, Size(640,480),false);
	VideoWriter vwriter2("MVA2.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25.0, Size(640,480),false);
	
	while (true)
	{
		DWORD t1, t2;
		eResult=mContext.WaitOneUpdateAll(depthGenerator);
		if (eResult!=XN_STATUS_OK)
		{
			break;
		}
		depthGenerator.GetMetaData(depthData);
		depthData.MakeDataWritable();
		memcpy(depthimg.data, (void*)(depthData.Data()), depthData.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		//vwriter1 << depthshow;
		imshow("depth", depthshow);

		transformDepthMD(depthData, params);
		mockDepth.SetData(depthData);
		memcpy(depthimg.data, (void*)(depthData.Data()), depthData.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		vwriter2 << depthshow;
		cv::cvtColor(depthshow, depthshow, CV_GRAY2BGR);
	
		XnUInt16 nUsers = xUser.GetNumberOfUsers();
		cout << nUsers << endl;
		if (nUsers > 0)
		{
			XnUserID* aUsers = new XnUserID[nUsers];
			xUser.GetUsers(aUsers, nUsers);

			for (int i = 0; i < nUsers; ++i)
			{
				bodydrawer.updateSkeleton(aUsers[i]);
				bodydrawer.drawSkeleton(aUsers[i], depthshow);
			}

			delete[] aUsers;
		}
		
		imshow("filterd", depthshow);
		//cout << 1000.0 / (t2 - t1) << endl;
		if (cv::waitKey(1) == 'q')
		{
			break;
		}
	}
	//vwriter1.release();
	vwriter2.release();
	mockDepth.Release();
	depthGenerator.Release();
	mContext.StopGeneratingAll();
	mContext.Release();
	
}

void mockrealtime()
{
	XnStatus eResult = XN_STATUS_OK;
	xn::Context m_Context;
	m_Context.Init();
	//set map mode
	XnMapOutputMode mapMode;
	mapMode.nFPS = 30;
	mapMode.nXRes = 640;
	mapMode.nYRes = 480;

	
	//create generator
	xn::DepthGenerator depthGenerator;
	eResult = depthGenerator.Create(m_Context);
	checkerror(eResult, "create depthgenerator");
	depthGenerator.SetMapOutputMode(mapMode);
	int maxdepth = depthGenerator.GetDeviceMaxDepth();
	xn::MockDepthGenerator mockDepth;
	mockDepth.CreateBasedOn(depthGenerator,"depth-mock");

	m_Context.StartGeneratingAll();
	DepthMetaData depthData;

	Mat depthimg(mapMode.nYRes, mapMode.nXRes, CV_16UC1);
	Mat depthshow;

	int imgsize = depthimg.rows*depthimg.step[0] / depthimg.elemSize();
	Kalmanpar params(imgsize);

	while (true)
	{
		DWORD t1, t2;
		t1 = GetTickCount();
		m_Context.WaitOneUpdateAll(depthGenerator);
		depthGenerator.GetMetaData(depthData);
		depthData.MakeDataWritable();
		memcpy(depthimg.data, (void*)(depthData.Data()), depthData.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		imshow("depth", depthshow);

		transformDepthMD(depthData,params);
		mockDepth.SetData(depthData);
		memcpy(depthimg.data, (void*)(depthData.Data()), depthData.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		imshow("depthmodify", depthshow);
		t2 = GetTickCount();
		cout << 1000.0/(t2-t1)<< endl;
		if (cv::waitKey(1) == 'q')
		{
			break;
		}
	}
	mockDepth.Release();
	depthGenerator.Release();
	m_Context.StopGeneratingAll();
	m_Context.Release();
}

void displaykinect()
{
	KinectMotors motors;
	if (motors.Open())
	{
		motors.Move(15);
		pause_();
	}

	XnStatus eResult = XN_STATUS_OK;

	//INITIAL CONTEXT
	xn::Context mContext;
	eResult = mContext.Init();
	checkerror(eResult, "initial error");
	
	//set map mode
	XnMapOutputMode mapMode;
	mapMode.nFPS = 30;
	mapMode.nXRes = 640;
	mapMode.nYRes = 480;

	//create generator
	xn::DepthGenerator depthGenerator;
	eResult = depthGenerator.Create(mContext);
	checkerror(eResult, "create depthgenerator");
	depthGenerator.SetMapOutputMode(mapMode);


	UserGenerator xUser;
	xUser.Create(mContext);
	XnCallbackHandle hUserCB;
	xUser.RegisterUserCallbacks(NewUser, NULL, NULL, hUserCB);
	SkeletonCapability mSc = xUser.GetSkeletonCap();
	mSc.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	XnCallbackHandle hCalibCB;
	mSc.RegisterToCalibrationComplete(CalibrationEnd, &xUser, hCalibCB);

	//start generate data
	mContext.StartGeneratingAll();
	Mat depthimg(mapMode.nYRes, mapMode.nXRes, CV_16UC1);
	Mat depthshow;
	xn::DepthMetaData depthdata;

	xn::Recorder mRecorder;
	string recordname = "test2.oni";
	bool playflag = true;
	bool isrecording = false;

	skeletondrawer bodydrawer(xUser, depthGenerator);

	while (playflag)
	{
		DWORD t1, t2;
		t1 = GetTickCount();
		mContext.WaitAndUpdateAll();
		
		depthGenerator.GetMetaData(depthdata);
		memcpy(depthimg.data, (void*)(depthdata.Data()), depthdata.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		cv::cvtColor(depthshow, depthshow, CV_GRAY2BGR);
		t2 = GetTickCount();

		XnUInt16 nUsers = xUser.GetNumberOfUsers();
		cout << nUsers << endl;
		if (nUsers > 0)
		{
			XnUserID* aUsers = new XnUserID[nUsers];
			xUser.GetUsers(aUsers, nUsers);

			for (int i = 0; i < nUsers; ++i)
			{
				bodydrawer.updateSkeleton(aUsers[i]);
				bodydrawer.drawSkeleton(aUsers[i],depthshow);
			}
			
			delete[] aUsers;
		}
		imshow("depth", depthshow);
		//cout << 1000.0 / (t2 - t1) << endl;
		switch (cv::waitKey(1))
		{
		case 'q':playflag = false; break;
		case 's':if (!isrecording)
		{
			isrecording = !isrecording;
			Record(recordname, mRecorder, mContext, depthGenerator);
		}
				 else
					 stopRecord(mRecorder);
			break;
		default:
			break;
		}
		//cout << depthdata.FrameID() << endl;
	}

	mRecorder.Release();
	depthGenerator.Release();
	mContext.StopGeneratingAll();
	mContext.Release();
}



void mockrealtimeskeleton()
{
	XnStatus eResult = XN_STATUS_OK;
	xn::Context m_Context;
	m_Context.Init();
	//set map mode
	XnMapOutputMode mapMode;
	mapMode.nFPS = 30;
	mapMode.nXRes = 640;
	mapMode.nYRes = 480;

	//create generator
	xn::DepthGenerator depthGenerator;
	eResult = depthGenerator.Create(m_Context);
	checkerror(eResult, "create depthgenerator");
	depthGenerator.SetMapOutputMode(mapMode);
	int maxdepth = depthGenerator.GetDeviceMaxDepth();

	xn::MockDepthGenerator mockDepth;
	mockDepth.CreateBasedOn(depthGenerator, "mock-depth");

	//Skeleton Part
	Query xQuery;
	xQuery.AddNeededNode("mock-depth");
	UserGenerator xUser;
	xUser.Create(m_Context, &xQuery);
	XnCallbackHandle hUserCB;
	xUser.RegisterUserCallbacks(NewUser, NULL, NULL, hUserCB);
	SkeletonCapability mSc = xUser.GetSkeletonCap();
	mSc.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	XnCallbackHandle hCalibCB;
	mSc.RegisterToCalibrationComplete(CalibrationEnd, &xUser, hCalibCB);
	m_Context.StartGeneratingAll();
	DepthMetaData depthData;

	Mat depthimg(mapMode.nYRes, mapMode.nXRes, CV_16UC1);
	Mat depthshow;

	int imgsize = depthimg.rows*depthimg.step[0] / depthimg.elemSize();
	Kalmanpar params(imgsize);
	skeletondrawer bodydrawer(xUser, depthGenerator);
	while (true)
	{
		DWORD t1, t2;
		m_Context.WaitOneUpdateAll(depthGenerator);
		t1 = GetTickCount();
		depthGenerator.GetMetaData(depthData);
		depthData.MakeDataWritable();
		memcpy(depthimg.data, (void*)(depthData.Data()), depthData.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		imshow("depth", depthshow);

		transformDepthMD(depthData, params);
		mockDepth.SetData(depthData);
		memcpy(depthimg.data, (void*)(depthData.Data()), depthData.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		cv::cvtColor(depthshow, depthshow, CV_GRAY2BGR);
		t2 = GetTickCount();

		XnUInt16 nUsers = xUser.GetNumberOfUsers();
		cout << nUsers << endl;
		if (nUsers > 0)
		{
			XnUserID* aUsers = new XnUserID[nUsers];
			xUser.GetUsers(aUsers, nUsers);

			for (int i = 0; i < nUsers; ++i)
			{
				bodydrawer.updateSkeleton(aUsers[i]);
				bodydrawer.drawSkeleton(aUsers[i],depthshow);
			}

			delete[] aUsers;
		}
		imshow("modifyed", depthshow);

		//cout << 1000.0 / (t2 - t1) << endl;
		if (cv::waitKey(1) == 'q')
		{
			break;
		}
	}
	mockDepth.Release();
	depthGenerator.Release();
	m_Context.StopGeneratingAll();
	m_Context.Release();
}




int main()
{
	displaykinect();
	//mockrealtime();
	//modifyrecordfile("../project/test.oni","modify.oni");
	//playBack("../project/test2.oni");
	//mockrealtimeskeleton();
	return 0;

}
