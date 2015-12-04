#define _CRT_SECURE_NO_WARNINGS
#include<iostream>
#include<string>
#include<XnCppWrapper.h>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "kinect-motors.h"
#include "depthfilterMeta.h"
using namespace cv;
using namespace std;
using namespace xn;

void transformDepthMD(DepthMetaData& depthMD)
{
	DepthMap& depthMap = depthMD.WritableDepthMap();

	Mat depthMat(depthMD.FullYRes(), depthMD.FullXRes(), CV_16UC1, depthMD.WritableData());

	Mat filterdMat;
	filterDepthMeta(depthMat, filterdMat);
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
	mContext.StartGeneratingAll();
	Mat depthimg(mapMode.nYRes, mapMode.nXRes, CV_16UC1);
	Mat depthshow;
	xn::DepthMetaData depthdata;

	xn::Recorder mRecorder;
	string recordname = "test.oni";
	bool playflag = true;
	bool isrecording = false;
	while (playflag&&(depthGenerator.WaitAndUpdateData()!=XN_STATUS_EOF))
	{
		mContext.WaitAndUpdateAll();

		depthGenerator.GetMetaData(depthdata);
		memcpy(depthimg.data, (void*)(depthdata.Data()), depthdata.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		imshow("depth", depthshow);

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

	}

	mRecorder.Release();
	mContext.StopGeneratingAll();
	mContext.Release();
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

	while (depthGenerator.WaitAndUpdateData() != XN_STATUS_EOF)
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
		cout << depthData.FrameID() << endl;
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
		motors.Move(0);
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

	//start generate data
	mContext.StartGeneratingAll();
	Mat depthimg(mapMode.nYRes, mapMode.nXRes, CV_16UC1);
	Mat depthshow;
	xn::DepthMetaData depthdata;

	xn::Recorder mRecorder;
	string recordname = "test.oni";
	bool playflag = true;
	bool isrecording = false;
	while (playflag)
	{
		mContext.WaitAndUpdateAll();

		depthGenerator.GetMetaData(depthdata);
		memcpy(depthimg.data, (void*)(depthdata.Data()), depthdata.DataSize());
		depthimg.convertTo(depthshow, CV_8U, 255 / 4096.0, 0);
		imshow("depth", depthshow);

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
		cout << depthdata.FrameID() << endl;
	}

	mRecorder.Release();
	depthGenerator.Release();
	mContext.StopGeneratingAll();
	mContext.Release();
}

// callback function of user generator: new user
void XN_CALLBACK_TYPE NewUser(xn::UserGenerator& generator,
	XnUserID user,
	void* pCookie)
{
	cout << "New user identified: " << user << endl;
	generator.GetPoseDetectionCap().StartPoseDetection("Psi", user);
}

// callback function of user generator: lost user
void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator,
	XnUserID user,
	void* pCookie)
{
	cout << "User " << user << " lost" << endl;
}

// callback function of skeleton: calibration start
void XN_CALLBACK_TYPE CalibrationStart(xn::SkeletonCapability& skeleton,
	XnUserID user,
	void* pCookie)
{
	cout << "Calibration start for user " << user << endl;
}

// callback function of skeleton: calibration end 
void XN_CALLBACK_TYPE CalibrationEnd(xn::SkeletonCapability& skeleton,
	XnUserID user,
	XnBool bSuccess,
	void* pCookie)
{
	cout << "Calibration complete for user " << user << ", ";
	if (bSuccess)
	{
		cout << "Success" << endl;
		skeleton.StartTracking(user);
	}
	else
	{
		cout << "Failure" << endl;
		((xn::UserGenerator*)pCookie)->GetPoseDetectionCap().StartPoseDetection("Psi", user);
	}
}

// callback function of pose detection: pose start
void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& poseDetection,
	const XnChar* strPose,
	XnUserID user,
	void* pCookie)
{
	cout << "Pose " << strPose << " detected for user " << user << endl;
	((xn::UserGenerator*)pCookie)->GetSkeletonCap().RequestCalibration(user, FALSE);
	poseDetection.StopPoseDetection(user);
}




int main()
{
	//displaykinect();
	mockrealtime();
	//modifyrecordfile("../project/test.oni","modify.oni");
	//playBack("../project/modify.oni");
	return 0;

}
