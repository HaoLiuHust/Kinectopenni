#ifndef DEPTHFILTERMETA_H
#define DEPTHFILTERMETA_H
#include<iostream>
#include<deque>
#include <XnCppWrapper.h>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"

using namespace cv;
using namespace xn;
using namespace std;

const int innerBandThreshold = 2;
const int outerBandThreshold = 4;
const int FrameCount = 3;
struct Kalmanpar
{
	ushort *xhat;
	const double R;
	const double Q;
	double *P;
	Kalmanpar(int len) :R(0.01), Q(0.001)
	{
		xhat = new ushort[len];
		P = new double[len];
		memset(xhat, 0, len*sizeof(ushort));
		memset(P, 0, len*sizeof(double));
	}
	~Kalmanpar()
	{
		if (xhat != NULL)
		{
			delete[] xhat;
			xhat = NULL;
		}
		if (P != NULL)
		{
			delete[] P;
			P = NULL;
		}
	}
};
void filterDepthMeta(Mat& src, Mat& dst);
void fiterDepthAverageMeta(Mat& src, Mat& dst);
void filterKalman(Mat& src, Mat& dst, Kalmanpar& params);

#endif