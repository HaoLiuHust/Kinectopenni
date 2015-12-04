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

const int innerBandThreshold = 4;
const int outerBandThreshold = 4;
const int FrameCount = 2;

void filterDepthMeta(Mat& src, Mat& dst);
void fiterDepthAverageMeta(Mat& src, Mat& dst);
#endif