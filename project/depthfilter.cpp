#define _CRT_SECURE_NO_WARNINGS
#include<iostream>
#include<deque>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"

using namespace cv;
using namespace std;

const int innerBandThreshold = 2;
const int outerBandThreshold = 5;
const int FrameCount = 4;
deque<Mat> averageQueue;

void filterDepth(Mat& src, Mat& dst)
{
	int width = src.cols;
	int height = src.rows;

	int widthBound = width - 1;
	int heightBound = height - 1;
	dst = Mat::zeros(src.size(), CV_16UC1);
	int **filterCollection = new int *[24];
	for (int i = 0; i < 24;++i)
	{
		filterCollection[i] = new int[2];
	}
	//memset(filterCollection, 0, sizeof(int) * 2 * 24);
	uchar *srcdata = src.data;
	uchar *dstdata = dst.data;
	int widthstep = src.step[1];
	for (int rowindex = 0; rowindex < height;++rowindex)
	{
		//uchar *srcdata = src.ptr<uchar>(rowindex);
		for (int colindex = 0; colindex < width;++colindex)
		{
			int depthindex = colindex + rowindex*widthstep;
			if (srcdata[depthindex] == 0)
			{
				memset(filterCollection, 0, sizeof(int) * 2 * 24);
				int innerBandCount = 0;
				int outerBandCount = 0;

				for (int yi = -2; yi < 3; ++yi)
				{
					for (int xi = -2; xi < 3; ++xi)
					{
						if (xi != 0 || yi != 0)
						{
							int xSearch = colindex + xi;
							int ySearch = rowindex + yi;

							if (xSearch >= 0 && xSearch <= widthBound&&ySearch >= 0 && ySearch <= heightBound)
							{
								int searchindex = xSearch + ySearch*widthstep;
								//uchar *rowdata = src.ptr<uchar>(ySearch);
								if (srcdata[searchindex] != 0)
								{
									for (int i = 0; i < 24; ++i)
									{
										if (filterCollection[i][0] == srcdata[searchindex])
										{
											++filterCollection[i][1];
											break;
										}
										else if (filterCollection[i][0] == 0)
										{
											filterCollection[i][0] = srcdata[searchindex];
											++filterCollection[i][1];
											break;
										}
									}

									if (yi != 2 && yi != -2 && xi != 2 && xi != -2)
									{
										++innerBandCount;
									}
									else
										++outerBandCount;
								}
							}
						}
					}
				}

				//filter
				if (innerBandCount >= innerBandThreshold || outerBandCount >= outerBandThreshold)
				{
					int frequency = 0;
					int depth = 0;
					for (int i = 0; i < 24; ++i)
					{
						if (filterCollection[i][0] == 0)
						{
							break;
						}
						if (filterCollection[i][1]>frequency)
						{
							depth = filterCollection[i][0];
							frequency = filterCollection[i][1];
						}
					}
					dstdata[depthindex] = depth;
				}
			}
			else
				dstdata[depthindex] = srcdata[depthindex];
		}
	}

	for (int i = 0; i < 24;++i)
	{
		delete[] filterCollection[i];
	}
	delete[] filterCollection;
}

void fiterDepthAverage(Mat& src, Mat& dst)
{
	averageQueue.push_back(src);
	while (averageQueue.size()>FrameCount)
	{
		averageQueue.pop_front();
	}

	int *sumDepth = new int[src.rows*src.cols];
	int *averDepth = new int[src.rows*src.cols];
	memset(sumDepth, 0, sizeof(int)*src.rows*src.cols);
	int Denominator = 0;
	int Count = 1;

	uchar *srcdata = src.data;
	uchar *dstdata = dst.data;
	int widthstep = src.step[1];
	int width = src.cols;
	int height = src.rows;
	for (int i = 0; i < FrameCount;++i)
	{
		uchar *temp = averageQueue[i].data;
		for (int rowindex = 0; rowindex < width;++rowindex)
		{
			for (int colindex = 0; colindex < height;++colindex)
			{
				int depthindex = colindex + rowindex*widthstep;
				sumDepth[depthindex] += (temp[depthindex] * Count);
			}		
		}
		Denominator += Count;
		++Count;
	}

	for (int rowindex = 0; rowindex < width; ++rowindex)
	{
		for (int colindex = 0; colindex < height; ++colindex)
		{
			int depthindex = colindex + rowindex*widthstep;
			averDepth[depthindex]=round(sumDepth[depthindex]/Denominator);
			dstdata[depthindex] = (uchar)averDepth[depthindex];
		}
	}

	delete[] sumDepth;
	delete[] averDepth;
}