#define _CRT_SECURE_NO_WARNINGS
#include "depthfilterMeta.h"
deque<Mat> averageQueueMeta;

void filterDepthMeta(Mat& src, Mat& dst)
{
	int width = src.cols;
	int height = src.rows;

	int widthBound = width - 1;
	int heightBound = height - 1;

	dst = Mat::zeros(src.size(), CV_16UC1);
	ushort **filterCollection = new ushort *[24];
	for (int i = 0; i < 24; ++i)
	{
		filterCollection[i] = new ushort[2];
	}
	//memset(filterCollection, 0, sizeof(int) * 2 * 24);
	ushort *srcdata = src.ptr<ushort>(0);
	//uchar *srcdata = src.data;
	//uchar *dstdata = dst.data;
	ushort *dstdata = dst.ptr<ushort>(0);
	int widthstep = src.step[0]/src.elemSize();
	for (int rowindex = 0; rowindex < height; ++rowindex)
	{
		//uchar *srcdata = src.ptr<uchar>(rowindex);
		for (int colindex = 0; colindex < width; ++colindex)
		{
			int depthindex = colindex + rowindex*widthstep;
			if (srcdata[depthindex] == 0)
			{
				for (int i = 0; i < 24; ++i)
				{
					memset(filterCollection[i], 0, sizeof(ushort) * 2);
				}
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
					ushort depth = 0;
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
					dstdata[depthindex] = depth ;
				}

			}
			else
				dstdata[depthindex] = srcdata[depthindex];
		}
	}

	for (int i = 0; i < 24; ++i)
	{
		delete[] filterCollection[i];
	}
	delete[] filterCollection;
}

void fiterDepthAverageMeta(Mat& src, Mat& dst)
{
	averageQueueMeta.push_back(src);
	while (averageQueueMeta.size() > FrameCount)
	{
		averageQueueMeta.pop_front();
	}

	UINT *sumDepth = new UINT[src.rows*src.cols];
	UINT *averDepth = new UINT[src.rows*src.cols];
	memset(sumDepth, 0, sizeof(UINT)*src.rows*src.cols);
	UINT Denominator = 0;
	UINT Count = 1;

	//uchar *srcdata = src.data;
	//uchar *dstdata = dst.data;
	ushort *dstdata = dst.ptr<ushort>(0);
	int widthstep = src.step[0] / src.elemSize();
	int width = src.cols;
	int height = src.rows;
	for (int i = 0; i < averageQueueMeta.size(); ++i)
	{
		ushort *temp = averageQueueMeta[i].ptr<ushort>(0);
		//uchar *temp = averageQueue[i].data;
		for (int rowindex = 0; rowindex < height; ++rowindex)
		{
			for (int colindex = 0; colindex < width; ++colindex)
			{
				int depthindex = colindex + rowindex*widthstep;
				sumDepth[depthindex] += (temp[depthindex] * Count);
			}
		}
		Denominator += Count;
		++Count;
	}

	for (int rowindex = 0; rowindex < height; ++rowindex)
	{
		for (int colindex = 0; colindex < width; ++colindex)
		{
			int depthindex = colindex + rowindex*widthstep;
			averDepth[depthindex] = round(sumDepth[depthindex] / Denominator);
			dstdata[depthindex] = (ushort)averDepth[depthindex];
		}
	}

	//averageQueueMeta.pop_back();
	//averageQueueMeta.push_back(dst);
	delete[] sumDepth;
	delete[] averDepth;
}

void filterKalman(Mat& src, Mat& dst,Kalmanpar& params)
{
	int width = src.cols;
	int height = src.rows;
	int widthstep = src.step[0] / src.elemSize();

	ushort *srcdata = src.ptr<ushort>(0);
	ushort *dstdata = dst.ptr<ushort>(0);

	for (int rowindex = 0; rowindex < height;++rowindex)
	{
		for (int colindex = 0; colindex < width;++colindex)
		{
			int depthindex = colindex + rowindex*widthstep;
			ushort preddepth = params.xhat[depthindex];
			double Pminus = params.P[depthindex] + params.Q;
			double K = Pminus / (Pminus + params.R);
			ushort finaldepth =static_cast<ushort>(preddepth + K*(srcdata[depthindex] - preddepth));
			params.P[depthindex] = (1 - K)*Pminus;

			params.xhat[depthindex] = finaldepth;
			dstdata[depthindex] = finaldepth;
		}
	}
}