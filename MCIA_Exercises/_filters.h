#pragma once
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <exception>

using namespace cv;

Mat lowpassKernel(size_t size);

Mat lowpass_filter(Mat &img, size_t size);

Mat highpass_filter(Mat &img);

Mat fractile_filter(Mat &img, size_t size, const float fraction);

Mat laplaceKernel9x9(void);

void fitContour(Mat &src, Mat &dst, Mat &contour, int thr = 125);

void nonMaximalSuppression(Mat &src, Mat &out, int dist, const int thr);
