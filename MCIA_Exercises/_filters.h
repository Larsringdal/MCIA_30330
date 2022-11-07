#pragma once
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <exception>
#include <vector>
#include <utility>

using namespace cv;

Mat lowpassKernel(size_t size);

Mat lowpass_filter(Mat &img, size_t size);

Mat highpass_filter(Mat &img);

Mat fractile_filter(Mat &img, size_t size, const float fraction);

Mat laplaceKernel9x9(void);

void fitContour(Mat &src, Mat &dst, Mat &contour, int thr = 125);

void nonMaximalSuppression(Mat &src, Mat &out, int dist, const int thr);

std::vector<std::pair<Point2i, Point2i>> correspondence(Mat &im1, Mat &im2, Point2i kernel_size, int n_correspondences);
