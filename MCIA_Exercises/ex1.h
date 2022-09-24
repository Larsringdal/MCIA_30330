#pragma once
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <stdexcept>

using namespace cv;

void flipChannel(Mat &img, uint channel);

Mat rgb2gray(Mat &colImg);

Mat showHist(Mat &img, int channel);

Mat binaryThreshold(Mat &img, int & thr);

void centerOfMass(Mat &imgThr, uint com[2]);

Mat drawCenterOfMass(Mat &imgThr, uint com[2]);

float centralMoments(Mat & img, uint com[2]);

int centralMoments_2(Mat & img, uint com[2], float out[3]);

float HuMoment(Mat & img);