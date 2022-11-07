#pragma once
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <exception>

using namespace cv;

double sceneReprojError(Mat & T, Mat & objPoints, Mat & imgPoints);

Mat calculateTransform(void);

