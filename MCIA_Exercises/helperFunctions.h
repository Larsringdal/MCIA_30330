#pragma once
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <exception>

using namespace cv;

Mat homogenizeMatrix(Mat & src, int axis = 0);


Mat deHomogenizeMatrix(Mat & src, int axis = 0);