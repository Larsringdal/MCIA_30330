#pragma once
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <exception>

using namespace cv;

std::vector<Mat> loadImagesFromFolder(std::string folderPath);

std::vector<std::vector<Point2f>> getCornerCoords(std::string folderPath, Size2i boardSize);

// returns reprojection error. boardsize is (rows, cols)
double runCameraCalibration(std::string folderPath, Size2i boardSize,
	Mat & cameraMatrix, Mat & distCoeffs);

double reprojectionError(std::vector<std::vector<Point2f>> & imgPoints, std::vector<std::vector<Point3f>> & objPoints, Mat & cameraMatrix, std::vector<Mat> & rvecs, std::vector <Mat> & tvecs, Mat & distCoeffs);

// Saves camera matrix to file
bool saveCameraMatrix(std::string fileName, Mat & cameraMatrix, std::string matName);

// read specified camera matrix from specified yml file
Mat readCameraMatrix(std::string fileName, std::string matName);
