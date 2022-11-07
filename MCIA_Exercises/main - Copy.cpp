#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "camCalib.h"
#include "sceneCalib.h"
#include "helperFunctions.h"

using namespace cv;


void cameraCalibration() {
	std::string photo_folder = "C:/CODE/MCIA_PROJECT/MCIA_PROJECT_IMAGES/checkerboard_session1_resized/";
	Size2i boardSize(6, 9);

	//Mat K = readCameraMatrix("camera1.yml", "cam1");

	//std::cout << "Camera Matrix:\n" << K << std::endl;
	Mat cameraMatrix, distCoeffs;

	double reprojErr = runCameraCalibration(photo_folder, boardSize, cameraMatrix, distCoeffs);

	std::cout << "reprojection error: " << reprojErr << std::endl;

	std::cout << "Camera Matrix = " << cameraMatrix << std::endl;

	saveCameraMatrix("camera1.yml", cameraMatrix, "cam1");
}


void sceneCalibration() {
	//Mat cameraMatrix = readCameraMatrix("camera1.yml", "cam1");
	

	Mat T = calculateTransform();
	std::cout << T;

	float calibPointsObj[28][3] = {
	{0.12, 0.15, 0.02}, {0.12, 0.12, 0.02}, {0.12, 0.09, 0.02}, {0.12, 0.06, 0.02},
	{0.12, 0.15, 0.04}, {0.12, 0.12, 0.04}, {0.12, 0.09, 0.04}, {0.12, 0.06, 0.04},
	{0.12, 0.15, 0.06}, {0.12, 0.12, 0.06}, {0.12, 0.09, 0.06}, {0.12, 0.06, 0.06},
	{0.12, 0.15, 0.08}, {0.12, 0.12, 0.08}, {0.12, 0.09, 0.08}, {0.12, 0.06, 0.08},
	{0.12, 0.15, 0.10}, {0.12, 0.12, 0.10}, {0.12, 0.09, 0.10}, {0.12, 0.06, 0.10},
	{0.12, 0.15, 0.12}, {0.12, 0.12, 0.12}, {0.12, 0.09, 0.12}, {0.12, 0.06, 0.12},
	{0.12, 0.15, 0.14}, {0.12, 0.12, 0.14}, {0.12, 0.09, 0.14}, {0.12, 0.06, 0.14}
	};

	float calibPointsImg[28][2] = {
		{200, 50}, {200, 114}, {202, 178}, {203, 241},
		{192, 51}, {192, 117}, {193, 181}, {194, 245},
		{183, 52}, {184, 119}, {186, 183}, {189, 249},
		{176, 53}, {177, 121}, {178, 189}, {181, 253},
		{169, 55}, {170, 123}, {172, 191}, {173, 258},
		{161, 56}, {162, 128}, {164, 196}, {166, 263},
		{154, 57}, {155, 129}, {159, 199}, {159, 268}
	};

	Mat points(28, 2, CV_32F, calibPointsImg);
	Mat hPoints = homogenizeMatrix(points, 0);

	std::cout << "dims of points : " << hPoints.rows << ", " << hPoints.cols << std::endl;
	std::cout << "dims of T : " << T.rows << ", " << T.cols << std::endl;
	
	Mat outHom = T * hPoints.t();

	Mat out = deHomogenizeMatrix(outHom, 1);

	std::cout << "\n projected\n" << out.t() << std::endl;
	std::cout << "\n actual\n" << Mat(28, 3, CV_32F, calibPointsObj) << std::endl;
	
	std::cout << "L2 NORM : " << norm(out.t(), Mat(28,3,CV_32F, calibPointsObj), NORM_L2) << std::endl;
	// STEP 1: Get camera pose from calibration photos

}

int main(void) {

	sceneCalibration();

	return 0;
}