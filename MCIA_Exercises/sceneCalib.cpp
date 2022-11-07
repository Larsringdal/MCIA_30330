#include "sceneCalib.h"

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

double sceneReprojError(Mat & T, Mat & objPoints, Mat & imgPoints) {

	Mat objReproj;

	objReproj = T.mul(imgPoints);

	return norm(objReproj, objPoints, NORM_L2);
}

Mat calculateTransform(void) {
	int N_POINTS = 12;
	Mat objPoints = Mat(N_POINTS, 3, CV_32F, calibPointsObj);
	Mat imgPoints = Mat(N_POINTS, 2, CV_32F, calibPointsImg);
	std::cout << objPoints << std::endl;

	Mat RC = Mat::ones(N_POINTS, 3, CV_32F);
	hconcat(imgPoints, Mat::ones(N_POINTS, 1, CV_32F), RC);

	Mat RCX = Mat::ones(N_POINTS, 2, CV_32F);
	Mat RCY = Mat::ones(N_POINTS, 2, CV_32F);
	Mat RCZ = Mat::ones(N_POINTS, 2, CV_32F);


	float temp;
	for (int i = 0; i < N_POINTS; i++)
	{
		// gotta swap these as I wrote them down as (x,y) instead of (y,x)
		//temp = imgPoints.at<float>(i, 0);
		//imgPoints.at<float>(i, 0) = imgPoints.at<float>(i, 1);
		//imgPoints.at<float>(i, 1) = temp;

		RCX.at<float>(i, 0) = -imgPoints.at<float>(i, 0) * calibPointsObj[i][0];
		RCX.at<float>(i, 1) = -imgPoints.at<float>(i, 1) * calibPointsObj[i][0];

		RCY.at<float>(i, 0) = -imgPoints.at<float>(i, 0) * calibPointsObj[i][1];
		RCY.at<float>(i, 1) = -imgPoints.at<float>(i, 1) * calibPointsObj[i][1];

		RCZ.at<float>(i, 0) = -imgPoints.at<float>(i, 0) * calibPointsObj[i][2];
		RCZ.at<float>(i, 1) = -imgPoints.at<float>(i, 1) * calibPointsObj[i][2];
	}

	Mat A = Mat::zeros(N_POINTS *3, RC.cols*3 + 2, CV_32F);

	RC.copyTo(A(Range(0, RC.rows), Range(0, RC.cols)));
	RC.copyTo(A(Range(N_POINTS, N_POINTS * 2), Range(RC.cols, RC.cols * 2)));
	RC.copyTo(A(Range(N_POINTS *2, N_POINTS * 3), Range(RC.cols*2, RC.cols * 3)));
	RCX.copyTo(A(Range(0, N_POINTS), Range(RC.cols * 3, A.cols)));
	RCY.copyTo(A(Range(N_POINTS, 2*N_POINTS), Range(RC.cols * 3, A.cols)));
	RCZ.copyTo(A(Range(2*N_POINTS, 3*N_POINTS), Range(RC.cols * 3, A.cols)));

	Mat obj_flat = objPoints.reshape(1, 1).t();

	Mat X;
	solve(A.t()*A, A.t()*objPoints.reshape(1, 1).t(), X, DECOMP_EIG);

	Mat T;
	vconcat(X, Mat::ones(1,1,CV_32F), T);

	return T.reshape(1,4);
}

