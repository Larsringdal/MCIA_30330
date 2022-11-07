#include "helperFunctions.h"

Mat homogenizeMatrix(Mat & src, int axis) {
	// Adds ones along specified axis. axis = 0 for a column of 1's, axis = 1 for a row of 1's
	Mat out;
	if (axis == 0)
	{
		hconcat(src, Mat::ones(src.rows, 1, src.type()), out);
	}
	else if (axis == 1) {
		vconcat(src, Mat::ones(1, src.cols, src.type()), out);
	}
	else {
		std::invalid_argument("Axis must be either 0 or 1\n");
	}
	return out;
}

Mat deHomogenizeMatrix(Mat & src, int axis) {
	// divides row/column wise with last row/column depending on axis.
	// axis = 0 for scale in last column, axis = 1 for scale in last row
	Mat hom = (axis == 0) ? src : src.t(); // transpose depending on given axis
	Mat out = Mat::zeros(hom.rows, hom.cols - 1, hom.type());


	for (int i = 0; i < hom.rows; i++)
	{
		float scale = hom.at<float>(i,hom.cols-1);
		for (int j = 0; j < hom.cols - 1; j++) {
			out.at<float>(i, j) = hom.at<float>(i,j) / scale;
		}
	}
	
	if (axis == 0)
		return out;
	else
		return out.t();
}