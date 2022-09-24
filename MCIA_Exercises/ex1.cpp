#include "ex1.h"
#include <math.h>

void flipChannel(Mat &img, uint channel) {

	int nRows = img.rows;
	int nCols = img.cols;

	Vec3b * temp;
	uint8_t temp_val;

	for (int i = 0; i < nRows; i++) {
		for (int j = 0; j < nCols; j++) {
			temp = &img.at<Vec3b>(i, j);
			temp_val = temp->val[channel];
			temp->val[channel] += 255 - temp_val;
		}
	}
}

Mat rgb2gray(Mat &colImg) {


	int nCols = colImg.cols;
	int nRows = colImg.rows;

	Mat grayImg(nRows, nCols, IMREAD_GRAYSCALE);

	Vec3b * temp;
	float temp_val;

	for (int i = 0; i < nRows; i++) {
		for (int j = 0; j < nCols; j++) {
			temp = &colImg.at<Vec3b>(i, j);

			temp_val = (temp->val[0] + temp->val[1] + temp->val[2]) / 3.0f;

			grayImg.at<uchar>(i, j) = (uchar)temp_val;
		}
	}
	return grayImg;
}

Mat showHist(Mat &img, int channel) {
	// channel = -1 for grayscale

	int hist_size[] = { img.cols, img.rows };

	unsigned char value = 0; // index value for the histogram
	int histogram[256] = { 0 }; // histogram array - remember to set to zero

	int width = img.cols; // INIT USING IMAGE VALUES
	int height = img.rows; // INIT USING IMAGE VALUES
	int k = 256;

	while (k-- > 0)
		histogram[k] = 0; // reset histogram entry

	int max_vals = 0;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			value = (channel == -1) ? img.at<uchar>(i, j) : img.at<Vec3b>(i, j)[channel];
			histogram[value] += 1;
			if (histogram[value] > max_vals) max_vals = histogram[value];
		}
	}

	if (max_vals != 0) {
		Mat histImg(hist_size[1], hist_size[0], IMREAD_GRAYSCALE);

		for (int i = 0; i < 256; i++) {
			// normalize histogram value based on max number:
			int hist_norm = (int)((histogram[i] / (float)max_vals) * hist_size[1]);
			// normalize bins with respect to image size
			int bin_norm = (int)((i / 256.0) * hist_size[0]);

			line(histImg, Point(bin_norm, hist_size[1] - 1), Point(bin_norm, hist_size[1] - hist_norm - 1), 255,
				2, LINE_8);
		}
		return histImg;
	}
	else {
		throw std::invalid_argument("Stopped division by zero due to max_vals=0 (matrix is zero)\n");
	}
}

Mat binaryThreshold(Mat &img, int & thr) {

	Mat out(img.rows, img.cols, IMREAD_GRAYSCALE);

	uchar p;

	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			p = (img.at<uchar>(i, j) < (uchar)thr) ? 255 : 0;
			out.at<uchar>(i, j) = p;
		}
	}
	return out;
}

void centerOfMass(Mat &imgThr, uint com[2]) {
	//calcultes center of mass and paints as white cross.
	com[0] = 0;
	com[1] = 0;

	uint n_elements = 0;
	uchar p;

	for (int i = 0; i < imgThr.rows; i++) {
		for (int j = 0; j < imgThr.cols; j++) {

			p = imgThr.at<uchar>(i, j);

			if (p != 0) {
				com[0] += j;
				com[1] += i;
				n_elements++;
			}
		}
	}
	if (n_elements != 0) {
		com[0] /= n_elements;
		com[1] /= n_elements;
	}
}

Mat drawCenterOfMass(Mat &imgThr, uint com[2]) {
	Mat imgCom(imgThr.rows, imgThr.cols, IMREAD_COLOR);
	cvtColor(imgThr, imgCom, IMREAD_COLOR);
	line(imgCom, Point(com[0], com[1] - 5), Point(com[0], com[1] + 5), Scalar(0, 0, 255), 2, LINE_8);
	line(imgCom, Point(com[0] - 5, com[1]), Point(com[0] + 5, com[1]), Scalar(0, 0, 255), 2, LINE_8);

	return imgCom;
}

float centralMoments(Mat & img, uint com[2]) {

	float mu11 = 0, mu20 = 0, mu02 = 0;
	float diff_x, diff_y;
	float mu00 = 0;

	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			if (img.at<uchar>(i, j) != 0) {
				mu00++;

				diff_x = (j - com[0]);
				diff_y = (i - com[1]);

				mu11 += diff_x * diff_y;
				mu20 += pow(diff_x, 2);
				mu02 += pow(diff_y, 2);
			}
		}
	}
	if (mu00 != 0) {
		mu11 = mu11 / mu00;
		mu20 = mu20 / mu00;
		mu02 = mu02 / mu00;

		std::cout << mu11 << std::endl;
		std::cout << mu02 << std::endl;
		std::cout << mu20 << std::endl;

		return -0.5 * atan((2 * mu11) / (mu20 - mu02));

	}
	else {
		return 0;
	}
}

int centralMoments_2(Mat & img, uint com[2], float out[3]) {
	// returns n of elements, output should be given as 3 floats, [mu_11, mu_20, mu_02]
	int mu00 = 0;

	int diff_x = 0, diff_y = 0;

	float mu11 = 0, mu20 = 0, mu02 = 0;

	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			if (img.at<uchar>(i, j) != 0) {
				mu00++;

				diff_x = (j - com[0]);
				diff_y = (i - com[1]);

				mu11 += diff_x * diff_y;
				mu20 += pow(diff_x, 2);
				mu02 += pow(diff_y, 2);
			}
		}
	}

	out[0] = mu11;
	out[1] = mu20;
	out[2] = mu02;

	return mu00;
}

float HuMoment(Mat & img) {

	int mu00;

	float mu[3] = { 0 };
	float *mu11 = &mu[0], *mu20 = &mu[1], *mu02 = &mu[2];

	uint com[2] = { 0 };

	centerOfMass(img, com);
	mu00 = centralMoments_2(img, com, mu);

	float eta20 = 0, eta02 = 0;

	eta20 = *mu20 / powf(mu00, 2);
	eta02 = *mu02 / powf(mu00, 2);
	
	return eta20 + eta02;
}