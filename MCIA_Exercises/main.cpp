#include "ex1.h"
#include "real_time_image.h"
#include "_filters.h"



void ex1() {

	std::string img_path = samples::findFile("ex1.jpg");
	Mat img = imread(img_path, IMREAD_COLOR);

	if (img.empty())
	{
		std::cout << "Could not read the image: " << std::endl;
	}
	else {

		//flipChannel(img, 2);

		
		Mat grayImg = rgb2gray(img);

		Mat histImg = showHist(grayImg, -1);

		Mat bothImg;
		hconcat(grayImg, histImg, bothImg);
		imshow("img", bothImg);
		waitKey(0);
	}
}

void ex2(int exposure) {

	int k = 0;

	Mat img;
	Mat imgGray;
	Mat imgHist;
	Mat imgConcat;

	while (k != 27) {
		
		img.setTo(Scalar(0, 0, 0));
		imgGray.setTo(Scalar(0));
		imgHist.setTo(Scalar(0));

		grab_img(img, exposure);
		imgGray = rgb2gray(img);
		imgHist = showHist(imgGray, -1);

		hconcat(imgGray, imgHist, imgConcat);
		imshow("img", imgConcat);
		k = waitKey(0);

		img.release();
		imgGray.release();
		imgHist.release();
		imgConcat.release();
	}
}


void ex3(int thr) {
	std::string filename = samples::findFile("pen.webp");
	Mat img = imread(filename, IMREAD_COLOR);

	if (img.empty())
	{
		std::cout << "Could not read the image: " << std::endl;
	}
	else 
	{
		Mat grayImg = rgb2gray(img);

		Mat imgThr = binaryThreshold(grayImg, thr);

		uint com[2];
		centerOfMass(imgThr, com);

		Mat imgCom = drawCenterOfMass(imgThr, com);

		Mat imgConcat;
		hconcat(img, imgCom, imgConcat);

		imshow("img with threshold = " + std::to_string(thr), imgConcat);

		//waitKey(0);

		float theta = centralMoments(imgThr, com);

		//Draw line through center of mass, with proper angle.
		int linelength = 500;
		int x1 = (int)(com[0] - 0.5*linelength * cosf(theta));
		int x2 = (int)(com[0] + 0.5*linelength * cosf(theta));
		int y1 = (int)(com[0] - 0.5*linelength * sinf(theta));
		int y2 = (int)(com[0] + 0.5*linelength * sinf(theta));
		line(img, Point(x1, y1), Point(x2, y2), Scalar(0, 0, 255), 2);

		imshow("With angle : " + std::to_string(theta) + " rad", img);
		waitKey(0);
	}
}

void ex_HuMoment(int thr) {

	std::string filename = samples::findFile("penrot1.jpg");
	Mat img1 = imread(filename, IMREAD_GRAYSCALE);
	filename = samples::findFile("penrot2.jpg");
	Mat img2 = imread(filename, IMREAD_GRAYSCALE);

	if (img1.empty() || img2.empty())
	{
		std::cout << "Could not read the image: " << std::endl;
	}
	else {
		Mat imgThr1 = binaryThreshold(img1, thr);
		Mat imgThr2 = binaryThreshold(img2, thr);

		std::cout << "Hu1 = " << HuMoment(imgThr1) << std::endl;
		std::cout << "Hu2 = " << HuMoment(imgThr2) << std::endl;
	}
}

void lowp_filter_img(size_t size) {
	std::string filepath = samples::findFile("ex1.jpg");
	Mat img = imread(filepath, IMREAD_GRAYSCALE);
	Mat img_filt = lowpass_filter(img, size);
	Mat img_comb;

	hconcat(img, img_filt, img_comb);

	imshow("Filtered 3x3 average", img_comb);
	waitKey(0);
}

void highp_filter_img() {
	std::string filepath = samples::findFile("ex1.jpg");
	Mat img = imread(filepath, IMREAD_GRAYSCALE);
	Mat img_filt = highpass_filter(img);
	Mat img_comb;

	imshow("Filtered", img_filt);
	waitKey(0);

	Mat filthist = showHist(img_filt, -1);
	Mat imhist = showHist(img, -1);
	
	hconcat(imhist, filthist, img_comb);

	imshow("Filtered 3x3 average", img_comb);
	waitKey(0);
}

void fracfilt() {
	std::string filepath = samples::findFile("ex1.jpg");
	Mat img = imread(filepath, IMREAD_GRAYSCALE);
	Mat img_filt = fractile_filter(img, 9, 0.5);
	Mat imcom;
	hconcat(img, img_filt, imcom);
	imshow("Filtered", imcom);
	waitKey(0);
}

void laplacefilt() {
	std::string filepath = samples::findFile("ex1.jpg");
	Mat img = imread(filepath, IMREAD_GRAYSCALE);

	Mat kern = laplaceKernel9x9();

	Mat out;

	filter2D(img, out, -1, kern);

	//Mat nonmax;
	//nonMaximalSuppression(out,nonmax, 10, 150);

	Mat imcom;
	hconcat(img, out, imcom);
	imshow("Laplace 9x9", imcom);
	waitKey(0);
}

void realTimeLaplace() {
	Mat output;
	VideoCapture cap(0); //Try changing to 1 if not working
	//Check if camera is available
	if (!cap.isOpened())
	{
		std::invalid_argument("Could not initialize capturing...\n");
	}
	Mat kern = laplaceKernel9x9();
	//Mat kern_lowpass = lowpassKernel(9);
	Mat grey;
	Mat imout, temp;
	//Show Camera output
	while (1) {
		cap >> output;
		cvtColor(output, grey, COLOR_BGR2GRAY);
		//grey = rgb2gray(output);
		//filter2D(grey, temp, -1, kern_lowpass);
		filter2D(grey, imout, -1, kern);
		//fitContour(grey, temp, imout);

		imshow("webcam input", imout);

		char c = (char)waitKey(10);
		if (c == 27) break; //Press escape to stop program
	}
	
}

void pixelCorrespondence() {

	String filepath = samples::findFile("mont.jpg");
	Mat img = imread(filepath, IMREAD_GRAYSCALE);


	//imshow("donkeh", img);
	//waitKey(0);

	int angle = 30;

	//artificially transform img :)
	Point2f center((img.cols - 1) / 2.0, (img.rows - 1) / 2.0);
	Mat rot_mat = getRotationMatrix2D(center, angle, 1.0);

	Mat rotated_img;
	warpAffine(img, rotated_img, rot_mat, img.size());
	//imshow("Rotated img", rotated_img);
	//waitKey(0);

	std::vector<std::pair<Point2i, Point2i>> points;
	points = correspondence(img, rotated_img, Point2i(31, 31), 20);

	Mat img_con;
	hconcat(img, rotated_img, img_con);

	Mat img_concat_rgb;
	cvtColor(img_con, img_concat_rgb, COLOR_GRAY2BGR);

	for (int i = 0; i < points.size(); i++) 
	{
		std::cout << points[i].first << " : " << points[i].second << std::endl;
		Point point2 = points[i].second;
		point2.x += img.cols;
		line(img_concat_rgb, points[i].first, point2, Scalar(0, 0, 255), 1);
	}

	imshow("test", img_concat_rgb);
	waitKey(0);
	
		
}

int main() {
	srand(time(0));
	//ex1();
	//ex2(150);
	//ex3(50);

	//ex_HuMoment(50);
	//filter_img(21);
	//highp_filter_img();
	//fracfilt();
	//laplacefilt();
	//realTimeLaplace();
	pixelCorrespondence();

	return 0;
}