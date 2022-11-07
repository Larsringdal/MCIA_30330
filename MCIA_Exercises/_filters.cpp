#include "_filters.h"


Mat lowpassKernel(size_t size) {
	// returns a lowpass filter kernel(average). If size is even, one is added to the size
	// size must be > 1 
	if (size < 2) std::invalid_argument("Size must be greater than 2");
	if ( !(size % 2) ) size++;

	return Mat::ones(size, size, CV_32F) / (float)(size*size);
}

Mat lowpass_filter(Mat &img, size_t size) {

	Mat out(img.rows, img.cols, CV_8U);

	Mat kernel = lowpassKernel(size);

	filter2D(img, out, -1, kernel);

	return out;
}

Mat highpass_filter(Mat &img) {

	float dada[] = { 0, -1 / 4.0f, 0, -1 / 4.0f, 2, -1 / 4.0f, 0, -1 / 4.0f, 0 };

	Mat kernel = Mat(3, 3, CV_32F, dada);

	Mat out(img.rows, img.cols, CV_8U);

	filter2D(img, out, -1, kernel);

	return out;
}

Mat fractile_filter(Mat &img, size_t size, const float fraction) {

	if (size < 2) std::invalid_argument("No\n");
	if (!(size % 2)) size++;

	int n = (size - 1) / 2;
	int nRows = img.rows;
	int nCols = img.cols;

	Mat out = Mat::zeros(nRows, nCols, IMREAD_GRAYSCALE);
	Mat sm = Mat::zeros(size, size, IMREAD_GRAYSCALE);
	Mat submat = Mat::zeros(size*size, 1, IMREAD_GRAYSCALE);
	Mat submat_sorted = Mat::zeros(size*size, 1, IMREAD_GRAYSCALE);

	int fracIdx = (int)(fraction*size*size) - 1;

	for (int i = n; i < nRows - n - 1; i++) {
		for (int j = n; j < nCols - n - 1; j++) {

			sm = img(Range(i - n, i + n + 1), Range(j - n, j + n + 1)).clone();
			submat = sm.reshape(1, 1);
			sort(submat, submat_sorted, SORT_ASCENDING);

			out.at<uchar>(i, j) = submat_sorted.at<uchar>(0, fracIdx);
		}
	}
	return out;
}


Mat laplaceKernel9x9(void) {
	int data[] = {
		0, 0, 1, 2, 2,
		0, 1, 5, 10, 12,
		1, 5, 15, 19, 16,
		2, 10, 19, -19, -64,
		2, 12, 16, -64, -148
	};
	Mat topleft(5, 5, CV_32S, data);

	Mat topright;// = topleft(Range(0, -1), Range::all());
	flip(topleft(Range::all(), Range(0,4)), topright, 1);
	Mat top;
	hconcat(topleft, topright, top);

	Mat botleft;
	flip(topleft(Range(0, 4), Range::all()), botleft, 0);

	Mat botright;
	flip(topright(Range(0, 4),Range::all()), botright, 0);

	Mat bot;
	hconcat(botleft, botright, bot);

	Mat kernel;
	vconcat(top, bot, kernel);

	return kernel;
}

void fitContour(Mat &src, Mat &dst, Mat &contour, int thr) {
	dst = Mat::zeros(src.rows, src.cols, CV_8UC3);
	int cont_val = 0;
	int src_val = 0;
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			cont_val = (int)contour.at<uchar>(i, j);
			src_val = (int)src.at<uchar>(i, j);
			dst.at<Vec3b>(i, j) = (cont_val > thr) ? Vec3b(0, 0, cont_val) : Vec3b(src_val, src_val, src_val);
		}
	}
}

void nonMaximalSuppression(Mat &src, Mat &out, int dist, const int thr) {
	// src : input matrix to be non maximally suppressed
	// dist : distance from current pixel to end of neighborhood
	// thr : threshold to disregard maxima
	out.create(src.rows, src.cols, CV_8U);
	out = 0u;

	if (dist % 2) dist++; // dist should be even 

	Mat submat(2 * dist + 1, 2 * dist + 1, CV_8U);
	submat = 0u;
	int nRows = src.rows;
	int nCols = src.cols;

	Point maxLoc(0, 0);
	bool check = false;

	for (int i = dist; i < src.rows - dist; i++)
	{
		for (int j = dist; j < src.cols - dist; j++)
		{
			if (src.at<uchar>(i, j) < (uchar)thr) continue;

			submat = src(Range(i - dist, i + dist - 1), Range(j - dist, j + dist - 1));

			minMaxLoc(submat, NULL, NULL, NULL, &maxLoc);
			/*
			if (!check) {
				std::cout << submat << std::endl;
				std::cout << maxLoc << std::endl;
				check = true;
			}
			*/
			if (maxLoc.x == dist && maxLoc.y == dist) {
				out.at<uchar>(i, j) = 255u;
			}
		}
	}
	imshow("test", out);
	waitKey(0);

}

std::vector<std::pair<Point2i, Point2i>> correspondence(Mat &im1, Mat &im2, Point2i kernel_size, int n_correspondences) {
	// returns n of correspondences as vector of points, ordered (x,y), NOT (rows, cols)
	if (im1.rows != im2.rows || im1.cols != im2.cols)
	{
		std::invalid_argument("Images must be same size!\n");
	}
	else if (kernel_size.x % 2 == 0 || kernel_size.y % 2 == 0) 
	{
		std::invalid_argument("Kernel size must be odd!\n");
	}

	std::vector<std::pair<Point2i, Point2i>> points;
	points.resize(n_correspondences);

	int r_sample, c_sample;

	// length from centre to kernel boundary
	int k_r_2 = ( kernel_size.x - 1 ) / 2 ; 
	int k_c_2 = ( kernel_size.y - 1 ) / 2 ;

	// for looping through each possible submatrix in img2
	int n_submat_r = im1.rows / kernel_size.x;
	int n_submat_c = im1.cols / kernel_size.y;

	Mat im1_submat = Mat::zeros(kernel_size.x, kernel_size.y, CV_32F);
	Mat im2_submat = Mat::zeros(kernel_size.x, kernel_size.y, CV_32F);

	int min_diff = INT_MAX, diff = 0;
	Point2i min_pos;  
	Mat std;

	//Mat diffmat = Mat::zeros(kernel_size.x, kernel_size.y, CV_32F);
	//Mat diffsq = Mat::zeros(kernel_size.x, kernel_size.y, CV_32F);

	for (int i = 0; i < n_correspondences; i++)
	{
		r_sample = k_r_2  + rand() % (im1.rows - k_r_2 - 1);
		c_sample = k_c_2  + rand() % (im1.cols - k_c_2 - 1);

		im1_submat = im1(Range(r_sample - k_r_2, r_sample + k_r_2 + 1), Range(c_sample - k_c_2  , c_sample + k_c_2 + 1));

		for (int j = k_r_2; j < im1.rows - k_r_2 - 1; j += 1)
		{
			for (int k = k_c_2; k < im1.cols - k_c_2 - 1; k += 1)
			{
				im2_submat = im2(Range(j - k_r_2, j + k_r_2 + 1), Range(k - k_c_2, k + k_c_2 + 1));
				meanStdDev(im2_submat, noArray(), std);
				

				diff = sum(abs(im1_submat - im2_submat))[0];
				
				if (diff < min_diff)
				{
					min_diff = diff;
					min_pos.x = k;
					min_pos.y = j;
				}
			}
		}

		points[i].first = Point2i(c_sample, r_sample);
		points[i].second = min_pos;

		min_diff = INT_MAX;
	}
	return points;
}

/*
void contourSearch(Mat &src) {
	unsigned char * pic = src.data;
	int B = src.cols;
	const int MAX_RAND = 100;
	int rimx[MAX_RAND], rimy[MAX_RAND];
	int newpos, local_tresh, draw_type;
	draw_type = 0;
	int count = 0;
	newpos = 0; // pos equals the starting position in the image ( =	y*Width + x)
	while (newpos >= 0L && newpos < src.rows * src.cols)
	{
		rimx[count] = newpos % B; // save current position in list
		rimy[count] = newpos / B;
		count++;
		draw_type = (draw_type + 6) % 8; // Select next search direction
		switch (draw_type)
		{
		case 0: if (pic[newpos + 1] > local_tresh) { newpos += 1; draw_type = 0; break; }
		case 1: if (pic[newpos + B + 1] > local_tresh) { newpos += B + 1; draw_type = 1; break; }
		case 2: if (pic[newpos + B] > local_tresh) { newpos += B; draw_type = 2; break; }
		case 3: if (pic[newpos + B - 1] > local_tresh) { newpos += B - 1; draw_type = 3; break; }
		case 4: if (pic[newpos - 1] > local_tresh) { newpos -= 1; draw_type = 4; break; }
		case 5: if (pic[newpos - B - 1] > local_tresh) { newpos -= B + 1; draw_type = 5; break; }
		case 6: if (pic[newpos - B] > local_tresh) { newpos -= B; draw_type = 6; break; }
		case 7: if (pic[newpos - B + 1] > local_tresh) { newpos -= B - 1; draw_type = 7; break; }
		case 8: if (pic[newpos + 1] > local_tresh) { newpos += 1; draw_type = 0; break; }
		case 9: if (pic[newpos + B + 1] > local_tresh) { newpos += B + 1; draw_type = 1; break; }
		case 10: if (pic[newpos + B] > local_tresh) { newpos += B; draw_type = 2; break; }
		case 11: if (pic[newpos + B - 1] > local_tresh) {
			newpos += B -1; draw_type = 3; break;
		}
		case 12: if (pic[newpos - 1] > local_tresh) { newpos -= 1; draw_type = 4; break; }
		case 13: if (pic[newpos - B - 1] > local_tresh) {
			newpos -= B + 1; draw_type = 5; break;
		}
		case 14: if (pic[newpos - B] > local_tresh) { newpos -= B; draw_type = 6; break; }
		}
		// If we are back at the beginning, we declare success
		if (newpos == 0)
			break;
		// Abort if the contour is too complex.
		if (count >= MAX_RAND)
			break;
	}
}
*/