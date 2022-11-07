#include "camCalib.h"

std::vector<Mat> loadImagesFromFolder(std::string folderPath) {
	std::vector<std::string> fileNames;
	glob(folderPath + "*.jpg", fileNames, false);

	std::vector<Mat> images;
	size_t count = fileNames.size();
	for (size_t i = 0; i < count; i++)
	{
		images.push_back(imread(fileNames[i]));
	}

	return images;
}


std::vector<std::vector<Point2f>> getCornerCoords(std::string folderPath, Size2i boardSize) {

	std::vector<std::string> fileNames;
	glob(folderPath + "*.jpg", fileNames, false); 

	size_t n_images = fileNames.size();

	std::vector<std::vector<Point2f>> corners;
	std::vector<Point2f> curCorner;
	Mat img;

	bool found;
	for (size_t i = 0; i < n_images; i++)
	{
		img = imread(fileNames[i]);
		found = findChessboardCorners(img, boardSize, curCorner, CALIB_CB_ADAPTIVE_THRESH);
		if (!found)
			std::cout << "Corner not found in image " << i << std::endl;
		else
			corners.push_back(curCorner);
	}
	return corners;
}

double runCameraCalibration(std::string folderPath, Size2i boardSize,
							Mat & cameraMatrix, Mat & distCoeffs) {

	std::vector<std::vector<Point2f>> corners = getCornerCoords(folderPath, boardSize);

	std::vector<Point3f> objp; // chessboard corners in world frame, assumes top left as origo
	int m = boardSize.width, n = boardSize.height;
	
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			objp.push_back(Point3f(j - (m - 1)/2.0, i - (n - 1)/2.0, 0));
		}
	}

	std::vector<std::vector<Point3f>> objpoints; // gotta store the 3d coords once for each calibration image
	for (int i = 0; i < corners.size(); i++)
	{
		objpoints.push_back(objp);
	}

	std::vector<Mat> rvecs;
	std::vector <Mat> tvecs;

	double rms = calibrateCamera(objpoints, corners, boardSize, cameraMatrix, distCoeffs, rvecs, tvecs);
	std::cout << "RMS BY ZHANG CALIBRATION === " << rms << std::endl;
	return reprojectionError(corners, objpoints, cameraMatrix, rvecs, tvecs, distCoeffs);
}


double reprojectionError(std::vector<std::vector<Point2f>> & imgPoints, std::vector<std::vector<Point3f>> & objPoints, 
						Mat & cameraMatrix, std::vector<Mat> & rvecs, std::vector <Mat> & tvecs, Mat & distCoeffs) {

	double sumError = 0, error;

	if (imgPoints.size() != objPoints.size() || imgPoints[0].size() != objPoints[0].size())
		std::invalid_argument("Image points and object points must be of same dimensions");

	std::vector<Point2f> projectedImgPoints;

	for (size_t i = 0; i < imgPoints.size(); i++)
	{
		
		projectPoints(objPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, projectedImgPoints);
		error = norm(Mat(imgPoints[i]), Mat(projectedImgPoints), NORM_L2);
		sumError += error * error;
	}
	return sqrt(sumError / (double)imgPoints.size());
}



bool saveCameraMatrix(std::string fileName, Mat & cameraMatrix, std::string matName) {

	FileStorage file(fileName, FileStorage::WRITE);

	if (!file.isOpened())
		return false;

	file << matName << cameraMatrix;
	file.release();

	return true;
}



Mat readCameraMatrix(std::string fileName, std::string matName) {
	FileStorage file(fileName, FileStorage::READ);

	if (!file.isOpened())
		std::cout << "Failed to load matrix with name: " << matName << " from file " << fileName << std::endl;

	Mat K;
	file[matName] >> K;
	file.release();

	return K;
}

