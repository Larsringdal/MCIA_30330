#include "real_time_image.h"

void grab_img(Mat &output, int exposure)
{
	VideoCapture cap(0); //Try changing to 1 if not working
	//Check if camera is available
	if (!cap.isOpened())
	{
		std::cout << "Could not initialize capturing...\n";
	}
	cap.set(CAP_PROP_EXPOSURE, exposure);
	//Show Camera output
	cap >> output;
}

