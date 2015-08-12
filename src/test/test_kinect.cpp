#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

int
main ()
{
  cout << "opening device(s)" << endl;

  cout << cv::getBuildInformation ();

  VideoCapture sensor1;
  sensor1.open (CV_CAP_OPENNI);

  if (!sensor1.isOpened ())
  {
    cout << "Can not open capture object 1." << endl;
    return -1;
  }

  double focal_length = sensor1.get (CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
  printf ("Focal length = %f\n", focal_length);

  double focal = sensor1.get (CV_CAP_PROP_OPENNI_FOCAL_LENGTH);
  printf ("Focal = %f\n", focal);

  for (;;)
  {
    Mat depth;
    Mat rgb;

    if (!sensor1.grab ())
    {
      cout << "Sensor1 can not grab images." << endl;
      return -1;
    }
    else if (sensor1.retrieve (depth, CV_CAP_OPENNI_DEPTH_MAP))
    {
      imshow ("depth1", depth);
      unsigned short depVal;
      depVal = depth.at<unsigned short> (320, 240);
      cout << "center point distance: " << depVal << endl;
    }

    if (sensor1.retrieve (rgb, CV_CAP_OPENNI_BGR_IMAGE))
      imshow ("rgb", rgb);

    if (waitKey (30) == 27)
      break;  //ESC to exit

  }
}
