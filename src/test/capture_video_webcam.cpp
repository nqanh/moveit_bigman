#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

int
main ()
{

  VideoCapture vcap (0);
  if (!vcap.isOpened ())
  {
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  cout << "Starting ..." << endl;
  int frame_width = vcap.get (CV_CAP_PROP_FRAME_WIDTH);
  int frame_height = vcap.get (CV_CAP_PROP_FRAME_HEIGHT);
  cout << "frame width:  " << frame_width << endl;
  cout << "frame height: " << frame_height << endl;

  VideoWriter video ("out.avi", CV_FOURCC ('M', 'J', 'P', 'G'), 10, Size (frame_width, frame_height), true);

  int i = 0;

  for (;;)
  {

    Mat frame;
    vcap >> frame;
    video.write (frame);
    imshow ("Frame", frame);
    if (i < 10)
    {
      ostringstream convert;
      convert << i;
      string name = "image_" + convert.str () + ".jpg";
      imwrite (name, frame);
      i++;
    }

    char c = (char) waitKey (33);
    if (c == 27)
      break;
  }
  return 0;
}

