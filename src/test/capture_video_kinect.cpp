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
  VideoWriter video_depth ("out_depth.avi", CV_FOURCC ('M', 'J', 'P', 'G'), 10, Size (frame_width, frame_height), false);

  int i = 0;

  for (;;)
  {

    Mat frame, depth;
    vcap.grab ();
    vcap.retrieve (frame, CV_CAP_OPENNI_BGR_IMAGE);
    vcap.retrieve (depth, CV_CAP_OPENNI_DEPTH_MAP);

    //vcap >> frame;
    video.write (frame);
    video_depth.write (depth);

    imshow ("Frame", frame);
    //if (i < 10)
    if (i < 30000)
    {
      ostringstream convert;
      convert << i;
      string icount = convert.str ();
      string name = "image_" + icount + ".jpg";
      string name_depth = "depth_" + icount + ".jpg";
      imwrite (name, frame);
      imwrite (name_depth, depth);
      i++;
    }

    char c = (char) waitKey (33);
    if (c == 27)
      break;
  }
  return 0;
}

