#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int
main (int argc,
      char** argv)
{

//  if (argc != 2)
//  {
//    cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
//    return -1;
//  }

  string path = "/home/anguyen/workspace/pre-grasp/data/drill01/depth_0.jpg";

  Mat image;
  //image = imread (argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
  //image = imread (path,  CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
  image = imread (path, CV_LOAD_IMAGE_UNCHANGED);

  if (!image.data)                              // Check for invalid input
  {
    cout << "Could not open or find the image" << std::endl;
    return -1;
  }

  namedWindow ("Display window", WINDOW_AUTOSIZE);                              // Create a window for display.
  imshow ("Display window", image);                   // Show our image inside it.

//  float disVal = image.at<float>(10,10);
//  cout << disVal;
  vector<unsigned short> temp;
  for (int unsigned i = 300; i < 350; ++i)
  {
    for (int unsigned j = 200; j < 250; ++j)
    {
      unsigned short disVal = image.at<unsigned short> (i, j);
      cout << disVal << " ";
      if (disVal != 0)
        temp.push_back (disVal);
    }
    cout << endl;
  }

//  cout << "-----------------------------------------";
//
//  for (int i=0; i<temp.size(); ++i)
//    cout << temp[i] << " ";

  waitKey (0);                                          // Wait for a keystroke in the window
  return 0;
}
