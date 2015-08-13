#include "iostream"
#include "Detection.h"
#include "Utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int
main (int argc,
      char *argv[])
{
  string tutorial_path = "/home/walkman/anh_nguyen/workspace/pre_grasp_data/";  // path to data
  int runningMode = 1;

#if(1) // for testing with image list only
  string testID = "";
  string object_class = "drill";  //name of object class
  string object_test_id = "07";  //test id - if want to run from "test07" folder --> testid=07
  string root_file_ID = "07_000";   //root rotation matrix to be loaded
  string save_root_file_ID = "000xxx";  //change to frame ID want to be saved as root rotation matrix(e.g. "000")

  string test_database_path = tutorial_path + object_class + "/test" + object_test_id + "/";
  string test_rgb_folder = test_database_path + "rgb/";
  string test_depth_folder = test_database_path + "depth/";

  unsigned int iCount = 0;  // count test images
  bool isBreak = false;
  vector<string> listDepthTest;
  vector<string> listRgbTest;
  listRgbTest = listAllFiles (test_rgb_folder);
  std::sort (listRgbTest.begin (), listRgbTest.end ());
  listDepthTest = listAllFiles (test_depth_folder);
#endif

  // main input frame
  Mat frame_rgb, frame_rgb_vis, frame_depth, frame_depth_vis;

  while (1)
  {
    string test_rgb;
    string test_depth;

    // capture rgb image and depth image
    switch (runningMode)
    {
      case 1:  // TEST_FROM_IMAGE_FOLDER
        if (iCount < listRgbTest.size ())
        {
          testID = getFileName (listRgbTest[iCount]);
          test_rgb = test_rgb_folder + testID + ".jpg";
          test_depth = test_depth_folder + testID + ".jpg";
          ++iCount;

          frame_rgb = imread (test_rgb, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
          frame_rgb_vis = frame_rgb.clone ();
          frame_depth = imread (test_depth, CV_LOAD_IMAGE_UNCHANGED);
          frame_depth_vis = imread (test_depth, CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH);
        }
        else
        {
          cout << "Run out of images ..." << endl;
          isBreak = true;
        }
        break;

      case 2: // TEST FROM VIDEO
        break;

      case 3: // TEST FROM LIVE FEED
        break;

      default:
        break;
    }

    if (isBreak)  // out of image
      break;

    if (!frame_rgb.data || !frame_depth.data)
    {
      cout << "RGB or Depth image is empty!" << endl;
      //break;
      continue;
    }

    string train_descriptor_folder = "/home/walkman/anh_nguyen/workspace/pre_grasp_data/drill/train/descriptor/";
    string train_mesh_folder = "/home/walkman/anh_nguyen/workspace/pre_grasp_data/drill/train/mesh/";
    string train_root_folder = "/home/walkman/anh_nguyen/workspace/pre_grasp_data/drill/train/root/";

    Detection detection;
    detection.setFrameRgb (frame_rgb);
    detection.setFrameRgbVis (frame_rgb_vis);
    detection.setFrameDepth (frame_depth);
    detection.setFrameDepthVis (frame_depth_vis);
    detection.setTrainDescriptorFolder (train_descriptor_folder);
    detection.setTrainMeshFolder (train_mesh_folder);
    detection.setTrainRootFolder (train_root_folder);

    // Create & Open Window
    namedWindow ("VIDEO", WINDOW_KEEPRATIO);

    int iresult = detection.detect ();

    imshow ("2D", detection.getFrameRgbVis ());
    imshow ("Depth", detection.getFrameDepthVis ());

    char waitKey;
    waitKey = cvvWaitKey(0);
    if (waitKey == 27)  // press "ECS" to break
      break;

  }

  destroyWindow ("VIDEO");
  cout << "All done...!" << endl;

  return 0;
}

