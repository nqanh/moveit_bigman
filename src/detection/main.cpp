#include "iostream"
#include "Detection.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int
main (int argc,
      char *argv[])
{
  string test_rgb = "/home/walkman/anh_nguyen/workspace/pre_grasp_data/drill/test07/rgb/000.jpg";
  string test_depth = "/home/walkman/anh_nguyen/workspace/pre_grasp_data/drill/test07/depth/000.jpg";
  string train_descriptor_folder = "/home/walkman/anh_nguyen/workspace/pre_grasp_data/drill/train/descriptor/";
  string train_mesh_folder = "/home/walkman/anh_nguyen/workspace/pre_grasp_data/drill/train/mesh/";
  string train_root_folder = "/home/walkman/anh_nguyen/workspace/pre_grasp_data/drill/train/root/";

  Mat frame_rgb = imread (test_rgb, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
  Mat frame_rgb_vis = frame_rgb.clone();
  Mat frame_depth = imread (test_depth, CV_LOAD_IMAGE_UNCHANGED);
  Mat frame_depth_vis = imread (test_depth, CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH);

  Detection detection;
  detection.setFrameRgb(frame_rgb);
  detection.setFrameRgbVis(frame_rgb_vis);
  detection.setFrameDepth(frame_depth);
  detection.setFrameDepthVis(frame_depth_vis);
  detection.setTrainDescriptorFolder(train_descriptor_folder);
  detection.setTrainMeshFolder(train_mesh_folder);
  detection.setTrainRootFolder(train_root_folder);

  // Create & Open Window
  namedWindow ("VIDEO", WINDOW_KEEPRATIO);

  int iresult = detection.detect();

  imshow("2D", detection.getFrameRgbVis());
  imshow("Depth", detection.getFrameDepthVis());

  waitKey(0);



  destroyWindow ("VIDEO");
  cout << "All done...!" << endl;

  return 0;
}

