#include <iostream>
#include <time.h>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>

#include "Mesh.h"
#include "Model.h"
#include "RootFrame.h"
#include "PnPProblem.h"
#include "RobustMatcher.h"
#include "ModelRegistration.h"
#include "Utils.h"

using namespace cv;
using namespace std;
using namespace Eigen;

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)

string tutorial_path = "/home/walkman/anh_nguyen/workspace/pre_grasp_data/";  // path to data

// Intrinsic camera parameters: Asus Xtion Pro for rgb
double params_WEBCAM[] = { 570.3422241210938,   // fx
    570.3422241210938,  // fy
    319.5,      // cx
    239.5 };     // cy

// control
bool isLive = false;  // "false" = use recored video; "true" = use live feed
bool isDebug = true;
int runningMode = 1;  // running mode 1-use image folder; 2-use recorded video; 3-use live feed

// kinect distance
const int nearDis = 500;  // mm
const int farDis = 2500;  // mm
const int search_radius = 200;

// Some basic colors
Scalar red (0, 0, 255);
Scalar green (0, 255, 0);
Scalar blue (255, 0, 0);
Scalar yellow (0, 255, 255);

// Robust Matcher parameters
int numKeyPoints = 2000;      // number of detected keypoints
float ratioTest = 0.70f;          // ratio test
bool fast_match = false;       // fastRobustMatch() or robustMatch()

// RANSAC parameters
int iterationsCount = 500;      // number of Ransac iterations.
float reprojectionError = 2.0;  // maximum allowed distance to consider it an inlier.
double confidence = 0.95;        // ransac successful confidence.

// Kalman Filter parameters
int minInliersKalman = 30;    // Kalman threshold updating

// PnP parameters
int pnpMethod = SOLVEPNP_ITERATIVE;

/**  Functions headers  **/
void
help ();

Point3f
calculate3DWorld (Point2f point2d,
                  float zw);

float
distanceToCenter (int x,
                  int y,
                  int cx,
                  int cy);

int
searchDepthPoint (vector<Point2f> posePoints,
                  Mat frame,
                  const int maxDistance,
                  cv::Point2f &root2D);

void
initKalmanFilter (KalmanFilter &KF,
                  int nStates,
                  int nMeasurements,
                  int nInputs,
                  double dt);
void
updateKalmanFilter (KalmanFilter &KF,
                    Mat &measurements,
                    Mat &translation_estimated,
                    Mat &rotation_estimated);
void
fillMeasurements (Mat &measurements,
                  const Mat &translation_measured,
                  const Mat &rotation_measured);

Mat
computeDiffOrientation (Mat rotation_root,
                        Mat rotation_est);

int
main (int argc,
      char *argv[])
{

  help ();

#if(1) // init MoveIt!

  ros::init (argc, argv, "main_detection");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner (1);
  spinner.start ();

//  // wait for RIVZ
//  sleep (15.0);
//
//  moveit::planning_interface::MoveGroup group ("right_arm");
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory> ("/move_group/display_planned_path", 1, true);
//  moveit_msgs::DisplayTrajectory display_trajectory;
//  moveit::planning_interface::MoveGroup::Plan my_plan;
//
//  // Getting Basic Information
//  std::string planning_frame = group.getPlanningFrame ();
//  std::string end_effector_link = group.getEndEffectorLink ();
//
//  ROS_INFO("Planing frame: %s", planning_frame.c_str ());
//  ROS_INFO("EndEffectorLink frame: %s", end_effector_link.c_str ());

#endif

#if(1) // help string
  const String keys = "{help h        |      | print this message                   }"
      "{video v       |      | path to recorded video               }"
      "{model         |      | path to yml model                    }"
      "{mesh          |      | path to ply mesh                     }"
      "{keypoints k   |2000  | number of keypoints to detect        }"
      "{ratio r       |0.7   | threshold for ratio test             }"
      "{iterations it |500   | RANSAC maximum iterations count      }"
      "{error e       |2.0   | RANSAC reprojection errror           }"
      "{confidence c  |0.95  | RANSAC confidence                    }"
      "{inliers in    |30    | minimum inliers for Kalman update    }"
      "{method  pnp   |0     | PnP method: (0) ITERATIVE - (1) EPNP - (2) P3P - (3) DLS}"
      "{fast f        |true  | use of robust fast match             }";
  CommandLineParser parser (argc, argv, keys);
#endif

#if(1) // parse input

  if (parser.has ("help"))
  {
    parser.printMessage ();
    return 0;
  }
  else
  {
//    video_read_path = parser.get<string> ("video").size () > 0 ? parser.get<string> ("video") : video_read_path;
//    yml_read_path = parser.get<string> ("model").size () > 0 ? parser.get<string> ("model") : yml_read_path;
//    ply_read_path = parser.get<string> ("mesh").size () > 0 ? parser.get<string> ("mesh") : ply_read_path;
    numKeyPoints = !parser.has ("keypoints") ? parser.get<int> ("keypoints") : numKeyPoints;
    ratioTest = !parser.has ("ratio") ? parser.get<float> ("ratio") : ratioTest;
    fast_match = !parser.has ("fast") ? parser.get<bool> ("fast") : fast_match;
    iterationsCount = !parser.has ("iterations") ? parser.get<int> ("iterations") : iterationsCount;
    reprojectionError = !parser.has ("error") ? parser.get<float> ("error") : reprojectionError;
    confidence = !parser.has ("confidence") ? parser.get<float> ("confidence") : confidence;
    minInliersKalman = !parser.has ("inliers") ? parser.get<int> ("inliers") : minInliersKalman;
    pnpMethod = !parser.has ("method") ? parser.get<int> ("method") : pnpMethod;
  }
#endif

#if(1) // set path
  string object_class = "drill";  //name of object class
  string object_test_id = "07";  //test id - if want to run from "test07" folder --> testid=07
  string root_file_ID = "07_000";   //root rotation matrix to be loaded
  string save_root_file_ID = "000xxx";  //change to frame ID want to be saved as root rotation matrix(e.g. "000")

  string test_database_path = tutorial_path + object_class + "/test" + object_test_id + "/";
  string train_database_path = tutorial_path + object_class + "/train/";

  string test_rgb_folder = test_database_path + "rgb/";
  string test_depth_folder = test_database_path + "depth/";

  string train_descriptor_folder = train_database_path + "descriptor/";
  string train_mesh_folder = train_database_path + "mesh/";
  string root_folder = train_database_path + "root/";

  string save_root_file_path = root_folder + object_test_id + "_" + save_root_file_ID + ".yml";
  string root_file_path = root_folder + root_file_ID + ".yml";

  vector<string> listDepthTest;
  vector<string> listRgbTest;
#endif

#if(1) // capture rgb and depth stream
  switch (runningMode)
  {
    case 1:  // TEST_FROM_IMAGE_FOLDER
      listRgbTest = listAllFiles (test_rgb_folder);
      std::sort (listRgbTest.begin (), listRgbTest.end ());
      listDepthTest = listAllFiles (test_depth_folder);
      break;
    case 2:  // TEST FROM RECORED VIDEO

      break;
    case 3:  // TEST FROM LIVE FEED
      break;

    default:
      break;
  }
#endif

#if(1) // variables for main()

  // planning
  bool success;

  // image and depth path - only use in TEST_FROM_IMAGE_FOLDER
  string test_rgb_path;
  string test_depth_path;

  // rgb and depth frame
  Mat frame_rgb, frame_rgb_vis, frame_depth, frame_depth_vis;

  // Create & Open Window
  namedWindow ("VIDEO", WINDOW_KEEPRATIO);

  unsigned int iCount = 0;  // count test images
  bool isBreak = false;

  vector<double> detection_score;
  vector<PnPProblem> detection_object;

  string mainLoop = "=============================================================================\n";
  string descLoop = "-------------------------------------\n";

#endif

#if(1) // load descriptor and root rotation matrix
  // load descriptor
  vector<string> listDescriptorTrain = listAllFiles (train_descriptor_folder);

  // load root rotation matrix
  RootFrame root_frame;
  cout << root_file_path << endl;
  root_frame.load (root_file_path);
  Mat rotation_matrix_root = root_frame.getRootRotation ();
  if (!rotation_matrix_root.data)
  {
    cout << "Error: Root rotation matrix is empty" << endl;
    return -1;
  }
//  cout << "Root rotation matrix: " << endl;
//  cout << rotation_matrix_root << endl;

#endif

  // main loop - via all image or frame
  //while (cap.read (frame) && cap_depth.read (frame_depth) && waitKey (30) != 27)  // capture frame until ESC is pressed
  //while (cap.read (frame) && waitKey (30) != 27)  // capture frame until ESC is pressed
  //while (waitKey (30) != 27)

  cout << ".......... START VISION SYSTEM .........." << endl;
  sleep(1);

  while (1)
  {
    cout << mainLoop;
    string testID = "";
    // capture rgb image and depth image
    switch (runningMode)
    {
      case 1:  // TEST_FROM_IMAGE_FOLDER
        if (iCount < listRgbTest.size ())
        {
          testID = getFileName (listRgbTest[iCount]);
          test_rgb_path = test_rgb_folder + testID + ".jpg";
          test_depth_path = test_depth_folder + testID + ".jpg";
          ++iCount;
          cout << "Test rgb input: " << test_rgb_path << endl;
          cout << "Test depth input: " << test_depth_path << endl;

          frame_rgb = imread (test_rgb_path, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
          frame_depth = imread (test_depth_path, CV_LOAD_IMAGE_UNCHANGED);
          frame_depth_vis = imread (test_depth_path, CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH);
          //frame_image_depth = imread (image_depth_path, CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH);
          //frame_image_depth = imread (image_depth_path, -1);
          //cout << "Type: " << frame_image_depth.type () << endl;
          //frame_image_depth.convertTo(frame_image_depth, CV_32F); // convert the image data to float type

        }
        else
        {
          cout << "Run out of images ..." << endl;
          isBreak = true;
        }
        break;
      case 2:

        // Capture video
        //VideoCapture cap, cap_depth;

        //  // open Kinect sensor
        //    cv::VideoCapture cap (CV_CAP_OPENNI);
        //    if (!cap.isOpened ())
        //    {
        //      printf ("Could not open OpenNI sensor\n");
        //      return -1;
        //    }
        //    printf ("OpenNI ok!\n");
        //    cap.set (CV_CAP_PROP_OPENNI_REGISTRATION, 1);
        //    double focal_length = cap.get (CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
        //    printf("Focal length = %f\n", focal_length);

        //  // open depth video
        //      cap_depth.open (video_depth_path);
        //      if (!cap_depth.isOpened ())
        //      {
        //        cout << "Could not open depth video" << endl;
        //        return -1;
        //      }

        //    // open video file
        //    cap.open (video_read_path);                      // open a recorded video
        //
        //    if (!cap.isOpened ())   // check if we succeeded
        //    {
        //      cout << "Could not open video device" << endl;
        //      return -1;
        //    }

        break;
      case 3:
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

    // clone for visualization,
    frame_rgb_vis = frame_rgb.clone ();
    if (runningMode != 1)
      frame_depth_vis = frame_depth.clone ();

    // search through descriptor database
    for (unsigned int iDes = 0; iDes < listDescriptorTrain.size (); ++iDes)
    {
      cout << descLoop;

#if(1) // Declare variables

      // get ID of train object
      string objectID = getFileName (listDescriptorTrain[iDes]);
      cout << "Object ID: " << objectID << endl;

      // get full path
      string descriptor_path = train_descriptor_folder + objectID + ".yml";
      string mesh_path = train_mesh_folder + objectID + ".ply";
      //cout << descriptor_path << endl;
      //cout << mesh_path << endl;

      // create object instance
      PnPProblem pnp_detection (params_WEBCAM);      // main detection
      PnPProblem pnp_detection_est (params_WEBCAM);  // estimated detection
      RobustMatcher rmatcher;                        // instantiate RobustMatcher
      Ptr<FeatureDetector> orb = ORB::create ();     // orb feature
      rmatcher.setFeatureDetector (orb);             // set feature detector
      rmatcher.setDescriptorExtractor (orb);         // set descriptor extractor

      // flann parameters
      Ptr<flann::IndexParams> indexParams = makePtr<flann::LshIndexParams> (6, 12, 1);  // instantiate LSH index parameters
      Ptr<flann::SearchParams> searchParams = makePtr<flann::SearchParams> (50);  // instantiate flann search parameters
      // FlannBased matcher
      Ptr<DescriptorMatcher> matcher = makePtr<FlannBasedMatcher> (indexParams, searchParams);
      rmatcher.setDescriptorMatcher (matcher);
      rmatcher.setRatio (ratioTest);  // set ratio test parameter

      // Kalman Filer
      KalmanFilter KF;             // Kalman Filter
      int nStates = 18;            // the number of states
      int nMeasurements = 6;       // the number of measured states
      int nInputs = 0;             // the number of control actions
      double dt = 0.125;           // time between measurements (1/FPS)
      initKalmanFilter (KF, nStates, nMeasurements, nInputs, dt);
      Mat measurements (nMeasurements, 1, CV_64F);
      measurements.setTo (Scalar (0));
      bool good_measurement = false;

      // Descriptor and mesh
      Model model;
      model.load (descriptor_path);
      Mesh mesh;
      mesh.load (mesh_path);
      // load descriptor data
      vector<Point3f> list_points3d_model = model.get_points3d ();  // list with model 3D coordinates
      Mat descriptors_model = model.get_descriptors ();  // list with descriptors of each 3D coordinate

      // timer
      time_t start, end;
      //double fps, sec;
      int counter = 0;  // frame counter
      time (&start);

      //matching between model and scene descriptors
      vector<DMatch> good_matches;  // to obtain the 3D points of the model
      vector<KeyPoint> keypoints_scene;  // to obtain the 2D points of the scene

#endif
      if (fast_match)
      {
        rmatcher.fastRobustMatch (frame_rgb, good_matches, keypoints_scene, descriptors_model);
      }
      else
      {
        rmatcher.robustMatch (frame_rgb, good_matches, keypoints_scene, descriptors_model);
      }

      // 2D/3D correspondences

      vector<Point3f> list_points3d_model_match;  // container for the model 3D coordinates found in the scene
      vector<Point2f> list_points2d_scene_match;  // container for the model 2D coordinates found in the scene

      for (unsigned int match_index = 0; match_index < good_matches.size (); ++match_index)
      {
        Point3f point3d_model = list_points3d_model[good_matches[match_index].trainIdx];  // 3D point from model
        Point2f point2d_scene = keypoints_scene[good_matches[match_index].queryIdx].pt;  // 2D point from the scene
        list_points3d_model_match.push_back (point3d_model);  // add 3D point
        list_points2d_scene_match.push_back (point2d_scene);  // add 2D point
      }

      // Draw outliers
      draw2DPoints (frame_rgb_vis, list_points2d_scene_match, red);

      Mat inliers_idx;
      vector<Point2f> list_points2d_inliers;

      if (good_matches.size () > 0)  // None matches, then RANSAC crashes
      {
        cout << "Matches size: " << good_matches.size () << endl;

        //  Estimate the pose pnp ransac
        pnp_detection.estimatePoseRANSAC (list_points3d_model_match, list_points2d_scene_match, pnpMethod, inliers_idx, iterationsCount, reprojectionError,
                                          confidence);

        // Catch the inliers keypoints to draw
        for (int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index)
        {
          int n = inliers_idx.at<int> (inliers_index);      // i-inlier
          Point2f point2d = list_points2d_scene_match[n];  // i-inlier point 2D
          list_points2d_inliers.push_back (point2d);  // add i-inlier to list
        }

        // Draw inliers points 2D
        draw2DPoints (frame_rgb_vis, list_points2d_inliers, blue);

        // Update Kalman Filter

        good_measurement = false;

        // GOOD MEASUREMENT
        if (inliers_idx.rows >= minInliersKalman)
        {

          // Get the measured translation
          Mat translation_measured (3, 1, CV_64F);
          translation_measured = pnp_detection.get_t_matrix ();
          //cout << "Translation measured: " << endl << translation_measured << endl;

          // Get the measured rotation
          Mat rotation_measured (3, 3, CV_64F);
          rotation_measured = pnp_detection.get_R_matrix ();
          //cout << "Rotation measured: " << endl << rotation_measured << endl;

          // fill the measurements vector
          fillMeasurements (measurements, translation_measured, rotation_measured);

          good_measurement = true;

        }

      }

      // always estimate pose
      // Instantiate estimated translation and rotation
      Mat translation_estimated (3, 1, CV_64F);
      Mat rotation_estimated (3, 3, CV_64F);

      // update the filter with good measurements
      updateKalmanFilter (KF, measurements, translation_estimated, rotation_estimated);
      //cout << "Translation estimated: " << endl << translation_estimated << endl;
      //cout << "Rotation estimated: " << endl << rotation_estimated << endl;

      // Set estimated projection matrix
      pnp_detection_est.set_P_matrix (rotation_estimated, translation_estimated);

      // Draw pose
      cout << "Good measurement: " << good_measurement << endl;
      if (good_measurement)
      {
        drawObjectMesh (frame_rgb_vis, &mesh, &pnp_detection, green);  // draw current pose

        float l = 5;
        vector<Point2f> pose_points2d;
        pose_points2d.push_back (pnp_detection_est.backproject3DPoint (Point3f (0, 0, 0)));  // axis center
        pose_points2d.push_back (pnp_detection_est.backproject3DPoint (Point3f (l, 0, 0)));  // axis x
        pose_points2d.push_back (pnp_detection_est.backproject3DPoint (Point3f (0, l, 0)));  // axis y
        pose_points2d.push_back (pnp_detection_est.backproject3DPoint (Point3f (0, 0, l)));  // axis z
        draw3DCoordinateAxes (frame_rgb_vis, pose_points2d);         // draw axes

        //for (unsigned i=0; i<pose_points2d.size(); ++i)
        for (unsigned i = 0; i < 1; ++i)
        {
          cout << "Original Point:" << " X=" << pose_points2d[i].x << " Y=" << pose_points2d[i].y << endl;
        }

        cv::Point3f root_3D;
        cv::Point2f root_2D;

        root_3D.z = searchDepthPoint (pose_points2d, frame_depth, search_radius, root_2D);

        //depth_val = searchDepthPoint (pose_points2d, frame_image_depth, search_radius, depth_idx, depth_idy);
        //if (depth_val == 0)
        if (root_3D.z == 0)
        {
          cout << "Can't find depth point. Should increase search radius." << endl;
        }
        draw2DPoints (frame_depth_vis, pose_points2d, green);
        draw2DSinglePoint (frame_depth_vis, root_2D, yellow);
        drawRegtangle (frame_depth_vis, pose_points2d[0], red, search_radius, search_radius);

        // calculate position of cordinate root at world space
        root_3D = calculate3DWorld (root_2D, root_3D.z);
        cout << "Root 3D: " << root_3D << endl;

        // compute different between root rotation matrix and current estimated matrix
        Mat rotation_diff = computeDiffOrientation (rotation_matrix_root, rotation_estimated);

#if(1) // compute Euler angles from rotation matrix
//        cout << "Rotation Diff: " << endl;
//        cout << rotation_diff << endl << endl;

        // Method 1: Use Eigen class
        // convert Opencv rotation to Eigen matrix
        Eigen::Matrix3f rotation_diff_eigen;
        for (unsigned int it = 0; it < 3; ++it)
          for (unsigned int jt = 0; jt < 3; ++jt)
          {
            rotation_diff_eigen (it, jt) = rotation_diff.at<double> (it, jt);
          }
//        cout << "Rotation Eigen: " << endl;
//        cout << rotation_diff_eigen << endl << endl;

        Eigen::Vector3f ea = rotation_diff_eigen.eulerAngles (0, 1, 2);
//        cout << "Euler angles in radian:" << endl;
//        cout << ea << endl << endl;

        cout << "Euler angles in degree:" << endl;
        float roll = ea (0) * RAD2DEG;
        float pitch = ea (1) * RAD2DEG;
        float yaw = ea (2) * RAD2DEG;
        string sroll = FloatToString (roll);
        string spitch = FloatToString (pitch);
        string syaw = FloatToString (yaw);
        string rpw1 = "R: " + sroll + " P: " + spitch + " Y: " + syaw;
        cout << rpw1 << endl;

//        Matrix3f ntemp;
//        ntemp = AngleAxisf (ea[0], Vector3f::UnitX ()) * AngleAxisf (ea[1], Vector3f::UnitY ()) * AngleAxisf (ea[2], Vector3f::UnitZ ());
//        cout << "Recall original rotation:" << endl;
//        cout << ntemp << endl;

        // Method 2: use rot2euler -- need to test
        Mat euler_angle = rot2euler (rotation_diff);
//        cout << "My new Euler angles: " << endl;
//        cout << euler_angle << endl << endl;
        float roll2 = euler_angle.at<double> (0) * RAD2DEG;
        float pitch2 = euler_angle.at<double> (1) * RAD2DEG;
        float yaw2 = euler_angle.at<double> (2) * RAD2DEG;
        string sroll2 = FloatToString (roll2);
        string spitch2 = FloatToString (pitch2);
        string syaw2 = FloatToString (yaw2);
        string rpw2 = "R: " + sroll2 + " P: " + spitch2 + " Y: " + syaw2;
        cout << rpw2 << endl;

        drawText5 (frame_rgb_vis, rpw1, yellow);
        drawText6 (frame_rgb_vis, rpw2, green);

#endif

#if(1) // count time and ratio
        // see how much time has elapsed
        time (&end);
        // calculate current FPS
        ++counter;
        //sec = difftime (end, start);
        //fps = counter / sec;
        //drawFPS (frame_rgb_vis, fps, yellow);  // frame ratio

        double detection_ratio = ((double) inliers_idx.rows / (double) good_matches.size ()) * 100;
        cout << "Detection ratio: " << detection_ratio << endl;
        drawConfidence (frame_rgb_vis, detection_ratio, yellow);
#endif

#if(1) // save root matrix if testID = save_root_file_ID
        // save root rotation matrix
        if (save_root_file_ID == testID)
        {
          cout << "Saving root rotation ... ";
          RootFrame root_frame_save;

          root_frame_save.setRootRotation (rotation_estimated);
          root_frame_save.save (save_root_file_path);
          cout << "done!" << endl;
          cout << "Rotation matrix was saved: " << endl;
          cout << rotation_estimated << endl;
        }
#endif

#if(1) // debug text
        int inliers_int = inliers_idx.rows;
        int outliers_int = (int) good_matches.size () - inliers_int;
        string inliers_str = IntToString (inliers_int);
        string outliers_str = IntToString (outliers_int);
        string n = IntToString ((int) good_matches.size ());
        string text = "Found " + inliers_str + " of " + n + " matches";
        string text2 = "Inliers: " + inliers_str + " - Outliers: " + outliers_str;

        drawText (frame_rgb_vis, text, green);
        drawText2 (frame_rgb_vis, text2, red);

        // draw frameID
        drawText3 (frame_rgb_vis, "Frame: " + testID, yellow);
        drawText4 (frame_rgb_vis, "Descriptor: " + objectID, red);

        // show image
        imshow ("2D", frame_rgb_vis);
        imshow ("Depth", frame_depth_vis);
#endif

#if(1) // publish message
//        ros::init(argc, argv, "pre_grasp_talker");
//        ros::NodeHandle node_handle;
//        ros::Publisher pre_grasp_pub = node_handle.advertise<std_msgs::String>("pre_grasp_data", 1000);
//
//        while(ros::ok())
//        {
//          std_msgs::String msg;
//          msg.data = "0 1 2 3 4 5 6";
//          pre_grasp_pub.publish(msg);
//          ros::spinOnce(); // no need here bc no callback()
//          cout << "Publish done! Message: " << msg.data << endl;
//        }

#endif

#if(1) // planning
//
//        // plan a motion to a desired pose for the end-effector.
//          geometry_msgs::Pose target_pose1;
//        //  target_pose1.orientation.x = 0.7071;
//        //  target_pose1.orientation.y = 0.0;
//        //  target_pose1.orientation.z = 0.7071;
//        //  target_pose1.orientation.w = 0.0;
//
//        //  target_pose1.orientation.x = 1.0;
//        //  target_pose1.orientation.y = 0.0;
//        //  target_pose1.orientation.z = 0.0;
//        //  target_pose1.orientation.w = 0.0;
//
//          Eigen::Quaternion<float> qua1 = Eigen::Quaternion<float> (Eigen::AngleAxis<float> (-90 * DEG2RAD, Eigen::Vector3f::UnitY ()));
//          std::cout << std::endl << "QUATERNION 1: " << "x= " << qua1.x () << " y= " << qua1.y () << " z= " << qua1.z () << " w= " << qua1.w () << std::endl
//              << std::endl;
//          target_pose1.orientation.w = qua1.w ();
//
//          target_pose1.orientation.x = qua1.x ();
//          target_pose1.orientation.y = qua1.y ();
//          target_pose1.orientation.z = qua1.z ();
//
//          target_pose1.position.x = 0.5;
//          target_pose1.position.y = -0.4;
//          target_pose1.position.z = 0.15;
//
//          group.setPoseTarget (target_pose1);
//
//          // call the planner
//        //  while (1)
//        //  {
//        //    success = group.plan (my_plan);
//        //    ROS_INFO("Visualizing plan: 1st plan %s", success ? "" : "FAILED");
//        //  }
//
//          success = group.plan (my_plan);
//          //ROS_INFO("Visualizing plan: 1st plan %s", success ? "" : "FAILED");
//
//          if (success)
//            cout << "Visualizing plan for " << testID << " - Result: SUCCESS " << endl;
//          else
//            cout << "Visualizing plan for " << testID << " - Result: FAILED " << endl;
//
//          cout << ".... Sleep for 5 seconds now ...." << endl;
//          sleep(5);

#endif

#if (1) //

#endif
        cout << "Found in " << iDes << " loop." << endl;
        cout << descLoop;
        break;
      }
      else  // not found good estimation --> save score to choose the best later --> NOT OK YET
      {
        double detection_ratio = ((double) inliers_idx.rows / (double) good_matches.size ()) * 100;
        //cout << "Inline: " << inliers_idx.rows << endl;
        //cout << "Good matches: " << good_matches.size() << endl;
        //cout << "Detection ratio: " << detection_ratio << endl;
        // save detection score and object
        detection_score.push_back (detection_ratio);
        detection_object.push_back (pnp_detection_est);

        //drawObjectMesh (frame_rgb_vis, &mesh, &pnp_detection_est, yellow);  // draw estimated pose
        cout << "Not found in this loop" << endl;
      }

      if (iDes == listDescriptorTrain.size () - 1)  // can't find good pose through database
      {
        cout << descLoop;
        cout << "CAN NOT FIND ANY GOOD POSE!" << endl;
        // find the best score
        double score_max = detection_score[0];
        unsigned int score_id = 0;
        for (unsigned int idetect = 1; idetect < detection_score.size (); ++idetect)
        {
          if (score_max < detection_score[idetect])
          {
            score_max = detection_score[idetect];
            score_id = idetect;
          }
        }

        cout << "Choose alternative pose, ID: " << score_id << endl;
        drawObjectMesh (frame_rgb_vis, &mesh, &pnp_detection_est, yellow);
      }

      cout << descLoop;

    }



    cout << mainLoop;
    char waitKey;
    waitKey = cvvWaitKey(0);
    if (waitKey == 27)  // press "ECS" to break
      break;

  }

  // Close and Destroy Window
  destroyWindow ("VIDEO");

  cout << "All done!!!" << endl;

}

/**********************************************************************************************************/
void
help ()
{
  //cout << "--------------------------------------------------------------------------" << endl;
}

/**********************************************************************************************************/

Mat
computeDiffOrientation (Mat rotation_root,
                        Mat rotation_est)
{
  Mat rotation_diff = rotation_root.t () * rotation_est;
  return rotation_diff;

}

/**********************************************************************************************************/
Point3f
calculate3DWorld (Point2f point2d,
                  float zw)
{
  Point3f point3d;

  point3d.x = (point2d.x - params_WEBCAM[2]) * zw / params_WEBCAM[0];
  point3d.y = (point2d.y - params_WEBCAM[3]) * zw / params_WEBCAM[1];
  point3d.z = zw;

  return point3d;
}

/**********************************************************************************************************/
int
searchDepthPoint (vector<Point2f> posePoints,
                  Mat frame,
                  const int maxDistance,
                  Point2f & root2D)
{
  int depth_Val;
  //for (unsigned count=0; count<posePoints.size(); ++count)
  for (unsigned count = 0; count < 1; ++count)
  {
    cv::Point2i point_Orgin;
    point_Orgin.x = (int) posePoints[count].x;  // not safe
    point_Orgin.y = (int) posePoints[count].y;

    cv::Point2i point_Top;
    point_Top.x = point_Orgin.x - maxDistance / 2;
    point_Top.y = point_Orgin.y - maxDistance / 2;

    //draw2DSinglePoint(frame, point_Top, yellow);
    //imshow("Deep Temp", frame);

    //depth_Val = frame.at<unsigned short> (point_Orgin.x, point_Orgin.y);
    depth_Val = frame.at<unsigned int> (point_Orgin.x, point_Orgin.y);
    if (depth_Val > nearDis && depth_Val < farDis)
      return depth_Val;
    else
    {
      //unsigned short dep_Matrix[maxDistance][maxDistance];
      unsigned int dep_Matrix[maxDistance][maxDistance];
      vector<unsigned short> vec_Depth;
      float min_Close = 999999999;  // Euclidean distance between points
      float current_Close = min_Close + 1;
      root2D.x = 0, root2D.y = 0;

      for (int i = 0; i < maxDistance; ++i)
      {
        for (int j = 0; j < maxDistance; ++j)
        {
          //dep_Matrix[i][j] = frame.at<unsigned short> (point_Top.x + i, point_Top.y + j);
          dep_Matrix[i][j] = frame.at<unsigned int> (point_Top.x + i, point_Top.y + j);
          //cout << dep_Matrix[i][j] << " ";

          if (dep_Matrix[i][j] > nearDis && dep_Matrix[i][j] < farDis)
          {
            current_Close = distanceToCenter (i, j, maxDistance / 2, maxDistance / 2);
            if (min_Close > current_Close)
            {
              min_Close = current_Close;
              root2D.x = i + point_Top.x;
              root2D.y = j + point_Top.y;
              depth_Val = dep_Matrix[i][j];
            }
          }
        }
        //cout << endl;
      }

      if (depth_Val != 0 && depth_Val > nearDis && depth_Val < farDis)
      {
        cout << "New Original Point: " << " X=" << root2D.x << " Y=" << root2D.y << endl;
        cout << "Difference: " << " x=" << root2D.x - point_Orgin.x << " y=" << root2D.y - point_Orgin.y << endl;
        cout << "Depth Value: " << depth_Val << endl;
        ;
      }
      else
        cout << "Can't find depth point!" << endl;
    }
  }

  return depth_Val;
}

float
distanceToCenter (int x,
                  int y,
                  int cx,
                  int cy)
{
  float dis = (x - cx) * (x - cx) + (y - cy) * (y - cy);
  return dis;
}

/**********************************************************************************************************/
void
initKalmanFilter (KalmanFilter &KF,
                  int nStates,
                  int nMeasurements,
                  int nInputs,
                  double dt)
{

  KF.init (nStates, nMeasurements, nInputs, CV_64F);      // init Kalman Filter

  setIdentity (KF.processNoiseCov, Scalar::all (1e-5));     // set process noise
  setIdentity (KF.measurementNoiseCov, Scalar::all (1e-2));  // set measurement noise
  setIdentity (KF.errorCovPost, Scalar::all (1));            // error covariance

  /** DYNAMIC MODEL **/

  //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
  // position
  KF.transitionMatrix.at<double> (0, 3) = dt;
  KF.transitionMatrix.at<double> (1, 4) = dt;
  KF.transitionMatrix.at<double> (2, 5) = dt;
  KF.transitionMatrix.at<double> (3, 6) = dt;
  KF.transitionMatrix.at<double> (4, 7) = dt;
  KF.transitionMatrix.at<double> (5, 8) = dt;
  KF.transitionMatrix.at<double> (0, 6) = 0.5 * pow (dt, 2);
  KF.transitionMatrix.at<double> (1, 7) = 0.5 * pow (dt, 2);
  KF.transitionMatrix.at<double> (2, 8) = 0.5 * pow (dt, 2);

  // orientation
  KF.transitionMatrix.at<double> (9, 12) = dt;
  KF.transitionMatrix.at<double> (10, 13) = dt;
  KF.transitionMatrix.at<double> (11, 14) = dt;
  KF.transitionMatrix.at<double> (12, 15) = dt;
  KF.transitionMatrix.at<double> (13, 16) = dt;
  KF.transitionMatrix.at<double> (14, 17) = dt;
  KF.transitionMatrix.at<double> (9, 15) = 0.5 * pow (dt, 2);
  KF.transitionMatrix.at<double> (10, 16) = 0.5 * pow (dt, 2);
  KF.transitionMatrix.at<double> (11, 17) = 0.5 * pow (dt, 2);

  /** MEASUREMENT MODEL **/

  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
  KF.measurementMatrix.at<double> (0, 0) = 1;  // x
  KF.measurementMatrix.at<double> (1, 1) = 1;  // y
  KF.measurementMatrix.at<double> (2, 2) = 1;  // z
  KF.measurementMatrix.at<double> (3, 9) = 1;  // roll
  KF.measurementMatrix.at<double> (4, 10) = 1;  // pitch
  KF.measurementMatrix.at<double> (5, 11) = 1;  // yaw

}

/**********************************************************************************************************/
void
updateKalmanFilter (KalmanFilter &KF,
                    Mat &measurement,
                    Mat &translation_estimated,
                    Mat &rotation_estimated)
{

  // First predict, to update the internal statePre variable
  Mat prediction = KF.predict ();

  // The "correct" phase that is going to use the predicted value and our measurement
  Mat estimated = KF.correct (measurement);

  // Estimated translation
  translation_estimated.at<double> (0) = estimated.at<double> (0);
  translation_estimated.at<double> (1) = estimated.at<double> (1);
  translation_estimated.at<double> (2) = estimated.at<double> (2);

  // Estimated euler angles
  Mat eulers_estimated (3, 1, CV_64F);
  eulers_estimated.at<double> (0) = estimated.at<double> (9);
  eulers_estimated.at<double> (1) = estimated.at<double> (10);
  eulers_estimated.at<double> (2) = estimated.at<double> (11);

  // Convert estimated quaternion to rotation matrix
  rotation_estimated = euler2rot (eulers_estimated);

}

/**********************************************************************************************************/
void
fillMeasurements (Mat &measurements,
                  const Mat &translation_measured,
                  const Mat &rotation_measured)
{
  // Convert rotation matrix to euler angles
  Mat measured_eulers (3, 1, CV_64F);
  measured_eulers = rot2euler (rotation_measured);

  // Set measurement to predict
  measurements.at<double> (0) = translation_measured.at<double> (0);  // x
  measurements.at<double> (1) = translation_measured.at<double> (1);  // y
  measurements.at<double> (2) = translation_measured.at<double> (2);  // z
  measurements.at<double> (3) = measured_eulers.at<double> (0);      // roll
  measurements.at<double> (4) = measured_eulers.at<double> (1);      // pitch
  measurements.at<double> (5) = measured_eulers.at<double> (2);      // yaw
}
