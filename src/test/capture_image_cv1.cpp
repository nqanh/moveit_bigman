//#include "opencv2/highgui/highgui.hpp"
//#include <opencv2/core/core.hpp>
//#include <iostream>
//
//using namespace cv;
//using namespace std;
//
//int
//main (int argc,
//      char* argv[])
//{
//  // Open Kinect sensor
//  cv::VideoCapture cap (CV_CAP_OPENNI);
//  if (!cap.isOpened ())
//  {
//    printf ("Could not open OpenNI-capable sensor\n");
//    return -1;
//  }
//  printf ("Open sensor done!\n");
//
////  cap.set (CV_CAP_PROP_OPENNI_REGISTRATION, 1);
//
////
////  VideoCapture cap (0);  // open the video camera no. 0
////
////  if (!cap.isOpened ())  // if not success, exit program
////  {
////    cout << "ERROR: Cannot open the video file" << endl;
////    return -1;
////  }
//
//
////  namedWindow ("MyVideo", CV_WINDOW_AUTOSIZE);  //create a window called "MyVideo"
//
////  double dWidth = cap.get (CV_CAP_PROP_FRAME_WIDTH);  //get the width of frames of the video
////  double dHeight = cap.get (CV_CAP_PROP_FRAME_HEIGHT);  //get the height of frames of the video
//
////  double dWidth = 640.0;
////  double dHeight = 480.0;
////  cout << "Frame Size = " << dWidth << "x" << dHeight << endl;
////
////  Size frameSize (static_cast<int> (dWidth), static_cast<int> (dHeight));
////
////  VideoWriter oVideoWriter ("MyVideo.avi", CV_FOURCC ('P', 'I', 'M', '1'), 20, frameSize, true);  //initialize the VideoWriter object
////
////  if (!oVideoWriter.isOpened ())  //if not initialize the VideoWriter successfully, exit the program
////  {
////    cout << "ERROR: Failed to write the video" << endl;
////    return -1;
////  }
////  else
////    cout << "Loading video writer ..." << endl;
//
//  cv::namedWindow ("frame");
//
//  while (1)
//  {
//
//    cout << "Starting ...\n";
//    Mat frame;
//
//    Mat depthMap;
//    Mat rgbImage;
//
//    cap.grab();
//
//    cap.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );
//    cap.retrieve( rgbImage, CV_CAP_OPENNI_BGR_IMAGE );
//
//    cout << "done 1 frame" << endl;
//
//    cv::Mat display = rgbImage.clone ();
//
//    cv::imshow("frame", display);
//
////
////    if (!bSuccess)  //if not success, break loop
////    {
////      cout << "ERROR: Cannot read a frame from video file" << endl;
////      break;
////    }
////    else
////      cout << "Captured 1 frame" << endl;
////
////    oVideoWriter.write (frame);  //writer the frame into the file
////
////    imshow ("MyVideo", frame);  //show the frame in "MyVideo" window
////
////    if (waitKey (10) == 27)  //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
////    {
////      cout << "esc key is pressed by user" << endl;
////      break;
////    }
////    else
////      cout << "Done 1 frame" << endl;
//
//  }
//
//  cout << "All done!" << endl;
//  return 0;
//}
//

//
//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//
//#include <iostream>
//
//using namespace cv;
//using namespace std;
//
//int main(){
//    cout << "opening device(s)" << endl;
//
//    VideoCapture sensor1;
//    sensor1.open(CV_CAP_OPENNI);
//
//    if( !sensor1.isOpened() ){
//        cout << "Can not open capture object 1." << endl;
//        return -1;
//    }
//
//    for(;;){
//        Mat depth1;
//
//        if( !sensor1.grab() ){
//            cout << "Sensor1 can not grab images." << endl;
//            return -1;
//        }else if( sensor1.retrieve( depth1, CV_CAP_OPENNI_DEPTH_MAP ) ) imshow("depth1",depth1);
//
//        if( waitKey( 30 ) == 27 )   break;//ESC to exit
//
//   }
//}

//
//#include "opencv2/highgui/highgui.hpp"
//#include <iostream>
//
//using namespace cv;
//using namespace std;
//
//int main(int argc, char* argv[])
//{
//    VideoCapture cap(0); // open the video camera no. 0
//
//    if (!cap.isOpened())  // if not success, exit program
//    {
//        cout << "Cannot open the video cam" << endl;
//        return -1;
//    }
//
//   double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
//   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
//
//    cout << "Frame size : " << dWidth << " x " << dHeight << endl;
//
//    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
//
//    while (1)
//    {
//        Mat frame;
//
//        bool bSuccess = cap.read(frame); // read a new frame from video
//
//        if (!bSuccess) //if not success, break loop
//        {
//             cout << "Cannot read a frame from video stream" << endl;
//             break;
//        }
//
//        imshow("MyVideo", frame); //show the frame in "MyVideo" window
//
//        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
//       {
//            cout << "esc key is pressed by user" << endl;
//            break;
//       }
//    }
//
//    cout << "All done!" << endl;
//    return 0;
//
//}
//

#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;
char key;
int
main ()
{
  cout << cv::getBuildInformation ();

  cvNamedWindow ("Camera_Output", 1);    //Create window
  CvCapture* capture = cvCaptureFromCAM (CV_CAP_ANY);  //Capture using any camera connected to your system
  while (1)
  {  //Create infinte loop for live streaming

    IplImage* frame = cvQueryFrame (capture);  //Create image frames from capture
    cvShowImage ("Camera_Output", frame);   //Show image frames on created window
    key = cvWaitKey (10);     //Capture Keyboard stroke
    if (char (key) == 27)
    {
      break;      //If you hit ESC key loop will break.
    }
  }
  cvReleaseCapture (&capture);  //Release capture.
  cvDestroyWindow ("Camera_Output");  //Destroy Window
  return 0;
}

//

//
//
//

/*
 #include "opencv2/highgui/highgui.hpp"
 #include <opencv2/core/core.hpp>
 #include <iostream>
 #include <stdio.h>

 using namespace cv;
 using namespace std;

 int
 main (int argc,
 char* argv[])
 {
 // Open Kinect sensor
 cv::VideoCapture cap (CV_CAP_OPENNI);
 //cv::VideoCapture cap (0);
 if (!cap.isOpened ())
 {
 printf ("Could not open OpenNI-capable sensor\n");
 return -1;
 }


 printf ("Open sensor done!\n");

 double focal_length = cap.get (CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
 printf ("Focal length = %f\n", focal_length);

 //  cv::namedWindow ("frame");
 //
 //  while (1)
 //  {
 //
 //    cout << "Starting ...\n";
 //    Mat frame;
 //
 //    Mat depthMap;
 //    Mat rgbImage;
 //
 //    cap.grab();
 //
 //    cap.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );
 //    cap.retrieve( rgbImage, CV_CAP_OPENNI_BGR_IMAGE );
 //
 //    cout << "done 1 frame" << endl;
 //
 //    cv::Mat display = rgbImage.clone ();
 //
 //    cv::imshow("frame", display);
 //
 //
 //  }

 cout << "All done!" << endl;
 return 0;
 }

 */

/*
 #include "opencv2/opencv.hpp"
 #include <iostream>

 using namespace std;
 using namespace cv;

 int main(){

 VideoCapture vcap(0); 
 if(!vcap.isOpened()){
 cout << "Error opening video stream or file" << endl;
 return -1;
 }

 
 cout << "Starting ..." << endl;  
 int frame_width=   vcap.get(CV_CAP_PROP_FRAME_WIDTH);
 int frame_height=   vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
 cout << "frame width:  " << frame_width << endl;
 cout << "frame height: " << frame_height << endl;
 
 VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);
 
 int i=0;
 
 
 for(;;){

 Mat frame;
 vcap >> frame;
 video.write(frame);
 imshow( "Frame", frame );
 if (i<10)
 {
 ostringstream convert;
 convert << i;
 string name = "image_" + convert.str() + ".jpg";
 imwrite(name, frame);
 i++;
 }
 
 char c = (char)waitKey(33);
 if( c == 27 ) break;
 }
 return 0;
 }

 */

/*
 #include <iostream>
 #include <cv.h>
 #include <highgui.h>
 #include "opencv2/opencv.hpp"

 using namespace std;
 using namespace cv;

 char key;
 int main()
 {
 cvNamedWindow("Camera_Output", 1);    //Create window
 CvCapture* capture = cvCaptureFromCAM(0);  //Capture using any camera connected to your system

 VideoWriter video("out1.avi",CV_FOURCC('M','J','P','G'),10, Size(640,480),true);
 
 
 while(1){ //Create infinte loop for live streaming

 IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
 cvShowImage("Camera_Output", frame);   //Show image frames on created window
 
 Mat mframe = cv::cvarrToMat(frame);
 
 video.write(mframe);
 
 key = cvWaitKey(10);     //Capture Keyboard stroke
 if (char(key) == 27){
 break;      //If you hit ESC key loop will break.
 }
 }
 cvReleaseCapture(&capture); //Release capture.
 cvDestroyWindow("Camera_Output"); //Destroy Window
 return 0;
 }

 */

/*
 #include <iostream>
 #include <cv.h>
 #include <highgui.h>

 using namespace std;
 char key;
 int main()
 {
 cvNamedWindow("Camera_Output", 1);    //Create window
 CvCapture* capture = cvCaptureFromCAM(0);  //Capture using any camera connected to your system

 while(1){ //Create infinte loop for live streaming

 IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
 cvShowImage("Camera_Output", frame);   //Show image frames on created window
 key = cvWaitKey(10);     //Capture Keyboard stroke
 if (char(key) == 27){
 break;      //If you hit ESC key loop will break.
 }
 }
 cvReleaseCapture(&capture); //Release capture.
 cvDestroyWindow("Camera_Output"); //Destroy Window
 return 0;
 }

 */

/*

 #include "opencv2/opencv.hpp"

 using namespace cv;
 using namespace std;

 int main(int, char**)
 {

 VideoCapture cap(0); // open the default camera
 if(!cap.isOpened())  // check if we succeeded
 cout << "Can't open device";
 return -1;

 Mat edges;
 namedWindow("edges",1);
 for(;;)
 {
 Mat frame;
 cap >> frame; // get a new frame from camera
 cvtColor(frame, edges, COLOR_BGR2GRAY);
 GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
 Canny(edges, edges, 0, 30, 3);
 imshow("edges", edges);
 if(waitKey(30) >= 0) break;
 }
 the camera will be deinitialized automatically in VideoCapture destructor
 return 0;
 }


 */
