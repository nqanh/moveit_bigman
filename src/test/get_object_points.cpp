#include "ros/ros.h"
#include "std_msgs/String.h"

#include <std_msgs/Header.h>

#include <iostream>

#include <ros/types.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>
#include <object_recognition_msgs/RecognizedObject.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <object_recognition_core/db/prototypes/object_info.h>
#include <object_recognition_core/common/pose_result.h>
#include <object_recognition_core/common/types.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace message_operations;

struct Point
{
    float x, y, z;
};

struct Object
{
    string key;
    string name;
    Point point;
    bool is_Found = false;
};

Object obj;

PointCloud<PointXYZRGBA>::Ptr cloudptr (new PointCloud<PointXYZRGBA>);  // A cloud that will store color info.
PointCloud<PointXYZ>::Ptr fallbackCloud (new PointCloud<PointXYZ>);    // A fallback cloud with just depth data.
boost::shared_ptr<visualization::CloudViewer> viewer;                 // Point cloud viewer object.
Grabber* openniGrabber;                                               // OpenNI grabber that takes data from the device.
unsigned int filesSaved = 0;                                          // For the numbering of the clouds saved to disk.
bool saveCloud (false), noColor (false);

class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () :
        viewer ("PCL OpenNI Viewer")
    {
    }

    void
    cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      pcl::io::savePCDFileASCII ("myData.pcd", *cloud);
      /*
       if (!viewer.wasStopped())
       {
       viewer.showCloud (cloud);
       pcl::io::savePCDFileASCII("myData.pcd", *cloud);
       }
       */
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber ();

      boost::function<void
      (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

      interface->registerCallback (f);
      interface->start ();

      while (!viewer.wasStopped ())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }

      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
};

void
grabberCallback (const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
  if (!viewer->wasStopped ())
    viewer->showCloud (cloud);

  if (saveCloud)
  {
    stringstream stream;
    stream << "inputCloud" << filesSaved << ".pcd";
    string filename = stream.str ();

    if (io::savePCDFileASCII (filename, *cloud) == 0)
    {
      filesSaved++;
      cout << "Saved " << filename << "." << endl;
    }
    else
      PCL_ERROR("Problem saving %s.\n", filename.c_str ());

    saveCloud = false;
  }
}

// For detecting when SPACE is pressed.
void
keyboardEventOccurred (const visualization::KeyboardEvent& event,
                       void* nothing)
{
  //if (event.getKeySym() == "space" && event.keyDown())
  cout << "in kb event ..." << endl;
  //if (obj.is_Found)
  saveCloud = true;
}

// Creates, initializes and returns a new viewer.
boost::shared_ptr<visualization::CloudViewer>
createViewer ()
{
  boost::shared_ptr<visualization::CloudViewer> v (new visualization::CloudViewer ("OpenNI viewer"));
  v->registerKeyboardCallback (keyboardEventOccurred);

  return (v);
}

void
chatterCallback (const object_recognition_msgs::RecognizedObjectArray::ConstPtr & msg)
{
  for (size_t i = 0; i < msg->objects.size (); ++i)
  {
    //std::vector<::sensor_msgs::PointCloud2> vpc;

    const object_recognition_msgs::RecognizedObject& object = msg->objects[i];
    const string space = "-----------------------------------------------------\n";

    obj.key = object.type.key;
    obj.point.x = object.pose.pose.pose.position.x;
    obj.point.y = object.pose.pose.pose.position.y;
    obj.point.z = object.pose.pose.pose.position.z;
    obj.is_Found = true;

    cout << space;
//    cout << obj.key << endl;

//    cout << "saving cloud ..." << endl;

    //SimpleOpenNIViewer v;
    //v.run ();

    cout << "Position: ";
    cout << " x=" << object.pose.pose.pose.position.x << " y=" << object.pose.pose.pose.position.y << " z=" << object.pose.pose.pose.position.z << endl;
//    cout << "y: " << object.pose.pose.pose.position.y << endl;
//    cout << "z: " << object.pose.pose.pose.position.z << endl;

    cout << "Orientation: ";
    cout << " x=" << object.pose.pose.pose.orientation.x << " y=" << object.pose.pose.pose.orientation.y << " z=" << object.pose.pose.pose.orientation.z
        << " w=" << object.pose.pose.pose.orientation.w << endl;

//
//    bool justVisualize(false);
//    string filename;
//    // First mode, open and show a cloud from disk.
//    if (justVisualize)
//    {
//        // Try with color information...
//        try
//        {
//            io::loadPCDFile<PointXYZRGBA>(filename.c_str(), *cloudptr);
//        }
//        catch (PCLException e1)
//        {
//            try
//            {
//                // ...and if it fails, fall back to just depth.
//                io::loadPCDFile<PointXYZ>(filename.c_str(), *fallbackCloud);
//            }
//            catch (PCLException e2)
//            {
//                //return -1;
//              cout << "Error" << endl;
//            }
//
//            noColor = true;
//        }
//
//        cout << "Loaded " << filename << "." << endl;
//        if (noColor)
//            cout << "This cloud has no RGBA color information present." << endl;
//        else cout << "This cloud has RGBA color information present." << endl;
//    }
//    // Second mode, start fetching and displaying frames from the device.
//    else
//    {
//        openniGrabber = new OpenNIGrabber();
//        if (openniGrabber == 0)
//            //return -1;
//          cout << "Error" << endl;
//        boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
//            boost::bind(&grabberCallback, _1);
//        openniGrabber->registerCallback(f);
//    }
//
//    viewer = createViewer();
//
//    if (justVisualize)
//    {
//        if (noColor)
//            viewer->showCloud(fallbackCloud);
//        else viewer->showCloud(cloudptr);
//    }
//    else openniGrabber->start();
//
//    // Main loop.
//    while (! viewer->wasStopped())
//        boost::this_thread::sleep(boost::posix_time::seconds(1));
//
//    if (! justVisualize)
//        openniGrabber->stop();
//
//

    /*
     cout << "Type:" << endl;
     cout << object.type.key << endl;
     cout << space;

     cout << "Confidence: " << endl;
     cout << object.confidence << endl;
     cout << space;

     cout << "Pose: " << endl;
     cout << "x: " << object.pose.pose.pose.position.x << endl;
     cout << "y: " <<  object.pose.pose.pose.position.y << endl;
     cout << "z: " <<  object.pose.pose.pose.position.z << endl;
     */
  }
}

int
main (int argc,
      char **argv)
{
  ros::init (argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe ("recognized_object_array", 1000, chatterCallback);

//  while(obj.is_Found == false)
//  {
//    cout << "Looking for object ..." << endl;
//    ros::spinOnce();
//  }

  ros::spin ();

  cout << "Done!!!" << endl;

  return 0;
}
