#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>
#include <object_recognition_msgs/RecognizedObject.h>

#include <pcl/io/pcd_io.h>

#include <pcl/conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/foreach.hpp>

#include <object_recognition_core/db/prototypes/object_info.h>
#include <object_recognition_core/common/pose_result.h>
#include <object_recognition_core/common/types.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace message_operations;

void
chatterCallback (const object_recognition_msgs::RecognizedObjectArray::ConstPtr & msg)
{

  for (size_t i = 0; i < msg->objects.size (); ++i)
  {
    //std::vector<::sensor_msgs::PointCloud2> vpc;

    const object_recognition_msgs::RecognizedObject& object = msg->objects[i];
    const string space = "----------------------\n";

    cout << "Object key: " << object.type.key << endl;
    cout << space;

    /*
     sensor_msgs::PointCloud2 pc2 = object.point_clouds[0];
     pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

     pcl::fromROSMsg(pc2, pcl_cloud);
     pcl::io::savePCDFileASCII("data.pcd", pcl_cloud);
     */

    cout << "Confidence: " << object.confidence << endl;
    cout << space;

//    cout << "Header:" << endl;
//    cout << object.header << endl;
//    cout << space;
//
//    cout << "Bounding contours:" << endl;
//    //cout << object.bounding_contours << endl;
//    cout << space;

//    cout << "Bounding box:" << endl;
//    cout << object.bounding_mesh << endl;
//    cout << space;

    cout << "Position: " << endl;
    cout << "x: " << object.pose.pose.pose.position.x << endl;
    cout << "y: " << object.pose.pose.pose.position.y << endl;
    cout << "z: " << object.pose.pose.pose.position.z << endl;

    cout << "Orientation: " << endl;
    cout << "x: " << object.pose.pose.pose.orientation.x << endl;
    cout << "y: " << object.pose.pose.pose.orientation.y << endl;
    cout << "z: " << object.pose.pose.pose.orientation.z << endl;
    cout << "w: " << object.pose.pose.pose.orientation.w << endl;

//
//     cout << "Point cloud:" << endl;
//     pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
//     pcl::fromROSMsg(object.point_clouds[0], pcl_cloud);
//     pcl::io::savePCDFileASCII("data.pcd", pcl_cloud);
//
//     vpc = object.point_clouds;
//     cout << vpc[0].header << endl;
//
//     cout << vpc[0].width << endl;
//     cout << vpc[0].height << endl;
//     cout << "DATA ----";
//
//
  }
}

int
main (int argc,
      char **argv)
{
  ros::init (argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe ("recognized_object_array", 1000, chatterCallback);
  ros::spin ();
  return 0;
}
