#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include <pcl/segmentation/min_cut_segmentation.h>

int
main (int argc,
      char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("min_cut_segmentation_tutorial.pcd", *cloud) == -1 )
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  float x, y, z;
  x = -0.231512;
  y = 0.0680692;
  z = 0.730932;
  float padding = 0.05;  // padding 0.1 to left/right; top/down, etc. --> total: 0.2

  pcl::IndicesPtr indices (new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setKeepOrganized (true);

//  pass.setFilterFieldName ("x");
//  pass.setFilterLimits (x-padding, x+padding);
//  pass.filter (*indices);
//
//  pass.setFilterFieldName ("y");
//  pass.setFilterLimits (y-padding, y+padding);
//  pass.filter (*indices);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z - padding, z + padding);
  pass.filter (*indices);

  pcl::MinCutSegmentation<pcl::PointXYZ> seg;
  seg.setInputCloud (cloud);
  seg.setIndices (indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointXYZ point;  // known center point of object
  point.x = x;
  point.y = y;
  point.z = z;
  foreground_points->points.push_back (point);
  seg.setForegroundPoints (foreground_points);

  seg.setSigma (0.25);
  seg.setRadius (0.1);
  seg.setNumberOfNeighbours (8);
  seg.setSourceWeight (0.5);

  std::vector<pcl::PointIndices> clusters;
  seg.extract (clusters);

//  std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

  //save the cluster
//  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  object_cloud->width = clusters.size();
//  object_cloud->height = 1;
//  for (int i=0; i<clusters.size(); ++i)
//  {
//    object_cloud->points[i] = clusters;
//  }

  pcl::PCDWriter writer;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]);  //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);  //*
    j++;
  }

//  pcl::io::savePCDFileASCII("object_cloud.pcd", *object_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud (colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}
