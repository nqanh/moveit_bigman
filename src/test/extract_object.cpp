#include "iostream"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudXYZRGBA;

void
printElapsedTimeAndNumberOfPoints (double t,
                                   int w,
                                   int h = 1)
{
  print_info ("[done, ");
  print_value ("%g", t);
  print_info (" ms : ");
  print_value ("%d", w * h);
  print_info (" points]\n");
}

bool
loadCloud (const std::string & filename,
           PointCloudXYZRGBA & cloud)
{
  TicToc tt;
  print_highlight ("Loading ");
  print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);

  printElapsedTimeAndNumberOfPoints (tt.toc (), cloud.width, cloud.height);

  print_info ("Available dimensions: ");
  print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

int
main (int argc,
      char** argv)
{
  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  PointCloudXYZRGBA::Ptr input_cloud (new PointCloudXYZRGBA);
  PointCloudXYZRGBA::Ptr inlierPoints (new PointCloudXYZRGBA);

  //if (!loadCloud (argv[1], *input_cloud))
  if (!loadCloud (argv[1], *input_cloud))
  {
    print_error ("Error loading input cloud");
    return (-1);
  }

  float min_z = 0;
  parse_argument (argc, argv, "-min_z", min_z);

  float max_z = 1;
  parse_argument (argc, argv, "-max_z", max_z);

  float min_x = 0;
  parse_argument (argc, argv, "-min_x", min_x);

  float max_x = 1;
  parse_argument (argc, argv, "-max_x", max_x);
  ;

  float min_y = 0;
  parse_argument (argc, argv, "-min_y", min_y);

  float max_y = 1;
  parse_argument (argc, argv, "-max_y", max_y);
  ;

  PassThrough<pcl::PointXYZRGBA> filter;
  filter.setInputCloud (input_cloud);
  filter.setKeepOrganized (true);

  filter.setFilterFieldName ("z");
  filter.setFilterLimits (min_z, max_z);
  filter.filter (*input_cloud);

  filter.setFilterFieldName ("x");
  filter.setFilterLimits (min_x, max_x);
  filter.filter (*input_cloud);

  filter.setFilterFieldName ("y");
  filter.setFilterLimits (min_y, max_y);
  filter.filter (*input_cloud);

  cout << "Cloud size after filtering: " << input_cloud->size () << endl;
  cout << "Cloud width               : " << input_cloud->width << endl;
  cout << "Cloud height              : " << input_cloud->height << endl;

  pcl::io::savePCDFileASCII ("filter_cloud.pcd", *input_cloud);

  /*
   // Plane segmentation
   pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients);
   pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
   seg.setInputCloud(input_cloud);
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setDistanceThreshold(0.01);
   seg.setOptimizeCoefficients(true);

   pcl::PointIndices inlierIndices;
   seg.segment(inlierIndices, *coef);

   if (inlierIndices.indices.size() == 0)
   cout << "Can't file any points that fitted the plane model" << std::endl;
   else
   {
   cerr << "Model coefficients: " << coef->values[0] << " "
   << coef->values[1] << " "
   << coef->values[2] << " "
   << coef->values[3] << endl;

   pcl::copyPointCloud<pcl::PointXYZRGBA> (*input_cloud, inlierIndices, *inlierPoints);


   }
   pcl::io::savePCDFileASCII("object.pcd", *inlierPoints);
   */

  /*
   // Objects for storing the point clouds.
   PointCloudXYZRGBA::Ptr cloud(new PointCloudXYZRGBA);
   PointCloudXYZRGBA::Ptr plane(new PointCloudXYZRGBA);
   PointCloudXYZRGBA::Ptr convexHull(new PointCloudXYZRGBA);
   PointCloudXYZRGBA::Ptr objects(new PointCloudXYZRGBA);

   if (!loadCloud ("coke1_filter_cloud.pcd", *cloud))
   {
   print_error("Error loading input cloud");
   return (-1);
   }

   float t = 0.01;
   parse_argument (argc, argv, "-t", t);

   float h1 = 0.01;
   parse_argument (argc, argv, "-h1", h1);

   float h2 = 0.15;
   parse_argument (argc, argv, "-h2", h2);

   // Get the plane model, if present.
   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
   pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation;
   segmentation.setInputCloud(cloud);
   segmentation.setModelType(pcl::SACMODEL_PLANE);
   segmentation.setMethodType(pcl::SAC_RANSAC);
   segmentation.setDistanceThreshold(t);
   segmentation.setOptimizeCoefficients(true);
   //segmentation.set
   pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
   segmentation.segment(*planeIndices, *coefficients);

   if (planeIndices->indices.size() == 0)
   std::cout << "Could not find a plane in the scene." << std::endl;
   else
   {
   // Copy the points of the plane to a new cloud.
   pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
   extract.setInputCloud(cloud);
   extract.setIndices(planeIndices);
   extract.filter(*plane);

   // Retrieve the convex hull.
   pcl::ConvexHull<pcl::PointXYZRGBA> hull;
   hull.setInputCloud(plane);

   // Make sure that the resulting hull is bidimensional.
   hull.setDimension(2);
   hull.reconstruct(*convexHull);

   // Redundant check.
   if (hull.getDimension() == 2)
   {
   // Prism object.
   pcl::ExtractPolygonalPrismData<pcl::PointXYZRGBA> prism;
   prism.setInputCloud(cloud);
   prism.setInputPlanarHull(convexHull);
   // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
   // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
   prism.setHeightLimits(h1, h2);
   pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

   prism.segment(*objectIndices);

   // Get and show all points retrieved by the hull.
   extract.setIndices(objectIndices);
   extract.filter(*objects);

   pcl::visualization::CloudViewer viewerObjects("Objects on table");
   viewerObjects.showCloud(objects);
   while (!viewerObjects.wasStopped())
   {
   // Do nothing but wait.
   }

   pcl::io::savePCDFileASCII("object_only.pcd", *objects);
   }
   else std::cout << "The chosen hull is not planar." << std::endl;
   }
   */
  return 1;
}
