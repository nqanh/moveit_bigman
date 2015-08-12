#include <pcl/io/pcd_io.h>

int
main (int argc,
      char** argv)
{
  // Object for storing the point cloud.
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

  // Read a PCD file from disk.
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) != 0)
  {
    return -1;
  }

  // Write it back to disk under a different name.
  // Another possibility would be "savePCDFileBinary()".
  pcl::io::savePCDFileASCII ("output.pcd", *cloud);
}
