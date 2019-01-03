#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
int
main (int argc, char** argv)   
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) 
        {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                return (-1);
        }
        std::cout << "Loaded "
                  << cloud->width * cloud->height
                  << " data points from .pcd with the following fields "<< std::endl;
        
#if 0
        for (size_t i = 0; i < cloud->points.size (); ++i)
                std::cout << "    " << cloud->points[i].x
                          << " "    << cloud->points[i].y
                          << " "    << cloud->points[i].z << std::endl;

          pcl::visualization::CloudViewer viewer("test");
          viewer.showCloud(cloud);
          //  viewer.spin(0);
          while (!viewer.wasStopped()){ }
#endif
          return (0);
}
