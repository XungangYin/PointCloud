#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ>  pointcloud;
int main(int argc, char *argv[])
{
    pointcloud::Ptr  cloud (new pointcloud);
    pcl::PointXYZ a, b, z;
    a.x = 0;
    a.y = 0;
    a.z = 0;
    b.x  = 5;
    b.y  = 8;
    b.z = 10;
    z.x = 4;
    z.y = 3;
    z.z = 20;
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer ("line Viewer"));
    view->setBackgroundColor(255,255,255);
    view->addLine<pcl::PointXYZ>(a,b,"line");
    view->addLine<pcl::PointXYZ> (a,z,255,0,0,"line1");
    view->addArrow<pcl::PointXYZ> (b,z,255,0,0,"a");
    

       while (!view->wasStopped ())
       {
           view->spinOnce(100);
           boost::this_thread::sleep (boost::posix_time::microseconds (100000));

       }

    return 0;
}
