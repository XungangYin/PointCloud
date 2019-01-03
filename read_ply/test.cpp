#include <iostream>
#include <string>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;


int main (int argc,char **argv)
{
         // The point clouds we will be using
        PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
        PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
        PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
        
        pcl::PolygonMesh mesh;
        pcl::io::loadPLYFile(argv[1],mesh);
        pcl::fromPCLPointCloud2(mesh.cloud,*cloud_in);

        cout<<"the cloud size is:"<<cloud_in->size()<<endl;
        pcl::io::savePCDFile("new.pcd",*cloud_in);

        
        return 0;
}
