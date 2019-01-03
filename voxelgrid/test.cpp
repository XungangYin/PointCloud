#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
int main(int argc,char **argv)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud);
        cout << "cloud size is " << cloud->size()<<endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr out (new pcl::PointCloud<pcl::PointXYZ>);

/*体素网格滤波*/

        //  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_grid; //这个算法是以格子的中心代替格子中所有点，属于估计的
        pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_grid; //以格子中点的平均值作为简化的点（相比较而言，精确）

        approximate_voxel_grid.setLeafSize(25,25,25); //这里的数值越大，则精简的越厉害（剩下的数据少）
        approximate_voxel_grid.setInputCloud(cloud);
        approximate_voxel_grid.filter(*out);
        cout << "voxel grid  Filte cloud size is " << out->size()<<endl;
        pcl::io::savePCDFile("voxelgrid.pcd",*out);
        
        
        /*
        //直通滤波器
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-100,0.0);
        pass.filter(*out);
        */

        pcl::visualization::CloudViewer viewer("滤波");
        viewer.showCloud(out);
        //  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample cloud");
        
        while(!viewer.wasStopped())
        {
        }
        return 0;
}
