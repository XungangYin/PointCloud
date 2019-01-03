#include <iostream>
#include<pcl/io/pcd_io.h> //添加此头文件需配置CmakeList.txt
#include<pcl/visualization/pcl_visualizer.h>
using namespace std;

pcl::PointCloud::Ptr cloud(new pcl::PointCloud); //定义指针类型变量cloud并开辟空间进行初始化

pcl::visualization::PCLVisualizer viewer("PCL Viewer"); //pcl::visualization::PCLVisualizer在#include头文件下，添加此横代码，运行结果能短暂弹出PCL Viewer窗口

int ReadPCD(const std::string &file_name)
{
        pcl::PCDReader bbb;
    if(bbb.read(file_name, *cloud) == -1)
    {
        PCL_ERROR("Cloudn't read file 521_test_pcd.pcd\n");
        return(-1);
    }
}

int main()
{
    ReadPCD("/home/yxg/pcl/pcd/capture00001.pcd");

    viewer.setCameraPosition(0,0,-200,0,0,0); //设置坐标原点

    viewer.addCoordinateSystem(200,0,0,0,0); //建立空间直角坐标系

while (!viewer.wasStopped()) //添加while空循环之后，运行结果弹出并保持PCL Viewer窗口
    {
        //用于显示点云
        viewer.removeAllPointClouds();
        viewer.removeAllShapes();
        pcl::visualization::PointCloudColorHandlerRGBField rgb(cloud);
        viewer.addPointCloud (cloud, rgb, "aaaaaa");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "aaaaaa");
        viewer.spinOnce();

    }
}
