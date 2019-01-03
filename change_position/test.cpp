#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
using namespace std;

int main (int argc, char **argv)
{
        if (argc == 0)
        {
                cout<<"终端输入点云"<<endl;
                return -1;
        }
        
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (argc = 0)
        {
                cout<<"请在终端输入点云"<<endl;
                return -1;
        }
        
        pcl::io::loadPCDFile(argv[1],*cloud);
        
        /*...首先对全局点云进行变化..*/
        //构造变化矩阵
        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        float theta = M_PI/4;   //旋转的度数，这里是45度
        transform_1 (0,0) = cos (theta);  //这里是绕的Z轴旋转
        transform_1 (0,1) = -sin(theta);
        transform_1 (1,0) = sin (theta);
        transform_1 (1,1) = cos (theta);
        //   transform_1 (0,2) = 0.3;   //这样会产生缩放效果
        //   transform_1 (1,2) = 0.6;
        //    transform_1 (2,2) = 1;
        

        transform_1 (0,3) = 25; //这里沿X轴平移
        transform_1 (1,3) = 30;
        transform_1 (2,3) = 380;
        
        printf ("Method #1: transform  Matrix4f\n");
        std::cout << transform_1 << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud,*transform_cloud1,transform_1);
        

        pcl::io::savePCDFile("change_simply_mid.pcd",*transform_cloud1);


        return 0;

}
