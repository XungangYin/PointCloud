#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>                    //滤波相关类头文件
#include <pcl/segmentation/sac_segmentation.h>             //基于采样一致性分割类定义的头文件
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/passthrough.h>  

using namespace std;
int main (int argc,char **argv)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),  cloud_project (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(argv[1],*cloud);

        //创建存储模型系数的对象
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // 创建存贮内点的索引

        pcl::SACSegmentation<pcl::PointXYZ> seg;  //此类是利用采样一致性算法实现的分割,即创建一个分割器
        seg.setOptimizeCoefficients(true); //可选。这一步主要是优化模型系数

        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);  //设置距离阀值
        seg.setInputCloud(cloud);
        
        //seg.segment(inliers,*coefficients);
        seg.segment(*inliers,*coefficients);  //执行分割，并存储分割点的索引到inliers，所分割的模型系数coefficients.

        if(inliers->indices.size()==0)
        {
                PCL_ERROR("COULD NOT ESTIMATE A PLANe MODEL FOR THE GIVEN DATA");
                
        } 
        
        cerr<<"Model coefficients:\n"<<coefficients->values[0]<<"\n"
            <<coefficients->values[1]<<"\n"<<coefficients->values[2]<<"\n"<<coefficients->values[3]<<endl;  //打印模型参数。模型以 Ax+By+Cz+d = 0的形式

        cerr<<"Model inliers:"<<inliers->indices.size()<<endl; //输出分割的大小
        pcl::PointCloud<pcl::PointXYZ>::Ptr out( new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud,*inliers,*out); //将内点复制到out中

#if 0
        pcl::ProjectInliers<pcl::PointXYZ> proj;  //创建投影对象
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(out);
        proj.setModelCoefficients(coefficients);  //设置投影参数为平面分割是系数
        proj.filter(*cloud_project);  //投影

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);//存储提取多边形的点
        pcl::ConvexHull<pcl::PointXYZ> chull; //创建凸包对象
        //  chull.setInputCloud(cloud_project);
          chull.setInputCloud(cloud);
        //  chull.setAlpha(0.5);
          chull.reconstruct(*cloud_hull);   //求解凸包

#endif
        
        pcl::visualization::CloudViewer viewer("test");
        viewer.showCloud(out);
        while(!viewer.wasStopped())
        {
        }
        
        return 0;
}
