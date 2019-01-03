#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
int main(int argc, char** argv)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(argv[1],*cloud);

         //创建存储模型系数的对象
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // 创建存贮内点的索引
        pcl::SACSegmentation<pcl::PointXYZ> seg;  //此类是利用采样一致性算法实现的分割,即创建一个分割器
        seg.setOptimizeCoefficients(true); //可选。这一步主要是优化模型系数
        seg.setModelType(pcl::SACMODEL_PLANE);  //分割类型为平面
        seg.setMethodType(pcl::SAC_RANSAC);  //方法
        seg.setDistanceThreshold(0.1);  //设置距离阀值，可调节分割平面的点数（平面精度）,需要根据实际采样点集设置
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients); //coefficients存储平面参数，inlier为平面点索引
        
        /*提取平面点*/
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false); //设置提取平面点。（若为true，则是提取的非平面点，inliers之外的点）
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_p);   //最终cloud_p为平面点云

        
        //保存结果
        pcl::io::savePCDFile("result.pcd",*cloud_p);
        return 0;
}
