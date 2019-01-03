#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;



namespace pcl
{
        template<>
        struct SIFTKeypointFieldSelector<PointXYZ>
        {
                inline float
                operator ()(const PointXYZ &p) const
                        {
                                return p.z;
                                
                        }
        };
}

//获得点的索引
void getIndices (pointcloud::Ptr cloudin, pointcloud keypoints, pcl::PointIndices::Ptr indices)
{
        for (int i = 0; i < keypoints.size();i++)
        {
                for (int j = 0;j < cloudin->size();j++)
                {
                        if (cloudin->points[j].x == keypoints.points[i].x && cloudin->points[j].y == keypoints.points[i].y &&  cloudin->points[j].z == keypoints.points[i].z     )
                        // if ( (cloudin->points[j].x - keypoints.points[i].x)<std::numeric_limits<double>::min)() && (cloudin->points[j].y - keypoints.points[i].y)<std::numeric_limits<double>::min)() && ( cloudin->points[j].z == keypoints.points[i].z)<numeric_limits<double>::min)()     )
                                
                                indices->indices.push_back(j);
                }
        }
}



int main(int argc,char **argv)
{
        string filename= argv[1];
        cout<<"reading "<<filename<<endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(filename,*cloud_xyz) == -1)
        {
                PCL_ERROR("could not read file!");
                return -1;
        }
        cout<<"point size is:"<<cloud_xyz->size()<<endl;
        
        //sift的参数
        const float min_scale = 0.0091; //尺度空间中最小尺度的标准偏差。该值越小，sift关键点越多（不一定）,这个值越小，计算的sift关键点越精确，如果偏大，则关键点可能和源点云中对应点有一些偏差
        const int n_octaves = 4 ; //此值越大，sift点越多，但是计算速度明显减慢
        const int n_scales_per_octave = 5;//越大，sift越多，速度减慢
        const float min_contrast = 0.0008f; ///the minimum contrast required for detection检测所需的最小对比度.

        pcl::console::TicToc time;
        time.tic();

        
        // Estimate the sift interest points using z values from xyz as the Intensity variants
        //  pcl::SIFTKeypoint<pcl::PointXYZ,pcl::PointWithScale> sift;
        pcl::SIFTKeypoint<pcl::PointXYZ,pcl::PointXYZ> sift;
        //  pcl::PointCloud<pcl::PointWithScale> result;
        pcl::PointCloud<pcl::PointXYZ> result;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        sift.setSearchMethod(tree);

        sift.setScales(min_scale,n_octaves,n_scales_per_octave); //第一个参数：点云体素尺度空间中的标准偏差，点云对应体素栅格中体素的最小尺寸;第二个参数：检测关键点时体素空间尺度的数目; 第三个参数：每一个体素空间尺度下计算高斯空间尺度的参数
        
        sift.setMinimumContrast(min_contrast);  //设置候选关键点时应有的对比度下限。该值设置越小，检测的关键点越多
        sift.setInputCloud(cloud_xyz);
        sift.setKSearch(20);
        sift.compute(result);
        
        pcl::io::savePCDFile("sift.pcd",result);
        
        
        /*以下是自己做测试用,证明必须进行重载*/
        /*
        pcl::SIFTKeypoint<pcl::PointXYZ,pcl::PointXYZ> sift;
        pcl::PointCloud<pcl::PointXYZ> result;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        sift.setSearchMethod(tree);

        sift.setScales(min_scale,n_octaves,n_scales_per_octave); //第一个参数：点云体素尺度空间中的标准偏差，点云对应体素栅格中体素的最小尺寸;第二个参数：检测关键点时体素空间尺度的数目; 第三个参数：每一个体素空间尺度下计算高斯空间尺度的参数
        
        sift.setMinimumContrast(min_contrast);  //设置候选关键点时应有的对比度下限。该值设置越小，检测的关键点越多
        sift.setInputCloud(cloud_xyz);
        sift.compute(result);
        */
        
        pcl::PointIndices::Ptr source_sift_indices (new pcl::PointIndices);
        getIndices(cloud_xyz,result,source_sift_indices);
        cout<<"\nsources indeces size is:"<<source_sift_indices->indices.size()<<endl;


        cout<<" sift poinst in the resulat are："<<result.size()<<endl;
        cout<<"computing the sift points takes "<<time.toc()/1000<<"s"<<endl;
        //  cout<<" sift poinst in the resulat are："<<result.points.size()<<endl;

        
        // Copying the pointwithscale to pointxyz so as visualize the cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(result,*cloud_temp);
        
        //可视化
        pcl::visualization::PCLVisualizer viewer ("PCL");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color (cloud_temp,255,0,0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud_xyz,0,255,0);
        viewer.setBackgroundColor(0.0,0.0,0.0);
        viewer.addPointCloud(cloud_xyz,cloud_color,"cloud");
        viewer.addPointCloud(cloud_temp,keypoints_color,"key");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,7,"key");
        

        while(!viewer.wasStopped())
        {
                viewer.spin();
        }
        return 0;
}
