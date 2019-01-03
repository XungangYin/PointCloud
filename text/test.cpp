#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <stdio.h>
#include <time.h>


using namespace std;
int main ()
{
        double dur;
        clock_t start,end;
        start = clock();
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yxg/pcl/pcd/room_scan1.pcd",*target_cloud) == -1)
        {
                PCL_ERROR("COULD NOT READ FILE mid.pcl \n");
                return (-1);
        }
        cout << "load  target size: " << target_cloud->size() << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yxg/pcl/pcd/room_scan2.pcd",*input_cloud);
        
        cout << "load  input size: " << input_cloud->size() << endl;

        //利用数据的10%计算,体速网格过滤
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter; //创建体素网格对象
        approximate_voxel_filter.setLeafSize(0.2,0.2,0.2);  //设置体素网格的大小
        approximate_voxel_filter.setInputCloud(input_cloud);
        approximate_voxel_filter.filter (*filtered_cloud);
        cout << "Filtered cloud contains" << filtered_cloud->size()<<endl;
        //初始化正太分布（NDT）变换对象
        pcl::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> ndt;
        //设置NDT相关参数
        ndt.setTransformationEpsilon(0.01);//好象是精度，迭代终止条件.
         // Setting maximum step size for More-Thuente line search.
        ndt.setStepSize (0.1); //为More-Thuente线搜索设置最大步长
        ndt.setResolution(1.0); //设置NDT网格结构的分辨率,与点云数据的尺度相关

        ndt.setMaximumIterations(35); //设置最大迭代次数

        ndt.setInputSource(filtered_cloud);
        ndt.setInputTarget(target_cloud);
        
        //设置使用机器人测距法得到的粗略初始变换矩阵结果
        Eigen::AngleAxisf init_rotation(0.6931,Eigen::Vector3f::UnitZ());
        
        Eigen::Translation3f init_translation(1.79387,0.720047,0);

        Eigen::Matrix4f init_guess=(init_translation*init_rotation).matrix();
        //计算需要的刚体变换以便将输入的源点云匹配到目标点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        ndt.align(*output_cloud,init_guess);
        //此处output_cloud不能作为最终的源点云变换，因为上面对源点云进行了滤波处理

        std::cout<<"Normal Distributions Transform has converged:"<<ndt.hasConverged()
                 <<" score: "<<ndt.getFitnessScore()<<std::endl; //ndt.FitnessScore 表示的是欧式适度评分，表示变换后两点云对应点对的距离平方和
         // Transforming unfiltered, input cloud using found transform.
        pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
  // Saving transformed input cloud.
        pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);
         // Initializing point cloud visualizer

     
        end = clock();
        dur = end - start;
        cout << "time is:"<< dur/CLOCKS_PER_SEC<<endl;
        
        boost::shared_ptr<pcl::visualization::PCLVisualizer>
                viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                target_color (target_cloud, 255, 0, 0);
        viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
        viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                output_color (output_cloud, 0, 255, 0);
        viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
        viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
        viewer_final->addCoordinateSystem (1.0, "global");
        viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
        while (!viewer_final->wasStopped ())
        {
                viewer_final->spinOnce (100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }

        
/*        
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; //创建KDtree
        kdtree.setInputCloud (in_cloud);
        pcl::PointXYZ searchPoint; //创建目标点，（搜索该点的近邻）
        searchPoint.x = 1;
        searchPoint.y = 2;
        searchPoint.z = 3;

        //查询近邻点的个数
        int k = 10; //近邻点的个数
        std::vector<int> pointIdxNKNSearch(k); //存储近邻点集的索引
        std::vector<float>pointNKNSquareDistance(k); //近邻点集的距离
        if (kdtree.nearestKSearch(searchPoint,k,pointIdxNKNSearch,pointNKNSquareDistance)>0)
        {
                for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
                        std::cout << "    "  <<   in_cloud->points[ pointIdxNKNSearch[i] ].x 
                                  << " " << in_cloud->points[ pointIdxNKNSearch[i] ].y 
                                  << " " <<in_cloud->points[ pointIdxNKNSearch[i] ].z 
                                  << " (squared distance: " <<pointNKNSquareDistance[i] << ")" << std::endl;
        }

        //半径为r的近邻点
        float radius = 40.0f;  //其实是求的100*100距离范围内的点
        std::vector<int> pointIdxRadiusSearch;  //存储的对应的平方距离
        std::vector<float> a;
        if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, a) > 0 )
        {
          for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                  std::cout << "    "  <<   in_cloud->points[ pointIdxRadiusSearch[i] ].x 
                            << " " <<in_cloud->points[ pointIdxRadiusSearch[i] ].y 
                            << " " << in_cloud->points[ pointIdxRadiusSearch[i] ].z 
                            << " (squared distance: " <<a[i] << ")" << std::endl;
        }
*/
        /*      
        double dur;
        end = clock();
        dur = end - start;
        cout << "time is:"<< dur/CLOCKS_PER_SEC<<endl;
*/
        
        return 0;
}



