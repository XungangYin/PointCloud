#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h> //包含基本可视化类
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
using namespace std;

int main(int argc, char **argv)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yxg/pcl/pcd/mid.pcd",*cloud) == -1)

        pcl::console::TicToc time;
        time.tic();
        
        
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud) == -1)
        {
                PCL_ERROR("COULD NOT READ FILE mid.pcl \n");
                return (-1);
        }
        
        std::cout << "points sieze is:"<< cloud->size()<<std::endl;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZ,pcl::Normal,pcl::Boundary> est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

        //平均密度计算
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
        kdtree.setInputCloud(cloud);
        int k =2;
        float everagedistance =0;
        for (int i =0; i < cloud->size()/2;i++)
        {
                vector<int> nnh ;
                vector<float> squaredistance;
                //  pcl::PointXYZ p;
                //   p = cloud->points[i];
                kdtree.nearestKSearch(cloud->points[i],k,nnh,squaredistance);
                everagedistance += sqrt(squaredistance[1]);
                //   cout<<everagedistance<<endl;
        }

        everagedistance = everagedistance/(cloud->size()/2);
        cout<<"everage distance is : "<<everagedistance<<endl;
        



        
        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
        normEst.setInputCloud(cloud);
        normEst.setSearchMethod(tree);
        // normEst.setRadiusSearch(2);  //法向估计的半径
        normEst.setKSearch(20);  //法向估计的点数
        normEst.compute(*normals);
        cout<<"normal size is "<< normals->size()<<endl;
        
        // normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
        est.setInputCloud(cloud);
        est.setInputNormals(normals);
        //  est.setAngleThreshold(90);
        //   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
        est.setSearchMethod (tree);

        int num;
        cout<<"请输入边界识别时近邻点个数："<<endl;
        cin>>num;
        est.setKSearch(num);  //一般这里的数值越高，最终边界识别的点数越少
        //  est.setRadiusSearch(everagedistance);  //搜索半径
        est.compute (boundaries);

        //  pcl::PointCloud<pcl::PointXYZ> boundPoints;
        pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
        int countBoundaries = 0;
	for (int i=0; i<cloud->size(); i++){
                uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if ( a == 1)
                {
                        //  boundPoints.push_back(cloud->points[i]);
                        ( *boundPoints).push_back(cloud->points[i]);
                        countBoundaries++;
                }
                else
                        noBoundPoints.push_back(cloud->points[i]);
                
	}
        
        cout<<"time is:"<<time.toc()/1000<<"s"<<endl;
        
        std::cout<<"boudary size is：" <<countBoundaries <<std::endl;
        //  pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);

        pcl::io::savePCDFileASCII("boudary.pcd", *boundPoints);
        pcl::io::savePCDFileASCII("NoBoundpoints.pcd",noBoundPoints);
        //  pcl::visualization::CloudViewer viewer ("test");
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("test"));
        
        // viewer.showCloud(boundPoints);
        viewer->setBackgroundColor(255,255,255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(boundPoints,0,0,0);
        viewer->addPointCloud<pcl::PointXYZ>(boundPoints,color,"s");
        
        
        while (!viewer->wasStopped())
        {
                viewer->spinOnce(100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
        return 0;
}
