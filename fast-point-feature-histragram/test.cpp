#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h> //应该是包含normal计算的omp
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp
#include <ctime>
#include <pcl/visualization/histogram_visualizer.h> //直方图的可视化
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_plotter.h>

using namespace std;

int main ()
{
        clock_t start,end;
        start = clock();
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

        pcl::io::loadPCDFile ("/home/yxg/pcl/pcd/yelun.pcd",*cloud);
        cout <<"point size is:"<<cloud->points.size()<<endl;
        
        //法向量计算  
        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod(tree);
        ne.setKSearch(8);
        ne.compute(*normals);

        //fpfh 计算
        // pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh;
        pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh;
        fpfh.setNumberOfThreads(4);
        
        fpfh.setInputCloud(cloud);
        fpfh.setInputNormals(normals);
        fpfh.setSearchMethod(tree);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
        // fpfh.setRadiusSearch(0.05);
        fpfh.setKSearch(8);
        fpfh.compute(*fpfhs);
        pcl::io::savePCDFile("fpfh.pcd",*fpfhs);
        
        cout << pcl::getFieldsList<pcl::FPFHSignature33>(*fpfhs)<<endl;
        cout <<fpfhs->points.size()<<endl;
        end = clock();
        cout<<"run time is:" <<float (end - start)/CLOCKS_PER_SEC<<endl;
        
        /*
        pcl::visualization::PCLHistogramVisualizer view;
        view.setBackgroundColor(255,0,0);
        view.addFeatureHistogram<pcl::FPFHSignature33> (*fpfhs,"fpfh",1000);   //对下标为1000的元素可视化
        //view.spinOnce(10000);  //循环的次数
        view.spin();  //无限循环
        */
        
        pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*fpfhs, 300);
	plotter.plot();
        
        return 0;

}
