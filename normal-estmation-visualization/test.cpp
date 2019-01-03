#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <iostream>
#include <ctime>

using namespace std;

int main()
{

        //  clock_t start,end,time1;
        //  start = clock();

        clock_t t1,t2;
        //   time(&t1);
        t1 = clock();
        
        
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile("/home/yxg/pcl/pcl/pcd/mid1.pcd",*cloud);
        cout <<"points sieze is "<<cloud->size()<<endl;
        
        
        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>()); // normal后边没有小括号
        ne.setKSearch(8);
        //ne.setRadisuSearch(0.3);
        ne.compute(*cloud_normals);
        cout <<"normal size is "<< cloud_normals->size()<<endl;
      
        
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_nomal (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_nomal);
        pcl::io::savePCDFile("cloud_with_normal.pcd",*cloud_with_nomal); 

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor(0.0,0,0);
        // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
        //  pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> v(cloud);    //随机添加颜色

        //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v(cloud,0,250,0);   //可以给点云加上自己喜欢的颜色
         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v(cloud_with_nomal,0,250,0);

         //viewer->addPointCloud<pcl::PointXYZ>(cloud,v,"sample cloud");  //引号中的字符串代表ID的索引
         viewer->addPointCloud<pcl::PointNormal>(cloud_with_nomal,v,"sample cloud");
        //   viewer->addPointCloud<pcl::PointXYZ>(cloud,"sample cloud");
        
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");//此处的数字3表示所渲染的点的大小
        
        //  pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal> v1(cloud_normals,255,0,0);
        // viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal> (cloud,cloud_normals,10,10,"cloud");  //5表示每5个点显示一个法向，15表示法向的长度
         viewer->addPointCloudNormals<pcl::PointNormal> (cloud_with_nomal,50,20,"cloud");
          viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
          //  viewer->setCameraPosition(0,0,200); //设置坐标原点
           // viewer->addCoordinateSystem(100,0.0,0.0,0.0);
        viewer->initCameraParameters();
        

        //   end = clock();
        //  time = end - start;

        // time(&t2);
        t2 =  clock();
        
        
        
        cout << "time is:"<<t2-t1<<"/"<<CLOCKS_PER_SEC<<"(S)" <<endl;
        cout << "time is:"<< float (t2-t1)/(CLOCKS_PER_SEC) <<endl;
        
        
        while(!viewer->wasStopped())
        {
                viewer->spinOnce(100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
        
        return 0;
        

}
