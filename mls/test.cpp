/*移动最小二乘法可用以：对点云进行平滑处理，数据重采样，并且可以计算优化的估计法线*/
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/console/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h> //包含基本可视化类
#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
using namespace std;
typedef pcl::PointXYZ point;
typedef pcl::PointCloud<point> pointcloud;


int main (int argc,char **argv)
{
#if 0
        pointcloud::Ptr cloud (new pointcloud);
        pcl::io::loadPCDFile(argv[1],*cloud);
        cout<<"points size is:"<<cloud->size()<<endl;
        pcl::search::KdTree<point>::Ptr tree (new pcl::search::KdTree<point>);

        //创建存储的mls对象
        pcl::PointCloud<pcl::PointNormal> mls_points;
        //   pcl::PointCloud<point> mls_points;

        //创建mls对象
        pcl::MovingLeastSquares<point,pcl::PointNormal> mls;
        
        //   pcl::MovingLeastSquares<point,point> mls;

        mls.setComputeNormals(true);
        mls.setInputCloud(cloud);
        mls.setPolynomialFit(true); //设置为true则在平滑过程中采用多项式拟合来提高精度
        mls.setPolynomialOrder(2); //MLS拟合的阶数，默认是2
        mls.setSearchMethod(tree);
        mls.setSearchRadius(5.1);  //搜索半径
        
        mls.process(mls_points);
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_points_normal (new pcl::PointCloud<pcl::PointNormal>);
        mls_points_normal = mls_points.makeShared();
        
        cout<<"mls poits size is: "<<mls_points.size()<<endl;

#endif 

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud1 (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud2 (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud3 (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud4 (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud5 (new pcl::PointCloud<pcl::PointNormal>);
        
        pcl::io::loadPCDFile(argv[1],*cloud1);
        pcl::io::loadPCDFile(argv[2],*cloud2);
        pcl::io::loadPCDFile(argv[3],*cloud3);
        pcl::io::loadPCDFile(argv[4],*cloud4);
        pcl::io::loadPCDFile(argv[5],*cloud5);
        
        boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer ("test"));
        view->setBackgroundColor(0.0,0,0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v1(cloud1,0,250,0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v2(cloud2,255,250,0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v3(cloud3,0,250,255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v4(cloud4,255,0,0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v5(cloud5,0,0,255);
        
        //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v(mls_points_normal,0,250,0);
        //   view->addPointCloud<pcl::PointNormal>(mls_points_normal,v,"sample");
        view->addPointCloud<pcl::PointNormal>(cloud1,v1,"sample1");
        view->addPointCloud<pcl::PointNormal>(cloud2,v2,"sample2");
        view->addPointCloud<pcl::PointNormal>(cloud3,v3,"sample3");
        view->addPointCloud<pcl::PointNormal>(cloud4,v4,"sample4");
        view->addPointCloud<pcl::PointNormal>(cloud5,v5,"sample5");

        //view->addPointCloudNormals<pcl::PointNormal>(mls_points_normal,500,10,"normal");
        view->addPointCloudNormals<pcl::PointNormal>(cloud1,50,20,"normal1");
        view->addPointCloudNormals<pcl::PointNormal>(cloud2,50,20,"normal2");
        view->addPointCloudNormals<pcl::PointNormal>(cloud3,50,20,"normal3");
        view->addPointCloudNormals<pcl::PointNormal>(cloud4,50,20,"normal4");
        view->addPointCloudNormals<pcl::PointNormal>(cloud5,50,20,"normal5");
        
        view->addCoordinateSystem(1.0); //建立空间直角坐标系
        view->spin();
        
        
        // Save output
        //    pcl::io::savePCDFile ("mid-mls5.pcd", mls_points);

}
