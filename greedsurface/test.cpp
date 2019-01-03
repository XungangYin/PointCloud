 #include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>   //贪婪投影三角化类定义的头文件
#include <pcl/io/vtk_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <vector>
/*..   概算发要求点云平滑而且密度变化连续 ..*/

using namespace std;
int main()
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile("/home/yxg/pcl/pcl/pcd/bunny.pcd",*cloud);
        cout <<"point size is :" <<cloud->size()<<endl;

        // 法向估计
        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(8);
        n.compute(*normals);  //注意是*normals而不是normal

        //将点和法向量链接
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normal (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud,*normals,*cloud_with_normal); // 将法向和点连接在一起
        
        
        /*
        for (size_t i =0;i < cloud_with_normal->points.size();++i)
        {
                cout <<cloud_with_normal->points[i].x<<","<<cloud_with_normal->points[i].y<<","<<cloud_with_normal->points[i].z<<"normal is:"<<cloud_with_normal->points[i].normal[0]<<","<<cloud_with_normal->points[i].normal[1]<<","<<cloud_with_normal->points[i].normal[2]<<endl;     
        } */

        //计算点云间的平局距离
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        int k=2;
        vector<int>nnh;
        vector<float> distance;
        float d =0.0;
        for (size_t i =0;i < cloud->size();i++)
        {
                kdtree.nearestKSearch(cloud->points[i],k,nnh,distance);
                d += sqrt(distance[1]);
        }
        float everagedistance = d/cloud->size();
        cout<<"average distance is ::"<<everagedistance <<endl;
        

        
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normal);

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; //初始化贪婪三角网格 对象
        pcl::PolygonMesh triangles;  //存储最终三角网格化的模型
        gp3.setSearchRadius(everagedistance*2.7); //设置链接点之间的最大距离（三角形的最大边长）

        //  gp3.setSearchRadius(0.085);
        gp3.setMu(5.5);  //设置被样本点搜索其近邻点的最远距离为2.5
        gp3.setMaximumNearestNeighbors(50);  //设置样本点可搜索的邻域点个数
        gp3.setMaximumSurfaceAngle(M_PI/3); //设置某点法线方向偏离样本点法线方向的最大距离为60度
        gp3.setMinimumAngle(M_PI/18);  //设置三角化后的三角形内角最小角度为10度
        gp3.setMaximumAngle(2*M_PI/3); //内角最大角度为120
        gp3.setNormalConsistency(false);   //设置该参数保持法向一致(false不进行法向一致性检查)

        gp3.setInputCloud(cloud_with_normal);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);

        pcl::io:: saveVTKFile("mesh.vtk",triangles);
        

        vector<int> parts = gp3.getPartIDs();
        vector<int> states = gp3.getPointStates();
        cout<<"parts size is:"<<parts.size()<<endl;
        cout<<"states size is :"<<states.size()<<endl;

        boost::shared_ptr<pcl::visualization::PCLVisualizer>view (new pcl::visualization::PCLVisualizer ("vtk "));
        view->setBackgroundColor(0,0,0);
        view->addPolygonMesh(triangles,"my");
        view->addCoordinateSystem(1.0);
        view->initCameraParameters();
        while(!view->wasStopped())
        {
                view->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        }
        
        return 0;

}
