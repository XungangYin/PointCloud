#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)   //为了显示，需要构造一个函数
{
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("random"));
        viewer->setBackgroundColor(0,0,0);
        viewer->addPointCloud<pcl::PointXYZ> (cloud,"sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"sample cloud");
        viewer->initCameraParameters();
        return viewer;
}
using namespace std;
int main(int argc, char **argv)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile("/home/yxg/pcl/pcd/parts.pcd",*cloud);
        cout <<"size is "<<cloud->size()<<endl;
        
        
        vector<int> inliers;
        //创建采样一致性对象
        pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));   //球体
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));  //平面

        if(pcl::console::find_argument (argc,argv,"-p")>=0)
        {
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
                ransac.setDistanceThreshold(0.1);  //设置点到平面的距离
                ransac.computeModel();  //执行随机参数估计
                ransac.getInliers(inliers); //存储估计得到的局内点的索引
        }
        else if (pcl::console::find_argument(argc,argv,"-s")>=0)  //命令行参数
        {
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);  //随机估计对应的圆球模型
                ransac.setDistanceThreshold(0.1); //与球面距离小于0.1的点作为局内点
                ransac.computeModel();
                ransac.getInliers(inliers);
        }

        pcl::copyPointCloud<pcl::PointXYZ>(*cloud,inliers,*final); //复制局内点到final
        
        //   pcl::io::savePCDFile("ransac.pcd",*final);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        if (pcl::console::find_argument(argc,argv,"-p")>=0 || pcl::console::find_argument(argc,argv,"-s")>=0)
                viewer = simpleVis(final);
        else
                viewer = simpleVis(cloud);

        while (!viewer->wasStopped())
        {
                viewer->spinOnce (100);  //这两行是必须的，在pcl::PCLVisualizer下
              boost::this_thread::sleep (boost::posix_time::microseconds (100000));   
        }
        return 0;
        
}

