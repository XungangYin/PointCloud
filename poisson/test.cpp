#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/poisson.h>   //泊松重建头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/surface/marching_cubes_hoppe.h> //移动立方体曲面重建
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/io/ply_io.h>

using namespace std;

int main(int argc,char **argv)
{

        // 选取局部样本点集
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(argv[1],*input_cloud);
        cout<<"input_cloud size is:"<<input_cloud->size()<<endl;
        int k = input_cloud->size();
        vector<int>nnh;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(input_cloud);
        vector<float>distance;
        kdtree.nearestKSearch(input_cloud->points[7900],k,nnh,distance);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud<pcl::PointXYZ>(*input_cloud,nnh,*cloud);

// pcl::io::loadPCDFile("/home/yxg/pcl/pcl/pcd/bunny.pcd",*cloud);
        // pcl::io::loadPCDFile(argv[1],*cloud);
        cout<<"points size is:"<<cloud->size()<<endl;
        
        //法向量计算
        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud);
        ne.setKSearch(10);
        ne.compute(*normals);

        //链接
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normal(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud,*normals,*cloud_with_normal);

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normal);

        pcl::PolygonMesh mesh;  //存储结果

        
//泊松重建
        pcl::Poisson<pcl::PointNormal> po;
        po.setConfidence(true); //是否使用法向量的长度作为置信信息。如果为false，则对法线进行归一化处理
        po.setDegree(2); //设置函数的阶数(不需要太高1～5即可)
        po.setDepth(8); //设置重建算法中八叉树的深度。（由于八叉树自适应点云，所以该值仅为最大深度）
        po.setIsoDivide(8); //设置等值面提取时的深度参数,设置该值可避免重建时可能的内存溢出，实践表明设置为7～8比较合适，当该值大于9时，内存使用明显减少，但是重建时间会变长
        po.setManifold(false); //流行标志（若为true，则对细分三角形添加形心）
        po.setOutputPolygons(false); //设置是否输出多边形，ture输出为多边形，false输出三角形 重建后的曲面
        po.setSamplesPerNode(1.0);//设置每个八叉树节点的最小采样数目。（噪声较小是 1～5,较大时15～20），有利于得到最终平滑无噪声的曲面
        po.setScale(1.09); //设置用于重构的立方体直径和样本边界立方体直径的比率。
        po.setSolverDivide(10); //设置求解线性方程组的Gauss-Seidel迭代方法的深度

        po.setInputCloud(cloud_with_normal);
        po.setSearchMethod(tree2);
        
        po.performReconstruction(mesh);  
        
      
        

        
#if 0
        //移动立方体曲面重建
        pcl::MarchingCubes<pcl::PointNormal>  *mc;
        mc = (new pcl::MarchingCubesHoppe<pcl::PointNormal>());
        mc->setInputCloud(cloud_with_normal);
         //设置MarchingCubes对象的参数
        mc->setIsoLevel (0.0f);   //等值面提取时所使用的水平值
        mc->setGridResolution (10, 10, 10);
        mc->setPercentageExtendGrid (0.0f);
        mc->reconstruct (mesh);
#endif
        
        pcl::io::saveVTKFile("mesh_mc.vtk",mesh);
            //保存网格图
        pcl::io::savePLYFile("result.ply", mesh);
        
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("test"));
        viewer->setBackgroundColor(255,255,255);
        viewer->addPolygonMesh(mesh,"my");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color (cloud,255,0,0);
        viewer->addPointCloud(cloud,cloud_color,"target_cloud_v1");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v1");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(input_cloud,0,0,0);
        viewer->addPointCloud(input_cloud,input_color,"input");
        
        
        // viewer->addCoordinateSystem(50.0);
        viewer->initCameraParameters();
        while(!viewer->wasStopped())
        {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
                
        }

        return 0;
        
}

