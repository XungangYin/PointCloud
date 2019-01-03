/*
  利用欧式聚类算法，将点云分类，并分别存贮在文件中
 */
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

int main(int argc,char **argv)
{
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        // reader.read("/home/yxg/pcl/pcd/mid.pcd",*cloud);
        reader.read(argv[1],*cloud);
        
        
        cout<<"cloud size is:  "<<cloud->size()<<endl;

        /*设置滤波器，对原始点集进行下采样 */
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01f,0.01f,0.01f);
        vg.filter(*cloud_filtered);
        cout<<"point cloud after voxelgrid filte size is:"<<cloud_filtered->size()<<endl;

        /*创建平面分割 */
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);

        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.02);

        int i = 0, nr_points = (int) cloud_filtered->points.size();
        while(cloud_filtered->points.size() > 0.3*nr_points)
        {
                seg.setInputCloud (cloud_filtered);
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size()==0)
                {
                        cout<<"could not estimate plannar "<<endl;
                        break;
                }

                //提取平面
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_filtered);
                extract.setIndices(inliers);
                extract.setNegative(false);  //到底是什么东西
                extract.filter(*cloud_plane);
                cout<<"plane size is:"<<cloud_plane->points.size()<<endl;

                //remove 平面数据
                extract.setNegative(true);
                extract.filter (*cloud_f);
                *cloud_filtered = *cloud_f;
                
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered);
        
        
        /*....创建聚类分割....*/
        vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02); //搜索近邻半径 2cm
        ec.setMinClusterSize(100); //设置一个聚类最少点的数目
        ec.setMaxClusterSize(25000); //一个聚类最大点的数目
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices); //从点云中提取聚类，并将点云索引号保存在cluster_indices中

        cout << "cluster size is:  "<<cluster_indices.size()<<endl;
        
        int j= 0 ;
        for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end();++it)
        {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

                for (vector<int>::const_iterator pit = it->indices.begin();pit != it->indices.end();++pit)
                        cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                  cout<<"cloud size is :  "<<cloud_cluster->size()<<endl;

                stringstream ss;
                ss<<"cloud_cluster_"<<j<<".pcd";
                writer.write<pcl::PointXYZ>(ss.str(),*cloud_cluster,false);
                j++;
                
        }
        return 0;
        
}
