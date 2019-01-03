#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>  //这个头文件
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

//tpes
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
using namespace std;

int main(int argc,char **argv)
{
        PointCloudT::Ptr object (new PointCloudT);
        PointCloudT::Ptr object_aligned (new PointCloudT);
        PointCloudT::Ptr scene (new PointCloudT);
        FeatureCloudT::Ptr object_features (new FeatureCloudT);
        FeatureCloudT::Ptr scene_features (new FeatureCloudT);

        // Get input object and scene
        if (argc != 3)
        {
                pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
                return (1);
        }
  
  // Load object and scene
        pcl::console::print_highlight ("Loading point clouds...\n"); //可以输出到屏幕上
        if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
            pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
        {
                pcl::console::print_error ("Error loading object/scene file!\n");
                return (1);
        }

// Downsample.体素网格简化以提高效率
        pcl::console::print_highlight ("Downsampling...\n");
        pcl::VoxelGrid<PointNT> grid;
        const float leaf = 0.05f;
        grid.setLeafSize (leaf, leaf, leaf);
        grid.setInputCloud (object);
        grid.filter (*object);
        grid.setInputCloud (scene);
        grid.filter (*scene);

        // Estimate normals for scene
        pcl::console::print_highlight ("Estimating scene normals...\n");
        pcl::NormalEstimationOMP<PointNT,PointNT> nest;
//        nest.setRadiusSearch (0.01);
        nest.setKSearch(10);
        nest.setInputCloud (scene);
        {
                pcl::ScopeTime time("calculate");
                
                nest.compute (*scene);
        }
        
// Estimate features。分别计算两个场景的fpfh特征
        pcl::console::print_highlight ("Estimating features...\n");
        FeatureEstimationT fest;
//        fest.setRadiusSearch (0.025);
        fest.setKSearch(10);
        fest.setInputCloud (object);
        fest.setInputNormals (object);
        fest.compute (*object_features);
        fest.setInputCloud (scene);
        fest.setInputNormals (scene);
        fest.compute (*scene_features);

         // Perform alignment。实施对齐变换
        pcl::console::print_highlight ("Starting alignment...\n");
        pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;   //这是啥方法？
        
        align.setInputSource (object);
        align.setSourceFeatures (object_features);
        align.setInputTarget (scene);
        align.setTargetFeatures (scene_features);
        align.setMaximumIterations (50000); // Number of RANSAC iterations
        align.setNumberOfSamples (3); // 采样点数。Number of points to sample for generating/prerejecting a pose
        align.setCorrespondenceRandomness (5); //t特征数目。 Number of nearest features to use
        align.setSimilarityThreshold (0.9f); // 长度阀值 。Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
        align.setInlierFraction (0.25f); 
        {
                pcl::ScopeTime t("Alignment");  //这个可以输出计算时间
                align.align (*object_aligned);
        }

        if(align.hasConverged())
        {
                // Print results
                printf ("the tranformation is:\n");
                // Eigen::Matrix4f transformation = align.getFinalTransformation ();
                cout<<align.getFinalTransformation ()<<endl;

                cout<<"the score is:"<<align.getFitnessScore()<<endl;
                
                pcl::console::print_info ("\n");
                pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
                
                // Show alignment
                pcl::visualization::PCLVisualizer visu("Alignment");
                visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
                visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
                visu.spin ();
                
                //  pcl::ScopeTime t("Alignment"); //这里为什么输出是0ms呢？
        }
        else
        {
                pcl::console::print_error ("Alignment failed!\n");
                return (1);
        }

        //   pcl::ScopeTime t("Alignment");
        return (0);
        
}
