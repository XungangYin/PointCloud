#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <string>
#include <ostream>
#include <math.h>
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/Vertices.h>
#include <float.h>
#include <gsl/gsl_linalg.h>

using namespace pcl;
using namespace std;

int main(int argc, char ** argv)
{
        /* 文件读入阶段 */
#if 1
        if (argc <= 1) {
                cout << "请输入点云数据文件名称，并指定输出数据文件名称" <<endl;
                return -1;
        }

        PointCloud<PointXYZ>::Ptr pointcloud (new PointCloud<PointXYZ>);
        if (io::loadPLYFile<PointXYZ> (argv[1], *pointcloud) == -1) {
                cout << "数据读入失败！！！" << endl;
        }
#endif
        pcl::PolygonMesh mesh;
        if (io::loadPLYFile(argv[1],  mesh)  == -1) {
                   cout << "数据读入失败！！！" << endl;
        }

        pcl::PointXYZ searchPoint; //目标点
        searchPoint.x = 42;
        searchPoint.y = 29;
        searchPoint.z = 36;

        /* 寻找距离目标点最近的三角网格顶点 */
        
        pcl::PointXYZ closestpoint;
        double d= DBL_MAX;
        size_t number = 0; //最近点面片编号
        for (size_t i = 0; i < pointcloud->points.size(); i++)
        {
                pcl::PointXYZ p = pointcloud->points[i];
                double sd = fabs( (searchPoint.x - p.x) * (searchPoint.x - p.x)
                                  + (searchPoint.y - p.y) * (searchPoint.y - p.y)
                                  + (searchPoint.z - p.z) * (searchPoint.z - p.z) );
                if (sd < d)
                {
                        d = sd;
                        number = i;
                }
        }

        /* 提取一阶邻域三角面片 */
        vector< vector<size_t> > f_num;
        
        for (size_t i = 0; i < mesh.polygons.size(); i++)
        {
                cout<<"面片数量是:"<<mesh.polygons.size()<<endl;
                for (size_t j = 0; j < 3; j++)
                { 
                        if (mesh.polygons[i].vertices[j] == number)
                        {
                                vector<size_t> num;
                                num.push_back(i);
                                num.push_back(j);
                                
                                for (size_t k = 0; k < 3; k++)
                                {
                                        if (k != j)
                                                num.push_back(k);
                                }
                                
                                f_num.push_back(num);
                        }   
                }       
        }

        return 0;
}
