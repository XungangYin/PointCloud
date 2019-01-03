#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/search/pcl_search.h>
#include <pcl/surface/reconstruction.h>
using namespace std;

int main(int argc,char **argv)
{
        pcl::PolygonMesh mesh;
        //   pcl::io::loadPolygonFileOBJ(argv[1], mesh);
        pcl::io::loadPLYFile(argv[1],mesh);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
        //pcl::io::savePCDFileASCII("result.pcd", *cloud);

        /*查询指定点的最近顶点*/
        //  pcl::PointXYZ point;
        pcl::PointXYZ *point = new pcl::PointXYZ;
        point->x =10;
        point->y=450;
        point->z = 300;
        int k  = 1;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        vector<int> nnh;
        vector<float> distance;
        kdtree.nearestKSearch(*point,k,nnh,distance);

        delete point;
        
        /*查询1环域三角面片*/
        /*查看pcl源码可知道，mesh.polygons 的数据是：vector<pcl::Vertices>类型*/
        /*然而pcl::Vertices类的数据为:vector<uint_32>*/

        int id  = nnh[0];  //这是面片某顶点的索引
        cout<<"id is:"<<id<<endl;

        pcl::PolygonMesh myMesh;
        pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZ>);
        /*遍历所有三角面片的顶点*/
        outcloud->push_back(cloud->points[id]);
        for(size_t i =0; i< mesh.polygons.size();i++)
        {
                for(size_t j = 0; j < 3;j++)
                {
                        if(mesh.polygons[i].vertices[j] == id)
                        {
                                
                                // myMesh.polygons.push_back(mesh.polygons[i]);
                                cout<<mesh.polygons[i]<<endl;
                                int a = mesh.polygons[i].vertices[0];
                                int b = mesh.polygons[i].vertices[1];
                                int c = mesh.polygons[i].vertices[2];
                                if(a != id)
                                        outcloud->push_back(cloud->points[a]);
                                if(b != id)
                                        outcloud->push_back(cloud->points[b]);
                                if(c != id)
                                        outcloud->push_back(cloud->points[c]);
#if 0
                                pcl::Vertices p;
                                p.push_back(i);
                                p.push_back(j);
#endif
                        }
                        
                }    

        }
       
        cout<<"对发生的发生法"<< mesh.polygons[10].vertices[2]<<endl;
        
#if 0
        myMesh.header = mesh.header;
        pcl::toPCLPointCloud2(*outcloud,myMesh.cloud);

        myMesh.polygons.reserve(5);
        int n= 0;
        for(int i = 0 ;i < 5; i++)
        {

                mesh.polygons[i].vertices[0]=n;
                mesh.polygons[i].vertices[1]=n+1;
                mesh.polygons[i].vertices[2]=n+2;
                n=n+3;
                // cout<<"myMesh is:"<<myMesh.polygons[i].vertices[0]<<endl;
                
                                //  myMesh.polygons[i].push_back(p);
        }
        cout<<n<<endl;
        
        cout<<"myMesh is:"<<myMesh.polygons.size()<<endl;
       
#endif
        
        
        
//performReconstruction(myMesh);
        // pcl::MeshConstruction<pcl::PointXYZ> re;
        // re.reconstruction(myMesh);
        
        
        pcl::io::savePCDFile("2.pcd",*outcloud);
        //  pcl::io::savePLYFile("1.ply", myMesh);
        
        cout<<" mesh header is:"<<mesh.header<<endl;        
        
        return 0;
}