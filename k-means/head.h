#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <cmath>
#include <fstream>
#include <iterator>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <time.h>
#include <stdlib.h>

using namespace std;

typedef struct st_pointxyz
{
        float x;
        float y;
        float z;
}st_pointxyz;
typedef struct st_point
{
        st_pointxyz pnt;
        int groupID;
        st_point()   //在c++中其实是构造函数，关键为什么这莫写
                {
                } 
        st_point(st_pointxyz &p,int id)
                {
                        pnt =p;
                        groupID= id;
                }
}st_point;

class KMeans
{
public:
        int m_k;
        typedef vector<st_point> VecPoint_t;  //定义命令别名

        VecPoint_t mv_pntcloud; //要聚类的点云
        vector<VecPoint_t>m_grp_pntcloud;  //k类，每一类存储若干点
        vector<st_pointxyz>mv_center; //每个类的中心

        KMeans()  //构造函数初始化
        {
                m_k =0;
        }
        inline void SetK(int k_) //设置聚类簇数
        {
                m_k = k_;
                m_grp_pntcloud.resize(m_k);  //resize是改变容器的大小
        }

        //设置输入点云
        bool SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud);  

        //初始化最初的k个类的中心
        bool InitKCenter();  

        //聚类
        bool Cluster();

        //更新k类的中心(参数为类和中心点)
       vector<st_pointxyz>  UpdateGroupCenter(vector<VecPoint_t> &grp_pntcloud,vector<st_pointxyz> cer);

        //计算两点欧式距离
        double DistBetweenPoints(st_pointxyz &p1,st_pointxyz &p2);

        //是否存在中心点转移动
        bool ExistCenterShift(vector<st_pointxyz> &prev_center,vector<st_pointxyz> &cur_center);

        //将聚类分别存储到各自的pcd文件中
        bool SaveFile(const char *prex_name);
        bool SaveFile(const char *dir_name,const char *prex_name);

};

