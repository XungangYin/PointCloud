#include "head.h"
#include <ctime>
//#include <pcl/common/common_headers.h>  //包含角度计算

const float DIST_NEAR_ZERO = 0.001;  //控制中心点移动的阀值
char szFileName[256]; 
bool KMeans::InitKCenter( )
{
        mv_center.resize(m_k);
        int size = mv_pntcloud.size();
        srand(unsigned(time(NULL)));  
        for (int i =0; i< m_k;i++)
        {
                int seed = random()%(size+1);
                //    cout<<"seed is:"<<seed<<endl; 
                mv_center[i].x = mv_pntcloud[seed].pnt.x;
                mv_center[i].y = mv_pntcloud[seed].pnt.y;
                mv_center[i].z = mv_pntcloud[seed].pnt.z;
                // cout<<"center x is:"<<mv_center[i].x<<endl;
                
        }
        return true;
}

/*... 设置setiputcloud的作用是将pcl::cloud<pcl::PointXyz>转换为自己定义的   ...*/
bool KMeans::SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud)
{
        size_t pntCount = (size_t) pPntCloud->points.size();
        for (size_t i = 0; i< pntCount;++i)
        {
                st_point point;
                point.pnt.x = pPntCloud->points[i].x;
                point.pnt.y = pPntCloud->points[i].y;
                point.pnt.z = pPntCloud->points[i].z;
                point.groupID = 0;

                mv_pntcloud.push_back(point);
        }
        
        return true;
}

bool KMeans::Cluster()
{
        InitKCenter();
        vector<st_pointxyz>v_center(mv_center.size());
        size_t pntCount = mv_pntcloud.size();

        do
        {
                for (size_t i = 0;i < pntCount;++i)  //初始分类计算
                {
                        double min_dist = DBL_MAX;  //double的最大值
                        int pnt_grp = 0;   //聚类索引号
                        for (size_t j =0;j <m_k;++j)  //这一循环主要计算源点云中某个点与聚类中心的距离，并将该点归入最小距离的聚类中
                        {
                                 double dist = DistBetweenPoints(mv_pntcloud[i].pnt, mv_center[j]);  
                                 if (min_dist - dist > 0.000001)  
                                 {  
                                         min_dist = dist;  
                                         pnt_grp = j;
                                 }
                        }
                        m_grp_pntcloud[pnt_grp].push_back(st_point(mv_pntcloud[i].pnt,pnt_grp)); //将该点和该点群组的索引存入聚类中
                }

                //保存上一次迭代的中心点
                for (size_t i = 0; i<mv_center.size();++i)
                {
                        v_center[i] = mv_center[i];
                }

                mv_center=UpdateGroupCenter(m_grp_pntcloud,mv_center);
                if ( !ExistCenterShift(v_center, mv_center))  
                {  
                        break;   
                }  
                for (size_t i = 0; i < m_k; ++i){  
                        m_grp_pntcloud[i].clear();  
                }  
                
        }while(true);
        
        return true;
}
double KMeans::DistBetweenPoints(st_pointxyz &p1, st_pointxyz &p2)  
{  
        double dist = 0;  
        double x_diff = 0, y_diff = 0, z_diff = 0;  
  
        x_diff = p1.x - p2.x;  
        y_diff = p1.y - p2.y;  
        z_diff = p1.z - p2.z;  
        dist = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);  
      
        return dist;  
}  
vector<st_pointxyz> KMeans::UpdateGroupCenter(std::vector<VecPoint_t> &grp_pntcloud, std::vector<st_pointxyz> center) 
{
    for (size_t i = 0; i < m_k; ++i)  
    {  
        float x = 0, y = 0, z = 0;  
        size_t pnt_num_in_grp = grp_pntcloud[i].size();  
  
        for (size_t j = 0; j < pnt_num_in_grp; ++j)  
        {             
                x += grp_pntcloud[i][j].pnt.x;  //二维数组，求的每一个聚类的坐标分量的和
                y += grp_pntcloud[i][j].pnt.y;  
                z += grp_pntcloud[i][j].pnt.z;  
        }  
        x /= pnt_num_in_grp;  
        y /= pnt_num_in_grp;  
        z /= pnt_num_in_grp;  
        center[i].x = x;   //每一个聚类的中心点
        center[i].y = y;  
        center[i].z = z;  
    }  
    return center;
    
}
//是否存在中心点移动  
bool KMeans::ExistCenterShift(std::vector<st_pointxyz> &prev_center, std::vector<st_pointxyz> &cur_center)  
{  
    for (size_t i = 0; i < m_k; ++i)  
    {  
        double dist = DistBetweenPoints(prev_center[i], cur_center[i]);  
        if (dist > DIST_NEAR_ZERO)  
        {  
            return true;  
        }  
    }  
  
    return false;  
}
//将聚类的点分别存到各自的pcd文件中  
bool KMeans::SaveFile(const char *prex_name)  
{  
    for (int i = 0; i < m_k; ++i)  
    {  
        pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud(new pcl::PointCloud<pcl::PointXYZ> ());  
  
        for (size_t j = 0, grp_pnt_count = m_grp_pntcloud[i].size(); j < grp_pnt_count; ++j)  
        {  
            pcl::PointXYZ pt;  
            pt.x = m_grp_pntcloud[i][j].pnt.x;  
            pt.y = m_grp_pntcloud[i][j].pnt.y;  
            pt.z = m_grp_pntcloud[i][j].pnt.z;  
  
            p_pnt_cloud->points.push_back(pt);  
        }  
  
        p_pnt_cloud->width = (int)m_grp_pntcloud[i].size();
        p_pnt_cloud->height = 1;  
  
        char newFileName[256] = {0};  
        char indexStr[16] = {0};  
  
        strcat(newFileName, szFileName);  
        strcat(newFileName, "-");  
        strcat(newFileName, prex_name);  
        strcat(newFileName, "-");  
        sprintf(indexStr, "%d", i + 1);  
        strcat(newFileName, indexStr);  
        strcat(newFileName, ".pcd");  
        pcl::io::savePCDFileASCII(newFileName, *p_pnt_cloud);  
    }  
      
    return true;  
}
bool KMeans::SaveFile(const char *dir_name, const char *prex_name)  
{  
    for (int i = 0; i < m_k; ++i)  
    {  
        pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud(new pcl::PointCloud<pcl::PointXYZ> ());  
  
        for (size_t j = 0, grp_pnt_count = m_grp_pntcloud[i].size(); j < grp_pnt_count; ++j)  
        {  
            pcl::PointXYZ pt;  
            pt.x = m_grp_pntcloud[i][j].pnt.x;  
            pt.y = m_grp_pntcloud[i][j].pnt.y;  
            pt.z = m_grp_pntcloud[i][j].pnt.z;  
  
            p_pnt_cloud->points.push_back(pt);  
        }  
  
        p_pnt_cloud->width = (int)m_grp_pntcloud[i].size();  
        p_pnt_cloud->height = 1;  
  
        char newFileName[256] = {0};  
        char indexStr[16] = {0};  
  
        strcat(newFileName, dir_name);  
        strcat(newFileName, "/"); //表示路径，如当dir_name = {..},表示存放在上级路径
        strcat(newFileName, prex_name);  
        strcat(newFileName, "-");  
        sprintf(indexStr, "%d", i + 1);  
        strcat(newFileName, indexStr);  
        strcat(newFileName, ".pcd");  
        pcl::io::savePCDFileASCII(newFileName, *p_pnt_cloud);   //我自己的加上的pcl::io
    }  
  
    return true;  
}

int main (int argc,char **argv)
{
        clock_t t1,t2;
        t1 = clock();
        

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        //    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yxg/pcl/pcd/mid.pcd",*cloud);
        pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud);
        cout<<"points size is :"<<cloud->size()<<endl;
        
        KMeans kmeans;
        
        int k =8 ;
        
        kmeans.SetInputCloud(cloud);
        kmeans.SetK(k);
        kmeans.Cluster();
        
        t2 = clock();

        cout << "process time is :"<<float (t2-t1)/CLOCKS_PER_SEC<<endl;

                
        kmeans.SaveFile(".","b");
        return 0;
        
}
