#include <pcl/features/rops_estimation.h>
#include <pcl/io/pcd_io.h>

using namespace std;

int main (int argc ,char **argv)
{
        if(argc != 2)
                return -1;
        
        std::vector <pcl::Vertices> triangles;
        std::ifstream triangles_file;
        triangles_file.open (argv[1], std::ifstream::in);
        for (std::string line; std::getline (triangles_file, line);)
        {
                pcl::Vertices triangle;
                std::istringstream in (line);
                unsigned int vertex = 0;
                //  double vertex = 0;
                
                in >> vertex;   //将字符类型转换为无符号整型
                cout<<vertex<<endl;
                
            
                
                triangle.vertices.push_back (vertex - 1);  //??为什么减去1呢 
                in >> vertex;
                triangle.vertices.push_back (vertex - 1);
                in >> vertex;
                triangle.vertices.push_back (vertex - 1);
                triangles.push_back (triangle);
                    
        }
        

        
        return 0;
        
}

