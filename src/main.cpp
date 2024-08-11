#include "read_write.h"
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


int main() {

    // 定义一个存储所有点云的容器
    PointCloud::Ptr combined_cloud(new PointCloud);
    vector<vector<int>> combined_faces;

    // 定义多个OBJ文件路径
    vector<string> obj_files = {"data/25.obj" };

   // 创建点云和面数据的容器
    PointCloud::Ptr cloud(new PointCloud);
    vector<vector<int>> faces;
    vector<int> point_indices;

    //读取所有点云文件
    for(const auto& file:obj_files){
        
     if(!loadOBJ(file,cloud,faces,point_indices)){
        cout<<"can't open source file;"<<endl;
        return -1;
     }

    

    // 创建MLS对象
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud(cloud);

        // 设置MLS参数
        mls.setSearchRadius(0.03); // 调整平滑半径
        mls.setPolynomialOrder(2);
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
        mls.setUpsamplingRadius(0.02);
        mls.setUpsamplingStepSize(0.01);

        // 处理点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>);
        mls.process(*cloud_smoothed);

        // 更新面数据的索引
        updateFaces(cloud_smoothed, faces, point_indices);

        string output_filename="result/first_filtered_25.obj";
        // 保存平滑后的点云和面数据到OBJ文件
        savecloudtoOBJ(output_filename, cloud_smoothed, faces);
    
    }

    
    return 0;
}