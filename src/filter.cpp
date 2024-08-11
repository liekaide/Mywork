#include"filter.h"
#include"read_write.h"
#include<cmath>
#include<unordered_map>
#include<Eigen/Dense>
#include <pcl/common/common.h>

using namespace std;
using namespace Eigen;

// 定义三维向量的哈希函数，用于在unordered_map中使用
struct Vector3iHash {
    size_t operator()(const Vector3i& v) const {
        return hash<int>()(v.x()) ^ hash<int>()(v.y()) ^ hash<int>()(v.z());
    }
};

void first_filter(const PointCloud::Ptr& cloud, PointCloud::Ptr& filtered_cloud, vector<vector<int>>& faces, float ratio, int N, int minPointsPerVoxel){

    // 计算点云包围盒的最大和最小坐标
    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    // 计算包围盒的边长
    float Lx = maxPt.x - minPt.x;
    float Ly = maxPt.y - minPt.y;
    float Lz = maxPt.z - minPt.z;

    // 计算小立方体栅格的边长
    float L = cbrt((Lx * Ly * Lz) / N) * ratio;

     // 创建一个哈希表来存储每个栅格内的点数，<>中的三个分别是键类型、值类型
     // 和哈希函数类型，键是一个三维列向量，用来存栅格的坐标，值用来存点云中点的索引
     // 目的是计算出每个栅格内存在哪些点
    unordered_map<Vector3i, vector<int>, Vector3iHash> voxelMap;

    // 遍历所有点，填充栅格哈希表
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const PointT& point = cloud->points[i];
        Vector3i voxelIndex = Vector3i(
            floor((point.x - minPt.x) / L),//floor函数向下取整
            floor((point.y - minPt.y) / L),
            floor((point.z - minPt.z) / L)
        );
        voxelMap[voxelIndex].push_back(i);
    }

    // 过滤点云，保留点数大于等于minPointsPerVoxel的栅格
    for (const auto& entry : voxelMap) {
        if (entry.second.size() >= minPointsPerVoxel) {
            for (int index : entry.second) {
                filtered_cloud->points.push_back(cloud->points[index]);
            }
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    

}