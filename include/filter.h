#ifndef FILTER_H
#define FILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>

using namespace std;

// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


// 函数声明
void first_filter(const PointCloud::Ptr& cloud, PointCloud::Ptr& filtered_cloud, vector<vector<int>>& faces,  float ratio, int N, int minPointsPerVoxel);

#endif // FILTER_H
