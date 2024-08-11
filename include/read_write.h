#ifndef READ_WRITE_H
#define READ_WRITE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>

using namespace std;
// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 函数声明
bool loadOBJ(const string& filename, PointCloud::Ptr& cloud, vector<vector<int>>& faces,vector<int>& point_indices);
void savecloudtoOBJ(const string& filename, const PointCloud::Ptr& cloud, const vector<vector<int>>& faces);
void updateFaces(PointCloud::Ptr& filtered_cloud, vector<vector<int>>& faces,const vector<int>& original_indices);
#endif // READ_WRITE_H