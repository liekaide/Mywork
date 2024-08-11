#include "DBSCAN.h"
#include <pcl/common/common.h>

// 使用DBSCAN聚类来移除噪声点云
void filterUsingDBSCAN(PointCloud::Ptr& cloud, PointCloud::Ptr& filtered_cloud, float eps_ratio, float minPoints_ratio) {
    // 创建KdTree对象用于搜索
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // 计算 minPoints，基于点云总数的比例
    int minPoints = static_cast<int>(cloud->points.size() * minPoints_ratio);

    // 计算点云的包围盒
    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    // 计算包围盒的最大边长
    float L = max({maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z});

    // 计算 eps邻域半径
    float eps = L * eps_ratio;

    // 保存聚类的点索引
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(eps);  // 设置邻域搜索半径
    ec.setMinClusterSize(minPoints);  // 最小的聚类尺寸
    ec.setMaxClusterSize(cloud->points.size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);  // 执行聚类提取

    // 提取最大的一个聚类
    pcl::PointIndices::Ptr largest_cluster(new pcl::PointIndices);
    for (const auto& indices : cluster_indices) {
        if (indices.indices.size() > largest_cluster->indices.size()) {
            *largest_cluster = indices;
        }
    }

    // 提取点云
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(largest_cluster);
    extract.setNegative(false);  // 保留索引中的点
    extract.filter(*filtered_cloud);

    // 更新过滤后的点云的属性
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
}