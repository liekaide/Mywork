#ifndef DBSCAN_H
#define DBSCAN_H
#include"read_write.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void filterUsingDBSCAN(PointCloud::Ptr& cloud, PointCloud::Ptr& filtered_cloud, float eps, int minPoints);

#endif