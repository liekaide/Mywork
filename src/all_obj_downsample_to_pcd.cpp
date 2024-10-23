#include <iostream>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <sstream>

using namespace pcl;
using namespace std;

int main() {
    // 创建一个空的点云，用于存储合并后的点数据
    PointCloud<PointXYZ>::Ptr merged_cloud(new PointCloud<PointXYZ>);
    for (int i = 26; i <= 43; ++i) {
        if (i == 35) continue;  // 跳过文件 35
        // 定义点云的容器
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

        // 构建文件名
        stringstream ss;
        ss << "data/" << i << ".obj";
        string input_filename = ss.str();

        // 读取OBJ文件
        if (io::loadOBJFile(input_filename, *cloud) == -1) {
            cerr << "Failed to load OBJ file: " << input_filename << " - skipping this file." << endl;
            continue; // 跳过当前文件，继续处理下一个文件
        }
        cout << "数据读入完成: " << input_filename << endl;
        cout << "输入点云信息: " << endl;
        cout << "Number of points: " << cloud->size() << endl;
        // 将读取的点数据添加到合并后的点云中
        *merged_cloud += *cloud;
        cout << input_filename << "已合并完成 " << endl;
    }

    /* 体素网格滤波阶段 - 下采样 */
    PointCloud<PointXYZ>::Ptr cloud_downsampled(new PointCloud<PointXYZ>());

    VoxelGrid<PointXYZ> sor;
    sor.setInputCloud(merged_cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素网格的大小
    sor.filter(*cloud_downsampled);

    cout << "体素网格滤波（下采样）完成" << endl;
    cout << "下采样后点云信息: " << endl;
    cout << "Number of points: " << cloud_downsampled->size() << endl;

    /* 平滑阶段 - 移动最小二乘平面滤波 */
    MovingLeastSquares<PointXYZ, PointXYZ> mls;
    mls.setInputCloud(cloud_downsampled);
    mls.setSearchRadius(0.5); // 设置搜索半径
    mls.setPolynomialOrder(2); // 设置移动最小二乘的阶数
    mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::NONE);

    PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
    mls.process(*cloud_smoothed);
    cout << "移动最小二乘平面滤波完成" << endl;

    // 构建输出文件名
    stringstream ss_out;
    ss_out << "result/output" << ".pcd";
    string output_filename = ss_out.str();

    // 保存平滑后的点云为PCD文件
    if (io::savePCDFileASCII(output_filename, *cloud_smoothed) == -1) {
        cerr << "Failed to save PCD file: " << output_filename << endl;
        return -1;  // 返回错误码
    }
    cout << "平滑后的数据已保存到 " << output_filename << endl;
    return 0;
}
