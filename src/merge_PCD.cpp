#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace std;

int main() {
    // 定义一个点云容器来存储融合后的点云
    PointCloud<PointXYZ>::Ptr cloud_merged(new PointCloud<PointXYZ>);

    for (int i = 25; i <= 43; ++i) {
        if (i == 35)i++;
        // 定义一个点云容器来存储当前文件的点云
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

        // 构建文件名
        stringstream ss;
        ss << "result/filtered_" << i << ".pcd";
        string input_filename = ss.str();

        // 读取PCD文件
        if (io::loadPCDFile(input_filename, *cloud) == -1) {
            cerr << "Failed to load PCD file: " << input_filename << endl;
            continue; // 跳过当前文件，继续处理下一个文件
        }
        cout << "数据读入完成: " << input_filename << endl;
        cout << "Number of points in current file: " << cloud->size() << endl;

        // 将当前点云添加到融合后的点云中
        *cloud_merged += *cloud;
    }

    // 打印融合后的点云信息
    cout << "融合后的点云信息: " << endl;
    cout << "Number of points: " << cloud_merged->size() << endl;

    // 保存融合后的点云为PCD文件
    string output_filename = "result/merged.pcd";
    if (io::savePCDFileASCII(output_filename, *cloud_merged) == -1) {
        cerr << "Failed to save PCD file: " << output_filename << endl;
        return -1;
    }
    cout << "融合后的数据已保存到 " << output_filename << endl;

    return 0;
}

