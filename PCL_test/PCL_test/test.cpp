#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>

using namespace std;
// 读取OBJ文件
bool loadOBJ(const string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<vector<int>>& faces) {
    ifstream infile(filename);
    if (!infile.is_open()) {
        cerr << "Couldn't read file " << filename << endl;
        return false;
    }

    string line;
    while (getline(infile, line)) {
        if (line.substr(0, 2) == "v ") {
            istringstream s(line.substr(2));
            pcl::PointXYZ point;
            s >> point.x; s >> point.y; s >> point.z;
            cloud->push_back(point);
        }
        else if (line.substr(0, 2) == "f ") {
            istringstream s(line.substr(2));
            vector<int> face;
            int vertex_index;
            char slash;
            while (s >> vertex_index) {
                face.push_back(vertex_index);
                if (s.peek() == '/') {
                    s.ignore();
                    if (s.peek() == '/') s.ignore();
                    else s >> vertex_index;
                }
            }
            faces.push_back(face);
        }
    }

    return true;
}

// 保存转换后的点云和面到OBJ文件
void saveOBJ(const string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const vector<vector<int>>& faces) {
    ofstream outfile(filename);
    if (!outfile.is_open()) {
        cerr << "Couldn't write file " << filename << endl;
        return;
    }

    for (const auto& point : cloud->points) {
        outfile << "v " << point.x << " " << point.y << " " << point.z << endl;
    }

    for (const auto& face : faces) {
        outfile << "f";
        for (const auto& index : face) {
            outfile << " " << index;
        }
        outfile << endl;
    }
}

int main() {
    // 定义点云类型
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

    // 读取两个OBJ文件
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr cloud_out(new PointCloud);
    vector<vector<int>> faces_in, faces_out;

    if (!loadOBJ("26.obj", cloud_in, faces_in) || !loadOBJ("25.obj", cloud_out, faces_out)) {
        return -1;
    }

    // 创建ICP实例
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    // 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云
    PointCloud::Ptr Final(new PointCloud);
    icp.align(*Final);

    // 输出配准结果
    cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
    const Eigen::Matrix4f& matrix = icp.getFinalTransformation();
    cout << "Transformation matrix: " << endl << matrix << endl;

    // 将第二个点云转换到第一个点云的坐标系
    for (auto& point : cloud_out->points) {
        Eigen::Vector4f p(point.x, point.y, point.z, 1.0);
        Eigen::Vector4f p_transformed = matrix * p;
        point.x = p_transformed.x();
        point.y = p_transformed.y();
        point.z = p_transformed.z();
    }

    // 保存转换后的点云和面到新的OBJ文件
    saveOBJ("transformed_26.obj", Final, faces_in);

    cout << "Transformed result saved to transformed_26.obj" << endl;

    return 0;
}
