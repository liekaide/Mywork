#include <iostream>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/surface/mls.h>
#include <pcl/PolygonMesh.h>
#include <boost/make_shared.hpp>

using namespace pcl;
using namespace std;

bool loadOBJ(const string& filename, PointCloud<PointXYZ>::Ptr& cloud, PolygonMesh& mesh) {
    // 读取OBJ文件
    if (io::loadOBJFile(filename, mesh) == -1) {
        cerr << "Failed to load OBJ file: " << filename << endl;
        return false;
    }

    // 转换点云数据
    fromPCLPointCloud2(mesh.cloud, *cloud);

    return true;
}

void saveOBJ(const string& filename, const PolygonMesh& mesh) {
    // 保存为OBJ文件
    io::saveOBJFile(filename, mesh);
}

void printMeshInfo(const PolygonMesh& mesh, const string& message) {
    cout << message << endl;
    cout << "Number of vertices: " << mesh.cloud.width * mesh.cloud.height << endl;
    cout << "Number of polygons: " << mesh.polygons.size() << endl;
}

int main() {
    // 定义点云和面数据的容器
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    PolygonMesh mesh;

    // 读取OBJ文件
    if (!loadOBJ("data/25.obj", cloud, mesh)) {
        return -1;
    }
    cout << "数据读入完成" << endl;

    // 打印输入网格信息
    printMeshInfo(mesh, "输入网格信息:");

    /* 滤波阶段 */
    MovingLeastSquares<PointXYZ, PointXYZ> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(0.5);
    mls.setPolynomialOrder(2); // 设置移动最小二乘的阶数
    mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::NONE);

    PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
    mls.process(*cloud_smoothed);
    cout << "移动最小二乘平面滤波完成" << endl;

    // 创建包含滤波后点云的PolygonMesh
    PolygonMesh filtered_mesh;
    toPCLPointCloud2(*cloud_smoothed, filtered_mesh.cloud);

    // 保留原始面的数据
    filtered_mesh.polygons = mesh.polygons;

    // 保存滤波后的点云为OBJ文件
    saveOBJ("result/filtered_25.obj", filtered_mesh);
    cout << "滤波后的数据已保存到result/filtered_25.obj" << endl;

    return 0;
}
