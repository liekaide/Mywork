#include <iostream>
#include <vector>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

using namespace pcl;
using namespace std;

bool loadOBJ(const string& filename, PointCloud<PointXYZ>::Ptr& cloud, vector<Vertices>& faces) {
    // 读取OBJ文件
    PolygonMesh mesh;
    if (io::loadOBJFile(filename, mesh) == -1) {
        cerr << "Failed to load OBJ file: " << filename << endl;
        return false;
    }

    // 转换点云数据
    fromPCLPointCloud2(mesh.cloud, *cloud);

    // 提取面数据
    faces = mesh.polygons;

    return true;
}

void saveOBJ(const string& filename, const PointCloud<PointXYZ>::Ptr& cloud, const vector<Vertices>& faces) {
    // 创建PolygonMesh对象
    PolygonMesh mesh;
    toPCLPointCloud2(*cloud, mesh.cloud);

    // 添加面数据
    mesh.polygons = faces;

    // 保存为OBJ文件
    io::saveOBJFile(filename, mesh);
}

int main() {
    // 定义点云和面数据的容器
    PointCloud<PointXYZ>::Ptr combined_cloud(new PointCloud<PointXYZ>);
    vector<Vertices> combined_faces;

    // 要合并的OBJ文件列表
    vector<string> obj_files = { "data/26.obj", "data/28.obj", "data/27.obj" };

    // 合并每个OBJ文件
    for (const auto& file : obj_files) {
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
        vector<Vertices> faces;

        if (!loadOBJ(file, cloud, faces)) {
            return -1;
        }

        // 合并点云数据
        *combined_cloud += *cloud;

        // 合并面数据，调整面数据的索引
        int offset = combined_cloud->size() - cloud->size();
        for (auto& face : faces) {
            for (auto& vertex_index : face.vertices) {
                vertex_index += offset;
            }
            combined_faces.push_back(face);
        }
    }

    // 保存合并后的OBJ文件
    saveOBJ("result/combined.obj", combined_cloud, combined_faces);

    cout << "OBJ文件合并完成，结果已保存到result/combined.obj" << endl;

    return 0;
}
