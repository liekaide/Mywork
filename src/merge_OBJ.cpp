#include <iostream>
#include <vector>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

using namespace pcl;
using namespace std;

bool loadOBJ(const string& filename, PointCloud<PointXYZ>::Ptr& cloud, vector<Vertices>& faces) {
    // ��ȡOBJ�ļ�
    PolygonMesh mesh;
    if (io::loadOBJFile(filename, mesh) == -1) {
        cerr << "Failed to load OBJ file: " << filename << endl;
        return false;
    }

    // ת����������
    fromPCLPointCloud2(mesh.cloud, *cloud);

    // ��ȡ������
    faces = mesh.polygons;

    return true;
}

void saveOBJ(const string& filename, const PointCloud<PointXYZ>::Ptr& cloud, const vector<Vertices>& faces) {
    // ����PolygonMesh����
    PolygonMesh mesh;
    toPCLPointCloud2(*cloud, mesh.cloud);

    // ���������
    mesh.polygons = faces;

    // ����ΪOBJ�ļ�
    io::saveOBJFile(filename, mesh);
}

int main() {
    // ������ƺ������ݵ�����
    PointCloud<PointXYZ>::Ptr combined_cloud(new PointCloud<PointXYZ>);
    vector<Vertices> combined_faces;

    // Ҫ�ϲ���OBJ�ļ��б�
    vector<string> obj_files = { "data/26.obj", "data/28.obj", "data/27.obj" };

    // �ϲ�ÿ��OBJ�ļ�
    for (const auto& file : obj_files) {
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
        vector<Vertices> faces;

        if (!loadOBJ(file, cloud, faces)) {
            return -1;
        }

        // �ϲ���������
        *combined_cloud += *cloud;

        // �ϲ������ݣ����������ݵ�����
        int offset = combined_cloud->size() - cloud->size();
        for (auto& face : faces) {
            for (auto& vertex_index : face.vertices) {
                vertex_index += offset;
            }
            combined_faces.push_back(face);
        }
    }

    // ����ϲ����OBJ�ļ�
    saveOBJ("result/combined.obj", combined_cloud, combined_faces);

    cout << "OBJ�ļ��ϲ���ɣ�����ѱ��浽result/combined.obj" << endl;

    return 0;
}
