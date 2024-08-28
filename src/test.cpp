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
    // ��ȡOBJ�ļ�
    if (io::loadOBJFile(filename, mesh) == -1) {
        cerr << "Failed to load OBJ file: " << filename << endl;
        return false;
    }

    // ת����������
    fromPCLPointCloud2(mesh.cloud, *cloud);

    return true;
}

void saveOBJ(const string& filename, const PolygonMesh& mesh) {
    // ����ΪOBJ�ļ�
    io::saveOBJFile(filename, mesh);
}

void printMeshInfo(const PolygonMesh& mesh, const string& message) {
    cout << message << endl;
    cout << "Number of vertices: " << mesh.cloud.width * mesh.cloud.height << endl;
    cout << "Number of polygons: " << mesh.polygons.size() << endl;
}

int main() {
    // ������ƺ������ݵ�����
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    PolygonMesh mesh;

    // ��ȡOBJ�ļ�
    if (!loadOBJ("data/25.obj", cloud, mesh)) {
        return -1;
    }
    cout << "���ݶ������" << endl;

    // ��ӡ����������Ϣ
    printMeshInfo(mesh, "����������Ϣ:");

    /* �˲��׶� */
    MovingLeastSquares<PointXYZ, PointXYZ> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(0.5);
    mls.setPolynomialOrder(2); // �����ƶ���С���˵Ľ���
    mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::NONE);

    PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
    mls.process(*cloud_smoothed);
    cout << "�ƶ���С����ƽ���˲����" << endl;

    // ���������˲�����Ƶ�PolygonMesh
    PolygonMesh filtered_mesh;
    toPCLPointCloud2(*cloud_smoothed, filtered_mesh.cloud);

    // ����ԭʼ�������
    filtered_mesh.polygons = mesh.polygons;

    // �����˲���ĵ���ΪOBJ�ļ�
    saveOBJ("result/filtered_25.obj", filtered_mesh);
    cout << "�˲���������ѱ��浽result/filtered_25.obj" << endl;

    return 0;
}
