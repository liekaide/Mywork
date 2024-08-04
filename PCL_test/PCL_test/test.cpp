#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>

using namespace std;
// ��ȡOBJ�ļ�
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

// ����ת����ĵ��ƺ��浽OBJ�ļ�
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
    // �����������
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

    // ��ȡ����OBJ�ļ�
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr cloud_out(new PointCloud);
    vector<vector<int>> faces_in, faces_out;

    if (!loadOBJ("26.obj", cloud_in, faces_in) || !loadOBJ("25.obj", cloud_out, faces_out)) {
        return -1;
    }

    // ����ICPʵ��
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    // ����һ�� pcl::PointCloud<pcl::PointXYZ>ʵ�� Final ����,�洢��׼�任���Դ����
    PointCloud::Ptr Final(new PointCloud);
    icp.align(*Final);

    // �����׼���
    cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
    const Eigen::Matrix4f& matrix = icp.getFinalTransformation();
    cout << "Transformation matrix: " << endl << matrix << endl;

    // ���ڶ�������ת������һ�����Ƶ�����ϵ
    for (auto& point : cloud_out->points) {
        Eigen::Vector4f p(point.x, point.y, point.z, 1.0);
        Eigen::Vector4f p_transformed = matrix * p;
        point.x = p_transformed.x();
        point.y = p_transformed.y();
        point.z = p_transformed.z();
    }

    // ����ת����ĵ��ƺ��浽�µ�OBJ�ļ�
    saveOBJ("transformed_26.obj", Final, faces_in);

    cout << "Transformed result saved to transformed_26.obj" << endl;

    return 0;
}
