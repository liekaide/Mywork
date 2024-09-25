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
    for (int i = 26; i <= 43; ++i) {
        if (i == 35)++i;
        // ������Ƶ�����
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

        // �����ļ���
        stringstream ss;
        ss << "data/" << i << ".obj";
        string input_filename = ss.str();

        // ��ȡOBJ�ļ�
        if (io::loadOBJFile(input_filename, *cloud) == -1) {
            cerr << "Failed to load OBJ file: " << input_filename << endl;
            continue; // ������ǰ�ļ�������������һ���ļ�
        }
        cout << "���ݶ������: " << input_filename << endl;
        cout << "���������Ϣ: " << endl;
        cout << "Number of points: " << cloud->size() << endl;

        /* ƽ���׶� */
        MovingLeastSquares<PointXYZ, PointXYZ> mls;
        mls.setInputCloud(cloud);
        mls.setSearchRadius(0.5);
        mls.setPolynomialOrder(2); // �����ƶ���С���˵Ľ���
        mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::NONE);

        PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
        mls.process(*cloud_smoothed);
        cout << "�ƶ���С����ƽ���˲����" << endl;

        /* ���������˲��׶� */
        PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>());

        VoxelGrid<PointXYZ> sor;
        sor.setInputCloud(cloud_smoothed);
        sor.setLeafSize(0.1f, 0.1f, 0.1f); // ������������Ĵ�С
        sor.filter(*cloud_filtered);

        cout << "���������˲����" << endl;
        cout << "�˲��������Ϣ: " << endl;
        cout << "Number of points: " << cloud_filtered->size() << endl;

        // ��������ļ���
        stringstream ss_out;
        ss_out << "result/filtered_" << i << ".pcd";
        string output_filename = ss_out.str();

        // �����˲���ĵ���ΪPCD�ļ�
        if (io::savePCDFileASCII(output_filename, *cloud_filtered) == -1) {
            cerr << "Failed to save PCD file: " << output_filename << endl;
            continue; // ������ǰ�ļ�������������һ���ļ�
        }
        cout << "�˲���������ѱ��浽 " << output_filename << endl;
    }

    return 0;
}
