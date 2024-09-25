#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace std;

int main() {
    // ����һ�������������洢�ںϺ�ĵ���
    PointCloud<PointXYZ>::Ptr cloud_merged(new PointCloud<PointXYZ>);

    for (int i = 25; i <= 43; ++i) {
        if (i == 35)i++;
        // ����һ�������������洢��ǰ�ļ��ĵ���
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

        // �����ļ���
        stringstream ss;
        ss << "result/filtered_" << i << ".pcd";
        string input_filename = ss.str();

        // ��ȡPCD�ļ�
        if (io::loadPCDFile(input_filename, *cloud) == -1) {
            cerr << "Failed to load PCD file: " << input_filename << endl;
            continue; // ������ǰ�ļ�������������һ���ļ�
        }
        cout << "���ݶ������: " << input_filename << endl;
        cout << "Number of points in current file: " << cloud->size() << endl;

        // ����ǰ������ӵ��ںϺ�ĵ�����
        *cloud_merged += *cloud;
    }

    // ��ӡ�ںϺ�ĵ�����Ϣ
    cout << "�ںϺ�ĵ�����Ϣ: " << endl;
    cout << "Number of points: " << cloud_merged->size() << endl;

    // �����ںϺ�ĵ���ΪPCD�ļ�
    string output_filename = "result/merged.pcd";
    if (io::savePCDFileASCII(output_filename, *cloud_merged) == -1) {
        cerr << "Failed to save PCD file: " << output_filename << endl;
        return -1;
    }
    cout << "�ںϺ�������ѱ��浽 " << output_filename << endl;

    return 0;
}

