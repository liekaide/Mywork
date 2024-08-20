#include <iostream>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


using namespace pcl;
using namespace std;

int main()
{


    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    if (io::loadOBJFile<PointXYZ>("data/25.obj", *cloud) == -1) {
        cout << "���ݶ���ʧ�ܣ���" << endl;
        return 1;
    }
    cout << "���ݶ������" << endl;


   
    /*�˲��׶�*/
    // MovingLeastSquares<PointXYZ, PointXYZ> mls;
    // mls.setInputCloud(filtered);
    // mls.setSearchRadius(0.01);
    // mls.setPolynomialFit(true);
    // mls.setPolynomialOrder(2);
    // mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
    // mls.setUpsamplingRadius(0.005);
    // mls.setUpsamplingStepSize(0.003);

    // PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ>());
    // mls.process(*cloud_smoothed);
    // cout << "�ƶ���С����ƽ���˲����" << endl;



    /*�������׶�*/
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setNumberOfThreads(8);//������8���߳������ټ���.
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(5);//���÷��߼���ʱ���õİ뾶
    Eigen::Vector4f centroid;
    compute3DCentroid(*cloud, centroid);//������Ƶ�����
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);//��������Ϊ�ӵ㣬�������Ա�֤����һ��

    PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());//�������ߵ���
    ne.compute(*cloud_normals);//���㷨��

   /* for (size_t i = 0; i < cloud_normals->size(); ++i) {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }*/


    PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
    //���������ݵ�����ͷ�����Ϣƴ��
    concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);

    cout << "������㡡�������" << endl;



    /*poission �ؽ��׶�*/
    //����poisson�ؽ�����
    Poisson<PointNormal> poisson;
    // poisson.setDepth(9);
    //����poisson�ؽ���������
    poisson.setInputCloud(cloud_smoothed_normals);
    //�����������ָ�룬���ڴ洢�ؽ����
    PolygonMesh mesh;
    //poisson�ؽ���ʼ
    poisson.reconstruct(mesh);

    //���ؽ�����洢��Ӳ�̣�������ΪPLY��ʽ
    io::saveOBJFile("result/reconstructed_25.obj", mesh);
    cout << "�����ؽ����������" << endl;


    /*ͼ����ʾ�׶�*/
    cout << "��ʼͼ����ʾ......" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("my viewer"));

    viewer->setBackgroundColor(0, 0, 7);
    viewer->addPolygonMesh(mesh, "my");
    viewer->addCoordinateSystem(50.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {

        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }


    return (0);
}

