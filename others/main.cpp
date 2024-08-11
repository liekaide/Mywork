#include <pcl/io/obj_io.h>
#include<pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/registration/icp.h>
#include <iostream>
#include<fstream>
#include<vector>
#include<string>
#include<sstream>

using namespace std;
//定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


//定义读取OBJ文件的函数
bool loadOBJ(const string&filename,PointCloud::Ptr&cloud, vector<vector<int>>&faces){
    ifstream file(filename);
    if(!file.is_open()){
        cerr<<"can't open the file: "<<filename<<endl;
        return false;
    }
    string line;
    while(getline(file,line)){
        istringstream iss(line);
        string type;
        iss>>type;
        if(type=="v"){
            PointT point;
            iss>>point.x>>point.y>>point.z;
            cloud->points.push_back(point);
        }
        else if(type=="f"){
            vector<int> face;
            int index;
            while(iss>>index){
                face.push_back(index-1);//OBJ文件的索引从1开始，PCL的索引从0开始。
            }
            faces.push_back(face);
        }
    }
    file.close();
    return true;

}
//定义保存为OBJ文件的的函数
void savecloudtoOBJ(const string& filename,const PointCloud::Ptr& cloud,const vector<vector<int>>&faces){
    ofstream file(filename);
    if(!file.is_open()){
        cerr<<"can't open the file: "<<filename<<endl;
        return;
    }

    //写入点数据
    for(const auto&point : cloud->points){
        file<<"v "<<point.x<<" "<<point.y<<" "<<point.z<<endl;
    }
    //写入面数据
    for(const auto&face:faces){
        file<<"f ";
        for(const auto& index:face){
            file<<" "<<index+1;//OBJ文件的索引从1开始,而PCL库的索引是从0开始
        }
        file<<endl;
    }  
    file.close();

}


int main(){
    
    //定义所有OBJ文件名
    vector<string>obj_files={"25.obj","26.obj","27.obj","28.obj"};
    //创建点云指针
    PointCloud::Ptr source_cloud(new PointCloud);
    PointCloud::Ptr target_cloud(new PointCloud);
    //创建存储面数据的向量
    vector<vector<int>>faces_in,faces_out;





    //读取所有点云文件
    for(const auto& file:obj_files){
        
    }
    if(!loadOBJ("data/25.obj",source_cloud,faces_in)){
        cout<<"can't open source file;"<<endl;
        return -1;
    }

    if(!loadOBJ("data/26.obj",target_cloud,faces_out)){
        cout<<"can't open source file;"<<endl;
        return -1;
    }

    //创建ICP对象
    pcl::IterativeClosestPoint<PointT,PointT>icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    //设置ICP参数
    //icp.setMaximumIterations(100);//设置最大迭代次数
    //icp.setTransformationEpsilon(1e-10);//容忍变换
    //icp.setMaxCorrespondenceDistance(0.0003);//最大对应点距离

    //定义存储变换后的点云
    PointCloud::Ptr aligned_cloud(new PointCloud);
  
    //执行ICP算法
    icp.align(*aligned_cloud);

    //检查配准是否收敛
    if (icp.hasConverged()){
        cout<<"ICP has converged,score: "<<icp.getFitnessScore()<<std::endl;
        cout<<"Transformation matrix: "<<endl<<icp.getFinalTransformation()<<endl;
        savecloudtoOBJ("result/transformed_25.obj",aligned_cloud,faces_in);
        cout<<"transformed_26.obj has been saved to result"<<endl;
    }



    return 0;
}
