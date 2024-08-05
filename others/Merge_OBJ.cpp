#include<pcl/point_cloud.h>
#include <pcl/point_types.h>
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

int main() {

    // 定义一个存储所有点云的容器
    PointCloud::Ptr combined_cloud(new PointCloud);
    vector<vector<int>> combined_faces;

    // 定义多个OBJ文件路径
    vector<string> obj_files = {"data/25.obj", "data/26.obj", "data/27.obj","data/28.obj","data/29.obj","data/30.obj","data/31.obj"};

    // 读取每个OBJ文件并合并
    int offset = 0;
    for (const auto& file : obj_files) {
        PointCloud::Ptr cloud(new PointCloud);
        vector<vector<int>> faces;
        if (!loadOBJ(file, cloud, faces)) {
            return -1;
        }

        // 合并点云
        *combined_cloud += *cloud;

        // 合并面数据，并更新索引
        for (const auto& face : faces) {
            vector<int> new_face;
            for (int idx : face) {
                new_face.push_back(idx + offset);
            }
            combined_faces.push_back(new_face);
        }

        // 更新索引偏移
        offset += cloud->points.size();
        cout<<"finish: "<<file<<endl;
    }

    // 保存合并后的点云和面数据到新的OBJ文件
    string output_file = "result/combined.obj";
    savecloudtoOBJ(output_file, combined_cloud, combined_faces);

    cout << "Successfully combined and saved the OBJ files into " << output_file << endl;
    return 0;
}