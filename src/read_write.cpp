#include"read_write.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>

using namespace std;




// 定义读取OBJ文件的函数，并同时建立索引
bool loadOBJ(const string& filename, PointCloud::Ptr& cloud, vector<vector<int>>& faces, vector<int>& point_indices) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "can't open the file: " << filename << endl;
        return false;
    }

    string line;
    int current_index = 0;  // 用于记录点的索引
    while (getline(file, line)) {
        istringstream iss(line);
        string type;
        iss >> type;
        if (type == "v") {
            PointT point;
            iss >> point.x >> point.y >> point.z;
            cloud->points.push_back(point);
            point_indices.push_back(current_index);  // 记录当前点的索引
            current_index++;
        } else if (type == "f") {
            vector<int> face;
            int index;
            while (iss >> index) {
                face.push_back(index - 1);  // OBJ 文件的索引从 1 开始，PCL 的索引从 0 开始。
            }
            faces.push_back(face);
        }
    }
    file.close();
    return true;
}

// 更新面数据
void updateFaces(PointCloud::Ptr& filtered_cloud, vector<vector<int>>& faces, const vector<int>& original_indices) {
    // 创建一个新的面数据存储
    vector<vector<int>> updated_faces;

    // 创建一个映射，跟踪点的原始索引到新的索引
    unordered_map<int, int> index_map;
    for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
        index_map[original_indices[i]] = i;
    }

    // 遍历所有面并更新索引
    for (const auto& face : faces) {
        vector<int> updated_face;
        bool valid_face = true;

        for (int index : face) {
            if (index_map.find(index) != index_map.end()) {
                updated_face.push_back(index_map[index]);
            } else {
                valid_face = false;
                break;  // 如果面中有一个顶点无效，则这个面无效
            }
        }

        if (valid_face) {
            updated_faces.push_back(updated_face);
        }
    }

    // 用更新后的面数据替换原始面数据
    faces.swap(updated_faces);
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
