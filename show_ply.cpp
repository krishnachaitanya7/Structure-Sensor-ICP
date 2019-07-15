#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

int main(){
    std::string ply_file = "/home/shine/CLionProjects/Structure-Sensor-ICP/ply_files/KKC_face3.ply";
    pcl::PointCloud<pcl::PointXYZ>::Ptr tea_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader Reader;
    Reader.read(ply_file, *tea_cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(tea_cloud);
    while (!viewer.wasStopped ())
    {
        // Don't write anything here
    }
    return 0;
}