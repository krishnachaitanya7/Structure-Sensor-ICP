#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

int main(){
    std::string kkc_ply = "/home/shine/CLionProjects/Structure-Sensor-ICP/cmake-build-debug/Ref1.ply";
    std::string tea_ply = "/home/shine/CLionProjects/Structure-Sensor-ICP/cmake-build-debug/Ref.ply";
    pcl::PointCloud<pcl::PointXYZ>::Ptr tea_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr test_tea_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader Reader;
    Reader.read(kkc_ply, *tea_cloud);
    Reader.read(tea_ply, *test_tea_cloud);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(tea_cloud);
    icp.setInputTarget(test_tea_cloud);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    *final = Final;
    std::cout << std::boolalpha;
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(final);
    while (!viewer.wasStopped ())
    {
        // Don't write anything here
    }
    return 0;
}