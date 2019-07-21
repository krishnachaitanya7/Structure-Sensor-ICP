#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <array>
#include <ST/CaptureSession.h>
#include <queue>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>


// Global Variable Declaration
ST::DepthFrame dp_frames;

struct SessionDelegate : ST::CaptureSessionDelegate {
    void captureSessionEventDidOccur(ST::CaptureSession *session, ST::CaptureSessionEventId event) override {
        printf("Received capture session event %d (%s)\n", (int)event, ST::CaptureSessionSample::toString(event));
        switch (event) {
            case ST::CaptureSessionEventId::Booting: break;
            case ST::CaptureSessionEventId::Ready:
                printf("Starting streams...\n");
                session->startStreaming();
                break;
            case ST::CaptureSessionEventId::Disconnected:
            case ST::CaptureSessionEventId::Error:
                printf("Capture session error\n");
                exit(1);

        }
    }

    void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) override {
        switch (sample.type) {
            case ST::CaptureSessionSample::Type::SynchronizedFrames:
                dp_frames = sample.depthFrame;
//                std::cout << "TimeStamp is:" << sample.depthFrame.timestamp() << std::endl;
                break;

        }
    }
};



void run() {
    ST::CaptureSessionSettings settings;
    settings.applyExpensiveCorrection = true;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.depthEnabled = true;
    settings.structureCore.visibleEnabled = false;
    settings.structureCore.infraredEnabled = true;
    settings.structureCore.accelerometerEnabled = false;
    settings.structureCore.gyroscopeEnabled = false;
    settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::_320x240;
    settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::VeryShort;

    SessionDelegate delegate;
    ST::CaptureSession session;
    session.setDelegate(&delegate);
    if (!session.startMonitoring(settings)) {
        printf("Failed to initialize capture session!\n");
        exit(1);
    }

    /* Loop forever. The SessionDelegate receives samples on a background thread
       while streaming. */
    int count {0};
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (count <= 5){
            if (count == 5) {
                ST::DepthFrame reference_frame = dp_frames;
                reference_frame.saveImageAsPointCloudMesh("Ref.ply");
                std::cout << "Reference Frame Written, You can move your head now" << std::endl;
            }

        }
        else{

            std::string kkc_ply = "/home/shine/CLionProjects/Structure-Sensor-ICP/cmake-build-debug/Ref.ply";
            std::string tea_ply = "/home/shine/CLionProjects/Structure-Sensor-ICP/cmake-build-debug/Ref1.ply";
            remove("/home/shine/CLionProjects/Structure-Sensor-ICP/cmake-build-debug/Ref1.ply");
            dp_frames.saveImageAsPointCloudMesh("Ref1.ply");
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
        }
        count++;



    }
}



int main() {
    run();
    return 0;
}
