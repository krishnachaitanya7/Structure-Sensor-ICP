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
// For z smoothing
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <iterator>
#include <numeric>
using namespace std;
// Z smoothing code starts
typedef long double ld;
typedef unsigned int uint;
typedef std::vector<ld>::iterator vec_iter_ld;

class VectorStats {
public:
    /**
     * Constructor for VectorStats class.
     *
     * @param start - This is the iterator position of the start of the window,
     * @param end   - This is the iterator position of the end of the window,
     */
    VectorStats(vec_iter_ld start, vec_iter_ld end) {
        this->start = start;
        this->end = end;
        this->compute();
    }

    /**
     * This method calculates the mean and standard deviation using STL function.
     * This is the Two-Pass implementation of the Mean & Variance calculation.
     */
    void compute() {
        ld sum = std::accumulate(start, end, 0.0);
        uint slice_size = std::distance(start, end);
        ld mean = sum / slice_size;
        std::vector<ld> diff(slice_size);
        std::transform(start, end, diff.begin(), [mean](ld x) { return x - mean; });
        ld sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        ld std_dev = std::sqrt(sq_sum / slice_size);

        this->m1 = mean;
        this->m2 = std_dev;
    }

    ld mean() {
        return m1;
    }

    ld standard_deviation() {
        return m2;
    }

private:
    vec_iter_ld start;
    vec_iter_ld end;
    ld m1;
    ld m2;
};

/**
 * This is the implementation of the Smoothed Z-Score Algorithm.
 * This is direction translation of https://stackoverflow.com/a/22640362/1461896.
 *
 * @param input - input signal
 * @param lag - the lag of the moving window
 * @param threshold - the z-score at which the algorithm signals
 * @param influence - the influence (between 0 and 1) of new signals on the mean and standard deviation
 * @return a hashmap containing the filtered signal and corresponding mean and standard deviation.
 */
unordered_map<string, vector<ld>> z_score_thresholding(vector<ld> input, int lag, ld threshold, ld influence) {
    unordered_map<string, vector<ld>> output;

    uint n = (uint) input.size();
    vector<ld> signals(input.size());
    vector<ld> filtered_input(input.begin(), input.end());
    vector<ld> filtered_mean(input.size());
    vector<ld> filtered_stddev(input.size());

    VectorStats lag_subvector_stats(input.begin(), input.begin() + lag);
    filtered_mean[lag - 1] = lag_subvector_stats.mean();
    filtered_stddev[lag - 1] = lag_subvector_stats.standard_deviation();

    for (int i = lag; i < n; i++) {
        if (abs(input[i] - filtered_mean[i - 1]) > threshold * filtered_stddev[i - 1]) {
            signals[i] = (input[i] > filtered_mean[i - 1]) ? 1.0 : -1.0;
            filtered_input[i] = influence * input[i] + (1 - influence) * filtered_input[i - 1];
        } else {
            signals[i] = 0.0;
            filtered_input[i] = input[i];
        }
        VectorStats lag_subvector_stats(filtered_input.begin() + (i - lag), filtered_input.begin() + i);
        filtered_mean[i] = lag_subvector_stats.mean();
        filtered_stddev[i] = lag_subvector_stats.standard_deviation();
    }

    output["signals"] = signals;
    output["filtered_mean"] = filtered_mean;
    output["filtered_stddev"] = filtered_stddev;

    return output;
};
// End Z Score implementation

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
    vector<ld> input;
    int lag = 20;
    ld threshold = 3.0;
    ld influence = 1.0;

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (count <= 5){
            if (count == 5) {
                ST::DepthFrame reference_frame = dp_frames;
                remove("/home/shine/CLionProjects/Structure-Sensor-ICP/cmake-build-debug/Ref.ply");
                reference_frame.saveImageAsPointCloudMesh("Ref.ply");
                std::cout << "Reference Frame Written, You can move your head now" << std::endl;
            }

        }
        else{

            std::string kkc_ply = "/home/shine/CLionProjects/Structure-Sensor-ICP/cmake-build-debug/Ref.ply";
            std::string tea_ply = "/home/shine/CLionProjects/Structure-Sensor-ICP/cmake-build-debug/Ref1.ply";
            remove("/home/shine/CLionProjects/Structure-Sensor-ICP/cmake-build-debug/Ref1.ply");
            dp_frames.saveImageAsPointCloudMesh("/home/shine/CLionProjects/Structure-Sensor-ICP/cmake-build-debug/Ref1.ply");
            pcl::PointCloud<pcl::PointXYZ>::Ptr tea_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr test_tea_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PLYReader Reader;
            Reader.read(kkc_ply, *tea_cloud);
            Reader.read(tea_ply, *test_tea_cloud);
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setTransformationEpsilon(1e-2);
            icp.setMaximumIterations(100);
//            icp.setMaxCorrespondenceDistance(0.3);
            icp.setEuclideanFitnessEpsilon (1e-8); //1e-5, 1e-8
            icp.setInputSource(tea_cloud);
            icp.setInputTarget(test_tea_cloud);
            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);
//            pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
//            *final = Final;
            std::cout << std::boolalpha;
            if (count < 40) {
                std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                          icp.getFitnessScore() << std::endl;
                //            std::cout << icp.getFinalTransformation() << std::endl;
                input.push_back(icp.getFitnessScore());
            } else{

                input.back() = icp.getFitnessScore();
                unordered_map<string, vector<ld>> output = z_score_thresholding(input, lag, threshold, influence);
                std::cout << // "has converged:" << icp.hasConverged() << " score: " <<
//                          icp.getFitnessScore() <<
                          " Moved or not: " << output["signals"].back() << std::endl;

            }

        }
        count++;



    }
}



int main() {
    run();
    return 0;
}
