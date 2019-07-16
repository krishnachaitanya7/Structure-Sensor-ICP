#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <array>
#include <ST/CaptureSession.h>
#include <queue>
#include <iostream>

// Global Variable Declaration
std::queue <ST::DepthFrame> dp_frames;

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
            default:
                printf("Capture session event unhandled\n");
        }
    }

    void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) override {
        switch (sample.type) {
            case ST::CaptureSessionSample::Type::SynchronizedFrames:
                dp_frames.push(sample.depthFrame);
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
    settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::VGA;
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
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (count == 5){
            ST::DepthFrame last_frame = dp_frames.back();
            last_frame.saveImageAsPointCloudMesh("Test.ply");
            const float *test {last_frame.depthInMillimeters()};
            std::cout << sizeof(test) << "x" << sizeof(test[0]) << std::endl;
//            std::cout << "TimeStamp is:" << last_frame.timestamp() << std::endl;

            exit(0);
        }
        count++;


    }
}

int main() {
    run();
    return 0;
}
