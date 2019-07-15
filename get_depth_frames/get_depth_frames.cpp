#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <ST/CaptureSession.h>

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
                break;
            default:
                printf("Capture session event unhandled\n");
        }
    }

    void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) override {
        char ply_save_path {'t'};
        switch (sample.type) {
            case ST::CaptureSessionSample::Type::DepthFrame:
                printf("Depth frame: size %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height());
                break;
            case ST::CaptureSessionSample::Type::SynchronizedFrames:
                printf("Synchronized frame: size %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height());
                sample.depthFrame.saveImageToPngFile(&ply_save_path);
                exit(0);
            default:
                printf("Is printf working?");
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
//    settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::VGA;
    SessionDelegate delegate;
    ST::CaptureSession session;
    session.setDelegate(&delegate);
    if (!session.startMonitoring(settings)) {
        printf("Failed to initialize capture session!\n");
        exit(1);
    }

    /* Loop forever. The SessionDelegate receives samples on a background thread
       while streaming. */
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(void) {
    run();
    return 0;
}
