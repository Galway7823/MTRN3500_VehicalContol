#pragma once
#pragma once
#include <UGVModule.h>

ref class CrashAvoidance : public UGVModule {
public:
    // Send/Receive crash-avoidance flags via shared memory
    error_state processSharedMemory() override;

    // Main loop: assess LiDAR + controller data and inhibit motion if needed
    void threadFunction() override;

    // Check shutdown flag
    bool getShutdownFlag() override;
};
