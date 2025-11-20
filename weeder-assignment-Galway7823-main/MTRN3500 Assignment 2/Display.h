#pragma once
#pragma once
#include <NetworkedModule.h>

ref class Display : public NetworkedModule {
public:
    // Send/Receive display data via shared memory
    error_state processSharedMemory() override;

    // Main loop: publish LiDAR/GNSS data to MATLAB
    void threadFunction() override;

    // Check shutdown flag
    bool getShutdownFlag() override;

    // --- Required by NetworkedModule (TCP) ---
    error_state connect(String^ hostName, int portNumber) override;
    error_state communicate() override;
};
