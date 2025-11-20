#pragma once
#include <NetworkedModule.h>

ref class GNSS : public NetworkedModule {
public:
    // Send/Receive data via shared memory
    error_state processSharedMemory() override;

    // Main thread loop
    void threadFunction() override;

    // Check shutdown flag
    bool getShutdownFlag() override;

    // --- Required by NetworkedModule (TCP) ---
    error_state connect(String^ hostName, int portNumber) override;
    error_state communicate() override;
};
#pragma once
