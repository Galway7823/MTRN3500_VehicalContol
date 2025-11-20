#pragma once
#include <NetworkedModule.h>

ref class VC : public NetworkedModule {
public:
    // Send/Receive vehicle control data via shared memory
    error_state processSharedMemory() override;

    // Main loop: build and send "# <steer> <speed> <wdog> #"
    void threadFunction() override;

    // Check shutdown flag
    bool getShutdownFlag() override;

    // --- Required by NetworkedModule (TCP) ---
    error_state connect(String^ hostName, int portNumber) override;
    error_state communicate() override;
};
#pragma once
