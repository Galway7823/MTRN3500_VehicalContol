#pragma once
#pragma once
#include <UGVModule.h>

ref class Controller : public UGVModule {
public:
    // Send/Receive controller state via shared memory
    error_state processSharedMemory() override;

    // Main loop: read Xbox/keyboard input and update SM
    void threadFunction() override;

    // Check shutdown flag
    bool getShutdownFlag() override;
};
