#include "ControllerInterface.h" // 必须包含这个
#include "Controller.h"
#include "SMStore.h"
#include <SMObjects.h>

using namespace System;
using namespace System::Threading;

error_state Controller::processSharedMemory() {
    return error_state::SUCCESS;
}

bool Controller::getShutdownFlag() {
    auto tm = SMStore::TM;
    if (tm == nullptr) return false;
    return (tm->shutdown & bit_CONTROLLER) != 0;
}

void Controller::threadFunction() {
    // 1. 初始化手柄接口 (0 = Xbox Controller)
    ControllerInterface* xbox = new ControllerInterface(1, 0);
    auto smvc = SMStore::VC;
    auto tm = SMStore::TM;

    if (smvc == nullptr || tm == nullptr) return;

    Console::WriteLine("[Controller] Thread started. Waiting for input...");

    while (!getShutdownFlag()) {
        // 2. 读取手柄状态
        controllerState state = xbox->GetState();

        double speed = 0.0;
        double steer = 0.0;

        if (state.isConnected) {
            // 3. 计算速度 (右扳机加速，左扳机倒车)
            // 扳机值范围 0.0 -> 1.0
            double fwd = state.rightTrigger;
            double rev = state.leftTrigger;

            // 简单逻辑：前 - 后，限制在 +/- 1.0 m/s
            speed = (fwd - rev) * 1.0;

            // 4. 计算转向 (右摇杆 X 轴)
            // 摇杆范围 -1.0 -> 1.0，映射到 +/- 40 度
            steer = - state.rightThumbX * 40.0;

            // 死区处理（防止漂移，可选）
            if (Math::Abs(speed) < 0.1) speed = 0;
            if (Math::Abs(steer) < 2.0) steer = 0;

            // 5. 检查是否按下了 'Back' 键触发关机 (题目要求)
            if (state.back) {
                // 通知 TMM 关机 (这也是一种触发方式)
                // 但通常我们在 TMM 里监听键盘 'Q' 更好，这里可以备用
                tm->shutdown = 0xFF;
            }
        }
        else {
            // 手柄断开，必须停车！
            speed = 0.0;
            steer = 0.0;
            // 可选：尝试重连
            // delete xbox; xbox = new ControllerInterface(1, 0);
        }

        // 6. 写入共享内存
        Monitor::Enter(smvc->lockObject);
        try {
            smvc->Speed = speed;
            smvc->Steering = steer;
        }
        finally {
            Monitor::Exit(smvc->lockObject);
        }

        // 更新心跳
        tm->heartbeat |= bit_CONTROLLER;

        Thread::Sleep(20); // 50Hz 采样
    }

    delete xbox;
}