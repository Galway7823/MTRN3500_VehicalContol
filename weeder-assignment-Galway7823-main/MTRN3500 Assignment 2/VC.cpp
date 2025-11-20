#include "VC.h"
#include "SMStore.h"
#include <SMObjects.h>
#include <NetworkedModule.h>

using namespace System;
using namespace System::Threading;
using namespace System::Net::Sockets;
using namespace System::Text;

// 发送单行辅助函数
static void sendLine(NetworkStream^ s, String^ line) {
    array<unsigned char>^ b = Encoding::ASCII->GetBytes(line + "\n");
    s->Write(b, 0, b->Length);
    s->Flush();
}

error_state VC::processSharedMemory() {
    return error_state::SUCCESS;
}

bool VC::getShutdownFlag() {
    auto tm = SMStore::TM;
    if (tm == nullptr) return false;
    return (tm->shutdown & bit_VC) != 0;
}

// 连接逻辑可以直接复用 LiDAR 的，或者简单写一下
error_state VC::connect(String^ hostName, int portNumber) {
    try {
        if (Client != nullptr) Client->Close();
        Client = gcnew TcpClient();
        Client->Connect(hostName, portNumber);
        Stream = Client->GetStream();
        Console::WriteLine("[VC] Connected to Vehicle Control.");
        return error_state::SUCCESS;
    }
    catch (...) { return error_state::ERR_CONNECTION; }
}

error_state VC::communicate() {
    // VC 主要是发送，不是接收，所以逻辑写在 threadFunction 里更方便
    return error_state::SUCCESS;
}

void VC::threadFunction() {
    auto smvc = SMStore::VC;
    auto tm = SMStore::TM;
    int wdog = 0; // 看门狗位

    const int VC_PORT = 25000;

RECONNECT:
    if (connect(gcnew String(WEEDER_ADDRESS), VC_PORT) != error_state::SUCCESS) {
        Thread::Sleep(1000);
        goto RECONNECT;
    }

    // ★ 身份验证 (关键！)
    try {
        sendLine(Stream, "5637231"); // 替换为你的学号
        // 简单等待一下 ACK，不做复杂判断了，直接发控制指令
        Thread::Sleep(100);
    }
    catch (...) { goto RECONNECT; }

    while (!getShutdownFlag()) {
        double speed = 0;
        double steer = 0;

        // 1. 从共享内存读取指令
        Monitor::Enter(smvc->lockObject);
        try {
            speed = smvc->Speed;
            steer = smvc->Steering;
        }
        finally {
            Monitor::Exit(smvc->lockObject);
        }

        // 2. 格式化指令字符串
        // 格式: # <steer> <speed> <wdog> #
        // 例如: # 10.5 0.8 1 #
        wdog = 1 - wdog; // 0/1 翻转
        String^ cmd = String::Format("# {0:F2} {1:F2} {2} #", steer, speed, wdog);

        // 3. 发送
        try {
            sendLine(Stream, cmd);
        }
        catch (...) {
            Console::WriteLine("[VC] Send failed, reconnecting...");
            goto RECONNECT;
        }

        // 更新心跳
        tm->heartbeat |= bit_VC;

        Thread::Sleep(20); // 50Hz 发送频率
    }

    if (Client) Client->Close();
}