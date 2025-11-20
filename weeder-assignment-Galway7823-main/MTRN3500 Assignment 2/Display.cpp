#include "Display.h"
#include "SMStore.h"
#include <SMObjects.h>
#include <NetworkedModule.h>

using namespace System;
using namespace System::Threading;
using namespace System::Net::Sockets;

// ====== 将 361×double 的 x/y 打包并发送到 MATLAB ======
static void sendDisplayData(array<double>^ xData, array<double>^ yData, NetworkStream^ stream) {
    if (xData == nullptr || yData == nullptr || stream == nullptr) return;
    if (xData->Length != STANDARD_LIDAR_LENGTH || yData->Length != STANDARD_LIDAR_LENGTH) return;

    // 按附录示例：double 数组 → 原始字节 → 依次写出
    array<Byte>^ dataX = gcnew array<Byte>(xData->Length * sizeof(double));
    Buffer::BlockCopy(xData, 0, dataX, 0, dataX->Length);

    array<Byte>^ dataY = gcnew array<Byte>(yData->Length * sizeof(double));
    Buffer::BlockCopy(yData, 0, dataY, 0, dataY->Length);

    stream->Write(dataX, 0, dataX->Length);
    // 给 MATLAB 消费一点时间（避免两块粘一起被误读）
    Thread::Sleep(10);
    stream->Write(dataY, 0, dataY->Length);
}

error_state Display::processSharedMemory() {
    // 本模块不直接修改共享内存；返回 SUCCESS 即可
    return error_state::SUCCESS;
}

bool Display::getShutdownFlag() {
    auto tm = SMStore::TM;
    if (tm == nullptr) return false;
    return (tm->shutdown & bit_DISPLAY) != 0;
}

error_state Display::connect(String^ hostName, int portNumber) {
    try {
        if (Client != nullptr) { try { Client->Close(); } catch (...) {} Client = nullptr; }
        Client = gcnew TcpClient();
        Client->NoDelay = true;
        Client->ReceiveTimeout = 1000;  // MATLAB 主要是接收端，读超时影响不大
        Client->SendTimeout = 1000;

        Client->Connect(hostName, portNumber);
        Stream = Client->GetStream();
        Stream->ReadTimeout = 1000;
        Stream->WriteTimeout = 1000;

        Console::WriteLine("[Display] Connected to MATLAB at {0}:{1}", hostName, portNumber);
        return error_state::SUCCESS;
    }
    catch (Exception^ ex) {
        Console::WriteLine("[Display] connect() failed: {0}", ex->Message);
        return error_state::ERR_SM;
    }
}

error_state Display::communicate() {
    if (Stream == nullptr) return error_state::ERR_SM;

    // 从共享内存复制一份 361×(x,y)
    auto sm = SMStore::Lidar;
    if (sm == nullptr) return error_state::ERR_SM;

    array<double>^ x = gcnew array<double>(STANDARD_LIDAR_LENGTH);
    array<double>^ y = gcnew array<double>(STANDARD_LIDAR_LENGTH);

    Monitor::Enter(sm->lockObject);
    try {
        for (int i = 0; i < STANDARD_LIDAR_LENGTH; ++i) {
            x[i] = sm->x[i];
            y[i] = sm->y[i];
        }
    }
    finally {
        Monitor::Exit(sm->lockObject);
    }

    // 发给 MATLAB
    try {
        sendDisplayData(x, y, Stream);
    }
    catch (Exception^ ex) {
        Console::WriteLine("[Display] send failed: {0}", ex->Message);
        return error_state::ERR_SM;
    }
    return error_state::SUCCESS;
}

void Display::threadFunction() {
    auto tm = SMStore::TM;
    if (tm == nullptr) return;

    // 将此端口改为你 MATLAB 监听的实际端口
    const int DISPLAY_PORT = 28000;

RECONNECT:
    if (connect(gcnew String(DISPLAY_ADDRESS), DISPLAY_PORT) != error_state::SUCCESS) {
        Thread::Sleep(500);
        goto RECONNECT;
    }

    // 主循环：~20Hz 推送一帧
    while (!getShutdownFlag()) {
        if (communicate() == error_state::SUCCESS) {
            tm->heartbeat |= bit_DISPLAY;
        }
        else {
            // 掉线自动重连
            try { if (Stream) Stream->Close(); }
            catch (...) {}
            try { if (Client) Client->Close(); }
            catch (...) {}
            Thread::Sleep(300);
            goto RECONNECT;
        }
        Thread::Sleep(50); // 约 20Hz
    }

    try { if (Stream) Stream->Close(); }
    catch (...) {}
    try { if (Client) Client->Close(); }
    catch (...) {}
}
