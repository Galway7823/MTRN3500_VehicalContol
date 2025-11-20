#include "LiDAR.h"
#include "SMStore.h"
#include <SMObjects.h>
#include <NetworkedModule.h>
#include <cmath>

using namespace System;
using namespace System::Text;
using namespace System::Threading;
using namespace System::Net::Sockets;
using namespace System::Net;
using namespace System::IO;

// ================= 工具函数 =================
static inline double deg2rad(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}

// 发送一行 ASCII（自动加 \n）
static void sendLine(NetworkStream^ s, String^ line) {
    auto b = Encoding::ASCII->GetBytes(line + "\n");
    s->Write(b, 0, b->Length);
    s->Flush();
}

// 发送命令（STX 0x02 + cmd + ETX 0x03）
static void sendLidarCmd(NetworkStream^ s, String^ cmd) {
    array<unsigned char>^ stx = gcnew array<unsigned char>(1) { 0x02 };
    array<unsigned char>^ etx = gcnew array<unsigned char>(1) { 0x03 };
    auto body = Encoding::ASCII->GetBytes(cmd);
    s->Write(stx, 0, 1);
    s->Write(body, 0, body->Length);
    s->Write(etx, 0, 1);
    s->Flush();
}

// 读取直到 ETX(0x03)，返回 STX..ETX 之间的 ASCII（不含 STX/ETX）
static String^ readUntilETX(NetworkStream^ s, int timeout_ms = 2000) {
    array<unsigned char>^ buf = gcnew array<unsigned char>(8192);
    MemoryStream^ ms = gcnew MemoryStream();
    DateTime t0 = DateTime::Now;
    bool started = false;

    while ((DateTime::Now - t0).TotalMilliseconds < timeout_ms) {
        int n = 0;
        try { n = s->Read(buf, 0, buf->Length); }
        catch (IOException^) { Thread::Sleep(10); continue; }
        catch (...) { break; }

        if (n <= 0) { Thread::Sleep(5); continue; }

        for (int i = 0; i < n; ++i) {
            unsigned char b = buf[i];
            if (!started) {
                if (b == 0x02) started = true;           // STX
            }
            else {
                if (b == 0x03) {                         // ETX
                    auto bytes = ms->ToArray();
                    return Encoding::ASCII->GetString(bytes);
                }
                else {
                    ms->WriteByte(b);
                }
            }
        }
    }
    return String::Empty;
}

// 将 token 解析为距离（单位：毫米）
// 若含 A-F 则视为十六进制 mm；否则按十进制 mm
static double parseDistance(String^ token) {
    token = token->Trim();
    if (token->Length == 0) return 0.0;

    double v;
    if (Double::TryParse(token, v)) {
        if (v >= 0 && std::isfinite(v)) return v;  // 已是 mm
    }

    bool hasHex = false;
    for each (wchar_t c in token)
        if ((c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) { hasHex = true; break; }

    if (hasHex) {
        try {
            int mm = Convert::ToInt32(token, 16);  // 十六进制毫米
            if (mm < 0) mm = 0;
            return mm * 1.0;                       // 保留毫米，不再除 1000
        }
        catch (...) { return 0.0; }
    }
    return 0.0;
}

// ================= UGVModule 接口实现 =================

error_state LiDAR::processSharedMemory()
{
    auto sm = SMStore::Lidar;
    if (sm == nullptr) return error_state::ERR_SM;

    Monitor::Enter(sm->lockObject);
    try {
        // 打印 (x, y)，单位为毫米
        for (int i = 0; i < STANDARD_LIDAR_LENGTH; ++i) {
            Console::WriteLine("({0:F1}, {1:F1})", sm->x[i], sm->y[i]);
        }
        Console::WriteLine("---- 361 points printed ----");
    }
    finally {
        Monitor::Exit(sm->lockObject);
    }
    return error_state::SUCCESS;
}

void LiDAR::threadFunction()
{
    auto sm = SMStore::Lidar;
    auto smtm = SMStore::TM;
    if (sm == nullptr || smtm == nullptr) return;

    const int lidarPort = 23000;

RECONNECT:
    if (connect(gcnew String(WEEDER_ADDRESS), lidarPort) != error_state::SUCCESS) {
        Console::WriteLine("LiDAR connect() failed. Retry in 1s...");
        Thread::Sleep(1000);
        goto RECONNECT;
    }
    Console::WriteLine("LiDAR thread started. Connected to {0}:{1}", gcnew String(WEEDER_ADDRESS), lidarPort);

    // 认证：发送学号（模拟器也要求）
    try {
        String^ studentID = "5637231";   // ← 改为你自己的学号
        sendLine(Stream, studentID);

        // 等待 OK
        String^ ack = String::Empty;
        {
            array<unsigned char>^ tmp = gcnew array<unsigned char>(256);
            DateTime t0 = DateTime::Now;
            while ((DateTime::Now - t0).TotalMilliseconds < 1500) {
                if (Stream->DataAvailable) {
                    int n = Stream->Read(tmp, 0, tmp->Length);
                    if (n > 0) {
                        ack += Encoding::ASCII->GetString(tmp, 0, n);
                        if (ack->IndexOf("OK", StringComparison::OrdinalIgnoreCase) >= 0) break;
                    }
                }
                else { Thread::Sleep(20); }
            }
        }
        Console::WriteLine("[LiDAR] auth resp: '{0}'", ack);
        if (ack->IndexOf("OK", StringComparison::OrdinalIgnoreCase) < 0) {
            Console::WriteLine("[LiDAR] auth failed, reconnecting...");
            try { if (Stream) Stream->Close(); }
            catch (...) {}
            try { if (Client) Client->Close(); }
            catch (...) {}
            Thread::Sleep(500);
            goto RECONNECT;
        }
    }
    catch (Exception^ ex) {
        Console::WriteLine("[LiDAR] auth exception: {0}", ex->Message);
        goto RECONNECT;
    }

    // 暖机：先抓一帧（可选）
    try {
        sendLidarCmd(Stream, "sRN LMDscandata");
        String^ warm = readUntilETX(Stream, 2000);
        Console::WriteLine("[LiDAR] warmup payload len={0}", warm == nullptr ? 0 : warm->Length);
    }
    catch (...) {}

    // 主循环
    while (!getShutdownFlag()) {
        if (communicate() == error_state::SUCCESS) {
            processSharedMemory();
            smtm->heartbeat |= bit_LIDAR;
        }
        else {
            Thread::Sleep(30);
        }
    }

    try { if (Stream) Stream->Close(); }
    catch (...) {}
    try { if (Client) Client->Close(); }
    catch (...) {}
}

bool LiDAR::getShutdownFlag()
{
    auto smtm = SMStore::TM;
    if (smtm == nullptr) return false;
    return (smtm->shutdown & bit_LIDAR) != 0;
}

// ================= NetworkedModule 接口实现 =================

error_state LiDAR::connect(String^ hostName, int portNumber)
{
    try {
        if (Client != nullptr) { try { Client->Close(); } catch (...) {} Client = nullptr; }

        Client = gcnew TcpClient();
        Client->NoDelay = true;
        Client->ReceiveTimeout = 1500;
        Client->SendTimeout = 200;

        Client->Connect(hostName, portNumber);
        Stream = Client->GetStream();
        Stream->ReadTimeout = 1500;
        Stream->WriteTimeout = 200;

        ReadData = gcnew array<unsigned char>(8192);

        try {
            auto lep = (IPEndPoint^)Client->Client->LocalEndPoint;
            auto rep = (IPEndPoint^)Client->Client->RemoteEndPoint;
            Console::WriteLine("[LiDAR] Local {0}:{1} -> Remote {2}:{3}",
                lep->Address, lep->Port, rep->Address, rep->Port);
        }
        catch (...) {}

        return error_state::SUCCESS;
    }
    catch (Exception^ ex) {
        Console::WriteLine("connect() exception: {0}", ex->Message);
        return error_state::ERR_SM;
    }
}

// ================= 主通信：定位 DIST1，稳定解析 =================

error_state LiDAR::communicate()
{
    if (Stream == nullptr) return error_state::ERR_SM;

    // 发送扫描请求
    try {
        sendLidarCmd(Stream, "sRN LMDscandata");
    }
    catch (Exception^ ex) {
        Console::WriteLine("[LiDAR] send failed: {0}", ex->Message);
        return error_state::ERR_SM;
    }

    // 读取完整帧
    String^ payload = readUntilETX(Stream, 2000);
    if (String::IsNullOrEmpty(payload)) {
        return error_state::ERR_SM;
    }

    // 拆分
    auto tk = payload->Split(gcnew array<wchar_t>{' ', '\t'}, StringSplitOptions::RemoveEmptyEntries);
    int T = tk->Length;

    // 寻找 "DIST1"
    int iDist = -1;
    for (int i = 0; i < T; ++i) {
        if (String::Compare(tk[i], "DIST1", StringComparison::OrdinalIgnoreCase) == 0) {
            iDist = i; break;
        }
    }
    if (iDist < 0) {
        if (T >= 380) {
            iDist = T - 360 - 1;
        }
        else {
            return error_state::ERR_SM;
        }
    }

    int iCount = iDist + 1;
    if (iCount >= T) return error_state::ERR_SM;

    int count = 0;
    try { count = Convert::ToInt32(tk[iCount], 16); }
    catch (...) {
        int tmp; if (Int32::TryParse(tk[iCount], tmp)) count = tmp;
    }
    if (count <= 0 || count > 1000) count = 360;

    int iData = iCount + 1;
    if (iData + count > T) return error_state::ERR_SM;

    // 将距离写入共享内存（单位 mm）
    auto sm = SMStore::Lidar;
    if (sm == nullptr) return error_state::ERR_SM;

    double stepDeg = (count > 1) ? (180.0 / (count - 1)) : 180.0;
    int useN = Math::Min(count, STANDARD_LIDAR_LENGTH);

    Monitor::Enter(sm->lockObject);
    try {
        for (int i = 0; i < STANDARD_LIDAR_LENGTH; ++i) { sm->x[i] = 0.0; sm->y[i] = 0.0; }
        for (int i = 0; i < useN; ++i) {
            String^ tok = tk[iData + i];
            double r = parseDistance(tok);   // 单位 mm
            if (!(r >= 0 && std::isfinite(r))) r = 0.0;

            double theta = deg2rad(stepDeg * i);
            sm->x[i] = r * std::cos(theta);  // 仍是 mm
            sm->y[i] = r * std::sin(theta);
        }
    }
    finally {
        Monitor::Exit(sm->lockObject);
    }

    return error_state::SUCCESS;
}
