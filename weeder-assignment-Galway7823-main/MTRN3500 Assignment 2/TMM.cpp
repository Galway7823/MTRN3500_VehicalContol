#include "TMM.h"
#include "LiDAR.h"
#include "Display.h"
#include "VC.h"           // ★ 新增：车辆控制模块
#include "Controller.h"   // ★ 新增：手柄控制模块
//#include "GNSS.h"         // ★ 新增：GNSS模块
#include "SMStore.h"      // 包含 SMStore 定义
#include <SMObjects.h>

using namespace System;
using namespace System::Threading;

// ============ ThreadManagement 实现 ============

error_state ThreadManagement::setupSharedMemory() {
    // 触发一次访问确保已初始化
    (void)SMStore::Lidar;
    (void)SMStore::TM;
    // ★ 确保其他共享内存也被创建
    (void)SMStore::VC;
    //(void)SMStore::GNSS;
    return error_state::SUCCESS;
}

error_state ThreadManagement::processSharedMemory() {
    return error_state::SUCCESS;
}

void ThreadManagement::shutdownModules() {
    // ★ 设置所有正在运行模块的关机标志位
    SMStore::TM->shutdown |= bit_LIDAR;
    SMStore::TM->shutdown |= bit_DISPLAY;
    SMStore::TM->shutdown |= bit_VC;          // ★ 新增
    SMStore::TM->shutdown |= bit_CONTROLLER;  // ★ 新增
    //SMStore::TM->shutdown |= bit_GNSS;        // ★ 新增
}

bool ThreadManagement::getShutdownFlag() {
    return false; // Wk8 保持运行直到按 'q'
}

void ThreadManagement::threadFunction() {
    // ========== 1. 启动 LiDAR 线程 ==========
    LiDAR^ lidar = gcnew LiDAR();
    Thread^ thLidar = gcnew Thread(gcnew ThreadStart(lidar, &LiDAR::threadFunction));
    thLidar->IsBackground = true;
    thLidar->Start();
    Console::WriteLine("LiDAR thread started.");

    // ========== 2. 启动 Display 线程 ==========
    Display^ display = gcnew Display();
    Thread^ thDisplay = gcnew Thread(gcnew ThreadStart(display, &Display::threadFunction));
    thDisplay->IsBackground = true;
    thDisplay->Start();
    Console::WriteLine("Display thread started.");

    /*
    // ========== 3. 启动 GNSS 线程 ==========
    GNSS^ gnss = gcnew GNSS();
    Thread^ thGnss = gcnew Thread(gcnew ThreadStart(gnss, &GNSS::threadFunction));
    thGnss->IsBackground = true;
    thGnss->Start();
    Console::WriteLine("GNSS thread started.");
    */

    // ========== 4. 启动 Vehicle Control (VC) 线程 ==========
    VC^ vc = gcnew VC();
    Thread^ thVC = gcnew Thread(gcnew ThreadStart(vc, &VC::threadFunction));
    thVC->IsBackground = true;
    thVC->Start();
    Console::WriteLine("Vehicle Control thread started.");

    // ========== 5. 启动 Controller 线程 ==========
    Controller^ ctrl = gcnew Controller();
    Thread^ thCtrl = gcnew Thread(gcnew ThreadStart(ctrl, &Controller::threadFunction));
    thCtrl->IsBackground = true;
    thCtrl->Start();
    Console::WriteLine("Controller thread started.");

    Console::WriteLine("Press 'q' to shutdown...");

    // ========== 按 'q' 键触发例行关机 ==========
    while (true) {
        if (Console::KeyAvailable && Console::ReadKey(true).Key == ConsoleKey::Q) {
            shutdownModules();
            break;
        }
        Thread::Sleep(50);
    }

    // ========== 等待线程退出 (Join) ==========
    // 注意：Join 顺序不重要，只要都 Join 到即可
    thLidar->Join();
    thDisplay->Join();
    //thGnss->Join();
    thVC->Join();
    thCtrl->Join();

    Console::WriteLine("Shutdown complete.");
}