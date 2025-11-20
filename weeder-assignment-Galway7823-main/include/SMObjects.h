#pragma once
/*****************
SMObjects.h
ALEXANDER CUNIO 2024
*****************/

/*
This file outlines the objects that should be utilised for shared memory between your
individual modules. These should all be created by the thread management module and 
shared to other modules as required.
*/

#using <System.dll>
#include <iostream>
#include <cstdint>

using namespace System;
using namespace System::Threading;
using namespace System::Net::Sockets;
using namespace System::Net;
using namespace System::Text;
using namespace System::Diagnostics;

// Defines which processes are critical and not critical
// Ordering of processes in the mask should be based on the order in the UnitFlags type below.
// You should change this based on what processes are and are not critical
constexpr uint8_t NONCRITICALMASK = 0xff;
constexpr uint8_t CRITICALMASK =    0x00;

constexpr uint8_t bit_ALL =             0b01111111;
constexpr uint8_t bit_TM =              0b00000001;
constexpr uint8_t bit_LIDAR =           0b00000010;
constexpr uint8_t bit_GNSS =            0b00000100;
constexpr uint8_t bit_VC =              0b00001000;
constexpr uint8_t bit_CONTROLLER =      0b00010000;
constexpr uint8_t bit_DISPLAY =         0b00100000;
constexpr uint8_t bit_CRASHAVOIDANCE =  0b01000000;

ref class SM_ThreadManagement
{
public:
    Object^ lockObject;
    uint8_t shutdown = 0;
    Barrier^ ThreadBarrier;
    array<Stopwatch^>^ WatchList;
    uint8_t heartbeat = 0;

    SM_ThreadManagement() {
        lockObject = gcnew Object();
    }
};

// This is the size of the data recieved from the lidar
#define STANDARD_LIDAR_LENGTH 361

ref class SM_Lidar
{
public:
    Object^ lockObject;
    array<double>^ x;
    array<double>^ y;

    SM_Lidar() {
        lockObject = gcnew Object();
        x = gcnew array<double>(STANDARD_LIDAR_LENGTH);
        y = gcnew array<double>(STANDARD_LIDAR_LENGTH);
    }
};

ref class SM_GNSS
{
public:
    Object^ lockObject;
    double Northing;
    double Easting;
    double Height;

    SM_GNSS() {
        lockObject = gcnew Object();
    }
};

ref class SM_VehicleControl
{
public:
    Object^ lockObject;
    double Speed;
    double Steering;

    SM_VehicleControl() {
        lockObject = gcnew Object();
    }
};
