#pragma once
#using <System.dll>
#include <SMObjects.h>

public ref class SMStore sealed {
public:
    static initonly SM_Lidar^ Lidar = gcnew SM_Lidar();
    static initonly SM_ThreadManagement^ TM = gcnew SM_ThreadManagement();

    static initonly SM_VehicleControl^ VC = gcnew SM_VehicleControl();
    static initonly SM_GNSS^ GNSS = gcnew SM_GNSS();
};