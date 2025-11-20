#include "TMM.h"

int main(array<System::String^>^)
{
    ThreadManagement^ tm = gcnew ThreadManagement();
    // 直接在主线程里跑 TMM（简单起见）
    tm->threadFunction();
    return 0;
}
