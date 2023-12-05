#include "FakeBattery.cpp"
#include <thread>

int main()
{

    FakeBattery fakeBattery;
    if (!fakeBattery.open()) {
        return 1;
    }

    std::make_shared<std::thread>(fakeBattery.start());
    while (true)
    {
        fakeBattery.discharge(0.1);
    }
    
    return 0;
}
