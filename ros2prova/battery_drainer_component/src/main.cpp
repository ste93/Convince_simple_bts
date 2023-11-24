#include "BatteryDrainerComponent.cpp"
int main()
{

    BatteryDrainerComponent batteryDrainerComponent;
    if (!batteryDrainerComponent.open()) {
        return 1;
    }
    batteryDrainerComponent.spin();

    batteryDrainerComponent.close();

    return 0;
}
