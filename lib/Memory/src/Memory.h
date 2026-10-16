#include <Arduino.h>

class Memory
{
private:
    const int EEPROM_SIZE = 512; // Size of EEPROM in bytes
    const int BASE_POSITION_ADDRESS = 0; // Address for base position
    const int ARM_JOINT1_POSITION_ADDRESS = 4; // Address for arm joint 1 position
    const int ARM_JOINT2_POSITION_ADDRESS = 8; // Address for arm joint 2 position
    const int GRIPPER_POSITION_ADDRESS = 12; // Address for gripper position
    const int GRIPPER_POSITION_STATUS_ADDRESS = 16; // Address for gripper position status (0 or 1 for the position)
    const int MANUAL_MODE_ADDRESS = 20; // Address for manual or auto
    const int WORLD_MODE_ADDRESS = 21; // Address for world or joint mode
    const int SELECTED_SLOT_ADDRESS = 22; // Address for selected slot
    const int Automatic_State = 23; // Address for automatic state (On route, or finished) (0 = finished, 1 = on route to grab spot, 2 = on route to drop spot)

public:
    Memory();

    void begin();
    void 




};