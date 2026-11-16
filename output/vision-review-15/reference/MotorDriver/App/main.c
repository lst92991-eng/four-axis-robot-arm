#include "at32f45x.h"
#include "at32f45x_clock.h"
#include "board_debug.h"
#include "App_main.h"

int main(void)
{
    system_clock_config();
    Board_MotorOff();
    Board_DebugInit();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    if (SysTick_Config(system_core_clock / 1000U) != 0U)
    {
        for (;;) {}
    }

    App_Main();

    for (;;) {}
}
