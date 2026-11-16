#include "board_safety.h"
#include "at32f45x.h"
#include "at32f45x_wdt.h"
#include "motor_driver.h"

uint8_t Board_WatchdogInit(void)
{
    uint32_t timeout = 100000U;
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_64);
    wdt_reload_value_set(62U);
    while ((wdt_flag_get(WDT_DIVF_UPDATE_FLAG) != RESET ||
            wdt_flag_get(WDT_RLDF_UPDATE_FLAG) != RESET) && timeout) --timeout;
    wdt_register_write_enable(FALSE);
    if (!timeout) return 0;
    wdt_counter_reload();
    wdt_enable();
    return 1;
}

void Board_WatchdogFeed(void) { wdt_counter_reload(); }

void Board_SafeHalt(void)
{
    __disable_irq();
    motor_driver_stop_all(MOTOR_STOP_BRAKE);
    __DSB();
    __ISB();
    for (;;) {}
}
