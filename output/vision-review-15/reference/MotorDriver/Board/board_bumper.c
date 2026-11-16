#include "board_bumper.h"
#include "at32f45x.h"

static volatile uint8_t pending;
static uint8_t history;
static uint32_t last_active_ms;
extern volatile uint32_t board_ms;
static const uint32_t lines[4] = {
    EXINT_LINE_15, EXINT_LINE_13, EXINT_LINE_14, EXINT_LINE_9
};
static const uint8_t bits[4] = {
    BUMPER_PC15, BUMPER_PC13, BUMPER_PC14, BUMPER_PB9
};

/* 返回当前按下位；电路在按下时输出低电平。 */
uint8_t Board_BumperRead(void)
{
    uint8_t value = 0;
    if (gpio_input_data_bit_read(GPIOC, GPIO_PINS_15) == RESET) value |= BUMPER_PC15;
    if (gpio_input_data_bit_read(GPIOC, GPIO_PINS_13) == RESET) value |= BUMPER_PC13;
    if (gpio_input_data_bit_read(GPIOC, GPIO_PINS_14) == RESET) value |= BUMPER_PC14;
    if (gpio_input_data_bit_read(GPIOB, GPIO_PINS_9) == RESET) value |= BUMPER_PB9;
    return value;
}

/* 先完成普通 GPIO 输入，后续调用 EnableIRQ 增加事件记录。 */
void Board_BumperInit(void)
{
    gpio_init_type gpio;
    nvic_irq_disable(EXINT9_5_IRQn);
    nvic_irq_disable(EXINT15_10_IRQn);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio);
    gpio.gpio_mode = GPIO_MODE_INPUT;
    gpio.gpio_pull = GPIO_PULL_NONE;
    gpio.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
    gpio_init(GPIOC, &gpio);
    gpio.gpio_pins = GPIO_PINS_9;
    gpio_init(GPIOB, &gpio);
    pending = 0;
    history = Board_BumperRead();
    last_active_ms = board_ms;
}

void Board_BumperEnableIRQ(void)
{
    exint_init_type exti;
    uint8_t i;
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE15);
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE13);
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE14);
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, SCFG_PINS_SOURCE9);
    exint_default_para_init(&exti);
    exti.line_enable = TRUE;
    exti.line_mode = EXINT_LINE_INTERRUPT;
    exti.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
    for (i = 0; i < 4; i++) {
        exti.line_select = lines[i];
        exint_flag_clear(lines[i]);
        exint_init(&exti);
    }
    history |= Board_BumperRead();
    last_active_ms = board_ms;
    nvic_irq_enable(EXINT9_5_IRQn, 2, 0);
    nvic_irq_enable(EXINT15_10_IRQn, 2, 0);
}

/* 下降沿挂起标志就是事件证据；先清硬件标志，再保存对应位。 */
void Board_BumperIRQ(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++) {
        if (exint_interrupt_flag_get(lines[i]) != RESET) {
            exint_flag_clear(lines[i]);
            pending |= bits[i];
        }
    }
}

uint8_t Board_BumperUpdate(uint32_t now_ms)
{
    uint32_t mask = __get_PRIMASK();
    uint8_t active;
    __disable_irq();
    active = Board_BumperRead() | pending;
    pending = 0;
    history |= active;
    if (active != 0) last_active_ms = now_ms;
    __set_PRIMASK(mask);
    return history;
}

/* 释放保持至少100ms，且没有新边沿，才接受显式清除。 */
uint8_t Board_BumperClear(uint32_t now_ms)
{
    uint32_t mask = __get_PRIMASK();
    uint8_t i, ok;
    __disable_irq();
    ok = (Board_BumperRead() == 0 && pending == 0 &&
          (uint32_t)(now_ms - last_active_ms) >= 100U);
    for (i = 0; i < 4; i++) {
        if (exint_interrupt_flag_get(lines[i]) != RESET) ok = 0;
    }
    if (ok != 0) history = 0;
    __set_PRIMASK(mask);
    return ok;
}
