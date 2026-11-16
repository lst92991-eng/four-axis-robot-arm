#include "at32f45x.h"
#include "board_debug.h"

/* 本章不启用电机 PWM，将四个 H 桥控制输入保持为低。 */
void Board_MotorOff(void)
{
    gpio_init_type gpio;
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    gpio_bits_reset(GPIOC, GPIO_PINS_6 | GPIO_PINS_7 |
                          GPIO_PINS_8 | GPIO_PINS_9);
    gpio_default_para_init(&gpio);
    gpio.gpio_mode = GPIO_MODE_OUTPUT;
    gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio.gpio_pull = GPIO_PULL_NONE;
    gpio.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7 |
                     GPIO_PINS_8 | GPIO_PINS_9;
    gpio_init(GPIOC, &gpio);
}

/* PA9 复用为 USART1_TX，串口参数为 115200、8N1。 */
void Board_DebugInit(void)
{
    gpio_init_type gpio;
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio);
    gpio.gpio_mode = GPIO_MODE_MUX;
    gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio.gpio_pull = GPIO_PULL_NONE;
    gpio.gpio_pins = GPIO_PINS_9;
    gpio_init(GPIOA, &gpio);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_7);
    usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(USART1, USART_PARITY_NONE);
    usart_transmitter_enable(USART1, TRUE);
    usart_enable(USART1, TRUE);
}

/* 逐字节发送；等待次数设有上限，外设异常时返回失败。 */
int Board_DebugWrite(const char *text)
{
    uint32_t remaining;
    while (*text != '\0') {
        remaining = 1000000U;
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET) {
            if (--remaining == 0U) {
                return 0;
            }
        }
        usart_data_transmit(USART1, (uint8_t)*text++);
    }
    return 1;
}

extern volatile uint32_t board_ms;

/* 无 RTOS 的第一阶段使用毫秒差值延时。 */
void Board_DelayMs(uint32_t milliseconds)
{
    uint32_t start = board_ms;
    while ((uint32_t)(board_ms - start) < milliseconds) {
    }
}
