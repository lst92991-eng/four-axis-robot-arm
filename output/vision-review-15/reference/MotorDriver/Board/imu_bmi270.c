#include "imu_bmi270.h"
#include "at32f45x.h"
#include "bmi270.h"
#include "motor_encoder.h"
#include "motor_lesson_config.h"
#include "board_debug.h"
#include <string.h>

static struct bmi2_dev device;
static ImuData state;
static uint32_t last_sample;
static uint8_t initialized;
extern volatile uint32_t board_ms;

static void Select(uint8_t active)
{
    if (active && spi_i2s_flag_get(SPI3, SPI_I2S_RDBF_FLAG) != RESET)
        (void)spi_i2s_data_receive(SPI3);
    gpio_bits_write(GPIOA, GPIO_PINS_15, active ? FALSE : TRUE);
}

static uint8_t Transfer(uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = 10000U;
    while (spi_i2s_flag_get(SPI3, SPI_I2S_TDBE_FLAG) == RESET && timeout) --timeout;
    if (!timeout) return 0;
    spi_i2s_data_transmit(SPI3, tx);
    timeout = 10000U;
    while (spi_i2s_flag_get(SPI3, SPI_I2S_RDBF_FLAG) == RESET && timeout) --timeout;
    if (!timeout) return 0;
    *rx = (uint8_t)spi_i2s_data_receive(SPI3);
    return 1;
}

static BMI2_INTF_RETURN_TYPE Read(uint8_t reg, uint8_t *data, uint32_t len, void *context)
{
    uint8_t value, ok;
    uint32_t i;
    (void)context;
    Select(1);
    ok = Transfer(reg, &value);
    /* SensorAPI已把dummy字节算入len，回调不再额外跳过。 */
    for (i = 0; ok && i < len; ++i) ok = Transfer(0, &data[i]);
    Select(0);
    return ok ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE Write(uint8_t reg, const uint8_t *data, uint32_t len, void *context)
{
    uint8_t value, ok;
    uint32_t i;
    (void)context;
    Select(1);
    ok = Transfer(reg, &value);
    for (i = 0; ok && i < len; ++i) ok = Transfer(data[i], &value);
    Select(0);
    return ok ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

static void DelayUs(uint32_t us, void *context)
{
    (void)context;
    while (us) {
        uint32_t part = us > 1000U ? 1000U : us;
        uint32_t start = DWT->CYCCNT;
        uint32_t ticks = (system_core_clock / 1000000U) * part;
        while ((uint32_t)(DWT->CYCCNT - start) < ticks) {}
        us -= part;
    }
}

static void SpiInit(void)
{
    gpio_init_type gpio;
    spi_init_type spi;
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio);
    gpio.gpio_pins = GPIO_PINS_10 | GPIO_PINS_11 | GPIO_PINS_12;
    gpio.gpio_mode = GPIO_MODE_MUX;
    gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio.gpio_pull = GPIO_PULL_NONE;
    gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOC, &gpio);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE10, GPIO_MUX_6);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE11, GPIO_MUX_6);
    gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE12, GPIO_MUX_6);
    gpio.gpio_pins = GPIO_PINS_15;
    gpio.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_bits_set(GPIOA, GPIO_PINS_15);
    gpio_init(GPIOA, &gpio);
    spi_default_para_init(&spi);
    spi.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi.master_slave_mode = SPI_MODE_MASTER;
    spi.mclk_freq_division = SPI_MCLK_DIV_32;
    spi.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi.frame_bit_num = SPI_FRAME_8BIT;
    spi.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi.clock_phase = SPI_CLOCK_PHASE_1EDGE;
    spi.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI3, &spi);
    spi_software_cs_internal_level_set(SPI3, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
    spi_enable(SPI3, TRUE);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t Configure(void)
{
    struct bmi2_sens_config config[2] = {0};
    uint8_t sensors[2] = {BMI2_ACCEL, BMI2_GYRO};
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;
    if (bmi2_get_sensor_config(config, 2, &device) != BMI2_OK) return 0;
    config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
    config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    if (bmi2_set_sensor_config(config, 2, &device) != BMI2_OK) return 0;
    return bmi2_sensor_enable(sensors, 2, &device) == BMI2_OK;
}

static uint8_t Calibrate(void)
{
    struct bmi2_sens_data data;
    uint16_t count = 0, tries;
    float sum = 0, square_sum = 0;
    int32_t left = motor_encoder_get_count(MOTOR_LEFT);
    int32_t right = motor_encoder_get_count(MOTOR_RIGHT);
    for (tries = 0; tries < 200 && count < 64; ++tries) {
        float rate, gravity2;
        Board_DelayMs(5);
        if (left != motor_encoder_get_count(MOTOR_LEFT) ||
            right != motor_encoder_get_count(MOTOR_RIGHT)) return 0;
        if (bmi2_get_sensor_data(&data, &device) != BMI2_OK) return 0;
        if ((data.status & (BMI2_DRDY_GYR | BMI2_DRDY_ACC)) !=
            (BMI2_DRDY_GYR | BMI2_DRDY_ACC)) continue;
        gravity2 = ((float)data.acc.x * data.acc.x + (float)data.acc.y * data.acc.y +
                    (float)data.acc.z * data.acc.z) / (16384.0f * 16384.0f);
        if (gravity2 < 0.49f || gravity2 > 1.69f) return 0;
        rate = data.gyr.z * (2000.0f / 32768.0f);
        sum += rate;
        square_sum += rate * rate;
        ++count;
    }
    if (count != 64) return 0;
    state.bias_dps = sum / 64.0f;
    return square_sum / 64.0f - state.bias_dps * state.bias_dps <= 1.0f;
}

uint8_t Imu_Init(void)
{
    initialized = 0;
    memset(&device, 0, sizeof(device));
    memset(&state, 0, sizeof(state));
    SpiInit();
    device.intf = BMI2_SPI_INTF;
    device.read = Read;
    device.write = Write;
    device.delay_us = DelayUs;
    device.read_write_len = 32;
    device.dummy_byte = 1;
    initialized = bmi270_init(&device) == BMI2_OK && Configure() && Calibrate();
    last_sample = board_ms;
    return initialized;
}

uint8_t Imu_Update(uint32_t now_ms)
{
    struct bmi2_sens_data data;
    state.valid = 0;
    if (!initialized || bmi2_get_sensor_data(&data, &device) != BMI2_OK) return 0;
    if ((data.status & BMI2_DRDY_GYR) != 0U) {
        state.ax_g = data.acc.x / 16384.0f;
        state.ay_g = data.acc.y / 16384.0f;
        state.az_g = data.acc.z / 16384.0f;
        state.gz_raw_dps = data.gyr.z * (2000.0f / 32768.0f);
        /* 与原驱动保持一致：扣零后做0.2系数低通，再供轮控和里程计使用。 */
        state.gz_dps += 0.2f * ((state.gz_raw_dps - state.bias_dps) *
                               LESSON_IMU_Z_SIGN - state.gz_dps);
        ++state.sequence;
        last_sample = now_ms;
    }
    state.valid = state.sequence != 0 && (uint32_t)(now_ms - last_sample) < 50U;
    return state.valid;
}

void Imu_Get(ImuData *data) { *data = state; }
