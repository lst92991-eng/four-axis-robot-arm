#include "CAN.h"
#include "at32m412_416_conf.h"
#include "config.h"
#include <string.h>

#define APP JOINT_APP_ADDRESS
#define META JOINT_APP_METADATA
#define SECTOR 1024u
#define MAGIC JOINT_APP_MAGIC
#if BOOT_NODE < 1 || BOOT_NODE > 4
#error "BOOT_NODE must be 1 through 4"
#endif
#define REQUEST (0x600u + BOOT_NODE)
#define REPLY (0x680u + BOOT_NODE)

enum { INFO = 1, BEGIN, DATA, FINISH };
static volatile uint32_t milliseconds;

void SysTick_Handler(void)
{
  ++milliseconds;
}

//低侧输入低有效，先设置输出锁存值再切成输出模式
static void bridge_off(void)
{
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  gpio_bits_reset(GPIOA, GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11);
  gpio_bits_set(GPIOA, GPIO_PINS_7);
  gpio_bits_set(GPIOB, GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4);
  gpio_init_type pins;
  gpio_default_para_init(&pins);
  pins.gpio_mode = GPIO_MODE_OUTPUT;
  pins.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
  gpio_init(GPIOA, &pins);
  pins.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4;
  gpio_init(GPIOB, &pins);
}

static uint32_t image_crc(uint32_t size)
{
  const volatile uint8_t *image = (const volatile uint8_t *)APP;
  uint32_t crc = ~0u;
  for (uint32_t i = 0; i < size; ++i)
  {
    crc ^= image[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

static int valid_image(uint32_t size, uint32_t crc)
{
  if (size < 512 || size > META - APP || (size & 3u))
    return 0;
  const volatile uint32_t *vectors = (const volatile uint32_t *)APP;
  uint32_t stack = vectors[0];
  uint32_t entry = vectors[1];
  return stack > 0x20000000u && stack <= 0x20004000u && !(stack & 7u) &&
         (entry & 1u) && entry >= APP + 1u && entry < APP + size &&
         image_crc(size) == crc;
}

static void launch(void)
{
  SysTick->CTRL = 0;
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
  crm_periph_reset(CRM_CAN1_PERIPH_RESET, TRUE);
  crm_periph_reset(CRM_CAN1_PERIPH_RESET, FALSE);
  SCB->VTOR = APP;
  __DSB();
  __ISB();
  //切换堆栈后直接跳转，禁止编译器再访问当前栈帧
  __asm volatile("ldr r0, =0x08004000\n"
                 "ldr r1, [r0]\n"
                 "ldr r2, [r0, #4]\n"
                 "msr msp, r1\n"
                 "bx r2\n" ::: "r0", "r1", "r2", "memory");
  __builtin_unreachable();
}

static int program_word(uint32_t address, uint32_t value)
{
  return flash_word_program(address, value) == FLASH_OPERATE_DONE &&
         *(const volatile uint32_t *)address == value;
}

static int erase_image(uint32_t size)
{
  //先撤销启动许可，再擦写应用区；引导和末尾标定区不在范围内
  if (flash_sector_erase(META) != FLASH_OPERATE_DONE ||
      *(const volatile uint32_t *)META != ~0u)
    return 0;
  for (uint32_t offset = 0; offset < size; offset += SECTOR)
    if (flash_sector_erase(APP + offset) != FLASH_OPERATE_DONE)
      return 0;
  return 1;
}

int main(void)
{
  bridge_off();
  const volatile uint32_t *meta = (const volatile uint32_t *)META;
  int ready = meta[0] == MAGIC && valid_image(meta[1], meta[2]);
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
  if (crm_hext_stable_wait() != SUCCESS || !CAN_init())
  {
    if (ready)
      launch();
    while (1) {}
  }
  system_core_clock_update();
  SysTick_Config(system_core_clock / 1000u);
  uint32_t deadline = 2500;
  uint32_t owner = 0, size = 0, crc = 0, written = 0;

  while (1)
  {
    if (ready && (int32_t)(milliseconds - deadline) >= 0)
      launch();

    uint32_t frame[16];
    uint16_t id;
    if (CAN_receive(&id, (uint8_t *)frame) != 1 || id != REQUEST ||
        frame[0] != MAGIC || !frame[1])
      continue;

    uint32_t command = frame[2], argument = frame[3];
    uint32_t result = 1;
    if (command == INFO && argument == BOOT_NODE)
    {
      result = 0;
      if (ready)
        deadline = milliseconds + 2500;
    }
    else if (command == BEGIN && argument >= 512 && argument <= META - APP &&
             !(argument & 3u) && frame[5] == BOOT_NODE)
    {
      ready = 0;
      owner = 0;
      flash_unlock();
      flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
      if (erase_image(argument))
      {
        owner = frame[1];
        size = argument;
        crc = frame[4];
        written = 0;
        result = 0;
      }
      flash_lock();
    }
    else if (owner == frame[1] && command == DATA && argument == written && written < size)
    {
      uint32_t bytes = size - written < 48 ? size - written : 48;
      flash_unlock();
      result = 0;
      for (uint32_t i = 0; i < bytes; i += 4)
        if (!program_word(APP + written + i, frame[4 + i / 4]))
        {
          result = 1;
          owner = 0;
          break;
        }
      flash_lock();
      if (!result)
        written += bytes;
    }
    else if (owner == frame[1] && command == FINISH && argument == size && written == size &&
             valid_image(size, crc))
    {
      flash_unlock();
      ready = program_word(META + 4, size) && program_word(META + 8, crc) &&
              program_word(META, MAGIC);
      flash_lock();
      owner = 0;
      result = !ready;
      deadline = milliseconds + 500;
    }

    memset(frame + 4, 0, 48);
    frame[4] = result;
    CAN_send(REPLY, (const uint8_t *)frame);
  }
}

