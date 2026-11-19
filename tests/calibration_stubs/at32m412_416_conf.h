#ifndef CALIBRATION_TEST_DEVICE
#define CALIBRATION_TEST_DEVICE
#include <stdint.h>
typedef enum { FLASH_OPERATE_DONE, FLASH_ERROR } flash_status_type;
#define FLASH_PRGMERR_FLAG 1
#define FLASH_EPPERR_FLAG 2
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void flash_unlock(void);
void flash_lock(void);
void flash_flag_clear(uint32_t flags);
flash_status_type flash_sector_erase(uintptr_t address);
flash_status_type flash_word_program(uintptr_t address,uint32_t word);
#endif
