#include "App_Main.h"
#include "chassis_app.h"
#include "board_safety.h"
#include "FreeRTOS.h"
#include "task.h"

static StaticTask_t control_tcb, idle_tcb;
static StackType_t control_stack[1024], idle_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack, uint32_t *size)
{
    *tcb = &idle_tcb; *stack = idle_stack; *size = configMINIMAL_STACK_SIZE;
}
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task; (void)name;
    Board_SafeHalt();
}

static void App_task(void *argument)
{
    TickType_t wake = xTaskGetTickCount();
    (void)argument;
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
        wake = xTaskGetTickCount();
        chassis_app_step();
    }
}
void App_Main(void)
{
    TaskHandle_t task;
    if(!chassis_app_init()) Board_SafeHalt();
    task = xTaskCreateStatic(App_task, "ChassisCtrl", 1024, NULL, 5, control_stack, &control_tcb);
    configASSERT(task != NULL);
    vTaskStartScheduler();
    Board_SafeHalt();
}
