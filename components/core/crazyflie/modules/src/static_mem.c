/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file static_mem.c
 * @brief Support for OS objects that use static memory, such as
 * queues, tasks and semaphores
 *
 * @copyright Copyright (c) 2019 Bitcraze AB
 */

#include <FreeRTOS.h>
#include "static_mem.h"

/**
 * @brief configSUPPORT_STATIC_ALLOCATION 设为 1，因此应用必须提供
 * vApplicationGetIdleTaskMemory() 的实现，用于提供空闲任务所需的内存。
 */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t xIdleTaskTCB;
  NO_DMA_CCM_SAFE_ZERO_INIT static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*———————————————————–*/

/**
 * @brief configSUPPORT_STATIC_ALLOCATION 与 configUSE_TIMERS 均设为 1，
 * 因此应用必须提供 vApplicationGetTimerTaskMemory() 的实现，
 * 以提供定时器服务任务使用的内存。
 */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t xTimerTaskTCB;
  NO_DMA_CCM_SAFE_ZERO_INIT static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
