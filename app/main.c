#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "serialManager.h"
#include "stm32f10x.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    /* init serial and tasks*/
    serialMngInit();
    
    vTaskStartScheduler();

    /* Will only get here if there was not enough heap space to create the idle task. */
    return 0;
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

