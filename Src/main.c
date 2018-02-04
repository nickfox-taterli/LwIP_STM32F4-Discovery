#include "stm32f4xx.h"

#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_eth.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rng.h"

#include "lwip.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "web_view.h"

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

  LL_RCC_HSI_SetCalibTrimming(16);

  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_2);

  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);

  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(168000000);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

  LL_SetSystemCoreClock(168000000);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
	
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_RNG);
  LL_RNG_Enable(RNG);
}

void MainTask(void const * argument)
{
  MX_LWIP_Init();
  
	//xTaskCreate(udpecho_thread, "udpecho_thread",DEFAULT_THREAD_STACKSIZE, NULL,UDPECHO_THREAD_PRIO,NULL);
	//xTaskCreate(udplite_thread, "udplite_thread",DEFAULT_THREAD_STACKSIZE, NULL,TCPECHO_THREAD_PRIO,NULL);
  //xTaskCreate(tcpecho_thread, "tcpecho_thread",DEFAULT_THREAD_STACKSIZE, NULL,TCPECHO_THREAD_PRIO,NULL);
	xTaskCreate(web_view, "web_view",DEFAULT_THREAD_STACKSIZE*3, NULL,WEB_THREAD_PRIO,NULL);
	//xTaskCreate(http_server_socket_thread,"httpd_thread",DEFAULT_THREAD_STACKSIZE * 2,NULL,WEBSERVER_THREAD_PRIO,NULL);
	//xTaskCreate(iperf_thread,"iperf_thread",DEFAULT_THREAD_STACKSIZE,NULL,WEBSERVER_THREAD_PRIO,NULL);
	
  for(;;)
  {
    vTaskDelete(NULL);
  }
}

int main(void)
{
  SystemClock_Config();

	xTaskCreate((TaskFunction_t)MainTask,"MainTask",configMINIMAL_STACK_SIZE, NULL, 0,NULL);
  vTaskStartScheduler();

  while (1)
  {
  }
}

