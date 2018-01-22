#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/apps/lwiperf.h"

static void
lwiperf_report(void *arg, enum lwiperf_report_type report_type,
  const ip_addr_t* local_addr, u16_t local_port, const ip_addr_t* remote_addr, u16_t remote_port,
  u32_t bytes_transferred, u32_t ms_duration, u32_t bandwidth_kbitpsec)
{
  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(local_addr);
  LWIP_UNUSED_ARG(local_port);
}


/**
  * @brief  iperf thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
void iperf_thread(void *arg)
{
	lwiperf_start_tcp_server_default(lwiperf_report,NULL);
	vTaskDelete(NULL);
}
