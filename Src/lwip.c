#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"

struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;

/**
  *  初始化总入口
  */
void MX_LWIP_Init(void)
{
  /* 初始化协议栈,其实就是申请线程申请锁. */
  tcpip_init( NULL, NULL );

  /* 默认清零地址(DHCP方式申请) */
  ipaddr.addr = 0;
  netmask.addr = 0;
  gw.addr = 0;

  /* 增加一个网络接口,ethernetif_init是底层初始化,tcpip_input是协议栈入口,这个有点复杂. */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /* 注册默认网络接口. */
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* 如果网络是连上,就介入网络好了. */
    netif_set_up(&gnetif);
  }
  else
  {
    /* 网络没连上就关闭网络. */
    netif_set_down(&gnetif);
  }

  /* DHCP其实完全可以开一条线程来守护,不然只能先插网线后上电. */
  dhcp_start(&gnetif);
}
