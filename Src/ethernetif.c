#include "stm32f4xx.h"
#include "lwip/opt.h"

#include "stm32f4xx_ll_eth.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"

#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "ethernetif.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/* 接收等待时间,如果不是MAX就是定期查询,是MAX就是等中断,一般都等MAX. */
#define TIME_WAITING_FOR_INPUT ( portMAX_DELAY )
/* 网络协议栈占用大小 */
#define INTERFACE_THREAD_STACK_SIZE ( 350 )
/* 网卡接口名,他定义为ST了. */
#define IFNAME0 's'
#define IFNAME1 't'

__align(4) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] ;/* Ethernet Rx MA Descriptor */
__align(4) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] ;/* Ethernet Tx DMA Descriptor */
__align(4) uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] ; /* Ethernet Receive Buffer */
__align(4) uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] ; /* Ethernet Transmit Buffer */

/* 用于接收处理上下文的信号量 */
SemaphoreHandle_t s_xSemaphore = NULL;
/* 以太网句柄(包括状态,缓冲区描述符指针) */
ETH_HandleTypeDef heth;

/* 主要就是IO初始化和加开一个中断 */
void LL_ETH_MspInit(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    /* Enable Peripheral clock */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ETHMAC);
    LL_AHB1_GRP1_EnableClock(RCC_AHB1ENR_ETHMACTXEN);
    LL_AHB1_GRP1_EnableClock(RCC_AHB1ENR_ETHMACRXEN);
    LL_AHB1_GRP1_EnableClock(RCC_AHB1ENR_ETHMACPTPEN);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1
    */

    GPIO_InitStruct.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_SetPriority(ETH_IRQn, 5);
    NVIC_EnableIRQ(ETH_IRQn);

}

/* 反过来,就是关了中断,恢复IO.但是IO时钟别关,其他地方可能还用这个时钟. */
void LL_ETH_MspDeInit(void)
{

    LL_GPIO_InitTypeDef GPIO_InitStruct;
    /* Peripheral clock disable */
    LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_ETHMAC);
    LL_AHB1_GRP1_DisableClock(RCC_AHB1ENR_ETHMACTXEN);
    LL_AHB1_GRP1_DisableClock(RCC_AHB1ENR_ETHMACRXEN);
    LL_AHB1_GRP1_DisableClock(RCC_AHB1ENR_ETHMACPTPEN);

    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* Peripheral interrupt Deinit*/
    NVIC_DisableIRQ(ETH_IRQn);
}

/**
* @brief  中断接收到的上下文中的上文,用来给下文一个解锁.让处理在中断外做.
  */
void LL_ETH_RxCpltCallback(void)
{
    portBASE_TYPE taskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_xSemaphore, &taskWoken) ;
    portEND_SWITCHING_ISR(taskWoken);
}

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
 * ethernetif_init() 时候会调用他,初始化设备.
 */
static void low_level_init(struct netif *netif)
{
    uint8_t MACAddr[6] ;

    heth.Init.PhyAddress = 1;
    MACAddr[0] = 0xB8;
    MACAddr[1] = 0x27;
    MACAddr[2] = 0xEB; /* 偷窃树莓派的MAC */
    MACAddr[3] = 0x10;
    MACAddr[4] = 0x20;
    MACAddr[5] = 0x30;
    heth.Init.MACAddr = &MACAddr[0];

    LL_ETH_Init(&heth);
    /* 设置为连接状态 */
    netif->flags |= NETIF_FLAG_LINK_UP;
    /* 初始化两个缓冲区 */
    LL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
    LL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

#if LWIP_ARP || LWIP_ETHERNET

    /* 设置MAC长度 */
    netif->hwaddr_len = ETH_HWADDR_LEN;

    /* 设置MAC */
    netif->hwaddr[0] =  heth.Init.MACAddr[0];
    netif->hwaddr[1] =  heth.Init.MACAddr[1];
    netif->hwaddr[2] =  heth.Init.MACAddr[2];
    netif->hwaddr[3] =  heth.Init.MACAddr[3];
    netif->hwaddr[4] =  heth.Init.MACAddr[4];
    netif->hwaddr[5] =  heth.Init.MACAddr[5];

    /* 以太网最大的MTU */
    netif->mtu = 1500;

    /* 支持广播/ARP,因为我打开了ARP支持. */
#if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
#else
    netif->flags |= NETIF_FLAG_BROADCAST;
#endif /* LWIP_ARP */

    /* 接收中断用的信号量 */
    vSemaphoreCreateBinary(s_xSemaphore);
    xTaskCreate((TaskFunction_t)ethernetif_input, "EthIf", INTERFACE_THREAD_STACK_SIZE, netif, configMAX_PRIORITIES, NULL);
    /* 启动ETH外设. */
    LL_ETH_Start();

#endif /* LWIP_ARP || LWIP_ETHERNET */

}

/**
 * pbuf是个链表,从pbuf对接到最底层的方法在这里.
 *
 * @param netif 就是网卡接口,LwIP支持多网卡,但是STM32只有单一的MAC.虽然PHY可以有很多地址.
 * @param p 就是著名的LwIP pbuf.
 * @return ERR_OK 证明发送OK.
 *         其他都是错误.
 *
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    err_t errval;
    struct pbuf *q;
    uint8_t *buffer = (uint8_t *)(heth.TxDesc->Buffer1Addr);
    __IO ETH_DMADescTypeDef *DmaTxDesc;
    uint32_t framelength = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t payloadoffset = 0;
    DmaTxDesc = heth.TxDesc;

    /* 遍历pbuf,如果pbuf的下一个元素是NULL,那么就是pbuf遍历完成了.整个for里面只处理一个pbuf. */
    for(q = p; q != NULL; q = q->next)
    {
        /* 该位置位才表示描述符(缓冲区)所有权是DMA,否则所有权是CPU.所有权是CPU的时候,我们才能操作,否则就是DMA事务中. */
        if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
        {
            errval = ERR_USE;
            goto error; /* 这里返回错误可以被LwIP捕捉. */
        }

        /* 查当前pbuf需要传输多少字节. */
        byteslefttocopy = q->len;
        payloadoffset = 0;

        /* 如果当前需要传输的总量大于ETH_TX_BUF_SIZE,那么一直传输东西好了. */
        while( (byteslefttocopy) > ETH_TX_BUF_SIZE )
        {
            /* 复制内容(这个用DMA会快点嘛?) */
            memcpy( (uint8_t *)((uint8_t *)buffer), (uint8_t *)((uint8_t *)q->payload + payloadoffset), (ETH_TX_BUF_SIZE) );

            /* 当前描述符已经复制了内容,移动到下一个描述符. */
            DmaTxDesc = (ETH_DMADescTypeDef *)(DmaTxDesc->Buffer2NextDescAddr);

            /* 确保这个描述符给CPU用,没有在DMA事务. */
            if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
            {
                errval = ERR_USE;
                goto error;
            }

            /* 新描述符的buf地址就是目标地址. */
            buffer = (uint8_t *)(DmaTxDesc->Buffer1Addr);

            /* 剩余需要复制的字节数,因为已经写了ETH_TX_BUF_SIZE大小了,所以这里就减掉他. */
            byteslefttocopy -= (ETH_TX_BUF_SIZE);
            /* pbuf的前部分大小(ETH_TX_BUF_SIZE)已经复制了,现在移动到下一步复制. */
            payloadoffset += (ETH_TX_BUF_SIZE);
            /* 帧总长就是加上所有写入的缓冲大小. */
            framelength += (ETH_TX_BUF_SIZE);
        }

        /* 复制剩余的部分. */
        memcpy( (uint8_t *)((uint8_t *)buffer), (uint8_t *)((uint8_t *)q->payload + payloadoffset), byteslefttocopy );
        framelength = framelength + byteslefttocopy;
    }

    /* 启动DMA,只需要写入长度,因为描述符刚才已经写了. */
    LL_ETH_TransmitFrame(&heth, framelength);

    errval = ERR_OK;

error:

    /* 如果DMA有下溢错误标志,那么清除他,以便继续传输.(为什么会发生?) */
    if ((ETH->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
    {
        /* 清除DMA下溢标志. */
        ETH->DMASR = ETH_DMASR_TUS;

        /* 恢复DMA传输 */
        ETH->DMATPDR = 0;
    }
    return errval;
}

/**
 * 这个接收操作,要收包其实都在这里.
 *
 * @param netif 就是网卡接口,LwIP支持多网卡,但是STM32只有单一的MAC.虽然PHY可以有很多地址.
 * @return pbuf - 返回这个还用解释?
 *
 */
static struct pbuf *low_level_input(struct netif *netif)
{
    struct pbuf *p = NULL;
    struct pbuf *q = NULL;
    uint16_t len = 0;
    uint8_t *buffer;
    __IO ETH_DMADescTypeDef *dmarxdesc;
    uint32_t payloadoffset = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t i = 0;


    /* 为0时候证明数据有效 */
    if (LL_ETH_GetReceivedFrame_IT(&heth) != 0)
        return NULL;

    /* 数据提取 */
    len = heth.RxFrameInfos.length;
    buffer = (uint8_t *)heth.RxFrameInfos.buffer;

    /* 申请内存 */
    p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    if (p != NULL)
    {
        /* 把数据指针指向首位. */
        dmarxdesc = heth.RxFrameInfos.FSRxDesc;
        for(q = p; q != NULL; q = q->next)
        {
            byteslefttocopy = q->len;
            payloadoffset = 0;

            /* 如果byteslefttocopy大于ETH_RX_BUF_SIZE,就意味着一个buf喂不饱一个pbuf. */
            while( (byteslefttocopy) > ETH_RX_BUF_SIZE )
            {
                /* 复制数据 */
                memcpy( (uint8_t *)((uint8_t *)q->payload + payloadoffset), (uint8_t *)((uint8_t *)buffer), (ETH_RX_BUF_SIZE));

                /* 指针指向下一个描述符 */
                dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
                buffer = (uint8_t *)(dmarxdesc->Buffer1Addr);

                /* 因为一次喂不饱pbuf,所以计算下还要多少. */
                byteslefttocopy -= (ETH_RX_BUF_SIZE);
                /* 因为前面已经拷贝了ETH_RX_BUF_SIZE大小,现在就得继续累积. */
                payloadoffset += (ETH_RX_BUF_SIZE);
            }
            /* 剩下部分的可以一次性给够pbuf了,不用切到下一个描述符了. */
            memcpy( (uint8_t *)((uint8_t *)q->payload + payloadoffset), (uint8_t *)((uint8_t *)buffer), byteslefttocopy);
        }
    }

    dmarxdesc = heth.RxFrameInfos.FSRxDesc;
    /* 把描述符所有权给DMA */
    for (i = 0; i < heth.RxFrameInfos.SegCount; i++)
    {
        dmarxdesc->Status |= ETH_DMARXDESC_OWN;
        dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
    }

    /* 其实清不清,重新接收也是重来. */
    heth.RxFrameInfos.SegCount = 0;

    /* 如果RXBuffer无效被置位,就清除,然后就可以开始传输. */
    if ((ETH->DMASR &ETH_DMASR_RBUS) != (uint32_t)RESET)
    {
        /* 清除RXBuffer标志 */
        ETH->DMASR = ETH_DMASR_RBUS;
        /* 恢复DMA传输 */
        ETH->DMARPDR = 0;
    }
    return p;
}

/**
 * 接收中断的上下文中的下文.
 */
void ethernetif_input( void const *argument )
{
    struct pbuf *p;
    struct netif *netif = (struct netif *) argument;

    for( ;; )
    {
        /* 中断内会把这个东西释放 */
        if (xSemaphoreTake( s_xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            do
            {
                /* 把接收到的存到pbuf里面. */
                p = low_level_input( netif );
                if   (p != NULL)
                {
										/* 由LwIP来分析pbuf.这个实际上是tcpip_input函数 */
                    if (netif->input( p, netif) != ERR_OK )
                    {
                        /* 用完要清理. */
                        pbuf_free(p);
                    }
                }
            }
            while(p != NULL);
        }
    }
}

#if !LWIP_ARP
/**
 * 关闭ARP时候调用
 */
static err_t low_level_output_arp_off(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr)
{
    err_t errval;
    errval = ERR_OK;

    return errval;

}
#endif /* LWIP_ARP */

/**
 * 以太网接口初始化
 */
err_t ethernetif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* 主机名 */
    netif->hostname = "raspberrypi";
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
    netif->output = etharp_output; /* IP层通过调用这个函数,来发送一个数据包.通常他先ARP,然后发送. */
#else
    /* 如果不适用ARP功能,那么还要有一个叫ARP关闭输出的函数. */
    netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */

#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */

    netif->linkoutput = low_level_output; /* ARP最终要发送也还算要调用他.这个不包含地址,属于直接发包. */

    /* 就是调用那个IO/INT初始化的. */
    low_level_init(netif);

    return ERR_OK;
}


/**
* @brief  当前毫秒时间
*/
u32_t sys_jiffies(void)
{
    return xTaskGetTickCount();
}

/**
* @brief  依然是当前毫秒时间
*/
u32_t sys_now(void)
{
    return xTaskGetTickCount();
}
