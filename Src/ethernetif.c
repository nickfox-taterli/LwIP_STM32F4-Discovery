/**
  ******************************************************************************
  * File Name          : ethernetif.c
  * Description        : This file provides code for the configuration
  *                      of the ethernetif.c MiddleWare.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
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
/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT ( portMAX_DELAY )
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE ( 350 )
/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Private variables ---------------------------------------------------------*/
__align(4) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] ;/* Ethernet Rx MA Descriptor */
__align(4) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] ;/* Ethernet Tx DMA Descriptor */
__align(4) uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] ; /* Ethernet Receive Buffer */
__align(4) uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] ; /* Ethernet Transmit Buffer */


/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/* Semaphore to signal incoming packets */
SemaphoreHandle_t s_xSemaphore = NULL;
/* Global Ethernet handle */
ETH_HandleTypeDef heth;

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

/* Private functions ---------------------------------------------------------*/

void LL_ETH_MspInit(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    /* USER CODE BEGIN ETH_MspInit 0 */

    /* USER CODE END ETH_MspInit 0 */
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


    /* Peripheral interrupt init */
    NVIC_SetPriority(ETH_IRQn, 5);
    NVIC_EnableIRQ(ETH_IRQn);

}

void LL_ETH_MspDeInit(void)
{

    LL_GPIO_InitTypeDef GPIO_InitStruct;
    /* USER CODE BEGIN ETH_MspDeInit 0 */

    /* USER CODE END ETH_MspDeInit 0 */
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

    /* USER CODE BEGIN ETH_MspDeInit 1 */

}

/**
  * @brief  Ethernet Rx Transfer completed callback
  * @param  heth: ETH handle
  * @retval None
  */
void LL_ETH_RxCpltCallback(void)
{
  portBASE_TYPE taskWoken = pdFALSE;
  xSemaphoreGiveFromISR(s_xSemaphore, &taskWoken) ;
  portEND_SWITCHING_ISR(taskWoken);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
    uint8_t MACAddr[6] ;

    heth.Init.PhyAddress = 1;
    MACAddr[0] = 0x00;
    MACAddr[1] = 0x80;
    MACAddr[2] = 0xE1;
    MACAddr[3] = 0x00;
    MACAddr[4] = 0x00;
    MACAddr[5] = 0x00;
    heth.Init.MACAddr = &MACAddr[0];

    /* USER CODE BEGIN MACADDRESS */

    /* USER CODE END MACADDRESS */

    LL_ETH_Init(&heth);
    /* Set netif link flag */
    netif->flags |= NETIF_FLAG_LINK_UP;
    /* Initialize Tx Descriptors list: Chain Mode */
    LL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);

    /* Initialize Rx Descriptors list: Chain Mode  */
    LL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

#if LWIP_ARP || LWIP_ETHERNET

    /* set MAC hardware address length */
    netif->hwaddr_len = ETH_HWADDR_LEN;

    /* set MAC hardware address */
    netif->hwaddr[0] =  heth.Init.MACAddr[0];
    netif->hwaddr[1] =  heth.Init.MACAddr[1];
    netif->hwaddr[2] =  heth.Init.MACAddr[2];
    netif->hwaddr[3] =  heth.Init.MACAddr[3];
    netif->hwaddr[4] =  heth.Init.MACAddr[4];
    netif->hwaddr[5] =  heth.Init.MACAddr[5];

    /* maximum transfer unit */
    netif->mtu = 1500;

    /* Accept broadcast address and ARP traffic */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
#if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
#else
    netif->flags |= NETIF_FLAG_BROADCAST;
#endif /* LWIP_ARP */

    /* create a binary semaphore used for informing ethernetif of frame reception */
vSemaphoreCreateBinary(s_xSemaphore);
	xTaskCreate((TaskFunction_t)ethernetif_input,"EthIf",INTERFACE_THREAD_STACK_SIZE, netif, configMAX_PRIORITIES,NULL);
    /* Enable MAC and DMA transmission and reception */
    LL_ETH_Start();

    /* USER CODE BEGIN PHY_PRE_CONFIG */

    /* USER CODE END PHY_PRE_CONFIG */

    /* USER CODE BEGIN PHY_POST_CONFIG */

    /* USER CODE END PHY_POST_CONFIG */

#endif /* LWIP_ARP || LWIP_ETHERNET */

    /* USER CODE BEGIN LOW_LEVEL_INIT */

    /* USER CODE END LOW_LEVEL_INIT */
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
 * @return pbuf 
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

    /* Release descriptors to DMA */
    /* Point to first descriptor */
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
        /* Clear RBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_RBUS;
        /* Resume DMA reception */
        ETH->DMARPDR = 0;
    }
    return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input( void const *argument )
{
    struct pbuf *p;
    struct netif *netif = (struct netif *) argument;

    for( ;; )
    {
        if (xSemaphoreTake( s_xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            do
            {
                p = low_level_input( netif );
                if   (p != NULL)
                {
                    if (netif->input( p, netif) != ERR_OK )
                    {
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
 * This function has to be completed by user in case of ARP OFF.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if ...
 */
static err_t low_level_output_arp_off(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr)
{
    err_t errval;
    errval = ERR_OK;

    /* USER CODE BEGIN 5 */

    /* USER CODE END 5 */

    return errval;

}
#endif /* LWIP_ARP */

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...) */

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
    netif->output = etharp_output;
#else
    /* The user should write ist own code in low_level_output_arp_off function */
    netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */

#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */

    netif->linkoutput = low_level_output;

    /* initialize the hardware */
    low_level_init(netif);

    return ERR_OK;
}

/* USER CODE BEGIN 6 */

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Time
*/
u32_t sys_jiffies(void)
{
    return xTaskGetTickCount();
}

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Time
*/
u32_t sys_now(void)
{
    return xTaskGetTickCount();
}

/* USER CODE END 6 */

/* USER CODE BEGIN 7 */

/* USER CODE END 7 */

#if LWIP_NETIF_LINK_CALLBACK
/**
  * @brief  Link callback function, this function is called on change of link status
  *         to update low level driver configuration.
* @param  netif: The network interface
  * @retval None
  */
void ethernetif_update_config(struct netif *netif)
{
    __IO uint32_t tickstart = 0;
    uint32_t regvalue = 0;

    if(netif_is_link_up(netif))
    {
        /* Restart the auto-negotiation */
        if(heth.Init.AutoNegotiation != ETH_AUTONEGOTIATION_DISABLE)
        {
            /* Enable Auto-Negotiation */
            LL_ETH_WritePHYRegister(&heth, PHY_BCR, PHY_AUTONEGOTIATION);

            /* Get tick */
            tickstart = HAL_GetTick();

            /* Wait until the auto-negotiation will be completed */
            do
            {
                LL_ETH_ReadPHYRegister(&heth, PHY_BSR, &regvalue);

                /* Check for the Timeout ( 1s ) */
                if((HAL_GetTick() - tickstart ) > 1000)
                {
                    /* In case of timeout */
                    goto error;
                }
            }
            while (((regvalue & PHY_AUTONEGO_COMPLETE) != PHY_AUTONEGO_COMPLETE));

            /* Read the result of the auto-negotiation */
            LL_ETH_ReadPHYRegister(&heth, PHY_SR, &regvalue);

            /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
            if((regvalue & PHY_DUPLEX_STATUS) != (uint32_t)RESET)
            {
                /* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
                heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
            }
            else
            {
                /* Set Ethernet duplex mode to Half-duplex following the auto-negotiation */
                heth.Init.DuplexMode = ETH_MODE_HALFDUPLEX;
            }
            /* Configure the MAC with the speed fixed by the auto-negotiation process */
            if(regvalue & PHY_SPEED_STATUS)
            {
                /* Set Ethernet speed to 10M following the auto-negotiation */
                heth.Init.Speed = ETH_SPEED_10M;
            }
            else
            {
                /* Set Ethernet speed to 100M following the auto-negotiation */
                heth.Init.Speed = ETH_SPEED_100M;
            }
        }
        else /* AutoNegotiation Disable */
        {
error :
            /* Check parameters */
            assert_param(IS_ETH_SPEED(heth.Init.Speed));
            assert_param(IS_ETH_DUPLEX_MODE(heth.Init.DuplexMode));

            /* Set MAC Speed and Duplex Mode to PHY */
            LL_ETH_WritePHYRegister(&heth, PHY_BCR, ((uint16_t)(heth.Init.DuplexMode >> 3) |
                                    (uint16_t)(heth.Init.Speed >> 1)));
        }

        /* ETHERNET MAC Re-Configuration */
        LL_ETH_ConfigMAC(&heth, (ETH_MACInitTypeDef *) NULL);

        /* Restart MAC interface */
        LL_ETH_Start(&heth);
    }
    else
    {
        /* Stop MAC interface */
        LL_ETH_Stop(&heth);
    }

    ethernetif_notify_conn_changed(netif);
}

/* USER CODE BEGIN 8 */
/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */
__weak void ethernetif_notify_conn_changed(struct netif *netif)
{
    /* NOTE : This is function could be implemented in user file
              when the callback is needed,
    */

}
/* USER CODE END 8 */
#endif /* LWIP_NETIF_LINK_CALLBACK */

/* USER CODE BEGIN 9 */

/* USER CODE END 9 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

