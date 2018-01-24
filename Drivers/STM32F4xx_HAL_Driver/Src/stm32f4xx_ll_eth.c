/* Includes ------------------------------------------------------------------*/
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

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
/** @addtogroup STM32F4xx_LL_Driver
  * @{
  */
	
/* 大多数情况下,首帧 = 末帧,包特大,比如下载,会出现其他情况. */

/** @defgroup ETH ETH
  * @brief ETH HAL module driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup ETH_Private_Constants ETH Private Constants
  * @{
  */
#define ETH_TIMEOUT_SWRESET               500U
#define ETH_TIMEOUT_LINKED_STATE          5000U
#define ETH_TIMEOUT_AUTONEGO_COMPLETED    5000U

SemaphoreHandle_t ETH_TransmitFrame_Semaphore;

/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup ETH_Private_Functions ETH Private Functions
  * @{
  */
static void ETH_MACDMAConfig(ETH_HandleTypeDef *heth);
static void ETH_MACAddressConfig(uint32_t MacAddr, uint8_t *Addr);
static void ETH_MACReceptionEnable(void);
static void ETH_MACReceptionDisable(void);
static void ETH_MACTransmissionEnable(void);
static void ETH_MACTransmissionDisable(void);
static void ETH_DMATransmissionEnable(void);
static void ETH_DMATransmissionDisable(void);
static void ETH_DMAReceptionEnable(void);
static void ETH_DMAReceptionDisable(void);
static void ETH_FlushTransmitFIFO(void);

/**
  * @}
  */
/* Private functions ---------------------------------------------------------*/

/** @defgroup ETH_Exported_Functions ETH Exported Functions
  * @{
  */

/** @defgroup ETH_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief   Initialization and Configuration functions
  *
  @verbatim
  ===============================================================================
            ##### Initialization and de-initialization functions #####
  ===============================================================================
  [..]  This section provides functions allowing to:
      (+) Initialize and configure the Ethernet peripheral
      (+) De-initialize the Ethernet peripheral

  @endverbatim
  * @{
  */

/**
  * @brief  Initializes the Ethernet MAC and DMA according to default
  *         parameters.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
void LL_ETH_Init(ETH_HandleTypeDef *heth)
{
    uint32_t tmpreg1 = 0U, phyreg = 0U;

    /* Allocate lock resource and initialize it */
    //heth->Lock
    /* Init the low level hardware : GPIO, CLOCK, NVIC. */
    LL_ETH_MspInit();

    /* Enable SYSCFG Clock */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    /* Select MII or RMII Mode*/
    SYSCFG->PMC &= ~(SYSCFG_PMC_MII_RMII_SEL);
    SYSCFG->PMC |= ETH_MEDIA_INTERFACE_RMII;

    /* Ethernet Software reset */
    /* Set the SWR bit: resets all MAC subsystem internal registers and logic */
    /* After reset all the registers holds their respective reset values */
    ETH->DMABMR |= ETH_DMABMR_SR;


    /* Wait for software reset */
    while ((ETH->DMABMR & ETH_DMABMR_SR) != (uint32_t)RESET)
    {

    }

    /*-------------------------------- MAC Initialization ----------------------*/
    /* Get the ETHERNET MACMIIAR value */
    tmpreg1 = ETH->MACMIIAR;
    /* Clear CSR Clock Range CR[2:0] bits */
    tmpreg1 &= ETH_MACMIIAR_CR_MASK;

    /* CSR Clock Range between 150-183 MHz */
    tmpreg1 |= (uint32_t)ETH_MACMIIAR_CR_Div102;

    /* Write to ETHERNET MAC MIIAR: Configure the ETHERNET CSR Clock Range */
    ETH->MACMIIAR = (uint32_t)tmpreg1;

    /*-------------------- PHY initialization and configuration ----------------*/
    /* Put the PHY in reset mode */
    LL_ETH_WritePHYRegister(heth, PHY_BCR, PHY_RESET);

    /* Delay to assure PHY reset */
    vTaskDelay(PHY_RESET_DELAY);

    /* We wait for linked status */
    do
    {
        LL_ETH_ReadPHYRegister(heth, PHY_BSR, &phyreg);
    }
    while (((phyreg & PHY_LINKED_STATUS) != PHY_LINKED_STATUS));


    /* Enable Auto-Negotiation */
    LL_ETH_WritePHYRegister(heth, PHY_BCR, PHY_AUTONEGOTIATION);

    /* Wait until the auto-negotiation will be completed */
    do
    {
        LL_ETH_ReadPHYRegister(heth, PHY_BSR, &phyreg);

    }
    while (((phyreg & PHY_AUTONEGO_COMPLETE) != PHY_AUTONEGO_COMPLETE));

    /* Read the result of the auto-negotiation */
    LL_ETH_ReadPHYRegister(heth, PHY_SR, &phyreg);

    /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
    if((phyreg & PHY_DUPLEX_STATUS) != (uint32_t)RESET)
    {
        /* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
        (heth->Init).DuplexMode = ETH_MODE_FULLDUPLEX;
    }
    else
    {
        /* Set Ethernet duplex mode to Half-duplex following the auto-negotiation */
        (heth->Init).DuplexMode = ETH_MODE_HALFDUPLEX;
    }
    /* Configure the MAC with the speed fixed by the auto-negotiation process */
    if((phyreg & PHY_SPEED_STATUS) == PHY_SPEED_STATUS)
    {
        /* Set Ethernet speed to 10M following the auto-negotiation */
        (heth->Init).Speed = ETH_SPEED_10M;
    }
    else
    {
        /* Set Ethernet speed to 100M following the auto-negotiation */
        (heth->Init).Speed = ETH_SPEED_100M;
    }

    /* Config MAC and DMA */
    ETH_MACDMAConfig(heth);

    /* Set ETH HAL State to Ready */

    ETH_TransmitFrame_Semaphore = xSemaphoreCreateMutex();

}

/**
  * @brief  De-Initializes the ETH peripheral.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
void LL_ETH_DeInit(void)
{
    /* De-Init the low level hardware : GPIO, CLOCK, NVIC. */
    LL_ETH_MspDeInit();
}

/**
  * @brief  Initializes the DMA Tx descriptors in chain mode.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  DMATxDescTab Pointer to the first Tx desc list
  * @param  TxBuff Pointer to the first TxBuffer list
  * @param  TxBuffCount Number of the used Tx desc in the list
  * @retval HAL status
  */
void LL_ETH_DMATxDescListInit(ETH_HandleTypeDef *heth, ETH_DMADescTypeDef *DMATxDescTab, uint8_t *TxBuff, uint32_t TxBuffCount)
{
    uint32_t i = 0U;
    ETH_DMADescTypeDef *dmatxdesc;
	
    /* Set the DMATxDescToSet pointer with the first one of the DMATxDescTab list */
    heth->TxDesc = DMATxDescTab;

    /* Fill each DMATxDesc descriptor with the right values */
    for(i = 0U; i < TxBuffCount; i++)
    {
        /* Get the pointer on the ith member of the Tx Desc list */
        dmatxdesc = DMATxDescTab + i;

        /* Set Second Address Chained bit */
        dmatxdesc->Status = ETH_DMATXDESC_TCH;

        /* Set Buffer1 address pointer */
        dmatxdesc->Buffer1Addr = (uint32_t)(&TxBuff[i * ETH_TX_BUF_SIZE]);

        /* Set the DMA Tx descriptors checksum insertion */
        dmatxdesc->Status |= ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL;


        /* Initialize the next descriptor with the Next Descriptor Polling Enable */
        if(i < (TxBuffCount - 1U))
        {
            /* Set next descriptor address register with next descriptor base address */
            dmatxdesc->Buffer2NextDescAddr = (uint32_t)(DMATxDescTab + i + 1U);
        }
        else
        {
            /* For last descriptor, set next descriptor address register equal to the first descriptor base address */
            dmatxdesc->Buffer2NextDescAddr = (uint32_t) DMATxDescTab;
        }
    }

    /* Set Transmit Descriptor List Address Register */
    ETH->DMATDLAR = (uint32_t) DMATxDescTab;

    /* Set ETH HAL State to Ready */
}

/**
  * @brief  Initializes the DMA Rx descriptors in chain mode.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  DMARxDescTab Pointer to the first Rx desc list
  * @param  RxBuff Pointer to the first RxBuffer list
  * @param  RxBuffCount Number of the used Rx desc in the list
  * @retval HAL status
  */
void LL_ETH_DMARxDescListInit(ETH_HandleTypeDef *heth, ETH_DMADescTypeDef *DMARxDescTab, uint8_t *RxBuff, uint32_t RxBuffCount)
{
    uint32_t i = 0U;
    ETH_DMADescTypeDef *DMARxDesc;

    /* Set the Ethernet RxDesc pointer with the first one of the DMARxDescTab list */
    heth->RxDesc = DMARxDescTab;

    /* Fill each DMARxDesc descriptor with the right values */
    for(i = 0U; i < RxBuffCount; i++)
    {
        /* Get the pointer on the ith member of the Rx Desc list */
        DMARxDesc = DMARxDescTab + i;

        /* Set Own bit of the Rx descriptor Status */
        DMARxDesc->Status = ETH_DMARXDESC_OWN;

        /* Set Buffer1 size and Second Address Chained bit */
        DMARxDesc->ControlBufferSize = ETH_DMARXDESC_RCH | ETH_RX_BUF_SIZE;

        /* Set Buffer1 address pointer */
        DMARxDesc->Buffer1Addr = (uint32_t)(&RxBuff[i * ETH_RX_BUF_SIZE]);


        /* Enable Ethernet DMA Rx Descriptor interrupt */
        DMARxDesc->ControlBufferSize &= ~ETH_DMARXDESC_DIC;

        /* Initialize the next descriptor with the Next Descriptor Polling Enable */
        if(i < (RxBuffCount - 1U))
        {
            /* Set next descriptor address register with next descriptor base address */
            DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDescTab + i + 1U);
        }
        else
        {
            /* For last descriptor, set next descriptor address register equal to the first descriptor base address */
            DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDescTab);
        }
    }

    /* Set Receive Descriptor List Address Register */
    ETH->DMARDLAR = (uint32_t) DMARxDescTab;
}

/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group2 IO operation functions
  *  @brief   Data transfers functions
  *
  @verbatim
  ==============================================================================
                          ##### IO operation functions #####
  ==============================================================================
  [..]  This section provides functions allowing to:
        (+) Transmit a frame
            LL_ETH_TransmitFrame();
        (+) Receive a frame
            LL_ETH_GetReceivedFrame();
            LL_ETH_GetReceivedFrame_IT();
        (+) Read from an External PHY register
            LL_ETH_ReadPHYRegister();
        (+) Write to an External PHY register
            LL_ETH_WritePHYRegister();

  @endverbatim

  * @{
  */

/**
  * @brief  Sends an Ethernet frame.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  FrameLength Amount of data to be sent
  * @retval HAL status
  */
void LL_ETH_TransmitFrame(ETH_HandleTypeDef *heth, uint32_t FrameLength)
{
    uint32_t bufcount = 0U, size = 0U, i = 0U;

    if (FrameLength == 0U)
    {
        return;
    }

    /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    if(((heth->TxDesc)->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
    {
        return;
    }

    /* Process Locked */
    xSemaphoreTake(ETH_TransmitFrame_Semaphore, portMAX_DELAY);

    /* Get the number of needed Tx buffers for the current frame */
    if (FrameLength > ETH_TX_BUF_SIZE)
    {
        bufcount = FrameLength / ETH_TX_BUF_SIZE;
        if (FrameLength % ETH_TX_BUF_SIZE)
        {
            bufcount++;
        }
    }
    else
    {
        bufcount = 1U;
    }
    if (bufcount == 1U)
    {
        /* Set LAST and FIRST segment */
        heth->TxDesc->Status |= ETH_DMATXDESC_FS | ETH_DMATXDESC_LS;
        /* Set frame size */
        heth->TxDesc->ControlBufferSize = (FrameLength & ETH_DMATXDESC_TBS1);
        /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
        heth->TxDesc->Status |= ETH_DMATXDESC_OWN;
        /* Point to next descriptor */
        heth->TxDesc = (ETH_DMADescTypeDef *)(heth->TxDesc->Buffer2NextDescAddr);
    }
    else
    {
        for (i = 0U; i < bufcount; i++)
        {
            /* Clear FIRST and LAST segment bits */
            heth->TxDesc->Status &= ~(ETH_DMATXDESC_FS | ETH_DMATXDESC_LS);

            if (i == 0U)
            {
                /* Setting the first segment bit */
                heth->TxDesc->Status |= ETH_DMATXDESC_FS;
            }

            /* Program size */
            heth->TxDesc->ControlBufferSize = (ETH_TX_BUF_SIZE & ETH_DMATXDESC_TBS1);

            if (i == (bufcount - 1U))
            {
                /* Setting the last segment bit */
                heth->TxDesc->Status |= ETH_DMATXDESC_LS;
                size = FrameLength - (bufcount - 1U) * ETH_TX_BUF_SIZE;
                heth->TxDesc->ControlBufferSize = (size & ETH_DMATXDESC_TBS1);
            }

            /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
            heth->TxDesc->Status |= ETH_DMATXDESC_OWN;
            /* point to next descriptor */
            heth->TxDesc = (ETH_DMADescTypeDef *)(heth->TxDesc->Buffer2NextDescAddr);
        }
    }

    /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
    if ((ETH->DMASR & ETH_DMASR_TBUS) != (uint32_t)RESET)
    {
        /* Clear TBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_TBUS;
        /* Resume DMA transmission*/
        ETH->DMATPDR = 0U;
    }

    /* Process Unlocked */
    xSemaphoreGive(ETH_TransmitFrame_Semaphore);

}

/**
  * @brief  Checks for received frames.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
void LL_ETH_GetReceivedFrame(ETH_HandleTypeDef *heth)
{
    uint32_t framelength = 0U;

    /* Check if segment is not owned by DMA */
    /* (((heth->RxDesc->Status & ETH_DMARXDESC_OWN) == (uint32_t)RESET) && ((heth->RxDesc->Status & ETH_DMARXDESC_LS) != (uint32_t)RESET)) */
    if(((heth->RxDesc->Status & ETH_DMARXDESC_OWN) == (uint32_t)RESET))
    {
        /* Check if last segment */
        if(((heth->RxDesc->Status & ETH_DMARXDESC_LS) != (uint32_t)RESET))
        {
            /* increment segment count */
            (heth->RxFrameInfos).SegCount++;

            /* Check if last segment is first segment: one segment contains the frame */
            if ((heth->RxFrameInfos).SegCount == 1U)
            {
                (heth->RxFrameInfos).FSRxDesc = heth->RxDesc;
            }

            heth->RxFrameInfos.LSRxDesc = heth->RxDesc;

            /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
            framelength = (((heth->RxDesc)->Status & ETH_DMARXDESC_FL) >> ETH_DMARXDESC_FRAMELENGTHSHIFT) - 4U;
            heth->RxFrameInfos.length = framelength;

            /* Get the address of the buffer start address */
            heth->RxFrameInfos.buffer = ((heth->RxFrameInfos).FSRxDesc)->Buffer1Addr;
            /* point to next descriptor */
            heth->RxDesc = (ETH_DMADescTypeDef *) ((heth->RxDesc)->Buffer2NextDescAddr);
        }
        /* Check if first segment */
        else if((heth->RxDesc->Status & ETH_DMARXDESC_FS) != (uint32_t)RESET)
        {
            (heth->RxFrameInfos).FSRxDesc = heth->RxDesc;
            (heth->RxFrameInfos).LSRxDesc = NULL;
            (heth->RxFrameInfos).SegCount = 1U;
            /* Point to next descriptor */
            heth->RxDesc = (ETH_DMADescTypeDef *) (heth->RxDesc->Buffer2NextDescAddr);
        }
        /* Check if intermediate segment */
        else
        {
            (heth->RxFrameInfos).SegCount++;
            /* Point to next descriptor */
            heth->RxDesc = (ETH_DMADescTypeDef *) (heth->RxDesc->Buffer2NextDescAddr);
        }
    }

}

/**
  * @brief  Gets the Received frame in interrupt mode.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
uint8_t LL_ETH_GetReceivedFrame_IT(ETH_HandleTypeDef *heth)
{
    uint32_t descriptorscancounter = 0U;

    /* Scan descriptors owned by CPU */
    while (((heth->RxDesc->Status & ETH_DMARXDESC_OWN) == (uint32_t)RESET) && (descriptorscancounter < ETH_RXBUFNB))
    {
			/* 防止缓冲区溢出 */
        descriptorscancounter++;

        /* Check if first segment in frame */
        /* ((heth->RxDesc->Status & ETH_DMARXDESC_FS) != (uint32_t)RESET) && ((heth->RxDesc->Status & ETH_DMARXDESC_LS) == (uint32_t)RESET)) */
			  /* 他只是第一帧,不同时是最后帧,所以就是有后续帧. */
			  if((heth->RxDesc->Status & (ETH_DMARXDESC_FS | ETH_DMARXDESC_LS)) == (uint32_t)ETH_DMARXDESC_FS)
        {
            heth->RxFrameInfos.FSRxDesc = heth->RxDesc;
            heth->RxFrameInfos.SegCount = 1U;
            /* Point to next descriptor */
            heth->RxDesc = (ETH_DMADescTypeDef *) (heth->RxDesc->Buffer2NextDescAddr);
        }
        /* Check if intermediate segment */
        /* ((heth->RxDesc->Status & ETH_DMARXDESC_LS) == (uint32_t)RESET)&& ((heth->RxDesc->Status & ETH_DMARXDESC_FS) == (uint32_t)RESET)) */
        /* 不是第一帧也不是最后帧,就是中间帧. */
				else if ((heth->RxDesc->Status & (ETH_DMARXDESC_LS | ETH_DMARXDESC_FS)) == (uint32_t)RESET)
        {
            /* Increment segment count */
            (heth->RxFrameInfos.SegCount)++;
            /* Point to next descriptor */
            heth->RxDesc = (ETH_DMADescTypeDef *)(heth->RxDesc->Buffer2NextDescAddr);
        }
				/* 前面排除了所有情况.这是最后帧.或者是第一帧最后帧都为同一帧.收到有末帧,才是真正的OK. */
        /* Should be last segment */
        else
        {
            /* 末帧 */
            heth->RxFrameInfos.LSRxDesc = heth->RxDesc;

            /* 最后一帧也是帧啊./ */
            (heth->RxFrameInfos.SegCount)++;

					/* 只有1帧,就是首帧和末帧是一个. */
            if ((heth->RxFrameInfos.SegCount) == 1U)
            {
							/* 首帧和末帧是一个. */
                heth->RxFrameInfos.FSRxDesc = heth->RxDesc;
            }

            /* 去掉4字节的CRC,就是真正的长度.这里提示的是接收长度. */
            heth->RxFrameInfos.length = (((heth->RxDesc)->Status & ETH_DMARXDESC_FL) >> ETH_DMARXDESC_FRAMELENGTHSHIFT) - 4U;

            /* 缓冲区的第一个缓冲区.(首帧) */
            heth->RxFrameInfos.buffer = ((heth->RxFrameInfos).FSRxDesc)->Buffer1Addr;

            /* 指向下一个缓冲区.等下次接收时候从下一个来. */
            heth->RxDesc = (ETH_DMADescTypeDef *) (heth->RxDesc->Buffer2NextDescAddr);
            return 0;
        }
    }
    return 1;
}

/**
  * @brief  This function handles ETH interrupt request.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
void ETH_IRQHandler(void)
{
    /* Frame received */
    if((ETH->DMASR & ( ETH_DMA_FLAG_R)) == ( ETH_DMA_FLAG_R))
    {
        /* Receive complete callback */
        LL_ETH_RxCpltCallback();

        /* Clear the Eth DMA Rx IT pending bits */
        ETH->DMASR = ETH_DMA_IT_R;

    }
    /* Frame transmitted */
    else if((ETH->DMASR & ( ETH_DMA_FLAG_T)) == ( ETH_DMA_FLAG_T))
    {
        /* Transfer complete callback */
        /* 发送完成中断 */

        /* Clear the Eth DMA Tx IT pending bits */
        ETH->DMASR = ETH_DMA_IT_T;

    }

    /* Clear the interrupt flags */
    ETH->DMASR = ETH_DMA_IT_NIS;

    /* ETH DMA Error */
    if((ETH->DMASR & ( ETH_DMA_FLAG_AIS)) == ( ETH_DMA_FLAG_AIS))
    {
        /* Ethernet Error callback */
        /* 此处需要错误处理 */

        /* Clear the interrupt flags */
        ETH->DMASR = ETH_DMA_FLAG_AIS;

    }
}

/**
  * @brief  Reads a PHY register
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param PHYReg PHY register address, is the index of one of the 32 PHY register.
  *                This parameter can be one of the following values:
  *                   PHY_BCR: Transceiver Basic Control Register,
  *                   PHY_BSR: Transceiver Basic Status Register.
  *                   More PHY register could be read depending on the used PHY
  * @param RegValue PHY register value
  * @retval HAL status
  */
void LL_ETH_ReadPHYRegister(ETH_HandleTypeDef *heth, uint16_t PHYReg, uint32_t *RegValue)
{
    uint32_t tmpreg1 = 0U;

    /* Get the ETHERNET MACMIIAR value */
    tmpreg1 = ETH->MACMIIAR;

    /* Keep only the CSR Clock Range CR[2:0] bits value */
    tmpreg1 &= ~ETH_MACMIIAR_CR_MASK;

    /* Prepare the MII address register value */
    tmpreg1 |= (((uint32_t)heth->Init.PhyAddress << 11U) & ETH_MACMIIAR_PA); /* Set the PHY device address   */
    tmpreg1 |= (((uint32_t)PHYReg << 6U) & ETH_MACMIIAR_MR);                /* Set the PHY register address */
    tmpreg1 &= ~ETH_MACMIIAR_MW;                                            /* Set the read mode            */
    tmpreg1 |= ETH_MACMIIAR_MB;                                             /* Set the MII Busy bit         */

    /* Write the result value into the MII Address register */
    ETH->MACMIIAR = tmpreg1;
    /* Check for the Busy flag */
    while((tmpreg1 & ETH_MACMIIAR_MB) == ETH_MACMIIAR_MB)
    {
        tmpreg1 = ETH->MACMIIAR;
    }
    /* Get MACMIIDR value */
    *RegValue = (uint16_t)(ETH->MACMIIDR);

}

/**
  * @brief  Writes to a PHY register.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  PHYReg PHY register address, is the index of one of the 32 PHY register.
  *          This parameter can be one of the following values:
  *             PHY_BCR: Transceiver Control Register.
  *             More PHY register could be written depending on the used PHY
  * @param  RegValue the value to write
  * @retval HAL status
  */
void LL_ETH_WritePHYRegister(ETH_HandleTypeDef *heth, uint16_t PHYReg, uint32_t RegValue)
{
    uint32_t tmpreg1 = 0U;

    /* Get the ETHERNET MACMIIAR value */
    tmpreg1 = ETH->MACMIIAR;

    /* Keep only the CSR Clock Range CR[2:0] bits value */
    tmpreg1 &= ~ETH_MACMIIAR_CR_MASK;

    /* Prepare the MII register address value */
    tmpreg1 |= (((uint32_t)heth->Init.PhyAddress << 11U) & ETH_MACMIIAR_PA); /* Set the PHY device address */
    tmpreg1 |= (((uint32_t)PHYReg << 6U) & ETH_MACMIIAR_MR);              /* Set the PHY register address */
    tmpreg1 |= ETH_MACMIIAR_MW;                                           /* Set the write mode */
    tmpreg1 |= ETH_MACMIIAR_MB;                                           /* Set the MII Busy bit */

    /* Give the value to the MII data register */
    ETH->MACMIIDR = (uint16_t)RegValue;

    /* Write the result value into the MII Address register */
    ETH->MACMIIAR = tmpreg1;

    /* Check for the Busy flag */
    while((tmpreg1 & ETH_MACMIIAR_MB) == ETH_MACMIIAR_MB)
    {
        tmpreg1 = ETH->MACMIIAR;
    }

}

/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group3 Peripheral Control functions
 *  @brief    Peripheral Control functions
 *
@verbatim
 ===============================================================================
                  ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Enable MAC and DMA transmission and reception.
          LL_ETH_Start();
      (+) Disable MAC and DMA transmission and reception.
          LL_ETH_Stop();
      (+) Set the MAC configuration in runtime mode
          LL_ETH_ConfigMAC();
      (+) Set the DMA configuration in runtime mode
          LL_ETH_ConfigDMA();

@endverbatim
  * @{
  */

/**
 * @brief  Enables Ethernet MAC and DMA reception/transmission
 * @param  heth pointer to a ETH_HandleTypeDef structure that contains
 *         the configuration information for ETHERNET module
 * @retval HAL status
 */
void LL_ETH_Start(void)
{

    /* Enable transmit state machine of the MAC for transmission on the MII */
    ETH_MACTransmissionEnable();

    /* Enable receive state machine of the MAC for reception from the MII */
    ETH_MACReceptionEnable();

    /* Flush Transmit FIFO */
    ETH_FlushTransmitFIFO();

    /* Start DMA transmission */
    ETH_DMATransmissionEnable();

    /* Start DMA reception */
    ETH_DMAReceptionEnable();

}

/**
  * @brief  Stop Ethernet MAC and DMA reception/transmission
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
void LL_ETH_Stop(void)
{

    /* Stop DMA transmission */
    ETH_DMATransmissionDisable();

    /* Stop DMA reception */
    ETH_DMAReceptionDisable();

    /* Disable receive state machine of the MAC for reception from the MII */
    ETH_MACReceptionDisable();

    /* Flush Transmit FIFO */
    ETH_FlushTransmitFIFO();

    /* Disable transmit state machine of the MAC for transmission on the MII */
    ETH_MACTransmissionDisable();

}

/**
  * @brief  Set ETH MAC Configuration.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  macconf MAC Configuration structure
  * @retval HAL status
  */
void LL_ETH_ConfigMAC(ETH_HandleTypeDef *heth, ETH_MACInitTypeDef *macconf)
{
    uint32_t tmpreg1 = 0U;

    if (macconf != NULL)
    {

        /*------------------------ ETHERNET MACCR Configuration --------------------*/
        /* Get the ETHERNET MACCR value */
        tmpreg1 = ETH->MACCR;
        /* Clear WD, PCE, PS, TE and RE bits */
        tmpreg1 &= ETH_MACCR_CLEAR_MASK;

        tmpreg1 |= (uint32_t)(macconf->Watchdog |
                              macconf->Jabber |
                              macconf->InterFrameGap |
                              macconf->CarrierSense |
                              (heth->Init).Speed |
                              macconf->ReceiveOwn |
                              macconf->LoopbackMode |
                              (heth->Init).DuplexMode |
                              macconf->ChecksumOffload |
                              macconf->RetryTransmission |
                              macconf->AutomaticPadCRCStrip |
                              macconf->BackOffLimit |
                              macconf->DeferralCheck);

        /* Write to ETHERNET MACCR */
        ETH->MACCR = (uint32_t)tmpreg1;

        /* Wait until the write operation will be taken into account :
        at least four TX_CLK/RX_CLK clock cycles */
        tmpreg1 = ETH->MACCR;
        vTaskDelay(ETH_REG_WRITE_DELAY);
        ETH->MACCR = tmpreg1;

        /*----------------------- ETHERNET MACFFR Configuration --------------------*/
        /* Write to ETHERNET MACFFR */
        ETH->MACFFR = (uint32_t)(macconf->ReceiveAll |
                                 macconf->SourceAddrFilter |
                                 macconf->PassControlFrames |
                                 macconf->BroadcastFramesReception |
                                 macconf->DestinationAddrFilter |
                                 macconf->PromiscuousMode |
                                 macconf->MulticastFramesFilter |
                                 macconf->UnicastFramesFilter);

        /* Wait until the write operation will be taken into account :
        at least four TX_CLK/RX_CLK clock cycles */
        tmpreg1 = ETH->MACFFR;
        vTaskDelay(ETH_REG_WRITE_DELAY);
        ETH->MACFFR = tmpreg1;

        /*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*/
        /* Write to ETHERNET MACHTHR */
        ETH->MACHTHR = (uint32_t)macconf->HashTableHigh;

        /* Write to ETHERNET MACHTLR */
        ETH->MACHTLR = (uint32_t)macconf->HashTableLow;
        /*----------------------- ETHERNET MACFCR Configuration --------------------*/

        /* Get the ETHERNET MACFCR value */
        tmpreg1 = ETH->MACFCR;
        /* Clear xx bits */
        tmpreg1 &= ETH_MACFCR_CLEAR_MASK;

        tmpreg1 |= (uint32_t)((macconf->PauseTime << 16U) |
                              macconf->ZeroQuantaPause |
                              macconf->PauseLowThreshold |
                              macconf->UnicastPauseFrameDetect |
                              macconf->ReceiveFlowControl |
                              macconf->TransmitFlowControl);

        /* Write to ETHERNET MACFCR */
        ETH->MACFCR = (uint32_t)tmpreg1;

        /* Wait until the write operation will be taken into account :
        at least four TX_CLK/RX_CLK clock cycles */
        tmpreg1 = ETH->MACFCR;
        vTaskDelay(ETH_REG_WRITE_DELAY);
        ETH->MACFCR = tmpreg1;

        /*----------------------- ETHERNET MACVLANTR Configuration -----------------*/
        ETH->MACVLANTR = (uint32_t)(macconf->VLANTagComparison |
                                    macconf->VLANTagIdentifier);

        /* Wait until the write operation will be taken into account :
        at least four TX_CLK/RX_CLK clock cycles */
        tmpreg1 = ETH->MACVLANTR;
        vTaskDelay(ETH_REG_WRITE_DELAY);
        ETH->MACVLANTR = tmpreg1;
    }
    else /* macconf == NULL : here we just configure Speed and Duplex mode */
    {
        /*------------------------ ETHERNET MACCR Configuration --------------------*/
        /* Get the ETHERNET MACCR value */
        tmpreg1 = ETH->MACCR;

        /* Clear FES and DM bits */
        tmpreg1 &= ~(0x00004800U);

        tmpreg1 |= (uint32_t)(heth->Init.Speed | heth->Init.DuplexMode);

        /* Write to ETHERNET MACCR */
        ETH->MACCR = (uint32_t)tmpreg1;

        /* Wait until the write operation will be taken into account:
        at least four TX_CLK/RX_CLK clock cycles */
        tmpreg1 = ETH->MACCR;
        vTaskDelay(ETH_REG_WRITE_DELAY);
        ETH->MACCR = tmpreg1;
    }

}

/**
  * @brief  Sets ETH DMA Configuration.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  dmaconf DMA Configuration structure
  * @retval HAL status
  */
void LL_ETH_ConfigDMA(ETH_HandleTypeDef *heth, ETH_DMAInitTypeDef *dmaconf)
{
    uint32_t tmpreg1 = 0U;

    /*----------------------- ETHERNET DMAOMR Configuration --------------------*/
    /* Get the ETHERNET DMAOMR value */
    tmpreg1 = ETH->DMAOMR;
    /* Clear xx bits */
    tmpreg1 &= ETH_DMAOMR_CLEAR_MASK;

    tmpreg1 |= (uint32_t)(dmaconf->DropTCPIPChecksumErrorFrame |
                          dmaconf->ReceiveStoreForward |
                          dmaconf->FlushReceivedFrame |
                          dmaconf->TransmitStoreForward |
                          dmaconf->TransmitThresholdControl |
                          dmaconf->ForwardErrorFrames |
                          dmaconf->ForwardUndersizedGoodFrames |
                          dmaconf->ReceiveThresholdControl |
                          dmaconf->SecondFrameOperate);

    /* Write to ETHERNET DMAOMR */
    ETH->DMAOMR = (uint32_t)tmpreg1;

    /* Wait until the write operation will be taken into account:
    at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->DMAOMR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->DMAOMR = tmpreg1;

    /*----------------------- ETHERNET DMABMR Configuration --------------------*/
    ETH->DMABMR = (uint32_t)(dmaconf->AddressAlignedBeats |
                             dmaconf->FixedBurst |
                             dmaconf->RxDMABurstLength | /* !! if 4xPBL is selected for Tx or Rx it is applied for the other */
                             dmaconf->TxDMABurstLength |
                             dmaconf->EnhancedDescriptorFormat |
                             (dmaconf->DescriptorSkipLength << 2U) |
                             dmaconf->DMAArbitration |
                             ETH_DMABMR_USP); /* Enable use of separate PBL for Rx and Tx */

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->DMABMR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->DMABMR = tmpreg1;

}

/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group4 Peripheral State functions
  *  @brief   Peripheral State functions
  *
  @verbatim
  ===============================================================================
                         ##### Peripheral State functions #####
  ===============================================================================
  [..]
  This subsection permits to get in run-time the status of the peripheral
  and the data flow.
       (+) Get the ETH handle state:
           LL_ETH_GetState();


  @endverbatim
  * @{
  */


/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup ETH_Private_Functions
  * @{
  */

/**
  * @brief  Configures Ethernet MAC and DMA with default parameters.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  err Ethernet Init error
  * @retval HAL status
  */
static void ETH_MACDMAConfig(ETH_HandleTypeDef *heth)
{
    ETH_MACInitTypeDef macinit;
    ETH_DMAInitTypeDef dmainit;
    uint32_t tmpreg1 = 0U;

    /* Ethernet MAC default initialization **************************************/
    macinit.Watchdog = ETH_WATCHDOG_ENABLE;
    macinit.Jabber = ETH_JABBER_ENABLE;
    macinit.InterFrameGap = ETH_INTERFRAMEGAP_96BIT;
    macinit.CarrierSense = ETH_CARRIERSENCE_ENABLE;
    macinit.ReceiveOwn = ETH_RECEIVEOWN_ENABLE;
    macinit.LoopbackMode = ETH_LOOPBACKMODE_DISABLE;
    macinit.ChecksumOffload = ETH_CHECKSUMOFFLAOD_ENABLE;
    macinit.RetryTransmission = ETH_RETRYTRANSMISSION_DISABLE;
    macinit.AutomaticPadCRCStrip = ETH_AUTOMATICPADCRCSTRIP_DISABLE;
    macinit.BackOffLimit = ETH_BACKOFFLIMIT_10;
    macinit.DeferralCheck = ETH_DEFFERRALCHECK_DISABLE;
    macinit.ReceiveAll = ETH_RECEIVEAll_DISABLE;
    macinit.SourceAddrFilter = ETH_SOURCEADDRFILTER_DISABLE;
    macinit.PassControlFrames = ETH_PASSCONTROLFRAMES_BLOCKALL;
    macinit.BroadcastFramesReception = ETH_BROADCASTFRAMESRECEPTION_ENABLE;
    macinit.DestinationAddrFilter = ETH_DESTINATIONADDRFILTER_NORMAL;
    macinit.PromiscuousMode = ETH_PROMISCUOUS_MODE_DISABLE;
    macinit.MulticastFramesFilter = ETH_MULTICASTFRAMESFILTER_PERFECT;
    macinit.UnicastFramesFilter = ETH_UNICASTFRAMESFILTER_PERFECT;
    macinit.HashTableHigh = 0x0U;
    macinit.HashTableLow = 0x0U;
    macinit.PauseTime = 0x0U;
    macinit.ZeroQuantaPause = ETH_ZEROQUANTAPAUSE_DISABLE;
    macinit.PauseLowThreshold = ETH_PAUSELOWTHRESHOLD_MINUS4;
    macinit.UnicastPauseFrameDetect = ETH_UNICASTPAUSEFRAMEDETECT_DISABLE;
    macinit.ReceiveFlowControl = ETH_RECEIVEFLOWCONTROL_DISABLE;
    macinit.TransmitFlowControl = ETH_TRANSMITFLOWCONTROL_DISABLE;
    macinit.VLANTagComparison = ETH_VLANTAGCOMPARISON_16BIT;
    macinit.VLANTagIdentifier = 0x0U;

    /*------------------------ ETHERNET MACCR Configuration --------------------*/
    /* Get the ETHERNET MACCR value */
    tmpreg1 = ETH->MACCR;
    /* Clear WD, PCE, PS, TE and RE bits */
    tmpreg1 &= ETH_MACCR_CLEAR_MASK;
    /* Set the WD bit according to ETH Watchdog value */
    /* Set the JD: bit according to ETH Jabber value */
    /* Set the IFG bit according to ETH InterFrameGap value */
    /* Set the DCRS bit according to ETH CarrierSense value */
    /* Set the FES bit according to ETH Speed value */
    /* Set the DO bit according to ETH ReceiveOwn value */
    /* Set the LM bit according to ETH LoopbackMode value */
    /* Set the DM bit according to ETH Mode value */
    /* Set the IPCO bit according to ETH ChecksumOffload value */
    /* Set the DR bit according to ETH RetryTransmission value */
    /* Set the ACS bit according to ETH AutomaticPadCRCStrip value */
    /* Set the BL bit according to ETH BackOffLimit value */
    /* Set the DC bit according to ETH DeferralCheck value */
    tmpreg1 |= (uint32_t)(macinit.Watchdog |
                          macinit.Jabber |
                          macinit.InterFrameGap |
                          macinit.CarrierSense |
                          (heth->Init).Speed |
                          macinit.ReceiveOwn |
                          macinit.LoopbackMode |
                          (heth->Init).DuplexMode |
                          macinit.ChecksumOffload |
                          macinit.RetryTransmission |
                          macinit.AutomaticPadCRCStrip |
                          macinit.BackOffLimit |
                          macinit.DeferralCheck);

    /* Write to ETHERNET MACCR */
    ETH->MACCR = (uint32_t)tmpreg1;

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->MACCR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->MACCR = tmpreg1;

    /*----------------------- ETHERNET MACFFR Configuration --------------------*/
    /* Set the RA bit according to ETH ReceiveAll value */
    /* Set the SAF and SAIF bits according to ETH SourceAddrFilter value */
    /* Set the PCF bit according to ETH PassControlFrames value */
    /* Set the DBF bit according to ETH BroadcastFramesReception value */
    /* Set the DAIF bit according to ETH DestinationAddrFilter value */
    /* Set the PR bit according to ETH PromiscuousMode value */
    /* Set the PM, HMC and HPF bits according to ETH MulticastFramesFilter value */
    /* Set the HUC and HPF bits according to ETH UnicastFramesFilter value */
    /* Write to ETHERNET MACFFR */
    ETH->MACFFR = (uint32_t)(macinit.ReceiveAll |
                             macinit.SourceAddrFilter |
                             macinit.PassControlFrames |
                             macinit.BroadcastFramesReception |
                             macinit.DestinationAddrFilter |
                             macinit.PromiscuousMode |
                             macinit.MulticastFramesFilter |
                             macinit.UnicastFramesFilter);

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->MACFFR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->MACFFR = tmpreg1;

    /*--------------- ETHERNET MACHTHR and MACHTLR Configuration --------------*/
    /* Write to ETHERNET MACHTHR */
    ETH->MACHTHR = (uint32_t)macinit.HashTableHigh;

    /* Write to ETHERNET MACHTLR */
    ETH->MACHTLR = (uint32_t)macinit.HashTableLow;
    /*----------------------- ETHERNET MACFCR Configuration -------------------*/

    /* Get the ETHERNET MACFCR value */
    tmpreg1 = ETH->MACFCR;
    /* Clear xx bits */
    tmpreg1 &= ETH_MACFCR_CLEAR_MASK;

    /* Set the PT bit according to ETH PauseTime value */
    /* Set the DZPQ bit according to ETH ZeroQuantaPause value */
    /* Set the PLT bit according to ETH PauseLowThreshold value */
    /* Set the UP bit according to ETH UnicastPauseFrameDetect value */
    /* Set the RFE bit according to ETH ReceiveFlowControl value */
    /* Set the TFE bit according to ETH TransmitFlowControl value */
    tmpreg1 |= (uint32_t)((macinit.PauseTime << 16U) |
                          macinit.ZeroQuantaPause |
                          macinit.PauseLowThreshold |
                          macinit.UnicastPauseFrameDetect |
                          macinit.ReceiveFlowControl |
                          macinit.TransmitFlowControl);

    /* Write to ETHERNET MACFCR */
    ETH->MACFCR = (uint32_t)tmpreg1;

    /* Wait until the write operation will be taken into account:
    at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->MACFCR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->MACFCR = tmpreg1;

    /*----------------------- ETHERNET MACVLANTR Configuration ----------------*/
    /* Set the ETV bit according to ETH VLANTagComparison value */
    /* Set the VL bit according to ETH VLANTagIdentifier value */
    ETH->MACVLANTR = (uint32_t)(macinit.VLANTagComparison |
                                macinit.VLANTagIdentifier);

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->MACVLANTR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->MACVLANTR = tmpreg1;

    /* Ethernet DMA default initialization ************************************/
    dmainit.DropTCPIPChecksumErrorFrame = ETH_DROPTCPIPCHECKSUMERRORFRAME_ENABLE;
    dmainit.ReceiveStoreForward = ETH_RECEIVESTOREFORWARD_ENABLE;
    dmainit.FlushReceivedFrame = ETH_FLUSHRECEIVEDFRAME_ENABLE;
    dmainit.TransmitStoreForward = ETH_TRANSMITSTOREFORWARD_ENABLE;
    dmainit.TransmitThresholdControl = ETH_TRANSMITTHRESHOLDCONTROL_64BYTES;
    dmainit.ForwardErrorFrames = ETH_FORWARDERRORFRAMES_DISABLE;
    dmainit.ForwardUndersizedGoodFrames = ETH_FORWARDUNDERSIZEDGOODFRAMES_DISABLE;
    dmainit.ReceiveThresholdControl = ETH_RECEIVEDTHRESHOLDCONTROL_64BYTES;
    dmainit.SecondFrameOperate = ETH_SECONDFRAMEOPERARTE_ENABLE;
    dmainit.AddressAlignedBeats = ETH_ADDRESSALIGNEDBEATS_ENABLE;
    dmainit.FixedBurst = ETH_FIXEDBURST_ENABLE;
    dmainit.RxDMABurstLength = ETH_RXDMABURSTLENGTH_32BEAT;
    dmainit.TxDMABurstLength = ETH_TXDMABURSTLENGTH_32BEAT;
    dmainit.EnhancedDescriptorFormat = ETH_DMAENHANCEDDESCRIPTOR_ENABLE;
    dmainit.DescriptorSkipLength = 0x0U;
    dmainit.DMAArbitration = ETH_DMAARBITRATION_ROUNDROBIN_RXTX_1_1;

    /* Get the ETHERNET DMAOMR value */
    tmpreg1 = ETH->DMAOMR;
    /* Clear xx bits */
    tmpreg1 &= ETH_DMAOMR_CLEAR_MASK;

    /* Set the DT bit according to ETH DropTCPIPChecksumErrorFrame value */
    /* Set the RSF bit according to ETH ReceiveStoreForward value */
    /* Set the DFF bit according to ETH FlushReceivedFrame value */
    /* Set the TSF bit according to ETH TransmitStoreForward value */
    /* Set the TTC bit according to ETH TransmitThresholdControl value */
    /* Set the FEF bit according to ETH ForwardErrorFrames value */
    /* Set the FUF bit according to ETH ForwardUndersizedGoodFrames value */
    /* Set the RTC bit according to ETH ReceiveThresholdControl value */
    /* Set the OSF bit according to ETH SecondFrameOperate value */
    tmpreg1 |= (uint32_t)(dmainit.DropTCPIPChecksumErrorFrame |
                          dmainit.ReceiveStoreForward |
                          dmainit.FlushReceivedFrame |
                          dmainit.TransmitStoreForward |
                          dmainit.TransmitThresholdControl |
                          dmainit.ForwardErrorFrames |
                          dmainit.ForwardUndersizedGoodFrames |
                          dmainit.ReceiveThresholdControl |
                          dmainit.SecondFrameOperate);

    /* Write to ETHERNET DMAOMR */
    ETH->DMAOMR = (uint32_t)tmpreg1;

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->DMAOMR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->DMAOMR = tmpreg1;

    /*----------------------- ETHERNET DMABMR Configuration ------------------*/
    /* Set the AAL bit according to ETH AddressAlignedBeats value */
    /* Set the FB bit according to ETH FixedBurst value */
    /* Set the RPBL and 4*PBL bits according to ETH RxDMABurstLength value */
    /* Set the PBL and 4*PBL bits according to ETH TxDMABurstLength value */
    /* Set the Enhanced DMA descriptors bit according to ETH EnhancedDescriptorFormat value*/
    /* Set the DSL bit according to ETH DesciptorSkipLength value */
    /* Set the PR and DA bits according to ETH DMAArbitration value */
    ETH->DMABMR = (uint32_t)(dmainit.AddressAlignedBeats |
                             dmainit.FixedBurst |
                             dmainit.RxDMABurstLength |    /* !! if 4xPBL is selected for Tx or Rx it is applied for the other */
                             dmainit.TxDMABurstLength |
                             dmainit.EnhancedDescriptorFormat |
                             (dmainit.DescriptorSkipLength << 2U) |
                             dmainit.DMAArbitration |
                             ETH_DMABMR_USP); /* Enable use of separate PBL for Rx and Tx */

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->DMABMR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->DMABMR = tmpreg1;

    /* Enable the Ethernet Rx Interrupt */
    ETH->DMAIER |= ETH_DMA_IT_NIS | ETH_DMA_IT_R;

    /* Initialize MAC address in ethernet MAC */
    ETH_MACAddressConfig(ETH_MAC_ADDRESS0, heth->Init.MACAddr);
}

/**
  * @brief  Configures the selected MAC address.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  MacAddr The MAC address to configure
  *          This parameter can be one of the following values:
  *             @arg ETH_MAC_Address0: MAC Address0
  *             @arg ETH_MAC_Address1: MAC Address1
  *             @arg ETH_MAC_Address2: MAC Address2
  *             @arg ETH_MAC_Address3: MAC Address3
  * @param  Addr Pointer to MAC address buffer data (6 bytes)
  * @retval HAL status
  */
static void ETH_MACAddressConfig(uint32_t MacAddr, uint8_t *Addr)
{
    uint32_t tmpreg1;

    /* Calculate the selected MAC address high register */
    tmpreg1 = ((uint32_t)Addr[5U] << 8U) | (uint32_t)Addr[4U];
    /* Load the selected MAC address high register */
    (*(__IO uint32_t *)((uint32_t)(ETH_MAC_ADDR_HBASE + MacAddr))) = tmpreg1;
    /* Calculate the selected MAC address low register */
    tmpreg1 = ((uint32_t)Addr[3U] << 24U) | ((uint32_t)Addr[2U] << 16U) | ((uint32_t)Addr[1U] << 8U) | Addr[0U];

    /* Load the selected MAC address low register */
    (*(__IO uint32_t *)((uint32_t)(ETH_MAC_ADDR_LBASE + MacAddr))) = tmpreg1;
}

/**
  * @brief  Enables the MAC transmission.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_MACTransmissionEnable(void)
{
    __IO uint32_t tmpreg1 = 0U;

    /* Enable the MAC transmission */
    ETH->MACCR |= ETH_MACCR_TE;

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->MACCR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->MACCR = tmpreg1;
}

/**
  * @brief  Disables the MAC transmission.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_MACTransmissionDisable(void)
{
    __IO uint32_t tmpreg1 = 0U;

    /* Disable the MAC transmission */
    ETH->MACCR &= ~ETH_MACCR_TE;

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->MACCR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->MACCR = tmpreg1;
}

/**
  * @brief  Enables the MAC reception.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_MACReceptionEnable(void)
{
    __IO uint32_t tmpreg1 = 0U;

    /* Enable the MAC reception */
    ETH->MACCR |= ETH_MACCR_RE;

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->MACCR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->MACCR = tmpreg1;
}

/**
  * @brief  Disables the MAC reception.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_MACReceptionDisable(void)
{
    __IO uint32_t tmpreg1 = 0U;

    /* Disable the MAC reception */
    ETH->MACCR &= ~ETH_MACCR_RE;

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->MACCR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->MACCR = tmpreg1;
}

/**
  * @brief  Enables the DMA transmission.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_DMATransmissionEnable(void)
{
    /* Enable the DMA transmission */
    ETH->DMAOMR |= ETH_DMAOMR_ST;
}

/**
  * @brief  Disables the DMA transmission.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_DMATransmissionDisable(void)
{
    /* Disable the DMA transmission */
    ETH->DMAOMR &= ~ETH_DMAOMR_ST;
}

/**
  * @brief  Enables the DMA reception.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_DMAReceptionEnable(void)
{
    /* Enable the DMA reception */
    ETH->DMAOMR |= ETH_DMAOMR_SR;
}

/**
  * @brief  Disables the DMA reception.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_DMAReceptionDisable(void)
{
    /* Disable the DMA reception */
    ETH->DMAOMR &= ~ETH_DMAOMR_SR;
}

/**
  * @brief  Clears the ETHERNET transmit FIFO.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_FlushTransmitFIFO(void)
{
    __IO uint32_t tmpreg1 = 0U;

    /* Set the Flush Transmit FIFO bit */
    ETH->DMAOMR |= ETH_DMAOMR_FTF;

    /* Wait until the write operation will be taken into account:
       at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = ETH->DMAOMR;
    vTaskDelay(ETH_REG_WRITE_DELAY);
    ETH->DMAOMR = tmpreg1;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
