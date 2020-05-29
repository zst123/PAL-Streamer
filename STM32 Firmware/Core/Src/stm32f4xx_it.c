/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_pcd.h"
#include "usbd_def.h"
#include "usbd_core.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void My_PCD_IRQHandler(PCD_HandleTypeDef *hpcd) {
	/* avoid spurious interrupt */
	if (__HAL_PCD_IS_INVALID_INTERRUPT(hpcd)) {
		return;
	}

	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_MMIS)) {
		/* incorrect mode, acknowledge the interrupt */
		__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_MMIS);
	}

	USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t i, ep_intr, epint, epnum;
	uint32_t fifoemptymsk, temp;
	USB_OTG_EPTypeDef *ep;

	// ZST: A packet is available for download
	/* Handle RxQLevel Interrupt */
	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_RXFLVL)) {
		USB_MASK_INTERRUPT(hpcd->Instance, USB_OTG_GINTSTS_RXFLVL);

		temp = USBx->GRXSTSP;

		ep = &hpcd->OUT_ep[temp & USB_OTG_GRXSTSP_EPNUM];

		if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_DATA_UPDT)
		{
			if ((temp & USB_OTG_GRXSTSP_BCNT) != 0U)
			{
				(void)USB_ReadPacket(USBx, ep->xfer_buff,
						(uint16_t)((temp & USB_OTG_GRXSTSP_BCNT) >> 4));

				ep->xfer_buff += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
				ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
			}
		}
		else if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_SETUP_UPDT)
		{
			(void)USB_ReadPacket(USBx, (uint8_t *)hpcd->Setup, 8U);
			ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
		}
		else
		{
			/* ... */
		}
		USB_UNMASK_INTERRUPT(hpcd->Instance, USB_OTG_GINTSTS_RXFLVL);
	}

	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_OEPINT))
	{
		epnum = 0U;

		/* Read in the device interrupt bits */
		ep_intr = USB_ReadDevAllOutEpInterrupt(hpcd->Instance);

		while (ep_intr != 0U)
		{
			if ((ep_intr & 0x1U) != 0U)
			{
				epint = USB_ReadDevOutEPInterrupt(hpcd->Instance, (uint8_t)epnum);

				if ((epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC)
				{
					CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_XFRC);
					//(void)PCD_EP_OutXfrComplete_int(hpcd, epnum);
				}

				if ((epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP)
				{
					CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STUP);
					/* Class B setup phase done for previous decoded setup */
					//(void)PCD_EP_OutSetupPacket_int(hpcd, epnum);
				}

				if ((epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS)
				{
					CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPDIS);
				}

				/* Clear Status Phase Received interrupt */
				if ((epint & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR)
				{
					CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
				}

				/* Clear OUT NAK interrupt */
				if ((epint & USB_OTG_DOEPINT_NAK) == USB_OTG_DOEPINT_NAK)
				{
					CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_NAK);
				}
			}
			epnum++;
			ep_intr >>= 1U;
		}
	}

	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_IEPINT)) {
		/* Read in the device interrupt bits */
		ep_intr = USB_ReadDevAllInEpInterrupt(hpcd->Instance);

		epnum = 0U;

		while (ep_intr != 0U) {
			if ((ep_intr & 0x1U) != 0U) { /* In ITR */
				epint = USB_ReadDevInEPInterrupt(hpcd->Instance, (uint8_t)epnum);

				if ((epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC) {
					fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
					USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;

					CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_XFRC);

					if (hpcd->Init.dma_enable == 1U) {
						hpcd->IN_ep[epnum].xfer_buff += hpcd->IN_ep[epnum].maxpacket;

						/* this is ZLP, so prepare EP0 for next setup */
						if ((epnum == 0U) && (hpcd->IN_ep[epnum].xfer_len == 0U)) {
							/* prepare to rx more setup packets */
							(void)USB_EP0_OutStart(hpcd->Instance, 1U, (uint8_t *)hpcd->Setup);
						}
					}
					HAL_PCD_DataInStageCallback(hpcd, (uint8_t)epnum);
				}
				if ((epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC) {
					CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_TOC);
				}
				if ((epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE) {
					CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_ITTXFE);
				}
				if ((epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE) {
					CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_INEPNE);
				}
				if ((epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD) {
					CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_EPDISD);
				}
				if ((epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE) {
					//(void)PCD_WriteEmptyTxFifo(hpcd, epnum);
				}
			}
			epnum++;
			ep_intr >>= 1U;
		}
	}

	/* Handle Resume Interrupt */
	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_WKUINT)) {
		/* Clear the Remote Wake-up Signaling */
		USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;

		if (hpcd->LPM_State == LPM_L1) {
			hpcd->LPM_State = LPM_L0;
			HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L0_ACTIVE);
		} else {
			HAL_PCD_ResumeCallback(hpcd);
		}
		__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_WKUINT);
	}

	/* Handle Suspend Interrupt */
	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_USBSUSP)) {
		if ((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS) {
			HAL_PCD_SuspendCallback(hpcd);
		}
		__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_USBSUSP);
	}

	/* Handle Reset Interrupt */
	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_USBRST))
	{
		USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
		(void)USB_FlushTxFifo(hpcd->Instance, 0x10U);

		for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
		{
			USBx_INEP(i)->DIEPINT = 0xFB7FU;
			USBx_INEP(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
			USBx_INEP(i)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
			USBx_OUTEP(i)->DOEPINT = 0xFB7FU;
			USBx_OUTEP(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
			USBx_OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
		}
		USBx_DEVICE->DAINTMSK |= 0x10001U;

		if (hpcd->Init.use_dedicated_ep1 != 0U)
		{
			USBx_DEVICE->DOUTEP1MSK |= USB_OTG_DOEPMSK_STUPM |
					USB_OTG_DOEPMSK_XFRCM |
					USB_OTG_DOEPMSK_EPDM;

			USBx_DEVICE->DINEP1MSK |= USB_OTG_DIEPMSK_TOM |
					USB_OTG_DIEPMSK_XFRCM |
					USB_OTG_DIEPMSK_EPDM;
		}
		else
		{
			USBx_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_STUPM |
					USB_OTG_DOEPMSK_XFRCM |
					USB_OTG_DOEPMSK_EPDM |
					USB_OTG_DOEPMSK_OTEPSPRM |
					USB_OTG_DOEPMSK_NAKM;

			USBx_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_TOM |
					USB_OTG_DIEPMSK_XFRCM |
					USB_OTG_DIEPMSK_EPDM;
		}

		/* Set Default Address to 0 */
		USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

		/* setup EP0 to receive SETUP packets */
		(void)USB_EP0_OutStart(hpcd->Instance, (uint8_t)hpcd->Init.dma_enable,
				(uint8_t *)hpcd->Setup);

		__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_USBRST);
	}

	/* Handle Enumeration done Interrupt */
	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_ENUMDNE)) {
		(void)USB_ActivateSetup(hpcd->Instance);
		hpcd->Init.speed = USB_GetDevSpeed(hpcd->Instance);

		/* Set USB Turnaround time */
		(void)USB_SetTurnaroundTime(hpcd->Instance,
				HAL_RCC_GetHCLKFreq(),
				(uint8_t)hpcd->Init.speed);

		HAL_PCD_ResetCallback(hpcd);
		__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_ENUMDNE);
	}

	/* Handle SOF Interrupt */
	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_SOF)) {
		//HAL_PCD_SOFCallback(hpcd);
		USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
		__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_SOF);
	}

	/* Handle Incomplete ISO IN Interrupt */
	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_IISOIXFR)) {
		/* Keep application checking the corresponding Iso IN endpoint causing the incomplete Interrupt */
		epnum = 0U;
		//HAL_PCD_ISOINIncompleteCallback(hpcd, (uint8_t)epnum);
		USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *) hpcd->pData, epnum);
		__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_IISOIXFR);
	}

	/* Handle Incomplete ISO OUT Interrupt */
	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)) {
		/* Keep application checking the corresponding Iso OUT endpoint causing the incomplete Interrupt */
		epnum = 0U;
		//HAL_PCD_ISOOUTIncompleteCallback(hpcd, (uint8_t)epnum);
		USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
		__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
	}

	/* Handle Connection event Interrupt */
	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_SRQINT)) {
		//HAL_PCD_ConnectCallback(hpcd);
		USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
		__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_SRQINT);
	}

	/* Handle Disconnection event Interrupt */
	if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_OTGINT)) {
		temp = hpcd->Instance->GOTGINT;
		if ((temp & USB_OTG_GOTGINT_SEDET) == USB_OTG_GOTGINT_SEDET) {
			// HAL_PCD_DisconnectCallback(hpcd);
			USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
		}
		hpcd->Instance->GOTGINT |= temp;
	}
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_usart2_rx;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

	if (__HAL_PCD_GET_FLAG(&hpcd_USB_OTG_FS, USB_OTG_GINTSTS_RXFLVL)) {
		USB_OTG_GlobalTypeDef *USBx = hpcd_USB_OTG_FS.Instance;
		uint32_t temp;
		USB_OTG_EPTypeDef *ep;

			USB_MASK_INTERRUPT(USBx, USB_OTG_GINTSTS_RXFLVL);

			temp = USBx->GRXSTSP;

			ep = &(hpcd_USB_OTG_FS.OUT_ep[temp & USB_OTG_GRXSTSP_EPNUM]);

			if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_DATA_UPDT)
			{
				if ((temp & USB_OTG_GRXSTSP_BCNT) != 0U)
				{
					(void)USB_ReadPacket(USBx, ep->xfer_buff,
							(uint16_t)((temp & USB_OTG_GRXSTSP_BCNT) >> 4));

					ep->xfer_buff += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
					ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
				}
			}
			else if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_SETUP_UPDT)
			{
				(void)USB_ReadPacket(USBx, (uint8_t *)hpcd_USB_OTG_FS.Setup, 8U);
				ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
			}
			USB_UNMASK_INTERRUPT(USBx, USB_OTG_GINTSTS_RXFLVL);
			return;
		}

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
