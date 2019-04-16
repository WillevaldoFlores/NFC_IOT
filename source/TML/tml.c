/*
*         Copyright (c), NXP Semiconductors Caen / France
*
*                     (C)NXP Semiconductors
*       All rights are reserved. Reproduction in whole or in part is
*      prohibited without the written consent of the copyright owner.
*  NXP reserves the right to make changes without notice at any time.
* NXP makes no warranty, expressed, implied or statutory, including but
* not limited to any implied warranty of merchantability or fitness for any
*particular purpose, or that the use will not infringe any third party patent,
* copyright or trademark. NXP must not be liable for any loss or damage
*                          arising from its use.
*/

#include <stdint.h>
#include "board.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_pint.h"
#include "fsl_inputmux.h"
#include "pin_mux.h"

#include <tool.h>


SemaphoreHandle_t IrqSem = NULL;
i2c_master_transfer_t masterXfer;
//#define NXPNCI_IRQ kINPUTMUX_GpioPort0Pin16ToPintsel

typedef enum {ERROR = 0, SUCCESS = !ERROR} Status;

void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	//PRINTF("\f\r\nPINT Pin Interrupt %d event detected.", pintr);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(IrqSem, &xHigherPriorityTaskWoken);
}

static status_t I2C_WRITE(uint8_t *pBuff, uint16_t buffLen)
{
    masterXfer.direction = kI2C_Write;
    masterXfer.data = pBuff;
    masterXfer.dataSize = buffLen;

    return I2C_MasterTransferBlocking(I2C2, &masterXfer);
}

static status_t I2C_READ(uint8_t *pBuff, uint16_t buffLen)
{
    masterXfer.direction = kI2C_Read;
    masterXfer.data = pBuff;
    masterXfer.dataSize = buffLen;

    return I2C_MasterTransferBlocking(I2C2, &masterXfer);
}

static Status tml_Init(void) {
    i2c_master_config_t masterConfig;
    uint32_t sourceClock;
    uint32_t priotiry = 0;

    gpio_pin_config_t ven_config = {kGPIO_DigitalOutput, 0,};

    /*NXPNCI_VEN_PIN init GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio2);
    GPIO_PinInit(NXPNCI_VEN_GPIO, NXPNCI_VEN_PORT, NXPNCI_VEN_PIN, &ven_config);

    /* Connect trigger sources to PINT */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt3, kINPUTMUX_GpioPort0Pin16ToPintsel);

    /* Initialize PINT */
    PINT_Init(PINT);
    /* Setup Pin Interrupt 3 for rising edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt3, kPINT_PinIntEnableRiseEdge, pint_intr_callback);

    //priotiry = NVIC_GetPriority(7);
    NVIC_SetPriority(7,5);
    //priotiry = NVIC_GetPriority(7);

    /* Enable callbacks for PINT */
    PINT_EnableCallback(PINT);

    /* attach 12 MHz clock to FLEXCOMM2 (I2C master) */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);

    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = NXPNCI_I2C_BAUDRATE;
    sourceClock = I2C_MASTER_CLOCK_FREQUENCY;
    masterXfer.slaveAddress = NXPNCI_I2C_ADDR;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.flags = kI2C_TransferDefaultFlag;
    I2C_MasterInit(I2C2, &masterConfig, sourceClock);

    IrqSem = xSemaphoreCreateBinary();

    return SUCCESS;
}

static Status tml_DeInit(void) {
	vSemaphoreDelete(IrqSem);
	GPIO_PortClear(NXPNCI_VEN_GPIO, NXPNCI_VEN_PORT, 1 << NXPNCI_VEN_PIN);
    return SUCCESS;
}

static Status tml_Reset(void) {
	GPIO_PortClear(NXPNCI_VEN_GPIO, NXPNCI_VEN_PORT, 1 << NXPNCI_VEN_PIN);
	Sleep(10);
	GPIO_PortSet(NXPNCI_VEN_GPIO, NXPNCI_VEN_PORT, 1 << NXPNCI_VEN_PIN);
	Sleep(10);
	return SUCCESS;
}

static Status tml_Tx(uint8_t *pBuff, uint16_t buffLen) {
    if (I2C_WRITE(pBuff, buffLen) != kStatus_Success)
    {
    	Sleep(10);
    	if(I2C_WRITE(pBuff, buffLen) != kStatus_Success)
    	{
    		return ERROR;
    	}
    }

	return SUCCESS;
}

static Status tml_Rx(uint8_t *pBuff, uint16_t buffLen, uint16_t *pBytesRead) {
    if(I2C_READ(pBuff, 3) == kStatus_Success)
    {
    	if ((pBuff[2] + 3) <= buffLen)
    	{
			if (pBuff[2] > 0)
			{
				if(I2C_READ(&pBuff[3], pBuff[2]) == kStatus_Success)
				{
					*pBytesRead = pBuff[2] + 3;
				}
				else return ERROR;
			} else
			{
				*pBytesRead = 3;
			}
    	}
		else return ERROR;
   }
    else return ERROR;

	return SUCCESS;
}

static Status tml_WaitForRx(uint32_t timeout) {
	if (xSemaphoreTake(IrqSem, (timeout==0)?(portMAX_DELAY):(portTICK_PERIOD_MS*timeout)) != pdTRUE)
		return ERROR;
	return SUCCESS;
}

void tml_Connect(void) {
	tml_Init();
	tml_Reset();
}

void tml_Disconnect(void) {
	tml_DeInit();
}

void tml_Send(uint8_t *pBuffer, uint16_t BufferLen, uint16_t *pBytesSent) {
	if(tml_Tx(pBuffer, BufferLen) == ERROR) *pBytesSent = 0;
	else *pBytesSent = BufferLen;
}

void tml_Receive(uint8_t *pBuffer, uint16_t BufferLen, uint16_t *pBytes, uint16_t timeout) {
	if (tml_WaitForRx(timeout) == ERROR)
		*pBytes = 0;
	else tml_Rx(pBuffer, BufferLen, pBytes);
}


