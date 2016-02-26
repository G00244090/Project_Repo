/*
 * main2.c
 *
 *  Created on: 21 Feb 2016
 *      Author: Aonghus
 */

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////
// Standard C Included Files
#include <stdio.h>
 // SDK Included Files
#include "board.h"
#include "fsl_spi_master_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
// ******************************************************************************/
//#define SPI_MASTER_INSTANCE         BOARD_SPI_INSTANCE  /*! User change define to choose SPI instance */
//#define TRANSFER_SIZE               (64)
//#define TRANSFER_BAUDRATE           (500000U)           /*! Transfer baudrate - 500k */
//#define MASTER_TRANSFER_TIMEOUT     (5000U)             /*! Transfer timeout of master - 5s */
#define SPI_MASTER_INSTANCE         (1) /*! User change define to choose SPI instance */
#define TRANSFER_SIZE               (1)
#define TRANSFER_BAUDRATE           (500000U)           /*! Transfer baudrate - 500k */
#define MASTER_TRANSFER_TIMEOUT     (10000000U)             /*! Transfer timeout of master - 5s */
#define MF_KEY_SIZE					 6

/*******************************************************************************
 * Variables
 ******************************************************************************/
// Buffer for storing data received by the SPI.
uint8_t s_spiSinkBuffer[TRANSFER_SIZE] = {0};

// Buffer that supplies data to be transmitted with the SPI.
uint8_t s_spiSourceBuffer[TRANSFER_SIZE] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief SPI master non-blocking.
 *
 * Thid function uses SPI master to send an array to slave
 * and receive the array back from slave,
 * then compare whether the two buffers are the same.
 */

// A struct used for passing a MIFARE Crypto1 key
typedef struct {
	uint8_t keyByte[6];
} MIFARE_Key;


void PCD_WriteRegister(	uint8_t reg,		///< The register to write to. One of the PCD_Register enums.
		uint8_t value		///< The value to write.
									) {
	uint32_t j;
	//printf("this is the WRITE register\n");
	 for (j = 0; j < TRANSFER_SIZE; j++)
	 {
		 	 s_spiSourceBuffer[j] = reg << 1 & 0x7E;
	 }
} // End PCD_WriteRegister()

void PCD_ReadRegister(	uint8_t reg,		///< The register to write to. One of the PCD_Register enums.
		uint8_t value		///< The value to write.
									) {
	uint32_t j;
	//printf("this is the READ register\n");
	 for (j = 0; j < TRANSFER_SIZE; j++)
	 {
		 	 s_spiSourceBuffer[j] = 0x80 | (reg << 1 & 0x7E);
	 }
} // End PCD_WriteRegister()

void PCD_Init() {
	PCD_WriteRegister(0x2A, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(0x2B, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25's.
	PCD_WriteRegister(0x2C, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(0x2D, 0xE8);
	PCD_WriteRegister(0x15, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(0x16, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
}

int main (void)
{
	MIFARE_Key Key;
    uint8_t loopCount = 0;
    uint32_t j;
    int8_t i;
    uint32_t failCount = 0;
    uint32_t calculatedBaudRate;
    spi_master_state_t spiMasterState;
    spi_master_user_config_t userConfig =
    {
#if FSL_FEATURE_SPI_16BIT_TRANSFERS
        .bitCount       = kSpi8BitMode,
#endif
        .polarity       = kSpiClockPolarity_ActiveHigh,
        .phase          = kSpiClockPhase_FirstEdge,
        .direction      = kSpiMsbFirst,
        .bitsPerSec     = TRANSFER_BAUDRATE
    };

    // init the hardware, this also sets up up the SPI pins for each specific SoC
    hardware_init();
    // Init OSA layer.
    OSA_Init();


    //PCD_ReadRegister(0x37,0x00);
    PRINTF("\r\nSPI board to board non-blocking example");
    PRINTF("\r\nThis example run on instance %d", (uint32_t)SPI_MASTER_INSTANCE);
    PRINTF("\r\nBe sure master's SPI%d and slave's SPI%d are connected\r\n",
                    (uint32_t)SPI_MASTER_INSTANCE, (uint32_t)SPI_MASTER_INSTANCE);

    // Init and setup baudrate for the master
    SPI_DRV_MasterInit(SPI_MASTER_INSTANCE, &spiMasterState);
    SPI_DRV_MasterConfigureBus(SPI_MASTER_INSTANCE,
                                &userConfig,
                                &calculatedBaudRate);
    PCD_Init();
        for (i = 0; i < 6; i++) {
            Key.keyByte[i] = 0xFF;
          }

        PRINTF("This code scan the MIFARE Classsic NUID.\n");
        PRINTF("Using the following key:\n");
        PRINTF("%02X with %i \n",Key.keyByte, MF_KEY_SIZE);

    // Check if the configuration is correct
    if (calculatedBaudRate > userConfig.bitsPerSec)
    {
        PRINTF("\r**Something failed in the master bus config \r\n");
        return -1;
    }
    else
    {
        PRINTF("\r\nBaud rate in Hz is: %d\r\n", calculatedBaudRate);
    }

    while(1)
    {
        // Initialize the source buffer
//        for (j = 0; j < TRANSFER_SIZE; j++)
//        {
        	PCD_ReadRegister(0x37,0x00);
            //s_spiSourceBuffer[j] = 0x80 | (0x37 << 1 & 0x7E);
//        }

        // Reset the sink buffer
        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            s_spiSinkBuffer[j] = 0;
        }

        // Start transfer data to slave
        if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, s_spiSourceBuffer,NULL, TRANSFER_SIZE) != kStatus_SPI_Success)
        {
            PRINTF("\r**ASync transfer failed \r\n");
        }
        while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
        {
        }

        // Delay sometime to wait slave receive and send back data
        OSA_TimeDelay(500U);

        // Start receive data from slave by transmit NULL bytes
        if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, NULL,s_spiSinkBuffer, TRANSFER_SIZE) != kStatus_SPI_Success)
        {
            PRINTF("\r**Sync transfer failed \r\n");
        }
        while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
        {
        }

        // Verify the contents of the master sink buffer
        // refer to the slave driver for the expected data pattern
        failCount = 0; // reset failCount variable

        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            if (s_spiSinkBuffer[j] != 0x88)
            {
                 failCount++;
            }
        }

        // Print out transmit buffer.
        PRINTF("\r\nMaster transmit:");
        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            // Print 16 numbers in a line.
            if ((j & 0x0F) == 0)
            {
                PRINTF("\r\n    ");
            }
            PRINTF(" %02X", s_spiSourceBuffer[j]);
        }
        // Print out receive buffer.
        PRINTF("\r\nMaster receive:");
        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            // Print 16 numbers in a line.
            if ((j & 0x0F) == 0)
            {
                PRINTF("\r\n    ");
            }
            PRINTF(" %02X", s_spiSinkBuffer[j]);
        }

        if (failCount == 0)
        {
            PRINTF("\r\n Spi master transfer succeed! \r\n");
        }
        else
        {
            PRINTF("\r\n **failures detected in Spi master transfer! \r\n");
        }

        // Wait for press any key.
        PRINTF("\r\nPress any key to run again\r\n");
        GETCHAR();
        loopCount++;
    }
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

