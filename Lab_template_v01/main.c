/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
 ******************************************************************************/
#define SPI_MASTER_INSTANCE         (1) /*! User change define to choose SPI instance */
#define TRANSFER_SIZE               (1)
#define TRANSFER_BAUDRATE           (500000U)           /*! Transfer baudrate - 500k */
#define MASTER_TRANSFER_TIMEOUT     (10000000U)             /*! Transfer timeout of master - 5s */

/*******************************************************************************
 * Variables
 ******************************************************************************/
// Buffer for storing data received by the SPI.
uint8_t s_spiSinkBuffer[TRANSFER_SIZE] = {0};
//char test = 1B;
//uint8_t s_spiSinkBuffer = {0x26};

// Buffer that supplies data to be transmitted with the SPI.
uint8_t s_spiSourceBuffer[TRANSFER_SIZE] = {0x37 <<1};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief SPI master blocking.
 *
 * Third function uses SPI master to send an array to slave
 * and receive the array back from slave,
 * then compare whether the two buffers are the same.
 */

int main (void)
{
	uint8_t loopCount = 0;
    uint32_t j;
    uint32_t failCount = 0;
    uint32_t calculatedBaudRate;
    spi_master_state_t spiMasterState;
    spi_master_user_config_t userConfig =
    {
#if FSL_FEATURE_SPI_16BIT_TRANSFERS
        .bitCount       = kSpi8BitMode,
#endif
        .polarity       = kSpiClockPolarity_ActiveLow,
        .phase          = kSpiClockPhase_FirstEdge,
        .direction      = kSpiMsbFirst,
        .bitsPerSec     = TRANSFER_BAUDRATE
    };
    // Init hardware
    hardware_init();
    // Init OSA layer.
    OSA_Init();

//   SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
//    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
//    PORTD_PCR4 |= PORT_PCR_MUX(0x2);
//     PORTD_PCR5 |= PORT_PCR_MUX(0x2);
//     PORTD_PCR6 |= PORT_PCR_MUX(0x2);
//     PORTD_PCR7 |= PORT_PCR_MUX(0x2);
//
//     SPI1_C1 |= (SPI_C1_MSTR_MASK | SPI_C1_SSOE_MASK );//Set SPI0 to Master & SS pin to auto SS
//
//       SPI1_BR |= (SPI_BR_SPPR(0x02) | SPI_BR_SPR(0x08));      //Set baud rate prescale divisor to 3 & set baud rate divisor to 64 for baud rate of 15625 hz
//           //Enable SPI0
//       SPI1_C2 |= (SPI_C2_MODFEN_MASK|SPI_C2_SPIMODE_MASK); //Master SS pin acts as slave select output
//       SPI1_C1 |= (SPI_C1_SPE_MASK);                           // Enable SPI module


//    PRINTF("\r\nSPI board to board blocking example");
//    PRINTF("\r\nThis example run on instance %d", (uint32_t)SPI_MASTER_INSTANCE);
//    PRINTF("\r\nBe sure master's SPI%d and slave's SPI%d are connected\r\n",
//                    (uint32_t)SPI_MASTER_INSTANCE, (uint32_t)SPI_MASTER_INSTANCE);

    // Init and setup baudrate for the master
    SPI_DRV_MasterInit(SPI_MASTER_INSTANCE, &spiMasterState);

    SPI_DRV_MasterConfigureBus(SPI_MASTER_INSTANCE,&userConfig,&calculatedBaudRate);
    PRINTF("The baudrate desired is 9600. calculated is %i\nbits per second is %i\n\r",calculatedBaudRate, userConfig.bitsPerSec);
    // Check if the configuration is correct
//    if (calculatedBaudRate > userConfig.bitsPerSec)
//    {
//
//        PRINTF("\r**Something failed in the master bus config \r\n");
//        return -1;
//    }
//    else
//    {
//        PRINTF("\r\nBaud rate in Hz is: %d\r\n", calculatedBaudRate);
//    }
    while(1)
    {
    	 PRINTF("Inside while 1 \n");
        // Initialize the source buffer
//        for (j = 0; j < 16; j++)
//        {
//            s_spiSourceBuffer[j] = 0x30;
//        }

        // Reset the sink buffer
        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            s_spiSinkBuffer[j] = 0;
        }
    	 PRINTF("MY CODE.\n");
         //Start transfer data to slave
        if (SPI_DRV_MasterTransferBlocking(SPI_MASTER_INSTANCE, NULL,s_spiSourceBuffer,s_spiSinkBuffer, TRANSFER_SIZE, MASTER_TRANSFER_TIMEOUT) == kStatus_SPI_Timeout)
        {
            PRINTF("\r\n**Sync transfer timed-out \r\n");
        }

        // Delay sometime to wait slave receive and send back data
        OSA_TimeDelay(500U);

//        // Start receive data from slave by transmit NULL bytes
//        if (SPI_DRV_MasterTransferBlocking(SPI_MASTER_INSTANCE, NULL, NULL,s_spiSinkBuffer, TRANSFER_SIZE, MASTER_TRANSFER_TIMEOUT) == kStatus_SPI_Timeout)
//        {
//            PRINTF("\r\n**Sync transfer timed-out \r\n");
//        }

        // Verify the contents of the master sink buffer
        // refer to the slave driver for the expected data pattern
        failCount = 0; // reset failCount variable

        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            if (s_spiSinkBuffer[j] != 0x1B)
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
