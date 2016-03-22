/*
 * Version3.c
 *
 *  Created on: 21 Mar 2016
 *      Author: Aonghus
 */
#include <stdio.h>
 // SDK Included Files
#include "board.h"
#include "fsl_spi_master_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_debug_console.h"

#define SPI_MASTER_INSTANCE         (1) /*! User change define to choose SPI instance */
#define TRANSFER_SIZE               (1)
#define TRANSFER_BAUDRATE           (500000U)           /*! Transfer baudrate - 500k */
#define MASTER_TRANSFER_TIMEOUT     (10000000U)             /*! Transfer timeout of master - 5s */
#define MF_KEY_SIZE					 6
uint8_t s_spiSinkBuffer[TRANSFER_SIZE] = {0};
uint8_t s_spiSourceBuffer[2] = {0};
#define MAX_LEN 16

void PCD_WriteRegister(uint8_t reg,uint8_t value);
void PCD_WriteRegister2(uint8_t reg,uint8_t* value,uint8_t** data);
uint8_t PCD_ReadRegister(uint8_t reg,uint8_t value);
uint8_t PCD_ReadRegister2(uint8_t reg,uint8_t n,uint8_t data,uint8_t value);
bool isCard();
uint8_t  MFRC522Request(uint8_t reqMode, uint8_t *TagType);
uint8_t MFRC522ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen);
void PCD_SetRegisterBitMask(uint8_t reg,uint8_t mask);
void PCD_ClearRegisterBitMask(	uint8_t reg,uint8_t mask);

// Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
enum StatusCode {
		STATUS_OK				,	// Success
		STATUS_ERROR			,	// Error in communication
		STATUS_COLLISION		,	// Collission detected
		STATUS_TIMEOUT			,	// Timeout in communication.
		STATUS_NO_ROOM			,	// A buffer is not big enough.
		STATUS_INTERNAL_ERROR	,	// Internal error in the code. Should not happen ;-)
		STATUS_INVALID			,	// Invalid argument.
		STATUS_CRC_WRONG		,	// The CRC_A does not match
		STATUS_MIFARE_NACK		= 0xff	// A MIFARE PICC responded with NAK.
	}StatusCode;

#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

typedef	enum PCD_Register {
			// Page 0: Command and status
			//						  0x00			// reserved for future use
			CommandReg				= 0x01 << 1,	// starts and stops command execution
			ComIEnReg				= 0x02 << 1,	// enable and disable interrupt request control bits
			DivIEnReg				= 0x03 << 1,	// enable and disable interrupt request control bits
			ComIrqReg				= 0x04 << 1,	// interrupt request bits
			DivIrqReg				= 0x05 << 1,	// interrupt request bits
			ErrorReg				= 0x06 << 1,	// error bits showing the error status of the last command executed
			Status1Reg				= 0x07 << 1,	// communication status bits
			Status2Reg				= 0x08 << 1,	// receiver and transmitter status bits
			FIFODataReg				= 0x09 << 1,	// input and output of 64 byte FIFO buffer
			FIFOLevelReg			= 0x0A << 1,	// number of bytes stored in the FIFO buffer
			WaterLevelReg			= 0x0B << 1,	// level for FIFO underflow and overflow warning
			ControlReg				= 0x0C << 1,	// miscellaneous control registers
			BitFramingReg			= 0x0D << 1,	// adjustments for bit-oriented frames
			CollReg					= 0x0E << 1,	// bit position of the first bit-collision detected on the RF interface
			//						  0x0F			// reserved for future use

			// Page 1: Command
			// 						  0x10			// reserved for future use
			ModeReg					= 0x11 << 1,	// defines general modes for transmitting and receiving
			TxModeReg				= 0x12 << 1,	// defines transmission data rate and framing
			RxModeReg				= 0x13 << 1,	// defines reception data rate and framing
			TxControlReg			= 0x14 << 1,	// controls the logical behavior of the antenna driver pins TX1 and TX2
			TxASKReg				= 0x15 << 1,	// controls the setting of the transmission modulation
			TxSelReg				= 0x16 << 1,	// selects the internal sources for the antenna driver
			RxSelReg				= 0x17 << 1,	// selects internal receiver settings
			RxThresholdReg			= 0x18 << 1,	// selects thresholds for the bit decoder
			DemodReg				= 0x19 << 1,	// defines demodulator settings
			// 						  0x1A			// reserved for future use
			// 						  0x1B			// reserved for future use
			MfTxReg					= 0x1C << 1,	// controls some MIFARE communication transmit parameters
			MfRxReg					= 0x1D << 1,	// controls some MIFARE communication receive parameters
			// 						  0x1E			// reserved for future use
			SerialSpeedReg			= 0x1F << 1,	// selects the speed of the serial UART interface

			// Page 2: Configuration
			// 						  0x20			// reserved for future use
			CRCResultRegH			= 0x21 << 1,	// shows the MSB and LSB values of the CRC calculation
			CRCResultRegL			= 0x22 << 1,
			// 						  0x23			// reserved for future use
			ModWidthReg				= 0x24 << 1,	// controls the ModWidth setting?
			// 						  0x25			// reserved for future use
			RFCfgReg				= 0x26 << 1,	// configures the receiver gain
			GsNReg					= 0x27 << 1,	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation
			CWGsPReg				= 0x28 << 1,	// defines the conductance of the p-driver output during periods of no modulation
			ModGsPReg				= 0x29 << 1,	// defines the conductance of the p-driver output during periods of modulation
			TModeReg				= 0x2A << 1,	// defines settings for the internal timer
			TPrescalerReg			= 0x2B << 1,	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
			TReloadRegH				= 0x2C << 1,	// defines the 16-bit timer reload value
			TReloadRegL				= 0x2D << 1,
			TCounterValueRegH		= 0x2E << 1,	// shows the 16-bit timer value
			TCounterValueRegL		= 0x2F << 1,

			// Page 3: Test Registers
			// 						  0x30			// reserved for future use
			TestSel1Reg				= 0x31 << 1,	// general test signal configuration
			TestSel2Reg				= 0x32 << 1,	// general test signal configuration
			TestPinEnReg			= 0x33 << 1,	// enables pin output driver on pins D1 to D7
			TestPinValueReg			= 0x34 << 1,	// defines the values for D1 to D7 when it is used as an I/O bus
			TestBusReg				= 0x35 << 1,	// shows the status of the internal test bus
			AutoTestReg				= 0x36 << 1,	// controls the digital self test
			VersionReg				= 0x37 << 1,	// shows the software version
			AnalogTestReg			= 0x38 << 1,	// controls the pins AUX1 and AUX2
			TestDAC1Reg				= 0x39 << 1,	// defines the test value for TestDAC1
			TestDAC2Reg				= 0x3A << 1,	// defines the test value for TestDAC2
			TestADCReg				= 0x3B << 1		// shows the value of ADC I and Q channels
			// 						  0x3C			// reserved for production tests
			// 						  0x3D			// reserved for production tests
			// 						  0x3E			// reserved for production tests
			// 						  0x3F			// reserved for production tests
		}PCD_Register;
// MFRC522 commands. Described in chapter 10 of the datasheet.
typedef	enum PCD_Command {
			PCD_Idle				= 0x00,		// no action, cancels current command execution
			PCD_Mem					= 0x01,		// stores 25 bytes into the internal buffer
			PCD_GenerateRandomID	= 0x02,		// generates a 10-byte random ID number
			PCD_CalcCRC				= 0x03,		// activates the CRC coprocessor or performs a self test
			PCD_Transmit			= 0x04,		// transmits data from the FIFO buffer
			PCD_NoCmdChange			= 0x07,		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
			PCD_Receive				= 0x08,		// activates the receiver circuits
			PCD_Transceive 			= 0x0C,		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
			PCD_MFAuthent 			= 0x0E,		// performs the MIFARE standard authentication as a reader
			PCD_SoftReset			= 0x0F		// resets the MFRC522
		}PCD_Command;

// MFRC522 RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the PCD).
// Described in 9.3.3.6 / table 98 of the datasheet at http://www.nxp.com/documents/data_sheet/MFRC522.pdf
typedef	enum PCD_RxGain {
			RxGain_18dB				= 0x00 << 4,	// 000b - 18 dB, minimum
			RxGain_23dB				= 0x01 << 4,	// 001b - 23 dB
			RxGain_18dB_2			= 0x02 << 4,	// 010b - 18 dB, it seems 010b is a duplicate for 000b
			RxGain_23dB_2			= 0x03 << 4,	// 011b - 23 dB, it seems 011b is a duplicate for 001b
			RxGain_33dB				= 0x04 << 4,	// 100b - 33 dB, average, and typical default
			RxGain_38dB				= 0x05 << 4,	// 101b - 38 dB
			RxGain_43dB				= 0x06 << 4,	// 110b - 43 dB
			RxGain_48dB				= 0x07 << 4,	// 111b - 48 dB, maximum
			RxGain_min				= 0x00 << 4,	// 000b - 18 dB, minimum, convenience for RxGain_18dB
			RxGain_avg				= 0x04 << 4,	// 100b - 33 dB, average, convenience for RxGain_33dB
			RxGain_max				= 0x07 << 4		// 111b - 48 dB, maximum, convenience for RxGain_48dB
		}PCD_RxGain;
// Commands sent to the PICC.
typedef	enum PICC_Command{
			// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
			PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
			PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
			PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
			PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
			PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
			PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
			PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
			// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
			// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
			// The read/write commands can also be used for MIFARE Ultralight.
			PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A
			PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
			PICC_CMD_MF_READ		= 0x30,		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
			PICC_CMD_MF_WRITE		= 0xA0,		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
			PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
			PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
			PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
			PICC_CMD_MF_TRANSFER	= 0xB0,		// Writes the contents of the internal data register to a block.
			// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
			// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
			PICC_CMD_UL_WRITE		= 0xA2		// Writes one 4 byte page to the PICC.
} PICC_Command;
void PCD_SetRegisterBitMask(uint8_t reg,uint8_t mask) {
	uint8_t tmp;
	uint8_t msk;
	tmp = PCD_ReadRegister(reg,0x00);
	msk = tmp|mask;
	PRINTF("Tmp: %02X, Mask: %02X, OR'ed: %02X",tmp,mask,msk);
	PCD_WriteRegister(reg,msk);			// set bit mask
} // End PCD_SetRegisterBitMask()

void PCD_ClearRegisterBitMask(	uint8_t reg,uint8_t mask) {
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg,0x00);
	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()
void PCD_WriteRegister(uint8_t reg,uint8_t value) {
	uint32_t j;
	 s_spiSourceBuffer[0] = reg;
	 s_spiSourceBuffer[1] = value;
	 if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, s_spiSourceBuffer,NULL, 2) != kStatus_SPI_Success)
	 {
		 PRINTF("\r**ASync transfer failed \r\n");
	 }
	 while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
	 {
	 }
} // End PCD_WriteRegister()
void PCD_WriteRegister2(uint8_t reg,uint8_t* value,uint8_t** data) {
	uint32_t j;
	s_spiSourceBuffer[0] = reg&0x7E;
	 if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, s_spiSourceBuffer,NULL, 2) != kStatus_SPI_Success)
	 {
		 PRINTF("\r**ASync transfer failed \r\n");
	 }
	 while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
	 {
	 }
} // End PCD_WriteRegister()


uint8_t PCD_ReadRegister(uint8_t reg,uint8_t value) {
	uint32_t j;
	//printf("this is the READ register\n");
	 for (j = 0; j < TRANSFER_SIZE; j++)
	 {
		 	 s_spiSourceBuffer[j] = 0x80 | (reg  & 0x7E);
	 }
	 // Start transfer data to slave
	         if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, s_spiSourceBuffer,NULL, TRANSFER_SIZE) != kStatus_SPI_Success)
	         {
	             PRINTF("\r**ASync transfer failed \r\n");
	         }
	         while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
	         {
	         }

	         // Start receive data from slave by transmit NULL bytes
	         if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, NULL,s_spiSinkBuffer, TRANSFER_SIZE) != kStatus_SPI_Success)
	         {
	             PRINTF("\r**Sync transfer failed \r\n");
	         }
	         while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
	         {
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
	                 return s_spiSinkBuffer[0];
} // End PCD_WriteRegister()
uint8_t PCD_ReadRegister2(uint8_t reg,uint8_t n,uint8_t data,uint8_t value) {
	uint32_t j;
	//printf("this is the READ register\n");
	 for (j = 0; j < TRANSFER_SIZE; j++)
	 {
		 	 s_spiSourceBuffer[j] = 0x80 | (reg  & 0x7E);
	 }
	 // Start transfer data to slave
	         if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, s_spiSourceBuffer,NULL, TRANSFER_SIZE) != kStatus_SPI_Success)
	         {
	             PRINTF("\r**ASync transfer failed \r\n");
	         }
	         while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
	         {
	         }

	         // Start receive data from slave by transmit NULL bytes
	         if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, NULL,s_spiSinkBuffer, TRANSFER_SIZE) != kStatus_SPI_Success)
	         {
	             PRINTF("\r**Sync transfer failed \r\n");
	         }
	         while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
	         {
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
	                 return s_spiSinkBuffer[0];
} // End PCD_WriteRegister()

void PCD_Init() {
	PCD_WriteRegister(0x2A, 0x8D);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(0x2B, 0x3E);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25's.
	PCD_WriteRegister(0x2C, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(0x2D, 0xE8);
	PCD_WriteRegister(0x15, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(0x16, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
}



bool isCard()
{
	uint8_t status;
	uint8_t str[MAX_LEN];

	status = MFRC522Request(0x26<<1, str);
   if (status == MI_OK) {
		return true;
	} else {
		return false;
	}
}

uint8_t  MFRC522Request(uint8_t reqMode, uint8_t *TagType)
{
	uint8_t status;
	uint16_t backBits;			//   RecibiÃ³ bits de datos

	PCD_WriteRegister(BitFramingReg, 0x07);		//TxLastBists = BitFramingReg[2..0]	???

	TagType[0] = reqMode;
	status = MFRC522ToCard(PCD_Transceive, TagType, 1, TagType, &backBits);

	if ((status != MI_OK) || (backBits != 0x10))
	{
		status = MI_ERR;
	}

	return status;
}

uint8_t MFRC522ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen)
{
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
	uint8_t lastBits;
    uint8_t n;
    uint16_t i;

    switch (command)
    {
        case PCD_MFAuthent:		// Tarjetas de certificaciÃ³n cerca
		{
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_Transceive:	//La transmisiÃ³n de datos FIFO
		{
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
    }


    PCD_WriteRegister(ComIEnReg, irqEn|0x80);	//De solicitud de interrupciÃ³n
    PCD_ClearRegisterBitMask(ComIrqReg, 0x80);			// Borrar todos los bits de peticiÃ³n de interrupciÃ³n
    PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			//FlushBuffer=1, FIFO de inicializaciÃ³n

    PCD_WriteRegister(CommandReg, PCD_Idle);	//NO action;Y cancelar el comando

	//Escribir datos en el FIFO
    for (i=0; i<sendLen; i++)
    {
    	PCD_WriteRegister(FIFODataReg, sendData[i]);
	}

	//???? ejecutar el comando
    PCD_WriteRegister(CommandReg, command);
    if (command == PCD_Transceive)
    {
    	PCD_SetRegisterBitMask(BitFramingReg, 0x80);		//StartSend=1,transmission of data starts
	}

	// A la espera de recibir datos para completar
	i = 2000;	//i????????,??M1???????25ms	??? i De acuerdo con el ajuste de frecuencia de reloj, el tiempo mÃ¡ximo de espera operaciÃ³n M1 25ms tarjeta??
    do
    {
		//CommIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = PCD_ReadRegister(ComIrqReg,0x00);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    PCD_ClearRegisterBitMask(BitFramingReg, 0x80);			//StartSend=0

    if (i != 0)
    {
        if(!(PCD_ReadRegister(ErrorReg,0x00) & 0x1B))	//BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
				status = MI_NOTAGERR;			//??
			}

            if (command == PCD_Transceive)
            {
               	n = PCD_ReadRegister(FIFOLevelReg,0x00);
              	lastBits = PCD_ReadRegister(ControlReg,0x00) & 0x07;
                if (lastBits)
                {
					*backLen = (n-1)*8 + lastBits;
				}
                else
                {
					*backLen = n*8;
				}

                if (n == 0)
                {
					n = 1;
				}
                if (n > MAX_LEN)
                {
					n = MAX_LEN;
				}

				//??FIFO??????? Lea los datos recibidos en el FIFO
                for (i=0; i<n; i++)
                {
					backData[i] = PCD_ReadRegister(FIFODataReg,0x00);
				}
            }
        }
        else
        {
			status = MI_ERR;
		}

    }

    //SetBitMask(ControlReg,0x80);           //timer stops
    //Write_MFRC522(CommandReg, PCD_IDLE);

    return status;
}
int main (void)
{
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
        .polarity       = kSpiClockPolarity_ActiveLow,
        .phase          = kSpiClockPhase_SecondEdge,
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
    SPI_DRV_MasterConfigureBus(SPI_MASTER_INSTANCE,&userConfig,&calculatedBaudRate);

        PCD_Init();
        PRINTF("This code scan the MIFARE Classic NUID.\n\r");
        while(isCard() == false){
        	/*PRINTF("Waiting\r");*/}

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
        PCD_ReadRegister(0x37,0x00);
        // Wait for press any key.
        PRINTF("\r\nPress any key to run again\r\n");
        GETCHAR();
        loopCount++;
    }
}
/*******************************************************************************
 * EOF
 ******************************************************************************/



