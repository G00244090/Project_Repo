/*
 * basic.c
 *
 *  Created on: 9 Mar 2016
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

void PCD_WriteRegister(uint8_t reg,uint8_t value);
void PCD_WriteRegister2(uint8_t reg,uint8_t value,uint8_t data);
uint8_t PCD_ReadRegister(uint8_t reg,uint8_t value);
uint8_t PCD_ReadRegister2(uint8_t reg,uint8_t n,uint8_t data,uint8_t value);
void PCD_SetRegisterBitMask(uint8_t reg,uint8_t mask) ;
void PCD_ClearRegisterBitMask(	uint8_t reg,uint8_t mask);
bool is_present();
void PCD_AntennaOn() ;


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



/*******************************************************************************
 * Definitions
// ******************************************************************************/
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
uint8_t s_spiSourceBuffer[2] = {0};

// A struct used for passing a MIFARE Crypto1 key
typedef struct {
	uint8_t keyBytes[6];
} MIFARE_Key;
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
	//printf("this is the WRITE register\n");
//	 for (j = 0; j < TRANSFER_SIZE; j++)
//	 {
//		 	 s_spiSourceBuffer[0] = reg  & value;
//	 }
	 s_spiSourceBuffer[0] = reg&0x7E;
	 s_spiSourceBuffer[1] = value;
	 if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, s_spiSourceBuffer,NULL, 2) != kStatus_SPI_Success)
	 	         {
	 	             PRINTF("\r**ASync transfer failed \r\n");
	 	         }
	 	         while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
	 	         {
	 	         }

} // End PCD_WriteRegister()
void PCD_WriteRegister2(uint8_t reg,uint8_t value,uint8_t data) {
	uint32_t j;
	//printf("this is the WRITE register\n");
//	 for (j = 0; j < TRANSFER_SIZE; j++)
//	 {
//		 	 s_spiSourceBuffer[j] = reg  & value;
//	 }
	s_spiSourceBuffer[0] = reg&0x7E;
	 for (j = 0; j < value; j++) {
		 s_spiSourceBuffer[j+1] = data+j;
	 	}
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
		 	 s_spiSourceBuffer[j] = 0x80 | reg;
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
	                     PRINTF(" %02X\n", s_spiSourceBuffer[j]);
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
	                     PRINTF(" %02X\n", s_spiSinkBuffer[j]);
	                 }
	                 return s_spiSinkBuffer[0];
} // End PCD_WriteRegister()
uint8_t PCD_ReadRegister2(uint8_t reg,uint8_t n,uint8_t data,uint8_t value) {
	uint32_t j;
	//printf("this is the READ register\n");
	 for (j = 0; j < TRANSFER_SIZE; j++)
	 {
		 	 s_spiSourceBuffer[j] = 0x80 | reg;
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
enum StatusCode PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
									uint8_t length,	///< In: The number of bytes to transfer.
									uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 ) {
	int i = 5000;
	uint8_t n = 0x00;
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister2(FIFODataReg, length, *data);	// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
	PRINTF("\t\t\t\t\t calc crc\r");
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73s.

	while (1) {
		n = PCD_ReadRegister(DivIrqReg,0x00);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		if (n & 0x04) {						// CRCIRq bit set - calculation done
			break;
		}
		if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.

	// Transfer the result from the registers to the result buffer
	result[0] = PCD_ReadRegister(CRCResultRegL,0x00);
	result[1] = PCD_ReadRegister(CRCResultRegH,0x00);
	return STATUS_OK;
} // End PCD_CalculateCRC()

bool is_present()
{
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
	//PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
	uint8_t validBits;
	uint8_t rxAlign;
	bool crc;
	uint8_t waitIRq = 0x30;
	uint8_t n, _validBits;
	uint8_t backData;
	uint8_t backLen;
	uint8_t sendLen;
	uint8_t sendData;
	uint8_t command = PCD_Transceive;
	bool checkCRC = false;
	unsigned int i,delay;
	if (bufferATQA == NULL || bufferSize < 2) // The ATQA response is 2 bytes long.
	{
		return false;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	PRINTF("CollReg\r\n");
	PCD_ReadRegister(CollReg, 0x00);
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]


		PRINTF("\nTrying to communicate with tag!!\n\r");
		// Prepare values for BitFramingReg
		uint8_t txLastBits = _validBits ? validBits : 0;
		uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

		PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
		PRINTF("CommandReg\r\n");
		PCD_ReadRegister(CommandReg, 0x00);

		PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
		PRINTF("ComIrqReg\r\n");
		PCD_ReadRegister(ComIrqReg, 0x00);

		PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
		PRINTF("FIFOLevelReg\r\n");
		PCD_ReadRegister(FIFOLevelReg, 0x00);

		PCD_WriteRegister(FIFODataReg, 0x22);	// Write sendData to the FIFO
		PRINTF("FIFODataReg\r\n");
		PCD_ReadRegister(FIFODataReg, 0x00);

		PRINTF("BitFramingReg before\r\n");
		PCD_ReadRegister(BitFramingReg, 0x00);
		PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
		PRINTF("BitFramingReg\r\n");
		PCD_ReadRegister(BitFramingReg, 0x00);

		PCD_WriteRegister(CommandReg, command);				// Execute the command
		PRINTF("CommandReg\r\n");
		PCD_ReadRegister(CommandReg, 0x00);
		PRINTF("\nCommands Executed\n\r");

		if (command == PCD_Transceive) {
			PRINTF("Matches\n\r");
			PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
			PRINTF("BitFramingReg\r\n");
			PCD_ReadRegister(BitFramingReg, 0x00);
		}

		PRINTF("Status2\r\n");
		PCD_ReadRegister(Status2Reg, 0x00);
		// Wait for the command to complete.
		// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
		// Each iteration of the do-while-loop takes 17.86s.
		i = 2000;
		while (1) {
			PRINTF("\nReading ComIrqReg\r\n");
			n = PCD_ReadRegister(ComIrqReg,0x00);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
			PRINTF("\n**Returned %d Also %02X \n\r",n,n);
//			for(delay =0;delay <2000000;delay++)
//			{}
			if (n & waitIRq) {					// One of the interrupts that signal success has been set.
				PRINTF("Interrupt : %20X",n&waitIRq);
				break;
			}
			if (n & 0x01) {
				PRINTF("time out1\n\r");// Timer interrupt - nothing received in 25ms
				return false;
			}
			if (--i == 0) {
				PRINTF("n:%02X time out2\n\r",n);// The emergency break. If all other conditions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
				return false;
			}
		}
		return true;
		PRINTF("\nNow at error checking\n\r");
		// Stop now if any errors except collisions were detected.
		uint8_t errorRegValue = PCD_ReadRegister(ErrorReg,0x00); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
		if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
			PRINTF("error picc\n\r");
			return false;
		}

		// If the caller wants data back, get it from the MFRC522.
		//if (backData && backLen) {
			printf("\nReading FIFOLevelReg\r\n");
			n = PCD_ReadRegister(FIFOLevelReg,0x00);			// Number of bytes in the FIFO
			if (n > backLen) {
				return false;
			}
			backLen = n;											// Number of bytes returned
			PCD_ReadRegister2(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
			_validBits = PCD_ReadRegister(ControlReg,0x00) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
			if (validBits) {
				validBits = _validBits;
			}
		//}

		// Tell about collisions
		if (errorRegValue & 0x08) {		// CollErr
			return STATUS_COLLISION;
		}
		PRINTF("\nBackdata: %02X :: backlen: %02X ",backData,backLen);
		// Perform CRC_A validation if requested.
		//if (backData && backLen && checkCRC) {
			// In this case a MIFARE Classic NAK is not OK.
			if (backLen == 1 && _validBits == 4) {
				printf("\nNak error\r\n");
			}
			// We need at least the CRC_A value and all 8 bits of the last byte must be received.
			if (backLen < 2 || _validBits != 0) {
				printf("\nCrc wrong\r\n");
				return false;
			}
			else
				PRINTF("buffer has something");
			// Verify CRC_A - do our own calculation and store the control in controlBuffer.
			uint8_t controlBuffer[2];
//			enum StatusCode status = PCD_CalculateCRC(backData[0], backLen - 2, controlBuffer[0]);
//			if (status != STATUS_OK) {
//				return status;
//			}
//			if ((backData[backLen - 2] != controlBuffer[0]) || (backData[backLen - 1] != controlBuffer[1])) {
//				return STATUS_CRC_WRONG;
//			}
		//}

	if (bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return false;
	}


}
void PCD_Init() {
	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	PRINTF("Testing\r\n");
	PRINTF("Write TModeReg\r\n");
	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_ReadRegister(TModeReg, 0x00);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds

	PRINTF("TPrescalerReg\n");
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25s.
	PCD_ReadRegister(TPrescalerReg,0x00);

	PRINTF("TReloadRegH\n");
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_ReadRegister(TReloadRegH,0x00);

	PRINTF("TReloadRegL\n");
	PCD_WriteRegister(TReloadRegL, 0xE8);
	PCD_ReadRegister(TReloadRegL,0x00);

	PRINTF("TxASKReg\n");
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_ReadRegister(TxASKReg,0x00);

	PRINTF("ModeReg\n");
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_ReadRegister(ModeReg,0x00);

	PRINTF("RFCfgReg\n");
	PCD_WriteRegister(RFCfgReg, (0x07<<4)); // Set Rx Gain to max
	PCD_ReadRegister(RFCfgReg,0x00);

	PRINTF("ComIEnReg\n");
	PCD_WriteRegister(ComIEnReg,0x20);
	PCD_ReadRegister(ComIEnReg,0x00);

	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}

void PCD_AntennaOn() {
	uint8_t value = PCD_ReadRegister(TxControlReg,0x00);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
	}
} // End PCD_AntennaOn()
int main (void)
{
	MIFARE_Key Key;
    uint8_t loopCount = 0;
    uint32_t j;
    int8_t i;
    int8_t n;
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
    PRINTF("\r\nBe sure master's SPI%d and slave's SPI%d are connected\r\n",(uint32_t)SPI_MASTER_INSTANCE, (uint32_t)SPI_MASTER_INSTANCE);

    // Init and setup baudrate for the master
    SPI_DRV_MasterInit(SPI_MASTER_INSTANCE, &spiMasterState);
    SPI_DRV_MasterConfigureBus(SPI_MASTER_INSTANCE,&userConfig,&calculatedBaudRate);

        for (i = 0; i < 6; i++) {
            Key.keyBytes[i] = 0xFF;
          }
        PCD_Init();

        n = PCD_ReadRegister(VersionReg,0x00);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        PRINTF("\n**Version %02X \n\r",n);

        PRINTF("This code scan the MIFARE Classsic NUID.\n\r");
        //PRINTF("Using the following key:\n\r");
        //PRINTF("%d with %i \n\r",Key.keyBytes, MF_KEY_SIZE);
        while(1)
        {
//        for(j=0;j<MF_KEY_SIZE;j++)
//        	PRINTF("t: %02X\n\r",Key.keyBytes[j]);
        while(is_present() == false)
        {/*PRINTF("Waiting\r");*/}

        	PRINTF("\nExited the card detection loop\r\n");
        	PCD_ReadRegister(VersionReg,0x00);


        // Wait for press any key.
        PRINTF("\r\nPress any key to run again\r\n");
        GETCHAR();
        loopCount++;
    }
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

