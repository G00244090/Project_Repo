/*
 * basic2.c
 *
 *  Created on: 29 Mar 2016
 *      Author: Aonghus
 */




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
void PCD_WriteRegister2(uint8_t reg,uint8_t value,uint8_t data[]);
uint8_t PCD_ReadRegister(uint8_t reg,uint8_t value);
uint8_t PCD_ReadRegister2(uint8_t reg,uint8_t n,uint8_t data,uint8_t value);
void PCD_SetRegisterBitMask(uint8_t reg,uint8_t mask) ;
void PCD_ClearRegisterBitMask(	uint8_t reg,uint8_t mask);
bool is_present();
void PCD_AntennaOn();

enum StatusCode PCD_TransceiveData(	uint8_t *sendData,uint8_t sendLen,uint8_t *backData,uint8_t *backLen,uint8_t *validBits,uint8_t rxAlign,bool checkCRC);
enum StatusCode PICC_REQA_or_WUPA(	uint8_t command,uint8_t *bufferATQA,uint8_t *bufferSize);
enum StatusCode PICC_RequestA(	uint8_t *bufferATQA,uint8_t *bufferSize);
bool PICC_IsNewCardPresent();
enum StatusCode PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
											uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
											uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
											uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
											uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
											uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
											uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
											uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
											bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 );
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
#define TRANSFER_SIZE               (12)
#define TRANSFER_BAUDRATE           (500000U)           /*! Transfer baudrate - 500k */
#define MASTER_TRANSFER_TIMEOUT     (10000000U)             /*! Transfer timeout of master - 5s */
#define MF_KEY_SIZE					 6

/*******************************************************************************
 * Variables
 ******************************************************************************/
// Buffer for storing data received by the SPI.
uint8_t s_spiSinkBuffer[20] = {0};
// Buffer that supplies data to be transmitted with the SPI.
uint8_t s_spiSourceBuffer[20] = {0};

// A struct used for passing a MIFARE Crypto1 key
typedef struct {
	uint8_t keyBytes[6];
} MIFARE_Key;
typedef struct {
	uint8_t		size;			// Number of bytes in the UID. 4, 7 or 10.
	uint8_t		uidByte[10];
	uint8_t		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
	} Uid;
Uid uid;
bool PICC_ReadCardSerial();
enum  StatusCode PICC_Select(Uid *uid,uint8_t validBits);
enum StatusCode MIFARE_Read(	uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
		uint8_t *buffer,		///< The buffer to store the data in
		uint8_t *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
											) {
		enum StatusCode result;

		// Sanity check
		if (buffer == NULL || *bufferSize < 18) {
			return STATUS_NO_ROOM;
		}

		// Build command buffer
		buffer[0] = PICC_CMD_MF_READ;
		buffer[1] = blockAddr;
		// Calculate CRC_A
//		result = PCD_CalculateCRC(&buffer, 2, &buffer[2]);
//		if (result != STATUS_OK) {
//			return result;
//		}

		// Transmit the buffer and receive the response, validate CRC_A.
		return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, true);
	} // End MIFARE_Read()
enum StatusCode PCD_Authenticate(uint8_t command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
								 uint8_t blockAddr, 	///< The block number. See numbering in the comments in the .h file.
								 MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
								 Uid *uid			///< Pointer to Uid struct. The first 4 bytes of the UID is used.
											) {
	uint8_t waitIRq = 0x10;		// IdleIRq

	// Build command buffer
	uint8_t sendData[12];
	sendData[0] = command;
	sendData[1] = 7;
	for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {	// 6 key bytes
		sendData[2+i] = key->keyBytes[i];
	}
//	for (uint8_t i = 0; i < 4; i++) {				// The first 4 bytes of the UID
//		sendData[8+i] = uid->uidByte[i];
//	}
	sendData[8] = 0x35;
	sendData[9] = 0x8E;
	sendData[10] = 0xF3;
	sendData[11] = 0x29;


	// Start the authentication.
	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, sendData, sizeof(sendData),0x00,0x00,0x00,0x00,false);
} // End PCD_Authenticate()
enum StatusCode PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
									uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
									uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
									uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
									uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
									uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
									bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()
enum StatusCode PICC_REQA_or_WUPA(	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
		uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
		uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											) {
	uint8_t validBits;
	enum StatusCode status;

	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits,0x00,false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()
enum StatusCode PICC_RequestA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
		uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()
bool PICC_IsNewCardPresent() {
	uint8_t bufferATQA[2]={0x26,0x26};
	uint8_t bufferSize = sizeof(bufferATQA);
	enum StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()
void PCD_SetRegisterBitMask(uint8_t reg,uint8_t mask) {
	uint8_t tmp;
	uint8_t msk;
	tmp = PCD_ReadRegister(reg,0x00);
	msk = tmp|mask;
	PRINTF("\r\nTmp: %02X, Mask: %02X, OR'ed: %02X",tmp,mask,msk);
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
	 s_spiSourceBuffer[0] = (reg&0x7E);
	 s_spiSourceBuffer[1] = value;
	 if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, s_spiSourceBuffer,NULL, 2) != kStatus_SPI_Success)
	 	         {
	 	             PRINTF("\r**ASync transfer failed \r\n");
	 	         }
	 	         while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
	 	         {
	 	         }

} // End PCD_WriteRegister()
void PCD_WriteRegister2(uint8_t reg,uint8_t value,uint8_t data[]) {
	uint32_t j;
	size_t size = value+1;
	s_spiSourceBuffer[0] = reg&0x7E;
	 for (j = 0; j < value; j++) {
		 s_spiSourceBuffer[j+1] = data[j];
	 	}
	 if (SPI_DRV_MasterTransfer(SPI_MASTER_INSTANCE, NULL, s_spiSourceBuffer,NULL,size) != kStatus_SPI_Success)
	 {
		 PRINTF("\r**ASync transfer failed \r\n");
	 }
	 while (SPI_DRV_MasterGetTransferStatus(SPI_MASTER_INSTANCE, NULL) == kStatus_SPI_Busy)
	 {
	 }
//	}
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
void PICC_DumpMifareClassicSectorToSerial(Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
										  MIFARE_Key *key,	///< Key A for the sector.
										  uint8_t sector			///< The sector to dump, 0..39.
													) {
	enum StatusCode status;
	uint8_t firstBlock;		// Address of lowest address to dump actually last block dumped)
	uint8_t no_of_blocks;		// Number of blocks in sector
	bool isSectorTrailer;	// Set to true while handling the "last" (ie highest address) in the sector.

	// The access bits are stored in a peculiar fashion.
	// There are four groups:
	//		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
	//		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
	//		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
	//		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
	// Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
	// The four CX bits are stored together in a nible cx and an inverted nible cx_.
	uint8_t c1, c2, c3;		// Nibbles
	uint8_t c1_, c2_, c3_;		// Inverted nibbles
	bool invertedError;		// True if one of the inverted nibbles did not match
	uint8_t g[4];				// Access bits for each of the four groups.
	uint8_t group;				// 0-3 - active group for access bits
	bool firstInGroup;		// True for the first block dumped in the group

	// Determine position and size of sector.
	if (sector < 32) { // Sectors 0..31 has 4 blocks each
		no_of_blocks = 4;
		firstBlock = sector * no_of_blocks;
	}
	else if (sector < 40) { // Sectors 32-39 has 16 blocks each
		no_of_blocks = 16;
		firstBlock = 128 + (sector - 32) * no_of_blocks;
	}
	else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
		return;
	}

	// Dump blocks, highest address first.
	uint8_t byteCount;
	uint8_t buffer[18];
	uint8_t blockAddr;
	isSectorTrailer = true;
	for (int8_t blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) {
		//blockAddr = firstBlock + blockOffset;
		// Sector number - only on first line
		if (isSectorTrailer) {
			if(sector < 10)
				PRINTF("   "); // Pad with spaces
			else
				PRINTF("  "); // Pad with spaces
			PRINTF("%02X",sector);
			PRINTF("   ");
		}
		else {
			PRINTF("       ");
		}
		// Block number
		if(blockAddr < 10)
			PRINTF("   "); // Pad with spaces
		else {
			if(blockAddr < 100)
				PRINTF("  "); // Pad with spaces
			else
				PRINTF(" "); // Pad with spaces
		}
		PRINTF("%02X",blockAddr);
		PRINTF("  ");
		// Establish encrypted communications before reading the first block
		if (isSectorTrailer) {
			status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
			if (status != STATUS_OK) {
				PRINTF("\r\nPCD_Authenticate() failed: ");
				switch (status) {
						case STATUS_OK:				PRINTF("Success.\n"); break;
						case STATUS_ERROR:			PRINTF("Error in communication.\n"); break;
						case STATUS_COLLISION:		PRINTF("Collission detected.\n"); break;
						case STATUS_TIMEOUT:		PRINTF("Timeout in communication.\n"); break;
						case STATUS_NO_ROOM:		PRINTF("A buffer is not big enough.\n"); break;
						case STATUS_INTERNAL_ERROR:	PRINTF("Internal error in the code. Should not happen.\n"); break;
						case STATUS_INVALID:		PRINTF("Invalid argument.\n"); break;
						case STATUS_CRC_WRONG:		PRINTF("The CRC_A does not match.\n"); break;
						case STATUS_MIFARE_NACK:	PRINTF("A MIFARE PICC responded with NAK.\n"); break;
						default:					PRINTF("Unknown error\n"); break;
					}
				//PRINTF(GetStatusCodeName(status));
				//return;
			}
		}
		// Read block
		byteCount = sizeof(buffer);
		status = MIFARE_Read(blockAddr, buffer, &byteCount);
		if (status != STATUS_OK) {
			PRINTF("\r\nMIFARE_Read() failed: ");
			switch (status) {
							case STATUS_OK:				PRINTF("Success.\n"); break;
							case STATUS_ERROR:			PRINTF("Error in communication.\n"); break;
							case STATUS_COLLISION:		PRINTF("Collission detected.\n"); break;
							case STATUS_TIMEOUT:		PRINTF("Timeout in communication.\n"); break;
							case STATUS_NO_ROOM:		PRINTF("A buffer is not big enough.\n"); break;
							case STATUS_INTERNAL_ERROR:	PRINTF("Internal error in the code. Should not happen.\n"); break;
							case STATUS_INVALID:		PRINTF("Invalid argument.\n"); break;
							case STATUS_CRC_WRONG:		PRINTF("The CRC_A does not match.\n"); break;
							case STATUS_MIFARE_NACK:	PRINTF("A MIFARE PICC responded with NAK.\n"); break;
							default:					PRINTF("Unknown error\n"); break;
							}
			//PRINTF(GetStatusCodeName(status);
			continue;
		}
		// Dump data
		for (uint8_t index = 0; index < 16; index++) {
			if(buffer[index] < 0x10)
				PRINTF(" 0");
			else
				PRINTF(" ");
			PRINTF("%02X",buffer[index]);
			if ((index % 4) == 3) {
				PRINTF(" ");
			}
		}
		// Parse sector trailer data
		if (isSectorTrailer) {
			c1  = buffer[7] >> 4;
			c2  = buffer[8] & 0xF;
			c3  = buffer[8] >> 4;
			c1_ = buffer[6] & 0xF;
			c2_ = buffer[6] >> 4;
			c3_ = buffer[7] & 0xF;
			invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
			g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
			g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
			g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
			g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
			isSectorTrailer = false;
		}

		// Which access group is this block in?
		if (no_of_blocks == 4) {
			group = blockOffset;
			firstInGroup = true;
		}
		else {
			group = blockOffset / 5;
			firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
		}

		if (firstInGroup) {
			// Print access bits
			PRINTF(" [ ");
			PRINTF("%d",(g[group] >> 2) & 1); PRINTF(" ");
			PRINTF("%d",(g[group] >> 1) & 1); PRINTF(" ");
			PRINTF("%d",(g[group] >> 0) & 1);
			PRINTF(" ] ");
			if (invertedError) {
				PRINTF(" Inverted access bits did not match! ");
			}
		}

		if (group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
		//	long value = (long)(buffer[3])<<24) | (long)(buffer[2])<<16) | (long)(buffer[1])<<8) | (long)(buffer[0]);
			//PRINTF("Value=0x"); PRINTF("%02X",value);
			PRINTF("Adr=0x"); PRINTF("%02X",buffer[12]);
		}
		PRINTF("");
	}

	return;
} // End PICC_DumpMifareClassicSectorToSerial()

enum StatusCode PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
									uint8_t length,	///< In: The number of bytes to transfer.
									uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 ) {
	int i = 5000;
	uint8_t n = 0x00;
	PRINTF("\r\nHERE!!!!!!!!!!!!\n");
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_ReadRegister(CommandReg,0x00);

	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_ReadRegister(DivIrqReg,0x00);

	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_ReadRegister(FIFOLevelReg,0x00);

	PCD_WriteRegister2(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_ReadRegister(FIFODataReg,0x00);

	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
	PCD_ReadRegister(CommandReg,0x00);
	PRINTF("\r\ncalc crc\r");
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
	uint8_t backData[2];
	uint8_t backLen;
	uint8_t sendLen;
	uint8_t sendData;
	uint8_t command = PCD_Transceive;
	uint8_t controlBuffer[2];
	bool checkCRC = false;
	int j =0;
	unsigned int i,delay;
	if (bufferATQA == NULL || bufferSize < 2) // The ATQA response is 2 bytes long.
	{
		return false;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]


		uint8_t txLastBits = _validBits ? validBits : 0;
		uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

		PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
		PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
		PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
		PCD_WriteRegister(FIFODataReg, 0x26);	// Write sendData to the FIFO
		PCD_WriteRegister(BitFramingReg, 0x07);		// Bit adjustments
		PCD_WriteRegister(CommandReg, command);				// Execute the command

		if (command == PCD_Transceive) {
			PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
		}
		for(j =0; j<50000;j++){}
		i = 2000;
		while (1) {
			n = PCD_ReadRegister(ComIrqReg,0x00);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
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
		//return true;
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
			PCD_ReadRegister2(FIFODataReg, n, backData[0], rxAlign);	// Get received data from FIFO
			_validBits = PCD_ReadRegister(ControlReg,0x00) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
			if (validBits) {
				validBits = _validBits;
			}
		//}

		// Tell about collisions
		if (errorRegValue & 0x08) {		// CollErr
			return STATUS_COLLISION;
		}
		PRINTF("\nBackdata: %02X :: backlen: %02X ",backData[0],backLen);
		// Perform CRC_A validation if requested.
		//if (backData[0] && backLen && checkCRC) {
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

			enum StatusCode status = PCD_CalculateCRC(&backData[0], backLen -2, &controlBuffer[0]);
			if (status != STATUS_OK) {
				return status;
			}
			if ((backData[backLen - 2] != controlBuffer[0]) || (backData[backLen - 1] != controlBuffer[1])) {
				return STATUS_CRC_WRONG;
			}
		//}

	if (bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return false;
	}
	else
		return true;


}
enum StatusCode PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
											uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
											uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
											uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
											uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
											uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
											uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
											uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
											bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	uint8_t n, _validBits;
	unsigned int i;
	uint8_t send=*sendData;

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister2(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);				// Execute the command

	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}

	i = 2000;
	while (1) {
		n = PCD_ReadRegister(ComIrqReg,0x00);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
		if (--i == 0) {						// The emergency break. If all other conditions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = PCD_ReadRegister(ErrorReg,0x00); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		n = PCD_ReadRegister(FIFOLevelReg,0x00);			// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;										// Number of bytes returned
		PCD_ReadRegister2(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg,0x00) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		enum StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End PCD_CommunicateWithPICC()
void PCD_Init() {

	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25s.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}

void PCD_AntennaOn() {
	uint8_t value = PCD_ReadRegister(TxControlReg,0x00);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
	}
} // End PCD_AntennaOn()
enum StatusCode PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
		uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										 ) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	enum StatusCode result;
	uint8_t count;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte.
	uint8_t *responseBuffer;
	uint8_t responseLength;
	uint8_t collisionPos;
	uint8_t valueOfCollReg;
	uint8_t maxBytes;
	uint8_t bytesToCopy;
	PRINTF("\r\nSElECT\n!!!!!!!!!!!");
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}

	// Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;

			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;

			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;

			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		 bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			 maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[0]=0x93;
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				 valueOfCollReg = PCD_ReadRegister(CollReg,0x00); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				 collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits = collisionPos;
				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << count);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()
bool PICC_ReadCardSerial() {
	enum StatusCode result = PICC_Select(&uid,0x00);
	return (result == STATUS_OK);
} // End
enum StatusCode MIFARE_Read(	uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
								uint8_t *buffer,		///< The buffer to store the data in
								uint8_t *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
										) {
	enum StatusCode result;

	// Sanity check
	if (buffer == NULL || *bufferSize < 18) {
		return STATUS_NO_ROOM;
	}

	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, true);
} // End MIFARE_Read()

int main (void)
{
	MIFARE_Key Key;
	//Uid Uid;
    uint8_t loopCount = 0;
    uint32_t j;
    int8_t i;
    int8_t n;
    uint8_t sector = 1;
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

       // n = PCD_ReadRegister(VersionReg,0x00);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
       // PRINTF("\n**Version %02X \n\r",n);

        PRINTF("This code scan the MIFARE Classsic NUID.\n\r");
        //PRINTF("Using the following key:\n\r");
        //PRINTF("%d with %i \n\r",Key.keyBytes, MF_KEY_SIZE);
        while(1)
        {
//        for(j=0;j<MF_KEY_SIZE;j++)
//        	PRINTF("t: %02X\n\r",Key.keyBytes[j]);
        while(PICC_IsNewCardPresent() == false)
        {}
        while(PICC_ReadCardSerial() == false)
        {}
       // PICC_DumpMifareClassicSectorToSerial(&Uid,&Key,sector);

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

