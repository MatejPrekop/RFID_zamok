/*
 * RFID.c
 *
 *  Created on: 7. 12. 2016
 *      Author: Mallto
 */


#include "RFID.h"
#include <stdio.h>
#include <stm32l1xx_spi.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>

/* SDA => PB 12
 * SCK => PB 13
 * MOSI => PB 15
 * MISO => PB14
 * IRQ => UNUSSED
 * GND => GND
 * RST => +3.3V
 * 3.3V => +3.3V
 */

//-------------------------------------------------
void TM_MFRC522_Init(void) {

	TM_MFRC522_InitSPI();
	//CS high
	MFRC522_CS_HIGH;

	TM_MFRC522_Reset();

	TM_MFRC522_WriteRegister(MFRC522_REG_T_MODE, 0x8D);
	TM_MFRC522_WriteRegister(MFRC522_REG_T_PRESCALER, 0x3E);
	TM_MFRC522_WriteRegister(MFRC522_REG_T_RELOAD_L, 30);
	TM_MFRC522_WriteRegister(MFRC522_REG_T_RELOAD_H, 0);

	/* 48dB gain */
	//TM_MFRC522_WriteRegister(MFRC522_REG_RF_CFG, 0x70);
	TM_MFRC522_WriteRegister(MFRC522_REG_TX_AUTO, 0x40);
	TM_MFRC522_WriteRegister(MFRC522_REG_MODE, 0x3D);

	TM_MFRC522_AntennaOn();		//Open the antenna
}

//SPI2
void TM_MFRC522_InitSPI(void) {
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable SPI2 and GPIOB clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	//
	//CS  ---- Output, duoc dieu khien khi read/write
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//SET CS HIGH LEVEL
	GPIO_SetBits(GPIOB, GPIO_Pin_12);

	/*Configure SPI2 pins: SCK, MISO and MOSI*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	/* SPI2 configuration */
	/* Data bytes on both MOSI and MISO lines are sent with the MSB first. Data on both MOSI
	 and MISO lines must be stable on the rising edge of the clock and can be changed on the
	 falling edge. Data is provided by the MFRC522 on the falling clock edge and is stable
	 during the rising clock edge. */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 72Mhz / 8 = 9Mhz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	/* Enable SPI2  */
	SPI_Cmd(SPI2, ENABLE);
}
//-------------------------------------------------
//SPI2
unsigned char SPI2_ReadWrite(unsigned char writedat) {
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;

	/* Send byte through the SPI2 peripheral */
	SPI_I2S_SendData(SPI2, writedat);

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI2);
}

//-------------------------------------------------
void TM_MFRC522_WriteRegister(uint8_t addr, uint8_t val) {
	//CS low
	unsigned char status;
	MFRC522_CS_LOW;
	//Send address
	Delay_us(10);
	status = SPI2_ReadWrite((addr << 1) & 0x7E);
	//Send data
	Delay_us(10);
	status = SPI2_ReadWrite(val);
	//CS high
	MFRC522_CS_HIGH;
}
//-------------------------------------------------
uint8_t TM_MFRC522_ReadRegister(uint8_t addr) {
	uint8_t val;
	//CS low
	MFRC522_CS_LOW;
	Delay_us(10);
	SPI2_ReadWrite(((addr << 1) & 0x7E) | 0x80);
	Delay_us(10);
	val = SPI2_ReadWrite(0x00); // SPI2_ReadWrite(0x00);
	//CS high
	MFRC522_CS_HIGH;

	return val;
}
//-------------------------------------------------
void TM_MFRC522_SetBitMask(uint8_t reg, uint8_t mask) {
	TM_MFRC522_WriteRegister(reg, TM_MFRC522_ReadRegister(reg) | mask);
}
//-------------------------------------------------
void TM_MFRC522_ClearBitMask(uint8_t reg, uint8_t mask) {
	TM_MFRC522_WriteRegister(reg, TM_MFRC522_ReadRegister(reg) & (~mask));
}
//-------------------------------------------------
void TM_MFRC522_AntennaOn(void) {
	uint8_t temp;

	temp = TM_MFRC522_ReadRegister(MFRC522_REG_TX_CONTROL);
	if (!(temp & 0x03)) {
		TM_MFRC522_SetBitMask(MFRC522_REG_TX_CONTROL, 0x03);
	}
}
//-------------------------------------------------
void TM_MFRC522_AntennaOff(void) {
	TM_MFRC522_ClearBitMask(MFRC522_REG_TX_CONTROL, 0x03);
}
//-------------------------------------------------
void TM_MFRC522_Reset(void) {
	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_RESETPHASE);
}
//-------------------------------------------------
uint8_t getFirmwareVersion() {
	uint8_t response;
	response = TM_MFRC522_ReadRegister(MFRC522_REG_VERSION);
	return response;
}
//-------------------------------------------------
TM_MFRC522_Status_t TM_MFRC522_Request(uint8_t reqMode, uint8_t* TagType) {
	TM_MFRC522_Status_t status;
	uint16_t backBits;			//The received data bits

	TM_MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x07);//TxLastBists = BitFramingReg[2..0]	???

	TagType[0] = reqMode;
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

	if ((status != MI_OK) || (backBits != 0x10)) {
		status = MI_ERR;
	}

	return status;
}

//-------------------------------------------------
TM_MFRC522_Status_t TM_MFRC522_Check(uint8_t* id) {
	TM_MFRC522_Status_t status;
	//Find cards, return card type
	status = TM_MFRC522_Request(PICC_REQIDL, id);
	if (status == MI_OK) {
		//Card detected
		//Anti-collision, return card serial number 4 bytes
		status = TM_MFRC522_Anticoll(id);
	}
	TM_MFRC522_Halt();			//Command card into hibernation

	return status;
}
//-------------------------------------------------
TM_MFRC522_Status_t TM_MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID) {
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (CardID[i] != CompareID[i]) {
			return MI_ERR;
		}
	}
	return MI_OK;
}
//-------------------------------------------------
TM_MFRC522_Status_t TM_MFRC522_ToCard(uint8_t command, uint8_t* sendData,
		uint8_t sendLen, uint8_t* backData, uint16_t* backLen) {
	TM_MFRC522_Status_t status = MI_ERR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint16_t i;

	switch (command) {
	case PCD_AUTHENT: {
		irqEn = 0x12;
		waitIRq = 0x10;
		break;
	}
	case PCD_TRANSCEIVE: {
		irqEn = 0x77;
		waitIRq = 0x30;
		break;
	}
	default:
		break;
	}

	TM_MFRC522_WriteRegister(MFRC522_REG_COMM_IE_N, irqEn | 0x80);
	TM_MFRC522_ClearBitMask(MFRC522_REG_COMM_IRQ, 0x80);
	TM_MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);

	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_IDLE);

	//Writing data to the FIFO
	for (i = 0; i < sendLen; i++) {
		TM_MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, sendData[i]);
	}

	//Execute the command
	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, command);
	if (command == PCD_TRANSCEIVE) {
		TM_MFRC522_SetBitMask(MFRC522_REG_BIT_FRAMING, 0x80);//StartSend=1,transmission of data starts
	}

	//Waiting to receive data to complete
	i = 2000;//i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
	do {
		//CommIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = TM_MFRC522_ReadRegister(MFRC522_REG_COMM_IRQ);
		i--;
	} while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

	TM_MFRC522_ClearBitMask(MFRC522_REG_BIT_FRAMING, 0x80);		//StartSend=0

	if (i != 0) {
		if (!(TM_MFRC522_ReadRegister(MFRC522_REG_ERROR) & 0x1B)) {
			status = MI_OK;
			if (n & irqEn & 0x01) {
				status = MI_NOTAGERR;
			}

			if (command == PCD_TRANSCEIVE) {
				n = TM_MFRC522_ReadRegister(MFRC522_REG_FIFO_LEVEL);
				lastBits = TM_MFRC522_ReadRegister(MFRC522_REG_CONTROL) & 0x07;
				if (lastBits) {
					*backLen = (n - 1) * 8 + lastBits;
				} else {
					*backLen = n * 8;
				}

				if (n == 0) {
					n = 1;
				}
				if (n > MFRC522_MAX_LEN) {
					n = MFRC522_MAX_LEN;
				}

				//Reading the received data in FIFO
				for (i = 0; i < n; i++) {
					backData[i] = TM_MFRC522_ReadRegister(
					MFRC522_REG_FIFO_DATA);
				}
			}
		} else {
			status = MI_ERR;
		}
	}

	return status;
}
//-------------------------------------------------
TM_MFRC522_Status_t TM_MFRC522_Anticoll(uint8_t* serNum) {
	TM_MFRC522_Status_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;

	TM_MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x00);//TxLastBists = BitFramingReg[2..0]

	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

	if (status == MI_OK) {
		//Check card serial number
		for (i = 0; i < 4; i++) {
			serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i]) {
			status = MI_ERR;
		}
	}
	return status;
}

void TM_MFRC522_CalculateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData) {
	uint8_t i, n;

	TM_MFRC522_ClearBitMask(MFRC522_REG_DIV_IRQ, 0x04);			//CRCIrq = 0
	TM_MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);//Clear the FIFO pointer
	//Write_MFRC522(CommandReg, PCD_IDLE);

	//Writing data to the FIFO
	for (i = 0; i < len; i++) {
		TM_MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, *(pIndata + i));
	}
	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_CALCCRC);

	//Wait CRC calculation is complete
	i = 0xFF;
	do {
		n = TM_MFRC522_ReadRegister(MFRC522_REG_DIV_IRQ);
		i--;
	} while ((i != 0) && !(n & 0x04));			//CRCIrq = 1

	//Read CRC calculation result
	pOutData[0] = TM_MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_L);
	pOutData[1] = TM_MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_M);
}
//-------------------------------------------------
uint8_t TM_MFRC522_SelectTag(uint8_t* serNum) {
	uint8_t i;
	TM_MFRC522_Status_t status;
	uint8_t size;
	uint16_t recvBits;
	uint8_t buffer[9];

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for (i = 0; i < 5; i++) {
		buffer[i + 2] = *(serNum + i);
	}
	TM_MFRC522_CalculateCRC(buffer, 7, &buffer[7]);		//??
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

	if ((status == MI_OK) && (recvBits == 0x18)) {
		size = buffer[0];
	} else {
		size = 0;
	}

	return size;
}
//-------------------------------------------------
TM_MFRC522_Status_t TM_MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr,
		uint8_t* Sectorkey, uint8_t* serNum) {
	TM_MFRC522_Status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[12];

	//Verify the command block address + sector + password + card serial number
	buff[0] = authMode;
	buff[1] = BlockAddr;
	for (i = 0; i < 6; i++) {
		buff[i + 2] = *(Sectorkey + i);
	}
	for (i = 0; i < 4; i++) {
		buff[i + 8] = *(serNum + i);
	}
	status = TM_MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

	if ((status != MI_OK)
			|| (!(TM_MFRC522_ReadRegister(MFRC522_REG_STATUS2) & 0x08))) {
		status = MI_ERR;
	}

	return status;
}
//-------------------------------------------------
TM_MFRC522_Status_t TM_MFRC522_Read(uint8_t blockAddr, uint8_t* recvData) {
	TM_MFRC522_Status_t status;
	uint16_t unLen;

	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;
	TM_MFRC522_CalculateCRC(recvData, 2, &recvData[2]);
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

	if ((status != MI_OK) || (unLen != 0x90)) {
		status = MI_ERR;
	}

	return status;
}
//-------------------------------------------------
TM_MFRC522_Status_t TM_MFRC522_Write(uint8_t blockAddr, uint8_t* writeData) {
	TM_MFRC522_Status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[18];

	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	TM_MFRC522_CalculateCRC(buff, 2, &buff[2]);
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

	if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
		status = MI_ERR;
	}

	if (status == MI_OK) {
		//Data to the FIFO write 16Byte
		for (i = 0; i < 16; i++) {
			buff[i] = *(writeData + i);
		}
		TM_MFRC522_CalculateCRC(buff, 16, &buff[16]);
		status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != MI_OK) || (recvBits != 4)
				|| ((buff[0] & 0x0F) != 0x0A)) {
			status = MI_ERR;
		}
	}

	return status;
}

void TM_MFRC522_Halt(void) {
	uint16_t unLen;
	uint8_t buff[4];

	buff[0] = PICC_HALT;
	buff[1] = 0;
	TM_MFRC522_CalculateCRC(buff, 2, &buff[2]);

	TM_MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}


void zapisanie(void) {
	char bufferRFID[30];
	char bufferDisplay[10];
	unsigned char MyID[5] = { 0x1a, 0x18, 0x3a, 0x45, 0x7d };
	unsigned char CardID[5];
	unsigned char newCardID[5];
	int pom = 0;
	for (int x = 0; x <= 5; x++) {
		if (pom == 0) {
			if (TM_MFRC522_Check(CardID) == MI_OK) {

				sprintf(bufferRFID, "[%x-%x-%x-%x-%x]", CardID[0], CardID[1],
						CardID[2], CardID[3], CardID[4]);

				if (karta_v_zozname(CardID) == MI_OK) {

					Send_string_uart("Prilozte novu kartu\n\r");

					lcdClearDisplay(decodeRgbValue(255, 255, 255));
					sprintf(bufferDisplay, "Prilozte novu kartu");
					lcdPutS(bufferDisplay, 10, 50, decodeRgbValue(0, 0, 255),
							decodeRgbValue(255, 255, 255));

					Delay(3);

					for (int i = 0; i <= 10; i++) {
						if (TM_MFRC522_Check(newCardID) == MI_OK) {
							//if (karta_v_zozname(CardID) != MI_OK) {
							if (karta_v_zozname(newCardID) == MI_ERR) {
								pridaj_kartu(newCardID);
								Send_string_uart("Uspesne zapisanie\n\r");
								lcdClearDisplay(decodeRgbValue(255, 255, 255));
								sprintf(bufferDisplay, "Uspesne zapisanie");
								//odober_kartu(newCardID);
								lcdPutS(bufferDisplay, 10, 50,
										decodeRgbValue(0, 0, 255),
										decodeRgbValue(255, 255, 255));

								//vykreslenie OK
								ok();
								//blikanie zltej LED
								for (int c = 0; c < 6; c++) {
									GPIO_ToggleBits(GPIOA, GPIO_Pin_4);
									Delay_us(200000);
								}
								Delay(1);
								pom = 1;
								//i = 10;
								//Delay(3);
								welcome();
								break;
							} else {
								lcdClearDisplay(decodeRgbValue(255, 255, 255));
								sprintf(bufferDisplay,
										"Karta uz je v      zozname");
								lcdPutS(bufferDisplay, 10, 50,
										decodeRgbValue(255, 0, 0),
										decodeRgbValue(255, 255, 255));

								//vykreslenie Vykricnika
								vykricnik();
								welcome();
								goto koniec;
							}
						} else {
							Delay(1);
						}
						if (i == 10) {
							x = 5;
							welcome();
						}
					}

				} else {
					Send_string_uart("Neopravnena karta\n\r");
					x = 5;
					lcdClearDisplay(decodeRgbValue(255, 255, 255));
					sprintf(bufferDisplay, "Neopravnena karta");
					lcdPutS(bufferDisplay, 10, 50, decodeRgbValue(255, 0, 0),
							decodeRgbValue(255, 255, 255));

					//vykreslenie Vykricnika
					vykricnik();
					welcome();
				}

			} else {
				Send_string_uart("Priloz opravnenu kartu\n\r");
				lcdClearDisplay(decodeRgbValue(255, 255, 255));
				sprintf(bufferDisplay, "Priloz opravnenu   kartu");
				lcdPutS(bufferDisplay, 10, 50, decodeRgbValue(0, 0, 255),
						decodeRgbValue(255, 255, 255));
				Delay(1);
				if (x == 5) {
					welcome();
				}
			}
			//Delay(1);
		} else {
			//welcome();
			break;
		}
	}
	koniec:;
}

void vymazanie(void) {
	char bufferRFID[30];
	char bufferDisplay[10];
	unsigned char MyID[5] = { 0x1a, 0x18, 0x3a, 0x45, 0x7d };
	unsigned char CardID[5];
	unsigned char newCardID[5];
	int pom = 0;
	int pom2=0;
	for (int x = 0; x <= 5; x++) {
		if (pom == 0) {
			if (TM_MFRC522_Check(CardID) == MI_OK) {

				sprintf(bufferRFID, "[%x-%x-%x-%x-%x]", CardID[0], CardID[1],
						CardID[2], CardID[3], CardID[4]);

				if (karta_v_zozname(CardID) == MI_OK) {

					Send_string_uart("Prilozte kartu na odstranenie\n\r");

					lcdClearDisplay(decodeRgbValue(255, 255, 255));
					sprintf(bufferDisplay, "Prilozte kartu na  odstranenie");
					lcdPutS(bufferDisplay, 10, 50, decodeRgbValue(0, 0, 255),
							decodeRgbValue(255, 255, 255));

					Delay(3);

					for (int i = 0; i <= 10; i++) {
						if (TM_MFRC522_Check(newCardID) == MI_OK) {
							for(int j=0;j<5;j++)
							{
								if(CardID[j]==newCardID[j]) {
									pom2++;
								}
							}

							if (pom2!=5) {
								odober_kartu(newCardID);
								Send_string_uart("Uspesne vymazane\n\r");
								lcdClearDisplay(decodeRgbValue(255, 255, 255));
								sprintf(bufferDisplay, "Uspesne vymazane");
								//odober_kartu(newCardID);
								lcdPutS(bufferDisplay, 10, 50,
										decodeRgbValue(0, 0, 255),
										decodeRgbValue(255, 255, 255));

								//vykreslenie OK
								ok();
								//blikanie zltej LED
								for (int c = 0; c < 6; c++) {
									GPIO_ToggleBits(GPIOA, GPIO_Pin_4);
									Delay_us(200000);
								}
								Delay(1);
								pom = 1;
								//i = 10;
								//Delay(3);
								welcome();
								break;
							} else {
								lcdClearDisplay(decodeRgbValue(255, 255, 255));
								sprintf(bufferDisplay,
										"Nemozno vymazat tu istu kartu");
								lcdPutS(bufferDisplay, 10, 50,
										decodeRgbValue(255, 0, 0),
										decodeRgbValue(255, 255, 255));

								//vykreslenie Vykricnika
								vykricnik();
								welcome();
								goto koniec;
							}

						} else {
							Delay(1);
						}
						if (i == 10) {
							x = 5;
							welcome();
						}

					}

				} else {
					Send_string_uart("Neopravnena karta\n\r");
					x = 5;
					lcdClearDisplay(decodeRgbValue(255, 255, 255));
					sprintf(bufferDisplay, "Neopravnena karta");
					lcdPutS(bufferDisplay, 10, 50, decodeRgbValue(255, 0, 0),
							decodeRgbValue(255, 255, 255));

					//vykreslenie Vykricnika
					vykricnik();
					welcome();
				}

			} else {
				Send_string_uart("Priloz opravnenu kartu\n\r");
				lcdClearDisplay(decodeRgbValue(255, 255, 255));
				sprintf(bufferDisplay, "Priloz opravnenu   kartu");
				lcdPutS(bufferDisplay, 10, 50, decodeRgbValue(0, 0, 255),
						decodeRgbValue(255, 255, 255));
				Delay(1);
				if (x == 5) {
					welcome();
				}
			}
			//Delay(1);
		} else {
			//welcome();
			break;
		}
	}
	koniec:;
}

//funkcia na prejdenie zoznamu zapisanych kariet na flash pameti na adresach
//0x08080000 az 0x08080100
TM_MFRC522_Status_t karta_v_zozname(uint8_t* CardID) {
	uint8_t i;
	uint8_t j;
	uint8_t pom;
	uint32_t *addres;
	uint8_t CompareID;
	for (j = 0; j < 8; j++) {
		pom = 0;
		addres = (uint32_t *) 0x08080000 + (0x05) + (j * 0x8);
		CompareID = *addres;
		if (1 == CompareID) {
			for (i = 0; i < 5; i++) {
				addres = (uint32_t *) 0x08080000 + 0x1 * i + j * 0x8;
				CompareID = *addres;
				if (CardID[i] == CompareID) {
					pom++;
				}

			}
			if (pom == 5) {
				return MI_OK;
			}
		}
	}
	return MI_ERR;
}

//funkcia na zapis do zoznamu kariet na adresach
//0x08080000 az 0x08080100

TM_MFRC522_Status_t pridaj_kartu(uint8_t* CardID) {
	uint8_t i;
	uint8_t j;
	uint32_t *addres;
	uint32_t PutInIDAddress;
	uint8_t PutInID;
	for (j = 0; j < 8; j++) {
		addres = (uint32_t *) 0x08080000 + (0x05) + (j * 0x8);
		PutInID = *addres;
		PutInIDAddress = (uint32_t) (addres);
		if (0 == PutInID) {
			FLASH_Unlock();
			DATA_EEPROM_ProgramByte(PutInIDAddress, 1);
			for (i = 0; i < 5; i++) {
				addres = (uint32_t *) 0x08080000 + 0x1 * i + j * 0x8;
				PutInIDAddress = (uint32_t) (addres);
				DATA_EEPROM_ProgramByte(PutInIDAddress, CardID[i]);
			}
			FLASH_Lock();
			return MI_OK;
		}
	}
	return MI_ERR;
}

TM_MFRC522_Status_t odober_kartu(uint8_t* CardID) {
	uint8_t i;
	uint8_t k;
	uint8_t j;
	uint8_t pom;
	uint32_t *addres;
	uint8_t RemoveID;
	uint32_t RemoveIDAddress;
	for (j = 0; j < 8; j++) {
		pom = 0;
		addres = (uint32_t *) 0x08080000 + (0x05) + (j * 0x8);
		RemoveID = *addres;
		if (1 == RemoveID) {
			for (i = 0; i < 5; i++) {
				addres = (uint32_t *) 0x08080000 + 0x1 * i + j * 0x8;
				RemoveID = *addres;
				if (CardID[i] == RemoveID) {
					pom++;
				}

			}
			if (pom == 5) {
				FLASH_Unlock();
				for (k = 0; k < 8; k++) {
					addres = (uint32_t *) 0x08080000 + 0x1 * k + j * 0x8;
					RemoveIDAddress = (uint32_t) (addres);

					DATA_EEPROM_EraseByte(RemoveIDAddress);
				}
				FLASH_Lock();
				return MI_OK;
			}
		}
	}
	return MI_ERR;
}

/*
// funkcia na zapis poslednych 5 pristupov na adresach
//0x08080500 az 0x080805A0
void zapis_pristupu(uint8_t* CardID) {
	uint8_t j;
	uint8_t k;
	uint8_t i;
	uint32_t TempIDAddres;
	uint8_t TempID;
	uint32_t *addres;
	FLASH_Unlock();
	for (i = 4; i > 0; i--) {
		for (j = 0; j < 5; j++) {
			addres = (uint32_t *) (0x08080500 + (0x4 * j)
					+ ((i - 1) * 0x8 * 0x4));
			TempID = *addres;
			addres = (uint32_t *) (0x08080500 + (0x4 * j) + ((i) * 0x8 * 0x4));
			TempIDAddres = (uint32_t) (addres);
			DATA_EEPROM_ProgramByte(TempIDAddres, TempID);
		}
	}
	for (k = 0; k < 5; k++) {
		addres = (uint32_t *) 0x08080500 + 0x1 * k;
		TempIDAddres = (uint32_t) (addres);
		DATA_EEPROM_ProgramByte(TempIDAddres, CardID[k]);
	}
	FLASH_Lock();
}

void vypis_pristupov(void) {
	uint8_t i;
	uint8_t j;
	uint8_t PrintID[5];
	uint32_t *addres;
	char buffer[30];
	lcdClearDisplay(decodeRgbValue(255, 255, 255));
	sprintf(buffer, "Zoznam poslednych  pristupov");
	lcdPutS(buffer, 10, 40, decodeRgbValue(0, 0, 255),
			decodeRgbValue(255, 255, 255));
	for (i = 0; i < 5; i++) {
		for (j = 0; j < 5; j++) {
			addres = (uint32_t *) 0x08080500 + 0x1 * j + i * 0x8;
			PrintID[j] = *addres;
		}
		sprintf(buffer, "%x-%x-%x-%x-%x", PrintID[0], PrintID[1], PrintID[2],
				PrintID[3], PrintID[4]);
		lcdPutS(buffer, 10, 58 + 9 * i, decodeRgbValue(0, 0, 255),
				decodeRgbValue(255, 255, 255));

	}
	while (((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)) + 1) % 2 != 0) {
		Delay_us(100);
	}

	welcome();
}*/

///////////////////////////////////////////////////////////
// funkcia na zapis poslednych 5 pristupov na adresach
//0x08080500 az 0x080805A0
void zapis_pristupu(uint8_t* CardID) {
	uint8_t j;
	uint8_t k;
	uint8_t i;
	uint32_t TempIDAddres;
	uint8_t TempID;
	uint32_t *addres;
	FLASH_Unlock();
	for (i = 24; i > 0; i--) {
		for (j = 0; j < 5; j++) {
			addres = (uint32_t *) (0x08080500 + (0x4 * j)
					+ ((i - 1) * 0x8 * 0x4));
			TempID = *addres;
			addres = (uint32_t *) (0x08080500 + (0x4 * j) + ((i) * 0x8 * 0x4));
			TempIDAddres = (uint32_t) (addres);
			DATA_EEPROM_ProgramByte(TempIDAddres, TempID);
		}
	}
	for (k = 0; k < 5; k++) {
		addres = (uint32_t *) 0x08080500 + 0x1 * k;
		TempIDAddres = (uint32_t) (addres);
		DATA_EEPROM_ProgramByte(TempIDAddres, CardID[k]);
	}
	FLASH_Lock();
}

void vypis_pristupov(void) {

	int pom;
	uint8_t i;
	uint8_t j;
	uint8_t PrintID[5];
	uint32_t *addres;
	char buffer[30];

	zaciatok:	pom=0;
	lcdClearDisplay(decodeRgbValue(255, 255, 255));
	sprintf(buffer, "Zoznam poslednych  pristupov %d /5",pom+1);
	lcdPutS(buffer, 10, 40, decodeRgbValue(0, 0, 255),
			decodeRgbValue(255, 255, 255));
	for (i = 0; i < 5; i++) {
		for (j = 0; j < 5; j++) {
			addres = (uint32_t *) 0x08080500 + 0x1 * j + i * 0x8;
			PrintID[j] = *addres;
		}
		sprintf(buffer, "%x-%x-%x-%x-%x", PrintID[0], PrintID[1], PrintID[2],
				PrintID[3], PrintID[4]);
		lcdPutS(buffer, 10, 67 + 9 * i, decodeRgbValue(0, 0, 255),
				decodeRgbValue(255, 255, 255));

	}
	Delay_us(1500000);
	while (1) {
		if (((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) + 1) % 2== 0) {
			pom++;
			if(pom==5)
						{

							goto zaciatok;
						}
			lcdClearDisplay(decodeRgbValue(255, 255, 255));
				sprintf(buffer, "Zoznam poslednych  pristupov %d/5",pom+1);
				lcdPutS(buffer, 10, 40, decodeRgbValue(0, 0, 255),
						decodeRgbValue(255, 255, 255));
			for (i = 0; i < 5; i++) {
				for (j = 0; j < 5; j++) {
					addres = (uint32_t *) 0x08080500 + 0x1 * j + i * 0x8
							+ (0x8 * 0x5 * pom);
					PrintID[j] = *addres;
				}
				sprintf(buffer, "%x-%x-%x-%x-%x", PrintID[0], PrintID[1],
						PrintID[2], PrintID[3], PrintID[4]);
				lcdPutS(buffer, 10, 67 + 9 * i, decodeRgbValue(0, 0, 255),
						decodeRgbValue(255, 255, 255));

			}
			Delay(1);

			}
			//((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)) + 1) % 2 != 0
		if(((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)) + 1) % 2 == 0)
		{
			goto koniec;
		}
		}
koniec:
	welcome();
}

void kontrola_zoznamu(void)
 {
	int pom = 0;
	uint8_t TempID;
	uint32_t *addres;
	unsigned char CardID[5];
	char bufferDisplay[76];
	for (uint8_t a = 0; a < 8; a++) {
		addres = (uint32_t *) (0x08080000 + (0x4 * a) + (0x5 * 0x4));
		TempID = *addres;
		//addres = (uint32_t *) 0x08080000 + (0x05) + (a * 0x8);
		//TempID = *addres;
		if (TempID == 1) {
			pom++;
		}

	}
	if (pom == 0) {
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		sprintf(bufferDisplay,"Ziadna karta v     zozname prilozte   kartu na pociatocnezapisanie");
		lcdClearDisplay(decodeRgbValue(255, 255, 255));
		lcdPutS(bufferDisplay, 10, 50, decodeRgbValue(0, 0, 255),
				decodeRgbValue(255, 255, 255));
	}
	while (pom == 0) {

		if (TM_MFRC522_Check(CardID) == MI_OK) {
			pridaj_kartu(CardID);
			lcdClearDisplay(decodeRgbValue(255, 255, 255));
			sprintf(bufferDisplay, "Uspesne zapisanie");
			//odober_kartu(newCardID);
			lcdPutS(bufferDisplay, 10, 50, decodeRgbValue(0, 0, 255),
					decodeRgbValue(255, 255, 255));

			//vykreslenie OK
			ok();
			for (int c = 0; c < 6; c++) {
				GPIO_ToggleBits(GPIOA, GPIO_Pin_4);
				Delay_us(200000);
			}
			Delay(1);

			pom++;
		} else {
			Delay_us(100000);
		}
	}
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}

