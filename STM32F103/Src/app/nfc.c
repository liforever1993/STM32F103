/*
 * nfc.c
 *
 *  Created on: 2018年9月17日
 *      Author: lifor
 */

#include "nfc.h"
#include "spi.h"
#include "stm32f1xx_hal.h"

#define HEADER_LENGTH	(6)

#define DATA_WRITE      (0x01)
#define STATUS_READ     (0x02)
#define DATA_READ       (0x03)

#define PN532_PREAMBLE					(0x00)
#define PN532_STARTCODE1				(0x00)
#define PN532_STARTCODE2				(0xFF)
#define PN532_POSTAMBLE					(0x00)

#define PN532_HOSTTOPN532				(0xD4)
#define PN532_PN532TOHOST				(0xD5)

#define PN532_ACK_WAIT_TIME				(10)  // ms, timeout of waiting for ACK

#define PN532_INVALID_ACK				(-1)
#define PN532_TIMEOUT					(-2)
#define PN532_INVALID_FRAME				(-3)
#define PN532_NO_SPACE					(-4)

#define NOR_FRAME_HEADER_SIZE			(0x06)

#define REVERSE_BITS_ORDER(b)         b = (b & 0xF0) >> 4 | (b & 0x0F) << 4; \
		b = (b & 0xCC) >> 2 | (b & 0x33) << 2; \
		b = (b & 0xAA) >> 1 | (b & 0x55) << 1
/* 强制字节对齐为1字节对齐 */
#pragma pack(1)
/* Normal information frame */
typedef struct{
	uint8_t Preamble;		// Preamble
	uint8_t StartCode[2];	// Start of Packet Code(0x00 and 0xff)
	uint8_t LEN;			// Packet Length (TFI and PD0 to PDn)
	uint8_t LCS;			// Packet Length Checksum [LEN + LCS] = 0x00
	uint8_t	TFI;			// frame identifier  D4H host to PN532      D5H PN532 to host
	uint8_t DATA;
}NOR_HEADER;

/* Extended information frame */
typedef struct{
	uint8_t Preamble;
	uint8_t StartCode[2];
	uint8_t NormaPacketLength;
	uint8_t NormaPacketLengthCS;
	uint8_t PacketLength[2];
	uint8_t LCS;				// packe tLength CheckSum;
	uint8_t TFI;
	uint8_t DATA;
}EXT_HEADER;

typedef struct{
	uint8_t DCS;		// 【TFI + PD0 + PD1 +...PDn +DCS】= 0x00
	uint8_t Postamble;
}FRAME_LAST;

/* ACK frame */
typedef struct{
	uint8_t Preamble;
	uint8_t StartCode[2];
	uint8_t AckPacketCode[2];
	uint8_t Postamble;
}ACK_FRAME;
#pragma pack()

HAL_StatusTypeDef spi_write(uint8_t *pdata,uiny8_t length,uint8_t timeout);
HAL_StatusTypeDef spi_read(uint8_t *pdata,uiny8_t length,uint8_t timeout);
HAL_StatusTypeDef write_reg(uint8_t *reg_addr, uint8_t data);
uint8_t get_status();

uint8_t ErrorFrame[8] = {0x00, 0x00, 0xff, 0x01, 0xff, 0x7f, 0x81, 0x00};


uint8_t readResponse(uint8_t *buff, uint8_t len, uint16_t timeout)
{
	const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF};
	uint8_t HeaderLength = sizeof(PN532_ACK);
	uint8_t ackBuff[HeaderLength];
	uint8_t result = PN532_INVALID_FRAME;
	memset(ackBuff, 0, HeaderLength);
	nfc_weakUp();
	HAL_Delay(2);
	/* start to read ack frame */
	uint8_t buff[1] = {DATA_READ};
	spi_write(buff, 1, 10);
	/* read ack frame */
	spi_read(ackBuff, HeaderLength, 10);

	/* return cmp result */
	if(0 != memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK) - 2) )
	{
		result = PN532_INVALID_FRAME;
		goto ERROR_OOUT;
	}

	if(0 != (uint8_t)(ackBuf[3] + ackBuff[4]))
	{
		result = PN532_INVALID_FRAME;
		goto ERROR_OOUT;
	}
	uint8_t DataLength = ackBuf[3];
	uint8_t CheckSum = 0;
	while(DataLength > len)
	{
		memset(buff, 0, len);
		spi_read(buff, len, 10);
		DataLength -= len;
		for(int i = 0; i < len; i++)
		{
			CheckSum += buff[i];
		}
	}
	memset(buff, 0, len);
	spi_read(buff, len, DataLength);
	for(int i = 0; i < len; i++)
	{
		CheckSum += buff[i];
	}
	memset(buff, 0, len);
	spi_read(ackBUff, 2, 10);
	if(0 != (CheckSum + ackBuff[0]))
	{
		result = PN532_INVALID_FRAME;
		goto ERROR_OOUT;
	}
ERROR_OOUT:
	nfc_sleep();
	return result;
}

bool write_frame(void *Data,uint8_t length)
{
	nfc_wakeup();
	HAL_Delay(2);
	uint8_t data_position = 0;
	uint8_t *buff = (uint8_t *)malloc(length + NOR_FRAME_HEADER_SIZE + 2);
	memset(buff, 0, length + NOR_FRAME_HEADER_SIZE + 2);
	/* start to write frame */
	buff[0] = DATA_WRITE;
	spi_write(buff, 1, 10);
	/* set Packet header */
	NOR_HEADER *header = (NOR_HEADER *)buff;
	header->Preamble = PN532_PREAMBLE;
	header->StartCode[0] = PN532_STARTCODE1;
	header->StartCode[1] = PN532_STARTCODE2;
	header->LEN = length + 1;
	header->LCS = ~(header->LEN) + 1;
	header->TFI = PN532_HOSTTOPN532;
	uint8_t CheckSum = header->TFI;
	/* set frame data */
	for(data_position = 0; data_position < length; data_position++)
	{
		buff[NOR_FRAME_HEADER_SIZE + data_position] = (uint8_t)Data[data_position];
		CheckSum += buff[NOR_FRAME_HEADER_SIZE + data_position];
	}
	/* Packet Data Checksum(DCS) */
	buff[data_position] =  ~CheckSum + 1;
	/* Post amble */
	buff[data_position + 1] = PN532_POSTAMBLE;
	/* Spi write frame */
	spi_write(buff, length + NOR_FRAME_HEADER_SIZE + 2, 10);
	/* free memory */
	free(buff);
	/* set nfc to sleep */
	nfc_sleep();
}

bool readAckFrame()
{
	const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};
	uint8_t ackBuff[sizeof(PN532_ACK)];
	memset(ackBuff, 0, sizeof(PN532_ACK));
	nfc_weakUp();
	HAL_Delay(2);
	/* start to read ack frame */
	uint8_t buff[1] = {DATA_READ};
	spi_write(buff,1,5);
	/* read ack frame */
	spi_read(ackBuff,sizeof(PN532_ACK),10);
	nfc_sleep();
	/* return cmp result */
	return memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK));
}

/* Send Message Data By Spi */
HAL_StatusTypeDef spi_write(uint8_t *pdata,uiny8_t length,uint8_t timeout)
{
	return HAL_SPI_Transmit(&hspi1, pData, length, timeout);
}

/* Recv data with spi */
HAL_StatusTypeDef spi_read(uint8_t *pdata,uiny8_t length,uint8_t timeout)
{
	return HAL_SPI_Receive(&hspi1, pData, length, timeout);
}

/* write one data to reg */
HAL_StatusTypeDef write_reg(uint8_t *reg_addr, uint8_t data)
{
	// reg_addr is 2bytes
	uint8_t buff[5];
	memset(buff,0,5);
	buff[0] = PN532_HOSTTOPN532;
	buff[1] = PN532_COMMAND_WRITEREGISTER;
	buff[2] = reg_addr[1];
	buff[3] = reg_addr[0];
	buff[4] = data;
	write_frame(buff, 5);
	if(readAckFrame())
	{
		// invalid ack
	}
}

uint8_t get_status()
{
	uint8_t status;
	uint8_t sendbuff[1] = {STATUS_READ};
	HAL_SPI_TransmitReceive(&hspi1, sendbuff, &status, 1, 100);
	return status;
}


void nfc_weakUp(void)
{
	HAL_GPIO_WritePin(GPIOA, NFC_SPI_NSS_Pin, GPIO_PIN_RESET);
}

void nfc_sleep(void)
{
	HAL_GPIO_WritePin(GPIOA, NFC_SPI_NSS_Pin, GPIO_PIN_SET);
}

uint32_t getFirmwareVersion()
{
	uint32_t response;
	uint8_t SendBuff[32];
	memset(SendBuff, 0, 32);

}

void nfc_init()
{
	nfc_weakUp();

}

void nfc_task()
{
	nfc_init();
	while(1)
	{


	}
}
