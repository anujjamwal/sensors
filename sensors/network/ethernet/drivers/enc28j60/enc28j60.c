/*
 * enc28j60.c
 *
 * Created: 10/13/2013 12:13:13 AM
 *  Author: anujjamwal
 */ 

#include <avr/io.h>
#include "enc28j60conf.h"
#include "enc28j60.h"
#include <util/delay.h>

#define cbi(port, bit) port &= ~(1<<bit) 
#define sbi(port, bit) port |= (1<<bit)
#define MIN(a,b) a<b?a:b
#define ENC28j60_ENABLE ENC28J60_SPI_PORT &= ~(1<<ENC28J60_SPI_SS);
#define ENC28J60_DISABLE ENC28J60_SPI_PORT |= (1<<ENC28J60_SPI_SS);

static uint8_t Enc28j60Bank;
static uint16_t NextPacketPtr;

__attribute__((always_inline)) static inline uint8_t spi_read()  {
	SPDR = 0xff;
	while ( !(SPSR & 0x80)) ;
	return SPDR;
}

__attribute__((always_inline)) static inline void spi_write(uint8_t data) {
	SPDR = data;
	while ( !(SPSR & 0x80)) ;
}

void spi_init() 
{
	// setup SPI I/O pins
        sbi(ENC28J60_SPI_PORT, ENC28J60_SPI_SCK);       // set SCK hi
        sbi(ENC28J60_SPI_DDR, ENC28J60_SPI_SCK);        // set SCK as output
        cbi(ENC28J60_SPI_DDR, ENC28J60_SPI_MISO);       // set MISO as input
        sbi(ENC28J60_SPI_DDR, ENC28J60_SPI_MOSI);       // set MOSI as output
        sbi(ENC28J60_SPI_DDR, ENC28J60_SPI_SS);         // SS must be output for Master mode to work
        // initialize SPI interface
        // master mode
        sbi(SPCR, MSTR);
        // select clock phase positive-going in middle of data
        cbi(SPCR, CPOL);
        // Data order MSB first
        cbi(SPCR,DORD);
        // switch to f/4 2X = f/2 bitrate
        cbi(SPCR, SPR0);
        cbi(SPCR, SPR1);
        sbi(SPSR, SPI2X);
        // enable SPI
        sbi(SPCR, SPE);
}

uint8_t enc28j60ReadOp(uint8_t op, uint8_t address)
{
	ENC28j60_ENABLE
	
	spi_write(op | (address & ADDR_MASK));
	uint8_t data = spi_read();
	
	ENC28J60_DISABLE

	return data;
}

void enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data)
{
	ENC28j60_ENABLE

	// issue write command
	spi_write(op | (address & ADDR_MASK));
	spi_write(data);

	// release CS
	ENC28J60_DISABLE
}

void enc28j60ReadBuffer(uint16_t len, uint8_t* data)
{
	ENC28j60_ENABLE
	
	// issue read command
	spi_write(ENC28J60_READ_BUF_MEM);
	
	while(len--)
	{
		*data++ = spi_read();
	}
	// release CS
	ENC28J60_DISABLE;
}

void enc28j60WriteBuffer(uint16_t len, uint8_t* data)
{
	ENC28j60_ENABLE
	
	spi_write(ENC28J60_WRITE_BUF_MEM);
	while(len--)
	{
		spi_write(*data++);
	}
	
	ENC28J60_DISABLE
}

void enc28j60SetBank(uint8_t address)
{
	// set the bank (if needed)
	if((address & BANK_MASK) != Enc28j60Bank)
	{
		// set the bank
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
		Enc28j60Bank = (address & BANK_MASK);
	}
}

uint8_t enc28j60Read(uint8_t address)
{
	// set the bank
	enc28j60SetBank(address);
	// do the read
	return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}

void enc28j60Write(uint8_t address, uint8_t data)
{
	// set the bank
	enc28j60SetBank(address);
	// do the write
	enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

uint16_t enc28j60PhyRead(uint8_t address)
{
	uint16_t data;

	// Set the right address and start the register read operation
	enc28j60Write(MIREGADR, address);
	enc28j60Write(MICMD, MICMD_MIIRD);

	// wait until the PHY read completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY);

	// quit reading
	enc28j60Write(MICMD, 0x00);
	
	// get data value
	data  = enc28j60Read(MIRDL);
	data |= enc28j60Read(MIRDH);
	// return the data
	return data;
}

void enc28j60PhyWrite(uint8_t address, uint16_t data)
{
	// set the PHY register address
	enc28j60Write(MIREGADR, address);
	
	// write the PHY data
	enc28j60Write(MIWRL, data);
	enc28j60Write(MIWRH, data>>8);

	// wait until the PHY write completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY);
}

void enc28j60Init(void)
{
        sbi(ENC28J60_SPI_PORT, ENC28J60_SPI_SS);
        
        spi_init();
        

        // perform system reset
        enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
        // check CLKRDY bit to see if reset is complete
        _delay_us(50);
        while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));

        // do bank 0 stuff
        // initialize receive buffer
        // 16-bit transfers, must write low byte first
        // set receive buffer start address
        NextPacketPtr = RXSTART_INIT;
        enc28j60Write(ERXSTL, RXSTART_INIT&0xFF);
        enc28j60Write(ERXSTH, RXSTART_INIT>>8);
        // set receive pointer address
        enc28j60Write(ERXRDPTL, RXSTART_INIT&0xFF);
        enc28j60Write(ERXRDPTH, RXSTART_INIT>>8);
        // set receive buffer end
        // ERXND defaults to 0x1FFF (end of ram)
        enc28j60Write(ERXNDL, RXSTOP_INIT&0xFF);
        enc28j60Write(ERXNDH, RXSTOP_INIT>>8);
        // set transmit buffer start
        // ETXST defaults to 0x0000 (beginnging of ram)
        enc28j60Write(ETXSTL, TXSTART_INIT&0xFF);
        enc28j60Write(ETXSTH, TXSTART_INIT>>8);

        // do bank 2 stuff
        // enable MAC receive
        enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
        // bring MAC out of reset
        enc28j60Write(MACON2, 0x00);
        // enable automatic padding and CRC operations
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
//      enc28j60Write(MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
        // set inter-frame gap (non-back-to-back)
        enc28j60Write(MAIPGL, 0x12);
        enc28j60Write(MAIPGH, 0x0C);
        // set inter-frame gap (back-to-back)
        enc28j60Write(MABBIPG, 0x12);
        // Set the maximum packet size which the controller will accept
        enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);      
        enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);

        // do bank 3 stuff
        // write MAC address
        // NOTE: MAC address in ENC28J60 is byte-backward
        enc28j60Write(MAADR5, ENC28J60_MAC0);
        enc28j60Write(MAADR4, ENC28J60_MAC1);
        enc28j60Write(MAADR3, ENC28J60_MAC2);
        enc28j60Write(MAADR2, ENC28J60_MAC3);
        enc28j60Write(MAADR1, ENC28J60_MAC4);
        enc28j60Write(MAADR0, ENC28J60_MAC5);

        // no loopback of transmitted frames
        enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);

        // switch to bank 0
        enc28j60SetBank(ECON1);
        // enable interrutps
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
        // enable packet reception
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

void enc28j60PacketSend(uint16_t len1, uint8_t* packet1)
{
	//Errata: Transmit Logic reset
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
	enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);

	// Set the write pointer to start of transmit buffer area
	enc28j60Write(EWRPTL, TXSTART_INIT&0xff);
	enc28j60Write(EWRPTH, TXSTART_INIT>>8);
	// Set the TXND pointer to correspond to the packet size given
	enc28j60Write(ETXNDL, (TXSTART_INIT+len1));
	enc28j60Write(ETXNDH, (TXSTART_INIT+len1)>>8);

	// write per-packet control byte
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);

	// copy the packet into the transmit buffer
	enc28j60WriteBuffer(len1, packet1);
	
	// send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}

uint16_t enc28j60PacketReceive(uint16_t maxlen, uint8_t* packet)
{
	uint16_t rxstat;
	uint16_t len;

	// check if a packet has been received and buffered
	//      if( !(enc28j60Read(EIR) & EIR_PKTIF) )
	if( !enc28j60Read(EPKTCNT) )
	return 0;
	
	// Make absolutely certain that any previous packet was discarded
	//if( WasDiscarded == FALSE)
	//      MACDiscardRx();

	// Set the read pointer to the start of the received packet
	enc28j60Write(ERDPTL, (NextPacketPtr));
	enc28j60Write(ERDPTH, (NextPacketPtr)>>8);
	// read the next packet pointer
	NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	NextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the packet length
	len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the receive status
	rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	rxstat |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

	// limit retrieve length
	// (we reduce the MAC-reported length by 4 to remove the CRC)
	len = MIN(len, maxlen);

	// copy the packet from the receive buffer
	enc28j60ReadBuffer(len, packet);

	// Move the RX read pointer to the start of the next received packet
	// This frees the memory we just read out
	enc28j60Write(ERXRDPTL, (NextPacketPtr));
	enc28j60Write(ERXRDPTH, (NextPacketPtr)>>8);

	// decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);

	return len;
}

