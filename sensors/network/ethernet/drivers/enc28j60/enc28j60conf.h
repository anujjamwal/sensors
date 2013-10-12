
/*
 * enc28j60conf.h
 *
 * Created: 10/13/2013 12:07:24 AM
 *  Author: anujjamwal
 */ 

#ifndef _ENC28j60CONF_H_
#define _ENC28j60CONF_H_

// MAC address for this interface
#ifdef ETHADDR0
	#define ENC28J60_MAC0 ETHADDR0
	#define ENC28J60_MAC1 ETHADDR1
	#define ENC28J60_MAC2 ETHADDR2
	#define ENC28J60_MAC3 ETHADDR3
	#define ENC28J60_MAC4 ETHADDR4
	#define ENC28J60_MAC5 ETHADDR5
#else
	#define ENC28J60_MAC0 'S'
	#define ENC28J60_MAC1 'E'
	#define ENC28J60_MAC2 'N'
	#define ENC28J60_MAC3 'S'
	#define ENC28J60_MAC4 'O'
	#define ENC28J60_MAC5 'R'
#endif

// SPI port config for interface
#define ENC28J60_SPI_PORT               PORTB
#define ENC28J60_SPI_DDR                DDRB
#define ENC28J60_SPI_SCK                PB5
#define ENC28J60_SPI_MOSI               PB3
#define ENC28J60_SPI_MISO               PB4
#define ENC28J60_SPI_SS                 PB2

#endif