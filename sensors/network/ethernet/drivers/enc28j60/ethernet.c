/*
 * network.c
 *
 * Created: 10/13/2013 12:52:18 AM
 *  Author: anujjamwal
 */ 
#include "enc28j60.h"

uint16_t network_read(uint16_t size, uint8_t * buffer) {
	return enc28j60PacketReceive(size, buffer);
}

void network_send(uint16_t size, uint8_t * buffer){
	enc28j60PacketSend(size, buffer);
}

void network_init(void)
{
	enc28j60Init();
	enc28j60PhyWrite(PHLCON, 0x476); //setup led
}

void network_get_MAC(uint8_t* macaddr)
{
	*macaddr++ = enc28j60Read(MAADR5);
	*macaddr++ = enc28j60Read(MAADR4);
	*macaddr++ = enc28j60Read(MAADR3);
	*macaddr++ = enc28j60Read(MAADR2);
	*macaddr++ = enc28j60Read(MAADR1);
	*macaddr++ = enc28j60Read(MAADR0);
}

void network_set_MAC(uint8_t* macaddr)
{
	enc28j60Write(MAADR5, *macaddr++);
	enc28j60Write(MAADR4, *macaddr++);
	enc28j60Write(MAADR3, *macaddr++);
	enc28j60Write(MAADR2, *macaddr++);
	enc28j60Write(MAADR1, *macaddr++);
	enc28j60Write(MAADR0, *macaddr++);
}