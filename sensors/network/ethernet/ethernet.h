/*
 * network.h
 *
 * Created: 10/13/2013 12:51:03 AM
 *  Author: anujjamwal
 */ 
#ifndef _NETWORK_H_
#define _NETWORK_H_

uint16_t network_read(uint16_t size, uint8_t * buffer);
void network_send(uint16_t size, uint8_t * buffer);
uint16_t network_read(void);
void network_send(void);
void network_set_MAC(uint8_t* mac);
void network_get_MAC(uint8_t* mac);
uint8_t network_link_state(void);

#endif