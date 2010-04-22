#define F_CPU 8000000UL

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "./modul/CYWUSB693x.h"

char x [50];


#define BV(bit) (1<<(bit)) // Byte Value => converts bit into a byte value. One at bit location.
#define cbi(reg, bit) reg &= ~(BV(bit)) // Clears the corresponding bit in register reg
#define sbi(reg, bit) reg |= (BV(bit))              // Sets the corresponding bit in register reg

#define HEX__(n) 0x##n##UL

#define B8__(x) ((x&0x0000000FLU)?1:0)  \
  +((x&0x000000F0LU)?2:0)  \
  +((x&0x00000F00LU)?4:0)  \
  +((x&0x0000F000LU)?8:0)  \
  +((x&0x000F0000LU)?16:0) \
  +((x&0x00F00000LU)?32:0) \
  +((x&0x0F000000LU)?64:0) \
  +((x&0xF0000000LU)?128:0)

#define B8(d) ((unsigned char)B8__(HEX__(d)))

#define CYWM_SCK		PB1 // Output
#define CYWM_MISO		PB3	// Input
#define CYWM_MOSI		PB2	// Output
#define CYWM_nSS		PC2	// Output	

#define wCS		PC2
#define wPD   	PC3       // pin 26

#define DD_MOSI    3
#define DD_SCK     5
#define DDR_SPI    PORTB

#define CYWM_nPD		PC3	// Output	
#define CYWM_nRESET		PC4	// Output

#define CYWM_nRESET		PC4	// Output
#define CYWM_IRQ		PD1 // Input

#define low(port, pin) (port &= ~_BV(pin))
#define high(port, pin) (port |= _BV(pin))



#ifndef F_CPU
#define F_CPU 			8000000       	// Mhz 
#endif

#define LED		PC0

#define baud 38400		// coba  9600
#define ubrr ((F_CPU/(baud*16UL))-1)
#define bingung UCSRA

void transmit(unsigned char data) { 	// kirim data 1 chr
	//menunggu buffer transmit kosong
	while(!(bingung&(1<<UDRE))){}; //letakkan data di buffer, lalu kirim
	
	//while ( !( UCSRA & (1<<UDRE)) );
     UDR=data;
	//return UDR;
}

void transmitString(unsigned char * data) {		// kirim data kata
	int len, count;
	len = strlen(data);
	for (count = 0; count < len; count++)
		transmit(*(data+count));
}

void SPI_Write(uint8_t byte)
{
	SPDR = byte;				// Send SPI byte
	while(!(SPSR & (1<<SPIF)));	// Wait for SPI transmission complete
}

void CYWM_WriteReg(uint8_t which, uint8_t data)	{
	low(PORTC, wCS);		// CYWM_nSS
	SPI_Write(REG_WRITE | which);
	SPI_Write(data);
	high(PORTC, wCS);
	_delay_ms(1);
}

uint8_t CYWM_ReadReg(uint8_t which) {
	low(PORTC, wCS);
	_delay_us(1);
	SPI_Write(which);
	SPI_Write(which);
	high(PORTC, wCS);
	_delay_ms(1);
	return SPDR;
}


// Delay hekto-seconds :P
void delay_hs(uint16_t hs) 
{
	uint16_t n;
	for (n=0; n<hs; n++) {
		_delay_ms(10);
	}
}

void init_WUSB(void) {
	// wPD   	PC3       // pin 26
	// wRST  	PC4
	
    //wRST = 1;
//    PORTC |= (_BV(wPD));
    //wPD  = 1;
//    PORTC |= (_BV(wRST));

	high(PORTC, CYWM_nPD);
	high(PORTC, CYWM_nRESET);
}

void blink(void) {
	PORTC |= (_BV(LED));		// hidup
	_delay_ms(100);
	PORTC &= ~(_BV(LED));		// mati
}

int main(void)
{
	init_nya();
	
	int dowo;
	for (dowo=0; dowo<3; dowo++) {
		blink();
		_delay_ms(100);	
	}
	
	unsigned char data, n, i;

	// init radio
	transmitString("tesSerial\r\n");
	cli();	// Disable interrupts

	// Set port I/O directions
	//DDRB = _BV(CYWM_SCK) | _BV(CYWM_MOSI) | _BV(CYWM_nSS);
	//DDRD = _BV(CYWM_nRESET);
	//	DDRD = 0;


	// Setup radio
	// Test
	data = CYWM_ReadReg( REG_ID );
	if (data == 0x07) {
		transmitString("REG_ID == 0x07: OK!\n\r");
	}
	else {
		transmitString("REG_ID == 0x07: Failed!\n\r");
	}


	CYWM_WriteReg( REG_CLOCK_MANUAL, 0x41 );		// Must be written with 0x41 after reset for correct operation
	CYWM_WriteReg( REG_CLOCK_ENABLE, 0x41 );		// Must be written with 0x41 after reset for correct operation
	CYWM_WriteReg( REG_SERDES_CTL, 0x03 | 0x08 );	// Enable SERDES
	CYWM_WriteReg( REG_TX_VALID, 0xFF );			// Set all SERDES bits valid for TX
	CYWM_WriteReg( REG_VCO_CAL, 0xC0 );				// Set VCO adjust to -5/+5
	CYWM_WriteReg( REG_ANALOG_CTL, 0x44 );			// Enable PA Control Output		// 7654 3210
	CYWM_WriteReg( REG_PWR_CTL, 0x80 );				// Conserve power (must set REG_ANALOG_CTL, bit 6=1 to enable writes)
	CYWM_WriteReg( REG_CHANNEL, 42 );				// Use channel 42
	//CYWM_WriteReg( REG_PA, 0x07 );					// Set maximum transmit power
	CYWM_WriteReg( REG_PA, 0x00 );					// Set maximum transmit power
	CYWM_WriteReg( REG_RX_INT_EN, 0x03 );			// Enable EOF and FULL interrupts for channel A

/*
	CYWM_WriteReg( REG_CLOCK_MANUAL, 0x41 );
	CYWM_WriteReg( REG_CLOCK_ENABLE, 0x41 );	
	CYWM_WriteReg( REG_CONTROL, 0x10 );	      
	CYWM_WriteReg( REG_ANALOG_CTL, 0x44 );			
	CYWM_WriteReg( REG_CRYSTAL_ADJ, 0x40 );			
	CYWM_WriteReg( REG_VCO_CAL, 0xC0 );	   
	CYWM_WriteReg( REG_PA, 0x07 ); 
//*/
	int tmp=0;

  while (1) {

    for (i=0;i<82;i++){
    //cycle through channels. get RSSI and print value

      CYWM_WriteReg( REG_CONTROL, 0x80 + 0x10);		   
      CYWM_WriteReg( REG_CHANNEL, i);

      data = CYWM_ReadReg( REG_CHANNEL );
      data = CYWM_ReadReg( REG_CHANNEL );
      data = CYWM_ReadReg( REG_CHANNEL );

      CYWM_WriteReg( REG_CARRIER_DETECT, 0x00 );	    
      CYWM_WriteReg( REG_CARRIER_DETECT, 0x80 );	    
      
      data = CYWM_ReadReg( REG_RSSI );
	
		sprintf(x, "%2d. 24%02d [%sValid] RSSI: %d  _____data: 0x%x\r\n", i, i, (data&0x20)?"":"Tak ",data&0x1F, data);  transmitString(x);
      //sprintf(x, "%d]: 0x%x\r\n", i, data); transmitString(x);

      //        spi_write_addr(p, 0x00 + 0x10, REG_CONTROL);

      CYWM_WriteReg( REG_CONTROL, 0x00 + 0x10 );	    
		_delay_ms(500);
    }
  }  
}

void init_io() {
	DDRD = 0x00;			//  SINYAL	PD2		0000 0000
	DDRC = 0x1D;			//	LED		PC0		0001 1101
	DDRB = 0x2F;			//  0010 1111, PB2, PB1 output relay (1)
	
	//DDRB = _BV(CYWM_SCK) | _BV(CYWM_MOSI) | _BV(RELAY1) | _BV(RELAY2);
	//DDRC = _BV(CYWM_nPD) | _BV(CYWM_nRESET) | _BV(CYWM_nSS) | _BV(wCS) | _BV(LED);
	
	__asm__ __volatile__ ("nop");		// untuk sinkronisasi
}

void init_nya(void) {
	init_io();
	init_SPI_Master();
	usart_init();
	init_WUSB();
	//init_int();

}

void init_SPI_Master() {   
    DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK);
    SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR0);
}

void usart_init(void) {
	//set baud rate
	UBRRH= (unsigned char) (ubrr>>8);
	UBRRL= (unsigned char) ubrr;
	
	//interupt receiver, enable receiver dan transmeiter
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
	//UCSRB=0x98;
	//set frame format: 8 data, no parity, 1stop bit
	UCSRC=0x86; 
//	sprintf(x, "Serial OK !!%c", CR); transmitString(x);
	//sprintf(x, "isi UCSRB : %x%c", UCSRB, CR); transmitString(x);
	//sprintf(x, "isi SREG  : %x%c", SREG, CR); transmitString(x);
}
