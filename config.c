


#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


//#define DEBUG
#define PAKAI_BLINK
#define PAKAI_RELAY
#define	PAKAI_RPM
				//#define PAKAI_WUSB
#define WUSB_CYPRESS


#define baud 38400		// coba  9600
#define ubrr ((F_CPU/(baud*16UL))-1)
#define bingung UCSRA

//#define NL		10
//#define CR		13

#define DD_MOSI    3
#define DD_SCK     5
#define DDR_SPI    PORTB

#define wCS		PC2
#define wPD   	PC3       // pin 26
#define wRST  	PC4       // pin 27 v
#define DATA  	PD0       // pin  2

#define CYWM_SCK		PB5 // Output
#define CYWM_MISO		PB4	// Input
#define CYWM_MOSI		PB3	// Output
#define CYWM_nSS		PC2	// Output

#define CYWM_nPD		PC3	// Output	
#define CYWM_nRESET		PC4	// Output
#define CYWM_IRQ		PD3 // Input

#define RELAY2	PB1
#define RELAY1	PB2

//#define SINYAL	PD2			// pin no 5, di hanya ada di RX
#define LED		PC0			// pin no 23, aktif high

#define low(port, pin) (port &= ~_BV(pin))
#define high(port, pin) (port |= _BV(pin))

void usart_init(void);
void transmit(unsigned char);
void transmitString(unsigned char *);
unsigned char receive(void);

void initnya(void);
void toogle_led(void);
void blink(void);
unsigned int hitung_rpm(void);
void iklan(void);

#ifdef PAKAI_TES_MAX
	void tes_max(void);
#endif

void init_SPI_Master(void);
void init_timer(void);
void init_io(void);
void init_int(void);

void init_watchdog(void);
void buang_watchdog(void);
void reset(void);

void motor_mati(void);
void motor_Relay2(void);
void motor_Relay1(void);

void init_WUSB(void);
void liatConfigWUSB(void);
void config_WUSB(void);
void shutdown_WUSB(void);
void reset_wusb(void);

void wireless_putc(char);
void wireless_puts(char *);
void wireless_puti(const int);

uint8_t CYWM_ReadReg(uint8_t);
void CYWM_WriteReg(uint8_t, uint8_t);

#ifdef PAKAI_EEPROM
void EEPROM_tulis(unsigned int uiAddress, unsigned char ucData)  {
  // Wait for completion of previous write
  while(EECR & (1<<EEWE))   ;
  // Set up address and data registers
  EEAR = uiAddress;
  EEDR = ucData;
  // Write logical one to EEMWE
  EECR |= (1<<EEMWE);
  // Start eeprom write by setting EEWE 
  EECR |= (1<<EEWE);

}

unsigned char EEPROM_baca(unsigned int uiAddress) {
	// Wait for completion of previous write */
	while(EECR & (1<<EEWE));
	// Set up address register */
	EEAR = uiAddress;
	// Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	// Return data from data register */
	return EEDR;
}
#endif
