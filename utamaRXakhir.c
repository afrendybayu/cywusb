
#define F_CPU 8000000
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>
//#include <string.h>
//#include <stdio.h>
#define baud 38400		// coba  9600
#define ubrr ((F_CPU/(baud*16UL))-1)
#define bingung UCSRA
#include "./modul/wirelessUSB.c"
//	#include <spi.h>

#define enter	13
#define pentung '!'
#define panjang	10  
#define NL		10
#define CR		13

#define DD_MOSI    3
#define DD_SCK     5
#define DDR_SPI    PORTB

#define wPD   	PC3       // pin 26
#define wRST  	PC4       // pin 27 v
#define DATA  	PD0       // pin  2

#define SINYAL	PD2			// pin no 5, di hanya ada di RX
#define LED		PC0			// pin no 23, aktif high

#define hidup 1
#define mati  0
#define P_HIDUP "MLP"

unsigned char perintah [panjang];
unsigned char nilai [2];
unsigned char x [50];
unsigned int panj;
char nyala;
unsigned char no;

main() {
	unsigned char tes;
	unsigned int dowo;
	init_io();
	init_SPI_Master();
	usart_init();
			//init_timer();
	init_int();
	
	init_WUSB();
	config_WUSB();
	
	//modeTerima();
//	sprintf(x, "Modul Penerima via wireless%c", CR);   transmitString(x);
	
	blink();
	PORTD |= _BV(SINYAL);
	_delay_ms(100);
	PORTD &= ~(_BV(SINYAL));
	
	nyala = 0;
	
//*	
	for(;;) {
		tes = bacaStatusRX();
		if (tes & 0x01) { // FULL A || EOF A
			tlsEnaRX(0x03);			// Enable EOF and FULL interrupts for channel A
			if (tes & 0x08) { // Valid A
				tes = bacaDataRX_A();
				if (tes == 'k' || tes == 'M') {
					blink();
					dowo = 1;
				} else if ((tes=='o' || tes=='L') && (dowo==1)) {
					dowo = 2;
				} else if ((tes=='P')&&(dowo==2)) {		// MLP -> sinyal jalan
					//blink();
					
					//PORTD |= _BV(SINYAL);
					//_delay_ms(100);
					//PORTD &= ~(_BV(SINYAL));
					
					//PORTD |= _BV(SINYAL);
					//*
					if (nyala) {
						PORTC &= ~(_BV(LED));
						PORTD &= ~(_BV(SINYAL));
						nyala = 0;
					} else {
						PORTC |= _BV(LED);
						PORTD |= _BV(SINYAL);
						nyala = 1;
					}
					//*/
					dowo = 0;
				} else if ((tes=='n')&&(dowo==2)) {
					dowo = 3;
				} else if ((tes=='e')&&(dowo==3)) {
					//blink();
					//*
					if (nyala) {
						PORTC &= ~(_BV(LED));
						nyala = 0;
					} else {
						PORTC |= (_BV(LED));
						nyala = 1;
					}
					//*/
					dowo = 4;
				} else if ((tes=='k')&&(dowo==4)) {
					blink();
					
					dowo = 0;
				} else {
					dowo = 0;
				}
				
				transmit(tes);
			}
			//data = CYWM_ReadReg( REG_RX_VALID_A );
		}
		//if (nyala & 0x02) {
		//	transmit(13);
		//		
		//}
	}
//*/


/*	
	while(1) {
		blink();
	}
//*/	
}

void usart_init() {
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

void transmit(unsigned char data) { 	// kirim data 1 chr
	//menunggu buffer transmit kosong
	while(!(bingung&(1<<UDRE))){}; //letakkan data di buffer, lalu kirim
	
	//while ( !( UCSRA & (1<<UDRE)) );
     UDR=data;
	//return UDR;
}

void transmitString(unsigned char * data) {		// kirim data kata
	unsigned int len, count;
	len = strlen(data);
	for (count = 0; count < len; count++)
		transmit(*(data+count));
}

unsigned char receive(){			// terima data 1 char
 //unsigned char data;
   //Wait for data to be received 
  //while ( !(bingung & (1<<RXC)) ) {};     
  while ( !(UCSRA & (1<<RXC)) );
  //Get and return received data from buffer 
   //return UDR;
   return UDR;
}

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

ISR(USART_RXC_vect) {
	//int len;
	cli();
	
	nilai[0] = receive();

    if (nilai[0] == 's') {
        shutdown_WUSB();
        return;
    }
    if (nilai[0] == 'k') {
        liatConfigWUSB();
        return;
    }
    if (nilai[0] == 'r') {
        shutdown_WUSB();
        reset();
        return;
    }
    if (nilai[0] == 'q') {
    	reset_wusb();
    	config_WUSB();
    	return;
	}
	if (nilai[0] == 'b') {
//		sprintf(x, "blink%c", CR); transmitString(x);
		blink();
		return;
	}
    
    transmit(nilai[0]);
	sei();
	
}

ISR(TIMER1_OVF_vect) {
    unsigned char y;
	cli();
//	buang_watchdog();					// perpanjang masa watchdog 2 detik lagi 
      
    TIMSK &= 0xFB;

	if (nyala) {
		PORTC &= ~(_BV(LED));
//		PORTD &= ~(_BV(SINYAL));
		PORTD &= ~(_BV(PD5));
		nyala = 0;
	} else {
		PORTC |= (_BV(LED));			// on , pin 23
//		PORTD |= (_BV(SINYAL));			// on , pin 5
		PORTD |= (_BV(PD5));
		nyala = 1;
	}

	no++;
    TCNT1  = 53817;
    TIFR  |= 0x04;        
    TIMSK |= 0x04;    
//    init_watchdog();
    sei();
}  




ISR(INT1_vect) {  // semula 
	char y;
	cli();
	//sprintf(x, "masuk INT1 MLP"); transmitString(x);


//	buang_watchdog();					// perpanjang masa watchdog 2 detik lagi 
/*
	y = bacaValidRSSI();
	sprintf(x, "%cValid Sinyal : %x", CR, bacaValidRSSI()); transmitString(x);
	if (y & 0x20) {
        sprintf(x," : VALID%c", CR);
        transmitString(x);
 	}
//	sprintf(x, "Sinyal Kuat  : %d (max:31)%c", bacaNilaiRSSI(), CR); transmitString(x);
	//sprintf(x, "baca nilai   : %x (max:31)%c", baca(), CR); transmitString(x);
	
//	y = bacaStatusRX();
//    sprintf(x, "Status RX    : %x%c", y, CR); transmitString(x);
    
//    sprintf(x, "Valid  RX    : %x%c", bacaValidRX_A(), CR); transmitString(x);
	//y = bacaDataRX_A();
	//sprintf(x, "Isi Data     : %c%c", y, CR); transmitString(x);
	
	//CYWM_WriteReg( REG_RX_INT_EN, 0x03 );
//	tlsEnaRX(0x03);


	if (nyala) {
		PORTC &= ~(_BV(LED));
		PORTD &= ~(_BV(SINYAL));
		PORTD &= ~(_BV(PD5));
		nyala = 0;
	} else {
		PORTC |= (_BV(LED));			// on , pin 23
		PORTD |= (_BV(SINYAL));			// on , pin 5
		PORTD |= (_BV(PD5));
		nyala = 1;
	}	
//*/	
	
	GIFR |= 0x80;
	//GICR |= 0x80;

    sei();
}


void init_SPI_Master() {   
    /*/ SPI master
        DORD = SPSR.5 = MSB dikirim dulu
        Fclk = Fosc/16
        CPOL = 0, leading edge = rising
        CPHA = 0, leading edge = sample    
    //*/ 
//    /* Set MOSI and SCK output, all others input */
    DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK);
    /* Enable SPI, Master, set clock rate fck/16 */
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
//    SPCR = 0x50;     // SPE = SPCR.6 = 1, MSTR = SPCR.4 = 1, SPR.0 = 1 (Fsoc/4)   0101 0001
}

void init_timer() { 
    TCCR1A = 0x00;//  init timer2
    TCCR1B = 0x05;                       // 0x05 (=128 us) = xtal/1024, 0x02 (=1 us) = xtal/8
    TCNT1  = 53817;						// jeda 2 detik (49911), 1.5 detik (53817)
	TIMSK |= 0x04;                      // timer1 overflow   
}

void init_io() {
	DDRD = 0x04;			//  SINYAL	PD2		0000 0100
	DDRC = 0x1D;			//	LED		PC0	
	DDRB = 0x2F;			//  
}

void init_int() {					// enable interrupt
//	GICR =  (1<<INT1);              // 0x40 : (1<<INT0);  
//	GICR = 0x80;
//	MCUCR |= 0x0C;    	         	// INT1 => 0x0C : rising edge, 0x08 : falling edge
    								// INT0 => 0x03 : rising edge, 0x02 : falling edge

//	GICR = 0x40;
//	MCUCR  |= 0x03;
	sei();
}

void reset() {
    //printf("%c%cReg : %x %c%c", CR, NL, WDTCR, CR, NL);    
    //printf("%c%cReset ! %c%c", CR, NL, CR, NL);
    WDTCR = 0x18;
    //printf("Reg1 : %x %c%c", WDTCR, CR, NL); 
    WDTCR |= 0x19;                          
    //printf("Reg2 : %x %c%c", WDTCR, CR, NL); 
    _delay_ms(100);
} 

void reset_wusb() {
	shutdown_WUSB();
	_delay_ms(100);
	init_WUSB();
}

void blink(void) {
	PORTC |= (_BV(LED));		// hidup
	_delay_ms(100);
	PORTC &= ~(_BV(LED));		// mati
}

void init_WUSB(void) {
	// wPD   	PC3       // pin 26
	// wRST  	PC4
	
    //wRST = 1;
    PORTC |= (_BV(wPD));
    //wPD  = 1;
    PORTC |= (_BV(wRST));

//    sprintf(x, "%cinit WUSB ..%c", CR, CR); transmitString(x);
//    sprintf(x, "WUSB OK   !!%c", CR); transmitString(x);
}

void shutdown_WUSB(void) {
    //wRST = 0;
    PORTC &= ~(_BV(wPD));
    //wPD  = 0;
    PORTC &= ~(_BV(wRST));
//    sprintf(x, "WUSB mati   %c", CR); transmitString(x);
}

void config_WUSB(void) {
	
	unsigned char wawa;
//	sprintf(x, "%cWireless ON ...%c", CR, CR); transmitString(x);
    _delay_ms(500);
    //blink();
    blink();
    blink();
//*
	
	tlsSingleClockManual(0x41);
    tlsSingleClockEna(0x41);
    tlsSingleSERDES(0x03 | 0x08);
    tlsSingleVcoCal(0xC0);
    tlsSingleAnalogCtrl(0x04);
    tlsSingleKanal(0x42);
    tlsSinglePABias(0x07);
    bacaConfig();
    tlsEnaRX(0x03);
    tlsSingleKontrol(0x80);
/*	
    sprintf(x, "Reg ID         : %x%c", bacaRegID(), CR); transmitString(x);
    EEPROM_tulis(3, bacaRegID());                                        
    
    sprintf(x, "Clock Manual   : %x%c", tlsSingleClockManual(0x41), CR);transmitString(x);
    sprintf(x, "Clock Enable   : %x%c", tlsSingleClockEna(0x41), CR); transmitString(x);
    sprintf(x, "Enable SerDes  : %x%c", tlsSingleSERDES(0x03 | 0x08), CR); transmitString(x);
//	sprintf(x, "Valid TX       : %x%c", tlsSingleValidTX(0xFF), CR);  transmitString(x);
    sprintf(x, "VCO Cal        : %x%c", tlsSingleVcoCal(0xC0), CR);  transmitString(x);     // 0x80, 0xC0
    sprintf(x, "Analog Control : %x%c", tlsSingleAnalogCtrl(0x04), CR); transmitString(x);
    sprintf(x, "Kanal          : %x%c", tlsSingleKanal(0x42), CR); transmitString(x);
    sprintf(x, "PA Bias        : %x%c", tlsSinglePABias(0x07), CR);  transmitString(x);
    sprintf(x, "Config         : %x%c", bacaConfig(), CR);  transmitString(x);
//	sprintf(x, "Ena TX Int     : %x%c", tlsEnaTX(0x01), CR); transmitString(x);
    sprintf(x, "Ena RX Int     : %x%c", tlsEnaRX(0x03), CR); transmitString(x);          // baca data A dan B
//	sprintf(x, "Kontrol RX TX  : %x%c", tlsSingleKontrol(0x40), CR); transmitString(x);
    sprintf(x, "Kontrol RX TX  : %x%c", tlsSingleKontrol(0x80), CR); transmitString(x);
//*/
          
} 

void liatConfigWUSB(void) {
    blink();
/*    
    sprintf(x, "%cKanal          : %x%c", CR, EEPROM_baca(0), CR); transmitString(x);
    sprintf(x, "Config         : %x%c", EEPROM_baca(1), CR); transmitString(x);
   	sprintf(x, "Ena TX Int     : %x%c", EEPROM_baca(2), CR); transmitString(x);
   	sprintf(x, "Reg ID         : %x%c", EEPROM_baca(3), CR); transmitString(x);
//*/    
/*
    sprintf(x, "%cReg ID         : %x%c", CR, bacaRegID(), CR); transmitString(x);
    sprintf(x, "Clock Manual   : %x%c", bacaClockManual(), CR); transmitString(x);
    sprintf(x, "Clock Enable   : %x%c", bacaClockEna(), CR); transmitString(x);
    sprintf(x, "Enable SerDes  : %x%c", bacaSERDES(), CR); transmitString(x);
    
    sprintf(x, "Valid RSSI     : %x%c", bacaValidRSSI(), CR);  transmitString(x);
    sprintf(x, "VCO Cal        : %x%c", bacaVcoCal(), CR);  transmitString(x);     // 0x80, 0xC0
    sprintf(x, "Analog Control : %x%c", bacaAnalogCtrl(), CR); transmitString(x);
    sprintf(x, "Kanal          : %x%c", bacaKanal(), CR); transmitString(x);
    sprintf(x, "PA Bias        : %x%c", bacaPABias(), CR);  transmitString(x);
    sprintf(x, "Config         : %x%c", bacaConfig(), CR);  transmitString(x);
    //printf("Ena TX Int     : %x%c%c", bacaEnaTX(), CR, NL); transmitString(x);
    sprintf(x, "Ena RX Int     : %x%c", bacaEnaRX(), CR);  transmitString(x);         // baca data A dan B
    sprintf(x, "Kontrol RX TX  : %x%c", bacaKontrol(), CR);  transmitString(x);       
    
    
    
    _delay_ms(50);
    
    
//*/
}
