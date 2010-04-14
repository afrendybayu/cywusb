
#include <stdio.h>
//#include <math.h>
#include "config.c"
#include "./modul/wirelessUSB.c"
//	#include <spi.h>

#define TERIMA

//unsigned char perintah [panjang];
unsigned char nilai [2];
unsigned char x [50];
char itoogle_led;

unsigned long lama,konter;
unsigned int iT2, iT1, iT0, iPU;
static unsigned int rpmnya;


int main() {
	unsigned char tes;
	unsigned idle=0;
	char flag_aksi=0;
	
	initnya();
	lama = 0;	rpmnya=0;
	itoogle_led=0;
	iT0 = 0; 	iPU = 0;
	iT2 = 0;
	
	int k=0;
	int l=0;
/*
//	motor_Relay2();
	while(1) {
		motor_Relay1();
		_delay_ms(1000);
		buang_watchdog();
		motor_Relay2();
		_delay_ms(1000);

	}
//*/
	konfig_alat();
//	buang_watchdog();
	while(1) {
		buang_watchdog();
		tes = bacaStatusRX();
		sprintf(x, "--%d--", tes);   transmitString(x);
		if (tes & 0x01) { // FULL A || EOF A
			tlsEnaRX(0x03);			// Enable EOF and FULL interrupts for channel A
			if (tes & 0x08) { // Valid A
				tes = bacaDataRX_A();
				sprintf(x, "**%d**", tes);   transmitString(x);
			}
		}
		//motor_Relay1();
		/*
		#ifdef PAKAI_RPM
		if (k==100) {
			rpmnya = hitung_rpm();
			k=0;
		} else {
			k++;
		}
		#endif
		//*/
		// setiap 2 detik akan toogle led //			penginnya .... pake Timer0 ko ga bisa :(
		// berguna ketika kincir tidak berputar //
		// sbg tanda wireless masih bekerja //
		if (l==10) {
			if (k==30000) {
				toogle_led();
				//sprintf(x, "__________iT0: %d, iT1: %d, iPU: %d ", iT0, iT1, iPU);   transmitString(x);
				//rpmnya = hitung_rpm();
				//sprintf(x, "^^^^ rpm: %d\r\n", rpmnya);		//transmitString(x);
				k=0;
			} else {
				k++;
			}
			l=0;
		} else {
			l++;
		}
		buang_watchdog();
		
//*
		if(iT0==31) {		// 62 = 2 detik, 31 = 1 detik
			sprintf(x, "__________RPM\r\n");   transmitString(x);
			rpmnya = hitung_rpm();
		} else if (iT0==62) {
			toogle_led();
		} else {
			iT0 = 0;
		}
		idle++;
//*/
	}
//*/	
	return 0;
}

unsigned int hitung_rpm(void) {
	unsigned long  rpm;
	unsigned int putaran=0;
	if (iT1>15)
		return 0;	
	rpm = 60000000/konter;	// 
	putaran = (unsigned int) rpm;
	#ifdef DEBUG
		sprintf(x, "konter: %ld, lama: %ld, rpm: %ld, putaran: %d  ", konter, lama, rpm, putaran);		transmitString(x);
	#endif
	
	return putaran;
} 



#ifdef DEBUG
void tes_max(void) {
	unsigned int max_int;
	long int a = 2147483647;		// 2147483648
	double max_float;
	max_int = 32767;		// 4294967295
	max_float = (float) 1000/3;	// 3.40282e+38
	sprintf(x, "Max int  : %d\r\n", max_int);		transmitString(x);
	sprintf(x, "Max float: %d\r\n", max_float);		transmitString(x);
	sprintf(x, "sizeof char: %d\n", sizeof(char));	transmitString(x);
	sprintf(x, "sizeof int: %d\n", sizeof(int));	transmitString(x);		// int 16 bit: max: 32767, 
	sprintf(x, "sizeof double: %d\n", sizeof(double));	transmitString(x);	// double 32 bit: max: 
	sprintf(x, "sizeof long: %d, long: %ld\n", sizeof(long int), a);	transmitString(x);	// double 32 bit: max: 
	int var1 = 5;
	float var2 = 6.4;
	double var3 = 8.7;

	sprintf(x, "Result: %d %f %f",var1,var2,var3); transmitString(x);
}
#endif

#ifdef PAKAI_RELAY
void motor_mati(void) {
	PORTB &= ~(_BV(RELAY1));
	PORTB &= ~(_BV(RELAY2));
	_delay_us(10);
	__asm__ __volatile__ ("nop");
}

void motor_Relay2(void) {
	motor_mati();
	PORTB |= _BV(RELAY2);
	_delay_us(10);
	__asm__ __volatile__ ("nop");
}

void motor_Relay1(void) {
	motor_mati();
	PORTB |= _BV(RELAY1);
	_delay_us(10);
	__asm__ __volatile__ ("nop");
}
#endif

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

unsigned char receive(void)	{			// terima data 1 char
 //unsigned char data;
   //Wait for data to be received 
  //while ( !(bingung & (1<<RXC)) ) {};     
  while ( !(UCSRA & (1<<RXC)) );
  //Get and return received data from buffer 
   //return UDR;
   return UDR;
}

//*
ISR(USART_RXC_vect) {
	cli();
	nilai[0] = receive();
    shell(nilai[0]);
	sei();
}
//*/
ISR(TIMER1_OVF_vect) {
	unsigned char sreg;
	sreg = SREG;			// simpan global interrupt flag

	__asm__ __volatile__ ("cli" ::);
	lama += 65535;
	iT1++;
    __asm__ __volatile__ ("sei" ::);
}  
/*
ISR(TIMER0_OVF_vect) {
	unsigned char sreg;
	sreg = SREG;			// simpan global interrupt flag
	cli();
	//__asm__ __volatile__ ("cli" ::);
	iT0++;
	TIFR  |= 0x01; 
	TIMSK |= 0x01;
	
    //__asm__ __volatile__ ("sei" ::);
    sei();
} 

ISR(TIMER2_OVF_vect) {
	unsigned char sreg;
	sreg = SREG;			// simpan global interrupt flag

	__asm__ __volatile__ ("cli" ::);
	TCNT2  = 0;
	iT0++;
    __asm__ __volatile__ ("sei" ::);
} 
//*/

ISR(INT0_vect) {
	unsigned char sreg;
	sreg = SREG;			// simpan global interrupt flag

	__asm__ __volatile__ ("cli" ::);
	lama+=TCNT1;
	TCNT1=0;
	konter=lama;
	__asm__ __volatile__ ("sei" ::);
	lama=0;
	iT1=0;
	iPU++;
	toogle_led();
}
/*
ISR(INT1_vect) {  // semula 
	cli();
	//sprintf(x, "masuk INT1 MLP"); transmitString(x);


//	buang_watchdog();					// perpanjang masa watchdog 2 detik lagi 

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

	
	GIFR |= 0x80;
	//GICR |= 0x80;

    sei();
}
//*/
void konfig_alat() {
	config_WUSB();
	buang_watchdog();
}

void initnya(void) {
	init_io();
	init_SPI_Master();
	usart_init();
	init_timer();
	init_int();
	
	init_watchdog();
	
	#ifdef PAKAI_WUSB
		init_WUSB();
	#endif
	PORTC |= (_BV(LED));		// hidup

	#ifdef DEBUG
		tes_max();
	#endif

	int i;
	#ifdef PAKAI_BLINK
	for (i=0; i<5; i++) {
		blink();
		_delay_ms(100);	
	}
	#endif
	iklan();
	karakter_in('\n');
	buang_watchdog();
}

void iklan() {
	sprintf(x, "\r\n\r\n"); 	transmitString(x);
	sprintf(x, "Aplikasi Wireless v1.1\r\n"); 	transmitString(x);
	sprintf(x, "Daun Biru Engineering\r\n"); 	transmitString(x);
	sprintf(x, "Monita LIPI Malingping 2010\r\n"); 	transmitString(x);
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
//*
	//  init timer1	//
    TCCR1A = 0x00;			
    TCCR1B = 0x02;			// 0x05 (=128 us) = xtal/1024, 0x02 (=1 us) = xtal/8
    TCNT1  = 0;				// jeda 2 detik (49911), 1.5 detik (53817)
	TIMSK |= 0x04;			// timer1 overflow
//*/
//*
	// init timer 2	//
	TCNT2 = 0;
	TCCR2 |= 0x07;			// 128 us
	TIMSK |= 0x40;
//*/	
//*	
	// init timer 0 //
	TCCR0 |= 0x05;
	TCNT0  = 0;
	TIMSK |= 0x01;
//*/		
}

void init_io() {
	DDRD = 0x00;			//  SINYAL	PD2		0000 0000
	DDRC = 0x1D;			//	LED		PC0	
	//DDRC = 0xFF;
	DDRB = 0x2F;			//  0010 1111, PB2, PB1 output relay (1)
	
	__asm__ __volatile__ ("nop");		// untuk sinkronisasi
}

int passc=0;
static char passin[16];

void shell(unsigned char c) {
	karakter_in(c);
}

void karakter_in(unsigned char c) {
	if(c=='\n' || c=='\r' || c=='!') {
		// eksekusi !!
		transmit('\n');
		
		if (passc>0) {
			aksinya(passin);
		}
		#ifdef DEBUG
			sprintf(x, "isi: %s\r\n", passin); 	transmitString(x);
		#endif
		passc = 0;
		strcpy(passin,"");
		sprintf(x, "Wireless$ "); 	transmitString(x);
	} else if (c==8 || c==127) {
		if (passc>0) {
			sprintf(x, "\b \b"); 	transmitString(x);
			passc--;
			passin[passc]=0;			
		}
	} else {
		transmit(c);
		//printf("%c",*c);
		passin[ passc ] = c;
		passc++;
	}
}

int aksinya(char * perintah) {
	if (strncmp(perintah, "rpm", strlen(perintah))==0) {
		sprintf(x, "rpm: %d\r\n", hitung_rpm()); 	transmitString(x);
	} else if (strncmp(perintah, "mtrA", strlen(perintah))==0) {
		sprintf(x, "perintah: mtrA\r\n"); 	transmitString(x);
		motor_Relay1();
	} else if (strncmp(perintah, "mtrB", strlen(perintah))==0) {
		sprintf(x, "perintah: mtrB\r\n"); 	transmitString(x);
		motor_Relay2();
	} else if (strncmp(perintah, "reset", strlen(perintah))==0) {
		sprintf(x, "_____Reset_____"); 	transmitString(x);
		reset();
	} else if (strncmp(perintah, "konfig", strlen(perintah))==0) {
		liatConfigWUSB();
//	} 
//	} else if (strncmp(perintah, "mtrmati", strlen(perintah))==0) {
//		sprintf(x, "perintah: matikan motor"); 	transmitString(x);
//		motor_mati();
	} else {
		sprintf(x, "perintah salah !!!\r\n"); 	transmitString(x);
		sprintf(x, "perintah: rpm, mtrA, mtrB\r\n"); 	transmitString(x);
	}
	transmit('\n');
	return 0;
}

void init_int() {					// enable interrupt
//	GICR =  (1<<INT1);              // 0x40 : (1<<INT0);  
//	GICR = 0x80;
//	MCUCR |= 0x0C;    	         	// INT1 => 0x0C : rising edge, 0x08 : falling edge
    								// INT0 => 0x03 : rising edge, 0x02 : falling edge

	// init INT0  //
	GICR = (1<<INT0);
	MCUCR |= 0x03;

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
    _delay_ms(2000);
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

void toogle_led(void)	{
	if (itoogle_led) {
		PORTC &= ~(_BV(LED));	// mati
		itoogle_led=0;
	} else {
		PORTC |= (_BV(LED));	// hidup
		itoogle_led=1;
	}
}

void init_watchdog() { 		// enable WDT
	//MCUCSR |= (0<<WDRF);
	//WDTCR  |= (1<<WDP2) | (1<<WDP1) | (1<<WDP0);		// 111: 2detik, 110: 1detik
	//WDTCR  |= (1<<WDE);
	
	
	WDTCR  |= (1<<WDCE) | (1<<WDE);
	WDTCR  |= (1<<WDP2) | (1<<WDP1) | (1<<WDP0);		// 111: 2detik, 110: 1detik
    //WDTCR = 0x18;
    //WDTCR |= 0x0F;       // 0000 1101  C = 0.25s , D = .5s, E = 1.1s, F = 2s
}



void buang_watchdog() {		// disable WDT
	__asm__ __volatile__ ("wdr");
	//_WDR();
    //WDTCR |= 0x18;
    //WDTCR  = 0x00;
    
    //WDTCR |= (1<<WDCE);
    //WDTCR |= (1<<WDE);
	/* Turn off WDT */
	//WDTCR |= (1<<WDCE) | (1<<WDE);
	//WDTCR = 0x00;

}

void init_WUSB(void) {
	// wPD   	PC3			// pin 26
	// wRST  	PC4			// 
	
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
	
//	unsigned char wawa;
//	sprintf(x, "%cWireless ON ...%c", CR, CR); transmitString(x);
//    _delay_ms(500);
    //blink();




	tlsSingleClockManual(0x41);
	_delay_ms(1);
	
    tlsSingleClockEna(0x41);
    _delay_ms(1);
    
    tlsSingleSERDES(0x03 | 0x08);
	_delay_ms(1);
    
    tlsSingleVcoCal(0xC0);
    _delay_ms(1);
    
    tlsSingleAnalogCtrl(0x04);
    _delay_ms(1);
    
    tlsSingleKanal(0x42);
    _delay_ms(1);
    
    tlsSinglePABias(0x07);
    _delay_ms(1);
    
    bacaConfig();
    _delay_ms(1);
    
    tlsEnaRX(0x03);
    _delay_ms(1);
    
	tlsSingleKontrol(0x80);
	_delay_ms(1);
/*	
	PORTD |= _BV(SINYAL);
	_delay_ms(300);
	PORTD &= ~(_BV(SINYAL));
	_delay_ms(100);
//*/
	
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
	sprintf(x, "Konfigurasi Wireless\r\n"); 	transmitString(x);
	sprintf(x, "--------------------\r\n"); 	transmitString(x);
//    blink();
/*    
    sprintf(x, "%cKanal          : %x%c", CR, EEPROM_baca(0), CR); transmitString(x);
    sprintf(x, "Config         : %x%c", EEPROM_baca(1), CR); transmitString(x);
   	sprintf(x, "Ena TX Int     : %x%c", EEPROM_baca(2), CR); transmitString(x);
   	sprintf(x, "Reg ID         : %x%c", EEPROM_baca(3), CR); transmitString(x);
//*/    

    sprintf(x, "Reg ID         : %x%c", bacaRegID(), CR); transmitString(x);
    sprintf(x, "Clock Manual   : %x%c", bacaClockManual(), CR); transmitString(x);
    sprintf(x, "Clock Enable   : %x%c", bacaClockEna(), CR); transmitString(x);
    sprintf(x, "Enable SerDes  : %x%c", bacaSERDES(), CR); transmitString(x);
  
    sprintf(x, "Valid RSSI     : %x%c", bacaValidRSSI(), CR);  transmitString(x);
    sprintf(x, "VCO Cal        : %x%c", bacaVcoCal(), CR);  transmitString(x);     // 0x80, 0xC0
    sprintf(x, "Analog Control : %x%c", bacaAnalogCtrl(), CR); transmitString(x);
    sprintf(x, "Kanal          : %x%c", bacaKanal(), CR); transmitString(x);
    sprintf(x, "PA Bias        : %x%c", bacaPABias(), CR);  transmitString(x);
    sprintf(x, "Config         : %x%c", bacaConfig(), CR);  transmitString(x);
	sprintf(x, "Ena TX Int     : %x%c", bacaEnaTX(), CR); transmitString(x);
    sprintf(x, "Ena RX Int     : %x%c", bacaEnaRX(), CR);  transmitString(x);         // baca data A dan B
    sprintf(x, "Kontrol RX TX  : %x%c", bacaKontrol(), CR);  transmitString(x);       
    
    
    
    _delay_ms(50);
    
    
//*/
}
