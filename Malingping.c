/*
	12 April 2010
	Afrendy Bayu
	Optimalkan fitur Mega8
*/ 


#include <stdio.h>
#include "config.c"
//#include "./modul/wirelessUSB.c"
#include "./modul/CYWUSB693x.h"
//	#include <spi.h>

//#define KIRIM
//#define TEST_RELAY
#define TERIMA			// mode wireless TERIMA [default]
#define ATAS			// jika dikomen berarti bagian BAWAH


unsigned char nilai [2];
unsigned char x [50];
char itoogle_led, f_rpm;

unsigned long lama, konter, HIDUP;
unsigned int iT2, iT1, iT0, iPU;
static unsigned int rpmnya;


int main() {
	unsigned char tes;
	//unsigned int dowo;
	unsigned idle=0;
	char fT0=0;
	//unsigned angka;

	
	init_nya();
	lama = 0;	rpmnya=0;
	itoogle_led=0;
	f_rpm = 0;
	iT0 = 0; 	iPU = 0;
	iT2 = 0;	HIDUP = 0;
	
	int k=0;
	//int l=0;
	//angka = 0;

	_delay_ms(100);

//*
#ifdef TEST_RELAY
	int keledai = 10;
	while(1) {
		buang_watchdog();
		
		motor_Relay1();
		delaylama(keledai);
		
		motor_mati();
		delaylama(keledai);
		
		motor_Relay2();
		delaylama(keledai);
		
		motor_mati();
		delaylama(keledai);
	}
//*/
#else
	konfig_alat();
	while(1) {
		buang_watchdog();
//		motor_Relay1();
		//*
		//#ifdef TERIMA
		#ifdef WUSB_CYPRESS
		#if defined(TERIMA) || defined(KIRIM_TERIMA)
		tes = CYWM_ReadReg( REG_RX_INT_STAT );
		if (tes & 0x01 || tes & 0x02) { // FULL A || EOF A
			CYWM_WriteReg( REG_RX_INT_EN, 0x03 );			// Enable EOF and FULL interrupts for channel A
			if (tes & 0x08) { // Valid A
				tes = CYWM_ReadReg( REG_RX_DATA_A );
				wusb_in(tes);
				//transmit(tes);
			}
			//data = CYWM_ReadReg( REG_RX_VALID_A );
		}
		#endif
		#endif
		
		if(f_rpm) {				// tiap detik
		#ifdef ATAS
			kirim_rpm();
			f_rpm=0;
		#endif	
			toogle_led();		// tiap 1 detik toogle led: tanda wireless masih bekerja
		}
//*
		//if (l==1) {
		#ifdef KIRIM
		//	if (k==32000) 
			if (k==1000) 
		#endif	
			
		//#ifdef TERIMA
		#if defined(TERIMA)
			if (k==1000) 
		#endif
			{
				#ifdef DEBUG
					sprintf(x, "__________iT0: %d, iT1: %d, iPU: %d ", iT0, iT1, iPU);   transmitString(x);
					rpmnya = hitung_rpm();
					sprintf(x, "^^^^ rpm: %d\r\n", rpmnya);		transmitString(x);
				#endif
				k=0;
			} else {
				k++;
			}
		idle++;
//*/
	}
#endif
//*/	
	return 0;
}

#ifdef TEST_RELAY
//*
void delaylama(int kali) {
	int i=0;
	for (i=0; i<kali; i++) {
		buang_watchdog();
		_delay_ms(500);
	}
}
#endif
//*/
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
/*
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
//*/
}
#endif

#ifdef PAKAI_RELAY
void motor_mati(void) {
	PORTB &= ~(_BV(RELAY1));
	PORTB &= ~(_BV(RELAY2));
	__asm__ __volatile__ ("nop");
	_delay_ms(10);
}

void motor_Relay2(void) {
	motor_mati();
	PORTB |= _BV(RELAY2);
	__asm__ __volatile__ ("nop");
	_delay_ms(1);
}

void motor_Relay1(void) {
	motor_mati();
	PORTB |= _BV(RELAY1);
	__asm__ __volatile__ ("nop");
	_delay_ms(1);
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
    //shell(nilai[0]);
    karakter_in(nilai[0]);
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

ISR(TIMER0_OVF_vect) {
	unsigned char sreg;
	sreg = SREG;			// simpan global interrupt flag
	__asm__ __volatile__ ("cli" ::);
	//TCNT0=2;
	
	if(iT0==31) {		// nambah setiap detik
		iT0=0;
		HIDUP=HIDUP+1;
		f_rpm=1;
	} else {
		iT0++;
	}
    __asm__ __volatile__ ("sei" ::);
    
    //sei();

	
} 

/*
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

//ISR(INT1_vect) {  // semula 
//	cli();
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
	
//	GIFR |= 0x80;
	//GICR |= 0x80;

//    sei();
//}
//*/

void konfig_alat() {
	buang_watchdog();
	#ifdef WUSB_CYPRESS
		#ifdef TERIMA
			konfig_WUSB('b');		// 'b'	
		#endif
		
		#ifdef KIRIM
			konfig_WUSB('s');
		#endif
	#endif
	//config_WUSB();
}

void init_nya(void) {
	init_io();
	init_SPI_Master();
	usart_init();
	init_timer();
	init_int();
	
	#ifdef PAKAI_RELAY
		motor_mati();
	#endif
	
	init_watchdog();
	
	#ifdef WUSB_CYPRESS
		init_WUSB();
	#endif
	PORTC |= (_BV(LED));		// LED hidup
	
	#ifdef DEBUG
		tes_max();
	#endif

	
	#ifdef PAKAI_BLINK
	int dowo;
	for (dowo=0; dowo<3; dowo++) {
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
	sprintf(x, "Aplikasi Wireless v1.1 2010\r\n"); 	transmitString(x);
	sprintf(x, "Daun Biru Engineering\r\n"); 	transmitString(x);
//	sprintf(x, "Monita LIPI Malingping\r\n"); 	transmitString(x);
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
/*
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
	DDRC = 0x1D;			//	LED		PC0		0001 1101
	DDRB = 0x2F;			//  0010 1111, PB2, PB1 output relay (1)
	
	//DDRB = _BV(CYWM_SCK) | _BV(CYWM_MOSI) | _BV(RELAY1) | _BV(RELAY2);
	//DDRC = _BV(CYWM_nPD) | _BV(CYWM_nRESET) | _BV(CYWM_nSS) | _BV(wCS) | _BV(LED);
	
	__asm__ __volatile__ ("nop");		// untuk sinkronisasi
}

int passc=0;
char passin[16];

void karakter_in(unsigned char c) {
	if(c=='\n' || c=='\r' || c=='!') {
		// eksekusi !!
		transmit('\n');
		
		if (passc>0) {
			passin[passc] = '\0';
			aksinya(passin);
		}
		passc = 0;
		passin[0] = '\0';

		#ifdef DEBUG
			sprintf(x, "isi: %s\r\n", passin); 	transmitString(x);
		#endif
		//sprintf(x, "DuduKabel$ "); 	transmitString(x);
		  sprintf(x, "Wireless$ "); 	transmitString(x);
	} else if (c==8 || c==127) {
		if (passc>0) {
			sprintf(x, "\b \b"); 	transmitString(x);
			passc--;
			passin[passc]=0;			
		}
	} else {
		transmit(c);
		passin[ passc ] = c;
		passc++;
	}
}

int wusbc=0;
char wusbin[16];

void wusb_in(unsigned char c) {
	if(c=='\n' || c=='\r') {
		// eksekusi !!
		transmit('\n');
		
		if (wusbc>0) {
			wusbin[wusbc] = '\0';
			#ifndef ATAS
				if (strncmp(wusbin, "w_rpm", 5)==0) {
					paket_rpm(wusbin);
				} else
			#endif
				{
					aksinya(wusbin);
				}
		}
		wusbc = 0;
		wusbin[0] = '\0';

		#ifdef DEBUG
			sprintf(x, "isi: %s\r\n", wusbin); 	transmitString(x);
		#endif
		//sprintf(x, "Wireless$ "); 	transmitString(x);
	} else if (c==8 || c==127) {
		if (wusbc>0) {
			sprintf(x, "\b \b"); 	transmitString(x);
			wusbc--;
			wusbin[wusbc]=0;			
		}
	} else {
		transmit(c);
		//printf("%c",*c);
		wusbin[ wusbc ] = c;
		wusbc++;
	}
}

int aksinya(char * perintah) {
	buang_watchdog();
	//sprintf(x, "Perintahnya: %s, p: %d\r\n", perintah, strlen(perintah)); 	transmitString(x);
	if (strcmp(perintah, "rpm")==0) {
		sprintf(x, "CMD rpm: %d", hitung_rpm()); 	transmitString(x);
	} else if (strcmp(perintah, "mtrA")==0) {
		sprintf(x, "CMD: mtrA"); 	transmitString(x);
		#ifdef PAKAI_RELAY
			motor_Relay1();
		#endif
	} else if (strcmp(perintah, "mtrB")==0) {
		sprintf(x, "CMD: mtrB"); 	transmitString(x);
		#ifdef PAKAI_RELAY
			motor_Relay2();
		#endif
	} else if (strcmp(perintah, "reset")==0) {
		sprintf(x, "_____Reset_____"); 	transmitString(x);
		reset();
	} else if (strcmp(perintah, "uptime")==0) {
		sprintf(x, "uptime : "); 	transmitString(x);
		uptime();
	} else if (strncmp(perintah, "wusb",4)==0) {
		sprintf(x, "Kirim ke wusb"); 	transmitString(x);
		#ifdef WUSB_CYPRESS
			kirim_wusb(perintah);
		#endif
		//reset();
	} else if (strcmp(perintah, "konfig")==0) {
		//liatConfigWUSB();
		#ifdef WUSB_CYPRESS
			lihatKonfig();
		#endif
	} else if (strcmp(perintah, "mtrmati")==0) {
		sprintf(x, "CMD: matikan motor"); 	transmitString(x);
		#ifdef PAKAI_RELAY
			motor_mati();
		#endif
	#ifndef ATAS
	} else {
		sprintf(x, "CMD salah!! : %s\r\n", perintah); 	transmitString(x);
		sprintf(x, "CMD: rpm, mtrmati, mtrA, mtrB, reset, konfig, wusb"); 	transmitString(x);
	#endif
	}
	transmit('\n');
	//strcpy(passin,"");
	return 0;
}

#ifndef ATAS			// BAWAH
void paket_rpm(char * data) {
	char *pch;
	pch=strstr(data," ");
	if (pch!=NULL) {
		sprintf(x, "RPM kincir: %d\r\n", atoi(pch+1)); 	transmitString(x);
	}
}
#endif

void uptime() {
	int tmp=0;
	int hari=0;
	int jam=0;
	int menit=0;
	int detik=0;
	
	hari  = HIDUP/86400;		// (24*60*60) 
	tmp   = HIDUP%86400;
	jam   = tmp/3600;
	tmp   = tmp%3600;
	menit = tmp/60;
	detik = tmp%60;

	if (hari>0)		{	sprintf(x,"%d hari ",hari);transmitString(x);		}
	if (jam>0)		{	sprintf(x,"%d jam ",jam);transmitString(x);			}
	if (menit>0)	{	sprintf(x,"%d menit ",menit);transmitString(x);		}
	if (detik>0)	{	sprintf(x,"%d detik",detik);transmitString(x);	}
	
	#ifdef WUSB_CYPRESS
		tmp = CYWM_ReadReg(REG_RSSI);
		sprintf(x, "\r\n[%sValid] RSSI: %d", (tmp&0x20)?"":"Tak ",tmp&0x1F);  transmitString(x);
	#endif
	
}
#ifdef WUSB_CYPRESS
void kirim_wusb(char *cmd) {
	unsigned char rssi = CYWM_ReadReg(REG_RSSI);
	if ( (rssi&0x20)  ) {		// && (rssi&0x1F)>10
		CYWM_WriteReg( REG_CONTROL, 0x40 );		// mode kirim
		char *pch;
		pch=strstr(cmd," ");
		if (pch!=NULL) {
			sprintf(x, "%s", pch+1); 	
			
			#ifndef ATAS
				transmitString(x);
			#endif
			
			wireless_puts(x); wireless_putc('\n');
		}
		CYWM_WriteReg( REG_CONTROL, 0x80 );		// mode terima
	}
}
//*
void kirim_rpm() {
	sprintf(x, "n w_rpm %d", hitung_rpm()); 	//transmitString(x);
	kirim_wusb(x);
}
//*/
#endif

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



void buang_watchdog() {		// reset WDT
	__asm__ __volatile__ ("wdr");
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

void shutdown_WUSB(void) {
    //wRST = 0;
    PORTC &= ~(_BV(wPD));
    //wPD  = 0;
    PORTC &= ~(_BV(wRST));
//    sprintf(x, "WUSB mati   %c", CR); transmitString(x);
}

#ifdef WUSB_ASLI
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
	buang_watchdog();
	sprintf(x, "Konfig Wireless\r\n"); 	transmitString(x);
//	sprintf(x, "---------------\r\n"); 	transmitString(x);
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
  	buang_watchdog();
    sprintf(x, "Valid RSSI     : %x%c", bacaValidRSSI(), CR);  transmitString(x);
    sprintf(x, "VCO Cal        : %x%c", bacaVcoCal(), CR);  transmitString(x);     // 0x80, 0xC0
    sprintf(x, "Analog Control : %x%c", bacaAnalogCtrl(), CR); transmitString(x);
    sprintf(x, "Kanal          : %x%c", bacaKanal(), CR); transmitString(x);
    sprintf(x, "PA Bias        : %x%c", bacaPABias(), CR);  transmitString(x);
    sprintf(x, "Config         : %x%c", bacaConfig(), CR);  transmitString(x);
	sprintf(x, "Ena TX Int     : %x%c", bacaEnaTX(), CR); transmitString(x);
    sprintf(x, "Ena RX Int     : %x%c", bacaEnaRX(), CR);  transmitString(x);         // baca data A dan B
    sprintf(x, "Kontrol RX TX  : %x%c", bacaKontrol(), CR);  transmitString(x);       

    _delay_ms(5);
    
    
//*/
}
#endif

#ifdef WUSB_CYPRESS
void SPI_Write(uint8_t byte) {
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

void konfig_WUSB(char tipe) {
	uint8_t data;
	int flag, oz;
	
	buang_watchdog();
	flag=0;
	for (oz=0; oz<25; oz++) {
		if (flag == 0) {
			data = CYWM_ReadReg( REG_ID );
			if (data == 0x07) {
				flag = 1;
//				sprintf(x, "REG_ID: %x oz: %d!\n\r", data, oz);	transmitString(x);
				continue;
			}
		}
	}
	if (oz == 15) {
		sprintf(x, "REG_ID: %x: GAGAL!\n\r", data);
		transmitString(x);
		//return;
	}

	//sprintf(x,"REG_ID: 0x07: OK!, oz:%d \n\r", oz); 	transmitString(x);
	buang_watchdog();
	CYWM_WriteReg( REG_CLOCK_MANUAL, 0x41 );		// Must be written with 0x41 after reset for correct operation
	CYWM_WriteReg( REG_CLOCK_ENABLE, 0x41 );		// Must be written with 0x41 after reset for correct operation
	CYWM_WriteReg( REG_SERDES_CTL, 0x03 | 0x08 );	// Enable SERDES
	CYWM_WriteReg( REG_TX_VALID, 0xFF );			// Set all SERDES bits valid for TX
	CYWM_WriteReg( REG_VCO_CAL, 0xC0 );				// Set VCO adjust to -5/+5
	CYWM_WriteReg( REG_ANALOG_CTL, 0x04 );			// Enable PA Control Output
	//CYWM_WriteReg( REG_PWR_CTL, 0x80 );				// Conserve power (must set REG_ANALOG_CTL, bit 6=1 to enable writes)
	CYWM_WriteReg( REG_CHANNEL, 42 );				// Use channel 42
	CYWM_WriteReg( REG_PA, 0x07 );					// Set maximum transmit power
	CYWM_WriteReg( REG_RX_INT_EN, 0x03 );			// Enable EOF and FULL interrupts for channel A
	if (tipe=='b')		// b = baca
		CYWM_WriteReg( REG_CONTROL, 0x80 );				// Enable RX saja
	else if (tipe=='s')	// s = semua
		CYWM_WriteReg( REG_CONTROL, 0x80 | 0x40 );				// Enable TX RX
	else {				// t = tulis		k = kirim
		CYWM_WriteReg( REG_CONTROL, 0x40 );				// Enable TX saja
	}
}

void lihatKonfig() {
	buang_watchdog();
	sprintf(x, "Konfig Wireless "); 	transmitString(x);
	#ifdef ATAS
		transmitString("KINCIR ATAS");
	#else
		transmitString("BUNKER BAWAH");
	#endif
//	sprintf(x, "----------------\r\n"); 	transmitString(x);

	sprintf(x, "\r\nReg ID         : 0x%x\r\n", CYWM_ReadReg(REG_ID)); transmitString(x);
    sprintf(x, "Clock Manual   : 0x%x\r\n", CYWM_ReadReg(REG_CLOCK_MANUAL)); transmitString(x);
    sprintf(x, "Clock Enable   : 0x%x\r\n", CYWM_ReadReg(REG_CLOCK_ENABLE)); transmitString(x);
//    sprintf(x, "Enable SerDes  : %x\r\n", CYWM_ReadReg()); transmitString(x);
  	buang_watchdog();
    sprintf(x, "Valid RSSI     : %d  [RSSI max: 31]\r\n", CYWM_ReadReg(REG_RSSI)&0x1F);  transmitString(x);
    sprintf(x, "VCO Cal        : 0x%x\r\n", CYWM_ReadReg(REG_VCO_CAL));  transmitString(x);     // 0x80, 0xC0
    sprintf(x, "Analog Control : 0x%x\r\n", CYWM_ReadReg(REG_ANALOG_CTL)); transmitString(x);
    sprintf(x, "Kanal          : 0x%x\r\n", CYWM_ReadReg(REG_CHANNEL)); transmitString(x);
    sprintf(x, "PA Bias        : 0x%x\r\n", CYWM_ReadReg(REG_PA));  transmitString(x);
//    sprintf(x, "Config         : %x\r\n", CYWM_ReadReg());  transmitString(x);
//	sprintf(x, "Ena TX Int     : %x\r\n", CYWM_ReadReg()); transmitString(x);
//    sprintf(x, "Ena RX Int     : %x\r\n", CYWM_ReadReg());  transmitString(x);         // baca data A dan B
    sprintf(x, "Kontrol RX TX  : 0x%x   mode ", CYWM_ReadReg(REG_CONTROL));  transmitString(x);
    
    if (CYWM_ReadReg(REG_CONTROL)==0x80) {
		sprintf(x, "terima [Receiver]");  transmitString(x);
	} else {
		sprintf(x, "kirim [Transmitter]");  transmitString(x);
	}
}

void wireless_putc(char data) {
    // Transmit data
	_delay_us(10);
	while (!(CYWM_ReadReg( REG_TX_INT_STAT ) & 0x01));
	CYWM_WriteReg( REG_TX_INT_EN, 0x01 );			// Enable EMPTY interrupt
	CYWM_WriteReg( REG_TX_DATA, data);
}

void wireless_puts(char *data) {
	int len, count;

	len = strlen(data);
	for (count = 0; count < len; count++)
		wireless_putc(*(data+count));
}

void wireless_puti( const int val ) {
    char buffer[sizeof(int)*8+1];
    wireless_puts( (char*)(itoa(val, buffer, 10)) );
}
#endif
