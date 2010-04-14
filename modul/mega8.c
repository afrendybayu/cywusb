#include "mega8.h"


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
	
	// init timer 1	//
	
}

void init_io() {
	DDRD = 0x04;			//  SINYAL	PD2		0000 0100
	DDRC = 0x1D;			//	LED		PC0	
	DDRB = 0x2F;			//  0010 1111, PB2, PB1 output relay (1)
	
	__asm__ __volatile__ ("nop");		// untuk sinkronisasi
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
    _delay_ms(1);
} 

