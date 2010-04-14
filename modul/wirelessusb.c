//#include <MEGA8.h>
//#include <spi.h>
#include <string.h>

//#ifndef _WUSB_
#define _WUSB_

#define uchr unsigned char
#define uint unsigned int

//#define wCS         PORTC.2           // change this wireless chip select pin based on your own design
#define wCS			PC2
#define wPD   		PC3       // pin 26

#define TLS         0x80
#define BURST       0x40

#define UnderflowTX 0x08
#define OverflowTX  0x04
#define DoneTX      0x02
#define EmptyTX     0x01

#define REG_ID		0x00
#define CONTROL     0x03
#define DATARATE    0x04
#define CONFIG      0x05
#define SERDES      0x06

#define RX_INT_EN   0x07
#define RX_INT_ST   0x08
#define RX_DATA_A   0x09
#define RX_VALID_A  0x0A
#define RX_DATA_B   0x0B
#define RX_VALID_B  0x0C
                        
#define TX_INT_EN   0x0D                        
#define TX_INT_ST   0x0E
#define TX_DATA     0x0F
#define TX_VALID    0x10 

#define WAKESTAT    0x1D
#define ANALOG_CTRL	0x20
#define CHANNEL     0x21
#define RSSI        0x22
#define PAB         0x23
#define VCO_CAL			0x26
#define PWR_CTL			0x2E
#define CARRIER_DETECT	0x2F
#define CLOCK_MANUAL	0x32
#define CLOCK_ENA	0x33

uchr bacaAnalogCtrl(void);
uchr tlsSingleAnalogCtrl(uchr);
uchr bacaVcoCal(void);
uchr tlsSingleVcoCal(uchr);
uchr bacaClockManual(void);
uchr tlsSingleClockManual(uchr);
uchr bacaClockEna(void);
uchr tlsSingleClockEna(uchr);
uchr bacaRegID(void);
uchr bacaPABias(void);
uchr tlsSinglePABias(uchr);
uchr bacaSERDES(void);
uchr tlsSingleSERDES(uchr);
uchr bacaKontrol(void);
uchr tlsSingleKontrol(uchr);
uchr modeKirim(void);
uchr modeTerima(void);
uchr bacaConfig(void);
uchr tlsSingleConfig(uchr);
uchr bacaDataRate(void);
uchr tlsSingleDataRate(uchr);
uchr bacaKanal(void);
uchr tlsSingleKanal(uchr);
uchr bacaEnaRX(void);
uchr tlsEnaRX(uchr);
uchr bacaEnaTX(void);
uchr tlsEnaTX(uchr);
uchr bacaStatusRX(void);
uchr bacaDataRX_A(void);
uchr bacaValidRX_A(void);
uchr bacaStatusTX(void);
uchr bacaValidTX(void);
uchr tlsSingleValidTX(uchr);
uchr bacaDataTX(void);
uchr tlsSingleDataTX(uchr);
uchr bacaValidRSSI(void);
uchr KirimData(uchr *);
uchr bacaStatusWake(void);
uchr bacaNilaiRSSI(void);


char spi(char cData) {
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}

uchr bacaAnalogCtrl() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(ANALOG_CTRL);        
    isi = spi(ANALOG_CTRL);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    
    return (isi);  
}

uchr tlsSingleAnalogCtrl(uchr data) {
    //wCS= 0;
    PORTC &= ~(_BV(wCS));
    spi(TLS | ANALOG_CTRL);      
    spi(data);
    //wCS= 1;
    PORTC |= (_BV(wCS));
    return (bacaAnalogCtrl());	
}

uchr bacaVcoCal() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(VCO_CAL);        
    isi = spi(VCO_CAL);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);  
}

uchr tlsSingleVcoCal(uchr data) {
    //wCS= 0;
    PORTC &= ~(_BV(wCS));
    spi(TLS | VCO_CAL);      
    spi(data);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (bacaVcoCal());	
}

uchr bacaClockManual() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(CLOCK_MANUAL);        
    isi = spi(CLOCK_MANUAL);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);  
}

uchr tlsSingleClockManual(uchr data) {
    //wCS= 0;
    PORTC &= ~(_BV(wCS));
    spi(TLS | CLOCK_MANUAL);      
    spi(data);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (bacaClockManual());	
}

uchr bacaClockEna() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(CLOCK_ENA);        
    isi = spi(CLOCK_ENA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);  
}

uchr tlsSingleClockEna(uchr data) {
    //wCS= 0;
    PORTC &= ~(_BV(wCS));
    spi(TLS | CLOCK_ENA);      
    spi(data);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (bacaClockEna());	
}

uchr bacaRegID() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(REG_ID);        
    isi = spi(REG_ID);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr bacaPABias() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(PAB);        
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr tlsSinglePABias(uchr data) {
    //wCS= 0;
    PORTC &= ~(_BV(wCS));
    spi(TLS | PAB);      
    spi(data);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (bacaPABias());
}

uchr bacaSERDES() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(SERDES);        
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr tlsSingleSERDES(uchr data) {
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TLS | SERDES);      
    spi(0x0F & data);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (bacaSERDES());
}

uchr bacaKontrol() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(CONTROL);        
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr tlsSingleKontrol(uchr data) {
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TLS | CONTROL);      
    spi(data);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (bacaKontrol());
}

uchr modeKirim() {
    return (tlsSingleKontrol(0x7F & bacaKontrol()));           
}

uchr modeTerima() {
    return (tlsSingleKontrol(0xBF & bacaKontrol()));           
}

uchr bacaConfig() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(CONFIG);          
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);                
}

uchr tlsSingleConfig(uchr data) {
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TLS | CONFIG);           
    spi(data);
    //wCS= 1;   
    PORTC |= (_BV(wCS));
    return (bacaConfig());
}

uchr bacaDataRate() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(DATARATE);          
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);                      
}

uchr tlsSingleDataRate(uchr data) {
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TLS | DATARATE);         
    spi(data);
    //wCS= 1;   
    PORTC |= (_BV(wCS));
    return (bacaDataRate());
}

uchr bacaKanal() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(CHANNEL);        
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr tlsSingleKanal(uchr data) {
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TLS | CHANNEL);      
    spi(0x7F & data);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (bacaKanal());
}

uchr bacaEnaRX() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(RX_INT_EN);
    isi = spi(0xAA);
    //wCS= 1;
    PORTC |= (_BV(wCS));   
    return (isi);           
}

uchr tlsEnaRX(uchr data) {
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TLS | RX_INT_EN);
    spi(0x0F & data);
    //wCS= 1;        
    PORTC |= (_BV(wCS));
    //return (bacaEnaRX());           
    return 1;
}

uchr bacaEnaTX() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TX_INT_EN);
    isi = spi(0xAA);
    //wCS= 1;
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr tlsEnaTX(uchr data) {
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TLS | TX_INT_EN);
    spi(0x0F & data);
    //wCS= 1;        
    PORTC |= (_BV(wCS));
    return (bacaEnaTX());           
}

uchr bacaStatusRX() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(RX_INT_ST);
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr bacaDataRX_A() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(RX_DATA_A);
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr bacaValidRX_A() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(RX_VALID_A);
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}


uchr bacaStatusTX() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TX_INT_ST);
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr bacaValidTX() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TX_VALID);        
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr tlsSingleValidTX(uchr data) {
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TLS | TX_VALID);      
    spi(data);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (bacaValidTX());
}

uchr bacaDataTX() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TX_DATA);
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);           
}

uchr tlsSingleDataTX(uchr data) {
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(TLS | TX_DATA);      
    spi(data);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (bacaDataTX());
}

uchr bacaStatusWake() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(WAKESTAT);          
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi);                
}

uchr bacaNilaiRSSI() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(RSSI);        
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi & 0x1F);           
}

uchr bacaValidRSSI() {
    uchr isi = 0;    
    //wCS= 0;   
    PORTC &= ~(_BV(wCS));
    spi(RSSI);        
    isi = spi(0xAA);
    //wCS= 1;    
    PORTC |= (_BV(wCS));
    return (isi & 0x20);           
}

uchr KirimData(uchr * data) {
    int i=0, j=0;
    modeKirim();
    tlsEnaTX(0x0F);
	j=strlen(data);
    for (i=0; i<j; i++) {
        while(!(bacaStatusTX() && EmptyTX));
            tlsSingleDataTX(data[i]);         
    }
    modeTerima();
    return 1;
}


//#endif
