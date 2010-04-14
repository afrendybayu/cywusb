#include <MEGA8.h>

void EEPROM_tulis(unsigned int uiAddress, unsigned char ucData)  {
//    while(EECR & (1<<EEWE));  // Wait for completion of previous write
    while(EECR.1 & 1);          // EECR.1 = EEWE, cek EEWE = 1 ?
    EEAR = uiAddress;           // Set up address and data registers
    EEDR = ucData;
//    EECR |= (1<<EEMWE);         // Write logical one to EEMWE
    EECR.2 = 1;                 // set EECR.2 = EEME aktif untuk enable nulis 
//    EECR |= (1<<EEWE);          // Start eeprom write by setting EEWE
    EECR.1 = 1;                 // set EECR.2 = EEWE aktif untuk nulis 
}

unsigned char EEPROM_baca(unsigned int uiAddress) {
//    while(EECR & (1<<EEWE));    // Wait for completion of previous write
    while(EECR.1 & 1);          // EECR.1 = EEWE, cek EEWE = 1 ?
    EEAR = uiAddress;           // Set up address register
//    EECR |= (1<<EERE);          // Start eeprom read by writing EERE
    EECR.0 = 1;                 // set EECR.0 = EERE aktif untuk baca
    return EEDR;                // Return data from data register
}

void init_SPI_Master(void)
{   
    /*/ SPI master
        DORD = SPSR.5 = MSB dikirim dulu
        Fclk = Fosc/16
        CPOL = 0, leading edge = rising
        CPHA = 0, leading edge = sample    
    //*/ 
//    /* Set MOSI and SCK output, all others input */
//    DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK);
    /* Enable SPI, Master, set clock rate fck/16 */
//    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
    SPCR = 0x50;     // SPE = SPCR.6 = 1, MSTR = SPCR.4 = 1, SPR.0 = 1 (Fsoc/4)   0101 0001
    
}