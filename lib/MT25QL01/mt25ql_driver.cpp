#include "mt25ql_driver.h"

//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// PUBLIC ///////////////////////////

MT25QL::MT25QL(int8_t sck, int8_t cs, int8_t io0, int8_t io1, int8_t io2, int8_t io3)
    : _sck(sck), _cs(cs), _io0(io0), _io1(io1), _io2(io2), _io3(io3) {}

void MT25QL::begin() {
    configureQSPI();
}

void MT25QL::end(void) {
    QSPI->CTRLA.bit.ENABLE = 0;
  
    MCLK->APBCMASK.bit.QSPI_ = false;
    MCLK->AHBMASK.bit.QSPI_ = false;
    // MCLK->AHBMASK.bit.QSPI_2X_ = false;
}

void MT25QL::eraseSector(uint32_t address) {
    // return runCommand(SPI_FLASH_INS_SE, address, nullptr, 0, QSPI_INSTRFRAME_TFRTYPE_WRITE);
}













//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// PRIVATE ///////////////////////////

void MT25QL::configureQSPI() {
    // Habilitar periférico QSPI
    MCLK->APBCMASK.bit.QSPI_ = true;
    MCLK->AHBMASK.bit.QSPI_ = true;

    QSPI->CTRLA.bit.SWRST = 1; // Reset del QSPI
    
    // Configurar los pines en modo QSPI
    pinPeripheral(_sck, PIO_COM);
    pinPeripheral(_cs,  PIO_COM);
    pinPeripheral(_io0, PIO_COM);
    pinPeripheral(_io1, PIO_COM);
    pinPeripheral(_io2, PIO_COM);
    pinPeripheral(_io3, PIO_COM);

    // Configurar velocidad del reloj
    QSPI->BAUD.reg = QSPI_BAUD_BAUD(120000000 / 4000000UL);
    
    // Configuración básica del QSPI
    QSPI->CTRLB.reg = QSPI_CTRLB_MODE_MEMORY | QSPI_CTRLB_DATALEN_8BITS |
                      QSPI_CTRLB_CSMODE_LASTXFER;
    
    QSPI->CTRLA.bit.ENABLE = 1;
}

void MT25QL::enableCache() {
    CMCC->CTRL.bit.CEN = 1;
}

void MT25QL::disableCache() {
    CMCC->CTRL.bit.CEN = 0;
    while (CMCC->SR.bit.CSTS) {}
    CMCC->MAINT0.bit.INVALL = 1;
}
