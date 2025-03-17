#ifndef MT25QL_DEFINITIONS_H
#define MT25QL_DEFINITIONS_H

enum {
    /* RESET commands */
    SPI_FLASH_INS_REN		  			= 0x66,	/* Reset enable */
    SPI_FLASH_INS_RMEM		  			= 0x99,	/* Reset memory */

    /* IDENTIFICATION commands */
    SPI_FLASH_INS_RDID        			= 0x9F,	/* Read Identification */
    SPI_FLASH_INS_RDID_ALT    			= 0x9E,	/* Read Identification (alternative command) */
    SPI_FLASH_INS_MULT_IO_RDID   		= 0xAF, /* Read multiple I/O read id */
    SPI_FLASH_INS_DISCOVER_PARAMETER	= 0x5A, /* Read serial flash discovery parameter */

    /* DATA READ commands */
    SPI_FLASH_INS_READ 					= 0x03, /* Read Data Bytes */
    SPI_FLASH_INS_FAST_READ 			= 0x0B, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_DOFR 					= 0x3B,	/* Dual Output Fast Read */
    SPI_FLASH_INS_DIOFR 				= 0xBB, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QOFR 					= 0x6B, /* Quad Output Fast Read */
    SPI_FLASH_INS_QIOFR 				= 0xEB, /* Quad Input/Output Fast Read */
    SPI_FLASH_INS_4READ4D 				= 0xE7, /* Word Read Quad I/O */

    /* DATA READ commands (DTR dedicated instructions) */
    SPI_FLASH_INS_FAST_READDTR 			= 0x0D, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_DOFRDTR 				= 0x3D, /* Dual Output Fast Read */
    SPI_FLASH_INS_DIOFRDTR 				= 0xBD, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QOFRDTR 				= 0x6D, /* Quad Output Fast Read */
    SPI_FLASH_INS_QIOFRDTR 				= 0xED, /* Quad Input/Output Fast Read */

    /* DATA READ commands (32-bit address) */
    SPI_FLASH_INS_READ4BYTE 			= 0x13, /* Read Data Bytes */
    SPI_FLASH_INS_FAST_READ4BYTE 		= 0x0C, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_DOFR4BYTE 			= 0x3C, /* Dual Output Fast Read */
    SPI_FLASH_INS_DIOFR4BYTE 			= 0xBC, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QOFR4BYTE 			= 0x6C, /* Quad Output Fast Read */
    SPI_FLASH_INS_QIOFR4BYTE 			= 0xEC, /* Quad Input/Output Fast Read */

    /* DATA READ commands (32-bit address in DTR mode) */
    SPI_FLASH_INS_FAST_READDTR4BYTE 	= 0x0E, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_DIOFRDTR4BYTE 		= 0xBE, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QIOFRDTR4BYTE 		= 0xEE, /* Quad Input/Output Fast Read */

    /* REGISTER commands */
    SPI_FLASH_INS_RDSR 					= 0x05, /* Read Status */
    SPI_FLASH_INS_WRSR 					= 0x01, /* Write Status */
    SPI_FLASH_INS_RDFSR 				= 0x70, /* Read Flag Status */
    SPI_FLASH_INS_CLRFSR 				= 0x50, /* Clear Flag Status */
    SPI_FLASH_INS_RDNVCR 				= 0xB5, /* Read NV Configuration */
    SPI_FLASH_INS_WRNVCR 				= 0xB1, /* Write NV Configuration */
    SPI_FLASH_INS_RDVCR 				= 0x85, /* Read Volatile Configuration */
    SPI_FLASH_INS_WRVCR 				= 0x81, /* Write Volatile Configuration */
    SPI_FLASH_INS_RDVECR 				= 0x65, /* Read Volatile Enhanced Configuration */
    SPI_FLASH_INS_WRVECR 				= 0x61, /* Write Volatile Enhanced Configuration */
    SPI_FLASH_INS_WREAR 				= 0xC5, /* Write Extended Address */
    SPI_FLASH_INS_RDEAR 				= 0xC8, /* Read Extended Address */
    SPI_FLASH_INS_PPMR 					= 0x68, /* Program Protection Mgmt */
    SPI_FLASH_INS_RDPMR 				= 0x2B, /* Read Protection Mgmt */
    SPI_FLASH_INS_RDGPRR 				= 0x96, /* Read General Purpose Read */

    /* WRITE ENABLE commands */
    SPI_FLASH_INS_WREN        			= 0x06,	/* Write enable */
    SPI_FLASH_INS_WRDI        			= 0x04,	/* Write disable */

    /* PROGRAM DATA commands */
    SPI_FLASH_INS_PP 					= 0x02, /* Page Program  */
    SPI_FLASH_INS_DIFP					= 0xA2, /* Dual Input Fast Program  */
    SPI_FLASH_INS_DIEFP 				= 0xD2, /* Dual Input Extended Fast Program */
    SPI_FLASH_INS_QIFP 					= 0x32, /* Quad Input Fast Program */
    SPI_FLASH_INS_QIEFP					= 0x12,	/* Quad Input Extended Fast Program */
    SPI_FLASH_INS_QIEFP_ALT				= 0x38, /* Quad Input Extended Fast Program (alternative command) */

    /* PROGRAM DATA commands (32-bit address) */
    SPI_FLASH_INS_PP4BYTE 				= 0x12, /* Page Program with */
    SPI_FLASH_INS_QIFP4BYTE 			= 0x34, /* Quad Input Fast Program with */
    SPI_FLASH_INS_QIEFP4BYTE 			= 0x3E, /* Quad Input Extended Fast Program */
    SPI_FLASH_INS_SE4BYTE 				= 0xDC, /* Sector Erase with */
    SPI_FLASH_INS_SSE4BYTE 				= 0x21, /* Sub-Sector Erase with */

    /* ERASE DATA commands */
    SPI_FLASH_INS_SE					= 0xD8, /* Sector Erase */
    SPI_FLASH_INS_SSE					= 0x20, /* Sub-Sector Erase */
    SPI_FLASH_INS_SSE32K				= 0x52, /* Sub-Sector Erase for 32KB */
    SPI_FLASH_INS_BE					= 0xC7, /* Bulk Erase */
	SPI_FLASH_INS_DE                    = 0xC4, /* Die Erase */
    SPI_FLASH_INS_BE_ALT				= 0x60, /* Bulk Erase (alternative command) */

    /* RESUME/SUSPEND commands */
    SPI_FLASH_INS_PER 					= 0x7A, /* Program/Erase Resume */
    SPI_FLASH_INS_PES 					= 0x75, /* Program/Erase Suspend */

    /* Advanced Sectors Protection Commands */
    SPI_FLASH_INS_ASPRD 				= 0x2D, /* ASP Read */
    SPI_FLASH_INS_ASPP 					= 0x2C, /* ASP Program */
    SPI_FLASH_INS_DYBRD 				= 0xE8, /* DYB Read */
    SPI_FLASH_INS_DYBWR 				= 0xE5, /* DYB Write */
    SPI_FLASH_INS_PPBRD 				= 0xE2, /* PPB Read */
    SPI_FLASH_INS_PPBP 					= 0xE3, /* PPB Program */
    SPI_FLASH_INS_PPBE 					= 0xE4, /* PPB Erase */
    SPI_FLASH_INS_PLBRD 				= 0xA7, /* PPB Lock Bit Read */
    SPI_FLASH_INS_PLBWR 				= 0xA6, /* PPB Lock Bit Write */
    SPI_FLASH_INS_PASSRD 				= 0x27, /* Password Read */
    SPI_FLASH_INS_PASSP 				= 0x28, /* Password Write */
    SPI_FLASH_INS_PASSU 				= 0x29, /* Password Unlock */
    SPI_FLASH_INS_DYBRD4BYTE 			= 0xE0, /* DYB Read with 32-bit Address */
    SPI_FLASH_INS_DYBWR4BYTE 			= 0xE1, /* DYB Write with 32-bit Address */

    /* 4-byte address Commands */
    SPI_FLASH_INS_EN4BYTEADDR 			= 0xB7, /* Enter 4-byte address mode */
    SPI_FLASH_INS_EX4BYTEADDR 			= 0xE9, /* Exit 4-byte address mode */

    /* OTP commands */
    SPI_FLASH_INS_RDOTP					= 0x4B, /* Read OTP array */
    SPI_FLASH_INS_PROTP					= 0x42, /* Program OTP array */

    /* DEEP POWER-DOWN commands */
    SPI_FLASH_INS_ENTERDPD				= 0xB9, /* Enter deep power-down */
    SPI_FLASH_INS_RELEASEDPD			= 0xAB,  /* Release deep power-down */

    /* ADVANCED SECTOR PROTECTION commands */
    SPI_FLASH_ASPRD						= 0x2D, /* Advanced sector protection read */
    SPI_FLASH_ASPP						= 0x2C, /* Advanced sector protection program */
    SPI_FLASH_DYBRD						= 0xE8, /* Dynamic protection bits read */
    SPI_FLASH_DYBWR						= 0xE5, /* Dynamic protection bits write */
    SPI_FLASH_PPBRD						= 0xE2, /* Permanent protection bits read */
    SPI_FLASH_PPBP						= 0xE3, /* Permanent protection bits write */
    SPI_FLASH_PPBE						= 0xE4, /* Permanent protection bits erase */
    SPI_FLASH_PLBRD						= 0xA7, /* Permanent protection bits lock bit read */
    SPI_FLASH_PLBWR						= 0xA6, /* Permanent protection bits lock bit write	*/
    SPI_FLASH_PASSRD					= 0x27, /* Password read */
    SPI_FLASH_PASSP						= 0x28, /* Password write */
    SPI_FLASH_PASSU						= 0x29  /* Password unlock */

};



#endif