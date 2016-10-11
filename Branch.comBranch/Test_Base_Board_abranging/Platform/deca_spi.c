/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include <string.h>

#include "deca_spi.h"
#include "deca_sleep.h"
#include "deca_device_api.h"
#include "dwm1000_port.h"


int writetospi_serial( uint16 headerLength,
			   	    const uint8 *headerBuffer,
					uint32 bodylength,
					const uint8 *bodyBuffer
				  );

int readfromspi_serial( uint16	headerLength,
			    	 const uint8 *headerBuffer,
					 uint32 readlength,
					 uint8 *readBuffer );
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPI_DW1000*/)
{
	// done by port.c, default SPI used is SPI1

	return 0;

} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
	while (port_SPI_DW1000_busy_sending()); //wait for tx buffer to empty

	port_SPI_DW1000_disable();

	return 0;

} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */

int writetospi_serial
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       bodylength,
    const uint8 *bodyBuffer
)
{

	int i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

    SPI_DW1000_CS_GPIO->BSRRH = SPI_DW1000_CS;

    for(i=0; i<headerLength; i++)
    {
    	SPI_DW1000->DR = headerBuffer[i];

    	while ((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

    	SPI_DW1000->DR ;
    }

    for(i=0; i<bodylength; i++)
    {
     	SPI_DW1000->DR = bodyBuffer[i];

    	while((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

		SPI_DW1000->DR ;
	}

    SPI_DW1000_CS_GPIO->BSRRL = SPI_DW1000_CS;

    decamutexoff(stat) ;

    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */

int readfromspi_serial
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       readlength,
    uint8       *readBuffer
)
{

	int i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

    /* Wait for SPI_DW1000 Tx buffer empty */
    //while (port_SPI_DW1000_busy_sending());

    SPI_DW1000_CS_GPIO->BSRRH = SPI_DW1000_CS;

    for(i=0; i<headerLength; i++)
    {
    	SPI_DW1000->DR = headerBuffer[i];

     	while((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

     	readBuffer[0] = SPI_DW1000->DR ; // Dummy read as we write the header
    }

    for(i=0; i<readlength; i++)
    {
    	SPI_DW1000->DR = 0;  // Dummy write as we read the message body

    	while((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
 
	   	readBuffer[i] = SPI_DW1000->DR ;//port_SPI_DW1000_receive_data(); //this clears RXNE bit
    }

    SPI_DW1000_CS_GPIO->BSRRL = SPI_DW1000_CS;

    decamutexoff(stat) ;

    return 0;
} // end readfromspi()
