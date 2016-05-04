/*!
 * \file fis1100_driver.c
 * \version 1.0.1 rev. 44584
 * \copyright
 * Copyright (c) 2015 Fairchild Semiconductor Corporation or subsidiaries
 * worldwide. All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. 	Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 * 2. 	Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 * 3. 	Neither the names of the copyright holders nor the names of their
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "fis1100_driver.h"
#include "fis1100_hal.h"
#include <string.h> // for memset
#include <assert.h>
#include <math.h>

#ifndef M_PI
/*! \brief Float value of Pi */
#define M_PI (3.14159265358979323846f)
#endif

/*! \brief Standard value of Earth gravitational acceleration in ms^-2 */
#define ONE_G (9.81f)

/*!
 * \brief Internal driver settings structure
 */
struct Fis1100_DriverSettings
{
	float m_accScaleFactor; /*!< \brief Conversion factor from LSB to
	                           acceleration */
	float m_gyrScaleFactor; /*!< \brief Conversion factor from LSB to calibrated
	                           rotational rate */
	float m_magScaleFactor; /*!< \brief Conversion factor from LSB to calibrated
	                           magnetic field */
	uint8_t m_fifoMode;     /*!< \brief Cached FIFO control register value  */
};

/*!
 * \brief Driver settings
 */
static struct Fis1100_DriverSettings g_fisDriverSettings;

/*! \brief HAL functions */
static struct Fis1100Hal const* g_fisDriverHal;

/*!
 * \brief Miscellaneous command codes
 *
 * These commands are use to configure advanced functions of the FIS1100
 */
enum Fis1100_Ctrl9Command
{
	/*! \brief No operation */
	Ctrl9_Nop = 0,
	/*! \brief Reset FIFO */
	Ctrl9_ResetFifo = 0x2,
	/*! \brief Set magnetometer X calibration values */
	Ctrl9_SetMagXCalibration = 0x6,
	/*! \brief Set magnetometer Y calibration values */
	Ctrl9_SetMagYCalibration = 0x7,
	/*! \brief Set magnetometer Z calibration values */
	Ctrl9_SetMagZCalibration = 0x8,
	/*! \brief Set accelerometer offset correction value */
	Ctrl9_SetAccelOffset = 0x12,
	/*! \brief Set gyroscope offset correction value */
	Ctrl9_SetGyroOffset = 0x13,
	/*! \brief Update magnemoter bias compensation */
	Ctrl9_UpdateMagBias = 0xB,
	/*! \brief Trigger motion on demand sample */
	Ctrl9_TriggerMotionOnDemand = 0x0c,
	/*! \brief Update gyroscope bias compensation */
	Ctrl9_UpdateAttitudeEngineGyroBias = 0xE,
	/*! \brief Prepare for FIFO read sequence */
	Ctrl9_ReadFifo = 0x0D,
	/*! \brief Set wake on motion paramenters */
	Ctrl9_ConfigureWakeOnMotion = 0x19
};

/*!
 * \brief Perform a miscellaneous command
 * \param cmd The command to perform
 */
void doCtrl9Command(enum Fis1100_Ctrl9Command cmd)
{
	while (g_fisDriverHal->int1Asserted())
	{
		// Ensure INT1 is low before starting Control-9 sequence
	}

	uint8_t data[2];
	data[0] = FisRegister_Ctrl9;
	data[1] = cmd;

	Fis1100_writeRegisters(data, sizeof(data));
	while (!g_fisDriverHal->int1Asserted())
	{
		// wait for INT1 to go high to indicate command is being processed
	}

	// Check that command has been executed
	uint8_t status = 0;
	Fis1100_readRegisters(FisRegister_Status1, &status, sizeof(status));
	assert(status & FIS1100_STATUS1_CMD_DONE);

	while (g_fisDriverHal->int1Asserted())
	{
		// wait for INT1 to go low to indicate execution is complete
	}
}

/*!
 * \brief Apply patch
 */
void applyPatch(void)
{
	uint8_t w1[] = {0x0b, 0xfe, 0xca};
	Fis1100_writeRegisters(w1, sizeof(w1));
	uint8_t w2[] = {0x0a, 0x10};
	Fis1100_writeRegisters(w2, sizeof(w2));

	g_fisDriverHal->delayMicroseconds(100000);
	while(!(Fis1100_readStatus1() & 0x01))
		;

	uint8_t w3[] = {0x0a, 0x00};
	Fis1100_writeRegisters(w3, sizeof(w3));
	Fis1100_writeRegisters(w3, sizeof(w3));

	uint8_t w4[] = {0x09, 0x01};
	Fis1100_writeRegisters(w4, sizeof(w4));

	g_fisDriverHal->delayMicroseconds(100000);
	while(!(Fis1100_readStatus1() & 0x80))
		;

	uint8_t w5[] = {0x09, 0x03};
	Fis1100_writeRegisters(w5, sizeof(w5));
	g_fisDriverHal->delayMicroseconds(100000);

	uint8_t w6[] = {0x3e, 0x02, 0xba};
	Fis1100_writeRegisters(w6, sizeof(w6));

	uint8_t data;
	Fis1100_readRegisters(0x0d, &data, sizeof(data));
	uint8_t w7[] = {0x0d, data & 0x7f, 0x32};
	Fis1100_writeRegisters(w7, sizeof(w7));

	Fis1100_readRegisters(0x0f, &data, sizeof(data));
	uint8_t w8[] = {0x0f, data & 0xf0};
	Fis1100_writeRegisters(w8, sizeof(w8));

	uint8_t w9[] = {0x3e, 0x00, 0x00};
	Fis1100_writeRegisters(w9, sizeof(w9));

	uint8_t w10[] = {0x09, 0x00};
	Fis1100_writeRegisters(w10, sizeof(w10));

	g_fisDriverHal->delayMicroseconds(100000);
	while(!(Fis1100_readStatus1() & 0x80))
		;
}

/*!
 * \brief Perform a hardware reset of the FIS1100
 *
 * \note This function will block until the reset is complete
 */
void Fis1100_reset(void)
{
	g_fisDriverHal->assertReset(true);
	g_fisDriverHal->delayMicroseconds(100);
	g_fisDriverHal->assertReset(false);
	while (!g_fisDriverHal->int1Asserted())
	{
		// wait for rising edge on INT1
	}
	while (g_fisDriverHal->int1Asserted())
	{
		// wait for INT1 to go low to indicate reset complete
	}

	uint8_t patch[] = {FisRegister_Cal1_L, 0x03, 0x23, 0xA7, 0x0F, 0x01, 0x01};
	uint8_t data[2];
	Fis1100_readRegisters(FisRegister_WhoAmI, data, 2);
	Fis1100_writeRegisters(patch, sizeof(patch));
	doCtrl9Command((enum Fis1100_Ctrl9Command)0x94);
	
#ifndef NEW_OIS_MODE
	applyPatch();
#endif

}

/*!
 * \brief Set or clear function enable bits in \a register
 * \param address The address of the register to configure
 * \param bitmask Bitmask representing functions to enable or disable \see CTRL7
 * \param enable \c true to enable function, \c false to disable
 */
void setEnableBits(uint8_t address, uint8_t bitmask, bool enable)
{
	uint8_t data[2];

	// Read the current configuration into data buffer
	Fis1100_readRegisters(address, &data[1], 1);

	// Update the required enable bits according to bitmask
	if (enable)
	{
		data[1] |= bitmask;
	}
	else
	{
		data[1] &= ~bitmask;
	}

	// Prepare register address
	data[0] = address;

	// Write the updated sensor enable register value
	Fis1100_writeRegisters(data, sizeof(data));
}

/*!
 * \brief Initialize FIS1100 driver
 * \param halFunctions Pointer to a Fis1100Hal structure containing HAL
 * function
 *pointers
 *
 * This function should be called before any other driver function to
 * initialize the driver internals.
 *
 * \note The callback function structure should remain in scope for duration of
 *the application.
 *
 * \remarks The FIS1100 component is reset as part of the initialization
 * procedure. This is a blocking operation.
 */
void Fis1100_init(struct Fis1100Hal const* halFunctions)
{
	memset(&g_fisDriverSettings, 0, sizeof(g_fisDriverSettings));
	g_fisDriverHal = halFunctions;
	Fis1100_reset();
}

/*!
 * \brief Write data to FIS1100 registers.
 * \param data The data buffer to write, with the first byte containing register
 * address.
 * \param dataLength The number of bytes to write, including the address byte.
 *
 * The first byte of data is assumed to be the register address to start
 * writing from. This byte is modified to set the auto-increment bit and clear
 * the read bit if required, depending on if the driver is using SPI or I2C
 * mode.
 */
void Fis1100_writeRegisters(uint8_t* data, uint8_t dataLength)
{
	const uint8_t readBit = g_fisDriverHal->spiMode ? 0x80 : 0x00;
	const uint8_t autoIncBit = g_fisDriverHal->spiMode ? 0x40 : 0x80;

	// Ensure read bit is cleared
	data[0] &= ~readBit;

	if (g_fisDriverHal->spiMode && (dataLength > 2))
	{
		// Work around bug in FIS1100 SPI implementation that causes issues
		// with bust write operations
		uint8_t address = data[0] & ~autoIncBit;
		uint8_t buf[2];
		for (int i = 1; i < dataLength; ++i)
		{
			buf[0] = address + i - 1;
			buf[1] = data[i];
			g_fisDriverHal->writeData(buf, sizeof(buf));
		}
	}
	else
	{
		// Set auto-increment bit if necessary
		data[0] |= (dataLength > 2) ? autoIncBit : 0;
		g_fisDriverHal->writeData(data, dataLength);
	}
}

/*!
 * \brief Read data from FIS1100 registers.
 * \param address The FIS1100 register address to start reading from.
 * \param data The data buffer to read into.
 * \param dataLength The number of bytes to read.
 *
 * The register address is modified to set the read and auto-increment bits if
 * required, depending on if the driver is using SPI or I2C mode.
 */
void Fis1100_readRegisters(uint8_t address, uint8_t* data, uint8_t dataLength)
{
	const uint8_t readBit = g_fisDriverHal->spiMode ? 0x80 : 0x00;
	const uint8_t autoIncBit = g_fisDriverHal->spiMode ? 0x40 : 0x80;

	// Set read bit
	address |= readBit;

	// Set auto-increment bit if necessary
	address |= (dataLength > 1) ? autoIncBit : 0;

	g_fisDriverHal->readData(address, data, dataLength);
}

/*!
 * \brief Configure FIS1100 programmable FIFO operation
 * \returns The number of samples available in the FIFO when the watermark
 * interrupt is triggered
 */
uint32_t Fis1100_configureFifo(enum Fis1100_FifoWatermarkLevel watermark,
                               enum Fis1100_FifoSize size,
                               enum Fis1100_FifoMode mode)
{
	doCtrl9Command(Ctrl9_ResetFifo);
	g_fisDriverSettings.m_fifoMode = (uint8_t)watermark | (uint8_t)size | (uint8_t)mode;

	uint8_t data[] = {FisRegister_FifoCtrl, g_fisDriverSettings.m_fifoMode};
	Fis1100_writeRegisters(data, sizeof(data));

	uint32_t fifoSize = 0;
	switch (size)
	{
		case FifoSize_16: fifoSize = 16; break;
		case FifoSize_32: fifoSize = 32; break;
		case FifoSize_64: fifoSize = 64; break;
		case FifoSize_128: fifoSize = 128; break;
	}
	uint32_t watermarkLevel = 0;
	switch (watermark)
	{
		//! \todo Check if this should be 0 or 1
		case Fifo_WatermarkEmpty: watermarkLevel = 0; break;
		case Fifo_WatermarkOneQuarter: watermarkLevel = fifoSize / 4; break;
		case Fifo_WatermarkHalf: watermarkLevel = fifoSize / 2; break;
		case Fifo_WatermarkThreeQuarters:
			watermarkLevel = (fifoSize * 3) / 4;
			break;
	}

	return (mode == FifoMode_Disable) ? 0 : watermarkLevel;
}

/*!
 * \brief Configure FIS1100 FIFO to interrupt after a target number of samples
 * \param samples The number of samples to configure as the FIFO watermark
 * \returns The number of samples available when watermark interrupt triggered
 *
 * \remark The FIFO only supports a limited set of watermark levels. These are
 * 4, 8, 12, 16, 24, 32, 48, 64 and 96 samples. Any other value will disable the
 * FIFO.
 *
 * \note FIFO configuration is based on individual data samples. When using
 * multiple sensors multiple samples will be written to the FIFO at each ODR.
 * For example, when accelerometer, magnetometer and gyroscope are configured
 * three samples will be written to the FIFO at each ODR.
 *
 * \note When using the FIFO the accelerometer and gyroscope must be configured
 * for the same ODR. The magnetometer, if enabled, will have its samples repeated
 * to match the gyroscope ODR.
 */
uint32_t Fis1100_configureFifoSamples(uint32_t samples)
{
	switch (samples)
	{
		case 4: return Fis1100_configureFifo(Fifo_WatermarkOneQuarter,
						FifoSize_16, FifoMode_Stream);
		case 8: return Fis1100_configureFifo(Fifo_WatermarkHalf,
						FifoSize_16, FifoMode_Stream);
		case 12: return Fis1100_configureFifo(Fifo_WatermarkThreeQuarters,
						FifoSize_16, FifoMode_Stream);
		case 16: return Fis1100_configureFifo(Fifo_WatermarkHalf,
						FifoSize_32, FifoMode_Stream);
		case 24: return Fis1100_configureFifo(Fifo_WatermarkThreeQuarters,
						FifoSize_32, FifoMode_Stream);
		case 32: return Fis1100_configureFifo(Fifo_WatermarkHalf,
						FifoSize_64, FifoMode_Stream);
		case 48: return Fis1100_configureFifo(Fifo_WatermarkThreeQuarters,
						FifoSize_64, FifoMode_Stream);
		case 64: return Fis1100_configureFifo(Fifo_WatermarkHalf,
						FifoSize_128, FifoMode_Stream);
		case 96: return Fis1100_configureFifo(Fifo_WatermarkThreeQuarters,
						FifoSize_128, FifoMode_Stream);
		default: return Fis1100_disableFifo();
	}
}

/*!
 * \brief Lock access to the FIFO for the host system.
 *
 * This function gives the host exclusive access to the FIS1100 FIFO memory and
 * must be called before reading FIFO data using Fis1100_readFifo().
 */
void Fis1100_lockFifo(void)
{
	doCtrl9Command(Ctrl9_ReadFifo);

	const uint8_t fifoReadLock = 0x80;
	uint8_t ctrl[] = {FisRegister_FifoCtrl, g_fisDriverSettings.m_fifoMode | fifoReadLock};
	Fis1100_writeRegisters(ctrl, sizeof(ctrl));
}

/*!
 * \brief Blocking read data from the FIS1100 FIFO
 * \param data Pointer to buffer to read FIFO data into
 * \param dataLength The number of bytes to read from the FIFO
 *
 * Before calling this function the Fis1100_lockFifo() function must
 * be called to lock the FIFO for read access. After the data has been read
 * the Fis1100_unlockFifo() function must be called to release the FIFO
 * lock.
 *
 * When multiple sensors are enabled the data in the FIFO is interleaved.
 * Starting from an empty FIFO the order is {magnetometer, accelerometer,
 * gyroscope}.
 */
void Fis1100_readFifo(uint8_t* data, uint8_t dataLength)
{
	const uint8_t readBit = g_fisDriverHal->spiMode ? 0x80 : 0x00;
	g_fisDriverHal->readData(FisRegister_FifoData | readBit, data, dataLength);
}

/*!
 * \brief Release FIFO access lock.
 *
 * This function must be called after reading data from the FIFO to allow the
 * FIS1100 to write new data to the FIFO.
 */
void Fis1100_unlockFifo(void)
{
	uint8_t ctrl[] = {FisRegister_FifoCtrl, g_fisDriverSettings.m_fifoMode};
	Fis1100_writeRegisters(ctrl, sizeof(ctrl));
}


/*!
 * \brief Set accelerometer configuration
 * \param range The full scale range to use
 * \param odr The output data rate to use
 * \param unit The output unit to use for processed data
 * \param lpfEnable #Lpf_Enable to enable low pass filter, #Lpf_Disable to
 * disable
 */
void Fis1100_configureAccelerometer(enum Fis1100_AccRange range,
                                    enum Fis1100_AccOdr odr,
                                    enum Fis1100_AccUnit unit,
                                    enum Fis1100_LpfConfig lpfEnable)
{
	// Set the CTRL2 register to configure dynamic range and ODR
	uint8_t data[] = {
	    FisRegister_Ctrl2,            // Register address
	    (uint8_t)range | (uint8_t)odr // Configuration
	};
	Fis1100_writeRegisters(data, sizeof(data));

	// Store the scale factor for use when processing raw data
	switch (range)
	{
		case AccRange_2g:
			g_fisDriverSettings.m_accScaleFactor = 1.0f / (1 << 14);
			break;
		case AccRange_4g:
			g_fisDriverSettings.m_accScaleFactor = 1.0f / (1 << 13);
			break;
		case AccRange_8g:
			g_fisDriverSettings.m_accScaleFactor = 1.0f / (1 << 12);
			break;

		default: assert(0); // Invalid dynamic range selection
	}

	// Conversion from g to m/s^2 if necessary
	if (unit == AccUnit_ms2)
	{
		g_fisDriverSettings.m_accScaleFactor *= ONE_G;
	}

	// Configure accelerometer Low Pass Filter enable bit
	setEnableBits(FisRegister_Ctrl5, FIS1100_CTRL5_ACC_LPF_ENABLE,
	              lpfEnable == Lpf_Enable);
}

/*!
 * \brief Set gyroscope configuration
 * \param range The full scale range to use
 * \param odr The output data rate to use
 * \param unit The output unit to use for processed data
 * \param lpfEnable #Lpf_Enable to enable low pass filter, #Lpf_Disable to
 * disable
 */
void Fis1100_configureGyroscope(enum Fis1100_GyrRange range,
                                enum Fis1100_GyrOdr odr,
                                enum Fis1100_GyrUnit unit,
                                enum Fis1100_LpfConfig lpfEnable)
{
	// Set the CTRL3 register to configure dynamic range and ODR
	uint8_t data[] = {
	    FisRegister_Ctrl3,            // Register address
	    (uint8_t)range | (uint8_t)odr // Configuration
	};
	Fis1100_writeRegisters(data, sizeof(data));

	// Store the scale factor for use when processing raw data
	switch (range)
	{
		case GyrRange_32dps:
			g_fisDriverSettings.m_gyrScaleFactor = 1.0f / (1 << 10);
			break;
		case GyrRange_64dps:
			g_fisDriverSettings.m_gyrScaleFactor = 1.0f / (1 << 9);
			break;
		case GyrRange_128dps:
			g_fisDriverSettings.m_gyrScaleFactor = 1.0f / (1 << 8);
			break;
		case GyrRange_256dps:
			g_fisDriverSettings.m_gyrScaleFactor = 1.0f / (1 << 7);
			break;
		case GyrRange_512dps:
			g_fisDriverSettings.m_gyrScaleFactor = 1.0f / (1 << 6);
			break;
		case GyrRange_1024dps:
			g_fisDriverSettings.m_gyrScaleFactor = 1.0f / (1 << 5);
			break;
		case GyrRange_2048dps:
			g_fisDriverSettings.m_gyrScaleFactor = 1.0f / (1 << 4);
			break;
		case GyrRange_2560dps:
			g_fisDriverSettings.m_gyrScaleFactor = 1.0f / (1 << 3);
			break;

		default: assert(0); // Invalid dynamic range selection
	}

	// Conversion from degrees/s to rad/s if necessary
	if (unit == GyrUnit_rads)
	{
		g_fisDriverSettings.m_gyrScaleFactor *= M_PI / 180;
	}

	// Configure gyroscope Low Pass Filter enable bit
	setEnableBits(FisRegister_Ctrl5, FIS1100_CTRL5_GYR_LPF_ENABLE,
	              lpfEnable == Lpf_Enable);
}

/*!
 * \brief Set slave magnetometer configuration
 * \param device The type of slave magnetometer
 * \param odr The output data rate to use
 * \param unit The output unit to use for processed data
 *
 * \todo Implement support for magnetometers other than AKM AK8975
 */
void Fis1100_configureMagnetometer(enum Fis1100_MagDev device,
                                   enum Fis1100_MagOdr odr,
                                   enum Fis1100_MagUnit unit)
{
	// Set the CTRL4 register to configure device type and ODR
	uint8_t data[] = {
	    FisRegister_Ctrl4,             // Register address
	    (uint8_t)device | (uint8_t)odr // Configuration
	};
	Fis1100_writeRegisters(data, sizeof(data));

	// Magnetometer output in Gauss with 11 fractional bits
	g_fisDriverSettings.m_magScaleFactor = 1.0f / (1 << 11);

	if (unit == MagUnit_microTesla)
	{
		// 100uT per Gauss
		g_fisDriverSettings.m_magScaleFactor *= 100;
	}

	// Set mag calibration values
	uint8_t calData[] = {
	    FisRegister_Cal1_L, // Write to calibration registers
	    0x27,               // Scale LSB
	    0x31,               // Scale MSB
	    0x00,               // Offset LSB
	    0x00,               // Offset MSB
	    0x00,               // Cross axis 1 LSB
	    0x00,               // Cross axis 1 MSB
	    0x00,               // Cross axis 2 LSB
	    0x00,               // Cross axis 2 MSB
	};
	Fis1100_writeRegisters(calData, sizeof(calData));
	doCtrl9Command(Ctrl9_SetMagXCalibration);
	doCtrl9Command(Ctrl9_SetMagYCalibration);
	doCtrl9Command(Ctrl9_SetMagZCalibration);
}

/*!
 * \brief Set AttitudeEngine configuration
 * \param odr The output rate of the AttitudeEngine
 *
 * \note When using the AttitudeEngine the accelerometer and gyroscope are
 * processed at 1024Hz with the full scale range set to 8g and 2048dps
 * respectively. It is not possible to change these parameters.
 */
void Fis1100_configureAttitudeEngine(enum Fis1100_AeOdr odr)
{
	// Configure Accelerometer and Gyroscope settings
	Fis1100_configureAccelerometer(AccRange_8g, AccOdr_1024Hz, AccUnit_ms2,
	                               Lpf_Enable);
	Fis1100_configureGyroscope(GyrRange_2048dps, GyrOdr_1024Hz, GyrUnit_rads,
	                           Lpf_Enable);

	// Configure the AttitudeEngine output rate
	uint8_t data[] = {
	    FisRegister_Ctrl6, // AttitudeEngine control register
	    odr            // Configuration
	};

	Fis1100_writeRegisters(data, sizeof(data));
}

/*!
 * \brief Enable or disable the accelerometer
 * \param enable \c true to enable accelerometer, \c false to disable
 *
 * \note When enabling or disabling multiple sensors it is recommended to use
 * Fis1100_enableSensors().
 */
void Fis1100_enableAccelerometer(bool enable)
{
	setEnableBits(FisRegister_Ctrl7, FIS1100_CTRL7_ACC_ENABLE, enable);
}

/*!
 * \brief Enable or disable the gyroscope
 * \param enable \c true to enable gyroscope, \c false to disable
 * \note When enabling or disabling multiple sensors it is recommended to use
 * Fis1100_enableSensors().
 */
void Fis1100_enableGyroscope(bool enable)
{
	setEnableBits(FisRegister_Ctrl7, FIS1100_CTRL7_GYR_ENABLE, enable);
}

/*!
 * \brief Enable or disable the magnetometer
 * \param enable \c true to enable magnetometer, \c false to disable
 * \note When enabling or disabling multiple sensors it is recommended to use
 * Fis1100_enableSensors().
 */
void Fis1100_enableMagnetometer(bool enable)
{
	setEnableBits(FisRegister_Ctrl7, FIS1100_CTRL7_MAG_ENABLE, enable);
}

/*!
 * \brief Enable or disable the AttitudeEngine
 * \param enable \c true to enable AttitudeEngine, \c false to disable
 *
 * \note Enabling or disabling the AttitudeEngine will also enable or disable
 * the accelerometer and gyroscope. This is done so that these sensors can be
 * reconfigured for the desired settings before being enabled.

 * \note When enabling or disabling multiple sensors it is recommended to use
 * Fis1100_enableSensors().
 */
void Fis1100_enableAttitudeEngine(bool enable)
{
	setEnableBits(FisRegister_Ctrl7,
	              FIS1100_CTRL7_ACC_ENABLE | FIS1100_CTRL7_GYR_ENABLE |
	                  FIS1100_CTRL7_AE_ENABLE,
	              enable);
}

/*!
 * \brief Enable multiple sensor functions at once
 * \param enableFlags Sensor enable flags \see FIS_CTRL7
 *
 * Use this function to enable multiple sensors at a time.  For example, to
 * enable the accelerometer and magnetometer include the following:
 * \code
 * Fis1100_enableSensors(FIS1100_CTRL7_ACC_ENABLE | FIS1100_CTRL7_MAG_ENABLE);
 * \endcode
 *
 * \remark If the the AttitudeEngine enable flag is set then the
 * accelerometer and gyroscope enable flags are automatically set.
 *
 * \note The passed enable flags will override the existing setting.
 * Therefore \a enableFlags should contain flags for all required sensors,
 * even if these have been previously enabled.
 */
void Fis1100_enableSensors(uint8_t enableFlags)
{
	if (enableFlags & FIS1100_CTRL7_AE_ENABLE)
	{
		enableFlags |= FIS1100_CTRL7_ACC_ENABLE | FIS1100_CTRL7_GYR_ENABLE;
	}

	uint8_t data[2];
	data[0] = FisRegister_Ctrl7;
	data[1] = enableFlags & FIS1100_CTRL7_ENABLE_MASK;
	Fis1100_writeRegisters(data, sizeof(data));
}
/*!
 * \brief Enable Wake on Motion mode
 * \param interrupt The interrupt pin to use to indicate wake up events
 * \param initialState The initial state of the interrupt pin
 * \param threshold The threshold to use for detecting wake up events
 * \param blankingTime Number of 3Hz accelerometer samples to ignore before
 * allowing wakeup event (0-63)
 */
void Fis1100_enableWakeOnMotion(enum Fis1100_Interrupt interrupt,
		enum Fis1100_InterruptInitialState initialState,
		enum Fis1100_WakeOnMotionThreshold threshold,
		uint8_t blankingTime)
{
	Fis1100_enableSensors(FIS1100_CTRL7_DISABLE_ALL);
	Fis1100_configureAccelerometer(AccRange_2g, AccOdr_LowPower_3Hz,
			AccUnit_ms2, Lpf_Disable);

	const uint8_t blankingTimeMask = 0x3F;
	uint8_t womCmd[] = {FisRegister_Cal1_L, threshold,
		(uint8_t)interrupt | (uint8_t)initialState | (blankingTime & blankingTimeMask)};
	Fis1100_writeRegisters(womCmd, sizeof(womCmd));

	doCtrl9Command(Ctrl9_ConfigureWakeOnMotion);
	Fis1100_enableAccelerometer(true);
}

/*!
 * \brief Check to see if a wake up event has occurred
 */
bool Fis1100_hasWakeupEventOccured(void)
{
	uint8_t status;
	Fis1100_readRegisters(FisRegister_Status1, &status, sizeof(status));
	return status & FIS1100_STATUS1_WAKEUP_EVENT;
}

/*!
 * \brief Disable Wake on Motion mode
 */
void Fis1100_disableWakeOnMotion(void)
{
	Fis1100_enableSensors(FIS1100_CTRL7_DISABLE_ALL);
	uint8_t womDisableCommand[] = {FisRegister_Cal1_L, 0};
	Fis1100_writeRegisters(womDisableCommand, sizeof(womDisableCommand));
	doCtrl9Command(Ctrl9_ConfigureWakeOnMotion);
}

/*!
 * \brief Write offset calibration vector to user calibration registers
 * \param offset Pointer to offset 3-vector
 * \param conversionFactor Conversion factor to apply to convert to native units
 * (g for accelerometer, deg/s for gyroscope)
 * \param fractionalBits The number of fractional bits to use in the conversion
 * from floating point to fixed point representation
 */
static inline void writeOffsetCalibrationBuffer(float const* offset,
		float conversionFactor, uint8_t fractionalBits)
{
	uint8_t calCmd[7];
	calCmd[0] = FisRegister_Cal1_L;
	for (int i = 0; i < 3; ++i)
	{
		int16_t o = (int16_t)roundf(offset[i] * conversionFactor * (1 << fractionalBits));
		calCmd[1 + (2 * i)] = o & 0xFF;
		calCmd[1 + (2 * i) + 1] = o >> 8;
	}
	Fis1100_writeRegisters(calCmd, sizeof(calCmd));
}

/*!
 * \brief Apply offset correction to accelerometer calibration
 * \param offset Pointer to a 3-vector offset
 * \param unit The unit used to represent the offset
 *
 * The offset parameters should be calculated as
 * \f$offset_i = \frac{(max(acc_i) + min(acc_i)}{2}\f$
 * The minimuim and maximum values should be estimated by placing the part in
 * six orientations so that each axis experiences +1g and -1g.
 */
void Fis1100_applyAccelerometerOffset(float const* offset, enum Fis1100_AccUnit unit)
{
	const float conversionFactor = (unit == AccUnit_ms2) ? 1 / ONE_G : 1;
	writeOffsetCalibrationBuffer(offset, conversionFactor, 11);
	doCtrl9Command(Ctrl9_SetAccelOffset);
}

/*!
 * \brief Apply offset correction to gyroscope calibration
 * \param offset Pointer to a 3-vector offset
 * \param unit The unit used to represent the offset
 *
 * The offset pararmeters can be estimated based on the gyroscope output while
 * the part is stationary.
 *
 * \note This function differs from Fis1100_updateAttitudeEngineGyroBias() in
 * that this function corrects for offset errors in both AttitudeEngine and
 * regular IMU mode. This function should be used during platform initialization
 * to correct for systematic calibration errors.
 * The Fis1100_updateAttitudeEngineGyroBias() function can be used in run to
 * feed back estimated bias updates from the XKF3 fusion filter to the Attitude
 * Engine in run.
 */
void Fis1100_applyGyroscopeOffset(float const* offset, enum Fis1100_GyrUnit unit)
{
	const float conversionFactor = (unit == GyrUnit_rads) ? 180 / M_PI : 1;
	writeOffsetCalibrationBuffer(offset, conversionFactor, 6);
	doCtrl9Command(Ctrl9_SetGyroOffset);
}

/*!
 * \brief Blocking read of data status register 0 (::FisRegister_Status0)
 * \returns Status byte \see STATUS0 for flag definitions
 */
uint8_t Fis1100_readStatus0(void)
{
	uint8_t status;
	Fis1100_readRegisters(FisRegister_Status0, &status, sizeof(status));
	return status;
}
/*!
 * \brief Blocking read of data status register 1 (::FisRegister_Status1)
 * \returns Status byte \see STATUS1 for flag definitions
 */
uint8_t Fis1100_readStatus1(void)
{
	uint8_t status;
	Fis1100_readRegisters(FisRegister_Status1, &status, sizeof(status));
	return status;
}

/*!
 * \brief Blocking read of data count register (::FisRegister_CountOut)
 * \returns Sample counter value
 */
uint8_t Fis1100_readCount(void)
{
	uint8_t count;
	Fis1100_readRegisters(FisRegister_CountOut, &count, sizeof(count));
	return count;
}

/*!
 * \brief Blocking read of accelerometer data
 * \param data Pointer to 3-vector where data should be stored
 */
void Fis1100_readAccelerometerData(float* data)
{
	uint8_t buffer[FIS1100_SAMPLE_SIZE];
	Fis1100_readRawAccelerometerData(buffer);
	Fis1100_processAccelerometerData(buffer, data);
}

/*!
 * \brief Blocking read of gyroscope data
 * \param data Pointer to 3-vector where data should be stored
 */
void Fis1100_readGyroscopeData(float* data)
{
	uint8_t buffer[FIS1100_SAMPLE_SIZE];
	Fis1100_readRawGyroscopeData(buffer);
	Fis1100_processGyroscopeData(buffer, data);
}

/*!
 * \brief Blocking read of magnetometer data
 * \param data Pointer to 3-vector where data should be stored
 */
void Fis1100_readMagnetometerData(float* data)
{
	uint8_t buffer[FIS1100_SAMPLE_SIZE];
	Fis1100_readRawMagnetometerData(buffer);
	Fis1100_processMagnetometerData(buffer, data);
}

/*!
 * \brief Blocking read of AttitudeEngine data
 * \param orientationIncrement Pointer to orientation increment quaternion
 * (w,x,y,z)
 * \param velocityIncrement Pointer to velocity increment 3-vector
 * \param clipFlags Pointer to AttitudeEngine clipping flag structure
 */
void Fis1100_readAttitudeEngineData(float* orientationIncrement,
                                    float* velocityIncrement,
                                    uint16_t* clipFlags)
{
	uint8_t buffer[FIS1100_AE_SAMPLE_SIZE];
	Fis1100_readRawAttitudeEngineData(buffer);
	Fis1100_processAttitudeEngineData(buffer, orientationIncrement,
	                                  velocityIncrement, clipFlags);
}

/*!
 * \brief Request a Motion on Demand sample from the AttitudeEngine
 *
 * This function triggers a motion on demand request to the FIS1100 Attitude
 * Engine. The AttitudeEngine will trigger a data ready event (INT2) when the
 * data is available to be read. The data may be read as usual using the
 * Fis1100_readAttitudeEngineData() function.
 *
 * \note The AttitudeEngine must have been configured for Motion on Demand
 * mode to use this function.
 */
void Fis1100_requestAttitudeEngineData(void)
{
	doCtrl9Command(Ctrl9_TriggerMotionOnDemand);
}

/*!
 * \brief Apply scale factor calibration to raw data buffer
 * \param scaleFactor Scale factor to convert from LSB to calibrated units
 * \param nElements The number of data elements in the calibrated result
 * \param rawData Raw data buffer containing data read from the device
 * \param calibratedData Destination to store calibrated result
 *
 * \note Function assumes that input data is little endian \c int16_t data
 * arranged contiguously in a byte buffer. Therefore the \a rawData array should
 * contain 2x \a nElements bytes, and the \a calibratedData array should have
 * space for \a nElements floats.
 */
static inline void applyScaleFactor(float scaleFactor, uint8_t nElements,
                                    uint8_t const* rawData,
                                    float* calibratedData)
{
	for (int i = 0; i < nElements; ++i)
	{
		calibratedData[i] =
		    scaleFactor * (int16_t)((uint16_t)rawData[2 * i] |
		                            ((uint16_t)rawData[2 * i + 1] << 8));
	}
}

/*!
 * \brief Convert raw accelerometer data to calibrated data
 * \param rawData Pointer to raw data buffer containing data to convert
 * \param calibratedData Pointer to calibrated data output
 *
 * The \a rawData should be the 6 bytes of data read from the #FisRegister_Ax_L
 * to the #FisRegister_Az_H registers.
 *
 * \see Fis1100_readAccelerometerData() for an example of reading the necessary
 * data.
 */
void Fis1100_processAccelerometerData(uint8_t const* rawData,
                                      float* calibratedData)
{
	applyScaleFactor(g_fisDriverSettings.m_accScaleFactor, 3, rawData,
	                 calibratedData);
}

/*!
 * \brief Convert raw gyroscope data to calibrated data
 * \param rawData Pointer to raw data buffer containing data to convert
 * \param calibratedData Pointer to calibrated data output
 *
 * The \a rawData should be the 6 bytes of data read from the #FisRegister_Gx_L
 * to the #FisRegister_Gz_H registers.
 *
 * \see Fis1100_readGyroscopeData() for an example of reading the necessary
 *data.
 */
void Fis1100_processGyroscopeData(uint8_t const* rawData, float* calibratedData)
{
	applyScaleFactor(g_fisDriverSettings.m_gyrScaleFactor, 3, rawData,
	                 calibratedData);
}

/*!
 * \brief Convert raw magnetometer data to calibrated data
 * \param rawData Pointer to raw data buffer containing data to convert
 * \param calibratedData Pointer to calibrated data output
 *
 * The \a rawData should be the 6 bytes of data read from the #FisRegister_Mx_L
 * to the #FisRegister_Mz_H registers.
 *
 * \see Fis1100_readMagnetometerData() for an example of reading the necessary
 *data.
 */
void Fis1100_processMagnetometerData(uint8_t const* rawData,
                                     float* calibratedData)
{
	applyScaleFactor(g_fisDriverSettings.m_magScaleFactor, 3, rawData,
	                 calibratedData);
}

/*!
 * \brief Convert raw SDI data to calibrated data
 * \param rawData Pointer to raw data buffer containing data to convert
 * \param orientationIncrement Pointer to orientation increment quaternion
 *(w,x,y,z)
 * \param velocityIncrement Pointer to velocity increment 3-vector
 * \param clipFlags Pointer to AttitudeEngine clipping flags structure
 *
 * The \a rawData should be the 17 bytes of data read from the
 * #FisRegister_Q1_L to the #FisRegister_AeOverflow registers.
 *
 * \see Fis1100_readAttitudeEngineData() for an example of reading the
 * necessary data.
 *
 * \todo Support overflow flags
 */
void Fis1100_processAttitudeEngineData(uint8_t const* rawData,
                                       float* orientationIncrement,
                                       float* velocityIncrement,
                                       uint16_t* clipFlags)
{
	// orientationIncrement has 14 fractional bits
	const float orientationIncrementScale = 1.0f / (1 << 14);
	// velocityIncrement has 10 fractional bits
	const float velocityIncrementScale = 1.0f / (1 << 10);
	applyScaleFactor(orientationIncrementScale, 4, &rawData[0],
	                 orientationIncrement);
	applyScaleFactor(velocityIncrementScale, 3, &rawData[8], velocityIncrement);

	*clipFlags = rawData[15];
}

/*!
 * \brief Update the gyroscope bias estimate used by the AttitudeEngine
 * \param bias Pointer to the estimated bias data
 *
 * \note This function is designed for in run feedback from the XKF3 fusion
 * filter to the AttitudeEngine. To fix a large initial bias, also in IMU mode,
 * the Fis1100_applyGyroscopeOffset() function can be used instead.
 */
void Fis1100_updateAttitudeEngineGyroBias(float const* bias)
{
	const float conversionFactor = 180 / M_PI; // convert from rad/s to deg/s
	writeOffsetCalibrationBuffer(bias, conversionFactor, 4);
	doCtrl9Command(Ctrl9_UpdateAttitudeEngineGyroBias);
}

/*!
 * \brief Run accelerometer self test to verify that accelerometer operation.
 * \returns \c true if accelerometer self test passed, else \c false.
 */
bool Fis1100_accelerometerFunctional(void)
{
	Fis1100_enableSensors(FIS1100_CTRL7_DISABLE_ALL);
	while (g_fisDriverHal->int2Asserted())
	{
		// Ensure that INT2 is deasserted before continuing.
	}
	uint8_t selfTestCmd[] = {FisRegister_Ctrl2, 0x20};
	Fis1100_writeRegisters(selfTestCmd, sizeof(selfTestCmd));
	while (!g_fisDriverHal->int2Asserted())
	{
		// Wait for FIS1100 to signal self test complete by asserting INT2.
	}

	uint8_t selfTestData[FIS1100_SAMPLE_SIZE];
	Fis1100_readRegisters(FisRegister_Dvx_L, selfTestData, sizeof(selfTestData));

	selfTestCmd[1] = 0x00; // disable self test mode.
	Fis1100_writeRegisters(selfTestCmd, sizeof(selfTestCmd));
	while (g_fisDriverHal->int2Asserted())
	{
		// INT2 should now be deasserted.
	}

	// Self test uses 2g full scale
	const float scaleFactor = 1.0f / (1 << 14);
	float selfTestResponse[3];
	applyScaleFactor(scaleFactor, 3, selfTestData, selfTestResponse);

	bool selfTestPassed = true;
	const float selfTestPassThreshold = 50e-3f;
	for (int i = 0; i < 3; ++i)
	{
		selfTestPassed &= selfTestResponse[i] >= selfTestPassThreshold;
	}
	return selfTestPassed;
}

/*!
 * \brief Run gyroscope self test to verify that gyroscope operation.
 * \returns \c true if gyroscope self test passed, else \c false.
 */
bool Fis1100_gyroscopeFunctional(void)
{
	Fis1100_enableSensors(FIS1100_CTRL7_DISABLE_ALL);
	while (g_fisDriverHal->int2Asserted())
	{
		// Ensure that INT2 is deasserted before continuing.
	}
	uint8_t selfTestCmd[] = {FisRegister_Ctrl3, 0x40};
	Fis1100_writeRegisters(selfTestCmd, sizeof(selfTestCmd));
	while (!g_fisDriverHal->int2Asserted())
	{
		// Wait for FIS1100 to signal self test complete by asserting INT2.
	}

	uint8_t selfTestData[FIS1100_SAMPLE_SIZE];
	Fis1100_readRegisters(FisRegister_Dvx_L, selfTestData, sizeof(selfTestData));

	selfTestCmd[1] = 0x00; // disable self test mode.
	Fis1100_writeRegisters(selfTestCmd, sizeof(selfTestCmd));
	while (g_fisDriverHal->int2Asserted())
	{
		// INT2 should now be deasserted.
	}

	// Self test uses 2560dps full scale
	const float scaleFactor = 1.0f / (1 << 3);
	float selfTestResponse[3];
	applyScaleFactor(scaleFactor, 3, selfTestData, selfTestResponse);

	bool selfTestPassed = true;
	const float selfTestPassThreshold = 300;
	for (int i = 0; i < 3; ++i)
	{
		selfTestPassed &= selfTestResponse[i] >= selfTestPassThreshold;
	}
	return selfTestPassed;
}
