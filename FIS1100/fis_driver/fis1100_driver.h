/*!
 * \file fis1100_driver.h
 * \version 1.0.1 rev. 43800
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

#ifndef __FIS1100_DRIVER_H
#define __FIS1100_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "fis1100_hal.h"

/*!
 * \defgroup FISDRIVER FIS1100 High Level Driver
 * \brief High level driver for the FIS1100
 *
 * The FIS1100 driver provides a high level C99 interface to the FIS1100 hardware.
 * This interface can be used to intialize the sensors, configure their operation,
 * and to read back calibrated data.
 *
 * The driver uses a \ref Fis1100Hal "hardware abstraction interface" to abstract the low level
 * platform specific functions required to communicate with the FIS1100 via I2C
 * or SPI.
 *
 * In typical operation the INT2 interrupt from the FIS1100 should be configured to
 * trigger an MCU interrupt handler to fetch new data from the hardware.
 *
 * Example:
 * \code
 * static struct Fis1100Hal fisHal;
 * static volatile bool fisDataReady;
 *
 * void INT2_IRQHandler(void)
 * {
 *     fisDataReady = true;
 * }
 *
 * void main(void)
 * {
 *     // Setup hardware, interrupts and FIS1100 HAL functions...
 *     configureHardware();
 *
 *     Fis1100_init(&fisHal);
 *     Fis1100_configureAccelerometer(AccRange_8g, AccOdr_256Hz, AccUnit_g, Lpf_Enable);
 *     Fis1100_configureGyroscope(GyrRange_2048dps, GyrOdr_256Hz, GyrUnit_dps, Lpf_Enable);
 *     Fis1100_enableSensors(FIS1100_CTRL7_ACC_ENABLE | FIS1100_CTRL7_GYR_ENABLE);
 *
 *     for (;;)
 *     {
 *         if (fisDataReady)
 *         {
 *              float acc[3];
 *              float gyr[3];
 *              Fis1100_readAccelerometerData(acc);
 *              Fis1100_readGyroscopeData(gyr);
 *              handleAccGyrData(acc, gyr);
 *         }
 *     }
 * }
 * \endcode
 */

/*!
 * \brief FIS1100 I2C slave address.
 * \ingroup FISDRIVER
 */
#define FIS1100_I2C_SLAVE_ADDRESS (0x6a)

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \defgroup FIS_CONFIGURATION FIS1100 Driver initialization and configuration
 * \ingroup FISDRIVER
 * \brief Initialization and configuration of the FIS1100 driver and hardware.
 *
 * The FIS1100 driver must be initialized before use to configure internal
 * data structures, and to ensure that the FIS1100 hardware is in a known
 * state.
 * \{
 */

/*!
 * \defgroup FIS_CTRL5 Ctrl5 register control bits.
 * \ingroup FIS_CONFIGURATION
 * \{
 */
/*! \brief Accelerometer high pass filter enable */
#define FIS1100_CTRL5_ACC_HPF_ENABLE	(0x01)
/*! \brief Accelerometer low pass filter enable */
#define FIS1100_CTRL5_ACC_LPF_ENABLE	(0x02)
/*! \brief Gyroscope high pass filter enable */
#define FIS1100_CTRL5_GYR_HPF_ENABLE	(0x04)
/*! \brief Gyroscope low pass filter enable */
#define FIS1100_CTRL5_GYR_LPF_ENABLE	(0x08)
/*! \brief Configure gyroscope high pass filter corner frequency */
#define FIS1100_CTRL5_GYR_HPF01_ENABLE	(0x10)
/*! \} */

/*!
 * \defgroup FIS_CTRL7 Ctrl7 register control bits.
 * \ingroup FIS_CONFIGURATION
 * \{
 */
/*! \brief Disable all sensors */
#define FIS1100_CTRL7_DISABLE_ALL (0x0)
/*! \brief Accelerometer enable */
#define FIS1100_CTRL7_ACC_ENABLE (0x1)
/*! \brief Gyroscope enable */
#define FIS1100_CTRL7_GYR_ENABLE (0x2)
/*! \brief Magnetometer enable */
#define FIS1100_CTRL7_MAG_ENABLE (0x4)
/*! \brief AttitudeEngine enable */
#define FIS1100_CTRL7_AE_ENABLE (0x8)
/*! \brief Mask for sensor enable bits */
#define FIS1100_CTRL7_ENABLE_MASK	(0xF)
 /*! \} */

void Fis1100_reset(void);
void Fis1100_init(struct Fis1100Hal const* halFunctions);

/*!
 * \brief Low pass filter configuration
 */
enum Fis1100_LpfConfig
{
	Lpf_Disable, /*!< \brief Disable low pass filter */
	Lpf_Enable   /*!< \brief Enable low pass filter */
};

/*!
 * \name Accelerometer configuration
 * \{
 */
/*!
 * \brief Accelerometer dynamic range configuration
 */
enum Fis1100_AccRange
{
	AccRange_2g = 0 << 3, /*!< \brief +/- 2g range */
	AccRange_4g = 1 << 3, /*!< \brief +/- 4g range */
	AccRange_8g = 2 << 3  /*!< \brief +/- 8g range */
};

/*!
 * \brief Accelerometer output data rate
 */
enum Fis1100_AccOdr
{
	AccOdr_1024Hz = 0,  /*!< \brief High resolution 1024Hz output rate */
	AccOdr_256Hz = 1, /*!< \brief High resolution 256Hz output rate */
	AccOdr_128Hz = 2, /*!< \brief High resolution 128Hz output rate */
	AccOdr_32Hz = 3,  /*!< \brief High resolution 32Hz output rate */
	AccOdr_LowPower_128Hz = 4, /*!< \brief Low power 128Hz output rate */
	AccOdr_LowPower_64Hz = 5,  /*!< \brief Low power 64Hz output rate */
	AccOdr_LowPower_25Hz = 6,  /*!< \brief Low power 25Hz output rate */
	AccOdr_LowPower_3Hz = 7    /*!< \brief Low power 3Hz output rate */
};

/*!
 * \brief Convert CTRL2 accelerometer ODR configuration to ODR in Hz
 */
static inline uint16_t Fis1100_accOdr(enum Fis1100_AccOdr odr)
{
	switch (odr)
	{
		case AccOdr_1024Hz: return 1024;
		case AccOdr_256Hz: return 256;
		case AccOdr_128Hz: return 128;
		case AccOdr_32Hz: return 32;
		case AccOdr_LowPower_128Hz: return 128;
		case AccOdr_LowPower_64Hz: return 64;
		case AccOdr_LowPower_25Hz: return 25;
		case AccOdr_LowPower_3Hz: return 3;
	}
	return 0;
}

/*!
 * \brief Accelerometer output units
 */
enum Fis1100_AccUnit
{
	AccUnit_g,  /*!< \brief Accelerometer output in terms of g (9.81m/s^2)*/
	AccUnit_ms2 /*!< \brief Accelerometer output in terms of m/s^2 */
};

void Fis1100_configureAccelerometer(enum Fis1100_AccRange range,
                                    enum Fis1100_AccOdr odr,
                                    enum Fis1100_AccUnit unit,
                                    enum Fis1100_LpfConfig lpfEnable);
/*! \} */

/*!
 * \name Gyroscope configuration
 * \{
 */

/*!
 * \brief Gyroscope dynamic range configuration
 */
enum Fis1100_GyrRange
{
	GyrRange_32dps = 0 << 3,   /*!< \brief +-32 degrees per second */
	GyrRange_64dps = 1 << 3,   /*!< \brief +-64 degrees per second */
	GyrRange_128dps = 2 << 3,  /*!< \brief +-128 degrees per second */
	GyrRange_256dps = 3 << 3,  /*!< \brief +-256 degrees per second */
	GyrRange_512dps = 4 << 3,  /*!< \brief +-512 degrees per second */
	GyrRange_1024dps = 5 << 3, /*!< \brief +-1024 degrees per second */
	GyrRange_2048dps = 6 << 3, /*!< \brief +-2048 degrees per second */
	GyrRange_2560dps = 7 << 3  /*!< \brief +-2560 degrees per second */
};

/*!
 * \brief Gyroscope output rate configuration
 */
enum Fis1100_GyrOdr
{
	GyrOdr_1024Hz = 0,  /*!< \brief High resolution 1024Hz output rate */
	GyrOdr_256Hz = 1, /*!< \brief High resolution 256Hz output rate */
	GyrOdr_128Hz = 2, /*!< \brief High resolution 128Hz output rate */
	GyrOdr_32Hz = 3,   /*!< \brief High resolution 32Hz output rate */
	GyrOdr_8192Hz = 6,
	GyrOdr_8192Hz_LP = 7
};

/*!
 * \brief Convert  CTRL3 gyroscope ODR setting to ODR in Hz
 */
static inline uint16_t Fis1100_gyrOdr(enum Fis1100_GyrOdr odrSetting)
{
	switch (odrSetting)
	{
		case GyrOdr_1024Hz: return 1024;
		case GyrOdr_256Hz: return 256;
		case GyrOdr_128Hz: return 128;
		case GyrOdr_32Hz: return 32;
	}
	return 0;
}

/*!
 * \brief Gyroscope output units
 */
enum Fis1100_GyrUnit
{
	GyrUnit_dps, /*!< \brief Gyroscope output in degrees/s */
	GyrUnit_rads /*!< \brief Gyroscope output in rad/s */
};

void Fis1100_configureGyroscope(enum Fis1100_GyrRange range,
                                enum Fis1100_GyrOdr odr,
                                enum Fis1100_GyrUnit unit,
                                enum Fis1100_LpfConfig lpfEnable);

/*! \} */

/*!
 * \name Magnetometer configuration
 * \{
 */
/*!
 * \brief Magnetometer slave device type
 */
enum Fis1100_MagDev
{
	MagDev_AK8975 = (0 << 4) /*!< \brief AKM AK8975  */
};

/*!
 * \brief Magnetometer output rate configuration
 */
enum Fis1100_MagOdr
{
	MagOdr_32Hz = 2   /*!< \brief 32Hz output rate */
};

/*!
 * \brief Convert CTRL4 magnetometer ODR settings to ODR in Hz
 */
static inline uint16_t Fis1100_magOdr(enum Fis1100_MagOdr odrSetting)
{
	switch (odrSetting)
	{
		case MagOdr_32Hz: return 32;
	}
	return 0;
}

/*!
 * \brief Magnetometer output units
 */
enum Fis1100_MagUnit
{
	MagUnit_gauss,     /*!< \brief Magnetometer output in Gauss */
	MagUnit_microTesla /*!< \brief Magnetometer output in micro-Tesla */
};
void Fis1100_configureMagnetometer(enum Fis1100_MagDev device,
                                   enum Fis1100_MagOdr odr,
                                   enum Fis1100_MagUnit unit);

/*! \} */

/*!
 * \name AttitudeEngine configuration
 * \{
 */
/*!
 * \brief AttitudeEngine output data rate configuration
 */
enum Fis1100_AeOdr
{
	AeOdr_1Hz = 0,  /*!< \brief 1Hz output rate */
	AeOdr_2Hz = 1,  /*!< \brief 2Hz output rate */
	AeOdr_4Hz = 2,  /*!< \brief 4Hz output rate */
	AeOdr_8Hz = 3,  /*!< \brief 8Hz output rate */
	AeOdr_16Hz = 4, /*!< \brief 16Hz output rate */
	AeOdr_32Hz = 5, /*!< \brief 32Hz output rate */
	AeOdr_64Hz = 6,  /*!< \brief 64Hz output rate */
	/*!
	 * \brief Motion on demand mode
	 *
	 * In motion on demand mode the application can trigger AttitudeEngine
	 * output samples as necessary. This allows the AttitudeEngine to be
	 * synchronized with external data sources.
	 *
	 * When in Motion on Demand mode the application should request new data
	 * by calling the Fis1100_requestAttitudeEngineData() function. The
	 * AttitudeEngine will respond with a data ready event (INT2) when the
	 * data is available to be read.
	 */
	AeOdr_motionOnDemand = 128
};

/*!
 * \brief Convert CTRL6 AttitudeEngine ODR setting into ODR in Hz
 */
static inline uint16_t Fis1100_aeOdr(enum Fis1100_AeOdr odr)
{
	switch (odr)
	{
		case AeOdr_64Hz: return 64;
		case AeOdr_32Hz: return 32;
		case AeOdr_16Hz: return 16;
		case AeOdr_8Hz: return 8;
		case AeOdr_4Hz: return 4;
		case AeOdr_2Hz: return 2;
		case AeOdr_1Hz: return 1;
		case AeOdr_motionOnDemand: return 1024;
	}
	return 0;
}
void Fis1100_configureAttitudeEngine(enum Fis1100_AeOdr odr);
/*! \} */

/*!
 * \name Sensor power control
 * \{
 */
void Fis1100_enableSensors(uint8_t enableFlags);
void Fis1100_enableAccelerometer(bool enable);
void Fis1100_enableGyroscope(bool enable);
void Fis1100_enableMagnetometer(bool enable);
void Fis1100_enableAttitudeEngine(bool enable);
/*! \} */
/*! \} */

/*!
 * \defgroup REGISTER_ACCESS Low level register access
 * \ingroup FISDRIVER
 * \brief Low level functions to read and write FIS1100 registers.
 *
 * These functions provide a mechanism to read and write data to the FIS1100
 * device registers. The functions automate the setting of the auto-increment
 * and read bits as required by the configured IO interface used by the driver.
 * \{
 */
/*!
 * \brief FIS1100 Register mapping
 */
enum Fis1100Register
{
	/*! \brief FIS1100 device identifier register */
	FisRegister_WhoAmI,
	/*! \brief General and power management modes */
	FisRegister_Ctrl1 = 2,
	/*! \brief Accelerometer control */
	FisRegister_Ctrl2,
	/*! \brief Gyroscope control */
	FisRegister_Ctrl3,
	/*! \brief Magnetometer control */
	FisRegister_Ctrl4,
	/*! \brief Data processing settings */
	FisRegister_Ctrl5,
	/*! \brief AttitudeEngine control */
	FisRegister_Ctrl6,
	/*! \brief Sensor enabled status */
	FisRegister_Ctrl7,
	/*! \brief Reserved - do not write */
	FisRegister_Ctrl8,
	/*! \brief Host command register */
	FisRegister_Ctrl9,
	/*! \brief Calibration register 1 least significant byte */
	FisRegister_Cal1_L,
	/*! \brief Calibration register 1 most significant byte */
	FisRegister_Cal1_H,
	/*! \brief Calibration register 2 least significant byte */
	FisRegister_Cal2_L,
	/*! \brief Calibration register 2 most significant byte */
	FisRegister_Cal2_H,
	/*! \brief Calibration register 3 least significant byte */
	FisRegister_Cal3_L,
	/*! \brief Calibration register 3 most significant byte */
	FisRegister_Cal3_H,
	/*! \brief Calibration register 4 least significant byte */
	FisRegister_Cal4_L,
	/*! \brief Calibration register 4 most significant byte */
	FisRegister_Cal4_H,
	/*! \brief FIFO control register */
	FisRegister_FifoCtrl,
	/*! \brief FIFO data register */
	FisRegister_FifoData,
	/*! \brief FIFO status register */
	FisRegister_FifoStatus,
	/*! \brief Output data overrun and availability */
	FisRegister_Status0,
	/*! \brief Miscellaneous status register */
	FisRegister_Status1,
	/*! \brief Sample counter */
	FisRegister_CountOut,
	/*! \brief Accelerometer X axis least significant byte */
	FisRegister_Ax_L,
	/*! \brief Accelerometer X axis most significant byte */
	FisRegister_Ax_H,
	/*! \brief Accelerometer Y axis least significant byte */
	FisRegister_Ay_L,
	/*! \brief Accelerometer Y axis most significant byte */
	FisRegister_Ay_H,
	/*! \brief Accelerometer Z axis least significant byte */
	FisRegister_Az_L,
	/*! \brief Accelerometer Z axis most significant byte */
	FisRegister_Az_H,
	/*! \brief Gyroscope X axis least significant byte */
	FisRegister_Gx_L,
	/*! \brief Gyroscope X axis most significant byte */
	FisRegister_Gx_H,
	/*! \brief Gyroscope Y axis least significant byte */
	FisRegister_Gy_L,
	/*! \brief Gyroscope Y axis most significant byte */
	FisRegister_Gy_H,
	/*! \brief Gyroscope Z axis least significant byte */
	FisRegister_Gz_L,
	/*! \brief Gyroscope Z axis most significant byte */
	FisRegister_Gz_H,
	/*! \brief Magnetometer X axis least significant byte */
	FisRegister_Mx_L,
	/*! \brief Magnetometer X axis most significant byte */
	FisRegister_Mx_H,
	/*! \brief Magnetometer Y axis least significant byte */
	FisRegister_My_L,
	/*! \brief Magnetometer Y axis most significant byte */
	FisRegister_My_H,
	/*! \brief Magnetometer Z axis least significant byte */
	FisRegister_Mz_L,
	/*! \brief Magnetometer Z axis most significant byte */
	FisRegister_Mz_H,
	/*! \brief Quaternion increment W least significant byte */
	FisRegister_Q1_L = 45,
	/*! \brief Quaternion increment W most significant byte */
	FisRegister_Q1_H,
	/*! \brief Quaternion increment X least significant byte */
	FisRegister_Q2_L,
	/*! \brief Quaternion increment X most significant byte */
	FisRegister_Q2_H,
	/*! \brief Quaternion increment Y least significant byte */
	FisRegister_Q3_L,
	/*! \brief Quaternion increment Y most significant byte */
	FisRegister_Q3_H,
	/*! \brief Quaternion increment Z least significant byte */
	FisRegister_Q4_L,
	/*! \brief Quaternion increment Z most significant byte */
	FisRegister_Q4_H,
	/*! \brief Velocity increment X least significant byte */
	FisRegister_Dvx_L,
	/*! \brief Velocity increment X most significant byte */
	FisRegister_Dvx_H,
	/*! \brief Velocity increment Y least significant byte */
	FisRegister_Dvy_L,
	/*! \brief Velocity increment Y most significant byte */
	FisRegister_Dvy_H,
	/*! \brief Velocity increment Z least significant byte */
	FisRegister_Dvz_L,
	/*! \brief Velocity increment Z most significant byte */
	FisRegister_Dvz_H,
	/*! \brief Temperature output */
	FisRegister_Temperature,
	/*! \brief AttitudeEngine clipping flags */
	FisRegister_AeClipping,
	/*! \brief AttitudeEngine overflow flags */
	FisRegister_AeOverflow,
};
void Fis1100_writeRegisters(uint8_t* data, uint8_t dataLength);
void Fis1100_readRegisters(uint8_t address, uint8_t* data, uint8_t dataLength);
/*! \} */

/*!
 * \defgroup READING_FIS_DATA Reading sensor data
 * \ingroup FISDRIVER
 * \brief Reading sensor data from the FIS1100 output registers
 * \{
 */

/*! \brief The size of a FIS1100 accelerometer, gyroscope or magnetometer sample in bytes */
#define FIS1100_SAMPLE_SIZE (3 * sizeof(int16_t))
/*! \brief The size of a FIS1100 AttitudeEngine sample in bytes */
#define FIS1100_AE_SAMPLE_SIZE ((4+3+1) * sizeof(int16_t) + sizeof(uint8_t))

/*!
 * \defgroup STATUS0 Status register 0
 * \ingroup READING_FIS_DATA
 * \brief Status register 0 bits.
 * \{
 */
/*! \brief Accelerometer data available */
#define FIS1100_STATUS0_ACC_DA	(0x01)
/*! \brief Gyroscope data available */
#define FIS1100_STATUS0_GYR_DA (0x02)
/*! \brief Magnetometer data available */
#define FIS1100_STATUS0_MAG_DA	(0x04)
/*! \brief AttitudeEngine data available */
#define FIS1100_STATUS0_AE_DA	(0x08)
/*! \brief Accelerometer data overrun */
#define FIS1100_STATUS0_ACC_OVRN	(0x10)
/*! \brief Gyroscope data overrun */
#define FIS1100_STATUS0_GYR_OVRN (0x20)
/*! \brief Magnetometer data overrun */
#define FIS1100_STATUS0_MAG_OVRN	(0x40)
/*! \brief AttitudeEngine data overrun */
#define FIS1100_STATUS0_AE_OVRN	(0x80)
 /*! \} */

/*!
 * \defgroup AE_CLIPPING_BITS AttitudeEngine clipping status flags
 * \ingroup READING_FIS_DATA
 * \brief AttitudeEngine clipping and status flag bits.
 * \{
 */
/*! \brief Accelerometer X axis data was clipped during dQ calculation */
#define FIS1100_AE_AX_CLIPPED	(0x01)
/*! \brief Accelerometer Y axis data was clipped during dQ calculation */
#define FIS1100_AE_AY_CLIPPED	(0x02)
/*! \brief Accelerometer Z axis data was clipped during dQ calculation */
#define FIS1100_AE_AZ_CLIPPED	(0x04)
/*! \brief Gyroscope X axis data was clipped during dQ calculation */
#define FIS1100_AE_GX_CLIPPED	(0x08)
/*! \brief Gyroscope Y axis data was clipped during dQ calculation */
#define FIS1100_AE_GY_CLIPPED	(0x10)
/*! \brief Gyroscope Z axis data was clipped during dQ calculation */
#define FIS1100_AE_GZ_CLIPPED	(0x20)
/*! \brief Acknowledgement that gyroscope bias was updated during the last interval*/
#define FIS1100_AE_GYRO_BIAS_ACK	(0x40)
/*! \} */

uint8_t Fis1100_readStatus0(void);
uint8_t Fis1100_readStatus1(void);
uint8_t Fis1100_readCount(void);
void Fis1100_requestAttitudeEngineData(void);

/*!
 * \name Raw data reading functions
 * \brief Blocking functions to read raw data from the FIS1100 output registers into
 * supplied buffers.
 *
 * The data buffers read by these functions can be processed by one of the \ref
 * FIS_PROCESSING_FUNCTIONS "raw data processing" functions.
 *
 * \remark These functions use the blocking read functions provided by the HAL.
 * It is recommended that these functions only be used if combined with a RTOS
 * that allows the MCU to perform other operations, or sleep, until the
 * transfer is completed. When not using a RTOS it is recommended to implement
 * non-blocking register reads using interrupts or DMA transfers instead of
 * using these functions.
 */
/*!
 * \brief Read raw accelerometer data
 * \param buffer Pointer to data buffer to read raw data into, should have at
 * least #FIS1100_SAMPLE_SIZE bytes available.
 *
 * The buffer contents can be converted to floating point values using the
 * Fis1100_processAccelerometerData() function.
 */
static inline void Fis1100_readRawAccelerometerData(uint8_t* buffer)
{
	Fis1100_readRegisters(FisRegister_Ax_L, buffer, FIS1100_SAMPLE_SIZE);
}

/*!
 * \brief Read raw gyroscope data
 * \param buffer Pointer to data buffer to read raw data into, should have at
 * least #FIS1100_SAMPLE_SIZE bytes available.
 *
 * The buffer contents can be converted to floating point values using the
 * Fis1100_processGyroscopeData() function.
 */
static inline void Fis1100_readRawGyroscopeData(uint8_t* buffer)
{
	Fis1100_readRegisters(FisRegister_Gx_L, buffer, FIS1100_SAMPLE_SIZE);
}

static inline void Fis1100_readRawOisGyroscopeData(uint8_t* buffer)
{
	Fis1100_readRegisters(FisRegister_Dvx_L, buffer, FIS1100_SAMPLE_SIZE);
}


/*!
 * \brief Read raw magnetometer data
 * \param buffer Pointer to data buffer to read raw data into, should have at
 * least #FIS1100_SAMPLE_SIZE bytes available.
 *
 * The buffer contents can be converted to floating point values using the
 * Fis1100_processMagnetometerData() function.
 */
static inline void Fis1100_readRawMagnetometerData(uint8_t* buffer)
{
	Fis1100_readRegisters(FisRegister_Mx_L, buffer, FIS1100_SAMPLE_SIZE);
}

/*!
 * \brief Read raw AttitudeEngine data
 * \param buffer Pointer to data buffer to read raw data into, should have at
 * least #FIS1100_AE_SAMPLE_SIZE bytes available.
 *
 * The buffer contents can be converted to floating point values using the
 * Fis1100_processAttitudeEngineData() function.
 */
static inline void Fis1100_readRawAttitudeEngineData(uint8_t* buffer)
{
	Fis1100_readRegisters(FisRegister_Q1_L, buffer, FIS1100_AE_SAMPLE_SIZE);
}
/*! \} */

/*!
 * \anchor FIS_PROCESSING_FUNCTIONS
 * \name Raw data processing functions
 * \brief Functions to convert raw data buffers into calibrated data.
 *
 * These functions will use the dynamic range and unit selected when the sensor
 * was configured to apply the necessary scaling factor.
 * \{
 */

void Fis1100_processAccelerometerData(uint8_t const* rawData,
                                      float* calibratedData);
void Fis1100_processGyroscopeData(uint8_t const* rawData,
                                  float* calibratedData);
void Fis1100_processMagnetometerData(uint8_t const* rawData,
                                     float* calibratedData);
void Fis1100_processAttitudeEngineData(uint8_t const* rawData,
                                       float* orientationIncrement,
                                       float* velocityIncrement,
                                       uint16_t* flags);
/*! \} */
/*!
 * \name Calibrated data reading functions
 * \brief Convenience functions to read calibrated data.
 *
 * Raw data is first read into a temporary buffer on the stack which is then
 * processed to provide calibrated data.

 * \remark These functions use the blocking read functions provided by the HAL.
 * It is recommended that these functions only be used if combined with a RTOS
 * that allows the MCU to perform other operations, or sleep, until the
 * transfer is completed. When not using a RTOS it is recommended to implement
 * non-blocking register reads using interrupts or DMA transfers instead of
 * using these functions.
 */
void Fis1100_readAccelerometerData(float* data);
void Fis1100_readGyroscopeData(float* data);
void Fis1100_readMagnetometerData(float* data);
void Fis1100_readAttitudeEngineData(float* orientationIncrement,
                                    float* velocityIncrement, uint16_t* flags);
/*! \} */
/*! \} */

/*!
 * \defgroup FIFO_ACCESS Using the FIFO to reduce interrupts
 * \ingroup FISDRIVER
 * \brief Configuration and access to the FIS1100 sample FIFO
 *
 * The FIS1100 FIFO can be used to group accelerometer, gyroscope and
 * magnetometer data for efficient data transfer to the host MCU. Sample data
 * for each of the enabled sensors is written to the FIFO which triggers a
 * data ready (INT2) event when a configurable watermark level is reached.
 *
 * Using the FIFO is recommended when high rate sensor data is required for
 * processing by user algorithms. For applications that require orientation
 * tracking it is recommended to use the AttitudeEngine instead of the FIFO as
 * this provides greater power reductions without compromising on sensor fusion
 * accuracy.
 *
 * The FIS1100 FIFO must be locked for read access, during which time
 * the FIS1100 cannot write new data to the FIFO. After reading data the FIFO
 * must be unlocked.
 *
 * \note If the host locks the FIFO for longer than one FIS1100 ODR period then
 * data will be lost as the FIS1100 cannot access the FIFO. It is important to
 * profile implementation timing to prevent this. This may limit the maximum
 * size of FIFO usable in an application.
 *
 * \note When using the FIFO with the accelerometer and gyroscope enabled the
 * ODRs for the sensors must be the same. When using the magnetometer the
 * magnetometer data samples will be repeated to match the
 * accelerometer/gyroscope ODR.
 *
 * Example:
 * \code
 * static struct Fis1100Hal fisHal;
 * static volatile bool fifoDataReady;
 *
 * void INT2_IRQHandler(void)
 * {
 *     fifoDataReady = true;
 * }
 *
 * void main(void)
 * {
 *     // Setup hardware, interrupts and FIS1100 HAL functions...
 *     configureHardware();
 *
 *     Fis1100_init(&fisHal);
 *     Fis1100_configureAccelerometer(AccRange_8g, AccOdr_256Hz, AccUnit_g, Lpf_Enable);
 *     Fis1100_configureGyroscope(GyrRange_2048dps, GyrOdr_256Hz, GyrUnit_dps, Lpf_Enable);
 *
 *     // Configure FIFO to interrupt after 8 interleaved acc and gyro samples
 *     // This will reduce the FIS1100 interrupt rate from 256Hz to 64Hz
 *     uint32_t fifoSize = Fis1100_configureFifoWatermark(8);
 *     uint8_t* fifoBuffer = malloc(bufSize);
 *     fifoDataReady = false;
 *
 *     Fis1100_enableSensors(FIS1100_CTRL7_ACC_ENABLE | FIS1100_CTRL7_GYR_ENABLE);
 *
 *     for (;;)
 *     {
 *          if (fifoDataReady)
 *          {
 *               Fis1100_lockFifo();
 *               Fis1100_readFifo(fifoBuffer, fifoSize);
 *               Fis1100_unlockFifo();
 *               fifoDataReady = false;
 *               uint8_t* dptr = fifoBuffer;
 *               for (int i = 0; i < 4; ++i)
 *               {
 *                   float acc[3];
 *                   float gyr[3];
 *                   Fis1100_processAccelerometerData(dptr, acc);
 *                   dptr += FIS1100_SAMPLE_SIZE;
 *                   Fis1100_processGyroscopeData(dptr, gyr);
 *                   dptr += FIS1100_SAMPLE_SIZE;
 *                   // do something with the data...
 *                   handleAccGyrData(acc, gyr);
 *               }
 *          }
 *     }
 * }
 * \endcode
 * \{
 */

/*!
 * \brief FIFO watermark level configuration
 *
 * The INT2 interrupt line is asserted when the FIFO is filled beyond the
 * watermark level.
 */
enum Fis1100_FifoWatermarkLevel
{
	/*! \brief Watermark at 0 */
	Fifo_WatermarkEmpty = (0 << 4),
	/*! \brief Watermark at 1/4 full */
	Fifo_WatermarkOneQuarter = (1 << 4),
	/*! \brief Watermark at 1/2 full */
	Fifo_WatermarkHalf = (2 << 4),
	/*! \brief Watermark at 3/4 full */
	Fifo_WatermarkThreeQuarters = (3 << 4)
};

/*!
 * \brief FIFO size configuration
 */
enum Fis1100_FifoSize
{
	FifoSize_16 = (0 << 2), /*!< \brief 16 sample FIFO */
	FifoSize_32 = (1 << 2), /*!< \brief 32 sample FIFO */
	FifoSize_64 = (2 << 2), /*!< \brief 64 sample FIFO */
	FifoSize_128 = (3 << 2) /*!< \brief 128 sample FIFO */
};

enum Fis1100_FifoMode
{
	/*! \brief Disable FIFO operation (Bypass mode) */
	FifoMode_Disable = 0,
	/*!
     * \brief FIFO mode.
     * \todo Document FIFO mode
     */
	FifoMode_Fifo = 1,
	/*!
     * \brief Stream mode
     * FIFO acts as a circular buffer
     */
	FifoMode_Stream = 2
};

uint32_t Fis1100_configureFifo(enum Fis1100_FifoWatermarkLevel watermark,
                               enum Fis1100_FifoSize size,
                               enum Fis1100_FifoMode mode);

/*!
 * \brief Disable the FIS1100 FIFO
 * \returns The number of samples to trigger a FIFO watermark interrupt, 0 by
 * definition.
 */
static inline uint32_t Fis1100_disableFifo(void)
{
	return Fis1100_configureFifo(Fifo_WatermarkEmpty, FifoSize_16, FifoMode_Disable);
}

uint32_t Fis1100_configureFifoSamples(uint32_t samples);
void Fis1100_lockFifo(void);
void Fis1100_readFifo(uint8_t* data, uint8_t dataLength);
void Fis1100_unlockFifo(void);
/*! \} */


/*!
 * \defgroup FIS_WAKE_ON_MOTION Wake on motion
 * \ingroup FISDRIVER
 * \brief Configuring wake on motion and checking for motion events
 *
 * The Wake on Motion (WoM) function puts the FIS1100 in a very low power state
 * that monitors for motion and sends an interrupt to the host MCU. This mode
 * can be used to implement a low power standby mode that is automatically exited
 * when a user interacts with the device.
 *
 * \{
 */
/*!
 * \defgroup STATUS1 Status register 1
 * \ingroup READING_FIS_DATA
 * \brief Status register 1 bits.
 * \{
 */
/*! \brief Ctrl9 command complete */
#define FIS1100_STATUS1_CMD_DONE (0x01)
/*! \brief Indicates a wakeup event has occurred */
#define FIS1100_STATUS1_WAKEUP_EVENT (0x04)

/*! \} */
/*!
 * \brief FIS1100 Interrupt line selection
 */
enum Fis1100_Interrupt
{
	Interrupt_Int1 = (0 << 6), /*!< Interrupt 1 */
	Interrupt_Int2 = (1 << 6)  /*!< Interrupt 2 */
};

enum Fis1100_InterruptInitialState
{
	InterruptInitialState_high = (1 << 7), /*!< Interrupt high */
	InterruptInitialState_low  = (0 << 7)  /*!< Interrupt low */
};

enum Fis1100_WakeOnMotionThreshold
{
	WomThreshold_high = 128, /*!< High threshold - large motion needed to wake */
	WomThreshold_low  = 32   /*!< Low threshold - small motion needed to wake */
};

void Fis1100_enableWakeOnMotion(enum Fis1100_Interrupt interrupt,
		enum Fis1100_InterruptInitialState initialState,
		enum Fis1100_WakeOnMotionThreshold threshold,
		uint8_t blankingTime);
bool Fis1100_hasWakeupEventOccured(void);
void Fis1100_disableWakeOnMotion(void);
/*! \} */

/*!
 * \defgroup FIS_CALIBRATION_FUNCTIONS Calibration functions
 * \ingroup FISDRIVER
 * \brief Functions to apply board level and in-run calibration parameters
 * \{
 */
void Fis1100_applyAccelerometerOffset(float const* offset, enum Fis1100_AccUnit unit);
void Fis1100_applyGyroscopeOffset(float const* offset, enum Fis1100_GyrUnit unit);
/*!
 * \brief Offset calibration structure.
 */
struct Fis1100_offsetCalibration
{
	/*! \brief Unit used for accelerometer offset. */
	enum Fis1100_AccUnit accUnit;
	/*! \brief Acceleremeter offset vector.*/
	float accOffset[3];
	/*! \brief Unit used for gyroscope offset. */
	enum Fis1100_GyrUnit gyrUnit;
	/*! \brief Gyroscope offset vector. */
	float gyrOffset[3];
};
/*!
 * \brief Apply calibration parameters from \a cal.
 */
static inline void Fis1100_applyOffsetCalibration(struct Fis1100_offsetCalibration const* cal)
{
	Fis1100_applyAccelerometerOffset(cal->accOffset, cal->accUnit);
	Fis1100_applyGyroscopeOffset(cal->gyrOffset, cal->gyrUnit);
}

void Fis1100_updateAttitudeEngineGyroBias(float const* bias);
/*! \} */

/*!
 * \defgroup FIS_SELFTEST_FUNCTIONS Self test functions.
 * \ingroup FISDRIVER
 * \brief Functions to run self test of accelerometer and gyroscope function.
 * \{
 */
bool Fis1100_accelerometerFunctional(void);
bool Fis1100_gyroscopeFunctional(void);
/*! \} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // __FIS1100_DRIVER_H
