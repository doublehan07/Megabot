/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Continuous wave mode example code
 *
 *           This example code activates continuous wave mode on channel 5 for 2 minutes before stopping operation.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "lcd.h"
#include "port.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "CONT WAVE v1.1"

/* Continuous wave duration, in milliseconds. */
#define CONT_WAVE_DURATION_MS 120000

/* Default communication configuration. Use channel 5 in this example as it is the recommended channel for crystal trimming operation. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Recommended TX power and Pulse Generator delay values for the mode defined above. */
static dwt_txconfig_t txconfig = {
    0xC0,            /* PG delay. */
    0x25456585,      /* TX power. */
};

/**
 * Application entry point.
 */
int main(void)
{
    /* Start with board specific hardware init. */
    peripherals_init();

    /* Display application name on LCD. */
    lcd_display_str(APP_NAME);

    /* During initialisation and continuous wave mode activation, DW1000 clocks must be set to crystal speed so SPI rate have to be lowered and will
     * not be increased again in this example. */
    spi_set_rate_low();

    /* Reset and initialise DW1000. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    dwt_initialise(DWT_LOADNONE);

    /* Configure DW1000. */
    dwt_configure(&config);
    dwt_configuretxrf(&txconfig);

    /* Activate continuous wave mode. */
    dwt_configcwmode(config.chan);

    /* Wait for the wanted duration of the continuous wave transmission. */
    deca_sleep(CONT_WAVE_DURATION_MS);

    /* Software reset of the DW1000 to deactivate continuous wave mode and go back to default state. Initialisation and configuration should be run
     * again if one wants to get the DW1000 back to normal operation. */
    dwt_softreset();

    /* End here. */
    while (1)
    { };
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
