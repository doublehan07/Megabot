// -------------------------------------------------------------------------------------------------------------------
//
//  File: range.h - Woxu header for application level dw1000
//
//  Copyright 2016 (c) Woxu Ltd, Nanjing, China.
//
//  All rights reserved.
//
//  Author: shenshumao, January 2016
//
// -------------------------------------------------------------------------------------------------------------------

#ifndef _DECA_API_H_
#define _DECA_API_H_

#include "deca_types.h"

#define DWT_PRF_64M_RFDLY   (515.6f)
#define DWT_PRF_16M_RFDLY   (515.0f)

typedef struct 
{
  uint8 PGdelay;
  
  //TX POWER
  //31:24     BOOST_0.125ms_PWR
  //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
  //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
  //7:0       DEFAULT_PWR-TX_DATA_PWR
  uint32 txPwr[2]; //
}tx_struct;

typedef struct
{
  uint8 channel ;
  uint8 prf ;
  uint8 datarate ;
  uint8 preambleCode ;
  uint8 preambleLength ;
  uint8 pacSize ;
  uint8 nsSFD ;
  uint16 sfdTO ;
} chConfig_t;


uint64 convertmicrosectodevicetimeu(double microsecu);
uint64 convertdevicetimetomicrosec(uint32 devicetimeh);
uint32 convertmicrosectodevicetimeu32(double microsecu);
double convertdevicetimetosec(int32 dt);
double convertdevicetimetosec8(uint8* dt);
int instanceframeduration(int datalength); //duration in symbols

void deca_init(void);
void deca_conf(void);
void deca_power(void);
double reportTOF_f(uint32 tof32);
#endif