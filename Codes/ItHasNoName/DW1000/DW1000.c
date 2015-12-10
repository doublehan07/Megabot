/* Includes ------------------------------------------------------------------*/
#include "DW1000.h"
#include "SPI.h"

/* Private function prototypes & variables -----------------------------------*/
uint32_t getDeviceID(void);                                                             // gets the Device ID which should be 0xDECA0130 (good for testing SPI!)
uint64_t getEUI(void);                                                                  // gets 64 bit Extended Unique Identifier according to IEEE standard
void setEUI(uint64_t EUI);                                                              // sets 64 bit Extended Unique Identifier according to IEEE standard
float getVoltage(void);                                                                 // gets the current chip voltage measurement form the A/D converter
uint64_t getStatus(void);                                                               // get the 40 bit device status
uint64_t getRXTimestamp(void);
uint64_t getTXTimestamp(void);

void sendString(char* message);                                                         // to send String with arbitrary length
void receiveString(char* message);                                                      // to receive char string (length of the buffer must be 1021 to be safe)
void sendFrame(uint8_t* message, uint16_t length);                                      // send a raw frame (length in bytes)
void sendDelayedFrame(uint8_t* message, uint16_t length, uint64_t TxTimestamp);
void startRX(void);                                                                     // start listening for frames
void stopTRX(void);                                                                     // disable tranceiver go back to idle mode

void loadLDE(void);                                                                     // load the leading edge detection algorithm to RAM, [IMPORTANT because receiving malfunction may occur] see User Manual LDELOAD on p22 & p158
void resetRX(void);                                                                     // soft reset only the tranciever part of DW1000
void resetAll(void);                                                                    // soft reset the entire DW1000 (some registers stay as they were see User Manual)

// Interrupt
void (*callbackRX)(void), (*callbackTX)(void);                                          // function pointer to callback which is called when successfull RX took place
void setCallbacks(void (*)(void), void (*)(void));
void setInterrupt(bool RX, bool TX);                                                    // set Interrupt for received a good frame (CRC ok) or transmission done
void ISR(void);                                                                         // interrupt handling method (also calls according callback methods)
uint16_t getFramelength(void);                                                          // to get the framelength of the received frame from the PHY header

// SPI Inteface
uint8_t readRegister8(uint8_t reg, uint16_t subaddress);                                // expressive methods to read or write the number of bits written in the name
uint16_t readRegister16(uint8_t reg, uint16_t subaddress);
uint32_t readRegister32(uint8_t reg, uint16_t subaddress);
uint64_t readRegister40(uint8_t reg, uint16_t subaddress);
void writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer);
void writeRegister16(uint8_t reg, uint16_t subaddress, uint16_t buffer);
void writeRegister32(uint8_t reg, uint16_t subaddress, uint32_t buffer);
void writeRegister40(uint8_t reg, uint16_t subaddress, uint64_t buffer);

void readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length);       // reads the selected part of a slave register into the buffer memory
void writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length);      // writes the buffer memory to the selected slave register
void setupTransaction(uint8_t reg, uint16_t subaddress, bool write);                    // sets up an SPI read or write transaction with correct register address and offset

static void wait_us(__IO uint32_t);


/* Private functions ---------------------------------------------------------*/
void DW1000_Init() {
	uint16_t buffer16;
	uint32_t buffer32;
		setCallbacks(NULL, NULL);
	
		deselect();
    SPIInit();             // with a 1MHz clock rate (worked up to 49MHz in our Test)
    
    resetAll();                         // we do a soft reset of the DW1000 everytime the driver starts

    // Configuration TODO: make method for that
    // User Manual "2.5.5 Default Configurations that should be modified" p. 22
    //Those values are for the standard mode (6.8Mbps, 5, 16Mhz, 32 Symbols) and are INCOMPLETE!
//    writeRegister16(DW1000_AGC_CTRL, 0x04, 0x8870);
//    writeRegister32(DW1000_AGC_CTRL, 0x0C, 0x2502A907);
//    writeRegister32(DW1000_DRX_CONF, 0x08, 0x311A002D);
//    writeRegister8 (DW1000_LDE_CTRL, 0x0806, 0xD);
//    writeRegister16(DW1000_LDE_CTRL, 0x1806, 0x1607);
//    writeRegister32(DW1000_TX_POWER, 0, 0x0E082848);
//    writeRegister32(DW1000_RF_CONF, 0x0C, 0x001E3FE0);
//    writeRegister8 (DW1000_TX_CAL, 0x0B, 0xC0);
//    writeRegister8 (DW1000_FS_CTRL, 0x0B, 0xA6);


    //Those values are for the 110kbps mode (5, 16MHz, 1024 Symbols) and are quite complete
    writeRegister16(DW1000_AGC_CTRL, 0x04, 0x8870);             //AGC_TUNE1 for 16MHz PRF
		buffer16 = readRegister16(DW1000_AGC_CTRL, 0x04);
    writeRegister32(DW1000_AGC_CTRL, 0x0C, 0x2502A907);         //AGC_TUNE2 (Universal)
		buffer32 = readRegister32(DW1000_AGC_CTRL, 0x0C);
    writeRegister16(DW1000_AGC_CTRL, 0x12, 0x0055);             //AGC_TUNE3 (Universal)
		buffer16 = readRegister16(DW1000_AGC_CTRL, 0x12);

    writeRegister16(DW1000_DRX_CONF, 0x02, 0x000A);             //DRX_TUNE0b for 110kbps
		buffer16 = readRegister16(DW1000_DRX_CONF, 0x02);
    writeRegister16(DW1000_DRX_CONF, 0x04, 0x0087);             //DRX_TUNE1a for 16MHz PRF
		buffer16 = readRegister16(DW1000_DRX_CONF, 0x04);
    writeRegister16(DW1000_DRX_CONF, 0x06, 0x0064);             //DRX_TUNE1b for 110kbps & > 1024 symbols
		buffer16 = readRegister16(DW1000_DRX_CONF, 0x06);
    writeRegister32(DW1000_DRX_CONF, 0x08, 0x351A009A);         //PAC size for 1024 symbols preamble & 16MHz PRF
		buffer32 = readRegister32(DW1000_DRX_CONF, 0x08);
    //writeRegister32(DW1000_DRX_CONF, 0x08, 0x371A011D);               //PAC size for 2048 symbols preamble

    writeRegister8 (DW1000_LDE_CTRL, 0x0806, 0xD);              //LDE_CFG1
    writeRegister16(DW1000_LDE_CTRL, 0x1806, 0x1607);           //LDE_CFG2 for 16MHz PRF

    writeRegister32(DW1000_TX_POWER, 0, 0x28282828);            //Power for channel 5

    writeRegister8(DW1000_RF_CONF, 0x0B, 0xD8);                 //RF_RXCTRLH for channel 5
    writeRegister32(DW1000_RF_CONF, 0x0C, 0x001E3FE0);          //RF_TXCTRL for channel 5

    writeRegister8 (DW1000_TX_CAL, 0x0B, 0xC0);                 //TC_PGDELAY for channel 5

    writeRegister32 (DW1000_FS_CTRL, 0x07, 0x0800041D);         //FS_PLLCFG for channel 5
    writeRegister8 (DW1000_FS_CTRL, 0x0B, 0xA6);                //FS_PLLTUNE for channel 5

    loadLDE();                          // important everytime DW1000 initialises/awakes otherwise the LDE algorithm must be turned off or there's receiving malfunction see User Manual LDELOAD on p22 & p158
    
    // 110kbps CAUTION: a lot of other registers have to be set for an optimized operation on 110kbps
    writeRegister16(DW1000_TX_FCTRL, 1, 0x0800 | 0x0100 | 0x0080); // use 1024 symbols preamble (0x0800) (previously 2048 - 0x2800), 16MHz pulse repetition frequency (0x0100), 110kbps bit rate (0x0080) see p.69 of DW1000 User Manual
    writeRegister8(DW1000_SYS_CFG, 2, 0x44);    // enable special receiving option for 110kbps (disable smartTxPower)!! (0x44) see p.64 of DW1000 User Manual [DO NOT enable 1024 byte frames (0x03) becuase it generates disturbance of ranging don't know why...]

    writeRegister16(DW1000_TX_ANTD, 0, 16384); // set TX and RX Antenna delay to neutral because we calibrate afterwards
    writeRegister16(DW1000_LDE_CTRL, 0x1804, 16384); // = 2^14 a quarter of the range of the 16-Bit register which corresponds to zero calibration in a round trip (TX1+RX2+TX2+RX1)

    writeRegister8(DW1000_SYS_CFG, 3, 0x20);    // enable auto reenabling receiver after error
    
    //irq.rise(this, &ISR);       // attach interrupt handler to rising edge of interrupt pin from DW1000
}

void setCallbacks(void (*cbRX)(void), void (*cbTX)(void)) {
    bool RX = false;
    bool TX = false;
    if (cbRX) {
        callbackRX = cbRX;
        RX = true;
    }
    if (cbTX) {
        callbackTX = cbTX;
        TX = true;
    }
    setInterrupt(RX,TX);
}

uint32_t getDeviceID() {
    uint32_t result;
    readRegister(DW1000_DEV_ID, 0, (uint8_t*)&result, 4);
    return result;
}

uint64_t getEUI() {
    uint64_t result;
    readRegister(DW1000_EUI, 0, (uint8_t*)&result, 8);
    return result;
}

void setEUI(uint64_t EUI) {
    writeRegister(DW1000_EUI, 0, (uint8_t*)&EUI, 8);
}

float getVoltage() {
		float Voltage;
    uint8_t buffer[7] = {0x80, 0x0A, 0x0F, 0x01, 0x00};             // algorithm form User Manual p57
    writeRegister(DW1000_RF_CONF, 0x11, buffer, 2);
    writeRegister(DW1000_RF_CONF, 0x12, &buffer[2], 1);
    writeRegister(DW1000_TX_CAL, 0x00, &buffer[3], 1);
    writeRegister(DW1000_TX_CAL, 0x00, &buffer[4], 1);
    readRegister(DW1000_TX_CAL, 0x03, &buffer[5], 2);               // get the 8-Bit readings for Voltage and Temperature
    Voltage = buffer[5] * 0.0057 + 2.3;
    //float Temperature = buffer[6] * 1.13 - 113.0;                 // TODO: getTemperature was always ~35 degree with better formula/calibration
    return Voltage;
}

uint64_t getStatus() {
    return readRegister40(DW1000_SYS_STATUS, 0);
}

uint64_t getRXTimestamp() {
    return readRegister40(DW1000_RX_TIME, 0);
}

uint64_t getTXTimestamp() {
    return readRegister40(DW1000_TX_TIME, 0);
}

void sendString(char* message) {
    sendFrame((uint8_t*)message, strlen(message)+1);
}

void receiveString(char* message) {
    readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)message, getFramelength());  // get data from buffer
}

void sendFrame(uint8_t* message, uint16_t length) {
    uint8_t backup;
		//if (length >= 1021) length = 1021;                            // check for maximim length a frame can have with 1024 Byte frames [not used, see constructor]
    if (length >= 125) length = 125;                                // check for maximim length a frame can have with 127 Byte frames
    writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer
    
    backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
    length += 2;                                                    // including 2 CRC Bytes
    length = ((backup & 0xFC) << 8) | (length & 0x03FF);
    writeRegister16(DW1000_TX_FCTRL, 0, length);
    
    stopTRX();                                                      // stop receiving
    writeRegister8(DW1000_SYS_CTRL, 0, 0x02);                       // trigger sending process by setting the TXSTRT bit
    startRX();                                                      // enable receiver again
}

void sendDelayedFrame(uint8_t* message, uint16_t length, uint64_t TxTimestamp) {
    uint8_t backup;
    //if (length >= 1021) length = 1021;                            // check for maximim length a frame can have with 1024 Byte frames [not used, see constructor]
    if (length >= 125) length = 125;                                // check for maximim length a frame can have with 127 Byte frames
    writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer

    backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
    length += 2;                                                    // including 2 CRC Bytes
    length = ((backup & 0xFC) << 8) | (length & 0x03FF);
    writeRegister16(DW1000_TX_FCTRL, 0, length);

    writeRegister40(DW1000_DX_TIME, 0, TxTimestamp);                //write the timestamp on which to send the message

    stopTRX();                                                      // stop receiving
    writeRegister8(DW1000_SYS_CTRL, 0, 0x02 | 0x04);                // trigger sending process by setting the TXSTRT and TXDLYS bit
    startRX();                                                      // enable receiver again
}

void startRX() {
    writeRegister8(DW1000_SYS_CTRL, 0x01, 0x01);                    // start listening for preamble by setting the RXENAB bit
}

void stopTRX() {
    writeRegister8(DW1000_SYS_CTRL, 0, 0x40);                       // disable tranceiver go back to idle mode
}

// PRIVATE Methods ------------------------------------------------------------------------------------
void loadLDE() {                                            // initialise LDE algorithm LDELOAD User Manual p22
    writeRegister16(DW1000_PMSC, 0, 0x0301);                        // set clock to XTAL so OTP is reliable
    writeRegister16(DW1000_OTP_IF, 0x06, 0x8000);                   // set LDELOAD bit in OTP
    wait_us(150);
    writeRegister16(DW1000_PMSC, 0, 0x0200);                        // recover to PLL clock
}

void resetRX() {    
    writeRegister8(DW1000_PMSC, 3, 0xE0);   // set RX reset
    writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear RX reset
}

void resetAll() {
    writeRegister8(DW1000_PMSC, 0, 0x01);   // set clock to XTAL
    writeRegister8(DW1000_PMSC, 3, 0x00);   // set All reset
    wait_us(10);                            // wait for PLL to lock
    writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear All reset
}


void setInterrupt(bool RX, bool TX) {
    writeRegister16(DW1000_SYS_MASK, 0, RX*0x4000 | TX*0x0080);  // RX good frame 0x4000, TX done 0x0080
}

void ISR() {
    uint64_t status = getStatus();
    if (status & 0x4000) {                                          // a frame was received
        callbackRX();
        writeRegister16(DW1000_SYS_STATUS, 0, 0x6F00);              // clearing of receiving status bits
    }
    if (status & 0x80) {                                            // sending complete
        callbackTX();
        writeRegister8(DW1000_SYS_STATUS, 0, 0xF8);                 // clearing of sending status bits
    }
}

uint16_t getFramelength() {
    uint16_t framelength = readRegister16(DW1000_RX_FINFO, 0);      // get framelength
    framelength = (framelength & 0x03FF) - 2;                       // take only the right bits and subtract the 2 CRC Bytes
    return framelength;
}

// SPI Interface ------------------------------------------------------------------------------------
uint8_t readRegister8(uint8_t reg, uint16_t subaddress) {
    uint8_t result;
    readRegister(reg, subaddress, &result, 1);
    return result;
}

uint16_t readRegister16(uint8_t reg, uint16_t subaddress) {
    uint16_t result;
    readRegister(reg, subaddress, (uint8_t*)&result, 2);
    return result;
}

uint32_t readRegister32(uint8_t reg, uint16_t subaddress) {
    uint32_t result;
    readRegister(reg, subaddress, (uint8_t*)&result, 4);
    return result;
}

uint64_t readRegister40(uint8_t reg, uint16_t subaddress) {
    uint64_t result;
    readRegister(reg, subaddress, (uint8_t*)&result, 5);
    result &= 0xFFFFFFFFFF;                                 // only 40-Bit
    return result;
}

void writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer) {
    writeRegister(reg, subaddress, &buffer, 1);
}

void writeRegister16(uint8_t reg, uint16_t subaddress, uint16_t buffer) {
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 2);
}

void writeRegister32(uint8_t reg, uint16_t subaddress, uint32_t buffer) {
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 4);
}

void writeRegister40(uint8_t reg, uint16_t subaddress, uint64_t buffer) {
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 5);
}

void readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
		int i;  
		setupTransaction(reg, subaddress, false);
    for(i=0; i<length; i++)                             // get data
        buffer[i] = SPI_RW(0x00);
    deselect();
}

void writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
		int i;  
    setupTransaction(reg, subaddress, true);
    for(i=0; i<length; i++)                             // put data
        SPI_RW(buffer[i]);
    deselect();
}

void setupTransaction(uint8_t reg, uint16_t subaddress, bool write) {
    reg |=  (write * DW1000_WRITE_FLAG);                                        // set read/write flag
    select();
    if (subaddress > 0) {                                                       // there's a subadress, we need to set flag and send second header byte
        SPI_RW(reg | DW1000_SUBADDRESS_FLAG);
        if (subaddress > 0x7F) {                                                // sub address too long, we need to set flag and send third header byte
            SPI_RW((uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG); // and 
            SPI_RW((uint8_t)(subaddress >> 7));
        } else {
            SPI_RW((uint8_t)subaddress);
        }
    } else {
        SPI_RW(reg);                                                         // say which register address we want to access
    }
}

extern __IO uint32_t us_Delay;
void wait_us(__IO uint32_t nTime)
{ 
		us_Delay = nTime/2;
		TIM_Cmd(TIM5, ENABLE);
		while(us_Delay != 0);
}
