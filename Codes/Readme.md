#学习日更
Updated by haldak 1:50am,Nov.28

##DW1000寄存器地址定义
<pre><code>// register addresses
//      Mnemonic                    Address Bytes Description
#define DW1000_DEV_ID               0x00 //     4 Device Identifier – includes device type and revision information
#define DW1000_EUI                  0x01 //     8 Extended Unique Identifier
#define DW1000_PANADR               0x03 //     4 PAN Identifier and Short Address
#define DW1000_SYS_CFG              0x04 //     4 System Configuration bitmap
#define DW1000_SYS_TIME             0x06 //     5 System Time Counter (40-bit)
#define DW1000_TX_FCTRL             0x08 //     5 Transmit Frame Control
#define DW1000_TX_BUFFER            0x09 //  1024 Transmit Data Buffer
#define DW1000_DX_TIME              0x0A //     5 Delayed Send or Receive Time (40-bit)
#define DW1000_RX_FWTO              0x0C //     2 Receive Frame Wait Timeout Period
#define DW1000_SYS_CTRL             0x0D //     4 System Control Register
#define DW1000_SYS_MASK             0x0E //     4 System Event Mask Register
#define DW1000_SYS_STATUS           0x0F //     5 System Event Status Register
#define DW1000_RX_FINFO             0x10 //     4 RX Frame Information                (in double buffer set)
#define DW1000_RX_BUFFER            0x11 //  1024 Receive Data Buffer                 (in double buffer set)
#define DW1000_RX_FQUAL             0x12 //     8 Rx Frame Quality information        (in double buffer set)
#define DW1000_RX_TTCKI             0x13 //     4 Receiver Time Tracking Interval     (in double buffer set)
#define DW1000_RX_TTCKO             0x14 //     5 Receiver Time Tracking Offset       (in double buffer set)
#define DW1000_RX_TIME              0x15 //    14 Receive Message Time of Arrival     (in double buffer set)
#define DW1000_TX_TIME              0x17 //    10 Transmit Message Time of Sending    (in double buffer set)
#define DW1000_TX_ANTD              0x18 //     2 16-bit Delay from Transmit to Antenna
#define DW1000_SYS_STATE            0x19 //     5 System State information
#define DW1000_ACK_RESP_T           0x1A //     4 Acknowledgement Time and Response Time
#define DW1000_RX_SNIFF             0x1D //     4 Pulsed Preamble Reception Configuration
#define DW1000_TX_POWER             0x1E //     4 TX Power Control
#define DW1000_CHAN_CTRL            0x1F //     4 Channel Control
#define DW1000_USR_SFD              0x21 //    41 User-specified short/long TX/RX SFD sequences
#define DW1000_AGC_CTRL             0x23 //    32 Automatic Gain Control configuration
#define DW1000_EXT_SYNC             0x24 //    12 External synchronisation control.
#define DW1000_ACC_MEM              0x25 //  4064 Read access to accumulator data
#define DW1000_GPIO_CTRL            0x26 //    44 Peripheral register bus 1 access - GPIO control
#define DW1000_DRX_CONF             0x27 //    44 Digital Receiver configuration
#define DW1000_RF_CONF              0x28 //    58 Analog RF Configuration
#define DW1000_TX_CAL               0x2A //    52 Transmitter calibration block
#define DW1000_FS_CTRL              0x2B //    21 Frequency synthesiser control block
#define DW1000_AON                  0x2C //    12 Always-On register set
#define DW1000_OTP_IF               0x2D //    18 One Time Programmable Memory Interface
#define DW1000_LDE_CTRL             0x2E //     - Leading edge detection control block
#define DW1000_DIG_DIAG             0x2F //    41 Digital Diagnostics Interface
#define DW1000_PMSC                 0x36 //    48 Power Management System Control Block

#define DW1000_WRITE_FLAG           0x80 // First Bit of the address has to be 1 to indicate we want to write
#define DW1000_SUBADDRESS_FLAG      0x40 // if we have a sub address second Bit has to be 1
#define DW1000_2_SUBADDRESS_FLAG    0x80 // if we have a long sub adress (more than 7 Bit) we set this Bit in the first part</code></pre>

##mbed上的DW1000操作代码
<pre><code>//in cpp
class DW1000 {
    public:            
        DW1000(PinName MOSI, PinName MISO, PinName SCLK, PinName CS, PinName IRQ);              // constructor, uses SPI class
        void setCallbacks(void (*callbackRX)(void), void (*callbackTX)(void));                  // setter for callback functions, automatically enables interrupt, if NULL is passed the coresponding interrupt gets disabled
        template<typename T>
            void setCallbacks(T* tptr, void (T::*mptrRX)(void), void (T::*mptrTX)(void)) {      // overloaded setter to treat member function pointers of objects
            callbackRX.attach(tptr, mptrRX);                                                    // possible client code: dw.setCallbacks(this, &A::callbackRX, &A::callbackTX);
            callbackTX.attach(tptr, mptrTX);                                                    // concept seen in line 100 of http://developer.mbed.org/users/mbed_official/code/mbed/docs/4fc01daae5a5/InterruptIn_8h_source.html
            setInterrupt(true,true);
        }

        // Device API
        uint32_t getDeviceID();                                                                 // gets the Device ID which should be 0xDECA0130 (good for testing SPI!)
        uint64_t getEUI();                                                                      // gets 64 bit Extended Unique Identifier according to IEEE standard
        void setEUI(uint64_t EUI);                                                              // sets 64 bit Extended Unique Identifier according to IEEE standard
        float getVoltage();                                                                     // gets the current chip voltage measurement form the A/D converter
        uint64_t getStatus();                                                                   // get the 40 bit device status
        uint64_t getRXTimestamp();
        uint64_t getTXTimestamp();
        
        void sendString(char* message);                                                         // to send String with arbitrary length
        void receiveString(char* message);                                                      // to receive char string (length of the buffer must be 1021 to be safe)
        void sendFrame(uint8_t* message, uint16_t length);                                      // send a raw frame (length in bytes)
        void sendDelayedFrame(uint8_t* message, uint16_t length, uint64_t TxTimestamp);
        void startRX();                                                                         // start listening for frames
        void stopTRX();                                                                         // disable tranceiver go back to idle mode
        
    //private:
        void loadLDE();                                                                         // load the leading edge detection algorithm to RAM, [IMPORTANT because receiving malfunction may occur] see User Manual LDELOAD on p22 & p158
        void resetRX();                                                                         // soft reset only the tranciever part of DW1000
        void resetAll();                                                                        // soft reset the entire DW1000 (some registers stay as they were see User Manual)

        // Interrupt
        InterruptIn irq;                                                                        // Pin used to handle Events from DW1000 by an Interrupthandler
        FunctionPointer callbackRX;                                                             // function pointer to callback which is called when successfull RX took place
        FunctionPointer callbackTX;                                                             // function pointer to callback which is called when successfull TX took place
        void setInterrupt(bool RX, bool TX);                                                    // set Interrupt for received a good frame (CRC ok) or transmission done
        void ISR();                                                                             // interrupt handling method (also calls according callback methods)
        uint16_t getFramelength();                                                              // to get the framelength of the received frame from the PHY header
        
        // SPI Inteface
        SPI spi;                                                                                // SPI Bus
        DigitalOut cs;                                                                          // Slave selector for SPI-Bus (here explicitly needed to start and end SPI transactions also usable to wake up DW1000)
        
        uint8_t readRegister8(uint8_t reg, uint16_t subaddress);                                // expressive methods to read or write the number of bits written in the name
        uint16_t readRegister16(uint8_t reg, uint16_t subaddress);
        uint64_t readRegister40(uint8_t reg, uint16_t subaddress);
        void writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer);
        void writeRegister16(uint8_t reg, uint16_t subaddress, uint16_t buffer);
        void writeRegister32(uint8_t reg, uint16_t subaddress, uint32_t buffer);
        void writeRegister40(uint8_t reg, uint16_t subaddress, uint64_t buffer);

        void readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length);       // reads the selected part of a slave register into the buffer memory
        void writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length);      // writes the buffer memory to the selected slave register
        void setupTransaction(uint8_t reg, uint16_t subaddress, bool write);                    // sets up an SPI read or write transaction with correct register address and offset
        void select();                                                                          // selects the only slave for a transaction
        void deselect();                                                                        // deselects the only slave after transaction
};</code></pre>

以上在DW1000.h中，类的具体实现放在了DW1000.cpp中
<pre><code>//需要每个函数的拆分与具体解析
DW1000::DW1000(PinName MOSI, PinName MISO, PinName SCLK, PinName CS, PinName IRQ) : irq(IRQ), spi(MOSI, MISO, SCLK), cs(CS) {
    setCallbacks(NULL, NULL);
    
    deselect();                         // Chip must be deselected first
    spi.format(8,0);                    // Setup the spi for standard 8 bit data and SPI-Mode 0 (GPIO5, GPIO6 open circuit or ground on DW1000)
    spi.frequency(5000000);             // with a 1MHz clock rate (worked up to 49MHz in our Test)
    
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
    writeRegister32(DW1000_AGC_CTRL, 0x0C, 0x2502A907);         //AGC_TUNE2 (Universal)
    writeRegister16(DW1000_AGC_CTRL, 0x12, 0x0055);             //AGC_TUNE3 (Universal)

    writeRegister16(DW1000_DRX_CONF, 0x02, 0x000A);             //DRX_TUNE0b for 110kbps
    writeRegister16(DW1000_DRX_CONF, 0x04, 0x0087);             //DRX_TUNE1a for 16MHz PRF
    writeRegister16(DW1000_DRX_CONF, 0x06, 0x0064);             //DRX_TUNE1b for 110kbps & > 1024 symbols
    writeRegister32(DW1000_DRX_CONF, 0x08, 0x351A009A);         //PAC size for 1024 symbols preamble & 16MHz PRF
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
    
    irq.rise(this, &DW1000::ISR);       // attach interrupt handler to rising edge of interrupt pin from DW1000
}

void DW1000::setCallbacks(void (*callbackRX)(void), void (*callbackTX)(void)) {
    bool RX = false;
    bool TX = false;
    if (callbackRX) {
        DW1000::callbackRX.attach(callbackRX);
        RX = true;
    }
    if (callbackTX) {
        DW1000::callbackTX.attach(callbackTX);
        TX = true;
    }
    setInterrupt(RX,TX);
}

uint32_t DW1000::getDeviceID() {
    uint32_t result;
    readRegister(DW1000_DEV_ID, 0, (uint8_t*)&result, 4);
    return result;
}

uint64_t DW1000::getEUI() {
    uint64_t result;
    readRegister(DW1000_EUI, 0, (uint8_t*)&result, 8);
    return result;
}

void DW1000::setEUI(uint64_t EUI) {
    writeRegister(DW1000_EUI, 0, (uint8_t*)&EUI, 8);
}

float DW1000::getVoltage() {
    uint8_t buffer[7] = {0x80, 0x0A, 0x0F, 0x01, 0x00};             // algorithm form User Manual p57
    writeRegister(DW1000_RF_CONF, 0x11, buffer, 2);
    writeRegister(DW1000_RF_CONF, 0x12, &buffer[2], 1);
    writeRegister(DW1000_TX_CAL, 0x00, &buffer[3], 1);
    writeRegister(DW1000_TX_CAL, 0x00, &buffer[4], 1);
    readRegister(DW1000_TX_CAL, 0x03, &buffer[5], 2);               // get the 8-Bit readings for Voltage and Temperature
    float Voltage = buffer[5] * 0.0057 + 2.3;
    //float Temperature = buffer[6] * 1.13 - 113.0;                 // TODO: getTemperature was always ~35 degree with better formula/calibration
    return Voltage;
}

uint64_t DW1000::getStatus() {
    return readRegister40(DW1000_SYS_STATUS, 0);
}

uint64_t DW1000::getRXTimestamp() {
    return readRegister40(DW1000_RX_TIME, 0);
}

uint64_t DW1000::getTXTimestamp() {
    return readRegister40(DW1000_TX_TIME, 0);
}

void DW1000::sendString(char* message) {
    sendFrame((uint8_t*)message, strlen(message)+1);
}

void DW1000::receiveString(char* message) {
    readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)message, getFramelength());  // get data from buffer
}

void DW1000::sendFrame(uint8_t* message, uint16_t length) {
    //if (length >= 1021) length = 1021;                            // check for maximim length a frame can have with 1024 Byte frames [not used, see constructor]
    if (length >= 125) length = 125;                                // check for maximim length a frame can have with 127 Byte frames
    writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer
    
    uint8_t backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
    length += 2;                                                    // including 2 CRC Bytes
    length = ((backup & 0xFC) << 8) | (length & 0x03FF);
    writeRegister16(DW1000_TX_FCTRL, 0, length);
    
    stopTRX();                                                      // stop receiving
    writeRegister8(DW1000_SYS_CTRL, 0, 0x02);                       // trigger sending process by setting the TXSTRT bit
    startRX();                                                      // enable receiver again
}

void DW1000::sendDelayedFrame(uint8_t* message, uint16_t length, uint64_t TxTimestamp) {
    //if (length >= 1021) length = 1021;                            // check for maximim length a frame can have with 1024 Byte frames [not used, see constructor]
    if (length >= 125) length = 125;                                // check for maximim length a frame can have with 127 Byte frames
    writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer

    uint8_t backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
    length += 2;                                                    // including 2 CRC Bytes
    length = ((backup & 0xFC) << 8) | (length & 0x03FF);
    writeRegister16(DW1000_TX_FCTRL, 0, length);

    writeRegister40(DW1000_DX_TIME, 0, TxTimestamp);                //write the timestamp on which to send the message

    stopTRX();                                                      // stop receiving
    writeRegister8(DW1000_SYS_CTRL, 0, 0x02 | 0x04);                // trigger sending process by setting the TXSTRT and TXDLYS bit
    startRX();                                                      // enable receiver again
}

void DW1000::startRX() {
    writeRegister8(DW1000_SYS_CTRL, 0x01, 0x01);                    // start listening for preamble by setting the RXENAB bit
}

void DW1000::stopTRX() {
    writeRegister8(DW1000_SYS_CTRL, 0, 0x40);                       // disable tranceiver go back to idle mode
}

// PRIVATE Methods ------------------------------------------------------------------------------------
void DW1000::loadLDE() {                                            // initialise LDE algorithm LDELOAD User Manual p22
    writeRegister16(DW1000_PMSC, 0, 0x0301);                        // set clock to XTAL so OTP is reliable
    writeRegister16(DW1000_OTP_IF, 0x06, 0x8000);                   // set LDELOAD bit in OTP
    wait_us(150);
    writeRegister16(DW1000_PMSC, 0, 0x0200);                        // recover to PLL clock
}

void DW1000::resetRX() {    
    writeRegister8(DW1000_PMSC, 3, 0xE0);   // set RX reset
    writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear RX reset
}

void DW1000::resetAll() {
    writeRegister8(DW1000_PMSC, 0, 0x01);   // set clock to XTAL
    writeRegister8(DW1000_PMSC, 3, 0x00);   // set All reset
    wait_us(10);                            // wait for PLL to lock
    writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear All reset
}


void DW1000::setInterrupt(bool RX, bool TX) {
    writeRegister16(DW1000_SYS_MASK, 0, RX*0x4000 | TX*0x0080);  // RX good frame 0x4000, TX done 0x0080
}

void DW1000::ISR() {
    uint64_t status = getStatus();
    if (status & 0x4000) {                                          // a frame was received
        callbackRX.call();
        writeRegister16(DW1000_SYS_STATUS, 0, 0x6F00);              // clearing of receiving status bits
    }
    if (status & 0x80) {                                            // sending complete
        callbackTX.call();
        writeRegister8(DW1000_SYS_STATUS, 0, 0xF8);                 // clearing of sending status bits
    }
}

uint16_t DW1000::getFramelength() {
    uint16_t framelength = readRegister16(DW1000_RX_FINFO, 0);      // get framelength
    framelength = (framelength & 0x03FF) - 2;                       // take only the right bits and subtract the 2 CRC Bytes
    return framelength;
}

// SPI Interface ------------------------------------------------------------------------------------
uint8_t DW1000::readRegister8(uint8_t reg, uint16_t subaddress) {
    uint8_t result;
    readRegister(reg, subaddress, &result, 1);
    return result;
}

uint16_t DW1000::readRegister16(uint8_t reg, uint16_t subaddress) {
    uint16_t result;
    readRegister(reg, subaddress, (uint8_t*)&result, 2);
    return result;
}

uint64_t DW1000::readRegister40(uint8_t reg, uint16_t subaddress) {
    uint64_t result;
    readRegister(reg, subaddress, (uint8_t*)&result, 5);
    result &= 0xFFFFFFFFFF;                                 // only 40-Bit
    return result;
}

void DW1000::writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer) {
    writeRegister(reg, subaddress, &buffer, 1);
}

void DW1000::writeRegister16(uint8_t reg, uint16_t subaddress, uint16_t buffer) {
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 2);
}

void DW1000::writeRegister32(uint8_t reg, uint16_t subaddress, uint32_t buffer) {
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 4);
}

void DW1000::writeRegister40(uint8_t reg, uint16_t subaddress, uint64_t buffer) {
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 5);
}

void DW1000::readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
    setupTransaction(reg, subaddress, false);
    for(int i=0; i<length; i++)                             // get data
        buffer[i] = spi.write(0x00);
    deselect();
}

void DW1000::writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
    setupTransaction(reg, subaddress, true);
    for(int i=0; i<length; i++)                             // put data
        spi.write(buffer[i]);
    deselect();
}

void DW1000::setupTransaction(uint8_t reg, uint16_t subaddress, bool write) {
    reg |=  (write * DW1000_WRITE_FLAG);                                        // set read/write flag
    select();
    if (subaddress > 0) {                                                       // there's a subadress, we need to set flag and send second header byte
        spi.write(reg | DW1000_SUBADDRESS_FLAG);
        if (subaddress > 0x7F) {                                                // sub address too long, we need to set flag and send third header byte
            spi.write((uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG); // and 
            spi.write((uint8_t)(subaddress >> 7));
        } else {
            spi.write((uint8_t)subaddress);
        }
    } else {
        spi.write(reg);                                                         // say which register address we want to access
    }
}

void DW1000::select() {     // always called to start an SPI transmission
    irq.disable_irq();      // disable interrupts from DW1000 during SPI becaus this leads to crashes!      TODO: if you have other interrupt handlers attached on the micro controller, they could also interfere.
    cs = 0;                 // set Cable Select pin low to start transmission
}
void DW1000::deselect() {   // always called to end an SPI transmission
    cs = 1;                 // set Cable Select pin high to stop transmission
    irq.enable_irq();       // reenable the interrupt handler
}</code></pre>