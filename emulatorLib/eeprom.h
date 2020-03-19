#ifndef __EEPROM__H_
#define __EEPROM__H_

/***********************************************************
Author: Bernard Borredon
Date : 21 decembre 2015
Version: 1.3
  - Correct write(uint32_t address, int8_t data[], uint32_t length) for eeprom >= T24C32.
    Tested with 24C02, 24C08, 24C16, 24C64, 24C256, 24C512, 24C1025 on LPC1768 (mbed online and µVision V5.16a).
  - Correct main test.
    
Date : 12 decembre 2013
Version: 1.2
  - Update api documentation
  
Date: 11 december 2013
Version: 1.1
  - Change address parameter size form uint16_t to uint32_t (error for eeprom > 24C256).
  - Change size parameter size from uint16_t to uint32_t (error for eeprom > 24C256).
    - Add EEPROM name as a private static const char array.
    - Add function getName.
    - Add a test program.

Date: 27 december 2011
Version: 1.0
************************************************************/

// Includes
#include <string> 

#include "mbed.h"

// Defines
#define EEPROM_Address     0xa0

#define EEPROM_NoError     0x00
#define EEPROM_BadAddress  0x01
#define EEPROM_I2cError    0x02
#define EEPROM_ParamError  0x03
#define EEPROM_OutOfRange  0x04
#define EEPROM_MallocError 0x05

#define EEPROM_MaxError       6

static std::string _ErrorMessageEEPROM[EEPROM_MaxError] = {
                                                            "",
                                                            "Bad chip address",
                                                            "I2C error (nack)",
                                                            "Invalid parameter",
                                                            "Data address out of range",
                                                            "Memory allocation error"
                                                          };

/** EEPROM Class
*/
class EEPROM {
public:
    enum TypeEeprom {T24C01=128,T24C02=256,T24C04=512,T24C08=1024,T24C16=2048,
                     T24C32=4096,T24C64=8192,T24C128=16384,T24C256=32768,
                     T24C512=65536,T24C1024=131072,T24C1025=131073} Type;
                                         
    /**
     * Constructor, initialize the eeprom on i2c interface.
     * @param sda sda i2c pin (PinName)
     * @param scl scl i2c pin (PinName)
     * @param address eeprom address, according to eeprom type (uint8_t)
     * @param type eeprom type (TypeEeprom) 
     * @return none
    */
    EEPROM(PinName sda, PinName scl, uint8_t address, int type);
    
    /**
     * Random read byte
     * @param address start address (uint32_t)
     * @param data byte to read (int8_t&)
     * @return none
    */
    void read(uint32_t address, int8_t& data);
    
    /**
     * Random read short
     * @param address start address (uint32_t)
     * @param data short to read (int16_t&)
     * @return none
    */
    void read(uint32_t address, int16_t& data);
    
    /**
     * Random read long
     * @param address start address (uint32_t)
     * @param data long to read (int32_t&)
     * @return none
    */
    void read(uint32_t address, int32_t& data);
    
    /**
     * Random read float
     * @param address start address (uint32_t)
     * @param data float to read (float&)
     * @return none
    */
    void read(uint32_t address, float& data);
    
    /**
     * Random read anything
     * @param address start address (uint32_t)
     * @param data data to read (void *)
     * @param size number of bytes to read (uint32_t)
     * @return none
    */
    void read(uint32_t address, void *data, uint32_t size);
    
    /**
     * Current address read byte
     * @param data byte to read (int8_t&)
     * @return none
    */
    void read(int8_t& data);
    
    /**
     * Sequential read byte
     * @param address start address (uint32_t)
     * @param data bytes array to read (int8_t[]&)
     * @param size number of bytes to read (uint32_t)
     * @return none
    */
    void read(uint32_t address, int8_t *data, uint32_t size);
    
    /**
     * Write byte
     * @param address start address (uint32_t)
     * @param data byte to write (int8_t)
     * @return none
    */
    void write(uint32_t address, int8_t data);
    
    /**
     * Write short
     * @param address start address (uint32_t)
     * @param data short to write (int16_t)
     * @return none
    */
    void write(uint32_t address, int16_t data);
    
    /**
     * Write long
     * @param address start address (uint32_t)
     * @param data long to write (int32_t)
     * @return none
    */
    void write(uint32_t address, int32_t data);
    
    /**
     * Write float
     * @param address start address (uint32_t)
     * @param data float to write (float)
     * @return none
    */
    void write(uint32_t address, float data);
    
    /**
     * Write anything (use the page write mode)
     * @param address start address (uint32_t)
     * @param data data to write (void *)
     * @param size number of bytes to write (uint32_t)
     * @return none
    */
    void write(uint32_t address, void *data, uint32_t size);
    
    /**
     * Write array of bytes (use the page mode)
     * @param address start address (uint32_t)
     * @param data bytes array to write (int8_t[])
     * @param size number of bytes to write (uint32_t)
     * @return none
    */
    void write(uint32_t address, int8_t data[], uint32_t size);
    
    /**
     * Wait eeprom ready
     * @param none
     * @return none
    */
    void ready(void);
    
    /**
     * Get eeprom size in bytes
     * @param none
     * @return size in bytes (uint32_t)
    */
    uint32_t getSize(void);
        
    /**
     * Get eeprom name
     * @param none
     * @return name (const char*)
    */
    const char* getName(void);
    
    /**
     * Clear eeprom (write with 0)
     * @param  none
     * @return none
    */
    void clear(void);
    
     /**
     * Get the current error number (EEPROM_NoError if no error)
     * @param  none
     * @return none
    */
    uint8_t getError(void);
    
    /**
     * Get current error message
     * @param  none
     * @return current error message(std::string)
    */
    std::string getErrorMessage(void)
    { 
      return(_ErrorMessageEEPROM[_errnum]);
    }
    
//---------- local variables ----------
private:
    I2C _i2c;              // Local i2c communication interface instance
    int _address;          // Local i2c address
    uint8_t _errnum;       // Error number
    int _type;      // EEPROM type
    uint8_t _page_write;   // Page write size
    uint8_t _page_number;  // Number of page
    uint32_t _size;        // Size in bytes
    bool checkAddress(uint32_t address); // Check address range
    static const char * const _name[]; // eeprom name
//-------------------------------------
};
#endif
