#include "RTClib.h"

#define PT7C433833_ADDRESS 0x68   ///< I2C address for PT7C433833
#define PT7C433833_TIME 0x00      ///< Time register
#define PT7C433833_CONTROL_REGISTER 0x7   ///< Control register
#define PT7C433833_STATUSREG 0x0F ///< Status register
#define PT7C433833_TEMPERATUREREG 0x11
/**************************************************************************/
/*!
    @brief  Start I2C for the PT7C433833 and test succesful connection
    @param  wireInstance pointer to the I2C bus
    @return True if Wire can find PT7C433833 or false otherwise.
*/
/**************************************************************************/
bool RTC_PT7C433833::begin(TwoWire *wireInstance) {
  if (i2c_dev)
    delete i2c_dev;
  i2c_dev = new Adafruit_I2CDevice(PT7C433833_ADDRESS, wireInstance);
  if (!i2c_dev->begin())
    return false;
  return true;
}

/**************************************************************************/
/*!
    @brief  Check the status register Oscillator Stop Flag to see if the PT7C433833
   stopped due to power loss
    @return True if the bit is set (oscillator stopped) or false if it is
   running
*/
/**************************************************************************/
bool RTC_PT7C433833::lostPower(void) {
  if(read_register(PT7C433833_Control_OSF)) return true;
  return false;
}

/**************************************************************************/
/*!
    @brief  Set the date and flip the Oscillator Stop Flag
    @param dt DateTime object containing the date/time to set
*/
/**************************************************************************/
void RTC_PT7C433833::adjust(const DateTime &dt) {
  uint8_t buffer[8] = {PT7C433833_TIME,
                       bin2bcd(dt.second()),
                       bin2bcd(dt.minute()),
                       bin2bcd(dt.hour()),
                       bin2bcd(dowToPT7C433833(dt.dayOfTheWeek())),
                       bin2bcd(dt.day()),
                       bin2bcd(dt.month()),
                       bin2bcd(dt.year() - 2000U)};
  i2c_dev->write(buffer, 8);

  uint8_t statreg = read_register(PT7C433833_STATUSREG);
  statreg &= ~0x80; // flip OSF bit
  write_register(PT7C433833_STATUSREG, statreg);
}


/**************************************************************************/
/*!
    @brief  Is the PT7C433833 running? Check the STOP bit in register Control_1
    @return 1 if the RTC is running, 0 if not
*/
/**************************************************************************/
uint8_t RTC_PT7C433833::isrunning() {
  return !(read_register(PT7C433833_Control_OSF) & 1);
}

/**************************************************************************/
/*!
    @brief  Get the current date/time
    @return DateTime object with the current date/time
*/
/**************************************************************************/
DateTime RTC_PT7C433833::now() {
  uint8_t buffer[7];
  buffer[0] = 0;
  i2c_dev->write_then_read(buffer, 1, buffer, 7);

  return DateTime(bcd2bin(buffer[6]) + 2000U, bcd2bin(buffer[5] & 0x7F),
                  bcd2bin(buffer[4]), bcd2bin(buffer[2]), bcd2bin(buffer[1]),
                  bcd2bin(buffer[0] & 0x7F));
}

/**************************************************************************/
/*!
    @brief  Read the SQW pin mode
    @return Pin mode, see PT7C433833_RateSelect_Register enum
*/
/**************************************************************************/
PT7C433833_RateSelect_Register RTC_PT7C433833::readSqwPinMode() {
  int mode;
  mode = read_register(PT7C433833_Control_SQWE);
  //if (mode & 0x04)
  //  mode = PT7C433833_OFF;
  return static_cast<PT7C433833_RateSelect_Register>(mode);
}

/**************************************************************************/
/*!
    @brief  Set the SQW pin mode
    @param mode Desired mode, see PT7C433833_RateSelect_Register enum
*/
/**************************************************************************/
void RTC_PT7C433833::writeSqwPinMode(PT7C433833_RateSelect_Register mode) {
  uint8_t ctrl = read_register(PT7C433833_CONTROL_REGISTER);

  ctrl &= ~(PT7C433833_Control_SQWE); // turn off INTCON
  ctrl &= ~(PT7C433833_Control_RS1 | PT7C433833_Control_RS0); // set freq bits to 0

  write_register(PT7C433833_CONTROL_REGISTER, ctrl | mode);
}

/**************************************************************************/
/*!
    @brief  Enable 32KHz Output
    @details The 32kHz output is enabled by default. It requires an external
    pull-up resistor to function correctly
*/
/**************************************************************************/
void RTC_PT7C433833::enable32K(void) {
  uint8_t status = read_register(PT7C433833_STATUSREG);
  status |= (0x1 << 0x03);
  write_register(PT7C433833_STATUSREG, status);
}

/**************************************************************************/
/*!
    @brief  Disable 32KHz Output
*/
/**************************************************************************/
void RTC_PT7C433833::disable32K(void) {
  uint8_t status = read_register(PT7C433833_STATUSREG);
  status &= ~(0x1 << 0x03);
  write_register(PT7C433833_STATUSREG, status);
}

/**************************************************************************/
/*!
    @brief  Get status of 32KHz Output
    @return True if enabled otherwise false
*/
/**************************************************************************/
bool RTC_PT7C433833::isEnabled32K(void) {
  return (read_register(PT7C433833_STATUSREG) >> 0x03) & 0x01;
}
