/**************************************************************************/
/*!
    @file     MPL115A2_SoftI2C.cpp
    @author   Albert Lederer
    @license  license.txt

    Driver for the MPL115A2 barometric pressure sensor
	using SoftI2C from the DigitalIO Library

    This is a library for the Adafruit MPL115A2 breakout
    ----> https://www.adafruit.com/products/992
	
	-->Based on the most excellent work done by K.Townsend from Adafruit
		Industries for the original Adafruit MPL115A2 Library.  Buy their stuff.
		
	I am on no shape or form affiliated with Adafruit Industries(other than
	buying their stuff).  This driver was written without their knowledge 
	(which I hope they won't sue me for).

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <DigitalIO.h>

#include "MPL115A2_SoftI2C.h"


/**************************************************************************/
/*!
    @brief  Gets the factory-set coefficients for this particular sensor
*/
/**************************************************************************/
void MPL115A2_SoftI2C::readCoefficients() {
  int16_t a0coeff;
  int16_t b1coeff;
  int16_t b2coeff;
  int16_t c12coeff;

  uint8_t address = MPL115A2_REGISTER_A0_COEFF_MSB;
  i2c->transfer(MPL115A2_ADDRESS | I2C_WRITE, &address, 1, I2C_STOP);
  char buf[8] = { 0 } ;
  i2c->transfer(MPL115A2_ADDRESS | I2C_READ, &buf, 8, I2C_STOP);

  a0coeff = (( (uint16_t) buf[0] << 8) | buf[1]);
  b1coeff = (( (uint16_t) buf[2] << 8) | buf[3]);
  b2coeff = (( (uint16_t) buf[4] << 8) | buf[5]);
  c12coeff = (( (uint16_t) (buf[6] << 8) | buf[7])) >> 2;

  /*  
  Serial.print("A0 = "); Serial.println(a0coeff, HEX);
  Serial.print("B1 = "); Serial.println(b1coeff, HEX);
  Serial.print("B2 = "); Serial.println(b2coeff, HEX);
  Serial.print("C12 = "); Serial.println(c12coeff, HEX);
  */
  

  _mpl115a2_a0 = (float)a0coeff / 8;
  _mpl115a2_b1 = (float)b1coeff / 8192;
  _mpl115a2_b2 = (float)b2coeff / 16384;
  _mpl115a2_c12 = (float)c12coeff;
  _mpl115a2_c12 /= 4194304.0;

  /*
  Serial.print("a0 = "); Serial.println(_mpl115a2_a0);
  Serial.print("b1 = "); Serial.println(_mpl115a2_b1);
  Serial.print("b2 = "); Serial.println(_mpl115a2_b2);
  Serial.print("c12 = "); Serial.println(_mpl115a2_c12);
  */
}

/**************************************************************************/
/*!
    @brief  Instantiates a new MPL115A2 class
*/
/**************************************************************************/
MPL115A2_SoftI2C::MPL115A2_SoftI2C(SoftI2cMaster *i2cMaster) {
  _mpl115a2_a0 = 0.0F;
  _mpl115a2_b1 = 0.0F;
  _mpl115a2_b2 = 0.0F;
  _mpl115a2_c12 = 0.0F;
  i2c = i2cMaster;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
void MPL115A2_SoftI2C::begin() {
  // Read factory coefficient values (this only needs to be done once)
  readCoefficients();
}

/**************************************************************************/
/*!
    @brief  Gets the floating-point pressure level in kPa
*/
/**************************************************************************/
float MPL115A2_SoftI2C::getPressure() {
  float     pressureComp,centigrade;

  getPT(&pressureComp, &centigrade);
  return pressureComp;
}


/**************************************************************************/
/*!
    @brief  Gets the floating-point temperature in Centigrade
*/
/**************************************************************************/
float MPL115A2_SoftI2C::getTemperature() {
  float     pressureComp, centigrade;

  getPT(&pressureComp, &centigrade);
  return centigrade;
}

/**************************************************************************/
/*!
    @brief  Gets both at once and saves a little time
*/
/**************************************************************************/
void MPL115A2_SoftI2C::getPT(float *P, float *T) {
  uint16_t 	pressure, temp;
  float     pressureComp;

  // Get raw pressure and temperature settings
  uint8_t address = MPL115A2_REGISTER_STARTCONVERSION;
  i2c->transfer(MPL115A2_ADDRESS | I2C_WRITE, &address, 1, I2C_STOP);
  delay(5);
  uint8_t cmd[2];
  cmd[0]=0x12;
  cmd[1]=0x01;
  i2c->transfer(MPL115A2_ADDRESS | I2C_WRITE, &cmd, 2, I2C_STOP);

  delay(5);

  address = MPL115A2_REGISTER_PRESSURE_MSB;
  i2c->transfer(MPL115A2_ADDRESS | I2C_WRITE, &address, 1, I2C_STOP);  
  delay(5);
  uint8_t buf[4] = { 0 };
  i2c->transfer(MPL115A2_ADDRESS | I2C_READ, &buf, 4, I2C_STOP);
  
  pressure = (( (uint16_t) buf[0] << 8) | buf[1]) >> 6;
  temp = (( (uint16_t) buf[2] << 8) | buf[3]) >> 6;
  

  // See datasheet p.6 for evaluation sequence
  pressureComp = _mpl115a2_a0 + (_mpl115a2_b1 + _mpl115a2_c12 * temp ) * pressure + _mpl115a2_b2 * temp;

  // Return pressure and temperature as floating point values
  *P = ((65.0F / 1023.0F) * pressureComp) + 50.0F;        // kPa
  *T = ((float) temp - 498.0F) / -5.35F +25.0F;           // C
  
}



