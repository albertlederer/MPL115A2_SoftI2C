/**************************************************************************/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <DigitalIO.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define MPL115A2_ADDRESS                       0xC0    
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define MPL115A2_REGISTER_PRESSURE_MSB         0x00
    #define MPL115A2_REGISTER_PRESSURE_LSB         0x01
    #define MPL115A2_REGISTER_TEMP_MSB             0x02
    #define MPL115A2_REGISTER_TEMP_LSB             0x03
    #define MPL115A2_REGISTER_A0_COEFF_MSB         0x04
    #define MPL115A2_REGISTER_A0_COEFF_LSB         0x05
    #define MPL115A2_REGISTER_B1_COEFF_MSB         0x06
    #define MPL115A2_REGISTER_B1_COEFF_LSB         0x07
    #define MPL115A2_REGISTER_B2_COEFF_MSB         0x08
    #define MPL115A2_REGISTER_B2_COEFF_LSB         0x09
    #define MPL115A2_REGISTER_C12_COEFF_MSB        0x0A
    #define MPL115A2_REGISTER_C12_COEFF_LSB        0x0B
    #define MPL115A2_REGISTER_STARTCONVERSION      0x12
/*=========================================================================*/

class MPL115A2_SoftI2C{
 public:
  MPL115A2_SoftI2C(SoftI2cMaster *i2cMaster);
  void begin(void);
  float getPressure(void);
  float getTemperature(void);
  void getPT(float *P, float *T);

 private:
  float _mpl115a2_a0;
  float _mpl115a2_b1;
  float _mpl115a2_b2;
  float _mpl115a2_c12;
  SoftI2cMaster *i2c;

  void readCoefficients(void);
};
