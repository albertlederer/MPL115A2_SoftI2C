#include <MPL115A2_SoftI2C.h>
#include <DigitalIO.h>

const uint8_t SDA_PIN = A4;
const uint8_t SCL_PIN = A5;

SoftI2cMaster i2c(SCL_PIN, SDA_PIN);

MPL115A2_SoftI2C mpl115a2(&i2c);

void setup() {
  mpl115a2.begin();
}

void loop() {
  float pressurekPaMpl = 0, temperatureCMpl = 0;    

  mpl115a2.getPT(&pressurekPaMpl,&temperatureCMpl);
  Serial.print("Pressure: "); Serial.print(pressurekPaMpl, 4); Serial.print(" kPa; ");
  Serial.print("Temperature: "); Serial.print(temperatureCMpl, 1); Serial.println(" C"); 
  
  delay(1000);
  
}
