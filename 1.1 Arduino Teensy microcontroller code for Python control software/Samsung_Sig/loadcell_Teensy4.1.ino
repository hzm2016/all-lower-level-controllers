#include "ads1292r.h"
ads1292r torque_sensor1;

void setup() {
  delay(1000);
  Serial.begin(115200);
  torque_sensor1.Torque_sensor_initial();                                           //initial the torque sensor see ads1292r.cpp.
  torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 2, 0.0003446 * (-1) * 2.35); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.
  torque_sensor1.Torque_sensor_offset_calibration();
  delay(1000);

  SPI1.setMOSI(26);
  SPI1.setMISO(1);
  SPI1.setSCK(27); 
}

void loop() {
  torque_sensor1.Torque_sensor_read(); 

  Serial.print(torque_sensor1.torque[0]); Serial.print("   ");
  Serial.print(torque_sensor1.torque[1]); Serial.print("   ");
  Serial.println();

  delay(100);
}
