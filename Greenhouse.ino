// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       Greenhouse.ino
    Created:	5/3/2019 11:04:45 PM
    Author:     bgbnbigben
*/

#include <Bridge.h>
#include <Console.h>

#include "HTU21D.h"
#include "GreenhouseLibraries.h"

DTProbes probes(8);
HTU21D htu;
Logger logger;
TDSProbe tdsProbe(A0);
pHProbe phProbe(A1);

enum Values: uint8_t {
  tds,
  dt_probes,
  htu_humidity,
  htu_temp,
  ph
};

void setup() {
  Bridge.begin();
  Console.begin();
  // Default all communication on the I2C wires to have a timeout of 100ms.
  // If this value passes the lines are released and reset.
  I2c.timeOut(100);
  while (!Console);
  probes.begin();
  htu.begin();
  probes.requestTemperatures();
}

void loop() {
  uint8_t toRead = B0000111;

  tdsProbe.triggerTdsRead();
  htu.triggerTemperatureReading();

  unsigned long start = millis();
  while (toRead) {
    for (int i = 0; i < 8; i++) {
      if (toRead & (1 << i)) {
        switch (i) {
          case tds:
            if (tdsProbe.tdsReadingAvailable()) {
              logger.transfer(ProbeLocation::TANK, ProbeType::TDS, tdsProbe.update());
              phProbe.triggerPHRead();
              toRead &= ~(1 << i);
              toRead |= (1 << ph);
            }
            break;
          case dt_probes:
            if (probes.isConversionComplete()) {
              for (int i = 0; i < probes.getDeviceCount(); i++) {
                float tempC = probes.getTempCByIndex(i);
                if (tempC != DTProbes::BROKENWIRE) {
                  logger.transfer(ProbeLocation::INSIDE, ProbeType::TEMPERATURE, tempC);
                }
              }
              toRead &= ~(1 << i);
              probes.requestTemperatures();
            }
            break;
          case htu_humidity: // Typical performance for RH sensing is 14ms, max of 16ms according to datasheet
            if (htu.isConversionComplete()) {
              logger.transfer(ProbeLocation::BOX, ProbeType::HUMIDITY, htu.getHumidity());
              toRead &= ~(1 << i);
            }
            break;
          case htu_temp:  // Typical performance for T sensing is 44ms, max of 50ms.
            if (htu.isConversionComplete()) {
              htu.triggerHumidityReading();
              logger.transfer(ProbeLocation::BOX, ProbeType::TEMPERATURE, htu.getTemperature());
              toRead &= ~(1 << i);
              toRead |= (1 << htu_humidity);
            }
            break;
          case ph:
            if (phProbe.pHReadingAvailable()) {
              logger.transfer(ProbeLocation::TANK, ProbeType::PH, phProbe.update());
              toRead &= ~(1 << i);
            }
            break;
        }
      }
    }
  }
  unsigned long delta = millis() - start;
  Console.println("It took " + String(delta) + " ms");
  Console.flush();
  delay(10000);
}
