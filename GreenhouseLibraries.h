/*
 Name:		GreenhouseLibraries.h
 Created:	5/3/2019 1:44:33 PM
 Author:	bgbnbigben
 Editor:	http://www.visualmicro.com
*/

#ifndef _GreenhouseLibraries_h
#define _GreenhouseLibraries_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <EEPROM.h>
#include <Bridge.h>
#include <Console.h>
#include <OneWire.h>
#include <I2C.h>

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

inline bool wasNack(uint8_t status) {
	return (status == MT_SLA_NACK || status == MT_DATA_NACK || status == MR_SLA_NACK || status == MR_DATA_NACK);
}

class TDSProbe {

  const float adcRange = 1024.0; // 1024 for 10bit ADC; 4096 for 12bit ADC. Fixed by the Arduino.
  const int kValueAddress = 8; // Default EEPROM address for the k value. Defaults to 0x08.
  const float tdsFactor = 0.67; // Some value between 0.55 and 0.87, with 0.67 being the standard approximation.

  float refVoltage;
  float refTemperature;
  // The "k value" is the unit for conductivity. To calculate k, given a solution with:
  //   - a known TDS value, TDS, and
  //   - a known temperature, T
  // You want something like as follows:
  // knownEC = (TDS / tdsFactor) * (1 + 0.02*(T - 25)) (note: this is the standard linear approximation for electrical resistivity changes given temperature dependence)
  // kValue = knownEC / (133.42 * refVoltage^3 - 255.86 * refVoltage^2 + 857.39 * refVoltage)
  // In theory kValue will be between .25 and 4.
  // The cubic equation is derived from empirical data, hence the insanity. 
  // "The equation is get from the experiment, which by measure different TDS value solution's voltage to get the curve between TDS and voltage, then get the equation between them."
  float kValue = 0.606; // standard k value for water.

public:
  TDSProbe(int);

  void setRefVoltage(float);
  void setRefTemperature(float);

  float update();
  float triggerTdsRead();

  bool tdsReadingAvailable();

  void readKValueFromChip();
};

class pHProbe {

  // Empirically derived values -- place the pH probe in buffer solution and let it sit for ~10m. Then, take a multimeter and measure ground to Po for a voltage
  // multiply by 1024 / 5.12 (since the 5v wall transformer was outputing 5.12v on that rail) for an equivalent analogue measure value.
  const static int pH4Val = 620;
  const static int pH7Val = 512;
  const static int pH10Val = 410;
  constexpr static double lowStep = -.0294; // interpolated value from pH 7 and pH 10
  constexpr static double highStep = -.0277; // interpolated value from pH 4 and pH 7
  constexpr static double lowStepFrom7 = 7.0 - lowStep * pH7Val;
  constexpr static double highStepFrom7 = 7.0 - highStep * pH7Val;

  const int smoothingMeasurements = 20; // The maximum number of measurements for smoothing purposes.

public:
  pHProbe(int);

  float update();
  float triggerPHRead();
  bool pHReadingAvailable(); 
};

enum class ProbeLocation : uint8_t {
  TANK,
  BOX,
  OUTSIDE,
  INSIDE
};

enum class ProbeType : uint8_t {
  TEMPERATURE,
  HUMIDITY,
  TDS,
  PH
};

class Logger {
public:
  Logger() {
    //Console.buffer(1024);
  }
  void transfer(ProbeLocation location, ProbeType type, float value) {
    Console.print(char('0' + int(location)));
    Console.print("-");
    Console.print(char('0' + int(type)));
    Console.print("-");
    Console.print(value);
    Console.print("\n");
  }
};

/** A Dallas Temperature DS18B20 probe management class */
class DTProbes {
  struct Device {
    uint8_t device_address[8];
    uint8_t last_known_scratch[9];
    Device() {
      memset(device_address, 0, sizeof device_address);
      memset(last_known_scratch, 0, sizeof last_known_scratch);
    }
  };
  Device* devices;
  OneWire _wire;
  int num_devices;

  static constexpr int READSCRATCH = 0xBE;
  static constexpr int WRITESCRATCH = 0x4E;
  static constexpr int COPYSCRATCH = 0x48;
  static constexpr int CONVERTT = 0x44;

  static constexpr int RESOLUTION_TO_VALUE[4]{ 9, 10, 11, 12 }; // scrach bits are 00 = 9, 01 = 10, 10 = 11, 11 = 12.
  static constexpr int TEMP_BYTE_LSB = 0;
  static constexpr int TEMP_BYTE_MSB = 1;
  static constexpr int HIGH_ALARM_BYTE = 2;
  static constexpr int LOW_ALARM_BYTE = 3;
  static constexpr int RESOLUTION_BYTE = 4;
  static constexpr int SCRATCHPAD_CRC_BYTE = 8;

  int getScratch(Device*);
  void writeScratch(Device*);

  public:
    static constexpr int BROKENWIRE = -1;

    DTProbes(int digitalPin) : _wire(OneWire(digitalPin)) {};
    void begin();

    int getResolution(int);
    void setResolution(int);
    void requestTemperatures();
    bool isConversionComplete();
    float getTempCByIndex(int);
    void burnUserData(int);

    const int getDeviceCount() const {
      return num_devices;
    }
};

float divideFloatByPow2(float, int);
uint8_t crc8(const uint8_t*, uint8_t);
#endif
