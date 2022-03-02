/*
 Name:		GreenhouseLibraries.cpp
 Created:	5/3/2019 1:44:33 PM
 Author:	bgbnbigben
 Editor:	http://www.visualmicro.com
*/

#include "GreenhouseLibraries.h"
#include <Arduino.h>

namespace {
  volatile int adcValue = 0;
  volatile int tdsAnalogPin = 0;
  volatile int phAnalogPin = 0;
  volatile boolean adcConversionWasTDS = false; // true => ADC conversion was triggered by TDS, false => triggered by pH probe.
}

/** Sets the reference voltage for the probe. Probably always 5v, but might fluctuate. 
  *
  * After using this function you'll probably need to call TDSProbe::update()
  */
void TDSProbe::setRefVoltage(float refVoltage) {
  this->refVoltage = refVoltage;
}

/** Sets the reference temperature for the probe. Reasonably, call this before getting the TDS value.
*
* After using this function you'll probably need to call TDSProbe::update()
*/
void TDSProbe::setRefTemperature(float refTemperature) {
  this->refTemperature = refTemperature;
}

/** Reads the K value from the chip's EEPROM.
  * 
  * It's possible that the chip's EEPROM isn't stable, in which case maybe I just hardcode a K value.
  * */
void TDSProbe::readKValueFromChip() {
  // Per data sheet, if there are 4 bytes of 0xFF then the sensor has gone insane.
  if (EEPROM.read(this->kValueAddress) == 0xFF && EEPROM.read(this->kValueAddress + 1) == 0xFF && EEPROM.read(this->kValueAddress + 2) == 0xFF && EEPROM.read(this->kValueAddress + 3) == 0xFF) {
    EEPROM_write(this->kValueAddress, this->kValue);
  }
  EEPROM_read(this->kValueAddress, this->kValue);

}

/** Initialise the probe on an ANALOG pin. */
TDSProbe::TDSProbe(int pin) : refVoltage(5.0), refTemperature(25.0) {
  readKValueFromChip();
  tdsAnalogPin = analogPinToChannel(pin >= 18 ? pin - 18 : pin);

  SREG |= (1 << 7);
  sbi(ADCSRA, ADIE);
}

/** Updates the TDS value for given calibration values (voltage, temperature).
  *
  * Does not re-read the value on the pin, but rather simply recalculates given new calibration parameters.
  * Note, however, that since the analog voltage is calculated via interrupts, sequential calls to this function
  *   may not return the same result even though this function is idempotent.
  */
float TDSProbe::update() {
  float voltage = adcValue / this->adcRange * this->refVoltage;
  float ecValue = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * this->kValue;
  float ecValue25 = ecValue / (1.0 + 0.02*(this->refTemperature - 25.0));  // temperature compensation
  return ecValue25 * tdsFactor;
}

/** Triggers a TDS read by setting the ADSC bit in ADCS Register A to 1. */
float TDSProbe::triggerTdsRead() {
  ADMUX = (DEFAULT << 6 | (tdsAnalogPin & 0x1F)); // Default voltage, (implicit) ADLAR = 0, bottom 5(!) bits of pin (ATmega32U4 has MUX4-0 in ADMUX and MUX5 in ADCSRB);
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((tdsAnalogPin >> 3) & 0x01) << MUX5);
  sbi(ADCSRA, ADSC);
}

/** Checks if a TDS reading has been made available.
  *
  * The hardware will flip ADSC in ADCSRA to 0 when the conversion is complete.
  */
bool TDSProbe::tdsReadingAvailable() {
  return (_SFR_BYTE(ADCSRA) & _BV(ADSC)) == 0 && adcConversionWasTDS;
}

ISR(ADC_vect) { // interrupt service routine for Analog->Digital Conversion Complete
  if ((ADMUX & B00011111) == tdsAnalogPin || (ADMUX & B00011111) == phAnalogPin) {
    adcValue = ADCL;
    adcValue |= (ADCH << 8);

    adcConversionWasTDS = (ADMUX & B00011111) == tdsAnalogPin;
  }
};

/** Initialise the probe on an ANALOG pin. */
pHProbe::pHProbe(int pin) {
  phAnalogPin = analogPinToChannel(pin >= 18 ? pin - 18 : pin);

  SREG |= (1 << 7);
  sbi(ADCSRA, ADIE);
}

/** Updates the ph value for given calibration values (voltage, temperature).
  *
  * Does not re-read the value on the pin, but rather simply recalculates given new calibration parameters.
  * Note, however, that since the analog voltage is calculated via interrupts, sequential calls to this function
  *   may not return the same result even though this function is idempotent.
  */
float pHProbe::update() {
  float phStep = adcValue < pHProbe::pH7Val ? pHProbe::lowStep : pHProbe::highStep;
  float interp = adcValue < pHProbe::pH7Val ? pHProbe::lowStepFrom7 : pHProbe::highStepFrom7;
  
  return phStep * adcValue + interp;
}

/** Triggers a pH read by setting the ADSC bit in ADCS Register A to 1. */
float pHProbe::triggerPHRead() {
  ADMUX = (DEFAULT << 6 | (phAnalogPin & 0x1F)); // Default voltage, (implicit) ADLAR = 0, bottom 5(!) bits of pin (ATmega32U4 has MUX4-0 in ADMUX and MUX5 in ADCSRB);
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((phAnalogPin >> 3) & 0x01) << MUX5);
  sbi(ADCSRA, ADSC);
}

/** Checks if a PH reading has been made available.
  *
  * The hardware will flip ADSC in ADCSRA to 0 when the conversion is complete.
  */
bool pHProbe::pHReadingAvailable() {
  return (_SFR_BYTE(ADCSRA) & _BV(ADSC)) == 0 && !adcConversionWasTDS;
}

constexpr int DTProbes::RESOLUTION_TO_VALUE[4];

/** Finds all DTProbes on the OneWire bus.
  * 
  * Runs two scans; one for internal bookkeeping and one to properly name devices.
  */
void DTProbes::begin() {
  _wire.reset();
  uint8_t _[8];
  while (_wire.search(_)) {
    num_devices++;
  }
  _wire.reset();
  devices = new DTProbes::Device[num_devices];
  for (num_devices = 0; _wire.search(devices[num_devices].device_address); num_devices++);
}

/** Gets the resolution of a given DT Device
 * 
 * Uses the last known scratch data; on boot-up these devices are 12-bit
 * resolution but if we've chosen to write something to them then we should have
 * it in last_known_scratch.
 *
 * If data was written to the EEPROM, though, on boot-up it will be recalled.
 */
int DTProbes::getResolution(int deviceIndex) {
  if (devices[deviceIndex].last_known_scratch[RESOLUTION_BYTE] == 0) {
    getScratch(&devices[deviceIndex]);
  }
  return DTProbes::RESOLUTION_TO_VALUE[(devices[deviceIndex].last_known_scratch[RESOLUTION_BYTE] & (B11 << 5)) >> 5];
}

/** Sets the resolution of all of the DT devices to the given value.
  *
  * Must be an int in [9, 12].
  *
  * I think my slightly-off-brand probes don't actually respect this value for
  * the conversion and instead just truncate. While normally a 9-bit read should
  * max out at 93.75ms, compared to the max 750ms for a 12-bit read, I think
  * these probes do a 12-bit read then just truncate the LSB bytes as necessary.
  * Unfortunate.
  */
void DTProbes::setResolution(int numBits) {
  numBits = max(min(numBits, 12), 9);
  for (int i = 0; i < num_devices; i++) {
    getScratch(&devices[i]);
    devices[i].last_known_scratch[RESOLUTION_BYTE] = ((numBits - 9) << 5);
    writeScratch(&devices[i]);
  }
}

/** Requests temperature conversions from all probes on the bus */
void DTProbes::requestTemperatures() {
  _wire.reset();
  _wire.skip();
  _wire.write(CONVERTT);
}

/** Returns true if ANY device has completed conversion
 *
 * From the data sheet, "If the DS18B20 is powered by an external supply, the
 * master can issue read time slots after the Convert T command and the DS18B20
 * will respond by transmitting a 0 while the temperature conversion is in
 * progress and a 1 when the conversion is done."

 * This actually means it will CONSTANTLY transmit a 1 on a read pulse, until
 * the reset command is issued. YIKES.
 */
bool DTProbes::isConversionComplete() {
  return _wire.read_bit() == 1;
}

/** Returns the temperature of a given probe.
  *
  * Assumes i < numDevices
  * Returns DTProbes::BROKENWIRE if we couldn't retrieve scratch or it failed crc.
  */
float DTProbes::getTempCByIndex(int i) {
  DTProbes::Device device = devices[i];
  int error = getScratch(&device);
  if (error) return BROKENWIRE;

  int16_t fpTemperature = (((int16_t)device.last_known_scratch[TEMP_BYTE_MSB]) << 11)
    | (((int16_t)device.last_known_scratch[TEMP_BYTE_LSB]) << 3);

  // now we have a 16-bit sign-extended two's compliment value. Fuck me.
  // The Celcius value is the raw fpTemperature value / 128.
  return divideFloatByPow2(fpTemperature, 7);
}

/** Gets the scratch for a given DTProbes::Device
  *
  * Returns BROKENWIRE if there are no devices on the bus. In theory should
  * return the same if we ask for a garbo device.
  * Returns 1 if we fail CRC.
  */
int DTProbes::getScratch(DTProbes::Device* device) {
  if (_wire.reset() == 0) return BROKENWIRE;
  _wire.select(device->device_address);
  _wire.write(READSCRATCH);

  // Read all registers in a simple loop
  // byte 0: temperature LSB
  // byte 1: temperature MSB
  // byte 2: high alarm temp
  // byte 3: low alarm temp
  // byte 4: DS18S20: store for crc
  //         DS18B20 & DS1822: configuration register
  // byte 5: internal use & crc
  // byte 6: DS18S20: COUNT_REMAIN
  //         DS18B20 & DS1822: store for crc
  // byte 7: DS18S20: COUNT_PER_C
  //         DS18B20 & DS1822: store for crc
  // byte 8: SCRATCHPAD_CRC
  for (uint8_t i = 0; i < 9; i++) {
    device->last_known_scratch[i] = _wire.read();
  }

  return crc8(device->last_known_scratch, 9);
}

/** Writes the contents of the last known scratchpad to the device. Does not
  * save to EEPROM
  */
void DTProbes::writeScratch(DTProbes::Device* device) {
  _wire.reset();
  _wire.select(device->device_address);
  _wire.write(WRITESCRATCH);
  _wire.write(device->last_known_scratch[HIGH_ALARM_BYTE]);
  _wire.write(device->last_known_scratch[LOW_ALARM_BYTE]);
  _wire.write(device->last_known_scratch[RESOLUTION_BYTE]);
}

/** Writes the contents of the "userdata" registers (Alarm low, Alarm high,
  * resolution) to the device EEPROM. Does not transfer any data.
  */
void DTProbes::burnUserData(int i) {
  _wire.reset();
  _wire.select(devices[i].device_address);
  _wire.write(COPYSCRATCH);
  while (!_wire.read_bit());
}

float divideFloatByPow2(float in, int exponent) {
  // https://stackoverflow.com/a/15902851
  uint32_t yi = *((uint32_t *)&in);                 // get float value as bits
  uint32_t exponent_bits = yi & 0x7f800000;         // extract exponent bits 30..23
  exponent_bits -= (((uint32_t)exponent) << 23);    // sub n from exponent for div by 2^n
  yi = yi & ~0x7f800000 | exponent_bits;            // insert modified exponent back into bits 30..23
  return *(float *)&yi;                             // copy bits back to float
}

// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t PROGMEM dscrc2x16_table[] = {
  0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
  0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
  0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
  0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

uint8_t crc_array[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

/* Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
 * and the registers.
 *
 * This function takes its data in LSB-first format, since it uses the generator
 * 0x8C which is the LSBF/reversed form of the expected generator 0x31. It uses
 * the polynomial x^8 + x^5 + x^4 + x^0. (Note that the DS18B20 probes seem to
 * offer their checksum in this format anyway, since it makes sense.)
 *
 * If the data/crc comes in MSB-first format, use
 * __builtin_avr_insert_bits(0x01234567, byte, 0) to reverse them.
 */
uint8_t crc8(const uint8_t *addr, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc = crc_array[*addr++ ^ crc];
  }
  return crc;
}
