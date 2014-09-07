/* Functions for talking to the DS18B20 temperature sensor */

#include "TDCtalk.h"

// Initialize DS and output its address into an 8-byte array
// Return false if failed
bool DS_init(byte *addr) {
  if ( !onewire.search(addr)) {
    if (DEBUG) {
      Serial.println(F("Temp sensor: no addresses."));
    }
    onewire.reset_search();
    return 0;;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    if (DEBUG) {
      Serial.println(F("CRC is not valid!"));
    }
    return 0;;
  }

  return 1;
}

void DS_start_conversion(byte* addr) {
  onewire.reset();
  onewire.select(addr);
  onewire.write(0x44);
}

float DS_read_temp_celsius(byte* addr) {
  onewire.reset();
  byte data[12];
  int i;

  onewire.select(addr);
  onewire.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {
    data[i] = onewire.read();
  }
  // Convert the data to temperature
  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms

  return (float)raw / 16.0;
}

// Return temp in celcius from DS sensor if it's ready
// Otherwise, return the last temp
float DS_update_temp_celcius()
{
  static byte addr[8];
  static float celsius = 0;
  static bool ok = false;
  if (!ok) {
    // attempt to connect
    ok = DS_init(addr);
    if (ok) {
      DS_start_conversion(addr);
      if(DEBUG) {
          Serial.println(F("Connected to DS sensor"));
      }
    }
  }
  if (ok) {
    byte res = onewire.read();
    if (res) {
      celsius = DS_read_temp_celsius(addr);
      DS_start_conversion(addr);
    }
  }

  return celsius;
}

