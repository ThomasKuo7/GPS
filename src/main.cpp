#include <Arduino.h>

// Holybro H-RTK F9P Helical (UART2 connector) -> ESP32
//   GPS VCC -> ESP32 5V / VIN   (module needs 5V, 3.3V is not enough)
//   GPS RX2 -> ESP32 GPIO17
//   GPS TX2 -> ESP32 GPIO16
//   GPS GND -> ESP32 GND
//   NC, PPS -> leave unconnected
//
// The ZED-F9P's UART2 does NOT send NMEA by default (RTCM only).
// On startup this sketch sends one config message that turns NMEA on for
// UART2 and saves it to the GPS's flash, then prints everything the GPS sends.

#define GPS_RX 26
#define GPS_TX 27

// Baud rates to try (UART2 default is 38400)
const uint32_t BAUDS[] = {38400, 9600, 115200, 230400, 460800};

// UBX CFG-VALSET payload: list of (key, value) settings
const uint8_t CONFIG[] = {
  0x00,                          // version
  0x07,                          // save to RAM + battery-backed RAM + flash
  0x00, 0x00,                    // reserved
  0x01, 0x00, 0x76, 0x10, 0x01,  // CFG-UART2OUTPROT-UBX  = on (so we get an ACK)
  0x02, 0x00, 0x76, 0x10, 0x01,  // CFG-UART2OUTPROT-NMEA = on
  0xBC, 0x00, 0x91, 0x20, 0x01,  // NMEA GGA on UART2 every fix
  0xAD, 0x00, 0x91, 0x20, 0x01,  // NMEA RMC on UART2 every fix
};

// Send a UBX message (adds header, length and checksum)
void sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t head[] = {cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  uint8_t ckA = 0, ckB = 0;
  for (uint8_t b : head)        { ckA += b; ckB += ckA; }
  for (int i = 0; i < len; i++) { ckA += payload[i]; ckB += ckA; }

  Serial2.write(0xB5);
  Serial2.write(0x62);
  Serial2.write(head, 4);
  Serial2.write(payload, len);
  Serial2.write(ckA);
  Serial2.write(ckB);
}

// Wait for the GPS to answer: 1 = ACK (accepted), 0 = NAK (rejected), -1 = no reply
int waitForAck(uint32_t timeoutMs) {
  uint8_t last[4] = {0};
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (Serial2.available()) {
      memmove(last, last + 1, 3);
      last[3] = Serial2.read();
      if (last[0] == 0xB5 && last[1] == 0x62 && last[2] == 0x05) {
        return last[3] == 0x01 ? 1 : 0;
      }
    }
  }
  return -1;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nConfiguring GPS UART2...");

  bool ok = false;
  for (uint32_t baud : BAUDS) {
    Serial.printf("Trying %lu baud... ", (unsigned long)baud);
    Serial2.end();
    Serial2.begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(100);
    while (Serial2.available()) Serial2.read();  // clear old data

    sendUbx(0x06, 0x8A, CONFIG, sizeof(CONFIG));  // CFG-VALSET
    int result = waitForAck(1000);

    if (result == 1) { Serial.println("ACK - NMEA enabled and saved!"); ok = true; break; }
    if (result == 0) { Serial.println("NAK - GPS rejected the config"); }
    else             { Serial.println("no reply"); }
  }

  if (!ok) {
    Serial.println("Could not configure GPS. Check wiring (TX/RX crossed?) and 5V power.");
  }
  Serial.println("--- GPS output ---");
}

void loop() {
  // Copy everything the GPS sends to the serial monitor
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
}
