// Pferdedecken-Monitor – Firmware v2
// Board: "Adafruit Feather nRF52840 Express" (Pro Micro nRF52840) + SHT40
// Neu in v2: Senden läuft in loop() (nicht im BLE-Callback), Teilschreibvorgänge werden wiederholt.

#include <bluefruit.h>
#include "nrf_gpio.h"

// ---------- Einstellungen ----------
#define BLANKET_NAME   "Ashley"      // zweite Decke: "Donni"
#define MEAS_INTERVAL_S 60UL         // zum Testen 60; im Einsatz 3600 (stündlich)
#define MAX_RECORDS    336           // 336 Stunden = 14 Tage

// ---------- Software-I2C (Rohpins) ----------
#define SDA_PIN NRF_GPIO_PIN_MAP(0, 20)
#define SCL_PIN NRF_GPIO_PIN_MAP(0, 17)
#define I2C_T   5
#define SHT_ADDR 0x44

static inline void sdaHigh() { nrf_gpio_cfg_input(SDA_PIN, NRF_GPIO_PIN_PULLUP); }
static inline void sdaLow()  { nrf_gpio_pin_clear(SDA_PIN); nrf_gpio_cfg_output(SDA_PIN); }
static inline void sclHigh() { nrf_gpio_cfg_input(SCL_PIN, NRF_GPIO_PIN_PULLUP); }
static inline void sclLow()  { nrf_gpio_pin_clear(SCL_PIN); nrf_gpio_cfg_output(SCL_PIN); }
static inline bool sdaRead() { return nrf_gpio_pin_read(SDA_PIN); }

static void i2cStart() { sdaHigh(); sclHigh(); delayMicroseconds(I2C_T); sdaLow(); delayMicroseconds(I2C_T); sclLow(); }
static void i2cStop()  { sdaLow(); delayMicroseconds(I2C_T); sclHigh(); delayMicroseconds(I2C_T); sdaHigh(); delayMicroseconds(I2C_T); }

static bool i2cWriteByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    if (b & (1 << i)) sdaHigh(); else sdaLow();
    delayMicroseconds(I2C_T); sclHigh(); delayMicroseconds(I2C_T); sclLow();
  }
  sdaHigh(); delayMicroseconds(I2C_T); sclHigh(); delayMicroseconds(I2C_T);
  bool ack = !sdaRead();
  sclLow();
  return ack;
}

static uint8_t i2cReadByte(bool ack) {
  uint8_t b = 0;
  sdaHigh();
  for (int i = 0; i < 8; i++) {
    delayMicroseconds(I2C_T); sclHigh(); delayMicroseconds(I2C_T);
    b = (b << 1) | (sdaRead() ? 1 : 0);
    sclLow();
  }
  if (ack) sdaLow(); else sdaHigh();
  delayMicroseconds(I2C_T); sclHigh(); delayMicroseconds(I2C_T); sclLow();
  sdaHigh();
  return b;
}

static uint8_t crc8(const uint8_t *d, int n) {
  uint8_t c = 0xFF;
  for (int i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c & 0x80) ? (c << 1) ^ 0x31 : (c << 1);
  }
  return c;
}

// liefert true bei gültiger Messung (CRC ok, plausibel)
static bool readSHT40(float &t, float &rh) {
  i2cStart();
  if (!i2cWriteByte(SHT_ADDR << 1)) { i2cStop(); return false; }
  if (!i2cWriteByte(0xFD)) { i2cStop(); return false; }
  i2cStop();
  delay(12);
  i2cStart();
  if (!i2cWriteByte((SHT_ADDR << 1) | 1)) { i2cStop(); return false; }
  uint8_t d[6];
  for (int i = 0; i < 6; i++) d[i] = i2cReadByte(i < 5);
  i2cStop();
  if (crc8(d, 2) != d[2] || crc8(d + 3, 2) != d[5]) return false;
  uint16_t rt = (d[0] << 8) | d[1], rr = (d[3] << 8) | d[4];
  t  = -45.0f + 175.0f * rt / 65535.0f;
  rh = -6.0f + 125.0f * rr / 65535.0f;
  if (rh > 100.0f) rh = 100.0f;
  if (rh < 0.0f) rh = 0.0f;
  return (t > -40.0f && t < 85.0f && rh >= 1.0f);   // 0 % gilt als Lesefehler
}

// ---------- Ringpuffer ----------
struct Rec { int16_t t100; uint16_t rh100; };
static Rec recs[MAX_RECORDS];
static uint16_t head = 0;      // nächster Schreibplatz
static uint16_t count = 0;
static uint32_t lastMeasMs = 0;
static bool everMeasured = false;

static void storeRecord(float t, float rh) {
  recs[head].t100  = (int16_t)lroundf(t * 100.0f);
  recs[head].rh100 = (uint16_t)lroundf(rh * 100.0f);
  head = (head + 1) % MAX_RECORDS;
  if (count < MAX_RECORDS) count++;
}

static void measureNow() {
  float t, rh;
  for (int tries = 0; tries < 3; tries++) {
    if (readSHT40(t, rh)) { storeRecord(t, rh); break; }
    delay(20);
  }
  lastMeasMs = millis();       // auch bei Fehlschlag, sonst Dauermessen
  everMeasured = true;
}

// ---------- BLE ----------
BLEUart bleuart;
volatile bool sendRequested = false;

static void rx_cb(uint16_t conn_hdl) {
  (void)conn_hdl;
  while (bleuart.available()) {
    int c = bleuart.read();
    if (c == 'G') sendRequested = true;   // nur Flag setzen, Senden macht loop()
  }
}

// sendet eine Zeile vollständig; wiederholt bei vollem Puffer
static bool sendLine(const char *s) {
  size_t len = strlen(s), off = 0;
  uint32_t t0 = millis();
  while (off < len) {
    if (!Bluefruit.connected()) return false;
    size_t n = bleuart.write((const uint8_t *)s + off, len - off);
    off += n;
    if (n == 0) {
      delay(20);
      if (millis() - t0 > 3000) return false;
    }
  }
  delay(15);                               // dem Funk Luft lassen
  return true;
}

static void sendAll() {
  char line[48];
  uint32_t age = everMeasured ? (millis() - lastMeasMs) / 1000UL : 0;
  snprintf(line, sizeof(line), "H,%s,%u,%lu,%lu\n", BLANKET_NAME, (unsigned)count,
           (unsigned long)MEAS_INTERVAL_S, (unsigned long)age);
  if (!sendLine(line)) return;
  uint16_t start = (head + MAX_RECORDS - count) % MAX_RECORDS;   // ältester Eintrag
  for (uint16_t i = 0; i < count; i++) {
    const Rec &r = recs[(start + i) % MAX_RECORDS];
    snprintf(line, sizeof(line), "D,%d,%u\n", (int)r.t100, (unsigned)r.rh100);
    if (!sendLine(line)) return;
  }
  sendLine("E\n");
}

static void startAdv() {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(1600, 3200);   // 1 s schnell, 2 s langsam (0,625 ms Einheiten)
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void setup() {
  Bluefruit.autoConnLed(false);
  Bluefruit.begin();
  Bluefruit.setTxPower(0);
  Bluefruit.setName(BLANKET_NAME);
  bleuart.begin();
  bleuart.setRxCallback(rx_cb);
  startAdv();
  measureNow();
}

void loop() {
  if (sendRequested) {
    sendRequested = false;
    if (Bluefruit.connected()) sendAll();
  }
  if (millis() - lastMeasMs >= MEAS_INTERVAL_S * 1000UL) measureNow();
  delay(Bluefruit.connected() ? 50 : 1000);
}
