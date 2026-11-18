// Pferdedecken-Monitor – Firmware v3
// Board: "Adafruit Feather nRF52840 Express" (Pro Micro nRF52840) + SHT40
//
// Neu in v3:
//  - Jeder Messwert hat einen echten Zeitstempel (die App schickt beim Abholen die aktuelle Uhrzeit).
//  - Die App fragt nur nach NEUEN Werten ("G,<jetzt>,<letzter Zeitstempel>") -> Abholen dauert Sekunden.
//  - Die Werte werden im Flash gesichert und überleben Neustart und leeren Akku.
//  - Ringpuffer: 1024 Werte = ca. 42 Tage bei stündlicher Messung.
// Diese Version funktioniert nur mit der aktualisierten App (index.html, Version 3).

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "nrf_gpio.h"
using namespace Adafruit_LittleFS_Namespace;

// ---------- Einstellungen ----------
#define BLANKET_NAME     "Ashley"    // zweite Decke: "Donni"
#define MEAS_INTERVAL_S  60UL        // zum Testen 60; im Einsatz 3600 (stündlich)
#define MAX_RECORDS      1024        // Größe des Ringpuffers
#define FLUSH_INTERVAL_S 21600UL     // Flash alle 6 h sichern; zum Testen 300 (5 min) einstellen

// ---------- Software-I2C (Rohpins) ----------
#define SDA_PIN NRF_GPIO_PIN_MAP(0, 17)
#define SCL_PIN NRF_GPIO_PIN_MAP(0, 20)
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

// ---------- Zeit ----------
#define PROV 0x80000000UL            // Markierung: Zeitstempel noch ohne Uhrzeit (Sekunden seit Start)
static uint32_t lastMillis = 0, millisWraps = 0;
static uint32_t epochBase = 0;       // Unix-Zeit bei Sekunde 0 des laufenden Starts; 0 = Uhrzeit unbekannt

static uint32_t upSec() {            // Sekunden seit Start, auch über den millis()-Überlauf hinweg
  uint32_t m = millis();
  if (m < lastMillis) millisWraps++;
  lastMillis = m;
  return (uint32_t)((((uint64_t)millisWraps << 32) | m) / 1000ULL);
}

// ---------- Ringpuffer ----------
struct Rec { uint32_t ts; int16_t t100; uint16_t rh100; };
static Rec recs[MAX_RECORDS];
static uint16_t head = 0;            // nächster Schreibplatz
static uint16_t count = 0;
static uint32_t lastMeasMs = 0, lastFlushMs = 0;
static bool dirty = false;

// i = 0: ältester Eintrag (als Makro, weil die Arduino IDE sonst vor der Definition von "Rec" einen Prototyp einfügt)
#define recAt(i) recs[(head + MAX_RECORDS - count + (i)) % MAX_RECORDS]

static void storeRecord(float t, float rh) {
  uint32_t u = upSec();
  recs[head].ts    = epochBase ? epochBase + u : (PROV | u);
  recs[head].t100  = (int16_t)lroundf(t * 100.0f);
  recs[head].rh100 = (uint16_t)lroundf(rh * 100.0f);
  head = (head + 1) % MAX_RECORDS;
  if (count < MAX_RECORDS) count++;
  dirty = true;
}

static void measureNow() {
  float t, rh;
  for (int tries = 0; tries < 3; tries++) {
    if (readSHT40(t, rh)) { storeRecord(t, rh); break; }
    delay(20);
  }
  lastMeasMs = millis();       // auch bei Fehlschlag, sonst Dauermessen
}

// Uhrzeit von der App übernehmen; Werte ohne Uhrzeit bekommen jetzt ihren echten Zeitstempel
static void syncTime(uint32_t unixNow) {
  if (unixNow < 1600000000UL) return;           // unplausibel (vor 2020)
  uint32_t u = upSec();
  if (unixNow < u) return;
  bool first = (epochBase == 0);
  epochBase = unixNow - u;
  if (first) {
    for (uint16_t i = 0; i < count; i++) {
      Rec &r = recAt(i);
      if (r.ts & PROV) { r.ts = epochBase + (r.ts & ~PROV); dirty = true; }
    }
  }
}

// ---------- Flash ----------
#define FS_FILE  "/rec.bin"
#define FS_MAGIC 0x33444B50UL        // "PKD3"
struct FileHdr { uint32_t magic; uint32_t count; };

static void flushToFlash() {
  InternalFS.remove(FS_FILE);
  File f(InternalFS);
  if (!f.open(FS_FILE, FILE_O_WRITE)) return;
  FileHdr h = { FS_MAGIC, count };
  f.write((uint8_t *)&h, sizeof(h));
  Rec chunk[32];
  uint16_t n = 0;
  for (uint16_t i = 0; i < count; i++) {
    chunk[n++] = recAt(i);
    if (n == 32 || i == count - 1) { f.write((uint8_t *)chunk, n * sizeof(Rec)); n = 0; }
  }
  f.close();
  dirty = false;
  lastFlushMs = millis();
}

static void loadFromFlash() {
  File f(InternalFS);
  if (!f.open(FS_FILE, FILE_O_READ)) return;
  FileHdr h;
  if (f.read((uint8_t *)&h, sizeof(h)) != (int)sizeof(h) || h.magic != FS_MAGIC || h.count > MAX_RECORDS) { f.close(); return; }
  head = 0; count = 0;
  Rec r;
  for (uint32_t i = 0; i < h.count; i++) {
    if (f.read((uint8_t *)&r, sizeof(r)) != (int)sizeof(r)) break;
    if (r.ts & PROV) continue;                  // Werte ohne Uhrzeit aus einem früheren Start sind nicht mehr zuordenbar
    recs[head] = r;
    head = (head + 1) % MAX_RECORDS;
    count++;
  }
  f.close();
}

// ---------- BLE ----------
BLEUart bleuart;
static char cmdBuf[48];
static uint8_t cmdLen = 0;
static volatile uint32_t reqUnix = 0, reqSince = 0;
static volatile bool sendRequested = false;

static void parseCmd() {
  if (cmdBuf[0] != 'G') return;
  uint32_t u = 0, s = 0;
  char *p = strchr(cmdBuf, ',');
  if (p) {
    char *q;
    u = strtoul(p + 1, &q, 10);
    if (*q == ',') s = strtoul(q + 1, NULL, 10);
  }
  reqUnix = u; reqSince = s;
  sendRequested = true;                         // nur Flag setzen, Senden macht loop()
}

static void rx_cb(uint16_t conn_hdl) {
  (void)conn_hdl;
  while (bleuart.available()) {
    int c = bleuart.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen) { cmdBuf[cmdLen] = 0; parseCmd(); cmdLen = 0; }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = (char)c;
    }
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
  delay(8);                                // dem Funk Luft lassen
  return true;
}

static void sendAll(uint32_t since) {
  char line[64];
  uint32_t n = 0;
  for (uint16_t i = 0; i < count; i++) {
    const Rec &r = recAt(i);
    if (!(r.ts & PROV) && r.ts > since) n++;
  }
  uint32_t nowUnix = epochBase ? epochBase + upSec() : 0;
  snprintf(line, sizeof(line), "H,%s,%lu,%lu,%lu,3\n", BLANKET_NAME,
           (unsigned long)n, (unsigned long)MEAS_INTERVAL_S, (unsigned long)nowUnix);
  if (!sendLine(line)) return;
  for (uint16_t i = 0; i < count; i++) {
    const Rec &r = recAt(i);
    if ((r.ts & PROV) || r.ts <= since) continue;
    snprintf(line, sizeof(line), "D,%lu,%d,%u\n", (unsigned long)r.ts, (int)r.t100, (unsigned)r.rh100);
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
  InternalFS.begin();
  loadFromFlash();
  Bluefruit.autoConnLed(false);
  Bluefruit.begin();
  Bluefruit.setTxPower(0);
  Bluefruit.setName(BLANKET_NAME);
  bleuart.begin();
  bleuart.setRxCallback(rx_cb);
  startAdv();
  measureNow();
  lastFlushMs = millis();
}

void loop() {
  upSec();                                      // hält die Laufzeit über den millis()-Überlauf aktuell
  if (sendRequested) {
    sendRequested = false;
    if (Bluefruit.connected()) {
      syncTime(reqUnix);
      sendAll(reqSince);
    }
  }
  if (millis() - lastMeasMs >= MEAS_INTERVAL_S * 1000UL) measureNow();
  if (dirty && !Bluefruit.connected() && millis() - lastFlushMs >= FLUSH_INTERVAL_S * 1000UL) flushToFlash();
  delay(Bluefruit.connected() ? 50 : 1000);
}
