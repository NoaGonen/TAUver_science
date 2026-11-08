/*
 * ============================================================================
 *  TAUVER Link Kit · firmware scale-1.0.0 · Mission 2: one real scale
 *  Board: NUCLEO-G491RE  (also builds for an Arduino Mega)
 *  Sensor: NAU7802 24-bit load-cell ADC (I2C 0x2A) + one 4-wire load cell
 * ============================================================================
 *
 *  WHAT IT DOES
 *    Everything the pinger did (heartbeat, PING/PONG, LED, fake lid), plus a
 *    real scale:
 *      - live weight in every heartbeat, 5 times a second ("rock_g")
 *      - CMD:TARE          empty scale -> this reading is zero
 *      - CMD:CALW=<grams>  known mass on -> learn counts per gram
 *      - CMD:WEIGH         one averaged, stable-checked reading
 *    Tare and calibration are saved in flash, so they survive a reset.
 *    The closed-loop lid demo now runs on REAL weight: put >= 100 g on.
 *
 *  WIRING (see the chat / README for the picture)
 *    NAU7802 VIN/VCC -> Nucleo 3V3       NAU7802 GND -> Nucleo GND
 *    NAU7802 SDA     -> D14 = PB9         NAU7802 SCL -> D15 = PB8
 *    Load cell: red -> E+, black -> E-, white -> A-, green -> A+
 *    (A+ and A- swapped only flips the sign; calibration handles it.)
 *
 *  ARDUINO IDE: same settings as the pinger
 *    Generic STM32G4 series · Generic G491RETx · U(S)ART: Enabled (generic 'Serial')
 *    · USB support: None · Export Compiled Binary · drag the .bin onto NOD_G491RE
 *    No extra library needed: the NAU7802 driver is in this file.
 *
 *  THE WIRE CONTRACT (same shape as Science Pack serial contract v3)
 *    up: one JSON object per line, every frame has "type"
 *    down: one command per line, answered by an "ack"
 *
 *  COMMANDS (case-insensitive, one per line, 115200 baud)
 *    CMD:PING[=id] · CMD:LED=ON|OFF|AUTO · CMD:LID=ROCK,CLOSE|OPEN
 *    CMD:TARE · CMD:CALW=<grams> · CMD:CAL_CLEAR · CMD:WEIGH · CMD:SCALE_RESET
 *    CMD:SIM=ON|OFF (kept for the station's lid demo; the weight is real now)
 *    CMD:STATUS · CMD:HELP · CMD:RESET
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

// ---------------------------------------------------------------------------
// [A] CONFIG — the only block you should need to touch
// ---------------------------------------------------------------------------
#define FW_VERSION "scale-1.0.0"

const uint32_t BAUD        = 115200;
const uint32_t TEL_MS      = 200;      // heartbeat + live weight, 5 Hz
const uint8_t  LINE_MAX    = 96;

// NAU7802 settings
const uint8_t  NAU_ADDR    = 0x2A;
const uint8_t  NAU_GAIN    = 0b111;    // x128 (the load-cell setting)
const uint8_t  NAU_LDO     = 0b101;    // 3.0 V excitation from the chip's own regulator
const uint8_t  NAU_RATE    = 0b011;    // 80 samples per second
const uint8_t  DISCARD_N   = 8;        // samples thrown away after (re)start

// Filtering and the stability test
const uint8_t  AVG_N       = 8;        // live value = mean of the last 8 samples (0.1 s)
const uint8_t  WIN_N       = 40;       // stability window = last 40 samples (0.5 s)
const float    STABLE_G    = 1.0f;     // stable if the window spans less than this (calibrated)
const int32_t  STABLE_CNT  = 3000;     // ... or less than this many counts (not calibrated yet)

// Jobs (tare / calibrate / weigh) average FRESH samples taken after the command
const uint8_t  TARE_N      = 40;       // 0.5 s
const uint8_t  CAL_N       = 80;       // 1.0 s
const uint8_t  WEIGH_N     = 40;       // 0.5 s
const int32_t  MAX_SPREAD  = 20000;    // counts: more = something moved -> refused
const int32_t  MIN_DELTA   = 1000;     // counts: less = "no weight seen" (wiring?)
const uint32_t JOB_TIMEOUT = 4000;     // ms

const uint32_t RETRY_MS    = 2000;     // look for a missing NAU7802 this often
const uint32_t STALL_MS    = 500;      // no new sample this long = scale lost

#if defined(ARDUINO_ARCH_STM32)
  #define BOARD_NAME "NUCLEO-G491RE"
  const uint32_t PIN_LED = PA5;        // LD2
#else
  #define BOARD_NAME "ARDUINO"
  const uint32_t PIN_LED = LED_BUILTIN;
#endif

// NAU7802 registers and bits (datasheet names)
const uint8_t R_PU_CTRL = 0x00, R_CTRL1 = 0x01, R_CTRL2 = 0x02, R_ADCO_B2 = 0x12,
              R_ADC = 0x15, R_PGA_PWR = 0x1C, R_REV = 0x1F;
const uint8_t PU_RR = 0x01, PU_PUD = 0x02, PU_PUA = 0x04, PU_PUR = 0x08,
              PU_CS = 0x10, PU_CR = 0x20, PU_AVDDS = 0x80;
const uint8_t C2_CALS = 0x04, C2_CAL_ERR = 0x08;

// ---------------------------------------------------------------------------
// [B] STATE
// ---------------------------------------------------------------------------
enum LedMode { LED_AUTO, LED_ON, LED_OFF };
LedMode  ledMode   = LED_AUTO;
bool     lidClosed = false;
bool     demoOn    = false;            // CMD:SIM=ON: the station's lid demo is waiting

uint32_t seq = 0, rxLines = 0, badLines = 0, lastTel = 0;
char     lineBuf[LINE_MAX + 1];
uint8_t  lineLen = 0;
bool     lineOverflow = false;

// scale
bool     scaleOk = false;
uint32_t lastRetry = 0, lastSampleMs = 0, lastPoll = 0, samples = 0;
uint8_t  discard = 0, readFails = 0;
int32_t  win[WIN_N];
uint8_t  winHead = 0, winCount = 0;

// calibration (kept in flash)
struct CalRecord {
  uint32_t magic;
  int32_t  offset;     // raw counts at zero
  float    cpg;        // counts per gram (sign included)
  uint8_t  tared;
  uint8_t  calibrated;
  uint8_t  pad[2];
  uint32_t check;
};
const uint32_t CAL_MAGIC = 0x54415632;   // "TAV2"
CalRecord cal;
bool calLoaded = false;

// one job at a time
enum JobKind { JOB_NONE, JOB_TARE, JOB_CAL, JOB_WEIGH };
JobKind  job = JOB_NONE;
uint8_t  jobN = 0, jobGot = 0;
int64_t  jobSum = 0;
int32_t  jobMin = 0, jobMax = 0;
float    jobGrams = 0;
uint32_t jobStart = 0;
char     jobCmd[LINE_MAX + 1];

// ---------------------------------------------------------------------------
// [C] JSON helpers
// ---------------------------------------------------------------------------
void printJsonStr(const char* s) {
  Serial.print('"');
  for (; *s; s++) {
    char c = *s;
    if (c == '"' || c == '\\') { Serial.print('\\'); Serial.print(c); }
    else if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E) Serial.print('?');
    else Serial.print(c);
  }
  Serial.print('"');
}
void printNum(float v, int digits) {           // NaN -> null
  if (isnan(v) || isinf(v)) Serial.print(F("null")); else Serial.print(v, digits);
}
const char* ledName() { return ledMode == LED_ON ? "ON" : (ledMode == LED_OFF ? "OFF" : "AUTO"); }

void sendAck(const char* cmd, bool ok, const char* msg) {
  Serial.print(F("{\"type\":\"ack\",\"cmd\":")); printJsonStr(cmd);
  Serial.print(F(",\"ok\":")); Serial.print(ok ? 1 : 0);
  Serial.print(F(",\"msg\":")); printJsonStr(msg);
  Serial.println('}');
}

// ---------------------------------------------------------------------------
// [D] CALIBRATION STORAGE (flash, checked)
// ---------------------------------------------------------------------------
uint32_t calCheck(const CalRecord& c) {        // FNV-1a over everything but 'check'
  const uint8_t* p = (const uint8_t*)&c;
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < offsetof(CalRecord, check); i++) { h ^= p[i]; h *= 16777619UL; }
  return h;
}
void calDefaults() { memset(&cal, 0, sizeof(cal)); cal.magic = CAL_MAGIC; cal.cpg = 1.0f; }
void calSave() { cal.magic = CAL_MAGIC; cal.check = calCheck(cal); EEPROM.put(0, cal); }
void calLoad() {
  EEPROM.get(0, cal);
  calLoaded = (cal.magic == CAL_MAGIC && cal.check == calCheck(cal) && cal.cpg != 0.0f && !isnan(cal.cpg));
  if (!calLoaded) calDefaults();
}

// ---------------------------------------------------------------------------
// [E] NAU7802 DRIVER (no library)
// ---------------------------------------------------------------------------
bool nauWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(NAU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool nauRead(uint8_t reg, uint8_t* val) {
  Wire.beginTransmission(NAU_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)NAU_ADDR, (uint8_t)1) != 1) return false;
  *val = (uint8_t)Wire.read();
  return true;
}
bool nauSetBits(uint8_t reg, uint8_t mask) {
  uint8_t v; if (!nauRead(reg, &v)) return false; return nauWrite(reg, v | mask);
}
bool nauReadAdc(int32_t* out) {
  Wire.beginTransmission(NAU_ADDR); Wire.write(R_ADCO_B2);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)NAU_ADDR, (uint8_t)3) != 3) return false;
  uint32_t v = ((uint32_t)Wire.read() << 16);
  v |= ((uint32_t)Wire.read() << 8);
  v |= (uint32_t)Wire.read();
  if (v & 0x800000UL) v |= 0xFF000000UL;       // sign-extend 24 -> 32 bits
  *out = (int32_t)v;
  return true;
}

bool wireStarted = false;
void wireRestart() {
  // STM32duino 3.0.0: one NACK can leave Wire dead until begin() runs again.
  // end() only on a bus that was started: on STM32, end() on a never-started
  // bus touches an uninitialised I2C handle.
  if (wireStarted) Wire.end();
  wireStarted = true;
#if defined(ARDUINO_ARCH_STM32)
  Wire.setSDA(PB9);                            // D14. Set BEFORE begin(): the default is PA8/PA9
  Wire.setSCL(PB8);                            // D15
#endif
  Wire.begin();
  Wire.setClock(100000);
}

// Power-up sequence from the datasheet (section 9.1). Blocks for at most ~0.6 s,
// and only at boot or when the scale comes back.
const char* nauInit() {
  Wire.beginTransmission(NAU_ADDR);
  if (Wire.endTransmission() != 0) return "NOT_FOUND";
  if (!nauWrite(R_PU_CTRL, PU_RR)) return "I2C_ERROR";          // register reset
  delay(1);
  if (!nauWrite(R_PU_CTRL, 0)) return "I2C_ERROR";
  if (!nauWrite(R_PU_CTRL, PU_PUD | PU_PUA)) return "I2C_ERROR"; // power up digital + analog
  uint32_t t0 = millis();
  uint8_t pu = 0;
  do {                                                         // wait for "power-up ready"
    delay(1);
    if (!nauRead(R_PU_CTRL, &pu)) return "I2C_ERROR";
  } while (!(pu & PU_PUR) && millis() - t0 < 200);
  if (!(pu & PU_PUR)) return "NO_POWER_READY";
  if (!nauWrite(R_CTRL1, (uint8_t)((NAU_LDO << 3) | NAU_GAIN))) return "I2C_ERROR";
  if (!nauWrite(R_PU_CTRL, PU_PUD | PU_PUA | PU_AVDDS | PU_CS)) return "I2C_ERROR"; // internal LDO, start
  if (!nauWrite(R_CTRL2, (uint8_t)(NAU_RATE << 4))) return "I2C_ERROR";             // 80 SPS, channel 1
  if (!nauWrite(R_ADC, 0x30)) return "I2C_ERROR";              // chopper clock off (datasheet 9.1)
  if (!nauSetBits(R_PGA_PWR, 0x80)) return "I2C_ERROR";        // 330 pF cap on channel 2
  delay(10);                                                   // let the LDO settle
  if (!nauWrite(R_CTRL2, (uint8_t)((NAU_RATE << 4) | C2_CALS))) return "I2C_ERROR"; // internal offset cal
  t0 = millis();
  uint8_t c2 = C2_CALS;
  do {
    delay(2);
    if (!nauRead(R_CTRL2, &c2)) return "I2C_ERROR";
  } while ((c2 & C2_CALS) && millis() - t0 < 400);
  if (c2 & C2_CALS) return "CAL_TIMEOUT";
  if (c2 & C2_CAL_ERR) return "CAL_ERROR";
  return nullptr;                                              // nullptr = OK
}

// ---------------------------------------------------------------------------
// [F] SCALE LOGIC
// ---------------------------------------------------------------------------
void sendEvent(const char* lvl, const char* code, const char* detail) {
  Serial.print(F("{\"type\":\"event\",\"ms\":")); Serial.print(millis());
  Serial.print(F(",\"lvl\":")); printJsonStr(lvl);
  Serial.print(F(",\"code\":")); printJsonStr(code);
  if (detail) { Serial.print(F(",\"detail\":")); printJsonStr(detail); }
  Serial.println('}');
}

void scaleTryStart() {
  wireRestart();
  const char* err = nauInit();
  if (err == nullptr) {
    uint8_t rev = 0; nauRead(R_REV, &rev);
    scaleOk = true; readFails = 0; discard = DISCARD_N; winCount = 0; winHead = 0;
    lastSampleMs = millis();
    Serial.print(F("{\"type\":\"event\",\"ms\":")); Serial.print(millis());
    Serial.print(F(",\"lvl\":\"info\",\"code\":\"SCALE_UP\",\"rev\":")); Serial.print(rev & 0x0F);
    Serial.println('}');
  } else if (scaleOk || lastRetry == 0) {
    sendEvent("warn", "SCALE_NOT_READY", err);   // once, not every retry
  }
  if (err != nullptr) scaleOk = false;
}

void scaleLost(const char* why) {
  if (!scaleOk) return;
  scaleOk = false;
  winCount = 0;
  sendEvent("warn", "SCALE_LOST", why);
  if (job != JOB_NONE) { sendAck(jobCmd, false, "SCALE_LOST"); job = JOB_NONE; }
  lastRetry = millis();
}

// window statistics
int32_t winMean(uint8_t n) {                   // mean of the newest n samples
  if (winCount == 0) return 0;
  if (n > winCount) n = winCount;
  int64_t s = 0;
  for (uint8_t i = 0; i < n; i++) s += win[(winHead + WIN_N - 1 - i) % WIN_N];
  return (int32_t)(s / n);
}
int32_t winSpread() {
  if (winCount == 0) return 0;
  int32_t lo = win[0], hi = win[0];
  for (uint8_t i = 0; i < winCount; i++) {
    int32_t v = win[(winHead + WIN_N - 1 - i) % WIN_N];
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  return hi - lo;
}
float countsToGrams(float counts) {
  if (!cal.tared || !cal.calibrated) return NAN;
  return (counts - (float)cal.offset) / cal.cpg;
}
bool isStable() {
  if (winCount < WIN_N) return false;
  int32_t sp = winSpread();
  if (cal.calibrated) return fabsf((float)sp / cal.cpg) < STABLE_G;
  return sp < STABLE_CNT;
}
float liveGrams() { return (scaleOk && winCount) ? countsToGrams((float)winMean(AVG_N)) : NAN; }

void jobBegin(JobKind k, uint8_t n, const char* cmd, float grams) {
  job = k; jobN = n; jobGot = 0; jobSum = 0; jobGrams = grams; jobStart = millis();
  strncpy(jobCmd, cmd, LINE_MAX); jobCmd[LINE_MAX] = '\0';
}

void jobFinish() {
  int32_t mean = (int32_t)(jobSum / jobGot);
  int32_t spread = jobMax - jobMin;
  JobKind k = job;
  job = JOB_NONE;

  if (k == JOB_TARE) {
    if (spread > MAX_SPREAD) { sendAck(jobCmd, false, "UNSTABLE_KEEP_IT_STILL"); return; }
    cal.offset = mean; cal.tared = 1; calSave();
    Serial.print(F("{\"type\":\"result\",\"kind\":\"tare\",\"offset\":")); Serial.print(mean);
    Serial.print(F(",\"spread\":")); Serial.print(spread);
    Serial.print(F(",\"n\":")); Serial.print(jobGot);
    Serial.print(F(",\"calibrated\":")); Serial.print(cal.calibrated);
    Serial.println('}');
    sendAck(jobCmd, true, cal.calibrated ? "TARED" : "TARED_NOW_CALIBRATE");
  } else if (k == JOB_CAL) {
    int32_t delta = mean - cal.offset;
    if (spread > MAX_SPREAD) { sendAck(jobCmd, false, "UNSTABLE_KEEP_IT_STILL"); return; }
    if (labs((long)delta) < MIN_DELTA) { sendAck(jobCmd, false, "NO_WEIGHT_SEEN_CHECK_WIRING"); return; }
    cal.cpg = (float)delta / jobGrams; cal.calibrated = 1; calSave();
    Serial.print(F("{\"type\":\"result\",\"kind\":\"cal\",\"grams\":")); printNum(jobGrams, 2);
    Serial.print(F(",\"delta\":")); Serial.print(delta);
    Serial.print(F(",\"cpg\":")); printNum(cal.cpg, 3);
    Serial.print(F(",\"spread\":")); Serial.print(spread);
    Serial.print(F(",\"saved\":1}"));
    Serial.println();
    sendAck(jobCmd, true, "CALIBRATED");
  } else if (k == JOB_WEIGH) {
    float g = countsToGrams((float)mean);
    float spreadG = cal.calibrated ? fabsf((float)spread / cal.cpg) : NAN;
    Serial.print(F("{\"type\":\"result\",\"kind\":\"weigh\",\"grams\":")); printNum(g, 2);
    Serial.print(F(",\"raw\":")); Serial.print(mean);
    Serial.print(F(",\"spread_g\":")); printNum(spreadG, 2);
    Serial.print(F(",\"stable\":")); Serial.print((cal.calibrated && spreadG < STABLE_G) ? 1 : 0);
    Serial.print(F(",\"calibrated\":")); Serial.print(cal.calibrated);
    Serial.println('}');
    sendAck(jobCmd, true, cal.calibrated ? "WEIGHED" : "RAW_ONLY_NOT_CALIBRATED");
  }
}

void onSample(int32_t raw) {
  samples++;
  if (discard) { discard--; return; }
  win[winHead] = raw; winHead = (winHead + 1) % WIN_N;
  if (winCount < WIN_N) winCount++;
  if (job != JOB_NONE) {
    if (jobGot == 0) { jobMin = raw; jobMax = raw; }
    if (raw < jobMin) jobMin = raw;
    if (raw > jobMax) jobMax = raw;
    jobSum += raw; jobGot++;
    if (jobGot >= jobN) jobFinish();
  }
}

void scalePoll(uint32_t now) {
  if (!scaleOk) {
    if (now - lastRetry >= RETRY_MS) { lastRetry = now; scaleTryStart(); }
    return;
  }
  if (now - lastPoll < 2) return;              // ~500 Hz poll; samples come at 80 Hz
  lastPoll = now;
  uint8_t pu;
  if (!nauRead(R_PU_CTRL, &pu)) { if (++readFails >= 3) scaleLost("I2C_ERROR"); return; }
  readFails = 0;
  if (pu & PU_CR) {
    int32_t raw;
    if (!nauReadAdc(&raw)) { scaleLost("I2C_ERROR"); return; }
    lastSampleMs = now;
    onSample(raw);
  } else if (now - lastSampleMs > STALL_MS) {
    scaleLost("NO_NEW_SAMPLES");
  }
  if (job != JOB_NONE && now - jobStart > JOB_TIMEOUT) { sendAck(jobCmd, false, "TIMEOUT"); job = JOB_NONE; }
}

// ---------------------------------------------------------------------------
// [G] FRAMES
// ---------------------------------------------------------------------------
void sendBoot() {
  Serial.print(F("{\"type\":\"boot\",\"v\":3,\"kit\":\"scale\",\"fw\":\"" FW_VERSION "\",\"board\":\"" BOARD_NAME "\",\"baud\":"));
  Serial.print(BAUD);
  Serial.print(F(",\"cal\":")); printJsonStr(calLoaded ? "loaded" : "defaults");
  Serial.println('}');
}

void sendTel(uint32_t now) {
  float g = liveGrams();
  Serial.print(F("{\"type\":\"tel\",\"seq\":")); Serial.print(seq++);
  Serial.print(F(",\"ms\":")); Serial.print(now);
  Serial.print(F(",\"data\":{\"rock_g\":")); printNum(g, 1);
  Serial.print(F(",\"raw\":"));
  if (scaleOk && winCount) Serial.print(winMean(AVG_N)); else Serial.print(F("null"));
  Serial.print(F(",\"stable\":")); Serial.print(isStable() ? 1 : 0);
  Serial.print(F(",\"tared\":")); Serial.print(cal.tared);
  Serial.print(F(",\"cal\":")); Serial.print(cal.calibrated);
  Serial.print(F(",\"scale\":\"")); Serial.print(scaleOk ? F("OK") : F("OFF"));
  Serial.print(F("\",\"lid_rock\":\"")); Serial.print(lidClosed ? F("CLOSED") : F("OPEN"));
  Serial.print(F("\",\"sim\":")); Serial.print(demoOn ? 1 : 0);
  Serial.print(F(",\"led\":\"")); Serial.print(ledName());
  Serial.println(F("\"}}"));
}

void sendHealth() {
  Serial.print(F("{\"type\":\"health\",\"v\":3,\"kit\":\"scale\",\"fw\":\"" FW_VERSION "\",\"up_s\":"));
  Serial.print(millis() / 1000UL);
  Serial.print(F(",\"rx_lines\":")); Serial.print(rxLines);
  Serial.print(F(",\"bad_lines\":")); Serial.print(badLines);
  Serial.print(F(",\"scale\":\"")); Serial.print(scaleOk ? F("OK") : F("OFF"));
  Serial.print(F("\",\"samples\":")); Serial.print(samples);
  Serial.print(F(",\"tared\":")); Serial.print(cal.tared);
  Serial.print(F(",\"cal\":")); Serial.print(cal.calibrated);
  Serial.print(F(",\"offset\":")); Serial.print(cal.offset);
  Serial.print(F(",\"cpg\":")); printNum(cal.calibrated ? cal.cpg : NAN, 3);
  Serial.print(F(",\"spread\":")); Serial.print(winSpread());
  Serial.print(F(",\"lid_rock\":\"")); Serial.print(lidClosed ? F("CLOSED") : F("OPEN"));
  Serial.print(F("\",\"led\":\"")); Serial.print(ledName());
  Serial.println(F("\"}"));
}

void sendHelp() {
  Serial.println(F("{\"type\":\"help\",\"cmds\":[\"CMD:PING[=id]\",\"CMD:LED=ON|OFF|AUTO\",\"CMD:LID=ROCK,CLOSE|OPEN\",\"CMD:TARE\",\"CMD:CALW=<grams>\",\"CMD:CAL_CLEAR\",\"CMD:WEIGH\",\"CMD:SCALE_RESET\",\"CMD:SIM=ON|OFF\",\"CMD:STATUS\",\"CMD:HELP\",\"CMD:RESET\"]}"));
}

// ---------------------------------------------------------------------------
// [H] COMMANDS
// ---------------------------------------------------------------------------
char* trim(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
  return s;
}
bool startsWith(const char* s, const char* p) { return strncmp(s, p, strlen(p)) == 0; }
bool parseId(const char* p, long* out) {
  if (!*p) return false;
  const char* q = p; if (*q == '-') q++;
  if (!*q) return false;
  for (; *q; q++) if (*q < '0' || *q > '9') return false;
  *out = atol(p); return true;
}
bool parseGrams(const char* p, float* out) {      // "100", "100.5"
  if (!*p) return false;
  char* end = nullptr;
  double v = strtod(p, &end);
  if (end == p || *end != '\0' || !(v > 0.0) || v > 100000.0) return false;
  *out = (float)v; return true;
}

void handleLine(char* raw) {
  char* s = trim(raw);
  if (!*s) return;
  char up[LINE_MAX + 1];
  size_t n = strlen(s);
  for (size_t i = 0; i <= n; i++) up[i] = (char)toupper((unsigned char)s[i]);

  if (startsWith(up, "CMD:PING")) {
    const char* p = up + 8; long id = 0; bool hasId = false;
    if (*p == '=') { hasId = parseId(p + 1, &id); if (!hasId) { badLines++; sendAck(s, false, "BAD_ID"); return; } }
    else if (*p != '\0') { badLines++; sendAck(s, false, "UNKNOWN_CMD"); return; }
    rxLines++;
    Serial.print(F("{\"type\":\"pong\",\"id\":"));
    if (hasId) Serial.print(id); else Serial.print(F("null"));
    Serial.print(F(",\"ms\":")); Serial.print(millis()); Serial.println('}');
    sendAck(s, true, "PONG");
    return;
  }
  if (startsWith(up, "CMD:LED=")) {
    const char* a = up + 8;
    if (!strcmp(a, "ON")) ledMode = LED_ON;
    else if (!strcmp(a, "OFF")) ledMode = LED_OFF;
    else if (!strcmp(a, "AUTO")) ledMode = LED_AUTO;
    else { badLines++; sendAck(s, false, "USE_ON_OFF_AUTO"); return; }
    rxLines++; sendAck(s, true, ledName()); return;
  }
  if (startsWith(up, "CMD:LID=")) {
    const char* a = up + 8;
    if (!startsWith(a, "ROCK,")) { badLines++; sendAck(s, false, "UNKNOWN_BAY"); return; }
    if (!strcmp(a + 5, "CLOSE")) { lidClosed = true; demoOn = false; rxLines++; sendAck(s, true, "LID_CLOSED"); sendEvent("info", "LID_CLOSED", nullptr); }
    else if (!strcmp(a + 5, "OPEN")) { lidClosed = false; rxLines++; sendAck(s, true, "LID_OPEN"); sendEvent("info", "LID_OPENED", nullptr); }
    else { badLines++; sendAck(s, false, "USE_CLOSE_OPEN"); }
    return;
  }
  if (startsWith(up, "CMD:SIM=")) {                 // the station's lid demo
    const char* a = up + 8;
    if (!strcmp(a, "ON")) {
      rxLines++;
      if (lidClosed) { sendAck(s, false, "LID_CLOSED_OPEN_IT_FIRST"); return; }
      demoOn = true; sendAck(s, true, "REAL_SCALE_PUT_A_SAMPLE_ON"); sendEvent("info", "SIM_START", nullptr);
    } else if (!strcmp(a, "OFF")) { demoOn = false; rxLines++; sendAck(s, true, "SIM_OFF"); }
    else { badLines++; sendAck(s, false, "USE_ON_OFF"); }
    return;
  }

  // ---- the scale ----
  if (!strcmp(up, "CMD:TARE") || startsWith(up, "CMD:CALW=") || !strcmp(up, "CMD:WEIGH")) {
    rxLines++;
    if (!scaleOk) { sendAck(s, false, "SCALE_OFFLINE"); return; }
    if (job != JOB_NONE) { sendAck(s, false, "BUSY"); return; }
    if (!strcmp(up, "CMD:TARE")) { jobBegin(JOB_TARE, TARE_N, s, 0); return; }
    if (!strcmp(up, "CMD:WEIGH")) { jobBegin(JOB_WEIGH, WEIGH_N, s, 0); return; }
    float g;
    if (!parseGrams(up + 9, &g)) { sendAck(s, false, "USE_CMD:CALW=<grams>"); return; }
    if (!cal.tared) { sendAck(s, false, "TARE_FIRST"); return; }
    jobBegin(JOB_CAL, CAL_N, s, g);
    return;
  }
  if (!strcmp(up, "CMD:CAL_CLEAR")) { rxLines++; calDefaults(); calSave(); sendAck(s, true, "CAL_CLEARED"); return; }
  if (!strcmp(up, "CMD:SCALE_RESET")) {
    rxLines++;
    if (job != JOB_NONE) { sendAck(jobCmd, false, "SCALE_RESET"); job = JOB_NONE; }
    scaleOk = false; scaleTryStart();
    sendAck(s, scaleOk, scaleOk ? "SCALE_OK" : "SCALE_NOT_FOUND");
    return;
  }

  if (!strcmp(up, "CMD:STATUS")) { rxLines++; sendHealth(); sendAck(s, true, "OK"); return; }
  if (!strcmp(up, "CMD:HELP"))   { rxLines++; sendHelp();   sendAck(s, true, "OK"); return; }
  if (!strcmp(up, "CMD:RESET"))  {                  // demo state only; calibration stays
    rxLines++; ledMode = LED_AUTO; lidClosed = false; demoOn = false;
    sendAck(s, true, "RESET"); return;
  }
  badLines++;
  sendAck(s, false, "UNKNOWN_CMD");
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineOverflow) { badLines++; sendAck("?", false, "LINE_TOO_LONG"); }
      else if (lineLen > 0) { lineBuf[lineLen] = '\0'; handleLine(lineBuf); }
      lineLen = 0; lineOverflow = false;
      continue;
    }
    if (lineLen < LINE_MAX) lineBuf[lineLen++] = c; else lineOverflow = true;
  }
}

// ---------------------------------------------------------------------------
// [I] LED
//   short flash every 1 s = alive, scale OK · double flash = alive, scale NOT found
//   solid = LED=ON or lid closed · dark = LED=OFF
// ---------------------------------------------------------------------------
void updateLed(uint32_t now) {
  bool on;
  if (lidClosed || ledMode == LED_ON) on = true;
  else if (ledMode == LED_OFF) on = false;
  else {
    uint32_t t = now % 1000;
    on = scaleOk ? (t < 60) : (t < 60 || (t >= 180 && t < 240));
  }
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// [J] SETUP + LOOP
// ---------------------------------------------------------------------------
void setup() {
#if defined(ARDUINO_ARCH_STM32)
  Serial.setRx(PA3);
  Serial.setTx(PA2);
#endif
  Serial.begin(BAUD);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  calLoad();
  sendBoot();
  scaleTryStart();
  lastRetry = millis();
  lastTel = millis();
}

void loop() {
  uint32_t now = millis();
  pollSerial();
  scalePoll(now);
  if (now - lastTel >= TEL_MS) {
    lastTel = now;
    sendTel(now);
  }
  updateLed(now);
}
