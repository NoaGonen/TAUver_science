/*
 * ============================================================================
 *  TAUVER Link Kit · firmware link-1.0.0 · Mission 1: prove the pipe
 *  Board: NUCLEO-G491RE  (also runs on an Arduino Mega as a backup board)
 * ============================================================================
 *
 *  WHAT IT DOES (no sensors needed, just the board and its USB cable)
 *    1. Every 1 s it sends a heartbeat frame   {"type":"tel",...}     "ping 1"
 *    2. It answers CMD:PING=<id> with          {"type":"pong",...}    "ping 2 -> ping 3"
 *    3. CMD:LED=ON|OFF|AUTO drives the green LED (LD2), so you can SEE that a
 *       command typed on the command station really arrived.
 *    4. A fake scale + fake lid for the closed-loop demo:
 *       CMD:SIM=ON makes "rock_g" climb, the station watches it, and at
 *       >= 100 g it sends CMD:LID=ROCK,CLOSE. The LED goes solid = "lid shut".
 *
 *  THE WIRE CONTRACT (a strict subset of Science Pack serial contract v3)
 *    up   (Nucleo -> host): one JSON object per line, every frame has "type"
 *    down (host -> Nucleo): one command per line, answered by an "ack"
 *    Same shape as the real firmware, so the pipe you prove today is the pipe
 *    the real payload uses later. Nothing here knows ROS exists.
 *
 *  ARDUINO IDE (STM32duino core 3.0.0, the same settings as the Science Pack)
 *    Tools > Board > STM32 MCU based boards > Generic STM32G4 series
 *    Tools > Board part number > Generic G491RETx
 *    Tools > U(S)ART support > Enabled (generic 'Serial')
 *    Sketch > Export Compiled Binary, then drag the .bin onto the NOD_G491RE drive
 *    (or Upload, if STM32CubeProgrammer is installed).
 *
 *  COMMANDS (case-insensitive, one per line, 115200 baud, newline at the end)
 *    CMD:PING            CMD:PING=<id>      liveness + round-trip timing
 *    CMD:LED=ON|OFF|AUTO                    AUTO = heartbeat blink (default)
 *    CMD:SIM=ON|OFF                         fake scale starts / stops climbing
 *    CMD:LID=ROCK,CLOSE  CMD:LID=ROCK,OPEN  fake lid (OPEN also empties the scale)
 *    CMD:STATUS  CMD:HELP  CMD:RESET        state / command list / back to boot state
 * ============================================================================
 */

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// [A] CONFIG — the only block you should need to touch
// ---------------------------------------------------------------------------
#define FW_VERSION "link-1.0.0"

const uint32_t BAUD           = 115200;  // must match the bridge (--baud)
const uint32_t TEL_MS_IDLE    = 1000;    // heartbeat period while idle
const uint32_t TEL_MS_SIM     = 250;     // faster frames while the fake scale climbs
const float    SIM_G_PER_TICK = 4.0f;    // +4 g every 250 ms = 16 g/s -> 100 g in ~6 s
const float    SIM_MAX_G      = 400.0f;  // the scale stops here if nobody closes the lid
const uint8_t  LINE_MAX       = 96;      // longest command line accepted

#if defined(ARDUINO_ARCH_STM32)
  #define BOARD_NAME "NUCLEO-G491RE"
  const uint32_t PIN_LED = PA5;          // LD2, the green user LED
#else
  #define BOARD_NAME "ARDUINO"
  const uint32_t PIN_LED = LED_BUILTIN;  // Mega: pin 13
#endif

// ---------------------------------------------------------------------------
// [B] STATE
// ---------------------------------------------------------------------------
enum LedMode { LED_AUTO, LED_ON, LED_OFF };

LedMode  ledMode   = LED_AUTO;
bool     simOn     = false;
bool     lidClosed = false;
float    rockG     = 0.0f;

uint32_t seq       = 0;   // heartbeat counter
uint32_t rxLines   = 0;   // good command lines received
uint32_t badLines  = 0;   // unknown / too long / malformed
uint32_t lastTel   = 0;

char     lineBuf[LINE_MAX + 1];
uint8_t  lineLen      = 0;
bool     lineOverflow = false;

// ---------------------------------------------------------------------------
// [C] TINY JSON HELPERS — print straight to Serial, no heap, no printf
// ---------------------------------------------------------------------------
// Print a C string as a JSON string. Quotes and backslashes are escaped,
// anything that is not printable ASCII becomes '?', so a garbage command
// can never break the JSON line we send back.
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

const char* ledName() {
  return ledMode == LED_ON ? "ON" : (ledMode == LED_OFF ? "OFF" : "AUTO");
}

void sendAck(const char* cmd, bool ok, const char* msg) {
  Serial.print(F("{\"type\":\"ack\",\"cmd\":"));
  printJsonStr(cmd);
  Serial.print(F(",\"ok\":"));
  Serial.print(ok ? 1 : 0);
  Serial.print(F(",\"msg\":"));
  printJsonStr(msg);
  Serial.println('}');
}

void sendEvent(const char* lvl, const char* code) {
  Serial.print(F("{\"type\":\"event\",\"ms\":"));
  Serial.print(millis());
  Serial.print(F(",\"lvl\":"));
  printJsonStr(lvl);
  Serial.print(F(",\"code\":"));
  printJsonStr(code);
  Serial.print(F(",\"rock_g\":"));
  Serial.print(rockG, 1);
  Serial.println('}');
}

void sendBoot() {
  Serial.print(F("{\"type\":\"boot\",\"v\":3,\"kit\":\"link\",\"fw\":\"" FW_VERSION "\",\"board\":\"" BOARD_NAME "\",\"baud\":"));
  Serial.print(BAUD);
  Serial.println('}');
}

void sendTel(uint32_t now) {
  Serial.print(F("{\"type\":\"tel\",\"seq\":"));
  Serial.print(seq++);
  Serial.print(F(",\"ms\":"));
  Serial.print(now);
  Serial.print(F(",\"data\":{\"rock_g\":"));
  Serial.print(rockG, 1);
  Serial.print(F(",\"lid_rock\":\""));
  Serial.print(lidClosed ? F("CLOSED") : F("OPEN"));
  Serial.print(F("\",\"sim\":"));
  Serial.print(simOn ? 1 : 0);
  Serial.print(F(",\"led\":\""));
  Serial.print(ledName());
  Serial.println(F("\"}}"));
}

void sendHealth() {
  Serial.print(F("{\"type\":\"health\",\"v\":3,\"kit\":\"link\",\"fw\":\"" FW_VERSION "\",\"up_s\":"));
  Serial.print(millis() / 1000UL);
  Serial.print(F(",\"rx_lines\":"));
  Serial.print(rxLines);
  Serial.print(F(",\"bad_lines\":"));
  Serial.print(badLines);
  Serial.print(F(",\"led\":\""));
  Serial.print(ledName());
  Serial.print(F("\",\"sim\":"));
  Serial.print(simOn ? 1 : 0);
  Serial.print(F(",\"lid_rock\":\""));
  Serial.print(lidClosed ? F("CLOSED") : F("OPEN"));
  Serial.println(F("\"}"));
}

void sendPong(long id, bool hasId) {
  Serial.print(F("{\"type\":\"pong\",\"id\":"));
  if (hasId) Serial.print(id); else Serial.print(F("null"));
  Serial.print(F(",\"ms\":"));
  Serial.print(millis());
  Serial.println('}');
}

void sendHelp() {
  Serial.println(F("{\"type\":\"help\",\"cmds\":[\"CMD:PING[=id]\",\"CMD:LED=ON|OFF|AUTO\",\"CMD:SIM=ON|OFF\",\"CMD:LID=ROCK,CLOSE|OPEN\",\"CMD:STATUS\",\"CMD:HELP\",\"CMD:RESET\"]}"));
}

// ---------------------------------------------------------------------------
// [D] COMMANDS
// ---------------------------------------------------------------------------
// Strip spaces from both ends, in place.
char* trim(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
  return s;
}

bool startsWith(const char* s, const char* prefix) {
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

// Parse "123" into id. False if empty or not all digits (an optional '-' is allowed).
bool parseId(const char* p, long* out) {
  if (!*p) return false;
  const char* q = p;
  if (*q == '-') q++;
  if (!*q) return false;
  for (; *q; q++) if (*q < '0' || *q > '9') return false;
  *out = atol(p);
  return true;
}

void resetDemo() {
  ledMode = LED_AUTO;
  simOn = false;
  lidClosed = false;
  rockG = 0.0f;
}

void handleLine(char* raw) {
  char* s = trim(raw);
  if (!*s) return;

  // Upper-case copy for matching; the original goes back in the ack.
  char up[LINE_MAX + 1];
  size_t n = strlen(s);
  for (size_t i = 0; i <= n; i++) up[i] = (char)toupper((unsigned char)s[i]);

  // ---- liveness ----
  if (startsWith(up, "CMD:PING")) {
    const char* p = up + 8;
    long id = 0;
    bool hasId = false;
    if (*p == '=') {
      hasId = parseId(p + 1, &id);
      if (!hasId) { badLines++; sendAck(s, false, "BAD_ID"); return; }
    } else if (*p != '\0') { badLines++; sendAck(s, false, "UNKNOWN_CMD"); return; }
    rxLines++;
    sendPong(id, hasId);          // pong first: the station times the round trip on it
    sendAck(s, true, "PONG");
    return;
  }

  // ---- LED ----
  if (startsWith(up, "CMD:LED=")) {
    const char* a = up + 8;
    if      (!strcmp(a, "ON"))   ledMode = LED_ON;
    else if (!strcmp(a, "OFF"))  ledMode = LED_OFF;
    else if (!strcmp(a, "AUTO")) ledMode = LED_AUTO;
    else { badLines++; sendAck(s, false, "USE_ON_OFF_AUTO"); return; }
    rxLines++;
    sendAck(s, true, ledName());
    return;
  }

  // ---- fake scale ----
  if (startsWith(up, "CMD:SIM=")) {
    const char* a = up + 8;
    if (!strcmp(a, "ON")) {
      if (lidClosed) { rxLines++; sendAck(s, false, "LID_CLOSED_OPEN_IT_FIRST"); return; }
      simOn = true;
      rockG = 0.0f;
      rxLines++;
      sendAck(s, true, "SIM_ON");
      sendEvent("info", "SIM_START");
    } else if (!strcmp(a, "OFF")) {
      simOn = false;
      rxLines++;
      sendAck(s, true, "SIM_OFF");
      sendEvent("info", "SIM_STOP");
    } else { badLines++; sendAck(s, false, "USE_ON_OFF"); }
    return;
  }

  // ---- fake lid ----
  if (startsWith(up, "CMD:LID=")) {
    const char* a = up + 8;
    if (!startsWith(a, "ROCK,")) { badLines++; sendAck(s, false, "UNKNOWN_BAY"); return; }
    const char* act = a + 5;
    if (!strcmp(act, "CLOSE")) {
      lidClosed = true;
      simOn = false;                // a closed lid freezes the scale
      rxLines++;
      sendAck(s, true, "LID_CLOSED");
      sendEvent("info", "LID_CLOSED");
    } else if (!strcmp(act, "OPEN")) {
      lidClosed = false;
      rockG = 0.0f;                 // "sample removed"
      rxLines++;
      sendAck(s, true, "LID_OPEN");
      sendEvent("info", "LID_OPENED");
    } else { badLines++; sendAck(s, false, "USE_CLOSE_OPEN"); }
    return;
  }

  // ---- housekeeping ----
  if (!strcmp(up, "CMD:STATUS")) { rxLines++; sendHealth(); sendAck(s, true, "OK"); return; }
  if (!strcmp(up, "CMD:HELP"))   { rxLines++; sendHelp();   sendAck(s, true, "OK"); return; }
  if (!strcmp(up, "CMD:RESET"))  { rxLines++; resetDemo();  sendAck(s, true, "RESET"); sendEvent("info", "DEMO_RESET"); return; }

  badLines++;
  sendAck(s, false, "UNKNOWN_CMD");
}

// Non-blocking line reader: collects bytes until '\n', never waits.
void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineOverflow) { badLines++; sendAck("?", false, "LINE_TOO_LONG"); }
      else if (lineLen > 0) { lineBuf[lineLen] = '\0'; handleLine(lineBuf); }
      lineLen = 0;
      lineOverflow = false;
      continue;
    }
    if (lineLen < LINE_MAX) lineBuf[lineLen++] = c;
    else lineOverflow = true;
  }
}

// ---------------------------------------------------------------------------
// [E] LED — what you see on the board tells you which state it is in
//   short flash every 1 s   = alive, idle          (AUTO)
//   fast flashing           = fake scale climbing  (AUTO + SIM)
//   solid on                = LED=ON, or the lid is closed
//   dark                    = LED=OFF (or the board is not running at all)
// ---------------------------------------------------------------------------
void updateLed(uint32_t now) {
  bool on;
  if (lidClosed)               on = true;
  else if (ledMode == LED_ON)  on = true;
  else if (ledMode == LED_OFF) on = false;
  else {
    uint32_t period = simOn ? 250 : 1000;
    on = (now % period) < 60;
  }
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// [F] SETUP + LOOP — the loop never blocks
// ---------------------------------------------------------------------------
void setup() {
#if defined(ARDUINO_ARCH_STM32)
  // Pin the host link to PA2/PA3: the ST-LINK USB port. Harmless if the core
  // already maps Serial there, and it makes the sketch independent of that.
  Serial.setRx(PA3);
  Serial.setTx(PA2);
#endif
  Serial.begin(BAUD);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  sendBoot();
  lastTel = millis();
}

void loop() {
  uint32_t now = millis();

  pollSerial();

  uint32_t period = simOn ? TEL_MS_SIM : TEL_MS_IDLE;
  if ((uint32_t)(now - lastTel) >= period) {
    lastTel = now;
    if (simOn) {
      rockG += SIM_G_PER_TICK;
      if (rockG >= SIM_MAX_G) {
        rockG = SIM_MAX_G;
        simOn = false;
        sendEvent("warn", "SIM_MAX_NOBODY_CLOSED_LID");
      }
    }
    sendTel(now);
  }

  updateLed(now);
}
