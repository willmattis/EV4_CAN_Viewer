/*
 * ============================================================
 *  BER (Bearcats Electric Racing) - EV4 CAN <-> Serial Bridge
 *
 *  Reads every frame off the Vehicle Bus (MCP2515) and streams
 *  it out over USB serial so a PC dashboard can decode it with
 *  the DBC.  ALSO accepts commands from the PC and transmits
 *  them onto the CAN bus (for writing to the Cascadia inverters).
 *
 *  --- CAN -> Serial (unchanged) -----------------------------
 *  One frame per line, candump style:
 *      <ID_HEX>#<DATA_HEX>[x]\n      (trailing 'x' = extended id)
 *
 *  --- Serial -> CAN (new) -----------------------------------
 *  The PC sends ASCII command lines (terminated by \n):
 *
 *    H <idHex> <dataHex>   Set the repeating "heartbeat" frame.
 *                          The ESP32 re-transmits it on CAN every
 *                          HEARTBEAT_MS until replaced or stopped.
 *                          Used for the inverter Command message
 *                          (must be sent continuously or the
 *                          inverter faults).
 *    S                     Stop the heartbeat (clears it).
 *    O <idHex> <dataHex>   Send ONE frame once (parameter writes,
 *                          fault clear, etc.).
 *
 *  dataHex is up to 16 hex chars (8 bytes); short data is right-
 *  padded with zeros.  ids <= 0x7FF are sent as standard frames,
 *  larger ids as 29-bit extended.
 *
 *  SAFETY - deadman: if a heartbeat is active but no serial line
 *  arrives for DEADMAN_MS, the heartbeat is cleared automatically.
 *  So if the PC/USB dies, the inverter stops getting commands and
 *  disables the motor via its own CAN timeout.
 *
 *  Baud: 921600.   Bus: 500 kbps, 8 MHz crystal.
 *  MCP2515 (HSPI):  SCK->14 SI->13 SO->12 CS->15 INT->4
 * ============================================================
 */

#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS    15
#define CAN_INT    4
#define HSPI_SCK  14
#define HSPI_MISO 12
#define HSPI_MOSI 13

#define SERIAL_BAUD  921600
#define HEARTBEAT_MS 20      // re-transmit the command frame this often
#define DEADMAN_MS   500     // stop heartbeat if PC is silent this long

SPIClass hspi(HSPI);
MCP_CAN  CAN(&hspi, CAN_CS);

static const char HEX_DIGITS[] = "0123456789ABCDEF";

// -- Heartbeat (repeating command) state -------------------------
bool     hbActive = false;
uint32_t hbId = 0;
bool     hbExt = false;
uint8_t  hbLen = 8;
uint8_t  hbData[8] = {0};
uint32_t hbLastSendMs = 0;
uint32_t hbLastRxMs   = 0;   // last time we heard ANY serial command

// -- Serial input line buffer ------------------------------------
char    inBuf[64];
uint8_t inLen = 0;

void setupCAN() {
  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, CAN_CS);
  pinMode(CAN_INT, INPUT);
  byte r = CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  while (r != CAN_OK) {
    Serial.println("# MCP2515 init failed, retrying...");
    delay(500);
    r = CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  }
  CAN.setMode(MCP_NORMAL);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println("# EV4 CAN<->Serial bridge boot");
  setupCAN();
  Serial.println("# CAN ready @500k");
}

// ---- CAN -> Serial ---------------------------------------------
void emitFrame(uint32_t id, uint8_t len, const uint8_t *buf, bool ext) {
  char line[40];
  uint8_t n = 0;
  char idbuf[9];
  uint8_t ni = 0;
  uint32_t tmp = id;
  if (tmp == 0) idbuf[ni++] = '0';
  else while (tmp) { idbuf[ni++] = HEX_DIGITS[tmp & 0xF]; tmp >>= 4; }
  while (ni) line[n++] = idbuf[--ni];
  line[n++] = '#';
  for (uint8_t i = 0; i < len; i++) {
    line[n++] = HEX_DIGITS[(buf[i] >> 4) & 0xF];
    line[n++] = HEX_DIGITS[buf[i] & 0xF];
  }
  if (ext) line[n++] = 'x';
  line[n++] = '\n';
  Serial.write((const uint8_t *)line, n);
}

void pollCANrx() {
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t rxId; uint8_t len; uint8_t buf[8];
    if (CAN.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
      bool ext = (rxId & 0x80000000) != 0;
      emitFrame(rxId & 0x1FFFFFFF, len, buf, ext);
    }
  }
}

// ---- helpers ----------------------------------------------------
int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Parse hex string into up to 8 bytes; returns byte count (always 8, zero-padded)
uint8_t parseData(const char *s, uint8_t *out) {
  for (uint8_t i = 0; i < 8; i++) out[i] = 0;
  uint8_t nb = 0;
  while (s[0] && s[1] && nb < 8) {
    int hi = hexNibble(s[0]); int lo = hexNibble(s[1]);
    if (hi < 0 || lo < 0) break;
    out[nb++] = (uint8_t)((hi << 4) | lo);
    s += 2;
  }
  return nb;
}

uint32_t parseHexU32(const char *s) {
  uint32_t v = 0;
  while (*s) { int d = hexNibble(*s); if (d < 0) break; v = (v << 4) | d; s++; }
  return v;
}

void sendCAN(uint32_t id, bool ext, uint8_t len, uint8_t *data) {
  CAN.sendMsgBuf(id, ext ? 1 : 0, len, data);
}

// ---- Serial -> CAN ---------------------------------------------
void handleLine(char *line) {
  hbLastRxMs = millis();
  char cmd = line[0];

  if (cmd == 'S' || cmd == 's') {            // stop heartbeat
    hbActive = false;
    return;
  }
  if (cmd != 'H' && cmd != 'h' && cmd != 'O' && cmd != 'o') return;

  // tokens: <cmd> <idHex> <dataHex>
  char *p = line + 1;
  while (*p == ' ') p++;
  char *idTok = p;
  while (*p && *p != ' ') p++;
  if (*p) *p++ = '\0';
  while (*p == ' ') p++;
  char *dataTok = p;

  uint32_t id = parseHexU32(idTok);
  uint8_t  data[8];
  parseData(dataTok, data);
  bool ext = (id > 0x7FF);

  if (cmd == 'O' || cmd == 'o') {
    sendCAN(id, ext, 8, data);               // one-shot
  } else {                                   // 'H' set heartbeat
    hbId = id; hbExt = ext; hbLen = 8;
    for (uint8_t i = 0; i < 8; i++) hbData[i] = data[i];
    hbActive = true;
    sendCAN(hbId, hbExt, hbLen, hbData);      // send immediately too
    hbLastSendMs = millis();
  }
}

void pollSerialRx() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inLen > 0) { inBuf[inLen] = '\0'; handleLine(inBuf); inLen = 0; }
    } else if (inLen < sizeof(inBuf) - 1) {
      inBuf[inLen++] = c;
    }
  }
}

void serviceHeartbeat() {
  uint32_t now = millis();
  if (!hbActive) return;
  if (now - hbLastRxMs > DEADMAN_MS) {        // PC went silent -> fail safe
    hbActive = false;
    return;
  }
  if (now - hbLastSendMs >= HEARTBEAT_MS) {
    sendCAN(hbId, hbExt, hbLen, hbData);
    hbLastSendMs = now;
  }
}

void loop() {
  pollCANrx();
  pollSerialRx();
  serviceHeartbeat();
}
