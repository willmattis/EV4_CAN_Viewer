/*
 * ============================================================
 *  BER (Bearcats Electric Racing) - EV3 Vehicle Bus Dash
 *  Target:  ST7789 170x320 TFT (mounted landscape = 320x170)
 *  CAN DBC: CAN/EV4_Vehicle_Bus.dbc
 *
 *  Button: GPIO 16 -> GND  (cycles display modes)
 *  Mode 0 - Overview:  two-column, all data, size-1 text
 *  Mode 1 - Driver:    large APPS / POWER / TORQUE / LV
 *  Mode 2 - Thermal:   large cell temps, voltages, BMS
 *
 *  ST7789 wiring:
 *    GND->GND  VCC->3.3V  SCL->GPIO18  SDA->GPIO23
 *    RES->GPIO22  DC->GPIO21  BLK->GPIO17  CS->GPIO5
 *  MCP2515 wiring (HSPI):
 *    SCK->GPIO14  SI->GPIO13  SO->GPIO12  CS->GPIO15  INT->GPIO4
 * ============================================================
 */

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <mcp_can.h>

// ── Pin definitions ──────────────────────────────────────────
#define TFT_CS    5
#define TFT_DC    21
#define TFT_RST   22
#define TFT_SCLK  18
#define TFT_MOSI  23
#define TFT_BL    17
#define CAN_CS    15
#define CAN_INT    4
#define HSPI_SCK  14
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define BTN_PIN   16

Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, TFT_RST);
SPIClass hspi(HSPI);
MCP_CAN CAN(&hspi, CAN_CS);

// ── Display geometry (landscape 320x170) ─────────────────────
#define SCREEN_W  320
#define SCREEN_H  170
#define HDR_H     22          // header bar height
#define MID_H     24          // badge / SOC strip height
#define DATA_Y    (HDR_H + MID_H)  // top of data area = 46
#define DATA_H    (SCREEN_H - DATA_Y)  // height of data area = 124
#define COL_X     161         // vertical divider x (overview mode)
#define LBL_L_X   5           // left col label x
#define VAL_L_X   155         // left col value right-align x
#define LBL_R_X   166         // right col label x
#define VAL_R_X   315         // right col value right-align x
#define ROW_STEP  18          // px between overview rows

// ── Colours (RGB565) ─────────────────────────────────────────
#define CLR_BG        0x0000
#define CLR_PANEL     0x1082
#define CLR_ACCENT    0xF800
#define CLR_WHITE     0xFFFF
#define CLR_GREY      0xC618
#define CLR_DARKGREY  0x630C
#define CLR_GREEN     0x07E0
#define CLR_YELLOW    0xFFE0
#define CLR_ORANGE    0xFD20
#define CLR_CYAN      0x07FF

const char* DRIVE_MODES[] = { "STANDBY", "DRIVE", "REGEN", "ENDURANCE", "SPORT", "LIMP" };

// ── Vehicle state ─────────────────────────────────────────────
struct VehicleState {
  bool     ecuFault, mcFault;
  uint16_t ecuFaultBits;
  bool     mcFaultInv1, mcFaultInv2;
  uint32_t mcFaultBitsInv1, mcFaultBitsInv2;
  uint16_t apps0, apps1;
  uint8_t  appsPct;
  int16_t  torqueCmd;
  uint16_t bpsRaw;
  uint8_t  water1C, water2C, water3C;
  bool     r2dButton, prog1, prog2, brakePressed;
  uint16_t powerKw;
  float    lvVoltage;
  uint8_t  battSoc;
  bool     initFinished, prechargeComplete, r2dActive, regenEnabled;
  uint8_t  driveMode;
  float    bmsSoc, bmsCurrent;
  float    bmsMaxCellV, bmsMaxCellTempC;
  float    bmsMinCellV, bmsMinCellTempC;
  uint8_t  bmsPowerLimitKw;
  float    speedMph;
  float    tsVoltage;
};

VehicleState state     = {};
VehicleState prevState = {};
bool     firstDraw  = true;
uint8_t  dispMode   = 1;     // 0=overview, 1=driver, 2=thermal
#define  NUM_MODES  3

uint32_t rxCount    = 0;
uint32_t lastRxMs   = 0;
uint32_t lastDrawMs = 0;

// ── CAN decode ───────────────────────────────────────────────
uint16_t getU16(const uint8_t *b, uint8_t i) { return b[i] | ((uint16_t)b[i+1] << 8); }
int16_t  getS16(const uint8_t *b, uint8_t i) { return (int16_t)getU16(b,i); }
bool     getBit(const uint8_t *b, uint8_t bit){ return (b[bit/8] >> (bit%8)) & 1; }
uint32_t getBitsLE(const uint8_t *b, uint8_t start, uint8_t len) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < len; i++) if (getBit(b, start+i)) v |= 1UL << i;
  return v;
}

void decodeFrame(uint32_t id, const uint8_t *b, uint8_t len) {
  if (len < 8) return;
  switch (id) {
    case 0x2: state.ecuFaultBits = getU16(b,0); state.ecuFault = getBit(b,7)||getBit(b,9)||getBit(b,6); break;
    case 0x3:
      state.mcFaultBitsInv1 = getBitsLE(b,0,26);
      state.mcFaultBitsInv2 = getBitsLE(b,32,26);
      state.mcFaultInv1 = getBit(b,25);
      state.mcFaultInv2 = getBit(b,57);
      state.mcFault = state.mcFaultInv1 || state.mcFaultInv2;
      break;
    case 0x4: state.apps0=getU16(b,0); state.apps1=getU16(b,2); state.appsPct=b[4]; state.torqueCmd=getS16(b,5); break;
    case 0x5:
      state.bpsRaw=getU16(b,0); state.water1C=b[2]; state.water2C=b[3]; state.water3C=b[4];
      state.r2dButton=getBit(b,40); state.prog1=getBit(b,41); state.prog2=getBit(b,42); state.brakePressed=getBit(b,43);
      break;
    case 0x6:
      state.powerKw=getU16(b,0); state.lvVoltage=getU16(b,2)*0.1f; state.battSoc=b[4];
      state.initFinished=getBit(b,40); state.prechargeComplete=getBit(b,41);
      state.r2dActive=getBit(b,42); state.driveMode=(uint8_t)getBitsLE(b,43,3); state.regenEnabled=getBit(b,46);
      break;
    case 0x7:
      state.bmsSoc=b[0]*0.392156863f; state.bmsCurrent=b[1]*0.78125f;
      state.bmsMaxCellV=b[2]*0.019607843f; state.bmsMaxCellTempC=b[3]*0.588235294f;
      state.bmsMinCellV=b[4]*0.019607843f; state.bmsMinCellTempC=b[5]*0.588235294f;
      state.bmsPowerLimitKw=b[6];
      break;
    case 0x8:
      state.speedMph  = getU16(b, 0) * 0.1f;
      state.tsVoltage = getU16(b, 2) * 0.1f;
      break;
  }
}

void readCANFrames() {
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t id; uint8_t len; uint8_t buf[8];
    if (CAN.readMsgBuf(&id, &len, buf) == CAN_OK) {
      decodeFrame(id & 0x1FFFFFFF, buf, len);
      rxCount++; lastRxMs = millis();
    }
  }
}

// ── Draw primitives ──────────────────────────────────────────
void gText(const char* t, int16_t x, int16_t y, uint8_t sz, uint16_t c) {
  tft.setTextSize(sz); tft.setTextColor(c); tft.setCursor(x, y); tft.print(t);
}
void gRightText(const char* t, int16_t rx, int16_t y, uint8_t sz, uint16_t c) {
  int16_t bx, by; uint16_t bw, bh;
  tft.setTextSize(sz); tft.getTextBounds(t,0,0,&bx,&by,&bw,&bh);
  gText(t, rx-bw, y, sz, c);
}
void gCenterText(const char* t, int16_t cx, int16_t y, uint8_t sz, uint16_t c) {
  int16_t bx, by; uint16_t bw, bh;
  tft.setTextSize(sz); tft.getTextBounds(t,0,0,&bx,&by,&bw,&bh);
  gText(t, cx-bw/2, y, sz, c);
}
void drawBadge(int x, int y, const char* lbl, bool on, uint16_t onColor) {
  tft.drawRect(x, y, 36, 15, on ? onColor : CLR_DARKGREY);
  tft.fillRect(x+1, y+1, 34, 13, CLR_BG);
  gCenterText(lbl, x+18, y+4, 1, on ? onColor : CLR_GREY);
}
void drawSocBar(int x, int y, int w, int h, float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  tft.drawRect(x, y, w, h, CLR_GREY);
  tft.fillRect(x+1, y+1, w-2, h-2, CLR_PANEL);
  uint16_t col = pct > 50 ? CLR_GREEN : (pct > 20 ? CLR_YELLOW : CLR_ACCENT);
  tft.fillRect(x+1, y+1, (int)((pct/100.0f)*(w-2)), h-2, col);
}
uint16_t faultColor() { return (state.ecuFault||state.mcFault) ? CLR_ACCENT : CLR_GREEN; }

uint16_t modeColor(uint8_t mode) {
  switch (mode) {
    case 1: return CLR_GREEN;
    case 2: return CLR_CYAN;
    case 3: return CLR_WHITE;
    case 4: return CLR_YELLOW;
    case 5: return CLR_ORANGE;
    default: return CLR_GREY;
  }
}

float curSoc()     { return state.bmsSoc > 0.1f ? state.bmsSoc : (float)state.battSoc; }
float prevSoc()    { return prevState.bmsSoc > 0.1f ? prevState.bmsSoc : (float)prevState.battSoc; }

// Shared header base — call once in drawStaticUI per mode
void drawHdrBase() {
  tft.fillRect(0, 0, SCREEN_W, HDR_H, CLR_ACCENT);
  tft.setTextSize(2); tft.setTextColor(CLR_WHITE, CLR_ACCENT);
  tft.setCursor(6, 4); tft.print("BER EV3");
}

// Shared header dynamic update — called every drawDashboard
void drawHdrDynamic() {
  bool canOk = lastRxMs && (millis() - lastRxMs < 1000);
  tft.fillRect(SCREEN_W-54, 4, 52, 14, CLR_ACCENT);
  gRightText(canOk ? "CAN" : "NO CAN", SCREEN_W-4, 6, 1, canOk ? CLR_WHITE : CLR_YELLOW);

  if (firstDraw || state.driveMode != prevState.driveMode || state.r2dActive != prevState.r2dActive) {
    tft.fillRect(100, 3, 116, 16, CLR_ACCENT);
    const char* m = state.driveMode < 6 ? DRIVE_MODES[state.driveMode] : "?";
    uint16_t tc = (dispMode == 1) ? CLR_WHITE : (state.r2dActive ? CLR_WHITE : CLR_GREY);
    gCenterText(m, SCREEN_W/2, 5, 2, tc);
  }
}

// Shared badges + SOC strip — call from each mode's static + dynamic
void drawBadgesStatic() {
  if (dispMode != 0) return;
  drawBadge(5,  HDR_H+4, "INIT", state.initFinished,      CLR_GREEN);
  drawBadge(44, HDR_H+4, "PRE",  state.prechargeComplete, CLR_GREEN);
  drawBadge(83, HDR_H+4, "R2D",  state.r2dActive,         CLR_GREEN);
  drawBadge(122,HDR_H+4, "RGN",  state.regenEnabled,      CLR_ORANGE);
}
void drawBadgesDynamic() {
  if (dispMode != 0) return;
  if (firstDraw || state.initFinished      != prevState.initFinished)      drawBadge(5,  HDR_H+4, "INIT", state.initFinished,      CLR_GREEN);
  if (firstDraw || state.prechargeComplete != prevState.prechargeComplete) drawBadge(44, HDR_H+4, "PRE",  state.prechargeComplete, CLR_GREEN);
  if (firstDraw || state.r2dActive         != prevState.r2dActive)         drawBadge(83, HDR_H+4, "R2D",  state.r2dActive,         CLR_GREEN);
  if (firstDraw || state.regenEnabled      != prevState.regenEnabled)      drawBadge(122,HDR_H+4, "RGN",  state.regenEnabled,      CLR_ORANGE);
}

// ═══════════════════════════════════════════════════════════════
//  MODE 0 — OVERVIEW  (two columns, all data)
// ═══════════════════════════════════════════════════════════════
void drawStaticOverview() {
  tft.fillScreen(CLR_BG);
  drawHdrBase();
  gText("SOC", LBL_R_X, HDR_H+3, 1, CLR_GREY);
  tft.drawFastHLine(0, DATA_Y, SCREEN_W, CLR_DARKGREY);
  tft.drawFastVLine(COL_X, DATA_Y, DATA_H, CLR_DARKGREY);

  const int ys[] = { DATA_Y+2, DATA_Y+2+ROW_STEP, DATA_Y+2+ROW_STEP*2, DATA_Y+2+ROW_STEP*3, DATA_Y+2+ROW_STEP*4, DATA_Y+2+ROW_STEP*5 };
  gText("APPS",      LBL_L_X, ys[0], 1, CLR_GREY);
  gText("TORQUE",    LBL_L_X, ys[1], 1, CLR_GREY);
  gText("POWER",     LBL_L_X, ys[2], 1, CLR_GREY);
  gText("LV",        LBL_L_X, ys[3], 1, CLR_GREY);
  gText("BPS RAW",   LBL_L_X, ys[4], 1, CLR_GREY);
  gText("INPUTS",    LBL_L_X, ys[5], 1, CLR_GREY);
  gText("BMS I/LIM", LBL_R_X, ys[0], 1, CLR_GREY);
  gText("CELL V",    LBL_R_X, ys[1], 1, CLR_GREY);
  gText("CELL T",    LBL_R_X, ys[2], 1, CLR_GREY);
  gText("WATER",     LBL_R_X, ys[3], 1, CLR_GREY);
  gText("FAULT",     LBL_R_X, ys[4], 1, CLR_GREY);
}

void drawLV(int y, const char* v, uint16_t c=CLR_WHITE) { tft.fillRect(50,y,COL_X-55,10,CLR_BG); gRightText(v,VAL_L_X,y,1,c); }
void drawRV(int y, const char* v, uint16_t c=CLR_WHITE) { tft.fillRect(222,y,VAL_R_X-222,10,CLR_BG); gRightText(v,VAL_R_X,y,1,c); }

void drawDashOverview() {
  char buf[32];
  drawHdrDynamic();
  drawBadgesDynamic();

  float soc = curSoc();
  if (firstDraw || soc != prevSoc()) {
    tft.fillRect(LBL_R_X+22, HDR_H+1, VAL_R_X-LBL_R_X-22, 12, CLR_BG);
    snprintf(buf, sizeof(buf), "%.0f%%", soc);
    gRightText(buf, VAL_R_X, HDR_H+3, 1, CLR_WHITE);
    drawSocBar(LBL_R_X, HDR_H+13, VAL_R_X-LBL_R_X, 7, soc);
  }

  const int ys[] = { DATA_Y+2, DATA_Y+2+ROW_STEP, DATA_Y+2+ROW_STEP*2, DATA_Y+2+ROW_STEP*3, DATA_Y+2+ROW_STEP*4, DATA_Y+2+ROW_STEP*5 };

  if (firstDraw || state.appsPct    != prevState.appsPct)    { snprintf(buf,sizeof(buf),"%u%%",state.appsPct);      drawLV(ys[0],buf); }
  if (firstDraw || state.torqueCmd  != prevState.torqueCmd)  { snprintf(buf,sizeof(buf),"%d Nm",state.torqueCmd);   drawLV(ys[1],buf); }
  if (firstDraw || state.powerKw    != prevState.powerKw)    { snprintf(buf,sizeof(buf),"%u kW",state.powerKw);     drawLV(ys[2],buf); }
  if (firstDraw || state.lvVoltage  != prevState.lvVoltage)  { snprintf(buf,sizeof(buf),"%.1f V",state.lvVoltage);  drawLV(ys[3],buf); }
  if (firstDraw || state.bpsRaw     != prevState.bpsRaw || state.brakePressed != prevState.brakePressed) {
    snprintf(buf,sizeof(buf),"%u",state.bpsRaw);
    drawLV(ys[4],buf, state.brakePressed ? CLR_ORANGE : CLR_WHITE);
  }
  if (firstDraw || state.brakePressed!=prevState.brakePressed || state.r2dButton!=prevState.r2dButton || state.prog1!=prevState.prog1 || state.prog2!=prevState.prog2) {
    snprintf(buf,sizeof(buf),"%s %s %s", state.brakePressed?"BRK":"---", state.r2dButton?"R2D":"---", (state.prog1||state.prog2)?"PRG":"---");
    drawLV(ys[5],buf, state.brakePressed ? CLR_ORANGE : CLR_WHITE);
  }
  if (firstDraw || state.bmsCurrent!=prevState.bmsCurrent || state.bmsPowerLimitKw!=prevState.bmsPowerLimitKw) {
    snprintf(buf,sizeof(buf),"%.0f A / %u kW",state.bmsCurrent,state.bmsPowerLimitKw); drawRV(ys[0],buf);
  }
  if (firstDraw || state.bmsMinCellV!=prevState.bmsMinCellV || state.bmsMaxCellV!=prevState.bmsMaxCellV) {
    snprintf(buf,sizeof(buf),"%.2f/%.2f V",state.bmsMinCellV,state.bmsMaxCellV); drawRV(ys[1],buf);
  }
  if (firstDraw || state.bmsMinCellTempC!=prevState.bmsMinCellTempC || state.bmsMaxCellTempC!=prevState.bmsMaxCellTempC) {
    snprintf(buf,sizeof(buf),"%.0f/%.0f C",state.bmsMinCellTempC,state.bmsMaxCellTempC); drawRV(ys[2],buf);
  }
  if (firstDraw || state.water1C!=prevState.water1C || state.water2C!=prevState.water2C || state.water3C!=prevState.water3C) {
    snprintf(buf,sizeof(buf),"%u %u %u C",state.water1C,state.water2C,state.water3C); drawRV(ys[3],buf);
  }
  if (firstDraw || state.ecuFault!=prevState.ecuFault || state.mcFault!=prevState.mcFault) {
    drawRV(ys[4], (state.ecuFault||state.mcFault)?"ACTIVE":"CLEAR", faultColor());
  }
  if (firstDraw || state.ecuFaultBits!=prevState.ecuFaultBits ||
      state.mcFaultBitsInv1!=prevState.mcFaultBitsInv1 ||
      state.mcFaultBitsInv2!=prevState.mcFaultBitsInv2) {
    snprintf(buf,sizeof(buf),"E%03X I1%07lX I2%07lX",state.ecuFaultBits,
             (unsigned long)state.mcFaultBitsInv1, (unsigned long)state.mcFaultBitsInv2);
    tft.fillRect(LBL_R_X, ys[5], SCREEN_W-LBL_R_X, 10, CLR_BG);
    gText(buf, LBL_R_X, ys[5], 1, faultColor());
  }
}

// ═══════════════════════════════════════════════════════════════
//  MODE 1 — DRIVER  (big display: SOC, Speed, TS Volt, GLV Volt, Mode)
// ═══════════════════════════════════════════════════════════════
// Layout (landscape 320x170):
//   y=  0..21  Header: "BER EV3" + CAN status
//   y= 22..45  Mode strip: drive mode name, large, colored
//   y= 46      Separator
//   y= 47..115 Top zone: SPEED (left) | SOC % (right), size-4 values
//   y=116      Separator
//   y=117..153 Bottom zone: TS VOLT (left) | GLV VOLT (right), size-3 values
//   y=154..169 Status bar: BRAKE | R2D

void drawStaticDriver() {
  tft.fillScreen(CLR_BG);
  drawHdrBase();

  gText("SOC", 6, HDR_H+7, 1, CLR_GREY);
  tft.drawFastHLine(0, DATA_Y, SCREEN_W, CLR_DARKGREY);
  tft.drawFastHLine(0, 137, SCREEN_W, CLR_DARKGREY);
  tft.drawFastVLine(190, DATA_Y, 91, CLR_DARKGREY);

  gText("SPEED", 8, 51, 1, CLR_GREY);
  gText("mph", 152, 113, 2, CLR_GREY);
  gText("TS", 198, 54, 1, CLR_GREY);
  gText("GLV", 198, 94, 1, CLR_GREY);
  gText("FAULT", 6, 144, 1, CLR_GREY);
}

void drawDashDriverLegacy() {
  char buf[32];
  drawHdrDynamic();

  // SOC bar in middle strip
  float soc = curSoc();
  if (firstDraw || soc != prevSoc()) {
    tft.fillRect(30, HDR_H+2, SCREEN_W-80, 20, CLR_BG);
    drawSocBar(30, HDR_H+4, SCREEN_W-82, 16, soc);
    snprintf(buf, sizeof(buf), "%.0f%%", soc);
    tft.fillRect(SCREEN_W-48, HDR_H+2, 44, 20, CLR_BG);
    gRightText(buf, SCREEN_W-4, HDR_H+8, 1, CLR_WHITE);
  }

  // SPEED — size 4, top-left
  if (firstDraw || state.speedMph != prevState.speedMph) {
    snprintf(buf, sizeof(buf), "%.0f", state.speedMph);
    tft.fillRect(5, DATA_Y+10, 150, 36, CLR_BG);
    gRightText(buf, 148, DATA_Y+10, 4, CLR_WHITE);
  }

  // TS VOLTAGE — size 3, top-right
  if (firstDraw || state.tsVoltage != prevState.tsVoltage) {
    snprintf(buf, sizeof(buf), "%.1fV", state.tsVoltage);
    tft.fillRect(165, DATA_Y+10, 150, 28, CLR_BG);
    gRightText(buf, 315, DATA_Y+10, 3, CLR_WHITE);
  }

  // GLV VOLTAGE — size 3, bottom-right
  if (firstDraw || state.lvVoltage != prevState.lvVoltage) {
    snprintf(buf, sizeof(buf), "%.1fV", state.lvVoltage);
    tft.fillRect(165, DATA_Y+57, 150, 28, CLR_BG);
    gRightText(buf, 315, DATA_Y+57, 3, CLR_WHITE);
  }

}

// ═══════════════════════════════════════════════════════════════
//  MODE 2 — THERMAL  (large cell/water temps and voltages)
// ═══════════════════════════════════════════════════════════════
void drawDashDriver() {
  char buf[32];
  drawHdrDynamic();

  float soc = curSoc();
  if (firstDraw || soc != prevSoc()) {
    tft.fillRect(32, HDR_H+3, 224, 16, CLR_BG);
    drawSocBar(32, HDR_H+5, 224, 12, soc);
    snprintf(buf, sizeof(buf), "%.0f%%", soc);
    tft.fillRect(262, HDR_H+4, 54, 14, CLR_BG);
    gRightText(buf, SCREEN_W-5, HDR_H+7, 1, CLR_WHITE);
  }

  if (firstDraw || state.speedMph != prevState.speedMph) {
    snprintf(buf, sizeof(buf), "%.0f", state.speedMph);
    tft.fillRect(8, 64, 174, 48, CLR_BG);
    gRightText(buf, 182, 64, 6, CLR_WHITE);
  }

  if (firstDraw || state.tsVoltage != prevState.tsVoltage) {
    snprintf(buf, sizeof(buf), "%.1fV", state.tsVoltage);
    tft.fillRect(226, 54, 88, 24, CLR_BG);
    gRightText(buf, 315, 57, 2, CLR_CYAN);
  }

  if (firstDraw || state.lvVoltage != prevState.lvVoltage) {
    snprintf(buf, sizeof(buf), "%.1fV", state.lvVoltage);
    tft.fillRect(226, 94, 88, 24, CLR_BG);
    gRightText(buf, 315, 97, 2, CLR_YELLOW);
  }

  if (firstDraw || state.ecuFault != prevState.ecuFault || state.mcFault != prevState.mcFault) {
    bool fault = state.ecuFault || state.mcFault;
    tft.fillRect(50, 141, 74, 20, CLR_BG);
    if (fault) {
      tft.fillRect(50, 141, 74, 18, CLR_ACCENT);
      gCenterText("ACTIVE", 87, 146, 1, CLR_WHITE);
    } else {
      tft.drawRect(50, 141, 74, 18, CLR_GREEN);
      gCenterText("CLEAR", 87, 146, 1, CLR_GREEN);
    }
  }

  if (firstDraw || state.brakePressed != prevState.brakePressed) drawBadge(164, 142, "BRK", state.brakePressed, CLR_ORANGE);
  if (firstDraw || state.r2dActive != prevState.r2dActive) drawBadge(203, 142, "R2D", state.r2dActive, CLR_GREEN);
  if (firstDraw || state.prechargeComplete != prevState.prechargeComplete) drawBadge(242, 142, "PRE", state.prechargeComplete, CLR_GREEN);
  if (firstDraw || state.regenEnabled != prevState.regenEnabled) drawBadge(281, 142, "RGN", state.regenEnabled, CLR_ORANGE);
}

#define TH_STEP   22   // px per thermal row
#define TH_Y0     (DATA_Y + 4)

void drawStaticThermal() {
  tft.fillScreen(CLR_BG);
  drawHdrBase();

  // SOC bar in middle strip (same as driver)
  gText("SOC", 5, HDR_H+6, 1, CLR_GREY);
  tft.drawFastHLine(0, DATA_Y, SCREEN_W, CLR_DARKGREY);

  // Labels left-aligned, values will be size 2 right-aligned
  gText("CELL T",   LBL_L_X, TH_Y0 + TH_STEP*0 + 6, 1, CLR_GREY);
  gText("WATER",    LBL_L_X, TH_Y0 + TH_STEP*1 + 6, 1, CLR_GREY);
  gText("CELL V",   LBL_L_X, TH_Y0 + TH_STEP*2 + 6, 1, CLR_GREY);
  gText("BMS I/LIM",LBL_L_X, TH_Y0 + TH_STEP*3 + 6, 1, CLR_GREY);
  gText("FAULT",    LBL_L_X, TH_Y0 + TH_STEP*4 + 6, 1, CLR_GREY);
}

void drawThermalRow(int row, const char* val, uint16_t color = CLR_WHITE) {
  int y = TH_Y0 + TH_STEP * row;
  tft.fillRect(68, y, SCREEN_W-72, 18, CLR_BG);
  gRightText(val, SCREEN_W-4, y, 2, color);
}

void drawDashThermal() {
  char buf[32];
  drawHdrDynamic();
  drawBadgesDynamic();

  // SOC
  float soc = curSoc();
  if (firstDraw || soc != prevSoc()) {
    tft.fillRect(28, HDR_H+2, SCREEN_W-80, 18, CLR_BG);
    drawSocBar(28, HDR_H+4, SCREEN_W-82, 14, soc);
    snprintf(buf, sizeof(buf), "%.0f%%", soc);
    tft.fillRect(SCREEN_W-52, HDR_H+2, 48, 18, CLR_BG);
    gRightText(buf, SCREEN_W-4, HDR_H+6, 1, CLR_WHITE);
  }

  // CELL T  max/min
  if (firstDraw || state.bmsMaxCellTempC!=prevState.bmsMaxCellTempC || state.bmsMinCellTempC!=prevState.bmsMinCellTempC) {
    snprintf(buf, sizeof(buf), "%.0f / %.0f C", state.bmsMaxCellTempC, state.bmsMinCellTempC);
    uint16_t col = state.bmsMaxCellTempC > 55.0f ? CLR_ACCENT : (state.bmsMaxCellTempC > 45.0f ? CLR_ORANGE : CLR_WHITE);
    drawThermalRow(0, buf, col);
  }

  // WATER temps
  if (firstDraw || state.water1C!=prevState.water1C || state.water2C!=prevState.water2C || state.water3C!=prevState.water3C) {
    snprintf(buf, sizeof(buf), "%u  %u  %u C", state.water1C, state.water2C, state.water3C);
    drawThermalRow(1, buf);
  }

  // CELL V  max/min
  if (firstDraw || state.bmsMaxCellV!=prevState.bmsMaxCellV || state.bmsMinCellV!=prevState.bmsMinCellV) {
    snprintf(buf, sizeof(buf), "%.3f / %.3f V", state.bmsMaxCellV, state.bmsMinCellV);
    uint16_t col = state.bmsMinCellV < 3.3f ? CLR_ACCENT : (state.bmsMinCellV < 3.5f ? CLR_ORANGE : CLR_WHITE);
    drawThermalRow(2, buf, col);
  }

  // BMS current / limit
  if (firstDraw || state.bmsCurrent!=prevState.bmsCurrent || state.bmsPowerLimitKw!=prevState.bmsPowerLimitKw) {
    snprintf(buf, sizeof(buf), "%.0f A / %u kW", state.bmsCurrent, state.bmsPowerLimitKw);
    drawThermalRow(3, buf);
  }

  // Fault
  if (firstDraw || state.ecuFault!=prevState.ecuFault || state.mcFault!=prevState.mcFault ||
      state.ecuFaultBits!=prevState.ecuFaultBits ||
      state.mcFaultBitsInv1!=prevState.mcFaultBitsInv1 ||
      state.mcFaultBitsInv2!=prevState.mcFaultBitsInv2) {
    uint16_t fc = faultColor();
    if (state.ecuFault || state.mcFault) {
      snprintf(buf, sizeof(buf), "I1%07lX I2%07lX",
               (unsigned long)state.mcFaultBitsInv1, (unsigned long)state.mcFaultBitsInv2);
    } else {
      snprintf(buf, sizeof(buf), "CLEAR");
    }
    drawThermalRow(4, buf, fc);
  }
}

// ═══════════════════════════════════════════════════════════════
//  Mode dispatch
// ═══════════════════════════════════════════════════════════════
void drawStaticUI() {
  switch (dispMode) {
    case 0: drawStaticOverview(); drawBadgesStatic(); break;
    case 1: drawStaticDriver();   drawBadgesStatic(); break;
    case 2: drawStaticThermal();  drawBadgesStatic(); break;
  }
}

void drawDashboard() {
  switch (dispMode) {
    case 0: drawDashOverview(); break;
    case 1: drawDashDriver();   break;
    case 2: drawDashThermal();  break;
  }
  prevState = state;
  firstDraw = false;
}

// ── Button handling ──────────────────────────────────────────
void checkButton() {
  static bool prevBtn = HIGH;
  static uint32_t lastChange = 0;
  bool btn = digitalRead(BTN_PIN);
  uint32_t now = millis();
  if (btn != prevBtn && now - lastChange > 50) {
    lastChange = now;
    prevBtn = btn;
    if (btn == LOW) {
      dispMode = (dispMode + 1) % NUM_MODES;
      firstDraw = true;
      drawStaticUI();
    }
  }
}

// ── CAN status ───────────────────────────────────────────────
void reportCANStatus() {
  static uint32_t lastReport = 0;
  if (millis() - lastReport < 2000) return;
  lastReport = millis();
  byte err = CAN.getError();
  Serial.print("STATUS RX="); Serial.print(rxCount);
  Serial.print(" EFLG=0x"); Serial.print(err, HEX);
  Serial.print(" TEC="); Serial.print(CAN.errorCountTX());
  Serial.print(" REC="); Serial.println(CAN.errorCountRX());
}

// ── Hardware init ────────────────────────────────────────────
void setupDisplay() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH); delay(10);
  digitalWrite(TFT_RST, LOW);  delay(10);
  digitalWrite(TFT_RST, HIGH); delay(120);
  tft.init(170, 320, SPI_MODE3);
  tft.setSPISpeed(40000000);
  tft.setRotation(3);
  tft.invertDisplay(true);
}

void setupCAN() {
  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, CAN_CS);
  pinMode(CAN_INT, INPUT);
  byte r = CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  Serial.print("CAN.begin(): "); Serial.println(r == CAN_OK ? "OK" : "FAIL");
  while (r != CAN_OK) {
    Serial.println("MCP2515 init failed, retrying...");
    delay(500);
    r = CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  }
  Serial.println(CAN.setMode(MCP_NORMAL) == MCP2515_OK ? "CAN normal" : "CAN mode FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[EV3 Dash] Boot");

  pinMode(BTN_PIN, INPUT_PULLUP);

  setupCAN();
  setupDisplay();

  // Splash
  tft.fillScreen(CLR_BG);
  gCenterText("BEARCATS", SCREEN_W/2, 45,  2, CLR_WHITE);
  gCenterText("ELECTRIC", SCREEN_W/2, 75,  2, CLR_ACCENT);
  gCenterText("RACING",   SCREEN_W/2, 105, 2, CLR_WHITE);
  delay(1500);

  drawStaticUI();
  drawDashboard();
}

void loop() {
  checkButton();
  readCANFrames();
  reportCANStatus();

  uint32_t now = millis();
  if (now - lastDrawMs >= 250) {
    lastDrawMs = now;
    drawDashboard();
  }
}
