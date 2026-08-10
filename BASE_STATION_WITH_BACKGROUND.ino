#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <FS.h>
#include <SD.h>
#include <PNGdec.h> 

// --- LORA PINS & TOUCH PINS ---
#define LORA_RX_PIN 16  
#define LORA_TX_PIN 17  
#define LORA_BAUD   9600

#define TOUCH_CS   33
#define TOUCH_IRQ  36 
#define TOUCH_MOSI 32 
#define TOUCH_MISO 39 
#define TOUCH_CLK  25 

#define SD_CS      4  // SD Card Chip Select 

#define TS_MINX 300
#define TS_MAXX 3800
#define TS_MINY 300
#define TS_MAXY 3800

TFT_eSPI tft = TFT_eSPI();
SPIClass touchSPI = SPIClass(HSPI); 
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);
PNG png;

// --- Parsed BMS Metrics ---
uint16_t c1_mV = 0, c2_mV = 0, c3_mV = 0;
uint16_t stack_mV = 0, pack_mV = 0;
int16_t  current_mA = 0;
float    temp_int = 0.0, temp_ts1 = 0.0;
int      dsg_fet_int = 0, chg_fet_int = 0;
bool     dsg_fet = false, chg_fet = false;
uint8_t  bms_pwm = 0;

// --- Parsed Diagnostics ---
int      statA_int = 0, statB_int = 0, statC_int = 0, alarm_int = 0;
uint8_t  statA = 0, statB = 0, statC = 0;
uint16_t alarmReg = 0;

// --- Connection Tracker ---
unsigned long lastPacketTime = 0;
const unsigned long TIMEOUT_MS = 4000; 
bool isConnected = false;

// --- Capacity & SoC ---
const float MAX_CAPACITY_MAH = 7000.0; 
float current_soc = 100.0; // Handled by BMS now

// --- Graphing Arrays ---
#define GRAPH_POINTS 100
int16_t voltage_history[GRAPH_POINTS] = {0};
int16_t current_history[GRAPH_POINTS] = {0};
uint8_t graph_index = 0;

// --- UI State Trackers ---
uint8_t currentPage = 0; 
unsigned long lastTouchTime = 0;
const unsigned long DEBOUNCE_MS = 300;

// Slider settings
#define SLIDER_X 20
#define SLIDER_Y 70
#define SLIDER_W 200
#define SLIDER_H 25
int sliderVal = 0;
int lastSentSliderVal = -1;

// Forward declarations
void refreshFullUI();
void updateConnectionStatus(bool connected);
void updateCellBarGraphs();
void updateMetricsDisplay();
void drawSliderUI(int val);
void updateDiagnosticsDisplay();
void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t textColor);
void updateGraphDisplay();
void drawGraph(int x, int y, int w, int h, int16_t *data, uint16_t color, const char* title);
void drawBackground();

// --- PNG DECODER CALLBACKS ---
File pngFile;
void * pngOpen(const char *filename, int32_t *size) {
  pngFile = SD.open(filename, FILE_READ);
  if (pngFile) *size = pngFile.size();
  return (void *)&pngFile;
}
void pngClose(void *handle) {
  if (pngFile) pngFile.close();
}
int32_t pngRead(PNGFILE *handle, uint8_t *buffer, int32_t length) {
  if (!pngFile) return 0;
  return pngFile.read(buffer, length);
}
int32_t pngSeek(PNGFILE *handle, int32_t position) {
  if (!pngFile) return 0;
  return pngFile.seek(position);
}
// Updated from void to int
int pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[320]; // Adjust 320 to your screen's width if different
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(0, pDraw->y, pDraw->iWidth, 1, lineBuffer);
  return 1; // Required by the latest PNGdec library
}


void setup() {
  Serial.begin(115200);
  Serial2.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  // Hardware Locks
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);

  tft.init();
  tft.setRotation(0); 

  // ADD THIS LINE HERE:
  tft.invertDisplay(false); // Change to false if 'true' makes it worse
  
  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  ts.begin(touchSPI);
  ts.setRotation(0);

  SPI.begin();
  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("SD Card Mount Failed.");
  } else {
    Serial.println("SD Card Mounted.");
  }

  refreshFullUI();
  updateConnectionStatus(false);
}

void loop() {
  readIncomingBmsData();
  checkConnectionTimeout();
  handleTouchInput();
  delay(5);
}

// ---------------------------------------------------------
// UI RENDERING ENGINE
// ---------------------------------------------------------

void drawBackground() {
  int16_t rc = png.open("/IMG1.png", pngOpen, pngClose, pngRead, pngSeek, pngDraw);
  if (rc == PNG_SUCCESS) {
    png.decode(NULL, 0); 
    png.close();
  } else {
    tft.fillScreen(TFT_BLACK); 
  }
}

void refreshFullUI() {
  drawBackground(); 

  tft.fillRect(0, 0, tft.width(), 28, TFT_NAVY);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setTextSize(1);
  
  const char* title = "BMS DASHBOARD";
  if (currentPage == 1) title = "BMS CONTROLS";
  if (currentPage == 2) title = "DIAGNOSTICS";
  if (currentPage == 3) title = "LIVE GRAPHS";
  tft.drawString(title, 8, 10);
  
  tft.drawFastHLine(0, 28, tft.width(), TFT_WHITE);
  drawButton(180, 2, 55, 24, "PAGE", TFT_BLUE, TFT_WHITE);
  updateConnectionStatus(isConnected);

  if (currentPage == 0) {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("CELL GROUP VOLTAGES", 10, 35);
    tft.drawString("SYSTEM METRICS", 10, 135);
    updateCellBarGraphs();
    updateMetricsDisplay();
  } 
  else if (currentPage == 1) {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("IRLZ44N PWM CONTROL", 10, 40);
    drawSliderUI(sliderVal);
    
    // Power Control Buttons
    tft.drawString("BQ76952 SYSTEM POWER", 10, 160);
    drawButton(20, 180, 80, 35, "SLEEP", TFT_PURPLE, TFT_WHITE);
    drawButton(120, 180, 80, 35, "WAKE", TFT_ORANGE, TFT_BLACK);
  }
  else if (currentPage == 2) {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("BQ76952 SAFETY STATUS", 10, 35);
    updateDiagnosticsDisplay();
  }
  else if (currentPage == 3) {
    updateGraphDisplay();
  }
}

void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t textColor) {
  tft.fillRoundRect(x, y, w, h, 4, bgColor);
  tft.drawRoundRect(x, y, w, h, 4, TFT_WHITE);
  tft.setTextColor(textColor, bgColor);
  int textWidth = strlen(label) * 6;
  tft.drawString(label, x + (w/2) - (textWidth/2), y + (h/2) - 4);
}

void updateConnectionStatus(bool connected) {
  int badgeX = 110; int badgeY = 6; int badgeW = 65; int badgeH = 16;
  tft.fillRect(badgeX, badgeY, badgeW, badgeH, connected ? TFT_GREEN : TFT_RED);
  tft.setTextColor(connected ? TFT_BLACK : TFT_WHITE, connected ? TFT_GREEN : TFT_RED);
  tft.drawString(connected ? " ONLINE" : " OFFLINE", badgeX + 2, badgeY + 4);
}

void checkConnectionTimeout() {
  if (isConnected && (millis() - lastPacketTime > TIMEOUT_MS)) {
    isConnected = false;
    updateConnectionStatus(false);
  }
}

// ---- DYNAMIC UPDATES ----
void updateCellBarGraphs() {
  if (currentPage != 0) return;
  int barWidth = 45; int maxHeight = 60; int startY = 115;
  int barXs[3] = {25, 95, 165};
  uint16_t cellMvs[3] = {c1_mV, c2_mV, c3_mV};

  for (int i = 0; i < 3; i++) {
    int fillHeight = map(constrain(cellMvs[i], 3000, 4200), 3000, 4200, 0, maxHeight);
    uint16_t barColor = (cellMvs[i] < 3300) ? TFT_RED : (cellMvs[i] < 3600 ? TFT_YELLOW : TFT_GREEN);
    
    tft.drawRect(barXs[i], startY - maxHeight, barWidth, maxHeight, TFT_WHITE);
    tft.fillRect(barXs[i] + 1, startY - maxHeight + 1, barWidth - 2, maxHeight - 2, TFT_BLACK);
    if (fillHeight > 0) tft.fillRect(barXs[i] + 2, startY - fillHeight + 1, barWidth - 4, fillHeight - 2, barColor);
    
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(barXs[i] + 2, startY + 5);
    tft.printf("%.2fV", cellMvs[i] / 1000.0);
  }
}

void updateMetricsDisplay() {
  if (currentPage != 0) return; 
// Correct the 10x scaling drop from the BQ76952 PACK register
  uint32_t corrected_pack_mV = (pack_mV < 2000) ? (uint32_t)pack_mV * 10 : pack_mV;
  uint32_t corrected_stack_mV = (stack_mV < 2000) ? (uint32_t)stack_mV * 10 : stack_mV;

  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.setCursor(10, 150); tft.printf("Stack Volts: %.2f V   ", stack_mV / 1000.0);
  tft.setCursor(10, 165); tft.printf("Pack Volts : %.2f V   ", corrected_pack_mV / 1000.0);
  tft.setCursor(10, 180); tft.printf("Die Temp   : %.1f deg C  ", temp_int);
  tft.setCursor(10, 195); tft.printf("TS1 Temp   : %.1f deg C  ", temp_ts1);
  
  tft.setCursor(10, 215); tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.printf("DSG FET: %s  ", dsg_fet ? "ON " : "OFF");
  tft.printf("| CHG FET: %s", chg_fet ? "ON " : "OFF");
  
  tft.setCursor(10, 235); tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.printf("State of Charge: %.1f%%   ", current_soc);
}

void drawSliderUI(int val) {
  if (currentPage != 1) return;

  // --- ANTI-SMEAR FIX ---
  // Erase the *entire* oversized handle bounding box before drawing
  tft.fillRect(SLIDER_X - 2, SLIDER_Y - 4, SLIDER_W + 20, SLIDER_H + 8, TFT_BLACK);

  // Draw Background Track
  tft.fillRect(SLIDER_X, SLIDER_Y, SLIDER_W, SLIDER_H, TFT_DARKGREY);
  tft.drawRect(SLIDER_X - 1, SLIDER_Y - 1, SLIDER_W + 2, SLIDER_H + 2, TFT_WHITE);
  
  int handleX = map(val, 0, 255, SLIDER_X, SLIDER_X + SLIDER_W - 15);
  tft.fillRect(SLIDER_X, SLIDER_Y, handleX - SLIDER_X, SLIDER_H, TFT_CYAN);
  
  // Draw Handle (which is taller than the track)
  tft.fillRect(handleX, SLIDER_Y - 3, 15, SLIDER_H + 6, TFT_ORANGE);
  tft.drawRect(handleX, SLIDER_Y - 3, 15, SLIDER_H + 6, TFT_WHITE);
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(80, SLIDER_Y + 35);
  tft.printf("Duty: %d / 255   ", val);
}

void updateDiagnosticsDisplay() {
  if (currentPage != 2) return;
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 50);
  tft.printf("RAW: A:0x%02X B:0x%02X C:0x%02X ALM:0x%04X", statA, statB, statC, alarmReg);

  int y = 75;
  auto printFault = [&](const char* name, bool isFault) {
    tft.setCursor(10, y);
    if (isFault) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.printf("[FAIL] %s          ", name);
    } else {
      tft.setTextColor(TFT_DARKGREEN, TFT_BLACK);
      tft.printf("[ OK ] %s          ", name);
    }
    y += 18;
  };

  printFault("CUV  (Under Voltage)",   (statA & 0x04) != 0);
  printFault("COV  (Over Voltage)",    (statA & 0x08) != 0);
  printFault("OCC  (Overcurr Charge)", (statB & 0x02) != 0);
  printFault("OCD  (Overcurr Dischg)", (statB & 0x01) != 0);
  printFault("SCD  (Short Circuit)",   (statB & 0x04) != 0);
  printFault("OTD  (Overtemp Dischg)", (statC & 0x01) != 0);
  printFault("OTC  (Overtemp Charge)", (statC & 0x02) != 0);
  printFault("UTD  (Undertemp Dischg)",(statC & 0x04) != 0);

  bool hasAnyFault = (statA != 0) || (statB != 0) || (statC != 0);
  tft.fillRect(10, 240, 220, 35, hasAnyFault ? TFT_MAROON : TFT_DARKGREEN);
  tft.drawRect(10, 240, 220, 35, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, hasAnyFault ? TFT_MAROON : TFT_DARKGREEN);
  tft.setCursor(20, 252);
  tft.printf(hasAnyFault ? "SYSTEM STATUS: FAULT LOCKED!" : "SYSTEM STATUS: ALL CLEAR");
}

void updateGraphDisplay() {
  if (currentPage != 3) return;
  drawGraph(10, 50, 180, 80, voltage_history, TFT_GREEN, "Pack Voltage (mV)");
  drawGraph(10, 170, 180, 80, current_history, TFT_RED, "Pack Current (mA)");
}

void drawGraph(int x, int y, int w, int h, int16_t *data, uint16_t color, const char* title) {
  tft.drawRect(x, y, w, h, TFT_DARKGREY);
  tft.fillRect(x + 1, y + 1, w - 2, h - 2, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.drawString(title, x, y - 15);
  
  int16_t maxVal = data[0]; int16_t minVal = data[0];
  for (int i = 0; i < GRAPH_POINTS; i++) {
    if (data[i] > maxVal) maxVal = data[i];
    if (data[i] < minVal) minVal = data[i];
  }
  if (maxVal == minVal) { maxVal += 10; minVal -= 10; }

  int prev_x = x;
  int prev_y = y + h - 1 - map(data[graph_index], minVal, maxVal, 0, h - 2);

  for (int i = 1; i < GRAPH_POINTS; i++) {
    int idx = (graph_index + i) % GRAPH_POINTS;
    int curr_x = x + (i * w / GRAPH_POINTS);
    int curr_y = y + h - 1 - map(data[idx], minVal, maxVal, 0, h - 2);
    
    tft.drawLine(prev_x, prev_y, curr_x, curr_y, color);
    prev_x = curr_x; prev_y = curr_y;
  }
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(x + w + 5, y); tft.printf("%d   ", maxVal);
  tft.setCursor(x + w + 5, y + h - 10); tft.printf("%d   ", minVal);
}

// ---------------------------------------------------------
// TOUCH CONTROLS & PARSER
// ---------------------------------------------------------

bool isHit(int tx, int ty, int bx, int by, int bw, int bh) {
  return (tx >= bx && tx <= bx + bw && ty >= by && ty <= by + bh);
}

void handleTouchInput() {
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    int x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
    int y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());

    // Header Swap Button
    if (isHit(x, y, 180, 2, 55, 24)) {
      if (millis() - lastTouchTime > DEBOUNCE_MS) {
        lastTouchTime = millis();
        currentPage = (currentPage + 1) % 4; 
        refreshFullUI(); 
      }
      return;
    }

    // Control Page Interactions
    if (currentPage == 1) {
      // 1. Slider Touch Logic
      if (isHit(x, y, SLIDER_X - 10, SLIDER_Y - 10, SLIDER_W + 20, SLIDER_H + 20)) {
        sliderVal = map(constrain(x, SLIDER_X, SLIDER_X + SLIDER_W - 15), SLIDER_X, SLIDER_X + SLIDER_W - 15, 0, 255);
        drawSliderUI(sliderVal);

        if (millis() - lastTouchTime > 100 && sliderVal != lastSentSliderVal) {
          lastTouchTime = millis();
          lastSentSliderVal = sliderVal;
          Serial2.println(sliderVal);
        }
      }

      // 2. Sleep/Wake Button Logic
      if (millis() - lastTouchTime > DEBOUNCE_MS) {
        if (isHit(x, y, 20, 180, 80, 35))  { 
          Serial2.println("SLEEP"); 
          Serial.println("Transmitting: SLEEP");
          lastTouchTime = millis(); 
        }
        if (isHit(x, y, 120, 180, 80, 35)) { 
          Serial2.println("WAKE");  
          Serial.println("Transmitting: WAKE");
          lastTouchTime = millis(); 
        }
      }
    }
  }
}

void readIncomingBmsData() {
  if (Serial2.available()) {
    String payload = Serial2.readStringUntil('\n');
    payload.trim();

    if (payload.length() > 0) {
      lastPacketTime = millis();
      
      if (!isConnected) {
        isConnected = true;
        updateConnectionStatus(true);
      }

      if (payload.startsWith("BMS,")) {
        int parsed = sscanf(payload.c_str(), 
               "BMS,C1:%hu,C2:%hu,C3:%hu,STACK:%hu,PACK:%hu,CURR:%hd,T_INT:%f,T_TS1:%f,DSG:%d,CHG:%d,PWM:%hhu,STA:%d,STB:%d,STC:%d,ALM:%d,SOC:%f",
               &c1_mV, &c2_mV, &c3_mV, &stack_mV, &pack_mV, &current_mA,
               &temp_int, &temp_ts1, &dsg_fet_int, &chg_fet_int, &bms_pwm,
               &statA_int, &statB_int, &statC_int, &alarm_int, &current_soc);

        if (parsed == 16) {
            dsg_fet = (dsg_fet_int != 0);
            chg_fet = (chg_fet_int != 0);
            statA = (uint8_t)statA_int;
            statB = (uint8_t)statB_int;
            statC = (uint8_t)statC_int;
            alarmReg = (uint16_t)alarm_int;
            
            voltage_history[graph_index] = pack_mV;
            current_history[graph_index] = current_mA;
            graph_index = (graph_index + 1) % GRAPH_POINTS;
    
            if (currentPage == 0) {
              updateCellBarGraphs();
              updateMetricsDisplay();
            } else if (currentPage == 2) {
              updateDiagnosticsDisplay();
            } else if (currentPage == 3) {
              updateGraphDisplay(); 
            }
        } else {
            Serial.println("Warning: Dropped corrupted LoRa packet.");
        }
      }
    }
  }
}