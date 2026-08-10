#include <Wire.h>

// --- ESP32-S3 I2C Pin Definitions ---
#define I2C_SDA 11
#define I2C_SCL 12
#define BQ76952_ADDR 0x08

// --- ESP32-S3 LoRa Hardware Serial (UART2) Pins ---
#define RXD2 44   // Connected to LoRa TX
#define TXD2 43   // Connected to LoRa RX
#define LORA_BAUD 9600

// --- MOSFET PWM Control Pin (IRLZ44N) ---
#define MOSFET_PWM_PIN 3      
#define PWM_FREQ       5000  
#define PWM_RESOLUTION 8

// --- BQ76952 Direct Commands ---
#define REG_SAFETY_STATUS_A 0x02
#define REG_SAFETY_STATUS_B 0x03
#define REG_SAFETY_STATUS_C 0x04
#define REG_ALARM_STATUS    0x62
#define REG_CELL1_VOLTAGE  0x14
#define REG_CELL2_VOLTAGE  0x16
#define REG_CELL16_VOLTAGE 0x32 
#define REG_STACK_VOLTAGE  0x34
#define REG_PACK_VOLTAGE   0x36
#define REG_CC2_CURRENT    0x3A 

// --- Temperature & FET Registers ---
#define REG_INT_TEMP       0x68 
#define REG_TS1_TEMP       0x70 
#define REG_FET_STATUS     0x7F 

// --- BQ76952 Subcommands & Data Memory ---
#define CMD_SUBCMD_LOWER  0x3E
#define CMD_SUBCMD_UPPER  0x3F
#define CMD_TRANSFER_BUF  0x40
#define CMD_CHECKSUM      0x60
#define CMD_LENGTH        0x61

#define SUBCMD_SET_CFGUPDATE  0x0090
#define SUBCMD_EXIT_CFGUPDATE 0x0092
#define MEM_VCELL_MODE        0x9304
#define MEM_FET_OPTIONS       0x9308

// --- FET Control Registers & Subcommands ---
#define REG_FET_STATUS     0x7F 
#define SUBCMD_FET_ENABLE  0x0022
#define MEM_FET_OPTIONS    0x9308

// --BQ76952 SLEEP

#define SUBCMD_DEEPSLEEP       0x000F
#define SUBCMD_EXIT_DEEPSLEEP  0x000E

// --- Global Telemetry Variables ---
uint16_t c1_mV = 0, c2_mV = 0, c3_mV = 0;
uint16_t stack_mV = 0, pack_mV = 0;
int16_t  current_mA = 0;
float    temp_int = 0.0, temp_ts1 = 0.0;
bool     dsg_on = false, chg_on = false;
uint8_t  statA = 0, statB = 0, statC = 0;
uint16_t alarmReg = 0;
uint8_t  current_pwm = 0;

// --- Coulomb Counter & SoC Variables ---
const float MAX_CAPACITY_MAH = 7000.0; 
float current_capacity_mAh = 7000.0;
unsigned long lastCoulombCalcTime = 0;
float current_soc = 100.0;
bool soc_initialized = false;

// --- Timing Variables ---
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 2000;

void setup() {
  Serial.begin(115200);
  Serial2.begin(LORA_BAUD, SERIAL_8N1, RXD2, TXD2); // Initialize LoRa Serial
  
// Initialize PWM for MOSFET (Updated for ESP32 Core v3.x)
  ledcAttach(MOSFET_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(MOSFET_PWM_PIN, current_pwm);
  
  delay(3000); 

  Wire.begin(I2C_SDA, I2C_SCL, 100000); 
  
  Serial.println("BMS ESP32-S3 Initialized. Configuring BQ76952...");
  delay(1000);

  configureFor3S();
  enableAutonomousFETControl();
  
  // Clear any latent faults after setup
  clearLatchedFaults();
  
  lastCoulombCalcTime = millis();
}


void loop() {
  // 1. Non-Blocking Telemetry Reading & LoRa Transmission
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = millis();
    
    // Read all data from BQ76952
    readCellVoltages();
    readSystemVoltages();
    readCurrent();
    readTemperatures();
    checkFETStatus();
    readSafetyStatus();
    
    // Process Coulomb Counter
    updateCoulombCounter();
    
    // Transmit to Base Station via LoRa
    transmitLoRaData();
  }

  // 2. Non-Blocking Command Parser (Listening to Base Station)
  if (Serial2.available()) {
    String command = Serial2.readStringUntil('\n');
    command.trim();
    
    if (command == "SLEEP") {
      Serial.println("Base Station commanded SLEEP.");
      // Enter Deep Sleep / Turn off FETs logic here
    } 
    else if (command == "WAKE") {
      Serial.println("Base Station commanded WAKE.");
      // Wake / Re-enable FETs logic here
    } 
    else {
      // Try to parse as PWM Duty Cycle (0-255)
      int newPwm = command.toInt();
      if (newPwm >= 0 && newPwm <= 255) {
        current_pwm = newPwm;
        ledcWrite(0, current_pwm);
        Serial.printf("Base Station commanded PWM: %d\n", current_pwm);
      }
    }
  }
}

// ---------------------------------------------------------
// COULOMB COUNTING MATH
// ---------------------------------------------------------
void initializeSoCFromVoltage() {
  if (pack_mV == 0) return; // Wait for valid reading
  
  float cell_v = (pack_mV / 1000.0) / 3.0; 
  float estimated_soc = 0;

  // Rough estimation curve based on open circuit voltage
  if (cell_v >= 4.20) estimated_soc = 100.0;
  else if (cell_v >= 4.00) estimated_soc = 80.0 + ((cell_v - 4.00) / 0.20) * 20.0;
  else if (cell_v >= 3.80) estimated_soc = 40.0 + ((cell_v - 3.80) / 0.20) * 40.0;
  else if (cell_v >= 3.30) estimated_soc = 5.0 + ((cell_v - 3.30) / 0.50) * 35.0;  
  else estimated_soc = 0.0;

  current_capacity_mAh = (estimated_soc / 100.0) * MAX_CAPACITY_MAH;
  current_soc = estimated_soc;
  soc_initialized = true;

  Serial.printf("SoC Initialized at %.1f%% based on %.2fV per cell\n", current_soc, cell_v);
}

void updateCoulombCounter() {
  // If this is our first reading, initialize the SoC based on voltage
  if (!soc_initialized) {
    initializeSoCFromVoltage();
  }

  unsigned long now = millis();
  unsigned long dt_ms = now - lastCoulombCalcTime;
  
  if (dt_ms > 0 && soc_initialized) {
    // Convert ms to hours to calculate milliamp-hours
    float dt_hours = dt_ms / 3600000.0;
    
    // Add (or subtract) the current integration
    // Note: If discharging, current_mA should be negative, dropping capacity.
    current_capacity_mAh += (current_mA * dt_hours);
    
    // Synchronize to physical bounds to prevent drift
    if (current_capacity_mAh > MAX_CAPACITY_MAH) current_capacity_mAh = MAX_CAPACITY_MAH;
    if (current_capacity_mAh < 0.0) current_capacity_mAh = 0.0;
    
    current_soc = (current_capacity_mAh / MAX_CAPACITY_MAH) * 100.0;
  }
  
  lastCoulombCalcTime = now;
}

// ---------------------------------------------------------
// LORA TRANSMISSION
// ---------------------------------------------------------
void transmitLoRaData() {
  // Construct the payload string. Added "SOC:%.1f" at the end.
  char payload[256];
  snprintf(payload, sizeof(payload), 
           "BMS,C1:%hu,C2:%hu,C3:%hu,STACK:%hu,PACK:%hu,CURR:%hd,T_INT:%.1f,T_TS1:%.1f,DSG:%d,CHG:%d,PWM:%hhu,STA:%d,STB:%d,STC:%d,ALM:%d,SOC:%.1f",
           c1_mV, c2_mV, c3_mV, stack_mV, pack_mV, current_mA,
           temp_int, temp_ts1, dsg_on ? 1 : 0, chg_on ? 1 : 0, current_pwm,
           statA, statB, statC, alarmReg, current_soc);
           
  // Send via Hardware Serial 2 (LoRa TX)
  Serial2.println(payload);
  
  // Also print to USB for debugging
  Serial.print("Transmitting: ");
  Serial.println(payload);
}

// ---------------------------------------------------------
// BQ76952 CONFIGURATION & FIXES
// ---------------------------------------------------------
void configureFor3S() {
  Serial.println("Writing VCell Mode to 3S (0x8003)...");
  sendSubcommand(SUBCMD_SET_CFGUPDATE);
  delay(5); 
  writeDataMemory(MEM_VCELL_MODE, 0x8003);
  delay(5);
  sendSubcommand(SUBCMD_EXIT_CFGUPDATE);
  delay(5);
}

void enableAutonomousFETControl() {
  Serial.println("Configuring Autonomous FET Control...");
  
  sendSubcommand(SUBCMD_SET_CFGUPDATE);
  delay(2); 

  // The default value of FET Options is 0x0D. 
  // Bit 4 is FET_CTRL_EN. Setting Bit 4 changes 0x0D to 0x1D.
  Serial.println("Writing FET Options (0x1D)...");
  writeDataMemoryByte(MEM_FET_OPTIONS, 0x1D);
  delay(2);

  sendSubcommand(SUBCMD_EXIT_CFGUPDATE);
  delay(2);
  
  // Wait a moment for the BQ76952 to process the exit command
  // and for its internal state machine to evaluate safety and turn the FETs on.
  delay(100); 
  Serial.println("Autonomous FET Control Enabled.");

  // ---> ADD THIS BACK IN! <---
  Serial.println("Sending FET_ENABLE Command...");
  sendSubcommand(SUBCMD_FET_ENABLE); // 0x0022 toggles the master FET_EN flag to 1
  delay(50);
}

void clearLatchedFaults() {
  // Wait for the BQ76952's internal 2-second CUV timer to expire
  delay(3000); 
  
  Serial.println("Clearing any latched safety faults...");
  // Write 0x0000 to the Alarm Status register to clear protections
  Wire.beginTransmission(BQ76952_ADDR);
  Wire.write(REG_ALARM_STATUS);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
}

void enterDeepSleep() {
  Serial.println("Entering DEEPSLEEP Mode...");
  sendSubcommand(SUBCMD_DEEPSLEEP);
  delay(10);
  Serial.println("BQ76952 is now in DEEPSLEEP.");
}

void exitDeepSleep() {
  Serial.println("Waking from DEEPSLEEP Mode...");
  sendSubcommand(SUBCMD_EXIT_DEEPSLEEP);
  delay(10);
  Serial.println("BQ76952 is WAKE.");
}

// ---------------------------------------------------------
// BQ76952 READ REGISTERS
// ---------------------------------------------------------
uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(BQ76952_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0; 
  Wire.requestFrom((uint16_t)BQ76952_ADDR, (uint8_t)2, true);
  if (Wire.available() >= 2) {
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    return (msb << 8) | lsb;
  }
  return 0;
}

uint8_t readRegister8(uint8_t reg) {
  Wire.beginTransmission(BQ76952_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0; 
  Wire.requestFrom((uint16_t)BQ76952_ADDR, (uint8_t)1, true);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void checkFETStatus() {
  uint8_t status = readRegister8(REG_FET_STATUS);
  dsg_on = (status & 0x04) >> 2; 
  chg_on = (status & 0x02) >> 1; 
}

void readCellVoltages() {
  c1_mV = readRegister16(REG_CELL1_VOLTAGE);
  c2_mV = readRegister16(REG_CELL2_VOLTAGE);
  c3_mV = readRegister16(REG_CELL16_VOLTAGE); 
}

void readSystemVoltages() {
  stack_mV = readRegister16(REG_STACK_VOLTAGE);
  pack_mV = readRegister16(REG_PACK_VOLTAGE);
}

void readCurrent() {
  current_mA = (int16_t)readRegister16(REG_CC2_CURRENT);
}

float convertTempToCelsius(uint16_t rawTemp) {
  if (rawTemp == 0) return 0.0;
  return (rawTemp * 0.1) - 273.15;
}

void readTemperatures() {
  uint16_t intTempRaw = readRegister16(REG_INT_TEMP);
  temp_int = convertTempToCelsius(intTempRaw);

  uint16_t ts1Raw = readRegister16(REG_TS1_TEMP);
  temp_ts1 = convertTempToCelsius(ts1Raw);
}

void readSafetyStatus() {
  statA = readRegister8(REG_SAFETY_STATUS_A);
  statB = readRegister8(REG_SAFETY_STATUS_B);
  statC = readRegister8(REG_SAFETY_STATUS_C);
  alarmReg = readRegister16(REG_ALARM_STATUS);
}

// ---------------------------------------------------------
// I2C DATA MEMORY WRITERS
// ---------------------------------------------------------
void sendSubcommand(uint16_t command) {
  Wire.beginTransmission(BQ76952_ADDR);
  Wire.write(CMD_SUBCMD_LOWER);
  Wire.write(command & 0xFF);         
  Wire.write((command >> 8) & 0xFF);  
  Wire.endTransmission();
}

void writeDataMemory(uint16_t address, uint16_t data) {
  uint8_t addr_lsb = address & 0xFF;
  uint8_t addr_msb = (address >> 8) & 0xFF;
  uint8_t data_lsb = data & 0xFF;
  uint8_t data_msb = (data >> 8) & 0xFF;
  uint8_t checksum = ~(addr_msb + addr_lsb + data_msb + data_lsb) & 0xFF;

  Wire.beginTransmission(BQ76952_ADDR);
  Wire.write(CMD_SUBCMD_LOWER);
  Wire.write(addr_lsb);
  Wire.write(addr_msb);
  Wire.endTransmission();
  delay(2);

  Wire.beginTransmission(BQ76952_ADDR);
  Wire.write(CMD_TRANSFER_BUF);
  Wire.write(data_lsb);
  Wire.write(data_msb);
  Wire.endTransmission();
  delay(2);

  Wire.beginTransmission(BQ76952_ADDR);
  Wire.write(CMD_CHECKSUM);
  Wire.write(checksum);
  Wire.write(0x04); 
  Wire.endTransmission();
  delay(2);
}

void writeDataMemoryByte(uint16_t address, uint8_t data) {
  uint8_t addr_lsb = address & 0xFF;
  uint8_t addr_msb = (address >> 8) & 0xFF;
  uint8_t checksum = ~(addr_msb + addr_lsb + data) & 0xFF;

  Wire.beginTransmission(BQ76952_ADDR);
  Wire.write(CMD_SUBCMD_LOWER);
  Wire.write(addr_lsb);
  Wire.write(addr_msb);
  Wire.endTransmission();
  delay(2);

  Wire.beginTransmission(BQ76952_ADDR);
  Wire.write(CMD_TRANSFER_BUF);
  Wire.write(data);
  Wire.endTransmission();
  delay(2);

  Wire.beginTransmission(BQ76952_ADDR);
  Wire.write(CMD_CHECKSUM);
  Wire.write(checksum);
  Wire.write(0x03); 
  Wire.endTransmission();
  delay(2);
}