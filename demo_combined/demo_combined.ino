#include <Wire.h>

// ---------------------------------------------------------
// HARDWARE DEFINITIONS
// ---------------------------------------------------------
const int pin_ADC_DC    = 6;
const int pin_ADC_Solar = 7;
const int pin_Control_DC    = 4;
const int pin_Solar_Control = 5;
const int pin_CHRG_OK = 40;
const int pin_I2C_SCL = 41;
const int pin_I2C_SDA = 42;

// ---------------------------------------------------------
// BQ25713 REGISTERS & SETTINGS
// ---------------------------------------------------------
#define BQ25713_ADDR          0x6B 
#define REG_CHARGE_OPTION_0   0x00
#define REG_CHARGE_CURRENT    0x02
#define REG_MAX_CHARGE_VOLT   0x04
#define REG_VINDPM            0x0A
#define REG_IIN_DPM           0x22
#define REG_ADC_VBUS_VSYS     0x2C
#define REG_ADC_IIN           0x2A // Input Current ADC
#define REG_ADC_ICHG          0x2E // Charge Current ADC
#define REG_ADC_OPTION        0x3A

// Global State Variables
bool isChargingEnabled = false;
int inputMode = 3; // 1 = Force DC, 2 = Force Solar, 3 = Auto
int currentActiveInput = 0; // Tracks state for VINDPM changes

void setup() {
  Serial.begin(115200);
  
  pinMode(pin_Control_DC, OUTPUT);
  pinMode(pin_Solar_Control, OUTPUT);
  pinMode(pin_CHRG_OK, INPUT); 

  digitalWrite(pin_Control_DC, HIGH);
  digitalWrite(pin_Solar_Control, HIGH);

  Wire.begin(pin_I2C_SDA, pin_I2C_SCL, 100000);
  delay(2000);

  Serial.println("\n=========================================");
  Serial.println("  ESP32 + BQ25713 Power Manager Booting  ");
  Serial.println("=========================================");
  
  write16(REG_CHARGE_OPTION_0, 0x020E); 
  write16(REG_MAX_CHARGE_VOLT, 0x3130); // 12.592V Max
  write16(REG_CHARGE_CURRENT,  0x0000); // Start OFF (0A)
 // write16(REG_IIN_DPM,         0x0190); // 2.0A Input Limit
 // write16(REG_VINDPM,          0x36B0); // Start at 14.0V Default
  write16(REG_ADC_OPTION,      0xA0FF); // Enable ADCs

  Serial.println(" [s] Start Charging | [x] Stop Charging | [1] DC | [2] Solar | [3] Auto\n");
}

void loop() {
  handleSerialCommands();
  manageInputPower();
  printSystemStatus();
  delay(1000); 
}
void handleSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      // Force rewrite all critical registers
      write16(REG_CHARGE_OPTION_0, 0x020E); // WDT Off, Charging Enabled
      write16(REG_MAX_CHARGE_VOLT, 0x3130); // 12.592V 
      write16(REG_IIN_DPM,         0x2800); // 2.0A Input Limit
      write16(REG_CHARGE_CURRENT,  0x0400); // 1024mA Charge
      
      // >>> ADD THIS LINE TO TURN ON THE CONTINUOUS ADC VOLTMETER <<<
      write16(0x3A, 0xA0FF); 
      
      isChargingEnabled = true;
      Serial.println("\n>>> CHARGER WOKEN UP & ADC ENABLED <<<");
    }
    else if (cmd == 'x' || cmd == 'X') {
      write16(REG_CHARGE_CURRENT, 0x0000); 
      isChargingEnabled = false;
    }
    else if (cmd == 'd' || cmd == 'D') {
      dumpAllRegisters();
    }
    else if (cmd == '1') inputMode = 1;
    else if (cmd == '2') inputMode = 2;
    else if (cmd == '3') inputMode = 3;
  }
}

void manageInputPower() {
  float dcVoltage = (analogRead(pin_ADC_DC) / 4095.0) * 3.3 * 11.0;
  float solarVoltage = (analogRead(pin_ADC_Solar) / 4095.0) * 3.3 * 11.0;
  int selectedInput = 0;

  if (inputMode == 1) selectedInput = 1;
  else if (inputMode == 2) selectedInput = 2;
  else if (inputMode == 3) {
      // Prioritize Solar if it is genuinely high (above 13V). 
      // Backfed DC voltage will never exceed 9V here.
      if (solarVoltage >= 13.0) {
        selectedInput = 2;
      }
      // If Solar is missing, but DC is present (above 8.0V)
      else if (dcVoltage >= 8.0) {
        selectedInput = 1;
      }
    }
 if (selectedInput == 1 && currentActiveInput != 1) {
      // 1. STOP CHARGING FIRST to prevent transient faults
      write16(REG_CHARGE_CURRENT, 0x0000); 
      
      // 2. BREAK-BEFORE-MAKE: Turn both inputs OFF
      digitalWrite(pin_Solar_Control, HIGH);
      digitalWrite(pin_Control_DC, HIGH);
      delay(200); // Wait 200ms for VBUS to drain and chip to reset
      
      // 3. Turn ON DC Supply
      digitalWrite(pin_Control_DC, LOW);
      delay(200); // Wait 200ms for 9V to fully stabilize
      
      // 4. SET CORRECT LIMITS
      // VINDPM (High Byte 0x51 = 8.4V), IIN_DPM (Low Byte 0x0A = 500mA)
      write16(REG_VINDPM, 0x510A);  
      // Host Current Limit (0x0A00 = 500mA)
      write16(REG_IIN_DPM, 0x0A00); 
      
      // 5. RE-START CHARGING
      write16(REG_CHARGE_CURRENT, 0x0100); // 256mA
      
      Serial.println("\n*** SWITCHED TO DC: VINDPM=8.4V, IIN=0.5A, CHG=256mA ***");
      currentActiveInput = 1;
    }
    else if (selectedInput == 2 && currentActiveInput != 2) {
      write16(REG_CHARGE_CURRENT, 0x0000); 
      
      digitalWrite(pin_Control_DC, HIGH);
      digitalWrite(pin_Solar_Control, HIGH);
      delay(200);
      
      digitalWrite(pin_Solar_Control, LOW);
      delay(200);
      
      // VINDPM (High Byte 0xA8 = 14.0V), IIN_DPM (Low Byte 0x28 = 2.0A)
      write16(REG_VINDPM, 0xA828);  
      // Host Current Limit (0x2800 = 2.0A)
      write16(REG_IIN_DPM, 0x2800); 
      
      write16(REG_CHARGE_CURRENT, 0x0200); // 512mA
      
      Serial.println("\n*** SWITCHED TO SOLAR: VINDPM=14.0V, IIN=2.0A, CHG=512mA ***");
      currentActiveInput = 2;
    }
  /*if (selectedInput == 1 && currentActiveInput != 1) {
    digitalWrite(pin_Control_DC, LOW);    
    digitalWrite(pin_Solar_Control, HIGH); 
    // FIXED HEX: 8448mV -> 132 steps * 64mV = 0x2100
    write16(REG_VINDPM, 0x2100); 
    Serial.println("\n*** SWITCHED TO DC: VINDPM lowered to 8.4V ***");
    currentActiveInput = 1;
  } 
  else if (selectedInput == 2 && currentActiveInput != 2) {
    digitalWrite(pin_Control_DC, HIGH);    
    digitalWrite(pin_Solar_Control, LOW); 
    // FIXED HEX: 13952mV -> 218 steps * 64mV = 0x3680
    write16(REG_VINDPM, 0x3680); 
    Serial.println("\n*** SWITCHED TO SOLAR: VINDPM raised to 14.0V ***");
    currentActiveInput = 2;
  } */
  else if (selectedInput == 0 && currentActiveInput != 0) {
    digitalWrite(pin_Control_DC, HIGH);    
    digitalWrite(pin_Solar_Control, HIGH); 
    currentActiveInput = 0;
  }
}

/*void handleSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      write16(REG_CHARGE_CURRENT, 0x0400); // 1024mA
      isChargingEnabled = true;
    } 
    else if (cmd == 'x' || cmd == 'X') {
      write16(REG_CHARGE_CURRENT, 0x0000); // 0mA
      isChargingEnabled = false;
    }
    else if (cmd == '1') inputMode = 1;
    else if (cmd == '2') inputMode = 2;
    else if (cmd == '3') inputMode = 3;
  }
}

void manageInputPower() {
  float dcVoltage = (analogRead(pin_ADC_DC) / 4095.0) * 3.3 * 11.0;
  float solarVoltage = (analogRead(pin_ADC_Solar) / 4095.0) * 3.3 * 11.0;
  int selectedInput = 0;

  if (inputMode == 1) selectedInput = 1;
  else if (inputMode == 2) selectedInput = 2;
  else if (inputMode == 3) { 
    if (dcVoltage > (solarVoltage + 0.5) && dcVoltage >= 8.5) selectedInput = 1;
    else if (solarVoltage > (dcVoltage + 0.5) && solarVoltage >= 8.5) selectedInput = 2;
  }

  // Apply hardware state and update VINDPM if input changed
  if (selectedInput == 1 && currentActiveInput != 1) {
    digitalWrite(pin_Control_DC, LOW);    
    digitalWrite(pin_Solar_Control, HIGH); 
    write16(REG_VINDPM, 0x0520); // Lower VINDPM to 8.5V for DC Jack
    Serial.println("\n*** SWITCHED TO DC: VINDPM lowered to 8.5V ***");
    currentActiveInput = 1;
  } 
  else if (selectedInput == 2 && currentActiveInput != 2) {
    digitalWrite(pin_Control_DC, HIGH);    
    digitalWrite(pin_Solar_Control, LOW); 
    write16(REG_VINDPM, 0x36B0); // Raise VINDPM to 14.0V for Solar MPPT
    Serial.println("\n*** SWITCHED TO SOLAR: VINDPM raised to 14.0V ***");
    currentActiveInput = 2;
  }
  else if (selectedInput == 0 && currentActiveInput != 0) {
    digitalWrite(pin_Control_DC, HIGH);    
    digitalWrite(pin_Solar_Control, HIGH); 
    currentActiveInput = 0;
  }
}*/
void printSystemStatus() {
  // 1. Read VBUS (Register 0x26 High Byte)
  uint16_t vbus_reg = read16(0x26);
  uint8_t vbus_byte = (vbus_reg >> 8) & 0xFF;
  // VBUS Offset is 3.2V, scale is 64mV
  float vbusV = (vbus_byte != 0xFF && vbus_byte != 0x00) ? 3.2 + (vbus_byte * 0.064) : 0.0;

  // 2. Read VSYS and VBAT (Register 0x2C)
  uint16_t vsys_vbat_reg = read16(0x2C);
  uint8_t vsys_byte = (vsys_vbat_reg >> 8) & 0xFF;
  uint8_t vbat_byte = vsys_vbat_reg & 0xFF;
  
  // VSYS and VBAT Offset is 2.88V, scale is 64mV
  float vsysV = (vsys_byte != 0xFF && vsys_byte != 0x00) ? 2.88 + (vsys_byte * 0.064) : 0.0;
  float vbatV = (vbat_byte != 0xFF && vbat_byte != 0x00) ? 2.88 + (vbat_byte * 0.064) : 0.0;

  // 3. Read Input Current (Register 0x2A High Byte)
  uint16_t iin_reg = read16(0x2A);
  uint8_t iin_byte = (iin_reg >> 8) & 0xFF;
  // Input Current scale is 50mA per step
  float iin_A = (iin_byte != 0xFF) ? (iin_byte * 50.0) / 1000.0 : 0.0;

  // 4. Read Charge Current (Register 0x28 High Byte)
  uint16_t ichg_reg = read16(0x28);
  uint8_t ichg_byte = (ichg_reg >> 8) & 0xFF;
  // Charge Current scale is 64mA per step (assuming a 10mOhm sense resistor)
  float ichg_A = (ichg_byte != 0xFF) ? (ichg_byte * 64.0) / 1000.0 : 0.0; 
  
  // 5. Print the corrected Telemetry
  Serial.print("[IN] "); Serial.print(vbusV); Serial.print("V @ "); Serial.print(iin_A); Serial.print("A");
  Serial.print(" ---> [SYS] "); Serial.print(vsysV); Serial.print("V | [BAT] "); Serial.print(vbatV); Serial.print("V @ "); Serial.print(ichg_A); Serial.println("A");
}

/*void printSystemStatus() {
  uint16_t vbus_vsys_raw = read16(REG_ADC_VBUS_VSYS);
  uint16_t ichg_raw = read16(REG_ADC_ICHG);
  uint16_t iin_raw = read16(REG_ADC_IIN); 

  byte vbus_byte = (vbus_vsys_raw >> 8) & 0xFF;
  byte vsys_byte = vbus_vsys_raw & 0xFF;
  byte ichg_byte = (ichg_raw >> 8) & 0xFF;
  byte iin_byte  = (iin_raw >> 8) & 0xFF;

  float vbusV = (vbus_byte != 0xFF) ? (vbus_byte * 64.0) / 1000.0 : 0.0;
  float vsysV = (vsys_byte != 0xFF) ? (vsys_byte * 64.0) / 1000.0 : 0.0;
  float chargeCurrent = 0.0;
  float inputCurrent = 0.0;

  // Only read the Current ADCs if the charger is actually running.
  // When STOPPED, the internal amplifiers turn off and output garbage noise!
  if (isChargingEnabled) {
    chargeCurrent = (ichg_byte != 0xFF) ? (ichg_byte * 64.0) : 0.0;
    inputCurrent = (iin_byte != 0xFF) ? (iin_byte * 50.0) : 0.0; 
  }
  //float chargeCurrent = (ichg_byte != 0xFF) ? (ichg_byte * 64.0) : 0.0;
  //float inputCurrent = (iin_byte != 0xFF) ? (iin_byte * 50.0) : 0.0; 

  float powerInW = (vbusV * inputCurrent) / 1000.0;
  float powerOutW = (vsysV * chargeCurrent) / 1000.0;

  String activeInput = "NONE";
  if (currentActiveInput == 1) activeInput = "DC JACK";
  if (currentActiveInput == 2) activeInput = "SOLAR JACK";

  Serial.print("[STATE] ");
  Serial.print(isChargingEnabled ? "CHARGING" : "STOPPED ");
  Serial.print(" | Mode: ");
  if (inputMode == 1) Serial.print("FORCE DC");
  if (inputMode == 2) Serial.print("FORCE SOL");
  if (inputMode == 3) Serial.print("AUTO");
  
  Serial.print(" | Active: ");
  Serial.print(activeInput);
  Serial.print(" || [IN] ");
  Serial.print(vbusV, 2);
  Serial.print("V @ ");
  Serial.print(inputCurrent, 0);
  Serial.print("mA (");
  Serial.print(powerInW, 2);
  
  Serial.print("W) ---> [OUT] ");
  Serial.print(vsysV, 2);
  Serial.print("V @ ");
  Serial.print(chargeCurrent, 0);
  Serial.print("mA (");
  Serial.print(powerOutW, 2);
  Serial.println("W)");

  // Read Charge Status (0x20) and Fault Status (0x21)
  uint16_t chrgStatus = read16(0x20);
  uint16_t faultStatus = read16(0x22);

  Serial.print(" || DIAGNOSTICS -> Status: 0x");
  Serial.print(chrgStatus, HEX);
  Serial.print(" | Fault: 0x");
  Serial.println(faultStatus, HEX);
} */

void write16(byte reg, uint16_t data) {
  Wire.beginTransmission(BQ25713_ADDR);
  Wire.write(reg);
  Wire.write(data & 0xFF);        
  Wire.write((data >> 8) & 0xFF); 
  Wire.endTransmission();
}

uint16_t read16(byte reg) {
  Wire.beginTransmission(BQ25713_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ25713_ADDR, 2);
  if (Wire.available() == 2) {
    byte low = Wire.read();
    byte high = Wire.read();
    return (high << 8) | low;
  }
  return 0;
}

void dumpAllRegisters() {
  Serial.println("\n=================================");
  Serial.println("   BQ25713 FULL I2C X-RAY DUMP   ");
  Serial.println("=================================");
  
  // Read every even register from 0x00 to 0x3A
  for (byte reg = 0x00; reg <= 0x3A; reg += 2) {
    uint16_t val = read16(reg);
    
    Serial.print("REG 0x");
    if (reg < 0x10) Serial.print("0");
    Serial.print(reg, HEX);
    Serial.print(" : 0x");
    
    // Pad with leading zeros for clean 16-bit formatting
    if (val < 0x1000) Serial.print("0");
    if (val < 0x0100) Serial.print("0");
    if (val < 0x0010) Serial.print("0");
    Serial.println(val, HEX);
  }
  Serial.println("=================================\n");
}