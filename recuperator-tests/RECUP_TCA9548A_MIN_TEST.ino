#include <Arduino.h>
#include <Wire.h>

static constexpr int I2C_SDA = 5;
static constexpr int I2C_SCL = 6;
static constexpr uint32_t I2C_FREQ = 10000;
static constexpr uint8_t TCA_FIRST = 0x70;
static constexpr uint8_t TCA_LAST = 0x77;

String inputLine;
uint8_t activeTcaAddr = 0x70;
bool activeTcaFound = false;
uint32_t lastAutoScanMs = 0;

void printBusPins(const char *label, int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delay(5);
  Serial.printf("BUS %s SDA_GPIO=%d SDA=%d SCL_GPIO=%d SCL=%d\n",
                label,
                sda,
                digitalRead(sda),
                scl,
                digitalRead(scl));
}

void printBus(const char *label) {
  printBusPins(label, I2C_SDA, I2C_SCL);
}

uint8_t ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

bool tcaWrite(uint8_t addr, uint8_t mask) {
  Wire.beginTransmission(addr);
  Wire.write(mask);
  const uint8_t err = Wire.endTransmission();
  Serial.printf("TCA_WRITE addr=0x%02X mask=0x%02X err=%u ok=%u\n",
                addr,
                mask,
                err,
                err == 0 ? 1 : 0);
  return err == 0;
}

bool scanTcaOnPins(const char *label, int sda, int scl, bool testChannels) {
  Wire.end();
  Wire.begin(sda, scl);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(20);

  bool found = false;
  uint8_t foundAddr = 0x70;
  Serial.printf("TCA_PIN_SCAN_BEGIN %s SDA=%d SCL=%d\n", label, sda, scl);
  printBusPins("pin_scan_before", sda, scl);
  for (uint8_t addr = TCA_FIRST; addr <= TCA_LAST; addr++) {
    const uint8_t err = ping(addr);
    Serial.printf("TCA_PROBE %s addr=0x%02X err=%u found=%u\n", label, addr, err, err == 0 ? 1 : 0);
    if (!found && err == 0) {
      foundAddr = addr;
      found = true;
    }
    delay(10);
  }
  if (found) {
    Serial.printf("TCA_FOUND %s addr=0x%02X\n", label, foundAddr);
    activeTcaAddr = foundAddr;
    activeTcaFound = true;
    if (testChannels) {
      tcaWrite(foundAddr, 0x00);
      for (uint8_t ch = 0; ch < 8; ch++) {
        tcaWrite(foundAddr, (uint8_t)(1u << ch));
        delay(20);
      }
      tcaWrite(foundAddr, 0x00);
    }
  } else {
    Serial.printf("TCA_FOUND %s none\n", label);
  }
  printBusPins("pin_scan_after", sda, scl);
  Serial.printf("TCA_PIN_SCAN_END %s\n", label);
  return found;
}

void scanTca() {
  activeTcaFound = false;
  Serial.println();
  Serial.println("TCA_SCAN_BEGIN");
  scanTcaOnPins("normal", I2C_SDA, I2C_SCL, true);
  scanTcaOnPins("swapped", I2C_SCL, I2C_SDA, false);
  Serial.printf("TCA_ACTIVE %s addr=0x%02X\n", activeTcaFound ? "found" : "none", activeTcaAddr);
  Serial.println("TCA_SCAN_END");
}

void printHelp() {
  Serial.println();
  Serial.println("RECUP_TCA9548A_MIN_TEST 2026-07-12");
  Serial.printf("I2C pins: SDA=%d SCL=%d freq=%lu\n", I2C_SDA, I2C_SCL, (unsigned long)I2C_FREQ);
  Serial.println("Commands:");
  Serial.println("  scan      - scan TCA addresses 0x70..0x77 and test channel masks");
  Serial.println("  bus       - print raw SDA/SCL levels");
  Serial.println("  pins      - scan both pin maps: SDA/SCL and swapped");
  Serial.println("  off       - write mask 0x00 to detected/default TCA");
  Serial.println("  ch <0..7> - select one TCA channel");
  Serial.println("  help      - print this help");
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  if (line == "help" || line == "h") {
    printHelp();
  } else if (line == "scan") {
    scanTca();
  } else if (line == "bus") {
    printBus("manual");
  } else if (line == "pins") {
    scanTca();
  } else if (line == "off") {
    tcaWrite(activeTcaAddr, 0x00);
  } else if (line.startsWith("ch ")) {
    const int ch = constrain(line.substring(3).toInt(), 0, 7);
    tcaWrite(activeTcaAddr, (uint8_t)(1u << ch));
  } else {
    Serial.printf("ERR unknown command: %s\n", line.c_str());
  }
}

void serviceSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(inputLine);
      inputLine = "";
    } else if (inputLine.length() < 80) {
      inputLine += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println();
  printHelp();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(20);
  scanTca();
}

void loop() {
  serviceSerial();
  if (millis() - lastAutoScanMs > 5000) {
    lastAutoScanMs = millis();
    scanTca();
  }
}
