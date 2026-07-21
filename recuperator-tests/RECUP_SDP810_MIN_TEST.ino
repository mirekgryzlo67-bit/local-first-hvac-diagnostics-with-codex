#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>

static constexpr int I2C_SDA = 8;
static constexpr int I2C_SCL = 9;
static constexpr int DS18B20_PIN = 10;
static constexpr uint32_t I2C_FREQ = 10000;
static constexpr uint8_t SDP810_ADDR = 0x25;
static constexpr uint8_t SHT40_ADDR = 0x44;
static constexpr uint8_t DIMMER_WYWIEW_ADDR = 0x50;
static constexpr uint8_t DIMMER_NAWIEW_ADDR = 0x51;
static constexpr uint8_t TCA9548A_ADDR = 0x70;
static constexpr uint16_t SDP810_CMD_START_AVG = 0x361E;
static constexpr uint16_t SDP810_CMD_STOP = 0x3FF9;

String inputLine;
uint32_t lastAutoMs = 0;
OneWire dsBus(DS18B20_PIN);

void printBus(const char *label) {
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(5);
  Serial.printf("BUS %s SDA_GPIO=%d SDA=%d SCL_GPIO=%d SCL=%d\n",
                label,
                I2C_SDA,
                digitalRead(I2C_SDA),
                I2C_SCL,
                digitalRead(I2C_SCL));
}

uint8_t ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

void bbSdaRelease() {
  pinMode(I2C_SDA, INPUT_PULLUP);
}

void bbSclRelease() {
  pinMode(I2C_SCL, INPUT_PULLUP);
}

void bbSdaLow() {
  pinMode(I2C_SDA, OUTPUT);
  digitalWrite(I2C_SDA, LOW);
}

void bbSclLow() {
  pinMode(I2C_SCL, OUTPUT);
  digitalWrite(I2C_SCL, LOW);
}

uint16_t bbHalfPeriodFor(uint32_t freqHz) {
  if (freqHz >= 100000) return 5;
  if (freqHz >= 50000) return 10;
  return 50;
}

void bbDelay(uint16_t halfPeriodUs) {
  delayMicroseconds(halfPeriodUs);
}

void bbStart(uint16_t halfPeriodUs) {
  bbSdaRelease();
  bbSclRelease();
  bbDelay(halfPeriodUs);
  bbSdaLow();
  bbDelay(halfPeriodUs);
  bbSclLow();
  bbDelay(halfPeriodUs);
}

void bbStop(uint16_t halfPeriodUs) {
  bbSdaLow();
  bbDelay(halfPeriodUs);
  bbSclRelease();
  bbDelay(halfPeriodUs);
  bbSdaRelease();
  bbDelay(halfPeriodUs);
}

bool bbWriteByte(uint8_t value, uint16_t halfPeriodUs) {
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
    if (value & mask) bbSdaRelease();
    else bbSdaLow();
    bbDelay(halfPeriodUs);
    bbSclRelease();
    bbDelay(halfPeriodUs);
    bbSclLow();
    bbDelay(halfPeriodUs);
  }

  bbSdaRelease();
  bbDelay(halfPeriodUs);
  bbSclRelease();
  bbDelay(halfPeriodUs);
  const bool ack = digitalRead(I2C_SDA) == LOW;
  bbSclLow();
  bbDelay(halfPeriodUs);
  return ack;
}

uint8_t bbReadByte(bool ack, uint16_t halfPeriodUs) {
  uint8_t value = 0;
  bbSdaRelease();
  for (uint8_t i = 0; i < 8; i++) {
    value <<= 1;
    bbDelay(halfPeriodUs);
    bbSclRelease();
    bbDelay(halfPeriodUs);
    if (digitalRead(I2C_SDA) == HIGH) value |= 0x01;
    bbSclLow();
    bbDelay(halfPeriodUs);
  }

  if (ack) bbSdaLow();
  else bbSdaRelease();
  bbDelay(halfPeriodUs);
  bbSclRelease();
  bbDelay(halfPeriodUs);
  bbSclLow();
  bbSdaRelease();
  bbDelay(halfPeriodUs);
  return value;
}

bool bbPing(uint8_t addr, uint32_t freqHz) {
  const uint16_t halfPeriodUs = bbHalfPeriodFor(freqHz);
  bbStart(halfPeriodUs);
  const bool ack = bbWriteByte((addr << 1) | 0x00, halfPeriodUs);
  bbStop(halfPeriodUs);
  bbSdaRelease();
  bbSclRelease();
  return ack;
}

bool bbWriteCommand(uint8_t addr, uint16_t cmd, uint32_t freqHz) {
  const uint16_t halfPeriodUs = bbHalfPeriodFor(freqHz);
  bbStart(halfPeriodUs);
  const bool ackAddr = bbWriteByte((addr << 1) | 0x00, halfPeriodUs);
  const bool ackHi = bbWriteByte((uint8_t)(cmd >> 8), halfPeriodUs);
  const bool ackLo = bbWriteByte((uint8_t)(cmd & 0xFF), halfPeriodUs);
  bbStop(halfPeriodUs);
  bbSdaRelease();
  bbSclRelease();
  const bool ok = ackAddr && ackHi && ackLo;
  Serial.printf("BB_SDP_CMD addr=0x%02X cmd=0x%04X addr_ack=%u hi_ack=%u lo_ack=%u ok=%u\n",
                addr, cmd, ackAddr ? 1 : 0, ackHi ? 1 : 0, ackLo ? 1 : 0, ok ? 1 : 0);
  return ok;
}

bool bbReadBytes(uint8_t addr, uint8_t *data, uint8_t len, uint32_t freqHz) {
  const uint16_t halfPeriodUs = bbHalfPeriodFor(freqHz);
  bbStart(halfPeriodUs);
  const bool ackAddr = bbWriteByte((addr << 1) | 0x01, halfPeriodUs);
  if (!ackAddr) {
    bbStop(halfPeriodUs);
    bbSdaRelease();
    bbSclRelease();
    Serial.printf("BB_I2C_READ_BYTES addr=0x%02X len=%u addr_ack=0 ok=0\n", addr, len);
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    data[i] = bbReadByte(i + 1 < len, halfPeriodUs);
  }
  bbStop(halfPeriodUs);
  bbSdaRelease();
  bbSclRelease();
  Serial.printf("BB_I2C_READ_BYTES addr=0x%02X len=%u addr_ack=1 ok=1\n", addr, len);
  return true;
}

bool bbWriteSingleByte(uint8_t addr, uint8_t value, uint32_t freqHz) {
  const uint16_t halfPeriodUs = bbHalfPeriodFor(freqHz);
  bbStart(halfPeriodUs);
  const bool ackAddr = bbWriteByte((addr << 1) | 0x00, halfPeriodUs);
  const bool ackValue = bbWriteByte(value, halfPeriodUs);
  bbStop(halfPeriodUs);
  bbSdaRelease();
  bbSclRelease();
  const bool ok = ackAddr && ackValue;
  Serial.printf("BB_I2C_WRITE1 addr=0x%02X value=0x%02X addr_ack=%u value_ack=%u ok=%u\n",
                addr, value, ackAddr ? 1 : 0, ackValue ? 1 : 0, ok ? 1 : 0);
  return ok;
}

bool bbWriteRegister(uint8_t addr, uint8_t reg, uint8_t value, uint32_t freqHz) {
  const uint16_t halfPeriodUs = bbHalfPeriodFor(freqHz);
  bbStart(halfPeriodUs);
  const bool ackAddr = bbWriteByte((addr << 1) | 0x00, halfPeriodUs);
  const bool ackReg = bbWriteByte(reg, halfPeriodUs);
  const bool ackValue = bbWriteByte(value, halfPeriodUs);
  bbStop(halfPeriodUs);
  bbSdaRelease();
  bbSclRelease();
  const bool ok = ackAddr && ackReg && ackValue;
  Serial.printf("BB_I2C_WRITE addr=0x%02X reg=0x%02X value=%u addr_ack=%u reg_ack=%u value_ack=%u ok=%u\n",
                addr, reg, value, ackAddr ? 1 : 0, ackReg ? 1 : 0, ackValue ? 1 : 0, ok ? 1 : 0);
  return ok;
}

void stopNawiew() {
  Wire.end();
  delay(10);
  const bool ok1 = bbWriteRegister(DIMMER_NAWIEW_ADDR, 0x10, 0, 100000);
  const bool ok2 = bbWriteRegister(DIMMER_NAWIEW_ADDR, 0x12, 0, 100000);
  Serial.printf("NAWIEW_STOP ok=%u\n", (ok1 && ok2) ? 1 : 0);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(25);
}

void startNawiewMin() {
  Wire.end();
  delay(10);
  const bool offCh2 = bbWriteRegister(DIMMER_NAWIEW_ADDR, 0x12, 0, 100000);
  const bool onCh1 = bbWriteRegister(DIMMER_NAWIEW_ADDR, 0x10, 55, 100000);
  Serial.printf("NAWIEW_START_MIN ch1_reg=0x10 pwm=55 off_ch2_ok=%u on_ch1_ok=%u ok=%u\n",
                offCh2 ? 1 : 0, onCh1 ? 1 : 0, (offCh2 && onCh1) ? 1 : 0);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(25);
}

void testDimmer50Ch1_70_10s() {
  Serial.println();
  Serial.println("DIM50_TEST_BEGIN addr=0x50 CH1=70% time=10s guard_CH2_off=1");
  Wire.end();
  delay(10);

  const bool offCh2Before = bbWriteRegister(DIMMER_WYWIEW_ADDR, 0x12, 0, 100000);
  const bool onCh1 = bbWriteRegister(DIMMER_WYWIEW_ADDR, 0x10, 70, 100000);
  Serial.printf("DIM50_TEST_ON off_ch2_ok=%u on_ch1_ok=%u ok=%u\n",
                offCh2Before ? 1 : 0, onCh1 ? 1 : 0, (offCh2Before && onCh1) ? 1 : 0);

  delay(10000);

  const bool offCh1 = bbWriteRegister(DIMMER_WYWIEW_ADDR, 0x10, 0, 100000);
  const bool offCh2After = bbWriteRegister(DIMMER_WYWIEW_ADDR, 0x12, 0, 100000);
  Serial.printf("DIM50_TEST_STOP off_ch1_ok=%u off_ch2_ok=%u ok=%u\n",
                offCh1 ? 1 : 0, offCh2After ? 1 : 0, (offCh1 && offCh2After) ? 1 : 0);
  Serial.println("DIM50_TEST_END");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(25);
}

void testDimmer50Ch2_70_10s() {
  Serial.println();
  Serial.println("DIM50_CH2_TEST_BEGIN addr=0x50 CH2=70% time=10s guard_CH1_off=1");
  Wire.end();
  delay(10);

  const bool offCh1Before = bbWriteRegister(DIMMER_WYWIEW_ADDR, 0x10, 0, 100000);
  const bool onCh2 = bbWriteRegister(DIMMER_WYWIEW_ADDR, 0x12, 70, 100000);
  Serial.printf("DIM50_CH2_TEST_ON off_ch1_ok=%u on_ch2_ok=%u ok=%u\n",
                offCh1Before ? 1 : 0, onCh2 ? 1 : 0, (offCh1Before && onCh2) ? 1 : 0);

  delay(10000);

  const bool offCh2 = bbWriteRegister(DIMMER_WYWIEW_ADDR, 0x12, 0, 100000);
  const bool offCh1After = bbWriteRegister(DIMMER_WYWIEW_ADDR, 0x10, 0, 100000);
  Serial.printf("DIM50_CH2_TEST_STOP off_ch2_ok=%u off_ch1_ok=%u ok=%u\n",
                offCh2 ? 1 : 0, offCh1After ? 1 : 0, (offCh2 && offCh1After) ? 1 : 0);
  Serial.println("DIM50_CH2_TEST_END");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(25);
}

uint8_t sensirionCrc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

bool writeCommand(uint16_t cmd) {
  Wire.beginTransmission(SDP810_ADDR);
  Wire.write((uint8_t)(cmd >> 8));
  Wire.write((uint8_t)(cmd & 0xFF));
  const uint8_t err = Wire.endTransmission();
  Serial.printf("SDP_CMD 0x%04X err=%u ok=%u\n", cmd, err, err == 0 ? 1 : 0);
  return err == 0;
}

bool isImportantAddr(uint8_t addr) {
  return addr == SDP810_ADDR ||
         addr == DIMMER_WYWIEW_ADDR ||
         addr == DIMMER_NAWIEW_ADDR ||
         addr == TCA9548A_ADDR;
}

void scanI2cAt(uint32_t freqHz, const char *label) {
  Wire.setClock(freqHz);
  uint8_t found = 0;
  Serial.println();
  Serial.printf("I2C_SCAN_BEGIN label=%s freq=%lu\n", label ? label : "", (unsigned long)freqHz);
  printBus("before_scan");
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    const uint8_t err = ping(addr);
    if (err == 0) {
      found++;
      Serial.printf("I2C_FOUND addr=0x%02X\n", addr);
    }
    if (isImportantAddr(addr) || err == 0) {
      Serial.printf("I2C_PROBE addr=0x%02X err=%u found=%u\n", addr, err, err == 0 ? 1 : 0);
    }
    delay(2);
  }
  Serial.printf("I2C_SCAN_END label=%s freq=%lu found=%u\n", label ? label : "", (unsigned long)freqHz, found);
}

void scanI2c() {
  scanI2cAt(I2C_FREQ, "default");
}

void scanI2cFreqs() {
  scanI2cAt(10000, "10k");
  scanI2cAt(50000, "50k");
  scanI2cAt(100000, "100k");
}

void bbScanI2cAt(uint32_t freqHz, const char *label) {
  uint8_t found = 0;
  Serial.println();
  Serial.printf("BB_I2C_SCAN_BEGIN label=%s freq=%lu\n", label ? label : "", (unsigned long)freqHz);
  printBus("before_bb_scan");
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    const bool ok = bbPing(addr, freqHz);
    if (ok) {
      found++;
      Serial.printf("BB_I2C_FOUND addr=0x%02X\n", addr);
    }
    if (isImportantAddr(addr) || ok) {
      Serial.printf("BB_I2C_PROBE addr=0x%02X found=%u\n", addr, ok ? 1 : 0);
    }
    delay(2);
  }
  Serial.printf("BB_I2C_SCAN_END label=%s freq=%lu found=%u\n", label ? label : "", (unsigned long)freqHz, found);
}

void bbScanI2cFreqs() {
  Wire.end();
  delay(10);
  bbScanI2cAt(10000, "10k");
  bbScanI2cAt(50000, "50k");
  bbScanI2cAt(100000, "100k");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(25);
}

void readSdp810() {
  Serial.println();
  Serial.println("SDP810_READ_BEGIN");
  printBus("before_sdp");

  const uint8_t probeErr = ping(SDP810_ADDR);
  Serial.printf("SDP810_PROBE addr=0x%02X err=%u found=%u\n", SDP810_ADDR, probeErr, probeErr == 0 ? 1 : 0);
  if (probeErr != 0) {
    Serial.println("SDP810_READ_SKIP not_found");
    return;
  }

  writeCommand(SDP810_CMD_STOP);
  delay(5);
  if (!writeCommand(SDP810_CMD_START_AVG)) {
    Serial.println("SDP810_READ_SKIP start_failed");
    return;
  }
  delay(60);

  const uint8_t got = Wire.requestFrom((int)SDP810_ADDR, 9);
  Serial.printf("SDP810_REQUEST got=%u\n", got);
  if (got != 9) {
    writeCommand(SDP810_CMD_STOP);
    Serial.println("SDP810_READ_FAIL short_read");
    return;
  }

  uint8_t data[9];
  for (uint8_t i = 0; i < 9; i++) data[i] = Wire.read();
  writeCommand(SDP810_CMD_STOP);

  const bool crcPressure = sensirionCrc8(&data[0], 2) == data[2];
  const bool crcTemp = sensirionCrc8(&data[3], 2) == data[5];
  const bool crcScale = sensirionCrc8(&data[6], 2) == data[8];
  const int16_t rawPressure = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  const int16_t rawTemp = (int16_t)(((uint16_t)data[3] << 8) | data[4]);
  const uint16_t scale = (uint16_t)(((uint16_t)data[6] << 8) | data[7]);
  const float pa = scale ? ((float)rawPressure / (float)scale) : NAN;
  const float tempC = (float)rawTemp / 200.0f;

  Serial.printf("SDP810_RAW pressure=%d temp=%d scale=%u crc_pressure=%u crc_temp=%u crc_scale=%u\n",
                rawPressure,
                rawTemp,
                scale,
                crcPressure ? 1 : 0,
                crcTemp ? 1 : 0,
                crcScale ? 1 : 0);
  Serial.printf("SDP810_VALUE pressure_pa=%.2f temp_c=%.2f\n", pa, tempC);
  Serial.println("SDP810_READ_END");
}

void bbReadSdp810At(uint32_t freqHz) {
  Serial.println();
  Serial.printf("BB_SDP810_READ_BEGIN freq=%lu\n", (unsigned long)freqHz);
  Wire.end();
  delay(10);
  printBus("before_bb_sdp");

  const bool found = bbPing(SDP810_ADDR, freqHz);
  Serial.printf("BB_SDP810_PROBE addr=0x%02X found=%u\n", SDP810_ADDR, found ? 1 : 0);
  if (!found) {
    Serial.println("BB_SDP810_READ_SKIP not_found");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    Wire.setTimeOut(25);
    return;
  }

  bbWriteCommand(SDP810_ADDR, SDP810_CMD_STOP, freqHz);
  delay(5);
  if (!bbWriteCommand(SDP810_ADDR, SDP810_CMD_START_AVG, freqHz)) {
    Serial.println("BB_SDP810_READ_SKIP start_failed");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    Wire.setTimeOut(25);
    return;
  }
  delay(60);

  uint8_t data[9] = {};
  const bool readOk = bbReadBytes(SDP810_ADDR, data, sizeof(data), freqHz);
  bbWriteCommand(SDP810_ADDR, SDP810_CMD_STOP, freqHz);
  if (!readOk) {
    Serial.println("BB_SDP810_READ_FAIL read_bytes");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    Wire.setTimeOut(25);
    return;
  }

  const bool crcPressure = sensirionCrc8(&data[0], 2) == data[2];
  const bool crcTemp = sensirionCrc8(&data[3], 2) == data[5];
  const bool crcScale = sensirionCrc8(&data[6], 2) == data[8];
  const int16_t rawPressure = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  const int16_t rawTemp = (int16_t)(((uint16_t)data[3] << 8) | data[4]);
  const uint16_t scale = (uint16_t)(((uint16_t)data[6] << 8) | data[7]);
  const float pa = scale ? ((float)rawPressure / (float)scale) : NAN;
  const float tempC = (float)rawTemp / 200.0f;

  Serial.printf("BB_SDP810_BYTES");
  for (uint8_t i = 0; i < sizeof(data); i++) Serial.printf(" %02X", data[i]);
  Serial.println();
  Serial.printf("BB_SDP810_RAW pressure=%d temp=%d scale=%u crc_pressure=%u crc_temp=%u crc_scale=%u\n",
                rawPressure,
                rawTemp,
                scale,
                crcPressure ? 1 : 0,
                crcTemp ? 1 : 0,
                crcScale ? 1 : 0);
  Serial.printf("BB_SDP810_VALUE pressure_pa=%.2f temp_c=%.2f\n", pa, tempC);
  Serial.println("BB_SDP810_READ_END");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(25);
}

void bbReadSdp810() {
  bbReadSdp810At(100000);
}

bool bbMuxSelect(uint8_t channel, uint32_t freqHz = 100000) {
  if (channel > 7) {
    Serial.println("BB_MUX_SELECT_FAIL bad_channel");
    return false;
  }
  const uint8_t mask = 1 << channel;
  const bool ok = bbWriteRegister(TCA9548A_ADDR, 0x00, mask, freqHz);
  Serial.printf("BB_MUX_SELECT channel=%u mask=0x%02X ok=%u\n", channel, mask, ok ? 1 : 0);
  return ok;
}

bool bbMuxOff(uint32_t freqHz = 100000) {
  const bool ok = bbWriteRegister(TCA9548A_ADDR, 0x00, 0x00, freqHz);
  Serial.printf("BB_MUX_OFF ok=%u\n", ok ? 1 : 0);
  return ok;
}

void bbMuxScanChannels(uint32_t freqHz = 100000) {
  Serial.println();
  Serial.printf("BB_MUX_SCAN_BEGIN mux=0x%02X sensor=0x%02X freq=%lu\n",
                TCA9548A_ADDR, SDP810_ADDR, (unsigned long)freqHz);
  Wire.end();
  delay(10);
  printBus("before_bb_muxscan");

  const bool muxFound = bbPing(TCA9548A_ADDR, freqHz);
  Serial.printf("BB_MUX_PROBE addr=0x%02X found=%u\n", TCA9548A_ADDR, muxFound ? 1 : 0);
  if (!muxFound) {
    Serial.println("BB_MUX_SCAN_SKIP mux_not_found");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    Wire.setTimeOut(25);
    return;
  }

  for (uint8_t ch = 0; ch < 8; ch++) {
    const bool selOk = bbMuxSelect(ch, freqHz);
    delay(2);
    const bool sdpOk = selOk && bbPing(SDP810_ADDR, freqHz);
    Serial.printf("BB_MUX_CH channel=%u select_ok=%u sdp810_0x25=%u\n",
                  ch, selOk ? 1 : 0, sdpOk ? 1 : 0);
  }

  bbMuxOff(freqHz);
  Serial.println("BB_MUX_SCAN_END");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(25);
}

void bbMuxReadChannel(uint8_t channel, uint32_t freqHz = 100000) {
  Serial.println();
  Serial.printf("BB_MUX_READ_BEGIN channel=%u mux=0x%02X sensor=0x%02X freq=%lu\n",
                channel, TCA9548A_ADDR, SDP810_ADDR, (unsigned long)freqHz);
  Wire.end();
  delay(10);

  if (!bbMuxSelect(channel, freqHz)) {
    Serial.println("BB_MUX_READ_SKIP select_failed");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    Wire.setTimeOut(25);
    return;
  }

  bbReadSdp810At(freqHz);
  Wire.end();
  delay(10);
  bbMuxOff(freqHz);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(25);
  Serial.println("BB_MUX_READ_END");
}

void scanDs18b20() {
  uint8_t addr[8];
  uint8_t found = 0;
  Serial.println();
  Serial.printf("DS_SCAN_BEGIN pin=%d\n", DS18B20_PIN);
  dsBus.reset_search();

  while (dsBus.search(addr)) {
    found++;
    const bool crcOk = OneWire::crc8(addr, 7) == addr[7];
    Serial.print("DS_FOUND rom=");
    for (uint8_t i = 0; i < 8; i++) {
      if (i) Serial.print(' ');
      if (addr[i] < 0x10) Serial.print('0');
      Serial.print(addr[i], HEX);
    }
    Serial.printf(" family=0x%02X crc=%u", addr[0], crcOk ? 1 : 0);
    if (addr[0] == 0x28) Serial.print(" type=DS18B20");
    else if (addr[0] == 0x10) Serial.print(" type=DS18S20");
    else if (addr[0] == 0x22) Serial.print(" type=DS1822");
    else Serial.print(" type=UNKNOWN");
    Serial.println();
  }

  Serial.printf("DS_SCAN_END found=%u\n", found);
}

void bbReadSht40(uint32_t freqHz = 100000) {
  Serial.println();
  Serial.printf("BB_SHT40_READ_BEGIN addr=0x%02X freq=%lu\n", SHT40_ADDR, (unsigned long)freqHz);
  Wire.end();
  delay(10);
  printBus("before_bb_sht40");

  const bool found = bbPing(SHT40_ADDR, freqHz);
  Serial.printf("BB_SHT40_PROBE addr=0x%02X found=%u\n", SHT40_ADDR, found ? 1 : 0);
  if (!found) {
    Serial.println("BB_SHT40_READ_SKIP not_found");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    Wire.setTimeOut(25);
    return;
  }

  // SHT4x high precision measurement, no heater.
  if (!bbWriteSingleByte(SHT40_ADDR, 0xFD, freqHz)) {
    Serial.println("BB_SHT40_READ_SKIP measure_cmd_failed");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    Wire.setTimeOut(25);
    return;
  }
  delay(15);

  uint8_t data[6] = {};
  const bool readOk = bbReadBytes(SHT40_ADDR, data, sizeof(data), freqHz);
  if (!readOk) {
    Serial.println("BB_SHT40_READ_FAIL read_bytes");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    Wire.setTimeOut(25);
    return;
  }

  const bool crcTemp = sensirionCrc8(&data[0], 2) == data[2];
  const bool crcRh = sensirionCrc8(&data[3], 2) == data[5];
  const uint16_t rawTemp = ((uint16_t)data[0] << 8) | data[1];
  const uint16_t rawRh = ((uint16_t)data[3] << 8) | data[4];
  const float tempC = -45.0f + 175.0f * (float)rawTemp / 65535.0f;
  float rh = -6.0f + 125.0f * (float)rawRh / 65535.0f;
  if (rh < 0.0f) rh = 0.0f;
  if (rh > 100.0f) rh = 100.0f;

  Serial.print("BB_SHT40_BYTES");
  for (uint8_t i = 0; i < sizeof(data); i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
  Serial.printf("BB_SHT40_RAW temp=%u rh=%u crc_temp=%u crc_rh=%u\n",
                rawTemp, rawRh, crcTemp ? 1 : 0, crcRh ? 1 : 0);
  Serial.printf("BB_SHT40_VALUE temp_c=%.2f rh_pct=%.2f\n", tempC, rh);
  Serial.println("BB_SHT40_READ_END");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(25);
}

void runAll() {
  scanI2c();
  readSdp810();
}

void printHelp() {
  Serial.println();
  Serial.println("RECUP_SDP810_MIN_TEST 2026-07-12");
  Serial.printf("I2C pins: SDA=%d SCL=%d freq=%lu SDP810=0x%02X DS18B20_PIN=%d\n",
                I2C_SDA,
                I2C_SCL,
                (unsigned long)I2C_FREQ,
                SDP810_ADDR,
                DS18B20_PIN);
  Serial.println("Commands:");
  Serial.println("  scan  - scan 0x08..0x77");
  Serial.println("  scanfreq - scan 0x08..0x77 at 10/50/100 kHz, explicit 0x25/0x50/0x51/0x70 probes");
  Serial.println("  bbscanfreq - bit-bang scan at 10/50/100 kHz, independent of Wire driver");
  Serial.println("  nawiew5 - bit-bang start NAWIEW 0x51 CH1/reg0x10 at 55%, CH2 forced off");
  Serial.println("  stopn   - bit-bang stop NAWIEW 0x51 CH1/CH2");
  Serial.println("  dim50test - bit-bang 0x50: CH2 off, CH1 70% for 10s, both off");
  Serial.println("  dim50ch2test - bit-bang 0x50: CH1 off, CH2 70% for 10s, both off");
  Serial.println("  read  - probe/read SDP810 at 0x25");
  Serial.println("  bbread - bit-bang read SDP810 at 0x25");
  Serial.println("  bbmuxscan - bit-bang scan TCA9548A 0x70 channels 0..7 for SDP810 0x25");
  Serial.println("  bbmux <0..7> - select TCA channel and bit-bang read SDP810");
  Serial.println("  dsscan - scan OneWire DS18B20 ROM addresses on GPIO10");
  Serial.println("  shtread - bit-bang read SHT40 at 0x44");
  Serial.println("  bus   - print raw SDA/SCL");
  Serial.println("  all   - scan and read");
  Serial.println("  help  - print this help");
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  if (line == "scan") scanI2c();
  else if (line == "scanfreq") scanI2cFreqs();
  else if (line == "bbscanfreq") bbScanI2cFreqs();
  else if (line == "nawiew5") startNawiewMin();
  else if (line == "stopn") stopNawiew();
  else if (line == "dim50test") testDimmer50Ch1_70_10s();
  else if (line == "dim50ch2test") testDimmer50Ch2_70_10s();
  else if (line == "read") readSdp810();
  else if (line == "bbread") bbReadSdp810();
  else if (line == "bbmuxscan") bbMuxScanChannels();
  else if (line.startsWith("bbmux ")) bbMuxReadChannel((uint8_t)constrain(line.substring(6).toInt(), 0, 7));
  else if (line == "dsscan") scanDs18b20();
  else if (line == "shtread") bbReadSht40();
  else if (line == "bus") printBus("manual");
  else if (line == "all") runAll();
  else if (line == "help" || line == "h") printHelp();
  else Serial.printf("ERR unknown command: %s\n", line.c_str());
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
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(25);
  printHelp();
  runAll();
}

void loop() {
  serviceSerial();
  if (millis() - lastAutoMs > 5000) {
    lastAutoMs = millis();
    runAll();
  }
}
