/*
  DongleESP_RX
  Odbiornik telemetrii ESP-NOW z CYD Hewalex.
  COM USB pokazuje pelny podglad w monitorze portu.
*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static constexpr uint8_t ESPNOW_CHANNEL = 7;
static constexpr uint8_t ESPNOW_CHANNEL_MIN = 1;
static constexpr uint8_t ESPNOW_CHANNEL_MAX = 13;
static constexpr uint32_t ESPNOW_SCAN_DWELL_MS = 1400;
static constexpr uint32_t TELEMETRY_MAGIC = 0x48575843UL;  // "HWXC"
static constexpr uint8_t TELEMETRY_VERSION = 2;
static const char *BUILD_ID = "DongleESP_RX 2026-07-21 ALARMD5";
static const char *WIFI_SSID = "YOUR_WIFI_SSID";
static const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static constexpr uint32_t RADIO_RETRY_MS = 10000;
static uint8_t CYD_ESPNOW_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Mikrus: OLED SSD1306 128x64, okno robocze 72x40.
static constexpr int OLED_SDA = 5;
static constexpr int OLED_SCL = 6;
static constexpr int X0 = 28;
static constexpr int Y0 = 24;
static constexpr int VIEW_W = 72;
static constexpr int VIEW_H = 40;
static constexpr int BLUE_LED_PIN = 8;     // ESP32-C3 SuperMini OLED: niebieska dioda sygnalowa.
static constexpr int RADIO_BUTTON_PIN = 9; // Przycisk BOOT, aktywny stan niski.
static constexpr bool BLUE_LED_ACTIVE_LOW = true;
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 60;
static constexpr uint8_t DONGLE_ECO_START_INDEX = 0; // 2 dBm
static constexpr uint8_t DONGLE_ECO_WINDOW_PACKETS = 30;
static constexpr uint8_t DONGLE_ECO_STABLE_WINDOWS = 3;
static constexpr int8_t DONGLE_ECO_LOWER_RSSI_DBM = -58;
static constexpr int8_t DONGLE_ECO_RAISE_RSSI_DBM = -78;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

struct __attribute__((packed)) HewalexTelemetryPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t packetType;
  uint16_t packetSize;
  uint32_t seq;
  uint32_t ms;
  uint8_t yy, mo, dd, hh, mi, ss;
  uint8_t demoMode;
  uint8_t rsValid;
  uint8_t rsOnline;
  uint8_t ntpOk;
  uint8_t pumpMode;
  uint8_t pumpC;
  uint8_t heaterE;
  uint8_t compressor;
  uint8_t sdOK;
  uint8_t logOK;
  uint8_t calLogOK;
  uint8_t dsCount;
  uint8_t pumpPwmPercent;
  uint8_t backlightDuty;
  uint8_t espNowTxPowerPercent;
  uint8_t wifiStatus;
  int8_t wifiRssi;
  uint16_t status196Raw;
  uint16_t status196Core;
  uint16_t lightRaw;
  uint16_t flowPulsesLast;
  uint16_t flowRejectedLast;
  uint32_t rsAgeMs;
  uint32_t rxBytes;
  uint32_t goodFrames;
  uint32_t tempFrames;
  uint32_t badFrames;
  uint32_t espSent;
  uint32_t espOk;
  uint32_t espFail;
  float t1, t2, t3, t4;
  float ds0, ds1, ds2;
  float solarCollectorTemp;
  float solarTankInTemp;
  float solarReturnTemp;
  float flowHz;
  float flowHzRaw;
  float pumpTargetFlowLMin;
  float deltaT;
  float powerKW;
  float pressureBar;
  uint8_t alarmActive;
  uint8_t alarmInReview;
  uint8_t alarmKind;
  uint8_t alarmSensorTarget;
  char alarmReason[40];
};

struct __attribute__((packed)) EspNowCommandPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t packetType;
  uint16_t packetSize;
  uint32_t cmdSeq;
  char command[96];
};

struct __attribute__((packed)) EspNowReplyPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t packetType;
  uint16_t packetSize;
  uint32_t cmdSeq;
  uint8_t ok;
  char message[160];
};

uint32_t packets = 0;
uint32_t badPackets = 0;
uint32_t lastPacketMs = 0;
int8_t lastRssi = 0;
bool haveRssi = false;
uint8_t lastTxPowerPercent = 0;
uint8_t dongleTxPowerIndex = DONGLE_ECO_START_INDEX;
uint8_t dongleEcoWindowPackets = 0;
uint8_t dongleEcoStableWindows = 0;
int16_t dongleEcoRssiSum = 0;
uint32_t dongleCmdSent = 0;
uint32_t dongleCmdOk = 0;
uint32_t dongleCmdFail = 0;
bool cydAlarmActive = false;
uint8_t cydAlarmKind = 0;
uint8_t cydAlarmSensorTarget = 0;
char cydAlarmReason[40] = "";
bool startupAttentionPrinted = false;
bool startupAlarmQuerySent = false;
uint32_t startupFirstLinkMs = 0;
uint32_t bootMs = 0;
char lastCydAlarmReply[160] = "";
enum DisplayState : uint8_t {
  DISPLAY_STATE_OFF = 0,
  DISPLAY_STATE_ZERO = 1,
  DISPLAY_STATE_ON = 2
};

DisplayState displayState = DISPLAY_STATE_OFF;
uint32_t lastDisplayMs = 0;
uint32_t commandSeq = 0;
bool espNowUserEnabled = true;
bool espNowRunning = false;
bool displayOverrideEnabled = false;
DisplayState displayOverrideState = DISPLAY_STATE_ZERO;
bool buttonReadingLast = HIGH;
bool buttonStableState = HIGH;
uint32_t buttonChangedMs = 0;
bool wifiLinkedForEspNow = false;
uint8_t espNowRadioChannel = ESPNOW_CHANNEL;

void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void onEspNowSent(const esp_now_send_info_t *info, esp_now_send_status_t status);

struct DonglePowerLevel {
  wifi_power_t power;
  float dbm;
  const char *label;
};

static const DonglePowerLevel DONGLE_POWER_LEVELS[] = {
  {WIFI_POWER_2dBm, 2.0f, "2dBm"},
  {WIFI_POWER_5dBm, 5.0f, "5dBm"},
  {WIFI_POWER_8_5dBm, 8.5f, "8.5dBm"},
  {WIFI_POWER_11dBm, 11.0f, "11dBm"},
  {WIFI_POWER_15dBm, 15.0f, "15dBm"},
  {WIFI_POWER_19_5dBm, 19.5f, "19.5dBm"},
};

uint8_t donglePowerLevelCount() {
  return sizeof(DONGLE_POWER_LEVELS) / sizeof(DONGLE_POWER_LEVELS[0]);
}

uint8_t dongleTxPowerPercent() {
  const float dbm = DONGLE_POWER_LEVELS[dongleTxPowerIndex].dbm;
  return (uint8_t)constrain((int)roundf(dbm * 100.0f / 19.5f), 0, 100);
}

const char *dongleTxPowerLabel() {
  return DONGLE_POWER_LEVELS[dongleTxPowerIndex].label;
}

void setDongleTxPowerIndex(uint8_t index, bool announce) {
  if (index >= donglePowerLevelCount()) index = donglePowerLevelCount() - 1;
  dongleTxPowerIndex = index;
  WiFi.setTxPower(DONGLE_POWER_LEVELS[dongleTxPowerIndex].power);
  if (announce) {
    Serial.printf("DONGLE ECO tx=%s (%u%%)\n", dongleTxPowerLabel(), dongleTxPowerPercent());
  }
}

void resetDongleEcoWindow() {
  dongleEcoWindowPackets = 0;
  dongleEcoRssiSum = 0;
}

void raiseDongleTxPower(const char *reason) {
  dongleEcoStableWindows = 0;
  if (dongleTxPowerIndex + 1 >= donglePowerLevelCount()) return;
  setDongleTxPowerIndex(dongleTxPowerIndex + 1, true);
  if (reason && reason[0]) Serial.printf("DONGLE ECO raise reason=%s\n", reason);
}

void serviceDongleEcoRssi(int8_t rssi) {
  dongleEcoRssiSum += rssi;
  if (dongleEcoWindowPackets < 255) dongleEcoWindowPackets++;
  if (dongleEcoWindowPackets < DONGLE_ECO_WINDOW_PACKETS) return;

  const int8_t avgRssi = (int8_t)(dongleEcoRssiSum / (int16_t)dongleEcoWindowPackets);
  if (avgRssi <= DONGLE_ECO_RAISE_RSSI_DBM) {
    raiseDongleTxPower("weak_rssi");
  } else if (avgRssi >= DONGLE_ECO_LOWER_RSSI_DBM) {
    if (dongleEcoStableWindows < 255) dongleEcoStableWindows++;
    if (dongleEcoStableWindows >= DONGLE_ECO_STABLE_WINDOWS && dongleTxPowerIndex > 0) {
      setDongleTxPowerIndex(dongleTxPowerIndex - 1, true);
      dongleEcoStableWindows = 0;
    }
  } else {
    dongleEcoStableWindows = 0;
  }
  resetDongleEcoWindow();
}

const char *pumpModeName(uint8_t mode) {
  switch (mode) {
    case 0: return "AUTO";
    case 1: return "MANUAL";
    case 2: return "STOP";
    default: return "?";
  }
}

const char *displayStateName(DisplayState state) {
  switch (state) {
    case DISPLAY_STATE_OFF: return "OFF";
    case DISPLAY_STATE_ZERO: return "0";
    case DISPLAY_STATE_ON: return "ON";
    default: return "?";
  }
}

void printFloatOrDash(float v, uint8_t prec = 1) {
  if (isnan(v)) {
    Serial.print("--");
    return;
  }
  Serial.print(v, prec);
}

void printMac(const uint8_t *mac) {
  for (uint8_t i = 0; i < 6; i++) {
    if (mac[i] < 0x10) Serial.print('0');
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(':');
  }
}

void printDongleBanner(bool radioOn, bool linkActive) {
  Serial.println();
  Serial.println("========================================");
  Serial.println("              DONGLE ESP");
  Serial.println("========================================");
  if (!radioOn) {
    Serial.println("       OOOOO   FFFFF  FFFFF");
    Serial.println("       O   O   F      F");
    Serial.println("       O   O   FFFF   FFFF");
    Serial.println("       O   O   F      F");
    Serial.println("       OOOOO   F      F");
  } else if (linkActive) {
    Serial.println("       OOOOO   N   N");
    Serial.println("       O   O   NN  N");
    Serial.println("       O   O   N N N");
    Serial.println("       O   O   N  NN");
    Serial.println("       OOOOO   N   N");
  } else {
    Serial.println("       OOOOO    0000");
    Serial.println("       O   O   0    0");
    Serial.println("       O   O   0    0");
    Serial.println("       O   O   0    0");
    Serial.println("       OOOOO    0000");
  }
  Serial.println("========================================");
  Serial.println();
}

const char *alarmShortText() {
  if (cydAlarmKind == 2) {
    switch (cydAlarmSensorTarget) {
      case 0: return "RS485 OFF";
      case 1: return "DS SOL";
      case 2: return "DS WEJ";
      case 3: return "DS WYJ";
    }
  }
  if (cydAlarmKind == 1) {
    return strstr(cydAlarmReason, "ZBIORNIK") ? "TEMP ZBIOR" : "TEMP SOL";
  }
  if (cydAlarmKind == 3) return "CISNIENIE";
  if (cydAlarmKind == 4) return "BRAK PRZEP";
  if (cydAlarmKind == 5) return "PRZEPLYW";
  return "ALARM CYD";
}

void updateCydAlarm(const HewalexTelemetryPacket &p) {
  const bool active = p.alarmActive != 0;
  const bool changed = active != cydAlarmActive ||
                       p.alarmKind != cydAlarmKind ||
                       p.alarmSensorTarget != cydAlarmSensorTarget ||
                       strncmp(p.alarmReason, cydAlarmReason, sizeof(cydAlarmReason) - 1) != 0;
  cydAlarmActive = active;
  cydAlarmKind = p.alarmKind;
  cydAlarmSensorTarget = p.alarmSensorTarget;
  snprintf(cydAlarmReason, sizeof(cydAlarmReason), "%s", active ? p.alarmReason : "");
  if (changed) {
    Serial.printf("ALARM=%s kind=%u sensor=%u reason=\"%s\"\n",
                  active ? "ON" : "OFF",
                  cydAlarmKind,
                  cydAlarmSensorTarget,
                  cydAlarmReason);
  }
}

void drawDongleDisplay(bool radioOn, bool linkActive) {
  u8g2.clearBuffer();

  if (linkActive && cydAlarmActive) {
    const bool blinkOn = ((millis() / 300UL) % 2UL) == 0;
    if (blinkOn) {
      u8g2.drawBox(X0, Y0, VIEW_W, VIEW_H);
      u8g2.setDrawColor(0);
    } else {
      u8g2.drawFrame(X0, Y0, VIEW_W, VIEW_H);
      u8g2.setDrawColor(1);
    }
    u8g2.setFont(u8g2_font_7x14B_tf);
    const char *alarmText = "ALARM";
    u8g2.drawStr(X0 + (VIEW_W - u8g2.getStrWidth(alarmText)) / 2, Y0 + 15, alarmText);
    u8g2.setFont(u8g2_font_5x8_tf);
    const char *reason = alarmShortText();
    u8g2.drawStr(X0 + max(2, (VIEW_W - u8g2.getStrWidth(reason)) / 2), Y0 + 29, reason);
    u8g2.setDrawColor(1);
    u8g2.sendBuffer();
    return;
  }

  u8g2.drawFrame(X0, Y0, VIEW_W, VIEW_H);

  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.drawStr(X0 + 3, Y0 + 11, "DongleESP");
  char diagBuf[8];
  char chBuf[8];
  u8g2.setFont(u8g2_font_5x8_tf);
  snprintf(diagBuf, sizeof(diagBuf), "W:%u", wifiLinkedForEspNow ? 1 : 0);
  u8g2.drawStr(X0 + 3, Y0 + 19, diagBuf);
  snprintf(chBuf, sizeof(chBuf), "CH%u", espNowRadioChannel);
  u8g2.setFont(u8g2_font_7x14B_tf);
  int chW = u8g2.getStrWidth(chBuf);
  u8g2.drawStr(X0 + VIEW_W - chW - 3, Y0 + 24, chBuf);
  u8g2.setFont(u8g2_font_logisoso16_tf);
  const char *stateText = !radioOn ? "OFF" : (linkActive ? "ON" : "0");
  const int stateX = !radioOn ? 10 : (linkActive ? 15 : 28);
  u8g2.drawStr(X0 + stateX, Y0 + 37, stateText);
  if (radioOn) {
    char txBuf[8];
    snprintf(txBuf, sizeof(txBuf), "%u%%", dongleTxPowerPercent());
    u8g2.setFont(u8g2_font_7x14B_tf);
    int w = u8g2.getStrWidth(txBuf);
    u8g2.drawStr(X0 + VIEW_W - w - 3, Y0 + VIEW_H - 2, txBuf);
  }
  u8g2.sendBuffer();
}

DisplayState currentDisplayState(bool radioOn, bool linkActive) {
  if (displayOverrideEnabled) return displayOverrideState;
  if (!radioOn) return DISPLAY_STATE_OFF;
  if (!linkActive) return DISPLAY_STATE_ZERO;
  return DISPLAY_STATE_ON;
}

void setBlueLed(bool on) {
  digitalWrite(BLUE_LED_PIN, BLUE_LED_ACTIVE_LOW ? !on : on);
}

void updateBlueLed(bool radioOn, bool linkActive) {
  if (!radioOn) {
    setBlueLed(false);
    return;
  }
  if (linkActive) {
    setBlueLed(true);
    return;
  }
  const bool blinkOn = ((millis() / 500UL) % 2UL) == 0;
  setBlueLed(blinkOn);
}

bool startEspNowRadioOnChannel(uint8_t channel) {
  if (espNowRunning) return true;
  channel = constrain(channel, ESPNOW_CHANNEL_MIN, ESPNOW_CHANNEL_MAX);

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, false);
  wifiLinkedForEspNow = false;
  espNowRadioChannel = channel;
  if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.printf("ESP-NOW channel %u set ERROR\n", channel);
    WiFi.mode(WIFI_OFF);
    espNowRadioChannel = 0;
    espNowRunning = false;
    return false;
  }
  dongleTxPowerIndex = DONGLE_ECO_START_INDEX;
  dongleEcoStableWindows = 0;
  resetDongleEcoWindow();
  setDongleTxPowerIndex(dongleTxPowerIndex, false);

  Serial.println();
  Serial.println("DongleESP RX ESP-NOW");
  Serial.print("MAC STA: ");
  Serial.println(WiFi.macAddress());
  Serial.print("WiFi: STA no-connect channel=");
  Serial.println(espNowRadioChannel);
  Serial.print("TX power: ");
  Serial.println(dongleTxPowerLabel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init ERROR");
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_OFF);
    espNowRunning = false;
    return false;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, CYD_ESPNOW_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (!esp_now_is_peer_exist(CYD_ESPNOW_MAC)) {
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("ESP-NOW CYD peer add ERROR");
      esp_now_deinit();
      WiFi.mode(WIFI_OFF);
      espNowRunning = false;
      return false;
    }
  }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  espNowRunning = true;
  lastPacketMs = 0;
  displayState = DISPLAY_STATE_ZERO;
  lastDisplayMs = millis();
  drawDongleDisplay(true, false);
  Serial.println("Czekam na pakiety z CYD...");
  printDongleBanner(true, false);
  return true;
}

void stopEspNowRadio() {
  if (espNowRunning) {
    esp_now_deinit();
  }
  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_OFF);
  espNowRunning = false;
  lastPacketMs = 0;
  wifiLinkedForEspNow = false;
  espNowRadioChannel = 0;
  displayState = DISPLAY_STATE_OFF;
  lastDisplayMs = millis();
  setBlueLed(false);
  drawDongleDisplay(false, false);
  Serial.println("ESP-NOW/WiFi wylaczone przyciskiem.");
}

bool waitForCydPacket(uint32_t dwellMs, uint32_t packetCountBefore) {
  const uint32_t startedMs = millis();
  while (millis() - startedMs < dwellMs) {
    if (packets > packetCountBefore) return true;
    delay(20);
  }
  return false;
}

bool scanCydChannel() {
  Serial.println("ESP-NOW channel scan: CYD telemetry");
  const uint8_t preferred = espNowRadioChannel >= ESPNOW_CHANNEL_MIN && espNowRadioChannel <= ESPNOW_CHANNEL_MAX
                              ? espNowRadioChannel
                              : ESPNOW_CHANNEL;

  for (uint8_t pass = 0; pass < 2; pass++) {
    for (uint8_t ch = ESPNOW_CHANNEL_MIN; ch <= ESPNOW_CHANNEL_MAX; ch++) {
      if (pass == 0 && ch != preferred) continue;
      if (pass == 1 && ch == preferred) continue;

      stopEspNowRadio();
      Serial.printf("SCAN ch=%u\n", ch);
      const uint32_t beforePackets = packets;
      if (!startEspNowRadioOnChannel(ch)) continue;
      if (waitForCydPacket(ESPNOW_SCAN_DWELL_MS, beforePackets)) {
        Serial.printf("SCAN FOUND ch=%u packets=%lu\n", ch, (unsigned long)packets);
        return true;
      }
    }
  }

  stopEspNowRadio();
  Serial.println("SCAN failed: no CYD telemetry");
  return false;
}

bool startEspNowRadio() {
  if (espNowRunning) return true;
  return scanCydChannel();
}

void toggleEspNowRadio() {
  espNowUserEnabled = !espNowUserEnabled;
  if (espNowUserEnabled) {
    Serial.println("ESP-NOW/WiFi wlaczanie przyciskiem...");
    if (!startEspNowRadio()) {
      espNowUserEnabled = false;
      drawDongleDisplay(false, false);
    }
  } else {
    stopEspNowRadio();
  }
}

void handleRadioButton() {
  bool reading = digitalRead(RADIO_BUTTON_PIN);
  if (reading != buttonReadingLast) {
    buttonReadingLast = reading;
    buttonChangedMs = millis();
  }
  if (millis() - buttonChangedMs < BUTTON_DEBOUNCE_MS) return;

  if (reading != buttonStableState) {
    buttonStableState = reading;
    if (buttonStableState == LOW) {
      toggleEspNowRadio();
    }
  }
}

void printLocalStatus() {
  uint32_t age = lastPacketMs ? millis() - lastPacketMs : 999999;
  bool radioOn = espNowUserEnabled && espNowRunning;
  bool linkActive = radioOn && lastPacketMs && age < 3000;
  DisplayState expected = currentDisplayState(radioOn, linkActive);
  Serial.printf("STATUS ver=%s radio=%s user=%u running=%u link=%u packets=%lu bad=%lu age=%lums display=%s override=%u cyd_tx=%u%% dongle_tx=%u%% rssi=%d cmd=%lu/%lu/%lu\n",
                BUILD_ID,
                radioOn ? "ON" : "OFF",
                espNowUserEnabled ? 1 : 0,
                espNowRunning ? 1 : 0,
                linkActive ? 1 : 0,
                (unsigned long)packets,
                (unsigned long)badPackets,
                (unsigned long)age,
                displayStateName(expected),
                displayOverrideEnabled ? 1 : 0,
                lastTxPowerPercent,
                dongleTxPowerPercent(),
                haveRssi ? lastRssi : 0,
                (unsigned long)dongleCmdSent,
                (unsigned long)dongleCmdOk,
                (unsigned long)dongleCmdFail);
  Serial.printf("STATUS wifiLinked=%u ch=%u wifi=%d rssi=%d\n",
                wifiLinkedForEspNow ? 1 : 0,
                espNowRadioChannel,
                (int)WiFi.status(),
                wifiLinkedForEspNow ? WiFi.RSSI() : 0);
}

void printHelp() {
  Serial.println();
  Serial.println("==================================================");
  Serial.println(" DONGLE ESP-NOW -> CYD SOLAR");
  Serial.printf(" FW: %s\n", BUILD_ID);
  Serial.println(" Rola: zdalny podglad i komendy serwisowe CYD.");
  Serial.println("--------------------------------------------------");
  Serial.println(" Lokalnie:");
  Serial.println("   status          - status dongla i linku");
  Serial.println("   scan            - skan kanalow ESP-NOW CYD");
  Serial.println("   ch 7            - wymus kanal 7");
  Serial.println("   disp auto/off/0/on");
  Serial.println("   ver             - wersja dongla");
  Serial.println("   help lub ?      - ta lista");
  Serial.println("--------------------------------------------------");
  Serial.println(" CYD przez ESP-NOW:");
  Serial.println("   cyd ver         - wersja CYD");
  Serial.println("   cyd status      - status pracy");
  Serial.println("   cyd alarm       - aktywny/ostatni alarm");
  Serial.println("   cyd alarm ack   - kasuj alarm w CYD i donglu");
  Serial.println("   cyd log today   - podsumowanie dnia");
  Serial.println("   cyd log last    - ostatni wpis HEW*.CSV");
  Serial.println("   cyd log alarm   - ostatni wpis ALARM.CSV");
  Serial.println("   cyd log energy  - ostatni wpis ENERGY.CSV");
  Serial.println("   cyd project     - krotki opis/start z CYD");
  Serial.println("   cyd readme N    - strona PROJECT.TXT z SD");
  Serial.println("--------------------------------------------------");
  Serial.println(" Surowe komendy bez prefiksu trafiaja do CYD.");
  Serial.println(" Upload firmware tylko po potwierdzeniu.");
  Serial.println("==================================================");
  Serial.println();
}

void printProjectManifest() {
  Serial.println();
  Serial.println("CODEX_PROJECT_MANIFEST_BEGIN");
  Serial.println("project=CYD_SOLAR_HEWALEX_DONGLE");
  Serial.println("role=ESP32-C3 USB dongle for ESP-NOW diagnostics/control of CYD solar controller");
  Serial.println("dongle_build=DongleESP_RX 2026-07-21 ALARMD5");
  Serial.println("cyd_expected_build=20260720N3");
  Serial.println("dongle_project=service-dongle/DongleESP_UPLOAD");
  Serial.println("cyd_project=solar-controller/CYD_SOLAR_OTA");
  Serial.println("dongle_default_port=COM3");
  Serial.println("cyd_default_port=COM4");
  Serial.println("serial_baud=115200");
  Serial.println("dongle_fqbn=esp32:esp32:esp32c3:CDCOnBoot=cdc");
  Serial.println("cyd_fqbn=esp32:esp32:esp32");
  Serial.println("normal_access=Use dongle COM3 serial; send commands below.");
  Serial.println("local_commands=status,help,project,manifest,info,ver,scan,ch 7,disp auto,disp off,disp 0,disp on");
  Serial.println("cyd_commands=cyd ver,cyd status,cyd alarm,cyd alarm ack,cyd log today,cyd log last,cyd log alarm,cyd log energy,cyd project,cyd readme N");
  Serial.println("raw_forwarding=Unknown commands are forwarded directly to CYD over ESP-NOW.");
  Serial.println("current_notes=CYD N3 has Telegram removed, log tail reads fixed, alarm ack and SD log status via ESP-NOW.");
  Serial.println("safety_rule=Do not upload firmware unless user confirms immediately before upload.");
  Serial.println("CODEX_PROJECT_MANIFEST_END");
  Serial.println();
}

void printCydAttentionBlock(const char *source) {
  const uint32_t age = lastPacketMs ? millis() - lastPacketMs : 999999UL;
  const bool radioOn = espNowUserEnabled && espNowRunning;
  const bool linkActive = radioOn && lastPacketMs && age < 3000;

  Serial.println();
  Serial.println("CYD_ATTENTION_BEGIN");
  Serial.printf("source=%s\n", source ? source : "unknown");
  Serial.printf("link=%s\n", linkActive ? "OK" : "OFF");
  Serial.printf("channel=%u\n", espNowRadioChannel);
  Serial.printf("packets=%lu\n", (unsigned long)packets);
  Serial.printf("last_age_ms=%lu\n", (unsigned long)age);
  Serial.printf("rssi=%d\n", haveRssi ? lastRssi : 0);
  Serial.printf("alarm_active=%u\n", cydAlarmActive ? 1 : 0);
  Serial.printf("alarm_kind=%u\n", cydAlarmKind);
  Serial.printf("alarm_sensor=%u\n", cydAlarmSensorTarget);
  Serial.printf("alarm_reason=%s\n", cydAlarmReason[0] ? cydAlarmReason : "-");
  Serial.printf("last_alarm_reply=%s\n", lastCydAlarmReply[0] ? lastCydAlarmReply : "-");
  Serial.println("recommended_commands=cyd status;cyd alarm;cyd log alarm;cyd log energy");
  Serial.println("project_commands=cyd project;cyd readme 1");
  Serial.println("readme_on_cyd_sd=/PROJECT.TXT");
  Serial.println("CYD_ATTENTION_END");
  Serial.println();
}

void sendRemoteCommand(const String &line) {
  if (!espNowUserEnabled || !espNowRunning) {
    Serial.println("CMD ERR: ESP-NOW OFF");
    return;
  }

  EspNowCommandPacket p = {};
  p.magic = TELEMETRY_MAGIC;
  p.version = TELEMETRY_VERSION;
  p.packetType = 2;
  p.packetSize = sizeof(EspNowCommandPacket);
  p.cmdSeq = ++commandSeq;
  snprintf(p.command, sizeof(p.command), "%s", line.c_str());

  esp_err_t err = esp_now_send(CYD_ESPNOW_MAC, (const uint8_t *)&p, sizeof(p));
  dongleCmdSent++;
  if (err != ESP_OK) {
    dongleCmdFail++;
    raiseDongleTxPower("send_err");
  }
  Serial.printf("CMD #%lu -> %s : %s\n",
                (unsigned long)p.cmdSeq,
                err == ESP_OK ? "SENT" : "ERR",
                p.command);
}

bool handleLocalCommand(const String &line) {
  if (line.equalsIgnoreCase("cyd ver") || line.equalsIgnoreCase("cyd version")) {
    sendRemoteCommand("ver");
    return true;
  }
  if (line.equalsIgnoreCase("cyd status")) {
    sendRemoteCommand("status");
    return true;
  }
  if (line.equalsIgnoreCase("cyd log today") || line.equalsIgnoreCase("cyd log day")) {
    sendRemoteCommand("log today");
    return true;
  }
  if (line.equalsIgnoreCase("cyd log last") || line.equalsIgnoreCase("cyd log hew")) {
    sendRemoteCommand("log last");
    return true;
  }
  if (line.equalsIgnoreCase("cyd log alarm") || line.equalsIgnoreCase("cyd log alarms")) {
    sendRemoteCommand("log alarm");
    return true;
  }
  if (line.equalsIgnoreCase("cyd log energy") || line.equalsIgnoreCase("cyd log power")) {
    sendRemoteCommand("log energy");
    return true;
  }
  if (line.equalsIgnoreCase("cyd project")) {
    sendRemoteCommand("project");
    return true;
  }
  if (line.length() >= 10 && line.substring(0, 10).equalsIgnoreCase("cyd readme")) {
    String arg = line.substring(10);
    arg.trim();
    if (arg.length()) sendRemoteCommand(String("readme ") + arg);
    else sendRemoteCommand("readme");
    return true;
  }
  if (line.equalsIgnoreCase("cyd alarm") || line.equalsIgnoreCase("cyd alarm status")) {
    sendRemoteCommand("alarm");
    return true;
  }
  if (line.equalsIgnoreCase("cyd alarm ack") || line.equalsIgnoreCase("cyd alarm clear")) {
    sendRemoteCommand("alarm ack");
    return true;
  }
  if (line.equalsIgnoreCase("ver") || line.equalsIgnoreCase("version")) {
    Serial.printf("VER %s\n", BUILD_ID);
    return true;
  }
  if (line.equalsIgnoreCase("help") || line == "?") {
    printHelp();
    return true;
  }
  if (line.equalsIgnoreCase("project") || line.equalsIgnoreCase("manifest") ||
      line.equalsIgnoreCase("info")) {
    printProjectManifest();
    return true;
  }
  if (line.equalsIgnoreCase("status")) {
    printLocalStatus();
    return true;
  }
  if (line.equalsIgnoreCase("scan") || line.equalsIgnoreCase("ch scan")) {
    espNowUserEnabled = true;
    const bool ok = scanCydChannel();
    Serial.printf("SCAN %s ch=%u packets=%lu\n",
                  ok ? "OK" : "ERR",
                  espNowRadioChannel,
                  (unsigned long)packets);
    return true;
  }
  if (line.length() >= 4 && line.substring(0, 3).equalsIgnoreCase("ch ")) {
    const uint8_t ch = constrain(line.substring(3).toInt(), ESPNOW_CHANNEL_MIN, ESPNOW_CHANNEL_MAX);
    espNowUserEnabled = true;
    stopEspNowRadio();
    const bool ok = startEspNowRadioOnChannel(ch);
    Serial.printf("CH %s ch=%u\n", ok ? "OK" : "ERR", espNowRadioChannel);
    return true;
  }
  if (line.equalsIgnoreCase("disp auto")) {
    displayOverrideEnabled = false;
    Serial.println("DISP AUTO");
    return true;
  }
  if (line.equalsIgnoreCase("disp off")) {
    displayOverrideEnabled = true;
    displayOverrideState = DISPLAY_STATE_OFF;
    Serial.println("DISP OFF");
    return true;
  }
  if (line.equalsIgnoreCase("disp 0")) {
    displayOverrideEnabled = true;
    displayOverrideState = DISPLAY_STATE_ZERO;
    Serial.println("DISP 0");
    return true;
  }
  if (line.equalsIgnoreCase("disp on")) {
    displayOverrideEnabled = true;
    displayOverrideState = DISPLAY_STATE_ON;
    Serial.println("DISP ON");
    return true;
  }
  return false;
}

void handleSerialCommands() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (line.length() < 95) line += c;
      continue;
    }

    line.trim();
    if (line.length()) {
      if (!handleLocalCommand(line)) sendRemoteCommand(line);
    }
    line = "";
  }
}

void onEspNowSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  (void)info;
  if (status == ESP_NOW_SEND_SUCCESS) {
    dongleCmdOk++;
    return;
  }
  dongleCmdFail++;
  raiseDongleTxPower("ack_fail");
}

void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!espNowUserEnabled || !espNowRunning) return;

  if (len == (int)sizeof(EspNowReplyPacket)) {
    EspNowReplyPacket r;
    memcpy(&r, data, sizeof(r));
    if (r.magic == TELEMETRY_MAGIC && r.version == TELEMETRY_VERSION &&
        r.packetType == 3 && r.packetSize == sizeof(EspNowReplyPacket)) {
      Serial.printf("REPLY #%lu %s: %s\n",
                    (unsigned long)r.cmdSeq,
                    r.ok ? "OK" : "ERR",
                    r.message);
      if (strstr(r.message, "alarm")) {
        snprintf(lastCydAlarmReply, sizeof(lastCydAlarmReply), "%s", r.message);
        if (startupAlarmQuerySent && !startupAttentionPrinted) {
          startupAttentionPrinted = true;
          printCydAttentionBlock("startup_alarm_reply");
        }
      }
      if (r.ok && strstr(r.message, "alarm ack")) {
        cydAlarmActive = false;
        cydAlarmKind = 0;
        cydAlarmSensorTarget = 0;
        cydAlarmReason[0] = '\0';
        snprintf(lastCydAlarmReply, sizeof(lastCydAlarmReply), "%s", r.message);
        drawDongleDisplay(true, true);
      }
      return;
    }
  }

  if (len != (int)sizeof(HewalexTelemetryPacket)) {
    badPackets++;
    Serial.printf("BAD LEN=%d expected=%u total_bad=%lu\n",
                  len,
                  (unsigned)sizeof(HewalexTelemetryPacket),
                  (unsigned long)badPackets);
    return;
  }

  HewalexTelemetryPacket p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != TELEMETRY_MAGIC || p.version != TELEMETRY_VERSION ||
      p.packetSize != sizeof(HewalexTelemetryPacket)) {
    badPackets++;
    Serial.printf("BAD MAGIC/VER size=%u total_bad=%lu\n",
                  p.packetSize,
                  (unsigned long)badPackets);
    return;
  }

  packets++;
  lastPacketMs = millis();
  if (!startupFirstLinkMs) startupFirstLinkMs = lastPacketMs;
  lastTxPowerPercent = p.espNowTxPowerPercent;
  updateCydAlarm(p);
  if (info && info->rx_ctrl) {
    lastRssi = info->rx_ctrl->rssi;
    haveRssi = true;
    serviceDongleEcoRssi(lastRssi);
  }

  Serial.printf("%02u:%02u:%02u #%lu from ",
                p.hh, p.mi, p.ss, (unsigned long)p.seq);
  if (info && info->src_addr) printMac(info->src_addr);
  else Serial.print("--");

  Serial.printf(" RS=%s age=%lums mode=%s pumpC=%s pwm=%u%%",
                p.rsOnline ? "OK" : "OFF",
                (unsigned long)p.rsAgeMs,
                pumpModeName(p.pumpMode),
                p.pumpC ? "ON" : "OFF",
                p.pumpPwmPercent);

  Serial.print(" T1=");
  printFloatOrDash(p.t1);
  Serial.print(" T2=");
  printFloatOrDash(p.t2);
  Serial.print(" T3=");
  printFloatOrDash(p.t3);
  Serial.print(" T4=");
  printFloatOrDash(p.t4);

  Serial.print(" DS[");
  Serial.print(p.dsCount);
  Serial.print("]=");
  printFloatOrDash(p.ds0);
  Serial.print('/');
  printFloatOrDash(p.ds1);
  Serial.print('/');
  printFloatOrDash(p.ds2);

  Serial.print(" SOL=");
  printFloatOrDash(p.solarCollectorTemp);
  Serial.print('/');
  printFloatOrDash(p.solarTankInTemp);
  Serial.print('/');
  printFloatOrDash(p.solarReturnTemp);

  Serial.print(" flowHz=");
  printFloatOrDash(p.flowHz, 2);
  Serial.print(" raw=");
  printFloatOrDash(p.flowHzRaw, 2);
  Serial.printf(" pulse=%u rej=%u", p.flowPulsesLast, p.flowRejectedLast);

  Serial.print(" dT=");
  printFloatOrDash(p.deltaT);
  Serial.print(" kW=");
  printFloatOrDash(p.powerKW, 2);
  Serial.print(" Q=");
  printFloatOrDash(p.pumpTargetFlowLMin);

  Serial.printf(" LDR=%u BL=%u SD=%s log=%s stat=%04X/%04X frames=%lu bad=%lu esp=%lu/%lu/%lu pkts=%lu badpkts=%lu\n",
                p.lightRaw,
                p.backlightDuty,
                p.sdOK ? "OK" : "OFF",
                p.logOK ? "OK" : "OFF",
                p.status196Raw,
                p.status196Core,
                (unsigned long)p.tempFrames,
                (unsigned long)p.badFrames,
                (unsigned long)p.espSent,
                (unsigned long)p.espOk,
                (unsigned long)p.espFail,
                (unsigned long)packets,
                (unsigned long)badPackets);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  bootMs = millis();
  Serial.println();
  Serial.printf("BOOT %s\n", BUILD_ID);
  printProjectManifest();
  printHelp();

  pinMode(BLUE_LED_PIN, OUTPUT);
  setBlueLed(false);
  pinMode(RADIO_BUTTON_PIN, INPUT_PULLUP);
  buttonReadingLast = digitalRead(RADIO_BUTTON_PIN);
  buttonStableState = buttonReadingLast;
  Serial.printf("BUTTON pin=%d start=%s\n", RADIO_BUTTON_PIN, buttonStableState == LOW ? "LOW" : "HIGH");

  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  if (startEspNowRadio()) {
    delay(80);
    displayState = DISPLAY_STATE_ZERO;
    lastDisplayMs = millis();
    drawDongleDisplay(true, false);
  } else {
    displayState = DISPLAY_STATE_OFF;
    lastDisplayMs = millis();
    drawDongleDisplay(false, false);
  }
}

void loop() {
  static uint32_t lastInfoMs = 0;
  static bool lastBannerOn = false;
  static uint32_t lastRetryMs = 0;
  handleRadioButton();
  handleSerialCommands();

  if (espNowUserEnabled && !espNowRunning && millis() - lastRetryMs >= RADIO_RETRY_MS) {
    lastRetryMs = millis();
    Serial.println("ESP-NOW/WiFi auto-retry...");
    startEspNowRadio();
  }

  uint32_t age = lastPacketMs ? millis() - lastPacketMs : 999999;
  bool radioOn = espNowUserEnabled && espNowRunning;
  bool linkActive = radioOn && lastPacketMs && age < 3000;

  if (linkActive && startupFirstLinkMs && !startupAlarmQuerySent &&
      millis() - startupFirstLinkMs >= 800) {
    startupAlarmQuerySent = true;
    sendRemoteCommand("alarm");
  }
  if (linkActive && startupAlarmQuerySent && !startupAttentionPrinted &&
      millis() - startupFirstLinkMs >= 4000) {
    startupAttentionPrinted = true;
    printCydAttentionBlock("startup_timeout");
  }
  if (radioOn && !lastPacketMs && !startupAttentionPrinted && bootMs &&
      millis() - bootMs >= 8000) {
    startupAttentionPrinted = true;
    printCydAttentionBlock("startup_no_link");
  }

  DisplayState newState = currentDisplayState(radioOn, linkActive);
  updateBlueLed(radioOn, linkActive);

  const uint32_t displayRefreshMs = linkActive && cydAlarmActive ? 250UL : 1000UL;
  if (newState != displayState || millis() - lastDisplayMs >= displayRefreshMs) {
    displayState = newState;
    lastDisplayMs = millis();
    drawDongleDisplay(radioOn, linkActive);
  }

  if (millis() - lastInfoMs >= 5000) {
    lastInfoMs = millis();
    if (linkActive != lastBannerOn) {
      lastBannerOn = linkActive;
      printDongleBanner(radioOn, linkActive);
    }
    Serial.printf("RX status: radio=%s user=%u running=%u link=%u packets=%lu bad=%lu last_age=%lums display=%s override=%u alarm=%u cyd_tx=%u%% dongle_tx=%u%% rssi=%d cmd=%lu/%lu/%lu\n",
                  radioOn ? "ON" : "OFF",
                  espNowUserEnabled ? 1 : 0,
                  espNowRunning ? 1 : 0,
                  linkActive ? 1 : 0,
                  (unsigned long)packets,
                  (unsigned long)badPackets,
                  (unsigned long)age,
                  displayStateName(newState),
                  displayOverrideEnabled ? 1 : 0,
                  cydAlarmActive ? 1 : 0,
                  lastTxPowerPercent,
                  dongleTxPowerPercent(),
                  haveRssi ? lastRssi : 0,
                  (unsigned long)dongleCmdSent,
                  (unsigned long)dongleCmdOk,
                  (unsigned long)dongleCmdFail);
  }
}
