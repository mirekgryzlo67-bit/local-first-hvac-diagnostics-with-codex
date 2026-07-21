/*
  CYD_HEWALEX_SCHEMAT

  Passive Hewalex / GECO RS485 monitor for CYD ESP32-2432S028R.

  Wiring, active RS1:
    CYD 3V3        -> isolated RS485 VIN, TTL side
    CYD GND        -> isolated RS485 GND, TTL side
    isolated TX    -> CYD GPIO27 (RX UART)
    isolated RX    -> CYD GPIO22 (TX UART)
    RS485 A+ / B+  -> Hewalex RS1
    RS485 earth    -> not connected

  Shows a panel-like schematic:
    - hydraulic diagram on the left
    - T1..T4 values on the right
    - C/F/E/K status markers at the bottom

  Status:
    GECO register 196 is used as status flags. In captured logs its lower
    status part is 0x0823 while the top bits rotate as a display counter.
    C and E flags below are based on the available GECO notes; F is shown as
    unknown until we capture a log where pump F switches.
*/

#define CYD_SOLAR_APP_VERSION "CYD_SOLAR PCWU/SOLAR"
#define CYD_SOLAR_BUILD_ID "20260721N6"
#define ENABLE_TELEGRAM 0
#pragma message("CYD_SOLAR_BUILD_ID=" CYD_SOLAR_BUILD_ID)

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Wire.h>
#include <WiFi.h>
#if ENABLE_TELEGRAM
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#endif
#include <ArduinoOTA.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <EspNowEco.h>
#include <time.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>

enum ScreenId : uint8_t;
enum SetupEditField : uint8_t;
struct HewalexTelemetryPacket;
void writeCalLog(const char *phase, const String &note, float rotameterValue = NAN);
void acknowledgeAlarm();

// ---------- DISPLAY ----------
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite ui = TFT_eSprite(&tft);

// ---------- TOUCH - HSPI ----------
#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33

#define TOUCH_MIN_X 402
#define TOUCH_MAX_X 3654
#define TOUCH_MIN_Y 522
#define TOUCH_MAX_Y 3542

SPIClass touchSpi(HSPI);
XPT2046_Touchscreen touch(XPT2046_CS, XPT2046_IRQ);

// ---------- UART RS485 ----------
HardwareSerial RSbus(1);

static constexpr int RS_RX_PIN = 27;
static constexpr int RS_TX_PIN = 22;
static constexpr uint32_t GECO_BAUD = 38400;

// ---------- SD LOGGER ----------
static constexpr int SD_CS = 5;
static constexpr int SD_CLK = 18;
static constexpr int SD_MISO = 19;
static constexpr int SD_MOSI = 23;
static constexpr const char *LOG_DIR = "/HEWALEX";
static constexpr uint32_t FULL_LOG_MS = 15UL * 60UL * 1000UL;
static constexpr uint32_t HEARTBEAT_MS = 5UL * 60UL * 1000UL;
static constexpr float TEMP_LOG_DELTA = 0.3f;

SPIClass sdSpi(VSPI);
File logFile;
String logFileName = "-";
bool sdOK = false;
bool logOK = false;
uint32_t lastLogHeartbeatMs = 0;
uint32_t lastLoggedFrameCount = 0;
uint32_t lastLogFlushMs = 0;

// false = prawdziwy pasywny podsluch RS485 Hewalex.
// true = ekran testowy z symulowanymi temperaturami i statusem pompy.
static constexpr bool DEMO_MODE = false;

// ---------- WIFI / NTP ----------
// Wpisz dane swojej sieci. Gdy SSID zostanie puste, NTP jest pomijane.
static const char *WIFI_SSID = "YOUR_WIFI_SSID";
static const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";
static const char *NTP_SERVER_1 = "pool.ntp.org";
static const char *NTP_SERVER_2 = "time.google.com";
static const char *TZ_EUROPE_WARSAW = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// ---------- TELEGRAM ----------
// Wylaczone w tej wersji, zeby odzyskac miejsce we flashu.
#if ENABLE_TELEGRAM
static constexpr const char *TELEGRAM_CFG_PATH = "/HEWALEX/TELEGRAM.CFG";
static constexpr uint32_t TELEGRAM_POLL_MS = 5000;
static constexpr uint32_t TELEGRAM_RELOAD_MS = 60000;
static constexpr uint32_t TELEGRAM_SEND_MIN_MS = 1200;
static constexpr uint32_t TELEGRAM_ONLINE_RETRY_MS = 60000;
#endif

// ---------- ESP-NOW TELEMETRY ----------
static constexpr bool ESPNOW_ENABLED = true;
static constexpr bool ESPNOW_FIXED_CHANNEL_ONLY = false;
static constexpr uint8_t ESPNOW_CHANNEL = 1;
static constexpr uint32_t ESPNOW_SEND_MS = 1000;
static constexpr uint32_t ESPNOW_LINK_TIMEOUT_MS = 5000;
static constexpr uint32_t TELEMETRY_MAGIC = 0x48575843UL;  // "HWXC"
static constexpr uint8_t TELEMETRY_VERSION = 2;
static uint8_t DONGLE_ESPNOW_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static constexpr const char *OTA_HOSTNAME = "CYD-Hewalex";
static constexpr uint16_t OTA_PORT = 3232;

// ---------- CYD LIGHT SENSOR / BACKLIGHT ----------
static constexpr int LIGHT_SENSOR_PIN = 34;   // typowy LDR w CYD ESP32-2432S028R
static constexpr int BACKLIGHT_PIN = 21;      // typowe podswietlenie TFT w CYD
static constexpr bool AUTO_BACKLIGHT = true;
static constexpr int LDR_SENSITIVITY = 35;
static constexpr int LDR_MIN_BRIGHTNESS = 30;
static constexpr int LDR_MAX_BRIGHTNESS = 150;
static constexpr int LDR_DARK_RAW = 220;
static constexpr int LDR_BRIGHT_RAW = 0;

// ---------- PRESSURE SENSOR JHM1200 I2C ----------
// Gotowe pod JHM1200, ale na CYD piny I2C trzeba jeszcze fizycznie ustalic.
// Standardowe GPIO21/22 sa w tym projekcie zajete przez podswietlenie i RS485.
static constexpr bool PRESSURE_I2C_ENABLED = false;
static constexpr int PRESSURE_SDA_PIN = -1;
static constexpr int PRESSURE_SCL_PIN = -1;
static constexpr uint8_t PRESSURE_I2C_ADDR = 0x78;
static constexpr uint32_t PRESSURE_READ_MS = 2000;
static constexpr float PRESSURE_P_OFFSET_PA = -125000.0f;
static constexpr float PRESSURE_P_SPAN_PA = 1250000.0f;

// ---------- HEATER / SSR OUTPUT ----------
// Brak wybranego GPIO = fizycznie zawsze OFF. Ustaw konkretny pin dopiero po potwierdzeniu wolnego GPIO.
static constexpr int HEATER_RELAY_PIN = -1;
static constexpr bool HEATER_RELAY_ACTIVE_HIGH = true;

// ---------- SOLAR PUMP / FLOW / EVENT LOG ----------
static constexpr int PUMP_PWM_PIN = 17;
static constexpr uint32_t PUMP_PWM_FREQ = 1500;
static constexpr uint8_t PUMP_PWM_BITS = 8;
static constexpr int FLOW_PIN = 35;
static constexpr uint32_t FLOW_SAMPLE_MS = 1000;
static constexpr uint32_t FLOW_MIN_PULSE_US = 5000;
static constexpr float FLOW_SMOOTH_ALPHA = 0.25f;
static constexpr uint32_t CAL_LOG_INTERVAL_MS = 60UL * 1000UL;
static constexpr uint32_t ENERGY_LOG_INTERVAL_MS = 60UL * 1000UL;
static constexpr uint32_t STATE_SYNC_INTERVAL_MS = 60UL * 1000UL;
static constexpr uint8_t ENERGY_PWM_BINS = 30;
static constexpr uint32_t TOUCH_DEBOUNCE_MS = 300;
static constexpr const char *CAL_LOG_PREFIX = "/HEWALEX/CAL";
static constexpr const char *ENERGY_LOG_PATH = "/HEWALEX/ENERGY.CSV";
static constexpr const char *ENERGY_STATE_PATH = "/HEWALEX/ENERGY_STATE.CSV";
static constexpr const char *ALARM_LOG_PATH = "/HEWALEX/ALARM.CSV";
static constexpr const char *PROJECT_INFO_PATH = "/PROJECT.TXT";
static constexpr uint8_t FLOW_SENSOR_START_OUTPUT_PCT = 70;    // pump output scale: flowmeter starts around here
static constexpr uint8_t FLOW_SENSOR_START_MARGIN_PCT = 5;     // requested reserve above start threshold
static constexpr uint8_t FLOW_SENSOR_TEST_OUTPUT_PCT = FLOW_SENSOR_START_OUTPUT_PCT + FLOW_SENSOR_START_MARGIN_PCT;
static constexpr float FLOW_PRESENT_MIN_HZ = 1.0f;
static constexpr uint8_t FLOW_PRESENT_MIN_PULSES = 1;
static constexpr float FLOW_TEMP_EVIDENCE_DELTA_C = 0.4f;
static constexpr float FLOW_SOL_T1_BLOCK_DELTA_C = 10.0f;
static constexpr uint32_t PUMP_START_BOOST_MS = 15000;
static constexpr uint32_t TUBE_COLLECTOR_REFRESH_MS = 60UL * 60UL * 1000UL;
static constexpr uint32_t NO_FLOW_RETRY_MS = 3UL * 60UL * 1000UL;
static constexpr bool FLOWMETER_INTERLOCK_ENABLED = false;
static constexpr bool FLOWMETER_FAULTS_ENABLED = false;
static constexpr bool FLOWMETER_FAULT_UI_ENABLED = false;
static constexpr uint32_t PUMP_DT_CONTROL_MS = 10000;
static constexpr float PUMP_DT_DEADBAND_C = 0.5f;
static constexpr uint8_t PUMP_DT_STEP_PCT = 2;
static constexpr uint8_t DAY_FALLBACK_START_HOUR = 5;
static constexpr uint8_t DAY_FALLBACK_END_HOUR = 19;
static constexpr uint32_t RS_ONLINE_TIMEOUT_MS = 5000;
static constexpr uint32_t RS_ALARM_START_DELAY_MS = 30UL * 1000UL;
static constexpr const char *APP_VERSION = CYD_SOLAR_APP_VERSION;
static constexpr const char *BUILD_ID = CYD_SOLAR_BUILD_ID;
static constexpr const char *APP_VERSION_SHORT = "SOLAR";

// ---------- SOLAR DS18B20 TEMPERATURES ----------
static constexpr int DS18B20_PIN = 16;
static constexpr uint8_t DS18B20_MAX_SENSORS = 3;
static constexpr uint32_t DS18B20_READ_MS = 2500;
static constexpr uint32_t DS18B20_FAIL_TIMEOUT_MS = 5000;
static constexpr const char *DS_PREFS_NS = "cydsolar";
static constexpr const char *DS_PREF_KEYS[DS18B20_MAX_SENSORS] = {"ds_sol", "ds_wej", "ds_wyj"};

// Docelowo wpisz tu stale adresy DS18B20 i ustaw DS18B20_USE_STATIC_ADDRESSES = true.
// Kolejnosc: [0] = Tsol/kolektor, [1] = Twej, [2] = Twyj/powrot.
// Gdy adresy sa zerowe albo flaga false, program robi jeden skan kompatybilnosci przy starcie.
// Normalnie nie wykonuje juz automatycznych reskanow w loop(); reczny skan zostaje w SETUP/komendach.
static constexpr bool DS18B20_USE_STATIC_ADDRESSES = false;
static const DeviceAddress DS18B20_STATIC_ADDRESSES[DS18B20_MAX_SENSORS] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Tsol
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Twej
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Twyj
};

OneWire dsOneWire(DS18B20_PIN);
DallasTemperature dsSensors(&dsOneWire);
DeviceAddress dsAddresses[DS18B20_MAX_SENSORS];
DeviceAddress dsDetectedAddresses[DS18B20_MAX_SENSORS];
float dsTemps[DS18B20_MAX_SENSORS] = {NAN, NAN, NAN};
float dsDetectedTemps[DS18B20_MAX_SENSORS] = {NAN, NAN, NAN};
uint8_t dsSensorCount = 0;
uint8_t dsDetectedCount = 0;
uint8_t dsDetectedRoles[DS18B20_MAX_SENSORS] = {0, 1, 2};
uint32_t lastDs18ReadMs = 0;
uint32_t lastDs18ScanMs = 0;
uint32_t dsLastGoodMs[DS18B20_MAX_SENSORS] = {0, 0, 0};
bool dsFault[DS18B20_MAX_SENSORS] = {false, false, false};
char dsRoleStatus[40] = "";
uint32_t dsRoleStatusMs = 0;

File calFile;
String calFileName = "-";
bool calLogOK = false;
bool calLogging = true;
bool energyLogOK = false;
uint32_t lastFlowSampleMs = 0;
uint32_t lastCalLogMs = 0;
uint32_t lastEnergyUpdateMs = 0;
uint32_t lastEnergyLogMs = 0;
uint32_t lastTouchMs = 0;
bool touchWasDown = false;
volatile uint32_t flowPulseCount = 0;
volatile uint32_t flowRejectedCount = 0;
volatile uint32_t flowLastPulseUs = 0;
uint32_t flowPulsesLast = 0;
uint32_t flowRejectedLast = 0;
float flowHz = 0.0f;
float flowHzRaw = 0.0f;
bool flowPresent = false;
uint32_t flowPresentSinceMs = 0;
uint32_t flowConfirmedRunMs = 0;
uint32_t lastFlowSeenMs = 0;
uint32_t lastTubeFlowConfirmMs = 0;
uint32_t lastTubeTestMs = 0;
uint32_t tubeBoostUntilMs = 0;
bool solarPumpAutoRunning = false;
bool noFlowFault = false;
bool flowSensorFault = false;
uint32_t noFlowTestUntilMs = 0;
uint32_t lastNoFlowTestMs = 0;
uint32_t lastPumpDtControlMs = 0;
float flowCheckBaseSol = NAN;
float flowCheckBaseWej = NAN;
float flowCheckBaseWyj = NAN;
float flowCheckBaseT1 = NAN;
float flowCheckBaseT3 = NAN;
uint8_t pumpPwmPercent = 0;
uint8_t pumpPwmRequestedPercent = 0;
uint8_t pumpDtOutputPercent = 0;
float pumpTargetFlowLMin = NAN;
float deltaT = NAN;
float powerKW = NAN;
float solarCollectorTemp = NAN;
float solarTankInTemp = NAN;
float solarReturnTemp = NAN;
float solarDeltaToT3 = NAN;
float controlDeltaT = NAN;
float reg428Temp = NAN;
uint8_t deltaTestTargetC = 3;
String calPhase = "IDLE";
String calNote = "";
float rotameterLMin = NAN;
bool pressureSensorOk = false;
float pressureSensorTempC = NAN;
uint32_t lastPressureReadMs = 0;
float dailyEnergyKwh = 0.0f;
float dailyMaxSolarTemp = NAN;
uint8_t energyDayKey = 0;
float energyHourSolar[25];
float energyHourTank[25];
float energyHourPower[25];
bool energyHourPump[25];
float energyPwmBins[ENERGY_PWM_BINS];
uint16_t energyPwmSum[ENERGY_PWM_BINS];
uint16_t energyPwmCount[ENERGY_PWM_BINS];
uint32_t lastStateSyncMs = 0;

bool serviceModeActive = false;
uint32_t serviceModeUntilMs = 0;
char serviceModeReason[32] = "";
bool heaterOutputOn = false;

struct SafetyState {
  bool sensorFault = false;
  bool solarOverheat = false;
  bool tankOverheat = false;
  bool heaterBlocked = false;
  bool dumpPumpRequest = false;
  bool pumpMaxForced = false;
  uint8_t solarOverheatCount = 0;
  uint8_t tankOverheatCount = 0;
  float hottestSolar = NAN;
  float hottestTank = NAN;
  char reason[40] = "OK";
};

SafetyState safety;

enum AlarmKind : uint8_t {
  ALARM_NONE,
  ALARM_TEMP,
  ALARM_SENSOR,
  ALARM_PRESSURE,
  ALARM_NO_FLOW,
  ALARM_FLOW_SENSOR
};

enum AlarmSensorTarget : uint8_t {
  ALARM_SENSOR_RS,
  ALARM_SENSOR_SOL,
  ALARM_SENSOR_WEJ,
  ALARM_SENSOR_WYJ
};

bool alarmActive = false;
bool alarmInReview = false;
AlarmKind alarmKind = ALARM_NONE;
AlarmSensorTarget alarmSensorTarget = ALARM_SENSOR_RS;
char alarmReason[40] = "SYMULACJA: SOLARY 92C";
uint32_t alarmAckMs = 0;

// ---------- COLORS ----------
#define C_BG      0x0000
#define C_TEXT    0x18C3
#define C_WHITE   TFT_WHITE
#define C_WARN    0xFD20
#define C_BAD     TFT_RED
#define C_DIM     0x9CD3
#define C_BLUE    0x04BF
#define C_LINE    0x632C
#define C_DARK    0x2104
#define C_HOT     0xF9C0
#define C_HOT2    0xF800
#define C_SOLAR_PIPE 0xFB20
#define C_COLD    0x057F
#define C_COLD2   0x001F
#define C_GREEN   0x47E0
#define C_METAL   0xBDF7
#define C_SHADOW  0xC638
#define C_PINK    0xFC9F
#define C_ASH     TFT_WHITE
#define C_PANEL   0x0822
#define C_PANEL2  0x1084
#define C_BORDER  0x2A8A

uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint16_t blend565(uint16_t a, uint16_t b, float t) {
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  uint8_t ar = ((a >> 11) & 0x1F) << 3;
  uint8_t ag = ((a >> 5) & 0x3F) << 2;
  uint8_t ab = (a & 0x1F) << 3;
  uint8_t br = ((b >> 11) & 0x1F) << 3;
  uint8_t bg = ((b >> 5) & 0x3F) << 2;
  uint8_t bb = (b & 0x1F) << 3;
  return rgb565(ar + (br - ar) * t, ag + (bg - ag) * t, ab + (bb - ab) * t);
}

uint16_t tempColor(float value) {
  if (isnan(value)) return C_DIM;
  if (value <= 5.0f) return 0x001F;    // blue
  if (value <= 18.0f) return blend565(0x001F, 0x07FF, (value - 5.0f) / 13.0f);
  if (value <= 30.0f) return blend565(0x07FF, 0x07E0, (value - 18.0f) / 12.0f);
  if (value <= 42.0f) return blend565(0x07E0, 0xFFE0, (value - 30.0f) / 12.0f);
  if (value <= 55.0f) return blend565(0xFFE0, 0xFD20, (value - 42.0f) / 13.0f);
  if (value <= 70.0f) return blend565(0xFD20, 0xF800, (value - 55.0f) / 15.0f);
  return 0xF800;
}

uint16_t readableTextColor(uint16_t bg) {
  uint8_t r = ((bg >> 11) & 0x1F) << 3;
  uint8_t g = ((bg >> 5) & 0x3F) << 2;
  uint8_t b = (bg & 0x1F) << 3;
  uint16_t lum = (uint16_t)r * 3 + (uint16_t)g * 6 + b;
  return lum > 950 ? C_DARK : C_WHITE;
}

uint16_t tempTextColor(float value) {
  if (isnan(value)) return C_DIM;
  if (value < 28.0f) return 0x04BF;
  if (value < 42.0f) return 0xFFE0;
  if (value < 55.0f) return 0xFD20;
  return 0xF800;
}

uint16_t tankRainbowColor(float pos) {
  if (pos < 0.18f) return blend565(0xF800, 0xFA60, pos / 0.18f);
  if (pos < 0.34f) return blend565(0xFA60, 0xFFE0, (pos - 0.18f) / 0.16f);
  if (pos < 0.52f) return blend565(0xFFE0, 0x07E0, (pos - 0.34f) / 0.18f);
  if (pos < 0.70f) return blend565(0x07E0, 0x07FF, (pos - 0.52f) / 0.18f);
  if (pos < 0.86f) return blend565(0x07FF, 0x049F, (pos - 0.70f) / 0.16f);
  return blend565(0x049F, 0x001F, (pos - 0.86f) / 0.14f);
}

// ---------- GECO FRAME ----------
static constexpr uint8_t FRAME_START = 0x69;
static constexpr uint8_t HARD_CONST = 0x84;
static constexpr uint16_t SOFT_CONST = 0x0080;
static constexpr uint8_t GECO_PUMP_PHY_ADDR = 0x02;
static constexpr uint8_t GECO_CYD_PHY_ADDR = 0x01;
static constexpr uint16_t GECO_PUMP_SOFT_ADDR = 0x0002;
static constexpr uint16_t GECO_CYD_SOFT_ADDR = 0x0001;
static constexpr uint32_t RS1_POLL_INTERVAL_MS = 1000;

static constexpr size_t FRAME_BUF_SIZE = 220;
uint8_t frameBuf[FRAME_BUF_SIZE];
uint16_t frameLen = 0;
int expectedLen = -1;
uint32_t lastRs1QueryMs = 0;
uint8_t rs1PollStep = 0;
uint32_t rs1TxCount = 0;

// ---------- DATA ----------
struct HewalexState {
  bool valid = false;
  uint32_t rxBytes = 0;
  uint32_t goodFrames = 0;
  uint32_t tempFrames = 0;
  uint32_t badFrames = 0;
  uint32_t lastFrameMs = 0;

  uint8_t yy = 0, mo = 0, dd = 0, hh = 0, mi = 0, ss = 0;
  float t1 = NAN;
  float t2 = NAN;
  float t3 = NAN;
  float t4 = NAN;

  uint16_t status196Raw = 0;
  uint16_t status196Core = 0;
  bool pumpC = false;
  bool heaterE = false;
  bool compressor = false;
};

HewalexState st;
uint32_t lastDrawMs = 0;
uint32_t lastDemoMs = 0;
uint32_t lastClockMs = 0;
uint32_t lastLightMs = 0;
bool ntpTimeOk = false;
bool ntpConfigured = false;
uint32_t lastNtpTryMs = 0;
uint32_t lastWifiTryMs = 0;
#if ENABLE_TELEGRAM
String telegramBotToken = "";
String telegramChatId = "";
bool telegramConfigured = false;
bool telegramOnlineSent = false;
uint32_t lastTelegramPollMs = 0;
uint32_t lastTelegramReloadMs = 0;
uint32_t lastTelegramSendMs = 0;
uint32_t lastTelegramOnlineTryMs = 0;
int32_t telegramLastUpdateId = 0;
int lastTelegramHttpCode = 0;
String lastTelegramError = "";
#endif
bool needFullRedraw = true;
int lightRaw = 0;
int lightMinRaw = 4095;
int lightMaxRaw = 0;
uint8_t backlightDuty = 180;

// ---------- SETUP UI ----------
static constexpr uint32_t SETUP_IDLE_RETURN_MS = 15000;

enum ScreenId : uint8_t {
  SCREEN_MAIN,
  SCREEN_SETUP1,
  SCREEN_SETUP2,
  SCREEN_SETUP3,
  SCREEN_SETUP4,
  SCREEN_SETUP5,
  SCREEN_SETUP6
};

enum PumpWorkMode : uint8_t {
  PUMP_MODE_AUTO,
  PUMP_MODE_MANUAL_START,
  PUMP_MODE_STOP
};

enum SetupEditField : uint8_t {
  SETUP_EDIT_NONE,
  SETUP_EDIT_TARGET_DT,
  SETUP_EDIT_START,
  SETUP_EDIT_STOP,
  SETUP_EDIT_DS_OFFSET,
  SETUP_EDIT_HEATER_TEMP
};

struct SetupSettings {
  PumpWorkMode pumpMode = PUMP_MODE_AUTO;
  SetupEditField editField = SETUP_EDIT_START;
  float deltaStart = 3.0f;
  float deltaStop = 1.0f;
  uint8_t manualPwmPercent = 50;
  float pressureBar = 1.4f;
  float pressureMinBar = 0.8f;
  float pressureMaxBar = 2.5f;
  uint8_t pwmMin = 18;
  uint8_t pwmMax = 90;
  float flowMinLMin = 1.5f;
  float flowTargetLMin = 2.5f;
  float targetDeltaT = 6.0f;
  float tempOffsetRs = 0.0f;
  float tempOffsetDs[3] = {0.0f, 0.0f, 0.0f};
  uint8_t dsOffsetIndex = 0;
  float heaterSetTemp = 45.0f;
  bool heaterEnabled = false;
  uint8_t ldrMinDuty = LDR_MIN_BRIGHTNESS;
  uint8_t ldrMaxDuty = LDR_MAX_BRIGHTNESS;
  float solarForcePwmTemp = 90.0f;
  float tankHeaterOffTemp = 75.0f;
  float tankDumpPumpOnTemp = 75.0f;
  uint16_t manualPwmTimeoutSec = 90;
  uint8_t otaMinutes = 10;
  bool espNowEnabled = true;
};

ScreenId currentScreen = SCREEN_MAIN;
SetupSettings setupCfg;
uint32_t lastSetupActivityMs = 0;

bool otaBegun = false;
bool otaActive = false;
uint32_t otaActivatedMs = 0;

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

bool espNowReady = false;
bool espNowLastSendOk = false;
uint32_t espNowSeq = 0;
uint32_t espNowSent = 0;
uint32_t espNowOk = 0;
uint32_t espNowFail = 0;
uint32_t lastEspNowSendMs = 0;
uint32_t lastEspNowOkMs = 0;
EspNowEcoLink espNowEco;
volatile bool remoteCmdPending = false;
uint32_t remoteCmdSeq = 0;
char remoteCmdText[96] = {0};

static constexpr uint32_t CLOCK_INTERVAL_MS = 1000;
static constexpr uint32_t NTP_RETRY_INTERVAL_MS = 30000UL;       // co 30 s do pierwszego zlapania czasu
static constexpr uint32_t NTP_RESYNC_INTERVAL_MS = 3600000UL;    // po zlapaniu NTP: odswiezanie co 1 h
static constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 30000UL;
static constexpr uint32_t LIGHT_INTERVAL_MS = 1500;
static constexpr uint32_t DEMO_INTERVAL_MS = 2000;
static constexpr uint32_t DRAW_INTERVAL_MS = 750;
static constexpr uint8_t BACKLIGHT_HYSTERESIS = 6;

struct LoggedState {
  bool valid = false;
  float t1 = NAN;
  float t2 = NAN;
  float t3 = NAN;
  float t4 = NAN;
  uint16_t status196Raw = 0;
  uint16_t status196Core = 0;
  bool pumpC = false;
  bool heaterE = false;
  bool compressor = false;
};

LoggedState lastLoggedState;

// Register 196 flags known/inferred from GECO notes.
static constexpr uint16_t STATUS_DISPLAY_COUNTER_MASK = 0xE000;
static constexpr uint16_t STATUS_PUMP_C_BIT = 0x0002;      // described as circulating pump in GECO notes
static constexpr uint16_t STATUS_COMPRESSOR_BIT = 0x0800;  // compressor
static constexpr uint16_t STATUS_HEATER_E_BIT = 0x1000;    // electric heater E

uint8_t crc8DvbS2(const uint8_t *buf, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

uint16_t crc16Xmodem(const uint8_t *buf, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)buf[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

uint16_t le16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

float temp10(const uint8_t *p) {
  int16_t raw = (int16_t)le16(p);
  return raw / 10.0f;
}

bool refreshClockFromSystem() {
  struct tm tmNow;
  if (!getLocalTime(&tmNow, 800)) return false;

  st.yy = (uint8_t)((tmNow.tm_year + 1900) % 100);
  st.mo = (uint8_t)(tmNow.tm_mon + 1);
  st.dd = (uint8_t)tmNow.tm_mday;
  st.hh = (uint8_t)tmNow.tm_hour;
  st.mi = (uint8_t)tmNow.tm_min;
  st.ss = (uint8_t)tmNow.tm_sec;
  ntpTimeOk = true;
  return true;
}

void configureNtpOnce() {
  if (ntpConfigured) return;
  configTzTime(TZ_EUROPE_WARSAW, NTP_SERVER_1, NTP_SERVER_2);
  ntpConfigured = true;
  lastNtpTryMs = 0;
}

void startWifiAndNtp() {
  if (!WIFI_SSID || WIFI_SSID[0] == 0) return;

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);

  if (ESPNOW_ENABLED && ESPNOW_FIXED_CHANNEL_ONLY) {
    WiFi.disconnect(false, false);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    return;
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiTryMs = millis();

  const uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 7000) {
    delay(100);
  }
  if (WiFi.status() != WL_CONNECTED) return;

  configureNtpOnce();
  for (uint8_t i = 0; i < 20; i++) {
    if (refreshClockFromSystem()) return;
    delay(100);
  }
}

void serviceWifiAndNtp() {
  if (!WIFI_SSID || WIFI_SSID[0] == 0) return;
  if (ESPNOW_ENABLED && ESPNOW_FIXED_CHANNEL_ONLY) return;

  const uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    ntpConfigured = false;
    if (now - lastWifiTryMs >= WIFI_RETRY_INTERVAL_MS) {
      lastWifiTryMs = now;
      WiFi.disconnect(false, false);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }

  configureNtpOnce();

  const uint32_t ntpIntervalMs = ntpTimeOk ? NTP_RESYNC_INTERVAL_MS : NTP_RETRY_INTERVAL_MS;
  if (now - lastNtpTryMs >= ntpIntervalMs) {
    lastNtpTryMs = now;
    refreshClockFromSystem();
  }
}

#if ENABLE_TELEGRAM
String urlEncode(const String &value) {
  String out;
  const char *hex = "0123456789ABCDEF";
  out.reserve(value.length() * 3);
  for (size_t i = 0; i < value.length(); i++) {
    const uint8_t c = (uint8_t)value[i];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
      out += (char)c;
    } else if (c == ' ') {
      out += '+';
    } else {
      out += '%';
      out += hex[(c >> 4) & 0x0F];
      out += hex[c & 0x0F];
    }
  }
  return out;
}

String tgTemp(float value) {
  if (isnan(value)) return "--.-";
  char buf[14];
  snprintf(buf, sizeof(buf), "%.1f", value);
  return String(buf);
}

bool loadTelegramConfig() {
  if (!sdOK) return false;
  File f = SD.open(TELEGRAM_CFG_PATH, FILE_READ);
  if (!f) {
    telegramConfigured = false;
    return false;
  }

  String token;
  String chat;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || line.startsWith("#")) continue;
    const int eq = line.indexOf('=');
    if (eq < 0) continue;
    String key = line.substring(0, eq);
    String val = line.substring(eq + 1);
    key.trim();
    val.trim();
    key.toUpperCase();
    if (key == "BOT_TOKEN") token = val;
    else if (key == "CHAT_ID") chat = val;
  }
  f.close();

  telegramBotToken = token;
  telegramChatId = chat;
  telegramConfigured = telegramBotToken.length() > 20 && telegramChatId.length() > 0;
  Serial.printf("TELEGRAM cfg=%d chat=%s\n", telegramConfigured ? 1 : 0,
                telegramConfigured ? "OK" : "--");
  return telegramConfigured;
}

bool telegramHttpPost(const String &method, const String &body, String *response = nullptr) {
  lastTelegramHttpCode = 0;
  lastTelegramError = "";
  if (!telegramConfigured) {
    lastTelegramError = "not configured";
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    lastTelegramError = "wifi off";
    return false;
  }
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  const String url = "https://api.telegram.org/bot" + telegramBotToken + "/" + method;
  if (!http.begin(client, url)) {
    lastTelegramError = "http begin";
    return false;
  }
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  const int code = http.POST(body);
  lastTelegramHttpCode = code;
  String resp = http.getString();
  if (code < 0 && !resp.length()) resp = http.errorToString(code);
  if (resp.length() > 120) resp = resp.substring(0, 120);
  lastTelegramError = resp;
  if (response) *response = resp;
  http.end();
  return code >= 200 && code < 300;
}

bool telegramHttpGet(const String &methodAndQuery, String &response) {
  lastTelegramHttpCode = 0;
  lastTelegramError = "";
  if (!telegramConfigured) {
    lastTelegramError = "not configured";
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    lastTelegramError = "wifi off";
    return false;
  }
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  const String url = "https://api.telegram.org/bot" + telegramBotToken + "/" + methodAndQuery;
  if (!http.begin(client, url)) {
    lastTelegramError = "http begin";
    return false;
  }
  const int code = http.GET();
  lastTelegramHttpCode = code;
  response = http.getString();
  if (code < 0 && !response.length()) response = http.errorToString(code);
  lastTelegramError = response.length() > 120 ? response.substring(0, 120) : response;
  http.end();
  return code >= 200 && code < 300;
}

bool telegramSocketProbe(IPAddress &ipOut) {
  ipOut = IPAddress((uint32_t)0);
  if (WiFi.status() != WL_CONNECTED) {
    lastTelegramError = "wifi off";
    return false;
  }
  if (!WiFi.hostByName("api.telegram.org", ipOut)) {
    lastTelegramError = "dns fail";
    return false;
  }
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(5000);
  const bool ok = client.connect("api.telegram.org", 443);
  if (!ok) {
    lastTelegramError = "socket connect fail";
    client.stop();
    return false;
  }
  client.stop();
  return true;
}

bool telegramTcpProbe(IPAddress &ipOut) {
  ipOut = IPAddress((uint32_t)0);
  if (WiFi.status() != WL_CONNECTED) {
    lastTelegramError = "wifi off";
    return false;
  }
  if (!WiFi.hostByName("api.telegram.org", ipOut)) {
    lastTelegramError = "dns fail";
    return false;
  }
  WiFiClient client;
  client.setTimeout(5000);
  const bool ok = client.connect("api.telegram.org", 443);
  if (!ok) {
    lastTelegramError = "tcp connect fail";
    client.stop();
    return false;
  }
  client.stop();
  return true;
}

bool telegramSendMessage(const String &text) {
  if (millis() - lastTelegramSendMs < TELEGRAM_SEND_MIN_MS) delay(TELEGRAM_SEND_MIN_MS);
  const String body = "chat_id=" + urlEncode(telegramChatId) +
                      "&text=" + urlEncode(text);
  const bool ok = telegramHttpPost("sendMessage", body);
  lastTelegramSendMs = millis();
  Serial.printf("TELEGRAM send=%d code=%d len=%u err=%s\n",
                ok ? 1 : 0, lastTelegramHttpCode, text.length(), lastTelegramError.c_str());
  return ok;
}

String telegramOnlineText() {
  IPAddress ip = WiFi.localIP();
  String msg = "STEROWNIK SOLARY ONLINE\n\n";
  msg += "IP: " + ip.toString() + "\n";
  msg += "RSSI: " + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) + " dBm\n";
  msg += "FW: " + String(APP_VERSION) + "\n";
  msg += "BUILD: " + String(BUILD_ID);
  return msg;
}

String telegramStatusText() {
  String msg = "STATUS\n\n";
  msg += "T1 = " + tgTemp(st.t1) + " C\n";
  msg += "T2 = " + tgTemp(st.t2) + " C\n";
  msg += "T3 = " + tgTemp(st.t3) + " C\n";
  msg += "T4 = " + tgTemp(st.t4) + " C\n\n";
  msg += "SOL = " + tgTemp(solarCollectorTemp) + " C\n";
  msg += "WEJ = " + tgTemp(solarTankInTemp) + " C\n";
  msg += "WYJ = " + tgTemp(solarReturnTemp) + " C\n\n";
  msg += "Pompa = " + String(isSolarPumpUiActive() ? "ON" : "OFF") + "\n";
  msg += "PWM = " + String(displayedPumpPercent()) + " %\n";
  msg += "Przeplyw = " + String(flowHz, 2) + " Hz\n";
  msg += "RS485 = " + String(isRsOnline() ? "OK" : "OFF") + "\n";
  msg += "SD = " + String(sdOK && logOK ? "OK" : "ERR") + "\n";
  msg += "RSSI = " + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) + " dBm";
  return msg;
}

String telegramInfoText() {
  String msg = "INFO\n\n";
  msg += "FW: " + String(APP_VERSION) + "\n";
  msg += "BUILD: " + String(BUILD_ID) + "\n";
  msg += "IP: " + WiFi.localIP().toString() + "\n";
  msg += "RS485: " + String(isRsOnline() ? "OK" : "OFF") + "\n";
  msg += "SD: " + String(sdOK ? "OK" : "ERR") + "\n";
  msg += "LOG: " + String(logOK ? logFileName : "ERR") + "\n";
  msg += "DS: " + String(dsSensorCount);
  return msg;
}

void sendTelegramOnline() {
  if (telegramOnlineSent) return;
  if (!telegramConfigured || WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastTelegramOnlineTryMs < TELEGRAM_ONLINE_RETRY_MS) return;
  lastTelegramOnlineTryMs = millis();
  if (telegramSendMessage(telegramOnlineText())) telegramOnlineSent = true;
}

void handleTelegramCommand(const String &cmd) {
  if (cmd == "/status") telegramSendMessage(telegramStatusText());
  else if (cmd == "/version") telegramSendMessage(String(APP_VERSION) + "\n" + BUILD_ID);
  else if (cmd == "/rssi") telegramSendMessage("RSSI: " + String(WiFi.RSSI()) + " dBm");
  else if (cmd == "/info" || cmd == "/start") telegramSendMessage(telegramInfoText());
  else if (cmd == "/log") telegramSendMessage("LOG: " + String(logOK ? logFileName : "ERR") +
                                              "\nSD: " + String(sdOK ? "OK" : "ERR"));
  else telegramSendMessage("Nieznana komenda.");
}

void parseTelegramUpdates(const String &payload) {
  int pos = 0;
  while (true) {
    const int u = payload.indexOf("\"update_id\":", pos);
    if (u < 0) break;
    const int idStart = u + 12;
    const int idEnd = payload.indexOf(',', idStart);
    if (idEnd < 0) break;
    const int32_t updateId = payload.substring(idStart, idEnd).toInt();
    const int nextU = payload.indexOf("\"update_id\":", idEnd);
    const String item = payload.substring(u, nextU < 0 ? payload.length() : nextU);
    if (updateId > telegramLastUpdateId) telegramLastUpdateId = updateId;

    const String chatNeedle = "\"chat\":{\"id\":" + telegramChatId;
    if (item.indexOf(chatNeedle) < 0) {
      pos = idEnd;
      continue;
    }

    const int t = item.indexOf("\"text\":\"");
    if (t >= 0) {
      const int textStart = t + 8;
      const int textEnd = item.indexOf('"', textStart);
      if (textEnd > textStart) {
        String cmd = item.substring(textStart, textEnd);
        cmd.trim();
        cmd.toLowerCase();
        handleTelegramCommand(cmd);
      }
    }
    pos = idEnd;
  }
}

void serviceTelegram() {
  const uint32_t now = millis();
  if (!telegramConfigured && sdOK && now - lastTelegramReloadMs >= TELEGRAM_RELOAD_MS) {
    lastTelegramReloadMs = now;
    loadTelegramConfig();
  }
  if (!telegramConfigured || WiFi.status() != WL_CONNECTED) return;

  sendTelegramOnline();
  if (now - lastTelegramPollMs < TELEGRAM_POLL_MS) return;
  lastTelegramPollMs = now;

  String response;
  String query = "getUpdates?timeout=0&limit=5";
  if (telegramLastUpdateId > 0) query += "&offset=" + String(telegramLastUpdateId + 1);
  if (telegramHttpGet(query, response)) parseTelegramUpdates(response);
}
#else
bool loadTelegramConfig() { return false; }
void sendTelegramOnline() {}
void serviceTelegram() {}
bool telegramSendMessage(const String &) { return false; }
#endif

void setupLightSensorAndBacklight() {
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(LIGHT_SENSOR_PIN, ADC_11db);
  if (AUTO_BACKLIGHT) {
    pinMode(BACKLIGHT_PIN, OUTPUT);
    ledcAttach(BACKLIGHT_PIN, 5000, 8);
    ledcWrite(BACKLIGHT_PIN, backlightDuty);
  }
}

void setupPressureSensor() {
  pressureSensorOk = false;
  if (!PRESSURE_I2C_ENABLED) return;
  if (PRESSURE_SDA_PIN < 0 || PRESSURE_SCL_PIN < 0) return;
  Wire.begin(PRESSURE_SDA_PIN, PRESSURE_SCL_PIN);
  Wire.setClock(100000);
}

bool readJhm1200Pressure(float &barOut, float &tempOut) {
  if (!PRESSURE_I2C_ENABLED) return false;

  Wire.beginTransmission(PRESSURE_I2C_ADDR);
  Wire.write(0xAC);
  if (Wire.endTransmission() != 0) return false;
  delay(5);

  uint8_t buf[6] = {0};
  for (uint8_t tries = 0; tries < 20; tries++) {
    const uint8_t got = Wire.requestFrom((int)PRESSURE_I2C_ADDR, 6);
    if (got == 6) {
      for (uint8_t i = 0; i < 6; i++) buf[i] = Wire.read();
      if (((buf[0] >> 5) & 0x01) == 0) {
        const uint32_t pressRaw = ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
        const uint16_t tempRaw = ((uint16_t)buf[4] << 8) | buf[5];
        const float pressurePa = ((float)pressRaw / 16777216.0f) * PRESSURE_P_SPAN_PA + PRESSURE_P_OFFSET_PA;
        barOut = pressurePa / 100000.0f;
        tempOut = ((float)tempRaw / 65536.0f) * 190.0f - 40.0f;
        return isfinite(barOut);
      }
    }
    delay(2);
  }
  return false;
}

void updatePressureSensor() {
  if (!PRESSURE_I2C_ENABLED) return;
  const uint32_t now = millis();
  if (now - lastPressureReadMs < PRESSURE_READ_MS) return;
  lastPressureReadMs = now;

  float bar = NAN;
  float temp = NAN;
  pressureSensorOk = readJhm1200Pressure(bar, temp);
  if (pressureSensorOk) {
    setupCfg.pressureBar = bar;
    pressureSensorTempC = temp;
  }
}

void updateLightSensorAndBacklight() {
  int sum = 0;
  for (uint8_t i = 0; i < 8; i++) sum += analogRead(LIGHT_SENSOR_PIN);
  lightRaw = sum / 8;
  lightMinRaw = min(lightMinRaw, lightRaw);
  lightMaxRaw = max(lightMaxRaw, lightRaw);

  static int smoothRaw = -1;
  if (smoothRaw < 0) smoothRaw = lightRaw;
  else smoothRaw = (smoothRaw * 7 + lightRaw) / 8;

  int minBright = setupCfg.ldrMinDuty;
  int maxBright = setupCfg.ldrMaxDuty;
  if (minBright > maxBright) {
    int tmp = minBright;
    minBright = maxBright;
    maxBright = tmp;
  }
  int rawForMap = constrain(smoothRaw, min(LDR_DARK_RAW, LDR_BRIGHT_RAW), max(LDR_DARK_RAW, LDR_BRIGHT_RAW));
  uint8_t newDuty = constrain(map(rawForMap, LDR_DARK_RAW, LDR_BRIGHT_RAW, minBright, maxBright), minBright, maxBright);
  if (abs((int)newDuty - (int)backlightDuty) >= BACKLIGHT_HYSTERESIS) {
    backlightDuty = newDuty;
  }

  if (AUTO_BACKLIGHT) {
    ledcWrite(BACKLIGHT_PIN, backlightDuty);
  }

  static uint32_t lastDebugMs = 0;
  if (millis() - lastDebugMs >= 2000) {
    lastDebugMs = millis();
    Serial.printf("LDR raw=%d min=%d max=%d pwm=%u\n", lightRaw, lightMinRaw, lightMaxRaw, backlightDuty);
  }
}

String nextLogPath() {
  for (uint16_t i = 1; i <= 999; i++) {
    char path[32];
    snprintf(path, sizeof(path), "%s/HEW%03u.CSV", LOG_DIR, i);
    if (!SD.exists(path)) return String(path);
  }
  return String(LOG_DIR) + "/HEW999.CSV";
}

bool initLogger() {
  sdSpi.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  sdOK = SD.begin(SD_CS, sdSpi);
  if (!sdOK) {
    logFileName = "SD ERR";
    return false;
  }

  if (!SD.exists(LOG_DIR)) SD.mkdir(LOG_DIR);
  String path = nextLogPath();
  int slash = path.lastIndexOf('/');
  logFileName = (slash >= 0) ? path.substring(slash + 1) : path;
  logFile = SD.open(path, FILE_WRITE);
  if (!logFile) {
    logFileName = "LOG ERR";
    return false;
  }

  logOK = true;
  logFile.println("ms,type,yy,mo,dd,hh,mi,ss,t1,t2,t3,t4,statusRaw,statusCore,pumpC,heaterE,compressor,rx,ok,bad,lightRaw,pwm");
  logFile.flush();
  lastLogFlushMs = millis();
  return true;
}

void printCsvTemp(float value) {
  if (isnan(value)) logFile.print("");
  else logFile.print(value, 1);
}

void printCsvTempTo(File &f, float value) {
  if (isnan(value)) f.print("");
  else f.print(value, 1);
}

void logState(const char *type) {
  if (!logOK || !logFile || !st.valid) return;

  logFile.print(millis());
  logFile.print(',');
  logFile.print(type);
  logFile.print(',');
  logFile.print(st.yy);
  logFile.print(',');
  logFile.print(st.mo);
  logFile.print(',');
  logFile.print(st.dd);
  logFile.print(',');
  logFile.print(st.hh);
  logFile.print(',');
  logFile.print(st.mi);
  logFile.print(',');
  logFile.print(st.ss);
  logFile.print(',');
  printCsvTemp(st.t1);
  logFile.print(',');
  printCsvTemp(st.t2);
  logFile.print(',');
  printCsvTemp(st.t3);
  logFile.print(',');
  printCsvTemp(st.t4);
  logFile.print(",0x");
  if (st.status196Raw < 0x1000) logFile.print('0');
  if (st.status196Raw < 0x0100) logFile.print('0');
  if (st.status196Raw < 0x0010) logFile.print('0');
  logFile.print(st.status196Raw, HEX);
  logFile.print(",0x");
  if (st.status196Core < 0x1000) logFile.print('0');
  if (st.status196Core < 0x0100) logFile.print('0');
  if (st.status196Core < 0x0010) logFile.print('0');
  logFile.print(st.status196Core, HEX);
  logFile.print(',');
  logFile.print(st.pumpC ? 1 : 0);
  logFile.print(',');
  logFile.print(st.heaterE ? 1 : 0);
  logFile.print(',');
  logFile.print(st.compressor ? 1 : 0);
  logFile.print(',');
  logFile.print(st.rxBytes);
  logFile.print(',');
  logFile.print(st.tempFrames);
  logFile.print(',');
  logFile.print(st.badFrames);
  logFile.print(',');
  logFile.print(lightRaw);
  logFile.print(',');
  logFile.println(backlightDuty);

  lastLoggedState.valid = true;
  lastLoggedState.t1 = st.t1;
  lastLoggedState.t2 = st.t2;
  lastLoggedState.t3 = st.t3;
  lastLoggedState.t4 = st.t4;
  lastLoggedState.status196Raw = st.status196Raw;
  lastLoggedState.status196Core = st.status196Core;
  lastLoggedState.pumpC = st.pumpC;
  lastLoggedState.heaterE = st.heaterE;
  lastLoggedState.compressor = st.compressor;
  lastLoggedFrameCount = st.tempFrames;

  logFile.flush();
  lastLogFlushMs = millis();
}

bool tempChanged(float a, float b) {
  if (isnan(a) != isnan(b)) return true;
  if (isnan(a) && isnan(b)) return false;
  return fabsf(a - b) >= TEMP_LOG_DELTA;
}

bool stateChangedForLog() {
  if (!lastLoggedState.valid) return true;
  if (st.status196Core != lastLoggedState.status196Core) return true;
  if ((st.status196Raw & ~STATUS_DISPLAY_COUNTER_MASK) != (lastLoggedState.status196Raw & ~STATUS_DISPLAY_COUNTER_MASK)) return true;
  if (st.pumpC != lastLoggedState.pumpC) return true;
  if (st.heaterE != lastLoggedState.heaterE) return true;
  if (st.compressor != lastLoggedState.compressor) return true;
  if (tempChanged(st.t1, lastLoggedState.t1)) return true;
  if (tempChanged(st.t2, lastLoggedState.t2)) return true;
  if (tempChanged(st.t3, lastLoggedState.t3)) return true;
  if (tempChanged(st.t4, lastLoggedState.t4)) return true;
  return false;
}

void serviceLogger() {
  if (!logOK || !st.valid) return;

  const uint32_t now = millis();
  const bool newFrame = st.tempFrames != lastLoggedFrameCount;
  if (newFrame && now <= FULL_LOG_MS) {
    logState("FULL");
    return;
  }

  if (newFrame && stateChangedForLog()) {
    logState("CHANGE");
    return;
  }

  if (now - lastLogHeartbeatMs >= HEARTBEAT_MS) {
    lastLogHeartbeatMs = now;
    logState("HEARTBEAT");
  }
}

void writeProjectInfoFile() {
  if (!sdOK) return;

  if (SD.exists(PROJECT_INFO_PATH)) SD.remove(PROJECT_INFO_PATH);
  File f = SD.open(PROJECT_INFO_PATH, FILE_WRITE);
  if (!f) return;

  f.println("PROJECT=CYD_SOLAR_HEWALEX");
  f.print("CYD_APP=");
  f.println(APP_VERSION);
  f.print("CYD_BUILD=");
  f.println(BUILD_ID);
  f.println("CYD_BOARD=ESP32-2432S028 Cheap Yellow Display");
  f.println("DONGLE=ESP32-C3 OLED SuperMini");
  f.println("DONGLE_ROLE=USB Serial <-> ESP-NOW diagnostics bridge");
  f.println("SERIAL_BAUD=115200");
  f.println("ESP_NOW_CHANNEL=7");
  f.println("CYD_SOURCE=solar-controller/CYD_SOLAR_OTA");
  f.println("DONGLE_SOURCE=service-dongle/DongleESP_UPLOAD");
  f.println("ARCHIVE_ROOT=local-archive");
  f.println("CYD_REMOTE_COMMANDS=ver,status,alarm,alarm ack,log today,log last,log alarm,log energy,project,readme N,ota on,ota off,ota min N");
  f.println("DONGLE_COMMANDS=help,manifest,status,scan,ch 7,cyd status,cyd alarm,cyd log alarm,cyd log energy,cyd project,cyd readme N");
  f.println("LOGS=/HEWALEX/HEWxxx.CSV,/HEWALEX/ALARM.CSV,/HEWALEX/ENERGY.CSV");
  f.println("RULE_UPLOAD=Do not upload firmware unless user confirms immediately before upload.");
  f.println("NOTE=Use dongle serial first. Read manifest/help, then ask CYD with cyd status/cyd alarm/cyd project.");
  f.close();
}

bool summarizeProjectInfo(char *reply, size_t replyLen) {
  snprintf(reply, replyLen,
           "OK project name=CYD_SOLAR build=%s sd=%s cmds=readme N,status,alarm,log alarm,log energy ota=%u",
           BUILD_ID, PROJECT_INFO_PATH, otaActive ? 1 : 0);
  return true;
}

bool summarizeProjectReadme(uint8_t page, char *reply, size_t replyLen) {
  if (!sdOK) {
    snprintf(reply, replyLen, "ERR readme no_sd");
    return false;
  }
  if (!SD.exists(PROJECT_INFO_PATH)) writeProjectInfoFile();

  File f = SD.open(PROJECT_INFO_PATH, FILE_READ);
  if (!f) {
    snprintf(reply, replyLen, "ERR readme no_file");
    return false;
  }

  if (page < 1) page = 1;
  static constexpr size_t PAGE_TEXT_MAX = 112;
  char text[PAGE_TEXT_MAX + 1] = {0};
  char current[PAGE_TEXT_MAX + 1] = {0};
  size_t currentLen = 0;
  uint8_t currentPage = 1;
  uint8_t totalPages = 1;

  while (f.available()) {
    char line[96] = {0};
    size_t lineLen = 0;
    while (f.available()) {
      char c = (char)f.read();
      if (c == '\r') continue;
      if (c == '\n') break;
      if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
    }
    line[lineLen] = '\0';
    if (lineLen == 0) continue;

    const size_t needed = lineLen + (currentLen ? 2 : 0);
    if (currentLen && currentLen + needed > PAGE_TEXT_MAX) {
      if (currentPage == page) snprintf(text, sizeof(text), "%s", current);
      currentPage++;
      totalPages = currentPage;
      current[0] = '\0';
      currentLen = 0;
    }

    if (currentLen) {
      strncat(current, "; ", sizeof(current) - strlen(current) - 1);
      currentLen = strlen(current);
    }
    strncat(current, line, sizeof(current) - strlen(current) - 1);
    currentLen = strlen(current);
  }

  if (currentLen) {
    if (currentPage == page) snprintf(text, sizeof(text), "%s", current);
    totalPages = currentPage;
  }
  f.close();

  if (!text[0]) {
    snprintf(reply, replyLen, "ERR readme page=%u total=%u", page, totalPages);
    return false;
  }

  snprintf(reply, replyLen, "OK readme %u/%u %s", page, totalPages, text);
  return true;
}

bool summarizeHewLastDay(char *reply, size_t replyLen) {
  if (!sdOK) {
    snprintf(reply, replyLen, "ERR logday no_sd");
    return false;
  }
  if (logFile) logFile.flush();
  snprintf(reply, replyLen,
           "OK ld %02u-%02u-%02u f=%s fr=%lu bad=%lu kWh=%.3f max=%.1f p=%u pwm=%u al=%u",
           st.yy, st.mo, st.dd, logFileName.c_str(),
           (unsigned long)st.tempFrames, (unsigned long)st.badFrames,
           dailyEnergyKwh, dailyMaxSolarTemp, isSolarPumpActive() ? 1 : 0,
           pumpOutputPercent(), alarmActive ? 1 : 0);
  return true;
}

const char *alarmKindText(AlarmKind kind) {
  switch (kind) {
    case ALARM_TEMP: return "TEMP";
    case ALARM_SENSOR: return "SENSOR";
    case ALARM_PRESSURE: return "PRESS";
    case ALARM_NO_FLOW: return "NO_FLOW";
    case ALARM_FLOW_SENSOR: return "FLOW_SENS";
    default: return "NONE";
  }
}

const char *alarmSensorText(AlarmSensorTarget target) {
  switch (target) {
    case ALARM_SENSOR_RS: return "RS";
    case ALARM_SENSOR_SOL: return "SOL";
    case ALARM_SENSOR_WEJ: return "WEJ";
    case ALARM_SENSOR_WYJ: return "WYJ";
    default: return "--";
  }
}

bool alarmSourceStillFaulted() {
  if (!alarmActive) return false;
  if (alarmKind == ALARM_SENSOR) {
    if (alarmSensorTarget == ALARM_SENSOR_RS) return !isRsOnline();
    if (alarmSensorTarget == ALARM_SENSOR_SOL) return dsFault[0] || !isTempPlausible(solarCollectorTemp);
    if (alarmSensorTarget == ALARM_SENSOR_WEJ) return dsFault[1] || !isTempPlausible(solarTankInTemp);
    if (alarmSensorTarget == ALARM_SENSOR_WYJ) return dsFault[2] || !isTempPlausible(solarReturnTemp);
  }
  if (alarmKind == ALARM_TEMP) return safety.solarOverheat || safety.tankOverheat;
  if (alarmKind == ALARM_PRESSURE) {
    return setupCfg.pressureBar < setupCfg.pressureMinBar ||
           setupCfg.pressureBar > setupCfg.pressureMaxBar;
  }
  return false;
}

bool readLastNonHeaderLine(const char *path, char *line, size_t lineLen) {
  if (!sdOK || !path || !line || lineLen == 0) return false;
  line[0] = '\0';
  File f = SD.open(path, FILE_READ);
  if (!f) return false;

  const uint32_t fileSize = f.size();
  if (fileSize == 0) {
    f.close();
    return false;
  }

  static constexpr uint32_t LOG_TAIL_READ_MAX = 4096;
  const uint32_t start = fileSize > LOG_TAIL_READ_MAX ? fileSize - LOG_TAIL_READ_MAX : 0;
  f.seek(start);

  if (start > 0) {
    while (f.available()) {
      if ((char)f.read() == '\n') break;
    }
  }

  char current[192] = {0};
  size_t pos = 0;
  while (f.available()) {
    char c = (char)f.read();
    if (c == '\r') continue;
    if (c == '\n') {
      current[pos] = '\0';
      if (pos > 0 && strncmp(current, "ms,", 3) != 0) {
        snprintf(line, lineLen, "%s", current);
      }
      pos = 0;
      current[0] = '\0';
      continue;
    }
    if (pos < sizeof(current) - 1) current[pos++] = c;
  }
  if (pos > 0) {
    current[pos] = '\0';
    if (strncmp(current, "ms,", 3) != 0) snprintf(line, lineLen, "%s", current);
  }
  f.close();
  return line[0] != '\0';
}

void csvField(const char *line, uint8_t wanted, char *out, size_t outLen) {
  if (!out || outLen == 0) return;
  out[0] = '\0';
  if (!line) return;

  uint8_t field = 0;
  size_t pos = 0;
  for (const char *p = line; ; p++) {
    const char c = *p;
    if (c == ',' || c == '\0') {
      if (field == wanted) {
        out[pos] = '\0';
        return;
      }
      field++;
      pos = 0;
      if (c == '\0') return;
      continue;
    }
    if (field == wanted && pos < outLen - 1) out[pos++] = c;
  }
}

bool summarizeLastAlarmLog(char *reply, size_t replyLen) {
  char line[192];
  if (!readLastNonHeaderLine(ALARM_LOG_PATH, line, sizeof(line))) {
    snprintf(reply, replyLen, "OK alarm_last none");
    return true;
  }

  char kind[8], sensor[8], reason[40], yy[4], mo[4], dd[4], hh[4], mi[4], ss[4];
  csvField(line, 2, kind, sizeof(kind));
  csvField(line, 3, sensor, sizeof(sensor));
  csvField(line, 4, reason, sizeof(reason));
  csvField(line, 8, yy, sizeof(yy));
  csvField(line, 9, mo, sizeof(mo));
  csvField(line, 10, dd, sizeof(dd));
  csvField(line, 11, hh, sizeof(hh));
  csvField(line, 12, mi, sizeof(mi));
  csvField(line, 13, ss, sizeof(ss));
  snprintf(reply, replyLen, "OK alarm_last %s-%s-%s %s:%s:%s kind=%s sensor=%s %s",
           yy, mo, dd, hh, mi, ss, kind, sensor, reason);
  return true;
}

bool summarizeAlarmStatus(char *reply, size_t replyLen) {
  if (alarmActive) {
    snprintf(reply, replyLen, "OK alarm=1 review=%u kind=%s sensor=%s fault=%u reason=%s",
             alarmInReview ? 1 : 0,
             alarmKindText(alarmKind),
             alarmSensorText(alarmSensorTarget),
             alarmSourceStillFaulted() ? 1 : 0,
             alarmReason);
    return true;
  }
  return summarizeLastAlarmLog(reply, replyLen);
}

bool summarizeLastEnergyLog(char *reply, size_t replyLen) {
  char line[192];
  if (!readLastNonHeaderLine(ENERGY_LOG_PATH, line, sizeof(line))) {
    snprintf(reply, replyLen, "ERR energy no_data");
    return false;
  }

  char ev[16], yy[4], mo[4], dd[4], hh[4], mi[4], ss[4], kwh[16], kw[12], pump[4], pwm[8], sol[12], t2[12], t3[12];
  csvField(line, 1, ev, sizeof(ev));
  csvField(line, 2, yy, sizeof(yy));
  csvField(line, 3, mo, sizeof(mo));
  csvField(line, 4, dd, sizeof(dd));
  csvField(line, 5, hh, sizeof(hh));
  csvField(line, 6, mi, sizeof(mi));
  csvField(line, 7, ss, sizeof(ss));
  csvField(line, 8, kwh, sizeof(kwh));
  csvField(line, 9, kw, sizeof(kw));
  csvField(line, 10, pump, sizeof(pump));
  csvField(line, 11, pwm, sizeof(pwm));
  csvField(line, 14, sol, sizeof(sol));
  csvField(line, 15, t2, sizeof(t2));
  csvField(line, 16, t3, sizeof(t3));
  snprintf(reply, replyLen, "OK energy %s %s:%s:%s %s kWh=%s kW=%s p=%s pwm=%s sol=%s t2=%s t3=%s",
           dd, hh, mi, ss, ev, kwh, kw, pump, pwm, sol, t2, t3);
  return true;
}

bool summarizeLastHewLog(char *reply, size_t replyLen) {
  if (!sdOK || !logOK) {
    snprintf(reply, replyLen, "ERR hew no_log");
    return false;
  }
  if (logFile) logFile.flush();

  char path[48];
  snprintf(path, sizeof(path), "%s/%s", LOG_DIR, logFileName.c_str());
  char line[192];
  if (!readLastNonHeaderLine(path, line, sizeof(line))) {
    snprintf(reply, replyLen, "ERR hew no_data");
    return false;
  }

  char type[12], dd[4], hh[4], mi[4], ss[4], t1[12], t2[12], t3[12], t4[12], ok[10], bad[10], backlight[8];
  csvField(line, 1, type, sizeof(type));
  csvField(line, 4, dd, sizeof(dd));
  csvField(line, 5, hh, sizeof(hh));
  csvField(line, 6, mi, sizeof(mi));
  csvField(line, 7, ss, sizeof(ss));
  csvField(line, 8, t1, sizeof(t1));
  csvField(line, 9, t2, sizeof(t2));
  csvField(line, 10, t3, sizeof(t3));
  csvField(line, 11, t4, sizeof(t4));
  csvField(line, 18, ok, sizeof(ok));
  csvField(line, 19, bad, sizeof(bad));
  csvField(line, 21, backlight, sizeof(backlight));
  snprintf(reply, replyLen, "OK hew %s %s:%s:%s %s T=%s/%s/%s/%s ok=%s bad=%s bl=%s f=%s",
           dd, hh, mi, ss, type, t1, t2, t3, t4, ok, bad, backlight, logFileName.c_str());
  return true;
}

void IRAM_ATTR countFlowPulse() {
  uint32_t nowUs = micros();
  uint32_t deltaUs = nowUs - flowLastPulseUs;
  if (deltaUs >= FLOW_MIN_PULSE_US) {
    flowPulseCount++;
    flowLastPulseUs = nowUs;
  } else {
    flowRejectedCount++;
  }
}

uint8_t pwmPercentToDuty(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  return (uint8_t)roundf(percent * 255.0f / 100.0f);
}

static constexpr uint8_t PUMP_H_STOP_PWM = 97;
static constexpr uint8_t PUMP_H_HYST_PWM = 95;
static constexpr uint8_t PUMP_H_MIN_RAW_PWM = 84;
static constexpr uint8_t PUMP_H_MAX_RAW_PWM = 10;
static constexpr uint8_t PUMP_H_BOOST_RAW_PWM = 6;

struct PumpFlowPoint {
  float flow;
  uint8_t pwm;
};

static constexpr PumpFlowPoint PUMP_FLOW_POINTS[] = {
  {0.0f, PUMP_H_STOP_PWM},
  {2.0f, PUMP_H_MIN_RAW_PWM},
  {2.5f, 75},
  {3.0f, 55},
  {4.0f, 35},
  {4.5f, 20},
  {4.7f, PUMP_H_MAX_RAW_PWM},
};

bool isTempPlausible(float value) {
  if (isnan(value)) return false;
  if (value <= -126.0f) return false;       // DS18B20 disconnected
  if (fabsf(value - 85.0f) < 0.05f) return false;  // DS18B20 power-on default
  return value >= -35.0f && value <= 125.0f;
}

float maxPlausible3(float a, float b, float c) {
  float result = NAN;
  if (isTempPlausible(a)) result = a;
  if (isTempPlausible(b) && (isnan(result) || b > result)) result = b;
  if (isTempPlausible(c) && (isnan(result) || c > result)) result = c;
  return result;
}

uint8_t pumpMaxFlowPwmCommand() {
  return PUMP_H_BOOST_RAW_PWM;
}

uint8_t rawPwmForPumpOutput(uint8_t outputPercent) {
  outputPercent = constrain(outputPercent, 0, 100);
  if (outputPercent == 0) return PUMP_H_STOP_PWM;
  if (outputPercent >= 100) return PUMP_H_MAX_RAW_PWM;
  const float k = (outputPercent - 1) / 99.0f;
  const float raw = PUMP_H_MIN_RAW_PWM + (PUMP_H_MAX_RAW_PWM - PUMP_H_MIN_RAW_PWM) * k;
  return (uint8_t)constrain((int)roundf(raw), 0, 100);
}

uint8_t outputPercentForRawPwm(uint8_t rawPercent) {
  rawPercent = constrain(rawPercent, 0, 100);
  if (rawPercent >= PUMP_H_HYST_PWM) return 0;
  if (rawPercent >= PUMP_H_MIN_RAW_PWM) return 1;
  if (rawPercent <= PUMP_H_MAX_RAW_PWM) return 100;
  const float k = (PUMP_H_MIN_RAW_PWM - rawPercent) / (float)(PUMP_H_MIN_RAW_PWM - PUMP_H_MAX_RAW_PWM);
  return (uint8_t)constrain((int)roundf(1.0f + k * 99.0f), 1, 100);
}

uint8_t pumpOutputPercent() {
  return outputPercentForRawPwm(pumpPwmPercent);
}

uint8_t pumpRequestedOutputPercent() {
  return outputPercentForRawPwm(pumpPwmRequestedPercent);
}

void setPumpPwm(uint8_t percent);

void setPumpOutputPercent(uint8_t outputPercent) {
  setPumpPwm(rawPwmForPumpOutput(outputPercent));
}

bool isSolarPumpActive() {
  return pumpOutputPercent() > 0;
}

bool isSolarPumpUiActive() {
  const bool tubeBoostActive = tubeBoostUntilMs != 0 && (int32_t)(tubeBoostUntilMs - millis()) > 0;
  const bool noFlowRetryActive = noFlowTestUntilMs != 0 && (int32_t)(noFlowTestUntilMs - millis()) > 0;
  if (serviceModeActive) return pumpOutputPercent() > 0;
  if (setupCfg.pumpMode == PUMP_MODE_MANUAL_START) return pumpOutputPercent() > 0;
  if (safety.pumpMaxForced || tubeBoostActive || noFlowRetryActive) return true;
  return solarPumpAutoRunning;
}

uint8_t displayedPumpPercent() {
  return isSolarPumpUiActive() ? pumpOutputPercent() : 0;
}

bool isSolarDayWindow() {
  if (!ntpTimeOk) return true;
  return st.hh >= DAY_FALLBACK_START_HOUR && st.hh < DAY_FALLBACK_END_HOUR;
}

bool isRsOnline() {
  return st.valid && st.lastFrameMs != 0 && (millis() - st.lastFrameMs) < RS_ONLINE_TIMEOUT_MS;
}

void invalidateRsTempsIfOffline() {
  if (isRsOnline()) return;
  st.t1 = NAN;
  st.t2 = NAN;
  st.t3 = NAN;
  st.t4 = NAN;
}

float effectiveTankControlTemp() {
  if (isRsOnline() && isTempPlausible(st.t3)) return st.t3;
  if (isTempPlausible(solarReturnTemp)) return solarReturnTemp;
  return NAN;
}

void writePumpPwmRaw(uint8_t percent) {
  pumpPwmPercent = constrain(percent, 0, 100);
  ledcWrite(PUMP_PWM_PIN, pwmPercentToDuty(pumpPwmPercent));
}

void applyPumpPwmWithSafety() {
  uint8_t applied = pumpPwmRequestedPercent;
  if (safety.pumpMaxForced) {
    applied = pumpMaxFlowPwmCommand();
  }
  writePumpPwmRaw(applied);
}

void setPumpPwm(uint8_t percent) {
  pumpPwmRequestedPercent = constrain(percent, 0, 100);
  pumpTargetFlowLMin = NAN;
  applyPumpPwmWithSafety();
}

uint8_t pwmForTargetFlow(float flowLMin) {
  if (flowLMin <= PUMP_FLOW_POINTS[0].flow) return PUMP_FLOW_POINTS[0].pwm;
  const size_t n = sizeof(PUMP_FLOW_POINTS) / sizeof(PUMP_FLOW_POINTS[0]);
  for (size_t i = 1; i < n; i++) {
    if (flowLMin <= PUMP_FLOW_POINTS[i].flow) {
      const float span = PUMP_FLOW_POINTS[i].flow - PUMP_FLOW_POINTS[i - 1].flow;
      const float k = span > 0 ? (flowLMin - PUMP_FLOW_POINTS[i - 1].flow) / span : 0.0f;
      const float pwm = PUMP_FLOW_POINTS[i - 1].pwm +
                        (PUMP_FLOW_POINTS[i].pwm - PUMP_FLOW_POINTS[i - 1].pwm) * k;
      return (uint8_t)constrain((int)roundf(pwm), 0, 100);
    }
  }
  return PUMP_FLOW_POINTS[n - 1].pwm;
}

void setPumpFlow(float flowLMin) {
  pumpTargetFlowLMin = constrain(flowLMin, 0.0f, 4.7f);
  pumpPwmRequestedPercent = pwmForTargetFlow(pumpTargetFlowLMin);
  applyPumpPwmWithSafety();
}

void stopPump() {
  pumpTargetFlowLMin = 0.0f;
  pumpDtOutputPercent = 0;
  lastPumpDtControlMs = 0;
  pumpPwmRequestedPercent = PUMP_H_STOP_PWM;
  applyPumpPwmWithSafety();
}

void setPumpModeAuto() {
  setupCfg.pumpMode = PUMP_MODE_AUTO;
  solarPumpAutoRunning = false;
  tubeBoostUntilMs = 0;
  noFlowTestUntilMs = 0;
  resetPumpDtController(FLOW_SENSOR_TEST_OUTPUT_PCT);
}

void setPumpModeManualStart() {
  setupCfg.pumpMode = PUMP_MODE_MANUAL_START;
  solarPumpAutoRunning = true;
  tubeBoostUntilMs = 0;
  noFlowTestUntilMs = 0;
  resetPumpDtController(setupCfg.manualPwmPercent);
  setPumpOutputPercent(setupCfg.manualPwmPercent);
}

void setPumpModeStop() {
  setupCfg.pumpMode = PUMP_MODE_STOP;
  solarPumpAutoRunning = false;
  tubeBoostUntilMs = 0;
  noFlowTestUntilMs = 0;
  stopPump();
}

void writeHeaterRelay(bool on) {
  if (HEATER_RELAY_PIN < 0) return;
  digitalWrite(HEATER_RELAY_PIN, on == HEATER_RELAY_ACTIVE_HIGH ? HIGH : LOW);
}

void setupHeaterRelay() {
  if (HEATER_RELAY_PIN < 0) return;
  pinMode(HEATER_RELAY_PIN, OUTPUT);
  writeHeaterRelay(false);  // po restarcie zawsze fizycznie OFF
}

void applyHeaterOutput() {
  const bool tempOk = isTempPlausible(st.t2);
  if (!setupCfg.heaterEnabled || safety.heaterBlocked || !tempOk) {
    heaterOutputOn = false;
    writeHeaterRelay(false);
    return;
  }

  const float onAt = setupCfg.heaterSetTemp - 2.0f;
  const float offAt = setupCfg.heaterSetTemp;
  if (heaterOutputOn) {
    if (st.t2 >= offAt) heaterOutputOn = false;
  } else {
    if (st.t2 <= onAt) heaterOutputOn = true;
  }
  writeHeaterRelay(heaterOutputOn);
}

void captureFlowCheckBaseline() {
  flowCheckBaseSol = solarCollectorTemp;
  flowCheckBaseWej = solarTankInTemp;
  flowCheckBaseWyj = solarReturnTemp;
  flowCheckBaseT1 = st.t1;
  flowCheckBaseT3 = st.t3;
}

bool tempMovedEnough(float before, float now, float threshold = FLOW_TEMP_EVIDENCE_DELTA_C) {
  return isTempPlausible(before) && isTempPlausible(now) && fabsf(now - before) >= threshold;
}

bool hasTemperatureFlowEvidence() {
  if (tempMovedEnough(flowCheckBaseWej, solarTankInTemp)) return true;
  if (tempMovedEnough(flowCheckBaseWyj, solarReturnTemp)) return true;

  if (isTempPlausible(flowCheckBaseWej) && isTempPlausible(flowCheckBaseWyj) &&
      isTempPlausible(solarTankInTemp) && isTempPlausible(solarReturnTemp)) {
    const float baseDelta = flowCheckBaseWej - flowCheckBaseWyj;
    const float nowDelta = solarTankInTemp - solarReturnTemp;
    return fabsf(nowDelta - baseDelta) >= FLOW_TEMP_EVIDENCE_DELTA_C;
  }
  return false;
}

bool hasCollectorBlockEvidence() {
  return isTempPlausible(solarCollectorTemp) && isTempPlausible(st.t1) &&
         (solarCollectorTemp - st.t1) >= FLOW_SOL_T1_BLOCK_DELTA_C;
}

bool pressureOutOfRange() {
  return setupCfg.pressureBar < setupCfg.pressureMinBar ||
         setupCfg.pressureBar > setupCfg.pressureMaxBar;
}

void printStateFloat(File &f, float value, uint8_t decimals = 3) {
  if (isnan(value)) f.println("nan");
  else f.println(value, decimals);
}

float parseStateFloat(const String &line) {
  String s = line;
  s.trim();
  s.toLowerCase();
  if (s.length() == 0 || s == "nan") return NAN;
  return s.toFloat();
}

void startNoFlowTest(const char *note) {
  const uint32_t now = millis();
  noFlowTestUntilMs = now + PUMP_START_BOOST_MS;
  lastNoFlowTestMs = now;
  captureFlowCheckBaseline();
  calPhase = "NOFLOW";
  calNote = note;
  writeCalLog("NOFLOW", calNote);
}

float flowForPwm(uint8_t pwm) {
  const size_t n = sizeof(PUMP_FLOW_POINTS) / sizeof(PUMP_FLOW_POINTS[0]);
  if (pwm >= PUMP_FLOW_POINTS[0].pwm) return PUMP_FLOW_POINTS[0].flow;
  for (size_t i = 1; i < n; i++) {
    if (pwm >= PUMP_FLOW_POINTS[i].pwm) {
      const float span = PUMP_FLOW_POINTS[i - 1].pwm - PUMP_FLOW_POINTS[i].pwm;
      const float k = span > 0 ? (PUMP_FLOW_POINTS[i - 1].pwm - pwm) / span : 0.0f;
      return PUMP_FLOW_POINTS[i - 1].flow +
             (PUMP_FLOW_POINTS[i].flow - PUMP_FLOW_POINTS[i - 1].flow) * k;
    }
  }
  return PUMP_FLOW_POINTS[n - 1].flow;
}

void setPumpRegulatedOutput(uint8_t outputPercent) {
  setPumpOutputPercent(outputPercent);
  pumpTargetFlowLMin = flowForPwm(pumpPwmPercent);
}

void resetPumpDtController(uint8_t startOutputPercent) {
  const uint8_t minOut = constrain(setupCfg.pwmMin, 1, 95);
  const uint8_t maxOut = constrain(setupCfg.pwmMax, minOut, 100);
  pumpDtOutputPercent = constrain(startOutputPercent, minOut, maxOut);
  lastPumpDtControlMs = 0;
}

void regulatePumpToTargetDeltaT() {
  const uint32_t now = millis();
  const uint8_t minOut = constrain(setupCfg.pwmMin, 1, 95);
  const uint8_t maxOut = constrain(setupCfg.pwmMax, minOut, 100);

  if (pumpDtOutputPercent < minOut || pumpDtOutputPercent > maxOut) {
    uint8_t currentOut = pumpOutputPercent();
    if (currentOut == 0) currentOut = max((uint8_t)FLOW_SENSOR_TEST_OUTPUT_PCT, minOut);
    pumpDtOutputPercent = constrain(currentOut, minOut, maxOut);
  }

  if (isTempPlausible(deltaT) && (lastPumpDtControlMs == 0 || now - lastPumpDtControlMs >= PUMP_DT_CONTROL_MS)) {
    lastPumpDtControlMs = now;
    if (deltaT > setupCfg.targetDeltaT + PUMP_DT_DEADBAND_C) {
      pumpDtOutputPercent = constrain((int)pumpDtOutputPercent + PUMP_DT_STEP_PCT, minOut, maxOut);
    } else if (deltaT < setupCfg.targetDeltaT - PUMP_DT_DEADBAND_C) {
      pumpDtOutputPercent = constrain((int)pumpDtOutputPercent - PUMP_DT_STEP_PCT, minOut, maxOut);
    }
  }

  setPumpRegulatedOutput(pumpDtOutputPercent);
}

void printDsAddress(const DeviceAddress address) {
  for (uint8_t i = 0; i < 8; i++) {
    if (address[i] < 16) Serial.print('0');
    Serial.print(address[i], HEX);
  }
}

void clearDsAddress(DeviceAddress address) {
  memset(address, 0, sizeof(DeviceAddress));
}

bool dsAddressEquals(const DeviceAddress a, const DeviceAddress b) {
  return memcmp(a, b, sizeof(DeviceAddress)) == 0;
}

void assignSolarDsTemps() {
  solarCollectorTemp = (dsSensorCount > 0 && !isnan(dsTemps[0])) ? dsTemps[0] + setupCfg.tempOffsetDs[0] : NAN;
  solarTankInTemp = (dsSensorCount > 1 && !isnan(dsTemps[1])) ? dsTemps[1] + setupCfg.tempOffsetDs[1] : NAN;
  solarReturnTemp = (dsSensorCount > 2 && !isnan(dsTemps[2])) ? dsTemps[2] + setupCfg.tempOffsetDs[2] : NAN;
}

bool dsAddressIsEmpty(const DeviceAddress address) {
  for (uint8_t i = 0; i < 8; i++) {
    if (address[i] != 0x00) return false;
  }
  return true;
}

bool anyStaticDsAddressConfigured() {
  for (uint8_t i = 0; i < DS18B20_MAX_SENSORS; i++) {
    if (!dsAddressIsEmpty(DS18B20_STATIC_ADDRESSES[i])) return true;
  }
  return false;
}

void copyDsAddress(DeviceAddress dst, const DeviceAddress src) {
  memcpy(dst, src, sizeof(DeviceAddress));
}

void defaultDetectedDsRoles() {
  for (uint8_t i = 0; i < DS18B20_MAX_SENSORS; i++) {
    dsDetectedRoles[i] = i;
  }
}

bool detectedDsRolesUnique() {
  for (uint8_t i = 0; i < dsDetectedCount; i++) {
    for (uint8_t j = i + 1; j < dsDetectedCount; j++) {
      if (dsDetectedRoles[i] == dsDetectedRoles[j]) return false;
    }
  }
  return true;
}

bool canSaveDetectedDsRoles() {
  return dsDetectedCount == DS18B20_MAX_SENSORS && detectedDsRolesUnique();
}

void setDsRoleStatus(const char *text) {
  snprintf(dsRoleStatus, sizeof(dsRoleStatus), "%s", text ? text : "");
  dsRoleStatusMs = millis();
}

void saveDsRolesToPrefs() {
  Preferences prefs;
  if (!prefs.begin(DS_PREFS_NS, false)) {
    setDsRoleStatus("BLAD ZAPISU");
    return;
  }
  for (uint8_t i = 0; i < DS18B20_MAX_SENSORS; i++) {
    prefs.putBytes(DS_PREF_KEYS[i], dsAddresses[i], sizeof(DeviceAddress));
  }
  prefs.end();
}

bool loadDsRolesFromPrefs() {
  Preferences prefs;
  if (!prefs.begin(DS_PREFS_NS, true)) return false;

  bool allValid = true;
  for (uint8_t i = 0; i < DS18B20_MAX_SENSORS; i++) {
    clearDsAddress(dsAddresses[i]);
    if (prefs.getBytesLength(DS_PREF_KEYS[i]) != sizeof(DeviceAddress)) {
      allValid = false;
      continue;
    }
    prefs.getBytes(DS_PREF_KEYS[i], dsAddresses[i], sizeof(DeviceAddress));
    if (dsAddressIsEmpty(dsAddresses[i])) allValid = false;
  }
  prefs.end();

  if (!allValid) {
    for (uint8_t i = 0; i < DS18B20_MAX_SENSORS; i++) clearDsAddress(dsAddresses[i]);
    return false;
  }

  dsSensorCount = DS18B20_MAX_SENSORS;
  const uint32_t now = millis();
  for (uint8_t i = 0; i < DS18B20_MAX_SENSORS; i++) {
    dsTemps[i] = NAN;
    dsLastGoodMs[i] = 0;
    dsFault[i] = false;
  }
  lastDs18ScanMs = now;
  return true;
}

void cycleDetectedRole(uint8_t index) {
  if (index >= dsDetectedCount) return;
  const uint8_t oldRole = dsDetectedRoles[index];
  const uint8_t newRole = (uint8_t)((oldRole + 1) % DS18B20_MAX_SENSORS);
  for (uint8_t i = 0; i < dsDetectedCount; i++) {
    if (i == index) continue;
    if (dsDetectedRoles[i] == newRole) {
      dsDetectedRoles[i] = oldRole;
      break;
    }
  }
  dsDetectedRoles[index] = newRole;
}

void commitDetectedRoles(bool persist) {
  if (!canSaveDetectedDsRoles()) {
    setDsRoleStatus("USTAW WSZYSTKIE ROLE");
    return;
  }

  for (uint8_t i = 0; i < DS18B20_MAX_SENSORS; i++) {
    clearDsAddress(dsAddresses[i]);
    dsTemps[i] = NAN;
    dsFault[i] = true;
    dsLastGoodMs[i] = 0;
  }

  const uint32_t now = millis();
  for (uint8_t detected = 0; detected < dsDetectedCount; detected++) {
    const uint8_t role = dsDetectedRoles[detected];
    copyDsAddress(dsAddresses[role], dsDetectedAddresses[detected]);
    dsTemps[role] = dsDetectedTemps[detected];
    dsFault[role] = isnan(dsDetectedTemps[detected]);
    dsLastGoodMs[role] = isnan(dsDetectedTemps[detected]) ? 0 : now;
  }

  dsSensorCount = DS18B20_MAX_SENSORS;
  assignSolarDsTemps();
  if (persist) saveDsRolesToPrefs();
  setDsRoleStatus("ROLE DS ZAPISANE");
}

void beginDs18b20Bus() {
  pinMode(DS18B20_PIN, INPUT_PULLUP);
  dsSensors.begin();
  dsSensors.setResolution(12);
  dsSensors.setWaitForConversion(false);
}

void loadStaticDs18b20Addresses() {
  beginDs18b20Bus();

  dsSensorCount = 0;
  const uint32_t now = millis();
  for (uint8_t i = 0; i < DS18B20_MAX_SENSORS; i++) {
    memset(dsAddresses[i], 0, sizeof(DeviceAddress));
    dsTemps[i] = NAN;
    dsLastGoodMs[i] = now;
    dsFault[i] = false;

    if (!dsAddressIsEmpty(DS18B20_STATIC_ADDRESSES[i])) {
      copyDsAddress(dsAddresses[i], DS18B20_STATIC_ADDRESSES[i]);
      dsSensorCount = i + 1;
    }
  }

  Serial.printf("DS18B20 static GPIO%d count=%u idle=%s\n",
                DS18B20_PIN,
                dsSensorCount,
                digitalRead(DS18B20_PIN) ? "HIGH" : "LOW");
  dsSensors.requestTemperatures();
  lastDs18ReadMs = now;
  lastDs18ScanMs = now;
  assignSolarDsTemps();
}

// Reczny skan DS18B20: uzywany z SETUP/Serial/ESP-NOW/Telegram albo jako kompatybilny start,
// gdy stale adresy nie sa jeszcze wpisane w kodzie.
bool initDs18b20() {
  beginDs18b20Bus();
  defaultDetectedDsRoles();
  dsDetectedCount = 0;
  for (uint8_t i = 0; i < DS18B20_MAX_SENSORS; i++) {
    clearDsAddress(dsDetectedAddresses[i]);
    dsDetectedTemps[i] = NAN;
  }

  const int found = dsSensors.getDeviceCount();
  Serial.printf("DS18B20 manual scan GPIO%d found=%d idle=%s\n",
                DS18B20_PIN,
                found,
                digitalRead(DS18B20_PIN) ? "HIGH" : "LOW");

  if (found <= 0) {
    Serial.println("DS18B20 manual scan: brak czujnikow, zostawiam biezace role");
    dsSensors.requestTemperatures();
    lastDs18ReadMs = millis();
    lastDs18ScanMs = lastDs18ReadMs;
    setDsRoleStatus("SCAN: BRAK DS");
    return false;
  }

  dsSensors.setWaitForConversion(true);
  dsSensors.requestTemperatures();
  const uint8_t newCount = (uint8_t)constrain(found, 0, (int)DS18B20_MAX_SENSORS);
  dsDetectedCount = newCount;
  const uint32_t now = millis();

  for (uint8_t i = 0; i < DS18B20_MAX_SENSORS; i++) {
    if (i < dsDetectedCount && dsSensors.getAddress(dsDetectedAddresses[i], i)) {
      float t = dsSensors.getTempC(dsDetectedAddresses[i]);
      dsDetectedTemps[i] = (t == DEVICE_DISCONNECTED_C || !isTempPlausible(t)) ? NAN : t;
      Serial.printf("DS18B20[%u] address=", i);
      printDsAddress(dsDetectedAddresses[i]);
      Serial.println();
    } else {
      clearDsAddress(dsDetectedAddresses[i]);
      dsDetectedTemps[i] = NAN;
      if (i < dsDetectedCount) {
        Serial.printf("DS18B20[%u] address read failed\n", i);
      }
    }
  }

  dsSensors.setWaitForConversion(false);
  dsSensors.requestTemperatures();
  lastDs18ReadMs = now;
  lastDs18ScanMs = now;
  setDsRoleStatus(dsDetectedCount == DS18B20_MAX_SENSORS ? "SCAN OK: USTAW ROLE" : "SCAN NIEPELNY");
  return dsDetectedCount > 0;
}

void setupDs18b20() {
  if (DS18B20_USE_STATIC_ADDRESSES && anyStaticDsAddressConfigured()) {
    loadStaticDs18b20Addresses();
  } else if (loadDsRolesFromPrefs()) {
    Serial.println("DS18B20: wczytano zapisane role SOL/WEJ/WYJ");
    beginDs18b20Bus();
    dsSensors.requestTemperatures();
    lastDs18ReadMs = millis();
    initDs18b20();
  } else {
    Serial.println("DS18B20: brak zapisanych rol - wykonuje skan startowy");
    if (initDs18b20() && dsDetectedCount == DS18B20_MAX_SENSORS) {
      commitDetectedRoles(true);
      Serial.println("DS18B20: domyslnie przypisano kolejnosc SOL/WEJ/WYJ");
      beginDs18b20Bus();
      dsSensors.requestTemperatures();
      lastDs18ReadMs = millis();
    } else {
      dsSensorCount = 0;
      assignSolarDsTemps();
    }
  }
}

void printDs18b20Status() {
  Serial.printf("DS STATUS pin=%d idle=%s count=%u kol=%.2f s_in=%.2f s_rt=%.2f fault=%d/%d/%d\n",
                DS18B20_PIN,
                digitalRead(DS18B20_PIN) ? "HIGH" : "LOW",
                dsSensorCount,
                isnan(solarCollectorTemp) ? -999.0f : solarCollectorTemp,
                isnan(solarTankInTemp) ? -999.0f : solarTankInTemp,
                isnan(solarReturnTemp) ? -999.0f : solarReturnTemp,
                dsFault[0] ? 1 : 0,
                dsFault[1] ? 1 : 0,
                dsFault[2] ? 1 : 0);
  for (uint8_t i = 0; i < dsSensorCount; i++) {
    Serial.printf("DS[%u] addr=", i);
    printDsAddress(dsAddresses[i]);
    Serial.printf(" temp=%.2f fault=%d age=%lus\n",
                  isnan(dsTemps[i]) ? -999.0f : dsTemps[i],
                  dsFault[i] ? 1 : 0,
                  dsLastGoodMs[i] ? (unsigned long)((millis() - dsLastGoodMs[i]) / 1000UL) : 9999UL);
  }
}

void scanDs18b20Pin(int pin) {
  pinMode(pin, INPUT_PULLUP);
  OneWire ow(pin);
  DallasTemperature ds(&ow);
  ds.begin();
  ds.setResolution(12);
  ds.setWaitForConversion(true);
  const int found = ds.getDeviceCount();

  Serial.printf("DS SCAN GPIO%d idle=%s found=%d\n",
                pin,
                digitalRead(pin) ? "HIGH" : "LOW",
                found);

  DeviceAddress address;
  const int used = constrain(found, 0, (int)DS18B20_MAX_SENSORS);
  if (used > 0) ds.requestTemperatures();
  for (int i = 0; i < used; i++) {
    if (!ds.getAddress(address, i)) {
      Serial.printf("  [%d] address read failed\n", i);
      continue;
    }
    Serial.printf("  [%d] addr=", i);
    printDsAddress(address);
    float t = ds.getTempC(address);
    Serial.printf(" temp=%.2f\n", t == DEVICE_DISCONNECTED_C ? -999.0f : t);
  }
}

void updateDs18b20() {
  uint32_t now = millis();

  // Brak automatycznego skanowania adresow w loop().
  // Adresy sa stale albo aktualizowane recznie przez SETUP/komendy.
  if (dsSensorCount == 0) return;

  if (now - lastDs18ReadMs < DS18B20_READ_MS) return;
  lastDs18ReadMs = now;

  for (uint8_t i = 0; i < dsSensorCount; i++) {
    float t = dsSensors.getTempC(dsAddresses[i]);
    if (t == DEVICE_DISCONNECTED_C || !isTempPlausible(t)) {
      // Nie kasujemy ostatniej dobrej temperatury przy pojedynczym bledzie.
      if (dsLastGoodMs[i] == 0 || now - dsLastGoodMs[i] >= DS18B20_FAIL_TIMEOUT_MS) {
        dsFault[i] = true;
        dsTemps[i] = NAN;
      }
      continue;
    }

    dsTemps[i] = t;
    dsLastGoodMs[i] = now;
    dsFault[i] = false;
  }

  assignSolarDsTemps();
  dsSensors.requestTemperatures();
}

float effectiveFlowForPower() {
  if (!isnan(pumpTargetFlowLMin)) return pumpTargetFlowLMin;
  if (pumpOutputPercent() > 0) return flowForPwm(pumpPwmPercent);
  return 0.0f;
}

void updateThermalPower() {
  deltaT = NAN;
  powerKW = NAN;
  solarDeltaToT3 = NAN;
  controlDeltaT = NAN;
  solarDeltaToT3 = (!isnan(solarCollectorTemp) && !isnan(st.t3)) ? solarCollectorTemp - st.t3 : NAN;
  if (isTempPlausible(solarCollectorTemp)) {
    if (isRsOnline() && isTempPlausible(st.t3)) controlDeltaT = solarCollectorTemp - st.t3;
    else if (isTempPlausible(solarReturnTemp)) controlDeltaT = solarCollectorTemp - solarReturnTemp;
  }
  if (isTempPlausible(solarTankInTemp) && isTempPlausible(solarReturnTemp)) {
    deltaT = solarTankInTemp - solarReturnTemp;
  }
  float flowForPower = effectiveFlowForPower();
  if (isnan(flowForPower) || isnan(deltaT)) return;

  powerKW = flowForPower * deltaT * 0.0566f;
  if (powerKW < 0.0f) powerKW = 0.0f;
}

void serviceSolarPumpAutomation() {
  if (DEMO_MODE) return;
  if (serviceModeActive) return;

  const uint32_t now = millis();
  const bool dayWindow = isSolarDayWindow();
  const bool rsControlOk = isRsOnline() && isTempPlausible(st.t3);
  const bool rsFallbackArmed = (st.lastFrameMs != 0) || (now >= RS_ONLINE_TIMEOUT_MS);
  const float tankControlTemp = effectiveTankControlTemp();
  const bool tankControlOk = isTempPlausible(tankControlTemp);
  const bool solarOk = isTempPlausible(solarCollectorTemp);
  const bool wyjOk = isTempPlausible(solarReturnTemp);
  const float fallbackDelta = (solarOk && wyjOk) ? (solarCollectorTemp - solarReturnTemp) : NAN;

  if (!FLOWMETER_FAULTS_ENABLED) {
    noFlowFault = false;
    flowSensorFault = false;
    noFlowTestUntilMs = 0;
  }

  // Tryb MANUAL START musi miec pierwszenstwo przed warunkami automatyki.
  // Wczesniej kod najpierw sprawdzal !dayWindow / !tankControlOk i wykonywal stopPump(),
  // przez co reczny START w SETUP natychmiast wracal do STOP/0% na ekranie roboczym.
  if (setupCfg.pumpMode == PUMP_MODE_MANUAL_START) {
    solarPumpAutoRunning = true;
    tubeBoostUntilMs = 0;
    noFlowTestUntilMs = 0;
    applyPumpPwmWithSafety();
    return;
  }

  if (setupCfg.pumpMode == PUMP_MODE_STOP || !dayWindow || !tankControlOk) {
    solarPumpAutoRunning = false;
    tubeBoostUntilMs = 0;
    stopPump();
    return;
  }

  if (!rsControlOk && !rsFallbackArmed) {
    solarPumpAutoRunning = false;
    tubeBoostUntilMs = 0;
    stopPump();
    return;
  }

  if (FLOWMETER_FAULTS_ENABLED && FLOWMETER_INTERLOCK_ENABLED && noFlowFault) {
    if (flowPresent || hasTemperatureFlowEvidence()) {
      noFlowFault = false;
      if (!flowPresent) flowSensorFault = true;
      calPhase = "NOFLOW";
      calNote = flowPresent ? "flow restored" : "temp movement, flowmeter suspect";
      writeCalLog("NOFLOW", calNote);
    } else {
      const bool retryActive = noFlowTestUntilMs != 0 && (int32_t)(noFlowTestUntilMs - now) > 0;
      if (retryActive) {
        setPumpPwm(pumpMaxFlowPwmCommand());
      } else {
        if (noFlowTestUntilMs != 0 && hasTemperatureFlowEvidence() && !hasCollectorBlockEvidence()) {
          noFlowTestUntilMs = 0;
          noFlowFault = false;
          flowSensorFault = true;
          calPhase = "FLOWMETER";
          calNote = "no pulses but temp moved";
          writeCalLog("FLOWMETER", calNote);
          return;
        }
        noFlowTestUntilMs = 0;
        if (lastNoFlowTestMs == 0 || now - lastNoFlowTestMs >= NO_FLOW_RETRY_MS) {
          startNoFlowTest("retry max pump");
          setPumpPwm(pumpMaxFlowPwmCommand());
        } else {
          stopPump();
        }
      }
      return;
    }
  }

  const bool startByCollector = rsControlOk && solarOk && ((solarCollectorTemp - tankControlTemp) >= setupCfg.deltaStart);
  const bool startByPipe = !rsControlOk && rsFallbackArmed && !isnan(fallbackDelta) && (fallbackDelta >= setupCfg.deltaStart);
  const bool stopByCollector = rsControlOk ? (!solarOk || ((solarCollectorTemp - tankControlTemp) <= setupCfg.deltaStop)) : true;
  const bool stopByPipe = (!rsControlOk && rsFallbackArmed) ? (isnan(fallbackDelta) || (fallbackDelta <= setupCfg.deltaStop)) : true;
  const bool shouldStart = startByCollector || startByPipe;
  const bool shouldStop = rsControlOk ? stopByCollector : stopByPipe;

  const uint32_t tubeReferenceMs = lastTubeFlowConfirmMs ? lastTubeFlowConfirmMs : lastTubeTestMs;
  const bool tubeRefreshDue = tubeReferenceMs == 0 || (now - tubeReferenceMs >= TUBE_COLLECTOR_REFRESH_MS);
  if (dayWindow && tubeRefreshDue && tubeBoostUntilMs == 0 && !solarPumpAutoRunning) {
    tubeBoostUntilMs = now + PUMP_START_BOOST_MS;
    lastTubeTestMs = now;
    captureFlowCheckBaseline();
    calPhase = "TUBE";
    calNote = "tube collector refresh";
  }

  if (shouldStart && !solarPumpAutoRunning) {
    solarPumpAutoRunning = true;
    tubeBoostUntilMs = now + PUMP_START_BOOST_MS;
    lastTubeTestMs = now;
    resetPumpDtController(FLOW_SENSOR_TEST_OUTPUT_PCT);
    captureFlowCheckBaseline();
    calPhase = "AUTO";
    calNote = "solar delta start";
  }

  if (tubeBoostUntilMs != 0 && (int32_t)(tubeBoostUntilMs - now) > 0) {
    setPumpOutputPercent(FLOW_SENSOR_TEST_OUTPUT_PCT);
    return;
  }
  if (tubeBoostUntilMs != 0 && lastTubeTestMs != 0 && lastTubeFlowConfirmMs < lastTubeTestMs) {
    tubeBoostUntilMs = 0;
    if (FLOWMETER_FAULTS_ENABLED) {
      if (hasTemperatureFlowEvidence() && !hasCollectorBlockEvidence()) {
        flowSensorFault = true;
        calPhase = "FLOWMETER";
        calNote = "no pulses but temp moved";
        writeCalLog("FLOWMETER", calNote);
      } else {
        // Przeplywomierz jest nadal w diagnostyce: sygnalizuj brak impulsow,
        // ale nie blokuj automatyki opartej na temperaturach.
        flowSensorFault = true;
        noFlowFault = false;
        calPhase = "FLOWMETER";
        calNote = "no pulses, interlock suspended";
        writeCalLog("FLOWMETER", calNote);
      }
    }
  }
  tubeBoostUntilMs = 0;

  if (solarPumpAutoRunning && shouldStop) {
    solarPumpAutoRunning = false;
    calPhase = "AUTO";
    calNote = "solar delta stop";
    stopPump();
    return;
  }

  if (solarPumpAutoRunning) {
    regulatePumpToTargetDeltaT();
  } else if (!shouldStart) {
    stopPump();
  }
}

void serviceSafety() {
  safety.hottestSolar = maxPlausible3(solarCollectorTemp, NAN, NAN);
  safety.hottestTank = maxPlausible3(st.t2, st.t3, solarTankInTemp);

  const bool solarSensorOk = isTempPlausible(safety.hottestSolar);
  const bool tankSensorOk = isTempPlausible(safety.hottestTank);
  safety.sensorFault = !solarSensorOk || !tankSensorOk;

  if (solarSensorOk && safety.hottestSolar >= setupCfg.solarForcePwmTemp) {
    if (safety.solarOverheatCount < 3) safety.solarOverheatCount++;
  } else {
    safety.solarOverheatCount = 0;
  }

  if (tankSensorOk && safety.hottestTank >= setupCfg.tankHeaterOffTemp) {
    if (safety.tankOverheatCount < 3) safety.tankOverheatCount++;
  } else {
    safety.tankOverheatCount = 0;
  }

  safety.solarOverheat = safety.solarOverheatCount >= 2;
  safety.tankOverheat = safety.tankOverheatCount >= 2;
  safety.pumpMaxForced = safety.solarOverheat;
  safety.heaterBlocked = !setupCfg.heaterEnabled || safety.tankOverheat || !tankSensorOk;
  safety.dumpPumpRequest = safety.tankOverheat;
  applyHeaterOutput();

  if (safety.solarOverheat) {
    snprintf(safety.reason, sizeof(safety.reason), "SOLAR %.1fC", safety.hottestSolar);
  } else if (safety.tankOverheat) {
    snprintf(safety.reason, sizeof(safety.reason), "TANK %.1fC", safety.hottestTank);
  } else if (safety.sensorFault) {
    snprintf(safety.reason, sizeof(safety.reason), "CZUJNIK");
  } else {
    snprintf(safety.reason, sizeof(safety.reason), "OK");
  }

  applyPumpPwmWithSafety();
}

String nextCalLogPath() {
  for (uint16_t i = 1; i <= 999; i++) {
    char path[32];
    snprintf(path, sizeof(path), "%s%03u.CSV", CAL_LOG_PREFIX, i);
    if (!SD.exists(path)) return String(path);
  }
  return String(CAL_LOG_PREFIX) + "999.CSV";
}

bool initCalLogger() {
  if (!sdOK) return false;
  if (!SD.exists(LOG_DIR)) SD.mkdir(LOG_DIR);

  String path = nextCalLogPath();
  int slash = path.lastIndexOf('/');
  calFileName = (slash >= 0) ? path.substring(slash + 1) : path;
  calFile = SD.open(path, FILE_WRITE);
  if (!calFile) {
    calFileName = "CAL ERR";
    return false;
  }

  calLogOK = true;
  calFile.println("ms,phase,pwm_pct,pwm_raw,pwm_out_pct,target_dt,control_delta_t,target_flow_l_min,delta_t,power_kw,flow_pulses,flow_hz,flow_hz_raw,flow_rejected,rotameter_l_min,delta_test_target,solar_delta_t3,rs_ok,t1,t2,t3,t4,ds_count,solar_collector,solar_tank_in,solar_return,ldr,backlight,sd_ok,note");
  calFile.flush();
  return true;
}

void resetDailyEnergy(uint8_t newDayKey) {
  dailyEnergyKwh = 0.0f;
  dailyMaxSolarTemp = NAN;
  energyDayKey = newDayKey;
  lastEnergyUpdateMs = millis();
  for (uint8_t i = 0; i < 25; i++) {
    energyHourSolar[i] = NAN;
    energyHourTank[i] = NAN;
    energyHourPower[i] = NAN;
    energyHourPump[i] = false;
  }
  for (uint8_t i = 0; i < ENERGY_PWM_BINS; i++) {
    energyPwmBins[i] = NAN;
    energyPwmSum[i] = 0;
    energyPwmCount[i] = 0;
  }
}

bool loadEnergyState() {
  if (!sdOK || !SD.exists(ENERGY_STATE_PATH)) return false;
  File f = SD.open(ENERGY_STATE_PATH, FILE_READ);
  if (!f) return false;

  const String head = f.readStringUntil('\n');
  if (!head.startsWith("day=")) {
    f.close();
    return false;
  }

  energyDayKey = (uint8_t)head.substring(4).toInt();
  dailyEnergyKwh = f.readStringUntil('\n').toFloat();
  dailyMaxSolarTemp = parseStateFloat(f.readStringUntil('\n'));

  for (uint8_t i = 0; i < 25; i++) {
    energyHourSolar[i] = parseStateFloat(f.readStringUntil('\n'));
  }
  for (uint8_t i = 0; i < 25; i++) {
    energyHourTank[i] = parseStateFloat(f.readStringUntil('\n'));
  }
  for (uint8_t i = 0; i < 25; i++) {
    energyHourPower[i] = parseStateFloat(f.readStringUntil('\n'));
  }
  for (uint8_t i = 0; i < 25; i++) {
    energyHourPump[i] = f.readStringUntil('\n').toInt() != 0;
  }
  for (uint8_t i = 0; i < ENERGY_PWM_BINS; i++) {
    energyPwmBins[i] = parseStateFloat(f.readStringUntil('\n'));
    if (!isnan(energyPwmBins[i])) {
      energyPwmSum[i] = (uint16_t)roundf(energyPwmBins[i]);
      energyPwmCount[i] = 1;
    } else {
      energyPwmSum[i] = 0;
      energyPwmCount[i] = 0;
    }
  }
  f.close();
  return true;
}

void saveEnergyState() {
  if (!sdOK) return;
  if (SD.exists(ENERGY_STATE_PATH)) SD.remove(ENERGY_STATE_PATH);
  File f = SD.open(ENERGY_STATE_PATH, FILE_WRITE);
  if (!f) return;
  f.print("day=");
  f.println(energyDayKey);
  f.println(dailyEnergyKwh, 5);
  printStateFloat(f, dailyMaxSolarTemp, 3);
  for (uint8_t i = 0; i < 25; i++) printStateFloat(f, energyHourSolar[i], 3);
  for (uint8_t i = 0; i < 25; i++) printStateFloat(f, energyHourTank[i], 3);
  for (uint8_t i = 0; i < 25; i++) printStateFloat(f, energyHourPower[i], 3);
  for (uint8_t i = 0; i < 25; i++) f.println(energyHourPump[i] ? 1 : 0);
  for (uint8_t i = 0; i < ENERGY_PWM_BINS; i++) printStateFloat(f, energyPwmBins[i], 3);
  f.close();
}

void serviceStateSync() {
  if (!sdOK) return;
  if (millis() - lastStateSyncMs < STATE_SYNC_INTERVAL_MS) return;
  lastStateSyncMs = millis();
  saveEnergyState();
}

bool initEnergyLogger() {
  if (!sdOK) return false;
  if (!SD.exists(LOG_DIR)) SD.mkdir(LOG_DIR);
  const bool exists = SD.exists(ENERGY_LOG_PATH);
  File f = SD.open(ENERGY_LOG_PATH, FILE_APPEND);
  if (!f) return false;
  if (!exists || f.size() == 0) {
    f.println("ms,event,yy,mo,dd,hh,mi,ss,kwh_day,power_kw,pump,pwm_pct,flow_l_min,flow_hz,solar_collector,tank_t2,t3,solar_in,solar_out");
  }
  f.close();
  energyLogOK = true;
  if (!loadEnergyState()) {
    resetDailyEnergy(st.dd);
  }
  writeEnergyLog("BOOT");
  return true;
}

void writeEnergyLog(const char *event) {
  if (!energyLogOK || !sdOK) return;
  File f = SD.open(ENERGY_LOG_PATH, FILE_APPEND);
  if (!f) return;
  f.print(millis());
  f.print(',');
  f.print(event ? event : "");
  f.print(',');
  f.print(st.yy);
  f.print(',');
  f.print(st.mo);
  f.print(',');
  f.print(st.dd);
  f.print(',');
  f.print(st.hh);
  f.print(',');
  f.print(st.mi);
  f.print(',');
  f.print(st.ss);
  f.print(',');
  f.print(dailyEnergyKwh, 4);
  f.print(',');
  if (!isnan(powerKW)) f.print(powerKW, 3);
  f.print(',');
  f.print(solarPumpAutoRunning || pumpPwmPercent < 100 ? 1 : 0);
  f.print(',');
  f.print(pumpPwmPercent);
  f.print(',');
  if (!isnan(pumpTargetFlowLMin)) f.print(pumpTargetFlowLMin, 2);
  f.print(',');
  f.print(flowHz, 2);
  f.print(',');
  printCsvTempTo(f, solarCollectorTemp);
  f.print(',');
  printCsvTempTo(f, st.t2);
  f.print(',');
  printCsvTempTo(f, st.t3);
  f.print(',');
  printCsvTempTo(f, solarTankInTemp);
  f.print(',');
  printCsvTempTo(f, solarReturnTemp);
  f.println();
  f.close();
}

void activateServiceMode(const char *reason) {
  serviceModeActive = true;
  serviceModeUntilMs = millis() + (uint32_t)setupCfg.manualPwmTimeoutSec * 1000UL;
  snprintf(serviceModeReason, sizeof(serviceModeReason), "%s", reason ? reason : "remote");
}

uint32_t serviceModeRemainingSec() {
  if (!serviceModeActive) return 0;
  const int32_t remainingMs = (int32_t)(serviceModeUntilMs - millis());
  if (remainingMs <= 0) return 0;
  return (uint32_t)((remainingMs + 999) / 1000);
}

void serviceRemoteServiceMode() {
  if (!serviceModeActive) return;
  if ((int32_t)(serviceModeUntilMs - millis()) > 0) return;
  serviceModeActive = false;
  serviceModeUntilMs = 0;
  stopPump();
  calPhase = "SERVICE";
  calNote = "service timeout, auto return";
  writeCalLog("SERVICE", calNote);
}

void serviceEnergyBalance() {
  const uint32_t now = millis();
  if (energyDayKey == 0 && st.dd != 0) resetDailyEnergy(st.dd);
  if (st.dd != 0 && energyDayKey != 0 && st.dd != energyDayKey) {
    writeEnergyLog("DAY_END");
    resetDailyEnergy(st.dd);
    writeEnergyLog("DAY_START");
  }

  if (lastEnergyUpdateMs == 0) lastEnergyUpdateMs = now;
  const uint32_t elapsed = now - lastEnergyUpdateMs;
  lastEnergyUpdateMs = now;

  if (!isnan(powerKW) && elapsed > 0 && elapsed < 10UL * 60UL * 1000UL) {
    dailyEnergyKwh += powerKW * ((float)elapsed / 3600000.0f);
  }
  if (isTempPlausible(solarCollectorTemp) &&
      (isnan(dailyMaxSolarTemp) || solarCollectorTemp > dailyMaxSolarTemp)) {
    dailyMaxSolarTemp = solarCollectorTemp;
  }

  const uint8_t hour = constrain(st.hh, 0, 23);
  if (isTempPlausible(solarCollectorTemp)) energyHourSolar[hour] = solarCollectorTemp;
  if (isTempPlausible(st.t2)) energyHourTank[hour] = st.t2;
  if (!isnan(powerKW)) energyHourPower[hour] = powerKW;
  if (displayedPumpPercent() > 0) energyHourPump[hour] = true;
  const uint16_t dayMinute = constrain((int)st.hh * 60 + (int)st.mi, 0, 1439);
  const uint8_t pwmBin = (uint8_t)min((uint16_t)(ENERGY_PWM_BINS - 1),
                                      (uint16_t)((uint32_t)dayMinute * ENERGY_PWM_BINS / 1440UL));
  static uint32_t lastPwmAvgSampleMs = 0;
  if (now - lastPwmAvgSampleMs >= 60UL * 1000UL || lastPwmAvgSampleMs == 0) {
    lastPwmAvgSampleMs = now;
    if (energyPwmCount[pwmBin] < 600) {
      energyPwmSum[pwmBin] += pumpOutputPercent();
      energyPwmCount[pwmBin]++;
    }
    energyPwmBins[pwmBin] = energyPwmCount[pwmBin] ? ((float)energyPwmSum[pwmBin] / energyPwmCount[pwmBin]) : NAN;
  }

  if (energyLogOK && now - lastEnergyLogMs >= ENERGY_LOG_INTERVAL_MS) {
    lastEnergyLogMs = now;
    writeEnergyLog("SAMPLE");
  }
}

void printCalTemp(float value) {
  if (isnan(value)) calFile.print("");
  else calFile.print(value, 1);
}

void writeCalLog(const char *phase, const String &note, float rotameterValue) {
  if (!calLogOK || !calFile || !calLogging) return;

  String cleanNote = note;
  cleanNote.replace(',', ';');

  calFile.print(millis());
  calFile.print(',');
  calFile.print(phase);
  calFile.print(',');
  calFile.print(pumpPwmPercent);
  calFile.print(',');
  calFile.print(pwmPercentToDuty(pumpPwmPercent));
  calFile.print(',');
  calFile.print(pumpOutputPercent());
  calFile.print(',');
  calFile.print(setupCfg.targetDeltaT, 1);
  calFile.print(',');
  if (!isnan(controlDeltaT)) calFile.print(controlDeltaT, 2);
  calFile.print(',');
  if (!isnan(pumpTargetFlowLMin)) calFile.print(pumpTargetFlowLMin, 2);
  calFile.print(',');
  if (!isnan(deltaT)) calFile.print(deltaT, 2);
  calFile.print(',');
  if (!isnan(powerKW)) calFile.print(powerKW, 3);
  calFile.print(',');
  calFile.print(flowPulsesLast);
  calFile.print(',');
  calFile.print(flowHz, 2);
  calFile.print(',');
  calFile.print(flowHzRaw, 2);
  calFile.print(',');
  calFile.print(flowRejectedLast);
  calFile.print(',');
  if (!isnan(rotameterValue)) calFile.print(rotameterValue, 2);
  calFile.print(',');
  calFile.print(deltaTestTargetC);
  calFile.print(',');
  if (!isnan(solarDeltaToT3)) calFile.print(solarDeltaToT3, 2);
  calFile.print(',');
  calFile.print(st.valid ? 1 : 0);
  calFile.print(',');
  printCalTemp(st.t1);
  calFile.print(',');
  printCalTemp(st.t2);
  calFile.print(',');
  printCalTemp(st.t3);
  calFile.print(',');
  printCalTemp(st.t4);
  calFile.print(',');
  calFile.print(dsSensorCount);
  calFile.print(',');
  printCalTemp(solarCollectorTemp);
  calFile.print(',');
  printCalTemp(solarTankInTemp);
  calFile.print(',');
  printCalTemp(solarReturnTemp);
  calFile.print(',');
  calFile.print(lightRaw);
  calFile.print(',');
  calFile.print(backlightDuty);
  calFile.print(',');
  calFile.print(sdOK ? 1 : 0);
  calFile.print(',');
  calFile.println(cleanNote);
  calFile.flush();
}

void updateFlowMeter() {
  uint32_t now = millis();
  if (now - lastFlowSampleMs < FLOW_SAMPLE_MS) return;

  noInterrupts();
  uint32_t count = flowPulseCount;
  uint32_t rejected = flowRejectedCount;
  flowPulseCount = 0;
  flowRejectedCount = 0;
  interrupts();

  uint32_t elapsed = now - lastFlowSampleMs;
  if (lastFlowSampleMs == 0 || elapsed == 0) elapsed = FLOW_SAMPLE_MS;
  lastFlowSampleMs = now;
  flowPulsesLast = count;
  flowRejectedLast = rejected;
  flowHzRaw = count * 1000.0f / elapsed;
  if (flowHz == 0.0f) flowHz = flowHzRaw;
  else flowHz = flowHz * (1.0f - FLOW_SMOOTH_ALPHA) + flowHzRaw * FLOW_SMOOTH_ALPHA;

  const bool sampleHasFlow = flowPulsesLast >= FLOW_PRESENT_MIN_PULSES && flowHzRaw >= FLOW_PRESENT_MIN_HZ;
  if (sampleHasFlow) {
    if (!flowPresent) flowPresentSinceMs = now;
    flowPresent = true;
    lastFlowSeenMs = now;
    lastTubeFlowConfirmMs = now;
    flowConfirmedRunMs += elapsed;
  } else {
    flowPresent = false;
    flowPresentSinceMs = 0;
  }
}

void serviceCalLogger() {
  if (!calLogOK || !calLogging) return;
  uint32_t now = millis();
  if (now - lastCalLogMs < CAL_LOG_INTERVAL_MS) return;
  lastCalLogMs = now;
  writeCalLog(calPhase.c_str(), calNote);
}

void markCalibration(float rotameterValue, const String &source) {
  rotameterLMin = rotameterValue;
  calPhase = "MARK";
  calNote = source;
  writeCalLog("MARK", source, rotameterValue);
  Serial.printf("MARK rotameter=%.2f pwm=%u flow_hz=%.2f pulses=%lu\n",
                rotameterValue,
                pumpPwmPercent,
                flowHz,
                (unsigned long)flowPulsesLast);
}

void printCalibrationStatus() {
  IPAddress ip = WiFi.localIP();
  Serial.printf("STATUS pwm=%u req=%u duty=%u safety=%s force=%d heater_block=%d dump_pump=%d target_flow=%.2f delta_t=%.2f power_kw=%.3f flow_pin=%d flow_hz=%.2f raw_hz=%.2f pulses=%lu rejected=%lu sd=%d cal=%d rs=%d log=%d ntp=%d wifi=%d rssi=%d ch=%d ip=%u.%u.%u.%u espnow_en=%d espnow_ready=%d espnow_ok=%d ota=%d srv=%d srv_rem=%lus srv_reason=%s file=%s t1=%.1f t2=%.1f t3=%.1f t4=%.1f ds=%u kol=%.1f s_in=%.1f s_rt=%.1f\n",
                pumpPwmPercent,
                pumpPwmRequestedPercent,
                pwmPercentToDuty(pumpPwmPercent),
                safety.reason,
                safety.pumpMaxForced ? 1 : 0,
                safety.heaterBlocked ? 1 : 0,
                safety.dumpPumpRequest ? 1 : 0,
                isnan(pumpTargetFlowLMin) ? -1.0f : pumpTargetFlowLMin,
                isnan(deltaT) ? -999.0f : deltaT,
                isnan(powerKW) ? -1.0f : powerKW,
                digitalRead(FLOW_PIN),
                flowHz,
                flowHzRaw,
                (unsigned long)flowPulsesLast,
                (unsigned long)flowRejectedLast,
                sdOK ? 1 : 0,
                calLogOK ? 1 : 0,
                st.valid ? 1 : 0,
                calLogging ? 1 : 0,
                ntpTimeOk ? 1 : 0,
                (int)WiFi.status(),
                WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0,
                WiFi.channel(),
                ip[0], ip[1], ip[2], ip[3],
                setupCfg.espNowEnabled ? 1 : 0,
                espNowReady ? 1 : 0,
                espNowLinkOk() ? 1 : 0,
                otaActive ? 1 : 0,
                serviceModeActive ? 1 : 0,
                (unsigned long)serviceModeRemainingSec(),
                serviceModeReason,
                calFileName.c_str(),
                st.t1,
                st.t2,
                st.t3,
                st.t4,
                dsSensorCount,
                isnan(solarCollectorTemp) ? -999.0f : solarCollectorTemp,
                isnan(solarTankInTemp) ? -999.0f : solarTankInTemp,
                isnan(solarReturnTemp) ? -999.0f : solarReturnTemp);
}

void printAppVersionStatus() {
  Serial.printf("VER app=%s build=%s\n", APP_VERSION, BUILD_ID);
}

bool handleSharedCommand(const String &line, const String &cmd, bool remote, char *reply, size_t replyLen) {
  const char *src = remote ? "espnow" : "serial";
  char reason[32];

  if (cmd.startsWith("pwm ")) {
    int value = constrain(line.substring(4).toInt(), 0, 100);
    snprintf(reason, sizeof(reason), "%s pwm", src);
    activateServiceMode(reason);
    setPumpOutputPercent((uint8_t)value);
    calPhase = "PWM";
    calNote = reason;
    writeCalLog("PWM", calNote);
    snprintf(reply, replyLen, "OK service pwm=%u raw=%u timeout=%us",
             pumpOutputPercent(), pumpPwmPercent, setupCfg.manualPwmTimeoutSec);
    return true;
  }

  if (cmd.startsWith("flow ")) {
    float value = line.substring(5).toFloat();
    snprintf(reason, sizeof(reason), "%s flow", src);
    activateServiceMode(reason);
    setPumpFlow(value);
    calPhase = "FLOW";
    calNote = reason;
    writeCalLog("FLOW", calNote, pumpTargetFlowLMin);
    snprintf(reply, replyLen, "OK service flow=%.2f pwm=%u raw=%u timeout=%us",
             pumpTargetFlowLMin, pumpOutputPercent(), pumpPwmPercent, setupCfg.manualPwmTimeoutSec);
    return true;
  }

  if (cmd == "stop" || cmd == "pwm0") {
    snprintf(reason, sizeof(reason), "%s stop", src);
    activateServiceMode(reason);
    stopPump();
    calPhase = "STOP";
    calNote = reason;
    writeCalLog("STOP", calNote);
    snprintf(reply, replyLen, "OK service stop timeout=%us", setupCfg.manualPwmTimeoutSec);
    return true;
  }

  if (cmd == "service" || cmd == "service status") {
    snprintf(reply, replyLen, "OK service=%d remaining=%lus timeout=%us reason=%s",
             serviceModeActive ? 1 : 0,
             (unsigned long)serviceModeRemainingSec(),
             setupCfg.manualPwmTimeoutSec,
             serviceModeReason);
    return true;
  }

  if (cmd == "service on" || cmd == "manual") {
    snprintf(reason, sizeof(reason), "%s service", src);
    activateServiceMode(reason);
    snprintf(reply, replyLen, "OK service on remaining=%lus timeout=%us",
             (unsigned long)serviceModeRemainingSec(),
             setupCfg.manualPwmTimeoutSec);
    return true;
  }

  if (cmd == "service off" || cmd == "auto") {
    serviceModeActive = false;
    serviceModeUntilMs = 0;
    calPhase = "SERVICE";
    calNote = remote ? "remote auto return" : "serial auto return";
    writeCalLog("SERVICE", calNote);
    snprintf(reply, replyLen, "OK service off, auto");
    return true;
  }

  if (cmd.startsWith("service timeout ")) {
    setupCfg.manualPwmTimeoutSec = constrain(line.substring(16).toInt(), 5, 3600);
    snprintf(reason, sizeof(reason), "%s service timeout", src);
    activateServiceMode(reason);
    snprintf(reply, replyLen, "OK service timeout=%us remaining=%lus",
             setupCfg.manualPwmTimeoutSec,
             (unsigned long)serviceModeRemainingSec());
    return true;
  }

  if (cmd.startsWith("mark ")) {
    float value = line.substring(5).toFloat();
    markCalibration(value, remote ? "espnow mark" : "serial mark");
    snprintf(reply, replyLen, "OK mark=%.2f pwm=%u raw=%u flow_hz=%.2f",
             value, pumpOutputPercent(), pumpPwmPercent, flowHz);
    return true;
  }

  if (cmd.startsWith("note ")) {
    calNote = line.substring(5);
    writeCalLog("NOTE", calNote);
    snprintf(reply, replyLen, "OK note");
    return true;
  }

  if (cmd == "log on") {
    calLogging = true;
    calNote = remote ? "espnow log on" : "log on";
    snprintf(reply, replyLen, "OK log on");
    return true;
  }

  if (cmd == "log off") {
    calNote = remote ? "espnow log off" : "log off";
    writeCalLog("LOGOFF", calNote);
    calLogging = false;
    snprintf(reply, replyLen, "OK log off");
    return true;
  }

  if (cmd == "log today") {
    return summarizeHewLastDay(reply, replyLen);
  }

  if (cmd == "log last" || cmd == "log hew") {
    return summarizeLastHewLog(reply, replyLen);
  }

  if (cmd == "log alarm" || cmd == "log alarms") {
    return summarizeLastAlarmLog(reply, replyLen);
  }

  if (cmd == "log energy" || cmd == "log power") {
    return summarizeLastEnergyLog(reply, replyLen);
  }

  if (cmd == "project" || cmd == "project status") {
    return summarizeProjectInfo(reply, replyLen);
  }

  if (cmd == "readme" || cmd == "readme 1") {
    return summarizeProjectReadme(1, reply, replyLen);
  }

  if (cmd.startsWith("readme ")) {
    uint8_t page = constrain(line.substring(7).toInt(), 1, 99);
    return summarizeProjectReadme(page, reply, replyLen);
  }

  if (cmd == "alarm" || cmd == "alarm status") {
    return summarizeAlarmStatus(reply, replyLen);
  }

  if (cmd == "alarm ack" || cmd == "alarm clear") {
    const bool wasActive = alarmActive;
    acknowledgeAlarm();
    snprintf(reply, replyLen, "OK alarm ack was=%u now=0", wasActive ? 1 : 0);
    return true;
  }

  if (cmd == "ota on") {
    activateOta();
    snprintf(reply, replyLen, otaActive ? "OK ota on" : "ERR ota not active");
    return true;
  }

  if (cmd == "ota off") {
    deactivateOta();
    snprintf(reply, replyLen, "OK ota off");
    return true;
  }

  if (cmd.startsWith("ota min ")) {
    setupCfg.otaMinutes = constrain(line.substring(8).toInt(), 1, 60);
    snprintf(reply, replyLen, "OK ota min=%u", setupCfg.otaMinutes);
    return true;
  }

  if (cmd == "ota" || cmd == "ota status") {
    uint32_t rem = 0;
    if (otaActive) {
      const uint32_t elapsed = (millis() - otaActivatedMs) / 1000UL;
      const uint32_t total = (uint32_t)setupCfg.otaMinutes * 60UL;
      rem = elapsed < total ? total - elapsed : 0;
    }
    snprintf(reply, replyLen, "OK ota=%u begun=%u min=%u rem=%lus wifi=%d host=%s",
             otaActive ? 1 : 0,
             otaBegun ? 1 : 0,
             setupCfg.otaMinutes,
             (unsigned long)rem,
             (int)WiFi.status(),
             OTA_HOSTNAME);
    return true;
  }

  if (cmd == "espnow on") {
    activateEspNowTelemetry();
    snprintf(reply, replyLen, espNowReady ? "OK espnow on" : "ERR espnow not active");
    return true;
  }

  if (cmd == "espnow off") {
    if (remote) {
      snprintf(reply, replyLen, "ERR use CYD setup for espnow off");
    } else {
      deactivateEspNowTelemetry();
      snprintf(reply, replyLen, "OK espnow off");
    }
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
      if (line.length() < 96) line += c;
      continue;
    }

    line.trim();
    if (!line.length()) return;

    String cmd = line;
    cmd.toLowerCase();
    char reply[160] = {0};
    if (handleSharedCommand(line, cmd, false, reply, sizeof(reply))) {
      Serial.println(reply);
    } else if (cmd == "status") {
      printCalibrationStatus();
    } else if (cmd == "ver" || cmd == "version") {
      printAppVersionStatus();
    } else if (cmd == "ds" || cmd == "ds status") {
      printDs18b20Status();
    } else if (cmd == "ds scan") {
      initDs18b20();
      printDs18b20Status();
#if ENABLE_TELEGRAM
    } else if (cmd == "tg reload" || cmd == "telegram reload") {
      Serial.println(loadTelegramConfig() ? "OK telegram cfg" : "ERR telegram cfg");
    } else if (cmd == "tg test" || cmd == "telegram test") {
      Serial.println(telegramSendMessage(telegramOnlineText()) ? "OK telegram test" : "ERR telegram test");
    } else if (cmd == "tg status" || cmd == "telegram status") {
      IPAddress tgIp;
      const bool tgTcpOk = telegramTcpProbe(tgIp);
      const bool tgSocketOk = telegramSocketProbe(tgIp);
      const bool tgDnsOk = tgIp != IPAddress((uint32_t)0);
      Serial.printf("OK telegram cfg=%d online=%d chat=%s code=%d err=%s\n",
                    telegramConfigured ? 1 : 0,
                    telegramOnlineSent ? 1 : 0,
                    telegramConfigured ? "OK" : "--",
                    lastTelegramHttpCode,
                    lastTelegramError.c_str());
      Serial.printf("OK telegram net wifi=%d rssi=%d dns=%d tcp=%d tls=%d ip=%s heap=%u max=%u\n",
                    WiFi.status() == WL_CONNECTED ? 1 : 0,
                    WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0,
                    tgDnsOk ? 1 : 0,
                    tgTcpOk ? 1 : 0,
                    tgSocketOk ? 1 : 0,
                    tgDnsOk ? tgIp.toString().c_str() : "--",
                    ESP.getFreeHeap(),
                    ESP.getMaxAllocHeap());
#endif
    } else if (cmd == "ota on") {
      activateOta();
      Serial.println(otaActive ? "OK ota on" : "ERR ota not active");
    } else if (cmd == "ota off") {
      deactivateOta();
      Serial.println("OK ota off");
    } else if (cmd.startsWith("ota min ")) {
      setupCfg.otaMinutes = constrain(line.substring(8).toInt(), 1, 60);
      Serial.printf("OK ota min=%u\n", setupCfg.otaMinutes);
    } else if (cmd == "espnow on") {
      activateEspNowTelemetry();
      Serial.println(espNowReady ? "OK espnow on" : "ERR espnow not active");
    } else if (cmd == "espnow off") {
      deactivateEspNowTelemetry();
      Serial.println("OK espnow off");
    } else {
      Serial.println("ERR unknown command");
    }
    line = "";
  }
}

bool readTouch(uint16_t &x, uint16_t &y, uint16_t &z) {
  if (!touch.touched()) return false;

  TS_Point p = touch.getPoint();
  if (p.z < 200) return false;

  int sx = map(p.x, TOUCH_MIN_X, TOUCH_MAX_X, 0, 320);
  int sy = map(p.y, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, 240);
  x = constrain(sx, 0, 319);
  y = constrain(sy, 0, 239);
  z = p.z;
  return true;
}

bool isSetupScreen() {
  return currentScreen == SCREEN_SETUP1 || currentScreen == SCREEN_SETUP2 ||
         currentScreen == SCREEN_SETUP3 || currentScreen == SCREEN_SETUP4 ||
         currentScreen == SCREEN_SETUP5 || currentScreen == SCREEN_SETUP6;
}

void clearAndRedraw() {
  ui.fillSprite(C_BG);
  tft.fillScreen(C_BG);
  needFullRedraw = true;
}

void setupPrevScreen() {
  if (currentScreen == SCREEN_SETUP1) currentScreen = SCREEN_MAIN;
  else if (currentScreen == SCREEN_SETUP2) currentScreen = SCREEN_SETUP1;
  else if (currentScreen == SCREEN_SETUP3) currentScreen = SCREEN_SETUP2;
  else if (currentScreen == SCREEN_SETUP4) currentScreen = SCREEN_SETUP3;
  else if (currentScreen == SCREEN_SETUP5) currentScreen = SCREEN_SETUP4;
  else if (currentScreen == SCREEN_SETUP6) currentScreen = SCREEN_SETUP5;
  clearAndRedraw();
}

void setupNextScreen() {
  if (currentScreen == SCREEN_SETUP1) currentScreen = SCREEN_SETUP2;
  else if (currentScreen == SCREEN_SETUP2) currentScreen = SCREEN_SETUP3;
  else if (currentScreen == SCREEN_SETUP3) currentScreen = SCREEN_SETUP4;
  else if (currentScreen == SCREEN_SETUP4) currentScreen = SCREEN_SETUP5;
  else if (currentScreen == SCREEN_SETUP5) currentScreen = SCREEN_SETUP6;
  else if (currentScreen == SCREEN_SETUP6) currentScreen = SCREEN_MAIN;
  clearAndRedraw();
}

void acknowledgeAlarm() {
  alarmActive = false;
  alarmInReview = false;
  alarmKind = ALARM_NONE;
  alarmAckMs = millis();
}

ScreenId alarmTargetScreen() {
  if (alarmKind == ALARM_TEMP) return SCREEN_SETUP5;
  if (alarmKind == ALARM_SENSOR) return alarmSensorTarget == ALARM_SENSOR_RS ? SCREEN_SETUP3 : SCREEN_SETUP2;
  if (alarmKind == ALARM_PRESSURE) return SCREEN_SETUP1;
  if (alarmKind == ALARM_NO_FLOW) return SCREEN_SETUP6;
  if (alarmKind == ALARM_FLOW_SENSOR) return SCREEN_SETUP6;
  return SCREEN_MAIN;
}

void serviceAlarmReviewState() {
  if (!alarmActive) return;
  if (currentScreen == alarmTargetScreen()) {
    alarmInReview = true;
  }
}

bool clearAlarmByCulpritTouch(uint16_t x, uint16_t y) {
  if (!alarmActive || !alarmInReview || currentScreen != alarmTargetScreen()) return false;
  if (alarmKind == ALARM_TEMP && x >= 10 && x <= 310 && y >= 62 && y <= 88) {
    acknowledgeAlarm();
    return true;
  }
  if (alarmKind == ALARM_SENSOR) {
    if (alarmSensorTarget == ALARM_SENSOR_RS && x >= 10 && x <= 245 && y >= 62 && y <= 88) {
      acknowledgeAlarm();
      return true;
    }
    if (alarmSensorTarget != ALARM_SENSOR_RS && x >= 10 && x <= 310 && y >= 60 && y <= 180) {
      acknowledgeAlarm();
      return true;
    }
  }
  if (alarmKind == ALARM_PRESSURE && x >= 10 && x <= 310 && y >= 202 && y <= 236) {
    acknowledgeAlarm();
    return true;
  }
  if (alarmKind == ALARM_NO_FLOW && x >= 10 && x <= 310 && y >= 40 && y <= 206) {
    acknowledgeAlarm();
    return true;
  }
  if (alarmKind == ALARM_FLOW_SENSOR && x >= 10 && x <= 310 && y >= 40 && y <= 206) {
    flowSensorFault = false;
    acknowledgeAlarm();
    return true;
  }
  return false;
}

void handleTouchInput() {
  uint16_t x, y, z;
  bool down = readTouch(x, y, z);
  uint32_t now = millis();

  if (!down) {
    touchWasDown = false;
    return;
  }
  if (currentScreen == SCREEN_SETUP5 && x >= 20 && x <= 310 && y >= 204 && y <= 239) {
    static uint32_t lastLdrTouchMs = 0;
    if (now - lastLdrTouchMs < 60) return;
    const int bx = 36;
    const int bw = 248;
    const uint8_t minGap = 20;
    if (x >= bx && x <= bx + bw) {
      uint8_t value = (uint8_t)constrain(map((int)x, bx, bx + bw, 0, 255), 0, 255);
      const int minX = bx + map(setupCfg.ldrMinDuty, 0, 255, 0, bw);
      const int maxX = bx + map(setupCfg.ldrMaxDuty, 0, 255, 0, bw);

      if (abs((int)x - minX) <= abs((int)x - maxX)) {
        setupCfg.ldrMinDuty = constrain(value, 0, max(0, (int)setupCfg.ldrMaxDuty - minGap));
      } else {
        setupCfg.ldrMaxDuty = constrain(value, min(255, (int)setupCfg.ldrMinDuty + minGap), 255);
      }
      backlightDuty = constrain(backlightDuty, setupCfg.ldrMinDuty, setupCfg.ldrMaxDuty);
      needFullRedraw = true;
    }
    lastSetupActivityMs = now;
    lastLdrTouchMs = now;
    return;
  }
  if (touchWasDown) return;
  if (now - lastTouchMs < TOUCH_DEBOUNCE_MS) return;

  touchWasDown = true;
  lastTouchMs = now;

  if (clearAlarmByCulpritTouch(x, y)) {
    needFullRedraw = true;
    return;
  }

  if (isSetupScreen()) {
    lastSetupActivityMs = now;
    if (x <= 44 && y <= 32) {
      setupPrevScreen();
      return;
    }
    if (x >= 288 && x <= 319 && y <= 32) {
      setupNextScreen();
      return;
    }
  }

  if (currentScreen == SCREEN_SETUP1) {
    lastSetupActivityMs = now;

    if (x <= 44 && y <= 32) {
      currentScreen = SCREEN_MAIN;
      ui.fillSprite(C_BG);
      tft.fillScreen(C_BG);
      needFullRedraw = true;
      return;
    }
    if (x >= 288 && x <= 319 && y <= 32) {
      currentScreen = SCREEN_SETUP4;
      ui.fillSprite(C_BG);
      tft.fillScreen(C_BG);
      needFullRedraw = true;
      return;
    }

    if (x >= 12 && x <= 78 && y >= 60 && y <= 87) {
      setPumpModeAuto();
      needFullRedraw = true;
      return;
    }
    if (x >= 86 && x <= 236 && y >= 60 && y <= 87) {
      setPumpModeManualStart();
      needFullRedraw = true;
      return;
    }
    if (x >= 244 && x <= 308 && y >= 60 && y <= 87) {
      setPumpModeStop();
      needFullRedraw = true;
      return;
    }

    if (setupCfg.pumpMode != PUMP_MODE_MANUAL_START) {
      if (x >= 12 && x <= 100 && y >= 111 && y <= 142) {
        setupCfg.editField = SETUP_EDIT_TARGET_DT;
        needFullRedraw = true;
        return;
      }
      if (x >= 12 && x <= 100 && y >= 145 && y <= 176) {
        setupCfg.editField = SETUP_EDIT_START;
        needFullRedraw = true;
        return;
      }
      if (x >= 12 && x <= 100 && y >= 179 && y <= 210) {
        setupCfg.editField = SETUP_EDIT_STOP;
        needFullRedraw = true;
        return;
      }
    }

    const bool minus = x >= 223 && x <= 262 && y >= 121 && y <= 187;
    const bool plus = x >= 270 && x <= 309 && y >= 121 && y <= 187;
    if (minus || plus) {
      if (setupCfg.pumpMode == PUMP_MODE_MANUAL_START) {
        const int step = plus ? 1 : -1;
        setupCfg.manualPwmPercent = constrain((int)setupCfg.manualPwmPercent + step, 0, 100);
        setPumpOutputPercent(setupCfg.manualPwmPercent);
      } else {
        float step = plus ? 1.0f : -1.0f;
        if (setupCfg.editField == SETUP_EDIT_TARGET_DT) {
          setupCfg.targetDeltaT = constrain(setupCfg.targetDeltaT + step, 3.0f, 15.0f);
        } else if (setupCfg.editField == SETUP_EDIT_START) {
          setupCfg.deltaStart = constrain(setupCfg.deltaStart + step, 0.5f, 15.0f);
          if (setupCfg.deltaStop > setupCfg.deltaStart - 1.0f) {
            setupCfg.deltaStop = max(0.0f, setupCfg.deltaStart - 1.0f);
          }
        } else if (setupCfg.editField == SETUP_EDIT_STOP) {
          setupCfg.deltaStop = constrain(setupCfg.deltaStop + step, 0.0f, setupCfg.deltaStart - 1.0f);
        }
      }
      needFullRedraw = true;
      return;
    }
    return;
  }

  if (currentScreen == SCREEN_SETUP2) {
    lastSetupActivityMs = now;

    for (uint8_t i = 0; i < 3; i++) {
      const int rowY = 65 + i * 42;
      if (x >= 248 && x <= 298 && y >= rowY + 4 && y <= rowY + 29 && i < dsDetectedCount) {
        cycleDetectedRole(i);
        needFullRedraw = true;
        return;
      }
    }
    if (x >= 120 && x <= 206 && y >= 187 && y <= 217) {
      initDs18b20();
      needFullRedraw = true;
      return;
    }
    if (x >= 216 && x <= 308 && y >= 187 && y <= 217) {
      commitDetectedRoles(true);
      needFullRedraw = true;
      return;
    }
    return;
  }

  if (currentScreen == SCREEN_SETUP3) {
    lastSetupActivityMs = now;

    if (x >= 8 && x <= 220 && y >= 86 && y <= 126) {
      setupCfg.editField = SETUP_EDIT_DS_OFFSET;
      needFullRedraw = true;
      return;
    }
    if (x >= 221 && x <= 319 && y >= 86 && y <= 126) {
      setupCfg.dsOffsetIndex = (setupCfg.dsOffsetIndex + 1) % 3;
      setupCfg.editField = SETUP_EDIT_DS_OFFSET;
      needFullRedraw = true;
      return;
    }
    if (x >= 8 && x <= 220 && y >= 160 && y <= 204) {
      setupCfg.editField = SETUP_EDIT_HEATER_TEMP;
      needFullRedraw = true;
      return;
    }

    if (x >= 4 && x <= 86 && y >= 205 && y <= 239) {
      setupCfg.heaterEnabled = true;
      applyHeaterOutput();
      needFullRedraw = true;
      return;
    }
    if (x >= 90 && x <= 172 && y >= 205 && y <= 239) {
      setupCfg.heaterEnabled = false;
      applyHeaterOutput();
      needFullRedraw = true;
      return;
    }

    const bool minus = x >= 218 && x <= 265 && y >= 125 && y <= 208;
    const bool plus = x >= 266 && x <= 319 && y >= 125 && y <= 208;
    if (minus || plus) {
      if (setupCfg.editField == SETUP_EDIT_DS_OFFSET) {
        const float step = plus ? 0.5f : -0.5f;
        setupCfg.tempOffsetDs[setupCfg.dsOffsetIndex] =
            constrain(setupCfg.tempOffsetDs[setupCfg.dsOffsetIndex] + step, -10.0f, 10.0f);
      } else if (setupCfg.editField == SETUP_EDIT_HEATER_TEMP) {
        setupCfg.heaterSetTemp = constrain(setupCfg.heaterSetTemp + (plus ? 1.0f : -1.0f), 20.0f, 65.0f);
      }
      needFullRedraw = true;
      return;
    }
    return;
  }

  if (currentScreen == SCREEN_SETUP4) {
    lastSetupActivityMs = now;

    if (x <= 44 && y <= 32) {
      currentScreen = SCREEN_SETUP1;
      ui.fillSprite(C_BG);
      tft.fillScreen(C_BG);
      needFullRedraw = true;
      return;
    }
    if (x >= 288 && x <= 319 && y <= 32) {
      currentScreen = SCREEN_MAIN;
      ui.fillSprite(C_BG);
      tft.fillScreen(C_BG);
      needFullRedraw = true;
      return;
    }
    if (x >= 45 && x <= 160 && y <= 110) {
      activateOta();
      needFullRedraw = true;
      return;
    }
    if (x >= 161 && x <= 287 && y <= 110) {
      deactivateOta();
      needFullRedraw = true;
      return;
    }
    if (x >= 12 && x <= 154 && y >= 111 && y <= 176) {
      activateEspNowTelemetry();
      needFullRedraw = true;
      return;
    }
    if (x >= 166 && x <= 308 && y >= 111 && y <= 176) {
      deactivateEspNowTelemetry();
      needFullRedraw = true;
      return;
    }
    if (x >= 36 && x <= 87 && y >= 184 && y <= 227) {
      setupCfg.otaMinutes = constrain((int)setupCfg.otaMinutes - 1, 1, 60);
      needFullRedraw = true;
      return;
    }
    if (x >= 233 && x <= 284 && y >= 184 && y <= 227) {
      setupCfg.otaMinutes = constrain((int)setupCfg.otaMinutes + 1, 1, 60);
      needFullRedraw = true;
      return;
    }
    return;
  }

  if (!isSetupScreen() && x >= 247 && y >= 197) {
    currentScreen = SCREEN_SETUP1;
    lastSetupActivityMs = now;
    ui.fillSprite(C_BG);
    tft.fillScreen(C_BG);
    needFullRedraw = true;
    return;
  }
}

void setupCalibrationIo() {
  pinMode(FLOW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), countFlowPulse, FALLING);
  ledcAttach(PUMP_PWM_PIN, PUMP_PWM_FREQ, PUMP_PWM_BITS);
  stopPump();
  setupHeaterRelay();

  touchSpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touch.begin(touchSpi);
  touch.setRotation(1);
}

#define tft ui

void printTempAt(int x, int y, const char *label, float value, uint16_t color, uint8_t size) {
  tft.setTextSize(size);
  tft.setTextColor(color, C_BG);
  tft.setCursor(x, y);
  if (label && label[0]) {
    tft.print(label);
    tft.print(" ");
  }
  if (isnan(value)) {
    tft.print("--.-");
  } else {
    char buf[10];
    dtostrf(value, 4, 1, buf);
    tft.print(buf);
  }
  tft.print("C");
}

void drawThickLine(int x1, int y1, int x2, int y2, uint16_t color, int thick) {
  if (y1 == y2) {
    tft.fillRect(min(x1, x2), y1 - thick / 2, abs(x2 - x1) + 1, thick, color);
  } else if (x1 == x2) {
    tft.fillRect(x1 - thick / 2, min(y1, y2), thick, abs(y2 - y1) + 1, color);
  } else {
    for (int i = -thick / 2; i <= thick / 2; i++) tft.drawLine(x1, y1 + i, x2, y2 + i, color);
  }
}

void drawArrowRight(int x, int y, int w, uint16_t color) {
  drawThickLine(x, y, x + w - 18, y, color, 9);
  tft.fillTriangle(x + w, y, x + w - 24, y - 18, x + w - 24, y + 18, color);
}

void drawArrowLeft(int x, int y, int w, uint16_t color) {
  drawThickLine(x + 18, y, x + w, y, color, 9);
  tft.fillTriangle(x, y, x + 24, y - 18, x + 24, y + 18, color);
}

void drawWaterDrop(int cx, int cy) {
  tft.fillCircle(cx, cy + 8, 18, C_COLD);
  tft.fillTriangle(cx, cy - 26, cx - 18, cy + 5, cx + 18, cy + 5, C_COLD);
  tft.drawCircle(cx, cy + 8, 18, 0x025D);
  tft.drawLine(cx - 8, cy + 5, cx - 3, cy + 18, C_WHITE);
}

bool isNightTime() {
  return st.valid && (st.hh < 6 || st.hh >= 20);
}

void drawCloud(int x, int y) {
  tft.fillCircle(x + 17, y + 16, 15, C_WHITE);
  tft.fillCircle(x + 35, y + 11, 19, C_WHITE);
  tft.fillCircle(x + 57, y + 17, 15, C_WHITE);
  tft.fillRoundRect(x + 8, y + 17, 62, 21, 10, C_WHITE);
  tft.drawCircle(x + 17, y + 16, 15, C_DIM);
  tft.drawCircle(x + 35, y + 11, 19, C_DIM);
  tft.drawCircle(x + 57, y + 17, 15, C_DIM);
  tft.drawLine(x + 8, y + 37, x + 70, y + 37, C_DIM);
}

void drawMoon(int cx, int cy) {
  tft.fillCircle(cx, cy, 18, 0xFFE0);
  tft.fillCircle(cx + 8, cy - 4, 18, C_BG);
  tft.drawCircle(cx, cy, 18, 0xD69A);
}

void drawSolarCollector(int x, int y) {
  const bool night = isNightTime();
  if (night) {
    drawMoon(x + 40, y + 38);
  } else {
    tft.fillCircle(x + 40, y + 38, 21, TFT_YELLOW);
    tft.drawCircle(x + 40, y + 38, 22, TFT_ORANGE);
    drawThickLine(x + 40, y - 5, x + 40, y + 14, TFT_ORANGE, 5);
    drawThickLine(x + 40, y + 62, x + 40, y + 82, TFT_ORANGE, 5);
    drawThickLine(x - 2, y + 38, x + 18, y + 38, TFT_ORANGE, 5);
    drawThickLine(x + 62, y + 38, x + 82, y + 38, TFT_ORANGE, 5);
    drawThickLine(x + 10, y + 8, x + 24, y + 22, TFT_ORANGE, 5);
    drawThickLine(x + 70, y + 8, x + 56, y + 22, TFT_ORANGE, 5);
    drawThickLine(x + 10, y + 68, x + 24, y + 54, TFT_ORANGE, 5);
    drawThickLine(x + 70, y + 68, x + 56, y + 54, TFT_ORANGE, 5);
    if (!isSolarPumpActive()) {
      drawCloud(x + 2, y + 18);
    }
  }

  tft.fillTriangle(x + 0, y + 66, x + 93, y + 55, x + 78, y + 143, C_DARK);
  tft.fillTriangle(x + 0, y + 66, x + 78, y + 143, x - 13, y + 153, C_DARK);
  tft.fillTriangle(x + 12, y + 77, x + 80, y + 69, x + 67, y + 131, 0x2A9F);
  tft.fillTriangle(x + 12, y + 77, x + 67, y + 131, x + 0, y + 139, 0x2A9F);
  drawThickLine(x + 6, y + 65, x + 94, y + 55, C_WHITE, 7);
  for (int i = 0; i < 5; i++) {
    int tx = x + 21 + i * 15;
    drawThickLine(tx, y + 75, tx - 12, y + 133, 0xB71F, 6);
  }
}

void drawPumpIcon(int cx, int cy, const char *label, bool on, bool known) {
  const uint16_t body = known ? C_GREEN : C_WARN;
  tft.fillCircle(cx, cy, 29, C_DARK);
  tft.fillCircle(cx, cy, 23, body);
  tft.drawCircle(cx, cy, 29, C_LINE);
  tft.drawCircle(cx, cy, 22, C_WHITE);

  const uint8_t phase = on ? (uint8_t)((millis() / 500) % 3) : 0;
  if (phase == 0) {
    tft.fillTriangle(cx - 13, cy - 16, cx + 17, cy, cx - 13, cy + 16, C_WHITE);
    tft.fillTriangle(cx - 8, cy - 7, cx + 3, cy, cx - 8, cy + 7, body);
    tft.drawLine(cx - 13, cy - 16, cx + 17, cy, C_WHITE);
    tft.drawLine(cx + 17, cy, cx - 13, cy + 16, C_WHITE);
  } else if (phase == 1) {
    tft.fillTriangle(cx - 16, cy + 13, cx, cy - 17, cx + 16, cy + 13, C_WHITE);
    tft.fillTriangle(cx - 7, cy + 8, cx, cy - 3, cx + 7, cy + 8, body);
    tft.drawLine(cx - 16, cy + 13, cx, cy - 17, C_WHITE);
    tft.drawLine(cx, cy - 17, cx + 16, cy + 13, C_WHITE);
  } else {
    tft.fillTriangle(cx + 13, cy - 16, cx - 17, cy, cx + 13, cy + 16, C_WHITE);
    tft.fillTriangle(cx + 8, cy - 7, cx - 3, cy, cx + 8, cy + 7, body);
    tft.drawLine(cx + 13, cy - 16, cx - 17, cy, C_WHITE);
    tft.drawLine(cx - 17, cy, cx + 13, cy + 16, C_WHITE);
  }

  tft.fillRoundRect(cx - 48, cy - 9, 20, 18, 4, C_COLD);
  tft.fillRoundRect(cx + 28, cy - 9, 20, 18, 4, C_COLD);
  tft.setTextSize(2);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(cx - 6, cy + 36);
  tft.print(label);
  if (!known) tft.print("?");
}

void drawPumpIconSmall(int cx, int cy, bool on, bool known) {
  const uint16_t body = known ? (on ? C_GREEN : C_METAL) : C_WARN;
  tft.fillCircle(cx, cy, 20, C_BG);
  tft.drawCircle(cx, cy, 17, C_WHITE);
  tft.drawCircle(cx, cy, 18, C_DARK);
  tft.fillTriangle(cx + 7, cy - 10, cx - 11, cy, cx + 7, cy + 10, body);
  tft.drawTriangle(cx + 7, cy - 10, cx - 11, cy, cx + 7, cy + 10, C_WHITE);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.fillRoundRect(cx - 16, cy - 42, 32, 18, 2, C_BG);
  tft.drawRoundRect(cx - 16, cy - 42, 32, 18, 2, on ? C_GREEN : C_BAD);
  tft.setTextColor(on ? C_GREEN : C_BAD, C_BG);
  tft.setCursor(cx - 10, cy - 41);
  tft.print(on ? "ON" : "OFF");
  tft.setCursor(cx - 9, cy - 41);
  tft.print(on ? "ON" : "OFF");
  tft.setTextFont(1);
}

void drawOutlinedTankTemp(int x, int y, float value) {
  char buf[10];
  if (isnan(value)) {
    snprintf(buf, sizeof(buf), "--.-");
  } else {
    snprintf(buf, sizeof(buf), "%.1f", value);
  }

  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK);
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      if (dx == 0 && dy == 0) continue;
      tft.setCursor(x + dx, y + dy);
      tft.print(buf);
    }
  }
  tft.setTextColor(C_WHITE);
  tft.setCursor(x, y);
  tft.print(buf);

  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      if (dx == 0 && dy == 0) continue;
      tft.setCursor(x + 50 + dx, y + dy);
      tft.print("C");
    }
  }
  tft.setTextColor(C_WHITE);
  tft.setCursor(x + 50, y);
  tft.print("C");
}

void drawAirIntake(int x, int y, float value, uint16_t color) {
  printTempAt(x, y, "T1", value, color, 1);
}

void drawTempValue(int x, int y, float value, uint16_t color) {
  tft.fillRect(x, y, 54, 16, C_BG);
  char buf[10];
  if (isnan(value)) snprintf(buf, sizeof(buf), "--.-");
  else snprintf(buf, sizeof(buf), "%.1f", value);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(color, C_BG);
  tft.setCursor(x, y);
  tft.print(buf);
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setCursor(x + 31, y);
  tft.print("C");
}

void drawTank(int x, int y, int w, int h) {
  const uint16_t topColor = tempColor(st.t2);
  const uint16_t midColor = tempColor(st.t3);
  const uint16_t lowColor = tempColor(st.t3);

  tft.fillRoundRect(x + 5, y + h - 5, w - 10, 10, 5, C_SHADOW);
  tft.fillRoundRect(x, y, w, h, 10, C_METAL);
  tft.fillRoundRect(x + 5, y + 8, w - 10, h - 16, 7, C_WHITE);

  const int innerX = x + 8;
  const int innerY = y + 12;
  const int innerW = w - 16;
  const int innerH = h - 24;
  for (int i = 0; i < innerH; i++) {
    float pos = (float)i / (float)(innerH - 1);
    const uint16_t rowColor = tankRainbowColor(pos);
    uint16_t c = rowColor;
    if (i > 0 && i < innerH - 1) {
      const float posPrev = (float)(i - 1) / (float)(innerH - 1);
      const float posNext = (float)(i + 1) / (float)(innerH - 1);
      c = blend565(blend565(tankRainbowColor(posPrev), rowColor, 0.65f),
                   tankRainbowColor(posNext), 0.18f);
    }
    tft.drawFastHLine(innerX, innerY + i, innerW, c);
    tft.drawPixel(innerX, innerY + i, blend565(rowColor, C_WHITE, 0.28f));
    tft.drawPixel(innerX + 1, innerY + i, blend565(rowColor, C_WHITE, 0.16f));
    tft.drawPixel(innerX + innerW - 2, innerY + i, blend565(rowColor, C_BG, 0.18f));
    tft.drawPixel(innerX + innerW - 1, innerY + i, blend565(rowColor, C_BG, 0.30f));
  }
  const int boxTop = innerY + 7;
  const int boxMid = innerY + innerH / 2 - 8;
  const int boxLow = innerY + innerH - 22;
  tft.drawRoundRect(x, y, w, h, 10, C_DARK);

  drawOutlinedTankTemp(x + 17, boxTop, st.t2);
  drawOutlinedTankTemp(x + 17, boxLow, st.t3);

}

void drawStatusPill(int x, int y, const char *label, bool on, bool known) {
  uint16_t c = known ? (on ? C_GREEN : C_METAL) : C_WARN;
  tft.fillRoundRect(x, y, 39, 20, 8, c);
  tft.drawRoundRect(x, y, 39, 20, 8, C_DARK);
  tft.setTextSize(1);
  tft.setTextColor(C_DARK, c);
  tft.setCursor(x + 6, y + 6);
  tft.print(label);
  if (!known) tft.print("?");
}

void drawRadiatorReadout() {
  const uint16_t kalColor = tempColor(st.t4);
  tft.fillRoundRect(224, 211, 88, 25, 7, kalColor);
  tft.drawRoundRect(224, 211, 88, 25, 7, C_DARK);
  tft.setTextSize(2);
  tft.setTextColor(readableTextColor(kalColor), kalColor);
  tft.setCursor(229, 216);
  tft.print("Kl.");
  if (isnan(st.t4)) {
    tft.print("--");
  } else {
    tft.print((int)round(st.t4));
  }
  tft.print("C");
}

void drawSolarTempReadouts() {
  drawTempValue(117, 77, solarCollectorTemp, TFT_YELLOW);
  drawTempValue(194, 77, solarTankInTemp, C_WARN);
  drawTempValue(194, 140, solarReturnTemp, C_COLD);
}

void drawSdCardIcon(int x, int y, bool ok) {
  const uint16_t c = ok ? C_GREEN : C_BAD;
  tft.drawRoundRect(x, y, 13, 9, 1, c);
  tft.drawFastVLine(x + 9, y + 1, 3, c);
  tft.drawFastHLine(x + 3, y + 2, 6, c);
  for (int i = 0; i < 3; i++) {
    tft.drawFastVLine(x + 3 + i * 3, y + 5, 3, c);
  }
}

void drawWifiSignalBars(int x, int y) {
  const bool ok = WiFi.status() == WL_CONNECTED;
  int bars = 0;
  if (ok) {
    const int rssi = WiFi.RSSI();
    if (rssi >= -55) bars = 4;
    else if (rssi >= -67) bars = 3;
    else if (rssi >= -78) bars = 2;
    else bars = 1;
  }

  for (int i = 0; i < 4; i++) {
    const int h = 3 + i * 2;
    const int bx = x + i * 4;
    const int by = y + 9 - h;
    const uint16_t c = ok ? (i < bars ? C_GREEN : C_DARK) : C_BAD;
    tft.fillRect(bx, by, 3, h, c);
  }
}

uint8_t espNowTxPowerPercent() {
  if (!espNowReady) return 0;
  const float dbm = espNowEco.txPowerDbm();
  const float pct = (dbm - 2.0f) * 100.0f / (19.5f - 2.0f);
  const int rounded = constrain((int)lroundf(pct), 0, 100);
  return (uint8_t)(rounded <= 0 ? 1 : rounded);
}

float displayedDeltaT() {
  if (isnan(controlDeltaT)) return NAN;
  return (displayedPumpPercent() > 0 || controlDeltaT > 0.0f) ? controlDeltaT : NAN;
}

void drawHeader() {
  tft.fillRect(0, 0, 320, 32, C_BG);

  const char *modeLabel = "AUTO RS485";
  uint16_t modeColor = C_GREEN;
  bool modeInvert = false;
  int modeW = 96;
  if (setupCfg.pumpMode == PUMP_MODE_MANUAL_START) {
    modeLabel = "MANUAL";
    modeColor = C_WARN;
    modeInvert = true;
    modeW = 66;
  } else if (setupCfg.pumpMode == PUMP_MODE_STOP) {
    modeLabel = "STOP";
    modeColor = C_BAD;
    modeInvert = true;
    modeW = 48;
  } else if (!isRsOnline()) {
    modeLabel = "AUTO DS";
    modeColor = C_BAD;
    modeInvert = true;
    modeW = 64;
  }

  if (modeInvert) {
    const bool blinkOn = ((millis() / 150) % 2) == 0;
    const uint16_t fill = blinkOn ? modeColor : C_BG;
    const uint16_t text = blinkOn ? C_BG : modeColor;
    tft.fillRoundRect(2, 3, modeW, 20, 5, fill);
    tft.drawRoundRect(2, 3, modeW, 20, 5, modeColor);
    tft.setTextColor(text, fill);
  } else {
    tft.setTextColor(modeColor, C_BG);
  }
  tft.setTextFont(2);
  tft.setTextSize(1);
  const int textW = strlen(modeLabel) * 8;
  const int modeTextX = 2 + max(3, (modeW - textW) / 2);
  tft.setCursor(modeTextX, 5);
  tft.print(modeLabel);
  if (modeInvert) {
    tft.setCursor(modeTextX + 1, 5);
    tft.print(modeLabel);
  }
  tft.setTextFont(1);

  if (ntpTimeOk) {
    tft.setTextColor(TFT_YELLOW, C_BG);
    tft.setTextSize(3);
    tft.setCursor(107, 7);
    tft.printf(((millis() / 500) % 2) ? "%02u:%02u" : "%02u %02u", st.hh, st.mi);
  } else {
    tft.setTextColor(TFT_YELLOW, C_BG);
    tft.setTextSize(3);
    tft.setCursor(107, 7);
    tft.print(((millis() / 500) % 2) ? "--:--" : "-- --");
  }

  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(212, 2);
  tft.print("RS485:");
  const bool rsOnline = isRsOnline();
  tft.setTextColor(rsOnline ? C_GREEN : C_BAD, C_BG);
  tft.print(rsOnline ? "OK" : "OFF");
  tft.drawFastVLine(262, 2, 13, C_BLUE);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(268, 2);
  tft.print("SD:");
  tft.setTextColor(sdOK ? C_GREEN : C_BAD, C_BG);
  tft.print(sdOK ? "OK" : "OFF");
  drawSdCardIcon(306, 2, sdOK);

  tft.drawFastHLine(212, 14, 105, C_BLUE);
  tft.drawFastVLine(262, 17, 9, C_BLUE);

  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(212, 19);
  tft.print("WiFi");
  drawWifiSignalBars(240, 19);

  char ldrBuf[5];
  const int ldrPct = map(backlightDuty, LDR_MIN_BRIGHTNESS, LDR_MAX_BRIGHTNESS, 0, 100);
  snprintf(ldrBuf, sizeof(ldrBuf), "%3d", constrain(ldrPct, 0, 100));
  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(273, 19);
  tft.print("LDR");
  tft.setTextColor(C_WARN, C_BG);
  tft.setCursor(293, 19);
  tft.print(ldrBuf);
  tft.print("%");

  const bool espOk = espNowLinkOk();
  tft.setTextColor(espOk ? C_GREEN : C_BAD, C_BG);
  tft.setCursor(252, 34);
  tft.print("ESP-NOW");
  tft.setCursor(296, 34);
  tft.printf("%u%%", espNowTxPowerPercent());
}

void drawFooter() {
  uint32_t age = st.lastFrameMs ? (millis() - st.lastFrameMs) / 1000 : 9999;
  tft.setTextSize(1);
  tft.setTextColor(C_DIM, C_BG);
  tft.setCursor(2, 228);
  tft.printf("L:%d P:%u", lightRaw, backlightDuty);

  tft.setTextColor(age < 3 ? C_TEXT : C_BAD, C_BG);
  tft.setCursor(80, 228);
  tft.printf("R:%lu O:%lu B:%lu", (unsigned long)st.rxBytes, (unsigned long)st.tempFrames, (unsigned long)st.badFrames);

  tft.setTextColor(logOK ? C_GREEN : C_BAD, C_BG);
  tft.setCursor(248, 228);
  tft.print(logOK ? logFileName.substring(0, 8) : "SD ERR");
}

void drawPumpControlReadout() {
  const float shownDeltaT = displayedDeltaT();
  tft.fillRoundRect(93, 204, 126, 22, 5, C_BG);
  tft.drawRoundRect(93, 204, 126, 22, 5, C_DARK);

  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(99, 208);
  tft.printf("PWM %3u%%", displayedPumpPercent());

  tft.setTextColor(C_BLUE, C_BG);
  tft.setCursor(153, 208);
  if (!isnan(pumpTargetFlowLMin)) tft.printf("Q %.1f", pumpTargetFlowLMin);
  else tft.print("Q --");

  tft.setTextColor(C_DIM, C_BG);
  tft.setCursor(99, 217);
  if (!isnan(shownDeltaT)) {
    tft.printf("dT %.1f  ", shownDeltaT);
  } else {
    tft.print("dT --.-  ");
  }
  if (!isnan(powerKW)) tft.printf("%.1fkW", powerKW);
  else tft.print("--.-kW");
}

void drawBgText(int x, int y, int w, int h, const String &text, uint16_t color, uint8_t size) {
  tft.fillRect(x, y, w, h, C_BG);
  tft.setTextSize(size);
  tft.setTextColor(color, C_BG);
  tft.setCursor(x, y);
  tft.print(text);
}

void writeAlarmLog(AlarmKind kind, const char *reason, AlarmSensorTarget sensorTarget) {
  if (!sdOK) return;
  if (!SD.exists(LOG_DIR)) SD.mkdir(LOG_DIR);
  const bool exists = SD.exists(ALARM_LOG_PATH);
  File f = SD.open(ALARM_LOG_PATH, FILE_APPEND);
  if (!f) return;
  if (!exists || f.size() == 0) {
    f.println("ms,build,kind,sensor,reason,rs_online,rs_valid,rs_age_ms,yy,mo,dd,hh,mi,ss,t1,t2,t3,t4,ds_count,solar_collector,solar_tank_in,solar_return,delta_t,solar_delta_t3,power_kw,pwm_raw,pwm_out_pct,pump_auto,pumpC,flow_hz,flow_hz_raw,flow_pulses,flow_rejected,target_flow_l_min,kwh_day,status_raw,status_core,frames,bad_frames,espnow_tx_pct,sd_ok,log_ok,cal_log_ok,ldr,backlight");
  }

  String cleanReason = reason ? String(reason) : "";
  cleanReason.replace(',', ';');
  const uint32_t rsAge = st.lastFrameMs ? millis() - st.lastFrameMs : 0xFFFFFFFFUL;
  f.print(millis());
  f.print(',');
  f.print(BUILD_ID);
  f.print(',');
  f.print((uint8_t)kind);
  f.print(',');
  f.print((uint8_t)sensorTarget);
  f.print(',');
  f.print(cleanReason);
  f.print(',');
  f.print(isRsOnline() ? 1 : 0);
  f.print(',');
  f.print(st.valid ? 1 : 0);
  f.print(',');
  f.print(rsAge);
  f.print(',');
  f.print(st.yy);
  f.print(',');
  f.print(st.mo);
  f.print(',');
  f.print(st.dd);
  f.print(',');
  f.print(st.hh);
  f.print(',');
  f.print(st.mi);
  f.print(',');
  f.print(st.ss);
  f.print(',');
  printCsvTempTo(f, st.t1);
  f.print(',');
  printCsvTempTo(f, st.t2);
  f.print(',');
  printCsvTempTo(f, st.t3);
  f.print(',');
  printCsvTempTo(f, st.t4);
  f.print(',');
  f.print(dsSensorCount);
  f.print(',');
  printCsvTempTo(f, solarCollectorTemp);
  f.print(',');
  printCsvTempTo(f, solarTankInTemp);
  f.print(',');
  printCsvTempTo(f, solarReturnTemp);
  f.print(',');
  if (!isnan(deltaT)) f.print(deltaT, 2);
  f.print(',');
  if (!isnan(solarDeltaToT3)) f.print(solarDeltaToT3, 2);
  f.print(',');
  if (!isnan(powerKW)) f.print(powerKW, 3);
  f.print(',');
  f.print(pumpPwmPercent);
  f.print(',');
  f.print(pumpOutputPercent());
  f.print(',');
  f.print(solarPumpAutoRunning ? 1 : 0);
  f.print(',');
  f.print(st.pumpC ? 1 : 0);
  f.print(',');
  f.print(flowHz, 2);
  f.print(',');
  f.print(flowHzRaw, 2);
  f.print(',');
  f.print(flowPulsesLast);
  f.print(',');
  f.print(flowRejectedLast);
  f.print(',');
  if (!isnan(pumpTargetFlowLMin)) f.print(pumpTargetFlowLMin, 2);
  f.print(',');
  f.print(dailyEnergyKwh, 4);
  f.print(",0x");
  if (st.status196Raw < 0x1000) f.print('0');
  if (st.status196Raw < 0x0100) f.print('0');
  if (st.status196Raw < 0x0010) f.print('0');
  f.print(st.status196Raw, HEX);
  f.print(",0x");
  if (st.status196Core < 0x1000) f.print('0');
  if (st.status196Core < 0x0100) f.print('0');
  if (st.status196Core < 0x0010) f.print('0');
  f.print(st.status196Core, HEX);
  f.print(',');
  f.print(st.tempFrames);
  f.print(',');
  f.print(st.badFrames);
  f.print(',');
  f.print(espNowTxPowerPercent());
  f.print(',');
  f.print(sdOK ? 1 : 0);
  f.print(',');
  f.print(logOK ? 1 : 0);
  f.print(',');
  f.print(calLogOK ? 1 : 0);
  f.print(',');
  f.print(lightRaw);
  f.print(',');
  f.println(backlightDuty);
  f.close();
}

void raiseAlarm(AlarmKind kind, const char *reason, AlarmSensorTarget sensorTarget = ALARM_SENSOR_RS) {
  if (alarmActive) return;
  alarmActive = true;
  alarmInReview = false;
  alarmKind = kind;
  alarmSensorTarget = sensorTarget;
  snprintf(alarmReason, sizeof(alarmReason), "%s", reason);
  writeAlarmLog(kind, reason, sensorTarget);

#if ENABLE_TELEGRAM
  String msg = "ALARM\n\n";
  msg += String(reason) + "\n\n";
  msg += "T1 = " + tgTemp(st.t1) + " C\n";
  msg += "T2 = " + tgTemp(st.t2) + " C\n";
  msg += "T3 = " + tgTemp(st.t3) + " C\n";
  msg += "SOL = " + tgTemp(solarCollectorTemp) + " C";
  telegramSendMessage(msg);
#endif
}

void serviceRealAlarms() {
  if (DEMO_MODE) return;
  if (alarmActive) return;

  if (millis() >= RS_ALARM_START_DELAY_MS && !isRsOnline()) {
    raiseAlarm(ALARM_SENSOR, "RS485 OFF - T3=WYJ", ALARM_SENSOR_RS);
    return;
  }

  if (dsFault[0] || !isTempPlausible(solarCollectorTemp)) {
    raiseAlarm(ALARM_SENSOR, "CZUJNIK SOL >30s", ALARM_SENSOR_SOL);
    return;
  }

  if (dsFault[1] || !isTempPlausible(solarTankInTemp)) {
    raiseAlarm(ALARM_SENSOR, "CZUJNIK WEJ >30s", ALARM_SENSOR_WEJ);
    return;
  }

  if (dsFault[2] || !isTempPlausible(solarReturnTemp)) {
    raiseAlarm(ALARM_SENSOR, "CZUJNIK WYJ >30s", ALARM_SENSOR_WYJ);
    return;
  }

  if (safety.solarOverheat) {
    char buf[40];
    snprintf(buf, sizeof(buf), "SOLARY %.1fC", safety.hottestSolar);
    raiseAlarm(ALARM_TEMP, buf);
    return;
  }

  if (safety.tankOverheat) {
    char buf[40];
    snprintf(buf, sizeof(buf), "ZBIORNIK %.1fC", safety.hottestTank);
    raiseAlarm(ALARM_TEMP, buf);
    return;
  }

  // Alarmy przeplywu chwilowo zawieszone: zostaje sama logika i sygnalizacja robocza.

  // Cisnienie jest informacyjne: sygnalizacja tylko w SETUP 1/6.
}

void drawOutlinedTextCentered(int y, const char *text, uint8_t size, uint16_t color) {
  tft.setTextSize(size);
  const int x = max(0, (320 - tft.textWidth(text)) / 2);
  tft.setTextColor(C_BG);
  for (int dy = -2; dy <= 2; dy++) {
    for (int dx = -2; dx <= 2; dx++) {
      if (dx == 0 && dy == 0) continue;
      tft.setCursor(x + dx, y + dy);
      tft.print(text);
    }
  }
  tft.setTextColor(color);
  tft.setCursor(x, y);
  tft.print(text);
}

void drawAlarmOverlay() {
  if (!alarmActive) return;
  if (alarmInReview) return;
  const bool pulse = ((millis() / 250) % 2) == 0;
  const uint16_t alarmColor = pulse ? C_BAD : TFT_YELLOW;

  tft.setFreeFont(&FreeSansBoldOblique18pt7b);
  const char *alarmText = "ALARM";
  const int textW = tft.textWidth(alarmText);
  const int x = max(0, (320 - textW) / 2);
  const int y = 107;
  tft.setTextColor(C_BG);
  for (int dy = -3; dy <= 3; dy++) {
    for (int dx = -3; dx <= 3; dx++) {
      if (dx == 0 && dy == 0) continue;
      tft.setCursor(x + dx, y + dy);
      tft.print(alarmText);
    }
  }
  tft.setTextColor(alarmColor);
  tft.setCursor(x, y);
  tft.print(alarmText);
  tft.setFreeFont(NULL);
  drawOutlinedTextCentered(126, alarmReason, 2, TFT_YELLOW);
}

void drawMain(bool full) {
  (void)full;

  tft.fillScreen(C_BG);
  drawHeader();

  drawSolarCollector(18, 32);

  tft.setTextSize(2);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(112, 42);
  tft.print("PCWU/SOLAR");
  if (otaActive && ((millis() / 350) % 2) == 0) {
    tft.setTextSize(2);
    tft.setTextColor(C_WARN, C_BG);
    tft.setCursor(284, 42);
    tft.print("OTA");
  }

  drawThickLine(104, 99, 232, 99, C_SOLAR_PIPE, 7);
  drawThickLine(89, 163, 132, 163, C_COLD, 8);
  drawThickLine(174, 163, 251, 163, C_COLD, 8);
  drawPumpIconSmall(153, 163, isSolarPumpUiActive(), true);

  drawSolarTempReadouts();
  drawTank(234, 61, 85, 124);
  drawAirIntake(249, 48, st.t1, C_COLD);

  tft.fillRoundRect(5, 191, 235, 46, 5, C_BG);
  const bool flowFaultVisible = FLOWMETER_FAULT_UI_ENABLED && flowSensorFault;
  const bool flowWarnBlink = flowFaultVisible && ((millis() / 250) % 2) == 0; // 2 Hz
  tft.drawRoundRect(5, 191, 235, 46, 5, flowWarnBlink ? C_BAD : C_LINE);
  if (flowFaultVisible) {
    tft.drawRoundRect(63, 192, 57, 44, 4, flowWarnBlink ? C_BAD : C_WARN);
  }
  tft.drawFastVLine(62, 194, 40, C_LINE);
  tft.drawFastVLine(121, 194, 40, C_LINE);
  tft.drawFastVLine(179, 194, 40, C_LINE);

  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(27, 196);
  tft.print("dT");
  tft.setCursor(76, 196);
  tft.print("PRZEP");
  tft.setCursor(139, 196);
  tft.print("PWM");
  tft.setCursor(199, 196);
  tft.print("MOC");

  char buf[18];
  const float shownDeltaT = displayedDeltaT();
  if (!isnan(shownDeltaT)) snprintf(buf, sizeof(buf), "%.1f", shownDeltaT);
  else snprintf(buf, sizeof(buf), "--.-");
  drawBgText(13, 207, 47, 16, buf, C_GREEN, 2);

  if (!isnan(pumpTargetFlowLMin)) snprintf(buf, sizeof(buf), "%.1f", pumpTargetFlowLMin);
  else snprintf(buf, sizeof(buf), "--.-");
  drawBgText(82, 207, 36, 16, buf, C_BLUE, 2);

  snprintf(buf, sizeof(buf), "%u%%", displayedPumpPercent());
  drawBgText(131, 207, 45, 16, buf, TFT_YELLOW, 2);

  if (!isnan(powerKW)) snprintf(buf, sizeof(buf), "%.1f", powerKW);
  else snprintf(buf, sizeof(buf), "--.-");
  drawBgText(188, 207, 48, 16, buf, C_WARN, 2);

  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(82, 226);
  tft.print("l/min");
  tft.setCursor(27, 226);
  tft.print("C");
  tft.setCursor(207, 226);
  tft.print("kW");

  tft.fillRoundRect(247, 197, 71, 36, 5, C_BG);
  tft.drawRoundRect(247, 197, 71, 36, 5, C_BAD);
  tft.setTextColor(C_BAD, C_BG);
  tft.setTextSize(2);
  tft.setCursor(253, 207);
  tft.print("SETUP");

  drawAlarmOverlay();

  tft.pushSprite(0, 0);
}

void drawSetupFrame(const char *title) {
  ui.fillSprite(C_BG);
  tft.fillScreen(C_BG);
  tft.fillRoundRect(4, 4, 312, 232, 7, C_PANEL);
  tft.drawRoundRect(4, 4, 312, 232, 7, C_BORDER);

  tft.fillRoundRect(7, 7, 34, 20, 6, 0x10A4);
  tft.drawRoundRect(7, 7, 34, 20, 6, C_BORDER);
  tft.setTextSize(2);
  tft.setTextColor(C_WHITE, 0x10A4);
  tft.setCursor(17, 10);
  tft.print("<");

  tft.fillRoundRect(279, 7, 34, 20, 6, 0x10A4);
  tft.drawRoundRect(279, 7, 34, 20, 6, C_BORDER);
  tft.setTextSize(2);
  tft.setTextColor(C_WHITE, 0x10A4);
  tft.setCursor(289, 10);
  tft.print(">");

  tft.setTextSize(2);
  tft.setTextColor(C_WHITE, C_PANEL);
  const int16_t titleW = strlen(title) * 12;
  tft.setCursor((320 - titleW) / 2, 10);
  tft.print(title);
  tft.drawFastHLine(10, 32, 300, C_BORDER);
}

void drawSetupInfoRow(int y, const char *label, const char *value, uint16_t valueColor) {
  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(15, y);
  tft.print(label);
  tft.setTextColor(valueColor, C_PANEL);
  tft.setCursor(174, y);
  tft.print(value);
}

void drawSetupSafetyInfoRow(int y, const char *label, const char *temp, const char *action, uint16_t color) {
  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(15, y);
  tft.print(label);

  tft.setTextColor(color, C_PANEL);
  tft.setCursor(126, y);
  tft.print(temp);

  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(color, C_PANEL);
  tft.setCursor(184, y - 1);
  tft.print("->");
  tft.setCursor(208, y - 1);
  tft.print(action);
  tft.setTextFont(1);
}

void drawSetupSectionTitle(int y, const char *title) {
  tft.setTextSize(2);
  tft.setTextColor(C_WHITE, C_PANEL);
  tft.setCursor(max(0, (320 - tft.textWidth(title)) / 2), y);
  tft.print(title);
}

const char *dsOffsetName() {
  static const char *names[] = {"SOL", "WEJ", "WYJ"};
  return names[setupCfg.dsOffsetIndex % 3];
}

void drawDsAddressShort(int x, int y, const DeviceAddress address) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X%02X..%02X%02X",
           address[0], address[1], address[6], address[7]);
  tft.setCursor(x, y);
  tft.print(buf);
}

void drawSetupEditableRow(int y, const char *label, const char *value,
                          SetupEditField field, uint16_t valueColor) {
  const bool selected = setupCfg.editField == field;
  const uint16_t fill = selected ? C_WARN : C_PANEL2;
  const uint16_t text = selected ? C_BG : C_WHITE;
  tft.fillRoundRect(12, y, 205, 31, 6, fill);
  tft.drawRoundRect(12, y, 205, 31, 6, selected ? C_WHITE : C_BORDER);
  if (selected) tft.drawRoundRect(14, y + 2, 201, 27, 5, C_BG);

  tft.setTextSize(2);
  tft.setTextColor(text, fill);
  tft.setCursor(20, y + 8);
  tft.print(label);

  tft.setTextSize(2);
  tft.setTextColor(selected ? C_BG : valueColor, fill);
  tft.setCursor(150, y + 8);
  tft.print(value);
}

void drawSetupSmallButton(int x, int y, int w, int h, const char *label, uint16_t border) {
  tft.fillRoundRect(x, y, w, h, 6, 0x0841);
  tft.drawRoundRect(x, y, w, h, 6, border);
  tft.setTextSize(2);
  tft.setTextColor(C_WHITE, 0x0841);
  const int tx = x + (w - (int)strlen(label) * 12) / 2;
  const int ty = y + (h - 16) / 2;
  tft.setCursor(tx, ty);
  tft.print(label);
}

void drawSetupModeButton(int x, int y, int w, int h, const char *label,
                         uint16_t color, bool active) {
  const uint16_t fill = active ? color : 0x0841;
  const uint16_t border = active ? C_WHITE : color;
  const uint16_t text = active ? C_BG : color;
  tft.fillRoundRect(x, y, w, h, 5, fill);
  tft.drawRoundRect(x, y, w, h, 5, border);
  if (active) tft.drawRoundRect(x + 2, y + 2, w - 4, h - 4, 4, color);
  tft.setTextColor(text, fill);
  tft.setTextSize(2);
  int tx = x + (w - (int)strlen(label) * 12) / 2;
  if (tx < x + 4) tx = x + 4;
  tft.setCursor(tx, y + 7);
  tft.print(label);
}

void drawSetupValueRow(int y, const char *label, float value, uint8_t field) {
  const bool selected = setupCfg.editField == field;
  const uint16_t border = selected ? C_WARN : C_BORDER;
  const uint16_t fill = selected ? C_WARN : C_PANEL;
  const uint16_t text = selected ? C_BG : C_WHITE;

  tft.fillRoundRect(12, y, 94, 31, 6, fill);
  tft.drawRoundRect(12, y, 94, 31, 6, border);
  if (selected) tft.drawRoundRect(14, y + 2, 90, 27, 5, C_BG);
  tft.setTextSize(2);
  tft.setTextColor(text, fill);
  const int labelW = strlen(label) * 12;
  tft.setCursor(12 + max(4, (94 - labelW) / 2), y + 8);
  tft.print(label);

  tft.setTextSize(2);
  tft.setTextColor(C_BLUE, C_BG);
  tft.setCursor(118, y + 8);
  tft.print("T3");
  tft.setCursor(119, y + 8);
  tft.print("T3");

  char buf[8];
  snprintf(buf, sizeof(buf), field == SETUP_EDIT_STOP ? "%.0f" : "+%.0f", value);
  tft.setTextSize(3);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(148, y + 6);
  tft.print(buf);
  tft.setTextSize(2);
  const int cX = 148 + (abs((int)round(value)) >= 10 ? 54 : 36);
  tft.setCursor(cX, y + 6);
  tft.print("C");
}

void drawSetupManualPwmRow(int y, uint8_t pwmPercent) {
  tft.fillRoundRect(12, y, 94, 31, 6, C_WARN);
  tft.drawRoundRect(12, y, 94, 31, 6, C_WARN);
  tft.drawRoundRect(14, y + 2, 90, 27, 5, C_BG);
  tft.setTextSize(2);
  tft.setTextColor(C_BG, C_WARN);
  tft.setCursor(38, y + 8);
  tft.print("PWM");

  char buf[10];
  snprintf(buf, sizeof(buf), "%u%%", pwmPercent);
  tft.setTextSize(3);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(148, y + 6);
  tft.print(buf);
}

void drawSetupPlusMinusButton(int x, int y, const char *label) {
  tft.fillRoundRect(x, y, 39, 66, 7, 0x0841);
  tft.drawRoundRect(x, y, 39, 66, 7, C_DIM);
  tft.setTextSize(3);
  tft.setTextColor(C_WHITE, 0x0841);
  tft.setCursor(x + 12, y + 21);
  tft.print(label);
}

void drawSetup1(bool full) {
  (void)full;

  drawSetupFrame("SETUP 1/6 PRACA");

  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(12, 43);
  tft.print("TRYB PRACY POMPY:");

  char manualLabel[18];
  snprintf(manualLabel, sizeof(manualLabel), "MANUAL %u%%", setupCfg.manualPwmPercent);
  drawSetupModeButton(12, 60, 66, 27, "AUTO", C_GREEN, setupCfg.pumpMode == PUMP_MODE_AUTO);
  drawSetupModeButton(86, 60, 150, 27, manualLabel, C_WARN, setupCfg.pumpMode == PUMP_MODE_MANUAL_START);
  drawSetupModeButton(244, 60, 64, 27, "STOP", C_BAD, setupCfg.pumpMode == PUMP_MODE_STOP);

  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(12, 98);
  if (setupCfg.pumpMode == PUMP_MODE_MANUAL_START) {
    tft.print("NASTAWA MANUAL PWM:");
    drawSetupManualPwmRow(119, setupCfg.manualPwmPercent);
  } else {
    tft.print("NASTAWY AUTOMATYKI:");
    drawSetupValueRow(111, "CEL", setupCfg.targetDeltaT, SETUP_EDIT_TARGET_DT);
    drawSetupValueRow(145, "START", setupCfg.deltaStart, SETUP_EDIT_START);
    drawSetupValueRow(179, "STOP", setupCfg.deltaStop, SETUP_EDIT_STOP);
  }
  drawSetupPlusMinusButton(223, 121, "-");
  drawSetupPlusMinusButton(270, 121, "+");

  tft.drawFastHLine(10, 202, 300, C_BORDER);
  const bool pressureOk = !pressureOutOfRange();
  if (!pressureOk) {
    const bool blink = ((millis() / 250) % 2) == 0; // 2 Hz
    const uint16_t c = blink ? C_BAD : C_WARN;
    tft.drawRoundRect(8, 207, 304, 29, 5, c);
    tft.drawRoundRect(9, 208, 302, 27, 5, c);
  }
  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(12, 216);
  tft.print("CISNIENIE");
  tft.setTextSize(2);
  tft.setTextColor(pressureOk ? C_GREEN : C_BAD, C_PANEL);
  tft.setCursor(129, 216);
  tft.printf("%.1f bar", setupCfg.pressureBar);
  tft.setTextSize(2);
  tft.setTextColor(C_WHITE, C_PANEL);
  tft.setCursor(224, 216);
  tft.printf("%.1f-%.1f", setupCfg.pressureMinBar, setupCfg.pressureMaxBar);

  tft.pushSprite(0, 0);
}

void drawSetup2(bool full) {
  (void)full;

  drawSetupFrame("SETUP 2/6 CZUJNIKI DS");

  drawSetupSectionTitle(43, "DETEKCJA I ROLE:");

  static const char *roles[] = {"SOL", "WEJ", "WYJ"};
  for (uint8_t i = 0; i < 3; i++) {
    const int y = 65 + i * 42;
    const bool present = i < dsDetectedCount;
    const uint8_t role = dsDetectedRoles[i % DS18B20_MAX_SENSORS];
    const uint16_t roleColor = role == 0 ? C_WARN : role == 1 ? C_GREEN : C_BLUE;
    const uint16_t color = present ? roleColor : C_BAD;

    tft.fillRoundRect(12, y, 296, 34, 6, C_PANEL2);
    tft.drawRoundRect(12, y, 296, 34, 6, color);
    tft.setTextSize(2);
    tft.setTextColor(color, C_PANEL2);
    tft.setCursor(20, y + 9);
    tft.printf("DS%u", i + 1);

    tft.setTextSize(1);
    tft.setTextColor(C_WHITE, C_PANEL2);
    if (present) {
      drawDsAddressShort(64, y + 8, dsDetectedAddresses[i]);
    } else {
      tft.setCursor(64, y + 8);
      tft.print("--");
    }

    tft.setTextSize(2);
    tft.setTextColor(present && !isnan(dsDetectedTemps[i]) ? C_WHITE : C_BAD, C_PANEL2);
    tft.setCursor(172, y + 9);
    if (present && !isnan(dsDetectedTemps[i])) tft.printf("%.1fC", dsDetectedTemps[i]);
    else tft.print("--.-C");

    tft.fillRoundRect(248, y + 4, 50, 25, 5, color);
    tft.drawRoundRect(248, y + 4, 50, 25, 5, C_WHITE);
    tft.setTextColor(C_BG, color);
    tft.setCursor(role == 1 ? 257 : 255, y + 11);
    tft.print(present ? roles[role] : "--");
  }

  tft.setTextSize(2);
  tft.setTextColor(dsDetectedCount >= 3 ? C_GREEN : C_BAD, C_PANEL);
  tft.setCursor(20, 193);
  tft.printf("DS: %u/3", dsDetectedCount);
  drawSetupModeButton(120, 187, 86, 30, "SCAN", C_BLUE, false);
  drawSetupModeButton(216, 187, 92, 30, "ZAPISZ", C_GREEN, false);

  tft.setTextSize(1);
  tft.setTextColor(canSaveDetectedDsRoles() ? C_GREEN : C_DIM, C_PANEL);
  tft.setCursor(16, 223);
  if (dsRoleStatus[0] != 0 && millis() - dsRoleStatusMs < 8000UL) tft.print(dsRoleStatus);
  else tft.print("SCAN -> role SOL/WEJ/WYJ -> ZAPISZ");

  tft.pushSprite(0, 0);
}

void drawSetup3(bool full) {
  (void)full;

  drawSetupFrame("SETUP 3/6 KOREKTA");
  if (setupCfg.editField != SETUP_EDIT_DS_OFFSET &&
      setupCfg.editField != SETUP_EDIT_HEATER_TEMP) {
    setupCfg.editField = SETUP_EDIT_DS_OFFSET;
  }

  drawSetupSectionTitle(43, "ZRODLA TEMPERATUR:");
  const bool rsOnline = isRsOnline();
  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(15, 68);
  tft.print("RS485 HEWALEX");
  tft.setTextColor(rsOnline ? C_GREEN : C_BAD, C_PANEL);
  tft.setCursor(186, 68);
  tft.print(rsOnline ? "OK" : "ERROR");
  if (alarmActive && alarmInReview && alarmKind == ALARM_SENSOR &&
      alarmSensorTarget == ALARM_SENSOR_RS) {
    const bool blink = ((millis() / 250) % 2) == 0;
    const uint16_t c = blink ? C_BAD : TFT_YELLOW;
    tft.fillRoundRect(10, 62, 236, 28, 5, C_PANEL);
    tft.drawRoundRect(10, 62, 236, 28, 5, c);
    tft.drawRoundRect(11, 63, 234, 26, 5, c);
    tft.setTextSize(2);
    tft.setTextColor(c, C_PANEL);
    tft.setCursor(15, 68);
    tft.print("RS485 HEWALEX");
    tft.setCursor(186, 68);
    tft.print("OFF");
  }

  char buf[22];
  drawSetupEditableRow(91, "KOREKTA CZUJNIKA", "", SETUP_EDIT_DS_OFFSET, C_WHITE);

  const bool dsSelected = setupCfg.editField == SETUP_EDIT_DS_OFFSET;
  tft.fillRoundRect(226, 91, 42, 31, 6, dsSelected ? C_WARN : C_BG);
  tft.drawRoundRect(226, 91, 42, 31, 6, dsSelected ? C_WHITE : C_WARN);
  tft.setTextSize(2);
  tft.setTextColor(dsSelected ? C_BG : C_WARN, dsSelected ? C_WARN : C_BG);
  tft.setCursor(setupCfg.dsOffsetIndex == 1 ? 229 : 231, 99);
  tft.print(dsOffsetName());
  drawSetupSmallButton(274, 91, 34, 31, ">", C_BORDER);

  snprintf(buf, sizeof(buf), "%+.1f C", setupCfg.tempOffsetDs[setupCfg.dsOffsetIndex]);
  tft.setTextSize(3);
  tft.setTextColor(C_WHITE, C_PANEL);
  tft.setCursor(72, 133);
  tft.print(buf);

  drawSetupSmallButton(223, 129, 39, 78, "-", C_DIM);
  drawSetupSmallButton(270, 129, 39, 78, "+", C_DIM);

  snprintf(buf, sizeof(buf), "%.0f C", setupCfg.heaterSetTemp);
  drawSetupEditableRow(167, "GRZALKA", buf, SETUP_EDIT_HEATER_TEMP, C_WARN);
  drawSetupModeButton(8, 209, 74, 25, "WL", C_GREEN, setupCfg.heaterEnabled);
  drawSetupModeButton(94, 209, 74, 25, "WYL", C_BAD, !setupCfg.heaterEnabled);
  tft.setTextFont(1);
  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(184, 215);
  tft.print("max 75C");

  tft.pushSprite(0, 0);
}

void drawSetup4(bool full) {
  (void)full;

  drawSetupFrame("SETUP 4/6 LACZNOSC");

  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(12, 43);
  tft.print("OTA:");
  tft.setTextSize(2);
  tft.setTextColor(WiFi.status() == WL_CONNECTED ? C_GREEN : C_BAD, C_PANEL);
  tft.setCursor(72, 43);
  if (WiFi.status() == WL_CONNECTED) {
    tft.printf("%d dBm", WiFi.RSSI());
  } else {
    tft.print("OFF");
  }

  tft.setTextSize(1);
  tft.setTextColor(C_WHITE, C_PANEL);
  tft.setCursor(194, 41);
  tft.print(APP_VERSION_SHORT);
  tft.setTextSize(1);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(194, 53);
  tft.print(BUILD_ID);

  drawSetupModeButton(36, 63, 103, 30, "ON", C_GREEN, otaActive);
  drawSetupModeButton(181, 63, 103, 30, "OFF", C_BAD, !otaActive);

  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(12, 106);
  tft.print("ESP-NOW:");
  tft.setTextSize(2);
  tft.setTextColor(espNowLinkOk() ? C_GREEN : C_BAD, C_PANEL);
  tft.setCursor(118, 106);
  tft.printf("TX %u%% CH%u", espNowTxPowerPercent(),
             ESPNOW_FIXED_CHANNEL_ONLY ? ESPNOW_CHANNEL : WiFi.channel());

  drawSetupModeButton(36, 126, 103, 30, "ON", C_GREEN, setupCfg.espNowEnabled);
  drawSetupModeButton(181, 126, 103, 30, "OFF", C_BAD, !setupCfg.espNowEnabled);

  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(12, 164);
  tft.print("CZAS OTA:");

  tft.fillRoundRect(36, 184, 51, 43, 7, 0x0841);
  tft.drawRoundRect(36, 184, 51, 43, 7, C_DIM);
  tft.setTextSize(3);
  tft.setTextColor(C_WHITE, 0x0841);
  tft.setCursor(53, 195);
  tft.print("-");

  tft.fillRoundRect(99, 184, 122, 43, 7, C_BG);
  tft.drawRoundRect(99, 184, 122, 43, 7, C_BORDER);
  uint32_t remainingSec = (uint32_t)setupCfg.otaMinutes * 60UL;
  if (otaActive) {
    const uint32_t elapsedSec = (millis() - otaActivatedMs) / 1000UL;
    remainingSec = (elapsedSec >= remainingSec) ? 0 : (remainingSec - elapsedSec);
  }
  const uint16_t remMin = remainingSec / 60;
  const uint16_t remSec = remainingSec % 60;
  char otaBuf[10];
  snprintf(otaBuf, sizeof(otaBuf), "%02u:%02u", remMin, remSec);
  tft.setTextSize(2);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setCursor(109, 199);
  tft.print(otaBuf);

  tft.fillRoundRect(233, 184, 51, 43, 7, 0x0841);
  tft.drawRoundRect(233, 184, 51, 43, 7, C_DIM);
  tft.setTextSize(3);
  tft.setTextColor(C_WHITE, 0x0841);
  tft.setCursor(249, 195);
  tft.print("+");

  tft.pushSprite(0, 0);
}

void drawSetup5(bool full) {
  (void)full;

  drawSetupFrame("SETUP 5/6 BEZP.");

  drawSetupSectionTitle(43, "ZABEZPIECZENIA:");
  char buf[24];
  snprintf(buf, sizeof(buf), "%.0f C", setupCfg.solarForcePwmTemp);
  drawSetupSafetyInfoRow(68, "SOLARY", buf, "MAX POMPA", C_WARN);
  if (alarmActive && alarmInReview && alarmKind == ALARM_TEMP) {
    const bool blink = ((millis() / 250) % 2) == 0;
    const uint16_t c = blink ? C_BAD : TFT_YELLOW;
    tft.drawRoundRect(9, 62, 302, 28, 5, c);
    tft.drawRoundRect(10, 63, 300, 26, 5, c);
    tft.setTextSize(2);
    tft.setTextColor(c, C_PANEL);
    tft.setCursor(15, 68);
    tft.print("SOLARY");
  }
  snprintf(buf, sizeof(buf), "%.0f C", setupCfg.tankDumpPumpOnTemp);
  drawSetupSafetyInfoRow(94, "ZBIORNIK", buf, "POMPA CO", C_GREEN);
  snprintf(buf, sizeof(buf), "%.0f C", setupCfg.tankHeaterOffTemp);
  drawSetupSafetyInfoRow(120, "GRZALKA", buf, "GRZALKA OFF", C_BAD);
  snprintf(buf, sizeof(buf), "%u s", setupCfg.manualPwmTimeoutSec);
  drawSetupInfoRow(146, "MANUAL PWM", buf, C_WHITE);

  tft.setTextSize(1);
  tft.setTextColor(C_BLUE, C_PANEL);
  tft.setCursor(12, 169);
  tft.print("To sa granice krytyczne bez regulacji w setupie.");
  tft.drawFastHLine(10, 184, 300, C_BLUE);

  tft.setTextSize(2);
  tft.setTextColor(C_WARN, C_PANEL);
  const char *ldrTitle = "PODSWIETLENIE / LDR";
  tft.setCursor((320 - tft.textWidth(ldrTitle)) / 2, 188);
  tft.print(ldrTitle);

  const int bx = 36;
  const int by = 211;
  const int bw = 248;
  const int bh = 17;
  int minX = bx + map(setupCfg.ldrMinDuty, 0, 255, 0, bw);
  int maxX = bx + map(setupCfg.ldrMaxDuty, 0, 255, 0, bw);
  if (minX > maxX) {
    int tmp = minX;
    minX = maxX;
    maxX = tmp;
  }
  const int curX = bx + map(backlightDuty, 0, 255, 0, bw);

  tft.setTextSize(2);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(12, 208);
  tft.print("-");
  tft.setCursor(296, 208);
  tft.print("+");
  tft.drawRoundRect(bx, by, bw, bh, 3, C_BORDER);
  tft.fillRoundRect(minX, by + 3, max(4, maxX - minX), bh - 6, 2, C_BLUE);
  tft.drawFastVLine(minX, by - 2, bh + 4, C_WHITE);
  tft.drawFastVLine(maxX, by - 2, bh + 4, C_WHITE);
  tft.drawFastVLine(curX, by - 4, bh + 8, C_WARN);
  tft.setTextSize(1);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setCursor(setupCfg.ldrMinDuty >= 100 ? 8 : 11, by + 14);
  tft.printf("%u", setupCfg.ldrMinDuty);
  tft.setCursor(setupCfg.ldrMaxDuty >= 100 ? 290 : 293, by + 14);
  tft.printf("%u", setupCfg.ldrMaxDuty);

  tft.pushSprite(0, 0);
}

void drawSetup6(bool full) {
  (void)full;

  drawSetupFrame("SETUP 6/6 DOBOWY");

  const int gx = 41;
  const int gy = 59;
  const int gw = 244;
  const int gh = 132;
  const int labelY = gy - 15;
  const float maxTempC = 60.0f;
  const float maxPowerKw = 6.0f;
  const uint16_t pwmColor = C_PINK;

  tft.drawFastHLine(gx, gy + gh, gw, C_DIM);
  tft.drawFastVLine(gx, gy, gh, C_WARN);
  tft.drawFastVLine(gx + gw, gy, gh, C_BLUE);
  tft.drawFastVLine(gx + gw - 6, gy, gh, pwmColor);

  for (int h = 0; h <= 24; h += 4) {
    int x = gx + h * gw / 24;
    tft.drawFastVLine(x, gy + gh - 3, 7, C_DIM);
    if (h > 0 && h < 24) tft.drawFastVLine(x, gy, gh, 0x18E3);
    tft.setTextSize(1);
    tft.setTextColor(C_DIM, C_PANEL);
    tft.setCursor(x - (h >= 10 ? 6 : 3), gy + gh + 7);
    tft.print(h);
  }

  for (int i = 0; i <= 4; i++) {
    int y = gy + gh - i * gh / 4;
    tft.drawFastHLine(gx - 3, y, 7, C_WARN);
    tft.drawFastHLine(gx + gw - 3, y, 7, C_BLUE);
    tft.drawFastHLine(gx + gw - 9, y, 7, pwmColor);
    if (i > 0 && i < 4) tft.drawFastHLine(gx, y, gw, 0x18E3);
  }

  tft.setTextSize(1);
  tft.setTextColor(C_WARN, C_PANEL);
  tft.setCursor(18, labelY);
  tft.print("Tsol/");
  tft.setTextColor(C_GREEN, C_PANEL);
  tft.print("T2");
  tft.setCursor(66, labelY);
  tft.print("zbiornik");
  tft.setTextColor(C_WARN, C_PANEL);
  for (int temp = 0; temp <= 60; temp += 20) {
    int y = gy + gh - (int)roundf(temp * gh / maxTempC);
    tft.setCursor(18, y - 4);
    tft.print(temp);
    tft.fillCircle(temp >= 10 ? 32 : 26, y - 4, 1, C_WARN);
  }

  tft.setTextColor(C_BLUE, C_PANEL);
  tft.setCursor(297, labelY);
  tft.print("kW");
  tft.setTextColor(pwmColor, C_PANEL);
  tft.setCursor(250, labelY);
  tft.print("PWM");
  tft.setTextColor(C_BLUE, C_PANEL);
  for (int kwLabel = 0; kwLabel <= 6; kwLabel += 2) {
    int y = gy + gh - (int)roundf(kwLabel * gh / maxPowerKw);
    tft.setCursor(gx + gw + 8, y - 4);
    tft.print(kwLabel);
  }
  tft.setTextColor(pwmColor, C_PANEL);
  for (int pwmLabel = 0; pwmLabel <= 100; pwmLabel += 20) {
    int y = gy + gh - (int)roundf(pwmLabel * gh / 100.0f);
    tft.setCursor(gx + gw - 28, y - 4);
    tft.print(pwmLabel);
  }

  auto mapX = [&](float hour) {
    return gx + (int)roundf(constrain(hour, 0.0f, 24.0f) * gw / 24.0f);
  };
  auto mapTempY = [&](float temp) {
    return gy + gh - (int)roundf(constrain(temp, 0.0f, maxTempC) * gh / maxTempC);
  };
  auto mapPowerY = [&](float kw) {
    return gy + gh - (int)roundf(constrain(kw, 0.0f, maxPowerKw) * gh / maxPowerKw);
  };
  auto mapPwmBinX = [&](uint8_t bin) {
    return gx + (int)roundf(constrain((float)bin, 0.0f, (float)(ENERGY_PWM_BINS - 1)) * gw /
                           (float)(ENERGY_PWM_BINS - 1));
  };
  auto mapPwmY = [&](float pwm) {
    return gy + gh - (int)roundf(constrain(pwm, 0.0f, 100.0f) * gh / 100.0f);
  };

  int lastSolarHour = -1;
  int lastTankHour = -1;
  int lastPowerHour = -1;
  int lastPwmBin = -1;
  for (int h = 0; h <= 24; h++) {
    if (!isnan(energyHourSolar[h])) {
      if (lastSolarHour >= 0) {
        drawThickLine(mapX(lastSolarHour), mapTempY(energyHourSolar[lastSolarHour]),
                      mapX(h), mapTempY(energyHourSolar[h]), C_WARN, 2);
      }
      lastSolarHour = h;
    }
    if (!isnan(energyHourTank[h])) {
      if (lastTankHour >= 0) {
        drawThickLine(mapX(lastTankHour), mapTempY(energyHourTank[lastTankHour]),
                      mapX(h), mapTempY(energyHourTank[h]), C_GREEN, 2);
      }
      lastTankHour = h;
    }
    if (!isnan(energyHourPower[h])) {
      if (lastPowerHour >= 0) {
        drawThickLine(mapX(lastPowerHour), mapPowerY(energyHourPower[lastPowerHour]),
                      mapX(h), mapPowerY(energyHourPower[h]), C_BLUE, 2);
      }
      lastPowerHour = h;
    }
  }
  for (uint8_t i = 0; i < ENERGY_PWM_BINS; i++) {
    if (isnan(energyPwmBins[i])) continue;
    if (lastPwmBin >= 0) {
      drawThickLine(mapPwmBinX(lastPwmBin), mapPwmY(energyPwmBins[lastPwmBin]),
                    mapPwmBinX(i), mapPwmY(energyPwmBins[i]), pwmColor, 2);
    }
    lastPwmBin = i;
  }

  tft.setTextSize(2);
  tft.setTextColor(C_WARN, C_PANEL);
  tft.setCursor(17, 212);
  tft.print("MAX ");
  tft.print(isnan(dailyMaxSolarTemp) ? 0 : (int)roundf(dailyMaxSolarTemp));
  tft.fillCircle(tft.getCursorX() + 3, 214, 2, C_WARN);
  tft.setTextColor(C_BLUE, C_PANEL);
  tft.setCursor(149, 212);
  tft.print("BILANS ");
  tft.print(dailyEnergyKwh, 1);
  tft.print("kWh");

  tft.pushSprite(0, 0);
}

#undef tft

bool espNowLinkOk() {
  return espNowReady && espNowEco.linkOk();
}

void syncEspNowEcoStats() {
  espNowSent = espNowEco.sent();
  espNowOk = espNowEco.ok();
  espNowFail = espNowEco.fail();
  lastEspNowOkMs = espNowEco.lastOkMs();
  espNowLastSendOk = espNowEco.linkOk();
}

void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(EspNowCommandPacket)) return;

  EspNowCommandPacket p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != TELEMETRY_MAGIC || p.version != TELEMETRY_VERSION ||
      p.packetType != 2 || p.packetSize != sizeof(EspNowCommandPacket)) {
    return;
  }

  if (remoteCmdPending) return;
  remoteCmdSeq = p.cmdSeq;
  memcpy(remoteCmdText, p.command, sizeof(remoteCmdText));
  remoteCmdText[sizeof(remoteCmdText) - 1] = '\0';
  remoteCmdPending = true;
}

void sendRemoteReply(uint32_t seq, bool ok, const char *msg) {
  if (!espNowReady) return;
  EspNowReplyPacket r = {};
  r.magic = TELEMETRY_MAGIC;
  r.version = TELEMETRY_VERSION;
  r.packetType = 3;
  r.packetSize = sizeof(EspNowReplyPacket);
  r.cmdSeq = seq;
  r.ok = ok ? 1 : 0;
  snprintf(r.message, sizeof(r.message), "%s", msg ? msg : "");
  espNowEco.send(&r, sizeof(r));
}

void executeRemoteCommand(uint32_t seq, String line) {
  line.trim();
    String cmd = line;
    cmd.toLowerCase();
    char reply[160] = {0};
    bool ok = true;

    if (!line.length()) {
      ok = false;
      snprintf(reply, sizeof(reply), "ERR empty command");
    } else if (handleSharedCommand(line, cmd, true, reply, sizeof(reply))) {
      ok = strncmp(reply, "ERR", 3) != 0;
    } else if (cmd == "status") {
      snprintf(reply, sizeof(reply),
             "OK pwm=%u raw=%u flow=%.2fHz rawHz=%.2fHz dT=%.1f kW=%.2f kWh=%.3f r428=%.1f srv=%d ota=%d rs=%d ds=%u log=%d tx=%u%%",
             pumpOutputPercent(), pumpPwmPercent, flowHz, flowHzRaw, controlDeltaT, powerKW,
             dailyEnergyKwh, reg428Temp, serviceModeActive ? 1 : 0, otaActive ? 1 : 0, st.valid ? 1 : 0, dsSensorCount, calLogging ? 1 : 0, espNowTxPowerPercent());
    } else if (cmd == "ver" || cmd == "version") {
      snprintf(reply, sizeof(reply), "OK ver=%s build=%s", APP_VERSION, BUILD_ID);
    } else if (cmd == "ds scan" || cmd == "ds init" || cmd == "ds rescan") {
      initDs18b20();
      snprintf(reply, sizeof(reply), "OK ds rescan pin=%d count=%u %.1f/%.1f/%.1f",
             DS18B20_PIN, dsSensorCount, dsTemps[0], dsTemps[1], dsTemps[2]);
    } else if (cmd == "ds" || cmd == "ds status") {
      snprintf(reply, sizeof(reply), "OK ds=%u %.1f/%.1f/%.1f",
             dsSensorCount, dsTemps[0], dsTemps[1], dsTemps[2]);
    } else {
      ok = false;
      snprintf(reply, sizeof(reply), "ERR unknown command");
  }

  Serial.printf("ESP-NOW CMD #%lu '%s' -> %s\n", (unsigned long)seq, line.c_str(), reply);
  sendRemoteReply(seq, ok, reply);
}

void serviceRemoteCommands() {
  if (!remoteCmdPending) return;

  char cmdCopy[96];
  uint32_t seq;
  noInterrupts();
  seq = remoteCmdSeq;
  memcpy(cmdCopy, remoteCmdText, sizeof(cmdCopy));
  remoteCmdPending = false;
  interrupts();
  cmdCopy[sizeof(cmdCopy) - 1] = '\0';

  executeRemoteCommand(seq, String(cmdCopy));
}

void beginOtaIfNeeded() {
  if (otaBegun) return;

  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA
    .onStart([]() {
      Serial.println("OTA start");
    })
    .onEnd([]() {
      Serial.println("OTA end");
    })
    .onError([](ota_error_t error) {
      Serial.printf("OTA error=%u\n", error);
    });
  ArduinoOTA.begin();
  otaBegun = true;
}

bool ensureOtaListener() {
  if (otaBegun) return true;
  beginOtaIfNeeded();
  delay(20);
  if (!otaBegun) {
    Serial.println("OTA begin failed");
    return false;
  }
  Serial.println("OTA ready");
  return true;
}

void activateOta() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("OTA not active: WiFi OFF");
    if (otaBegun) {
      ArduinoOTA.end();
      otaBegun = false;
    }
    otaActive = false;
    return;
  }
  otaActive = ensureOtaListener();
  if (!otaActive) return;
  otaActivatedMs = millis();
  Serial.printf("OTA ON for %u min, host=%s\n", setupCfg.otaMinutes, OTA_HOSTNAME);
}

void deactivateOta() {
  otaActive = false;
  if (otaBegun) {
    ArduinoOTA.end();
    otaBegun = false;
  }
  Serial.println("OTA OFF");
}

void serviceOta() {
  if (!otaActive) return;
  if (WiFi.status() != WL_CONNECTED) {
    deactivateOta();
    return;
  }
  if (!otaBegun) {
    deactivateOta();
    return;
  }
  if (millis() - otaActivatedMs >= setupCfg.otaMinutes * 60000UL) {
    deactivateOta();
    return;
  }
  ArduinoOTA.handle();
}

void activateEspNowTelemetry() {
  setupCfg.espNowEnabled = true;
  if (espNowReady) {
    espNowEco.startCalibration();
    Serial.printf("ESP-NOW already ON, recalibration tx=%s\n", espNowEco.txPowerLabel());
    return;
  }
  initEspNowTelemetry();
}

void deactivateEspNowTelemetry() {
  setupCfg.espNowEnabled = false;
  if (espNowReady) {
    espNowEco.end();
  }
  espNowReady = false;
  espNowLastSendOk = false;
  lastEspNowOkMs = 0;
  Serial.println("ESP-NOW OFF");
}

void initEspNowTelemetry() {
  if (!ESPNOW_ENABLED || !setupCfg.espNowEnabled) return;
  if (espNowReady) return;

  WiFi.mode(WIFI_STA);
  if (ESPNOW_FIXED_CHANNEL_ONLY) {
    WiFi.disconnect(false, false);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  }

  EspNowEcoConfig cfg;
  memcpy(cfg.peerMac, DONGLE_ESPNOW_MAC, 6);
  cfg.channel = ESPNOW_FIXED_CHANNEL_ONLY ? ESPNOW_CHANNEL : 0;
  cfg.startPowerIndex = 2; // 8.5 dBm
  cfg.minOkPercent = 90;
  cfg.lowerAtPercent = 95;
  cfg.raiseBelowPercent = 85;
  cfg.windowPackets = 30;
  cfg.stableWindowsBeforeLower = 3;
  cfg.linkTimeoutMs = ESPNOW_LINK_TIMEOUT_MS;
  cfg.prefsNamespace = "cydnoweco";
  cfg.prefsKey = "txidx";

  if (!espNowEco.begin(cfg)) {
    espNowReady = false;
    Serial.println("ESP-NOW init ERROR");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);

  espNowReady = true;
  espNowEco.startCalibration();
  Serial.print("ESP-NOW TX ready, dongle=");
  for (uint8_t i = 0; i < 6; i++) {
    if (DONGLE_ESPNOW_MAC[i] < 0x10) Serial.print('0');
    Serial.print(DONGLE_ESPNOW_MAC[i], HEX);
    if (i < 5) Serial.print(':');
  }
  Serial.printf(" channel=%u tx=%s ECO calibration\n",
                ESPNOW_FIXED_CHANNEL_ONLY ? ESPNOW_CHANNEL : WiFi.channel(),
                espNowEco.txPowerLabel());
}

void fillTelemetryPacket(HewalexTelemetryPacket &p) {
  const uint32_t now = millis();
  const uint32_t rsAge = st.lastFrameMs ? now - st.lastFrameMs : 0xFFFFFFFFUL;

  memset(&p, 0, sizeof(p));
  p.magic = TELEMETRY_MAGIC;
  p.version = TELEMETRY_VERSION;
  p.packetType = 1;
  p.packetSize = sizeof(HewalexTelemetryPacket);
  p.seq = ++espNowSeq;
  p.ms = now;
  p.yy = st.yy;
  p.mo = st.mo;
  p.dd = st.dd;
  p.hh = st.hh;
  p.mi = st.mi;
  p.ss = st.ss;
  p.demoMode = DEMO_MODE ? 1 : 0;
  p.rsValid = st.valid ? 1 : 0;
  p.rsOnline = isRsOnline() ? 1 : 0;
  p.ntpOk = ntpTimeOk ? 1 : 0;
  p.pumpMode = (uint8_t)setupCfg.pumpMode;
  p.pumpC = isSolarPumpActive() ? 1 : 0;
  p.heaterE = st.heaterE ? 1 : 0;
  p.compressor = st.compressor ? 1 : 0;
  p.sdOK = sdOK ? 1 : 0;
  p.logOK = logOK ? 1 : 0;
  p.calLogOK = calLogOK ? 1 : 0;
  p.dsCount = dsSensorCount;
  p.pumpPwmPercent = pumpOutputPercent();
  p.backlightDuty = backlightDuty;
  p.espNowTxPowerPercent = espNowTxPowerPercent();
  p.wifiStatus = (uint8_t)WiFi.status();
  p.wifiRssi = WiFi.status() == WL_CONNECTED ? (int8_t)WiFi.RSSI() : 0;
  p.status196Raw = st.status196Raw;
  p.status196Core = st.status196Core;
  p.lightRaw = (uint16_t)constrain(lightRaw, 0, 65535);
  p.flowPulsesLast = (uint16_t)min(flowPulsesLast, (uint32_t)65535);
  p.flowRejectedLast = (uint16_t)min(flowRejectedLast, (uint32_t)65535);
  p.rsAgeMs = rsAge;
  p.rxBytes = st.rxBytes;
  p.goodFrames = st.goodFrames;
  p.tempFrames = st.tempFrames;
  p.badFrames = st.badFrames;
  p.espSent = espNowSent;
  p.espOk = espNowOk;
  p.espFail = espNowFail;
  p.t1 = st.t1;
  p.t2 = st.t2;
  p.t3 = st.t3;
  p.t4 = st.t4;
  p.ds0 = dsTemps[0];
  p.ds1 = dsTemps[1];
  p.ds2 = dsTemps[2];
  p.solarCollectorTemp = solarCollectorTemp;
  p.solarTankInTemp = solarTankInTemp;
  p.solarReturnTemp = solarReturnTemp;
  p.flowHz = flowHz;
  p.flowHzRaw = flowHzRaw;
  p.pumpTargetFlowLMin = pumpTargetFlowLMin;
  p.deltaT = controlDeltaT;
  p.powerKW = powerKW;
  p.pressureBar = setupCfg.pressureBar;
  p.alarmActive = alarmActive ? 1 : 0;
  p.alarmInReview = alarmInReview ? 1 : 0;
  p.alarmKind = (uint8_t)alarmKind;
  p.alarmSensorTarget = (uint8_t)alarmSensorTarget;
  snprintf(p.alarmReason, sizeof(p.alarmReason), "%s", alarmActive ? alarmReason : "");
}

void serviceEspNowTelemetry() {
  if (!setupCfg.espNowEnabled || !espNowReady) return;
  espNowEco.service();
  syncEspNowEcoStats();
  if (millis() - lastEspNowSendMs < ESPNOW_SEND_MS) return;
  lastEspNowSendMs = millis();

  HewalexTelemetryPacket p;
  fillTelemetryPacket(p);
  esp_err_t err = espNowEco.send(&p, sizeof(p));
  if (err != ESP_OK) {
    espNowLastSendOk = false;
  }
  syncEspNowEcoStats();
}

bool parseGecoFrame(const uint8_t *f, size_t len) {
  if (len < 20) return false;
  if (f[0] != FRAME_START) return false;
  if (f[3] != HARD_CONST || f[4] != 0 || f[5] != 0) return false;
  if (len != (size_t)f[6] + 8) return false;

  const uint8_t crc8 = crc8DvbS2(f, 7);
  if (crc8 != f[7]) return false;

  const uint8_t payloadLen = f[6];
  if (payloadLen < 12) return false;

  const uint8_t *p = f + 8;
  const uint16_t got16 = ((uint16_t)p[payloadLen - 2] << 8) | p[payloadLen - 1];
  const uint16_t calc16 = crc16Xmodem(p, payloadLen - 2);
  if (got16 != calc16) return false;

  const uint8_t fnc = p[4];
  const uint16_t type = le16(p + 5);
  const uint8_t regLen = p[7];
  const uint16_t regStart = le16(p + 8);
  const uint8_t dataLen = payloadLen - 12;

  if (type != SOFT_CONST) return true;

  // RS1 / EKO-LAN active read response: function 0x50 for our 0x40 read query.
  if (fnc == 0x50) {
    if (dataLen != regLen) return false;
    parseDataBlock(regStart, dataLen, p + 10);
    st.goodFrames++;
    st.tempFrames++;
    st.lastFrameMs = millis();
    st.valid = true;
    return true;
  }

  // ACK for write. We count it as good protocol traffic, but it does not update temperatures.
  if (fnc == 0x70) {
    st.goodFrames++;
    st.lastFrameMs = millis();
    st.valid = true;
    return true;
  }

  return true;
}

size_t buildReadFrame(uint8_t *out, size_t maxLen, uint16_t startReg, uint8_t count) {
  if (!out || maxLen < 20) return 0;

  uint8_t payload[12] = {};
  payload[0] = GECO_PUMP_SOFT_ADDR & 0xFF;
  payload[1] = GECO_PUMP_SOFT_ADDR >> 8;
  payload[2] = GECO_CYD_SOFT_ADDR & 0xFF;
  payload[3] = GECO_CYD_SOFT_ADDR >> 8;
  payload[4] = 0x40;                 // read request
  payload[5] = SOFT_CONST & 0xFF;
  payload[6] = SOFT_CONST >> 8;
  payload[7] = count;
  payload[8] = startReg & 0xFF;
  payload[9] = startReg >> 8;

  const uint16_t crc16 = crc16Xmodem(payload, 10);
  payload[10] = crc16 >> 8;
  payload[11] = crc16 & 0xFF;

  out[0] = FRAME_START;
  out[1] = GECO_PUMP_PHY_ADDR;
  out[2] = GECO_CYD_PHY_ADDR;
  out[3] = HARD_CONST;
  out[4] = 0x00;
  out[5] = 0x00;
  out[6] = sizeof(payload);
  out[7] = crc8DvbS2(out, 7);
  memcpy(out + 8, payload, sizeof(payload));
  return 20;
}

void sendReadQuery(uint16_t startReg, uint8_t count) {
  uint8_t frame[24] = {};
  const size_t len = buildReadFrame(frame, sizeof(frame), startReg, count);
  if (!len) return;

  while (RSbus.available()) RSbus.read();
  frameLen = 0;
  expectedLen = -1;

  RSbus.write(frame, len);
  RSbus.flush();
  rs1TxCount++;
  lastRs1QueryMs = millis();
}

void parseDataBlock(uint16_t startReg, uint8_t dataLen, const uint8_t *data) {
  if (!data) return;

  if (startReg <= 120 && startReg + dataLen >= 127) {
    st.yy = data[120 - startReg];
    st.mo = data[121 - startReg];
    st.dd = data[122 - startReg];
    st.hh = data[124 - startReg];
    st.mi = data[125 - startReg];
    st.ss = data[126 - startReg];
  }

  // Known GECO temperature block: T1..T10 start at registers 128,130,...
  if (startReg <= 128 && startReg + dataLen >= 130) st.t1 = temp10(data + (128 - startReg));
  if (startReg <= 130 && startReg + dataLen >= 132) st.t2 = temp10(data + (130 - startReg));
  if (startReg <= 132 && startReg + dataLen >= 134) st.t3 = temp10(data + (132 - startReg));
  if (startReg <= 134 && startReg + dataLen >= 136) st.t4 = temp10(data + (134 - startReg));

  if (startReg <= 196 && startReg + dataLen >= 198) {
    st.status196Raw = le16(data + (196 - startReg));
    st.status196Core = st.status196Raw & ~STATUS_DISPLAY_COUNTER_MASK;
    st.pumpC = (st.status196Core & STATUS_PUMP_C_BIT) != 0;
    st.compressor = (st.status196Core & STATUS_COMPRESSOR_BIT) != 0;
    st.heaterE = (st.status196Core & STATUS_HEATER_E_BIT) != 0;
  }

  if (startReg <= 428 && startReg + dataLen >= 430) {
    reg428Temp = temp10(data + (428 - startReg));
  }
}

void serviceRs1Polling() {
  const uint32_t now = millis();
  if (now - lastRs1QueryMs < RS1_POLL_INTERVAL_MS) return;

  switch (rs1PollStep) {
    case 0:
      sendReadQuery(120, 104);
      break;
    case 1:
      sendReadQuery(200, 50);
      break;
    case 2:
      sendReadQuery(300, 120);
      break;
    default:
      sendReadQuery(428, 16);
      break;
  }

  rs1PollStep++;
  if (rs1PollStep >= 4) rs1PollStep = 0;
}

void feedByte(uint8_t b) {
  st.rxBytes++;

  if (frameLen == 0) {
    if (b != FRAME_START) return;
    frameBuf[frameLen++] = b;
    expectedLen = -1;
    return;
  }

  if (frameLen >= FRAME_BUF_SIZE) {
    frameLen = 0;
    expectedLen = -1;
    st.badFrames++;
    return;
  }

  frameBuf[frameLen++] = b;

  if (frameLen == 7) {
    expectedLen = 8 + frameBuf[6];
    if (expectedLen < 20 || expectedLen > (int)FRAME_BUF_SIZE) {
      frameLen = 0;
      expectedLen = -1;
      st.badFrames++;
    }
  }

  if (expectedLen > 0 && frameLen == (uint16_t)expectedLen) {
    if (!parseGecoFrame(frameBuf, frameLen)) st.badFrames++;
    frameLen = 0;
    expectedLen = -1;
  }
}

void updateDemoState() {
  const float s = millis() / 1000.0f;
  st.valid = true;
  st.lastFrameMs = millis();
  st.tempFrames++;

  const uint8_t demoSlot = (uint8_t)((millis() / 12000) % 3);
  if (!ntpTimeOk) {
    st.yy = 26;
    st.mo = 5;
    st.dd = 23;
    st.hh = demoSlot == 2 ? 22 : 14;
    st.mi = 54;
    st.ss = (uint8_t)((millis() / 1000) % 60);
  }

  st.t1 = 13.5f + 4.0f * sinf(s * 0.22f);   // kolektor / otoczenie
  st.t3 = 39.7f + 8.0f * sinf(s * 0.17f);   // gora zbiornika
  st.t4 = 43.5f + 6.0f * sinf(s * 0.13f);   // srodek / powrot
  st.t2 = 47.7f + 5.0f * sinf(s * 0.10f);   // dol zbiornika

  solarCollectorTemp = 58.4f + 2.0f * sinf(s * 0.19f);
  solarTankInTemp = 51.8f + 1.5f * sinf(s * 0.16f);
  solarReturnTemp = 32.6f + 1.2f * sinf(s * 0.21f);
  pumpPwmPercent = 55;
  pumpTargetFlowLMin = 3.1f;
  deltaT = solarTankInTemp - solarReturnTemp;
  powerKW = pumpTargetFlowLMin * deltaT * 0.0566f;

  st.pumpC = demoSlot == 0;
  st.heaterE = ((millis() / 9000) % 2) == 0;
  st.compressor = false;
  st.status196Core = 0x0823;
  st.status196Raw = st.status196Core | (uint16_t)(((millis() / 2000) % 8) << 13);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  tft.init();
  tft.setRotation(1);
  // Pelny sprite 16-bit zajmuje ok. 150 kB i na niektorych CYD potrafi
  // nie wystartowac stabilnie. 8-bit jest znacznie lzejzy i usuwa miganie.
  ui.setColorDepth(8);
  ui.createSprite(320, 240);
  setupLightSensorAndBacklight();
  setupCalibrationIo();
  setupPressureSensor();
  setupDs18b20();
  resetDailyEnergy(st.dd);
  updateLightSensorAndBacklight();
  initLogger();
  writeProjectInfoFile();
  initCalLogger();
  initEnergyLogger();
  loadTelegramConfig();
  tft.fillScreen(C_BG);
  tft.setTextColor(C_TEXT, C_BG);
  tft.setTextSize(2);
  tft.setCursor(12, 35);
  tft.print(DEMO_MODE ? "CYD_SOLAR DEMO" : "CYD_SOLAR");
  tft.setCursor(12, 70);
  tft.print(DEMO_MODE ? "symulacja temp." : "PCWU/SOLAR start");
  tft.setTextSize(2);
  tft.setCursor(12, 100);
  tft.print(APP_VERSION);

  startWifiAndNtp();
  sendTelegramOnline();
  initEspNowTelemetry();

  if (!DEMO_MODE) {
    RSbus.begin(GECO_BAUD, SERIAL_8N1, RS_RX_PIN, RS_TX_PIN);
    delay(50);
    sendReadQuery(120, 104);
  }
  delay(500);
  needFullRedraw = true;
}

void loop() {
  serviceWifiAndNtp();
  serviceTelegram();

  if (ntpTimeOk && millis() - lastClockMs >= CLOCK_INTERVAL_MS) {
    lastClockMs = millis();
    refreshClockFromSystem();
  }

  handleSerialCommands();
  serviceRemoteCommands();
  updateDs18b20();
  updatePressureSensor();
  updateFlowMeter();
  handleTouchInput();

  if (millis() - lastLightMs >= LIGHT_INTERVAL_MS) {
    lastLightMs = millis();
    updateLightSensorAndBacklight();
  }

  if (DEMO_MODE) {
    if (millis() - lastDemoMs >= DEMO_INTERVAL_MS) {
      lastDemoMs = millis();
      updateDemoState();
    }
  } else {
    while (RSbus.available()) {
      feedByte((uint8_t)RSbus.read());
    }
    serviceRs1Polling();
  }

  invalidateRsTempsIfOffline();
  updateThermalPower();
  serviceEnergyBalance();
  serviceRemoteServiceMode();
  serviceSolarPumpAutomation();
  serviceSafety();
  serviceRealAlarms();
  serviceAlarmReviewState();
  serviceOta();
  serviceEspNowTelemetry();
  serviceLogger();
  serviceCalLogger();
  serviceStateSync();

  if (!DEMO_MODE && isSetupScreen() && millis() - lastSetupActivityMs >= SETUP_IDLE_RETURN_MS) {
    currentScreen = SCREEN_MAIN;
    ui.fillSprite(C_BG);
    tft.fillScreen(C_BG);
    needFullRedraw = true;
  }

  if (millis() - lastDrawMs >= DRAW_INTERVAL_MS) {
    lastDrawMs = millis();
    const bool full = needFullRedraw;
    needFullRedraw = false;
    if (currentScreen == SCREEN_SETUP1) drawSetup1(full);
    else if (currentScreen == SCREEN_SETUP2) drawSetup2(full);
    else if (currentScreen == SCREEN_SETUP3) drawSetup3(full);
    else if (currentScreen == SCREEN_SETUP4) drawSetup4(full);
    else if (currentScreen == SCREEN_SETUP5) drawSetup5(full);
    else if (currentScreen == SCREEN_SETUP6) drawSetup6(full);
    else drawMain(full);
  }
}
