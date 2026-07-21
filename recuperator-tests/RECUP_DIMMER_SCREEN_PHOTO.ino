/*
  RECUP_DIMMER_SCREEN_PHOTO

  Static OLED screen for a photo of ESP32-C3 SuperMini OLED 0.42".
  No DimmerLink, no ACS, no I2C scan, no motor control.

  OLED I2C: SDA GPIO5, SCL GPIO6.
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

static const char *BUILD_ID = "RECUP_DIMMER_SCREEN_PHOTO 2026-07-21";

static constexpr int OLED_SDA = 5;
static constexpr int OLED_SCL = 6;
static constexpr int OLED_X0 = 28;
static constexpr int OLED_Y0 = 24;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

void drawScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.setCursor(OLED_X0, OLED_Y0 + 18);
  u8g2.print("PWM48%");
  u8g2.setCursor(OLED_X0, OLED_Y0 + 39);
  u8g2.print("0.62A");
  u8g2.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(BUILD_ID);
  Serial.println("Static photo screen only: PWM48% / 0.62A");

  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  u8g2.setContrast(180);
  drawScreen();
}

void loop() {
  delay(1000);
}
