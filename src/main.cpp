#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
// ttgo-esp32-display ->
#include <NTPClient.h>
// <- ttgo-esp32-display
#include <WiFi.h>
#include <Wire.h>
// ttgo-esp32-display ->
#include <WiFiUdp.h>
// <- ttgo-esp32-display
#include <esp_adc_cal.h>
#include <Button2.h>
#include <bmp.h>

// ttgo-esp32-display ->
#include "Alert.h" // Out of range alert icon

const char *ssid = "MAMD-HomeG";
const char *password = "ElaNanniRalf3";

#ifndef TFT_DISPOFF
  #define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
  #define TFT_SLPIN 0x10
#endif
// <- ttgo-esp32-display

// TFT Pins has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
// #define TFT_MOSI            19
// #define TFT_SCLK            18
// #define TFT_CS              5
// #define TFT_DC              16
// #define TFT_RST             23
#define TFT_BL              4   // Display backlight control pin


#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34
#define BUTTON_1            35
#define BUTTON_2            0

// ttgo-esp32-display ->
void drawAlert(int x, int y, boolean draw);
void drawIcon(const unsigned short *icon, int16_t x, int16_t y, int8_t width, int8_t height);
// <- ttgo-esp32-display


TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
// ttgo-esp32-display ->
WiFiUDP ntpUDP;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP,"europe.pool.ntp.org", 3600, 60000);

#define TFT_GREY 0xBDF7

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0; // Saved H, M, S x & y multipliers
float sdeg = 0, mdeg = 0, hdeg = 0;
uint16_t osx = 64, osy = 64, omx = 64, omy = 64, ohx = 64, ohy = 64; // Saved H, M, S x & y coords
uint16_t x0 = 0, x1 = 0, yy0 = 0, yy1 = 0;
uint32_t targetTime = 0; // for next 1 second timeout
bool isTimeUpdated = false;

static uint8_t conv2d(const char *p)
  {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
      v = *p - '0';
    return 10 * v + *++p - '0';
  }

uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); // Get H, M, S from compile time

boolean initial = 1;
// <- ttgo-esp32-display

Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

char buff[512];
int vref = 1100;
int btnCick = false;

#define ENABLE_SPI_SDCARD

//Uncomment will use SDCard, this is just a demonstration,
//how to use the second SPI
#ifdef ENABLE_SPI_SDCARD
    #include "FS.h"
    #include "SD.h"

    SPIClass SDSPI(HSPI);

    #define MY_CS       33
    #define MY_SCLK     25
    #define MY_MISO     27
    #define MY_MOSI     26

    void setupSDCard()
      {
        SDSPI.begin(MY_SCLK, MY_MISO, MY_MOSI, MY_CS);
        //Assuming use of SPI SD card
        if (!SD.begin(MY_CS, SDSPI))
          {
            Serial.println("Card Mount Failed");
            tft.setTextColor(TFT_RED);
            tft.drawString("SDCard Mount FAIL", tft.width() / 2, tft.height() / 2 - 32);
            tft.setTextColor(TFT_GREEN);
          }
          else
          {
            tft.setTextColor(TFT_GREEN);
            Serial.println("SDCard Mount PASS");
            tft.drawString("SDCard Mount PASS", tft.width() / 2, tft.height() / 2 - 48);
            String size = String((uint32_t)(SD.cardSize() / 1024 / 1024)) + "MB";
            tft.drawString(size, tft.width() / 2, tft.height() / 2 - 32);
          }
      }
  #else
      #define setupSDCard()
  #endif

void wifi_scan();

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
  {
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_light_sleep_start();
  }
void showVoltage()
  {
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000)
      {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        String voltage = "Voltage :" + String(battery_voltage) + "V";
        Serial.println(voltage);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(voltage,  tft.width() / 2, tft.height() / 2 );
      }
  }
void button_init()
  {
    btn1.setLongClickHandler([](Button2 & b)
      {
        btnCick = false;
        int r = digitalRead(TFT_BL);
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
        espDelay(6000);
        digitalWrite(TFT_BL, !r);

        tft.writecommand(TFT_DISPOFF);
        tft.writecommand(TFT_SLPIN);
        //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
        delay(200);
        esp_deep_sleep_start();
      });
    btn1.setPressedHandler([](Button2 & b)
      {
        Serial.println("Detect Voltage..");
        btnCick = true;
      });
    btn2.setPressedHandler([](Button2 & b)
      {
        btnCick = false;
        Serial.println("btn press wifi scan");
          wifi_scan();
      });
  }
void button_loop()
  {
    btn1.loop();
    btn2.loop();
  }
void wifi_scan()
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    tft.drawString("Scan Network", tft.width() / 2, tft.height() / 2);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    int16_t n = WiFi.scanNetworks();
    tft.fillScreen(TFT_BLACK);
    if (n == 0)
      {
        tft.drawString("no networks found", tft.width() / 2, tft.height() / 2);
      }
      else
      {
        tft.setTextDatum(TL_DATUM);
        tft.setCursor(0, 0);
        Serial.printf("Found %d net\n", n);
        for (int i = 0; i < n; ++i)
          {
            sprintf(buff,
                    "[%d]:%s(%d)",
                    i + 1,
                    WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i));
            tft.println(buff);
          }
      }
    // WiFi.mode(WIFI_OFF);
  }
  // <- ttgo-esp32-display
  // MD-ttgo ->
void test_screen_colors()
  {
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(0, 0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    /* if (TFT_BL > 0)
        {                           // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
          pinMode(TFT_BL, OUTPUT);                // Set backlight pin to output mode
          digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
        }
      */
    tft.setSwapBytes(true);
    tft.pushImage(0, 0,  240, 135, ttgo);
    espDelay(5000);

    tft.setRotation(0);
    tft.fillScreen(TFT_RED);
    espDelay(1000);
    tft.fillScreen(TFT_BLUE);
    espDelay(1000);
    tft.fillScreen(TFT_GREEN);
    espDelay(1000);
  };

void init_Clock()
  {
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Adding a black background colour erases previous text automatically

    if (TFT_BL > 0)
      {
        pinMode(TFT_BL, OUTPUT);
        digitalWrite(TFT_BL, HIGH);
      }

    // Draw clock face
    tft.fillCircle(64, 64, 61, TFT_BLUE);
    tft.fillCircle(64, 64, 57, TFT_BLACK);

    // Draw 12 lines
    for (int i = 0; i < 360; i += 30)
      {
        sx = cos((i - 90) * 0.0174532925);
        sy = sin((i - 90) * 0.0174532925);
        x0 = sx * 57 + 64;
        yy0 = sy * 57 + 64;
        x1 = sx * 50 + 64;
        yy1 = sy * 50 + 64;

        tft.drawLine(x0, yy0, x1, yy1, TFT_BLUE);
      }

    // Draw 60 dots
    for (int i = 0; i < 360; i += 6)
      {
        sx = cos((i - 90) * 0.0174532925);
        sy = sin((i - 90) * 0.0174532925);
        x0 = sx * 53 + 64;
        yy0 = sy * 53 + 64;

        tft.drawPixel(x0, yy0, TFT_BLUE);
        if (i == 0 || i == 180)
          tft.fillCircle(x0, yy0, 1, TFT_CYAN);
        if (i == 0 || i == 180)
          tft.fillCircle(x0 + 1, yy0, 1, TFT_CYAN);
        if (i == 90 || i == 270)
          tft.fillCircle(x0, yy0, 1, TFT_CYAN);
        if (i == 90 || i == 270)
          tft.fillCircle(x0 + 1, yy0, 1, TFT_CYAN);
      }

    tft.fillCircle(65, 65, 3, TFT_RED);

    // Draw text at position 64,125 using fonts 4
    // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . a p m
    // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
    //tft.drawCentreString("Connecting ...", 64, 145, 2);

    drawAlert(64, 177, true);

    delay(5000);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.print(".");
      }

    drawAlert(64, 177, false);

    tft.fillRect(0, 140, 135, 100, TFT_BLACK);
    tft.drawRect(0, 140, 135, 100, TFT_WHITE);
    tft.drawCentreString("Connected to", 64, 145, 2);
    tft.drawCentreString(ssid, 64, 185, 2);

    timeClient.setTimeOffset(7200);
    timeClient.begin();
    timeClient.update();
    ss = timeClient.getSeconds() + 2;
    mm = timeClient.getMinutes();
    hh = timeClient.getHours();
  }
void run_Clock()
  {
    if (targetTime < millis())
      {
        if (isTimeUpdated && (hh == 2))
          {
            isTimeUpdated = false;
          }

        if (!isTimeUpdated && (hh == 3))
          {
            timeClient.update();
            ss = timeClient.getSeconds() + 2;
            mm = timeClient.getMinutes();
            hh = timeClient.getHours();
          }

        targetTime = millis() + 1000;
        ss++; // Advance second
        if (ss > 59)
          {
            ss = 0;
            mm++; // Advance minute
            if (mm > 59)
              {
                mm = 0;
                hh++; // Advance hour
                if (hh > 23)
                  {
                    hh = 0;
                  }
              }
          }

        // Pre-compute hand degrees, x & y coords for a fast screen update
        sdeg = ss * 6;                     // 0-59 -> 0-354
        mdeg = mm * 6 + sdeg * 0.01666667; // 0-59 -> 0-360 - includes seconds
        hdeg = hh * 30 + mdeg * 0.0833333; // 0-11 -> 0-360 - includes minutes and seconds
        hx = cos((hdeg - 90) * 0.0174532925);
        hy = sin((hdeg - 90) * 0.0174532925);
        mx = cos((mdeg - 90) * 0.0174532925);
        my = sin((mdeg - 90) * 0.0174532925);
        sx = cos((sdeg - 90) * 0.0174532925);
        sy = sin((sdeg - 90) * 0.0174532925);

        if (ss == 0 || initial)
          {
            initial = 0;
            // Erase hour and minute hand positions every minute
            tft.drawLine(ohx, ohy, 65, 65, TFT_BLACK);
            ohx = hx * 33 + 65;
            ohy = hy * 33 + 65;
            tft.drawLine(omx, omy, 65, 65, TFT_BLACK);
            omx = mx * 44 + 65;
            omy = my * 44 + 65;
          }

        // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
        tft.drawLine(osx, osy, 65, 65, TFT_BLACK);
        tft.drawLine(ohx, ohy, 65, 65, TFT_WHITE);
        tft.drawLine(omx, omy, 65, 65, TFT_WHITE);
        osx = sx * 47 + 65;
        osy = sy * 47 + 65;
        tft.drawLine(osx, osy, 65, 65, TFT_RED);

        tft.fillCircle(65, 65, 3, TFT_RED);
      }
  }
  // <- MD-ttgo

void setup()
  {
    Serial.begin(115200);
    Serial.println("Start");
    /*  ADC_EN is the ADC detection enable port
        If the USB port is used for power supply, it is turned on by default.
        If it is powered by battery, it needs to be set to high level
      */
    pinMode(ADC_EN, OUTPUT);
    digitalWrite(ADC_EN, HIGH);
    tft.init();

    // MD-ttgo ->
    test_screen_colors();
          //tft.setRotation(1);
          //tft.fillScreen(TFT_BLACK);
          //tft.setTextSize(2);
          //tft.setTextColor(TFT_GREEN);
          //tft.setCursor(0, 0);
          //tft.setTextDatum(MC_DATUM);
          //tft.setTextSize(1);
          //tft.setSwapBytes(true);
          //tft.pushImage(0, 0,  240, 135, ttgo);
          //espDelay(5000);

          //tft.setRotation(0);
          //tft.fillScreen(TFT_RED);
          //espDelay(1000);
          //tft.fillScreen(TFT_BLUE);
          //espDelay(1000);
          //tft.fillScreen(TFT_GREEN);
          //espDelay(1000);

    init_Clock();
    targetTime = millis() + 1000;
    //espDelay(5000);
    // <- MD-ttgo

    button_init();

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t) ADC_UNIT_1, (adc_atten_t) ADC1_CHANNEL_6,
                                                            (adc_bits_width_t) ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
      {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
      }
      else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
      {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
      }
      else
      {
        Serial.println("Default Vref: 1100mV");
      }
    #ifdef DISP_BUTTONS
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);

        setupSDCard();

        tft.drawString("LeftButton:", tft.width() / 2, tft.height() / 2 - 16);
        tft.drawString("[WiFi Scan]", tft.width() / 2, tft.height() / 2 );
        tft.drawString("RightButton:", tft.width() / 2, tft.height() / 2 + 16);
        tft.drawString("[Voltage Monitor]", tft.width() / 2, tft.height() / 2 + 32 );
        tft.drawString("RightButtonLongPress:", tft.width() / 2, tft.height() / 2 + 48);
        tft.drawString("[Deep Sleep]", tft.width() / 2, tft.height() / 2 + 64 );
        tft.setTextDatum(TL_DATUM);
      #endif
  }
void loop()
  {
    if (btnCick)
      {
        showVoltage();
      }
    button_loop();
    run_Clock();
  }
void drawAlert(int x, int y, boolean draw)
  {
    if (draw)
      {
        drawIcon(alert, x - alertWidth / 2, y - alertHeight / 2, alertWidth, alertHeight);
      }
      else if (!draw)
      {
        tft.fillRect(x - alertWidth / 2, y - alertHeight / 2, alertWidth, alertHeight, TFT_BLACK);
      }
  }
//====================================================================================
// This is the function to draw the icon stored as an array in program memory (FLASH)
//====================================================================================

// To speed up rendering we use a 64 pixel buffer
#define BUFF_SIZE 64

// Draw array "icon" of defined width and height at coordinate x,y
// Maximum icon size is 255x255 pixels to avoid integer overflow
void drawIcon(const unsigned short *icon, int16_t x, int16_t y, int8_t width, int8_t height)
  {
    uint16_t pix_buffer[BUFF_SIZE]; // Pixel buffer (16 bits per pixel)
    tft.startWrite();
    // Set up a window the right size to stream pixels into
    tft.setAddrWindow(x, y, width, height);
    // Work out the number whole buffers to send
    uint16_t nb = ((uint16_t)height * width) / BUFF_SIZE;
    // Fill and send "nb" buffers to TFT
    for (int i = 0; i < nb; i++)
      {
        for (int j = 0; j < BUFF_SIZE; j++)
          {
            pix_buffer[j] = pgm_read_word(&icon[i * BUFF_SIZE + j]);
          }
        tft.pushColors(pix_buffer, BUFF_SIZE);
      }

    // Work out number of pixels not yet sent
    uint16_t np = ((uint16_t)height * width) % BUFF_SIZE;
    // Send any partial buffer left over
    if (np)
      {
        for (int i = 0; i < np; i++)
          pix_buffer[i] = pgm_read_word(&icon[nb * BUFF_SIZE + i]);
        tft.pushColors(pix_buffer, np);
      }

    tft.endWrite();
  }

