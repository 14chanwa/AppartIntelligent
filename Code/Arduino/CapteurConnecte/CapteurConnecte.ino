#include <Wire.h>
#include <Adafruit_AHTX0.h> // Library for AHT20
#include <Adafruit_BMP280.h> // Library for BMP280

// Create sensor objects
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>

// ESP8266 CS(SS)=15,SCL(SCK)=14,SDA(MOSI)=13,BUSY=16,RES(RST)=5,DC=4

// 1.54'' EPD Module
//GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS=5*/ 15, /*DC=*/ 4, /*RES=*/ 5, /*BUSY=*/ 16)); // GDEH0154D67 200x200, SSD1681

// 2.13'' EPD Module
GxEPD2_BW<GxEPD2_213_BN, GxEPD2_213_BN::HEIGHT> display(GxEPD2_213_BN(/*CS=5*/ 15, /*DC=*/ 12, /*RES=*/ 2, /*BUSY=*/ 16)); // DEPG0213BN 122x250, SSD1680
//GxEPD2_3C<GxEPD2_213_Z98c, GxEPD2_213_Z98c::HEIGHT> display(GxEPD2_213_Z98c(/*CS=5*/ 15, /*DC=*/ 4, /*RES=*/ 5, /*BUSY=*/ 16)); // GDEY0213Z98 122x250, SSD1680

// 2.9'' EPD Module
//GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(/*CS=5*/ 15, /*DC=*/ 4, /*RES=*/ 5, /*BUSY=*/ 16)); // DEPG0290BS 128x296, SSD1680
//GxEPD2_3C<GxEPD2_290_C90c, GxEPD2_290_C90c::HEIGHT> display(GxEPD2_290_C90c(/*CS=5*/ 15, /*DC=*/ 4, /*RES=*/ 5, /*BUSY=*/ 16)); // GDEM029C90 128x296, SSD1680

// 4.2'' EPD Module
//GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> display(GxEPD2_420_GDEY042T81(/*CS=5*/ 15, /*DC=*/ 4, /*RES=*/ 5, /*BUSY=*/ 16)); // 400x300, SSD1683
//GxEPD2_3C<GxEPD2_420c_GDEY042Z98, GxEPD2_420c_GDEY042Z98::HEIGHT> display(GxEPD2_420c_GDEY042Z98(/*CS=5*/ 15, /*DC=*/ 4, /*RES=*/ 5, /*BUSY=*/ 16)); // 400x300, SSD1683

void setup()
{
  display.init(115200,true,50,false);

  // Initialize AHT20 sensor
  if (!aht.begin()) {
    Serial.println("Failed to initialize AHT20 sensor!");
    while (1);
  }
  Serial.println("AHT20 sensor initialized.");

  // Initialize BMP280 sensor
  if (!bmp.begin(0x77)) { // Default I²C address for BMP280 is 0x76
    Serial.println("Failed to initialize BMP280 sensor!");
    while (1);
  }
  Serial.println("BMP280 sensor initialized.");

  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);

  // helloWorld();
  // helloFullScreenPartialMode();
  // delay(1000);
  // if (display.epd2.hasFastPartialUpdate)
  // {
  //   showPartialUpdate();
  //   delay(1000);
  // }
  // display.hibernate();
}

char TemperatureBuffer[20];
char HumidityBuffer[20];
char PressureBuffer[20];

// void updateDisplay()
// {
//   // Read data from AHT20
//   sensors_event_t humidity, temp;
//   aht.getEvent(&humidity, &temp);

//   sprintf(TemperatureBuffer, "Temp : %.2f °C", temp.temperature);
//   sprintf(HumidityBuffer, "Humidity : %.2f %%", humidity.relative_humidity);

//   int16_t tbx, tby; uint16_t tbw, tbh;
//   display.getTextBounds(TemperatureBuffer, 0, 0, &tbx, &tby, &tbw, &tbh);
//   // center the bounding box by transposition of the origin:
//   uint16_t x = ((display.width() - tbw) / 2) - tbx;
//   uint16_t y = ((display.height() - tbh) / 2) - tby;
//   display.setFullWindow();
//   display.firstPage();
//   {
//     display.fillScreen(GxEPD_WHITE);
//     display.setCursor(x, y-tbh);
//     display.print(TemperatureBuffer);
//     display.setTextColor(display.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);

//     display.getTextBounds(HumidityBuffer, 0, 0, &tbx, &tby, &tbw, &tbh);
//     x = ((display.width() - tbw) / 2) - tbx;
//     display.setCursor(x, y+tbh);
//     display.print(HumidityBuffer);
//   }
//   while (display.nextPage());
// }

void updateDisplay()
{
    // Read data from AHT20
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  // Read data from BMP280
  float pressure = bmp.readPressure() / 100.0F; // Convert to hPa
  float altitude = bmp.readAltitude(1013.25); // Sea level pressure in hPa

  sprintf(TemperatureBuffer, "Temp : %.2f °C", temp.temperature);
  sprintf(HumidityBuffer, "Humidity : %.2f %%", humidity.relative_humidity);
  sprintf(PressureBuffer, "Pres : %.2f hPa", pressure);

  display.setPartialWindow(0, 0, display.width(), display.height());
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);

  // do this outside of the loop
  int16_t tbx, tby; uint16_t tbw, tbh;
  // center update text
  display.getTextBounds(TemperatureBuffer, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t utx = ((display.width() - tbw) / 2) - tbx;
  uint16_t uty = ((display.height() / 4) - tbh / 2) - tby;
  // center update mode
  display.getTextBounds(PressureBuffer, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t umx = ((display.width() - tbw) / 2) - tbx;
  uint16_t umy = ((display.height() * 3 / 4) - tbh / 2) - tby;
  // center HelloWorld
  display.getTextBounds(HumidityBuffer, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t hwx = ((display.width() - tbw) / 2) - tbx;
  uint16_t hwy = ((display.height() - tbh) / 2) - tby;
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(hwx, hwy);
    display.print(HumidityBuffer);
    display.setCursor(utx, uty);
    display.print(TemperatureBuffer);
    display.setCursor(umx, umy);
    display.print(PressureBuffer);
  }
  while (display.nextPage());
}

// void showPartialUpdate()
// {
//   // some useful background
//   helloWorld();
//   // use asymmetric values for test
//   uint16_t box_x = 10;
//   uint16_t box_y = 15;
//   uint16_t box_w = 70;
//   uint16_t box_h = 20;
//   uint16_t cursor_y = box_y + box_h - 6;
//   if (display.epd2.WIDTH < 104) cursor_y = box_y + 6;
//   float value = 13.95;
//   uint16_t incr = display.epd2.hasFastPartialUpdate ? 1 : 3;
//   display.setFont(&FreeMonoBold9pt7b);
//   if (display.epd2.WIDTH < 104) display.setFont(0);
//   display.setTextColor(GxEPD_BLACK);
//   // show where the update box is
//   for (uint16_t r = 0; r < 4; r++)
//   {
//     display.setRotation(r);
//     display.setPartialWindow(box_x, box_y, box_w, box_h);
//     display.firstPage();
//     do
//     {
//       display.fillRect(box_x, box_y, box_w, box_h, GxEPD_BLACK);
//       //display.fillScreen(GxEPD_BLACK);
//     }
//     while (display.nextPage());
//     delay(2000);
//     display.firstPage();
//     do
//     {
//       display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
//     }
//     while (display.nextPage());
//     delay(1000);
//   }
//   //return;
//   // show updates in the update box
//   for (uint16_t r = 0; r < 4; r++)
//   {
//     display.setRotation(r);
//     display.setPartialWindow(box_x, box_y, box_w, box_h);
//     for (uint16_t i = 1; i <= 10; i += incr)
//     {
//       display.firstPage();
//       do
//       {
//         display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
//         display.setCursor(box_x, cursor_y);
//         display.print(value * i, 2);
//       }
//       while (display.nextPage());
//       delay(500);
//     }
//     delay(1000);
//     display.firstPage();
//     do
//     {
//       display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
//     }
//     while (display.nextPage());
//     delay(1000);
//   }
// }

void loop() {
  // put your main code here, to run repeatedly:
  updateDisplay();
  delay(5000);
}
