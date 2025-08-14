#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ADCReading.hpp>


#define AOUT_PIN 34 
#define BUTTON_PIN 25

#define SCREEN_SDA 21 
#define SCREEN_SCL 22 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MOISTURE_VALUE_THRESHOLD 1950
#define TIME_TO_WATER_S 20
#define TIME_TO_WAIT_S 1800

#define MOTOR_1A 16
#define MOTOR_2A 17
#define MOTOR_EN12 27

#define MOTOR_3A 32
#define MOTOR_4A 33
#define MOTOR_EN34 14

#define MOTOR_PWM_SPEED 90

uint64_t lastCheckedTime = 0;
bool firstTime = true;

#define EXPONENTIAL_SMOOTHING_COEFFICIENT 0.995
float moistureReading = 0.0;
bool moistureWasInit = false;


void task_print_display(void* parameters)
{
  for (;;)
  {
    int remaining_s = (int)(((TIME_TO_WAIT_S * 1000000.0f) - (esp_timer_get_time() - lastCheckedTime)) / 1000000.0);
    int value = int(moistureReading); // read the analog value from sensor
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 10);
    display.printf("%d/%d", value, MOISTURE_VALUE_THRESHOLD);
    display.setCursor(10, 40);
    display.printf("%d/%d", remaining_s, TIME_TO_WAIT_S);
    display.display();

    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(AOUT_PIN, INPUT);

  Serial.println("Motor test!");

  // Display
  Wire.setPins(SCREEN_SDA, SCREEN_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.setRotation(2);
  display.display();

  // ADC
  analogSetAttenuation(ADC_11db);

  // Motor pins
  pinMode(MOTOR_1A, OUTPUT);
  pinMode(MOTOR_2A, OUTPUT);
  pinMode(MOTOR_EN12, OUTPUT);

  pinMode(MOTOR_3A, OUTPUT);
  pinMode(MOTOR_4A, OUTPUT);
  pinMode(MOTOR_EN34, OUTPUT);

  // Set direction
  digitalWrite(MOTOR_1A, LOW);
  digitalWrite(MOTOR_2A, HIGH);
  digitalWrite(MOTOR_3A, LOW);
  digitalWrite(MOTOR_4A, HIGH);

  // Disable motors
  digitalWrite(MOTOR_EN12, LOW);
  digitalWrite(MOTOR_EN34, LOW);

  analogWriteFrequency(50);
  analogWriteResolution(8);

  // Create update display task
  xTaskCreate(task_print_display, "task_print_display", 15000, NULL, 50, NULL);
}


void loop() {
  
  if (!moistureWasInit)
  {
    moistureReading = analogRead(AOUT_PIN);
  }
  moistureReading = moistureReading * EXPONENTIAL_SMOOTHING_COEFFICIENT + (1.0-EXPONENTIAL_SMOOTHING_COEFFICIENT) * analogRead(AOUT_PIN);
  
  Serial.print("Moisture value: ");
  Serial.println(moistureReading);

  if (digitalRead(BUTTON_PIN) == LOW)
  {
    analogWrite(MOTOR_EN34, MOTOR_PWM_SPEED);
    // digitalWrite(MOTOR_EN34, HIGH);
  }
  else
  {
    // analogWrite(MOTOR_EN34, 0);
    digitalWrite(MOTOR_EN34, LOW);
  }

  if (firstTime || (esp_timer_get_time() - lastCheckedTime) >= (TIME_TO_WAIT_S * 1000000.0f))
  {

    if (moistureReading > MOISTURE_VALUE_THRESHOLD)
    {
      analogWrite(MOTOR_EN34, MOTOR_PWM_SPEED);
      //digitalWrite(MOTOR_EN34, HIGH);

      delay(TIME_TO_WATER_S * 1000);

      // analogWrite(MOTOR_EN34, 0);
      digitalWrite(MOTOR_EN34, LOW);
    }
    
    lastCheckedTime = esp_timer_get_time();
    firstTime = false;
  }

  delay(100);
}