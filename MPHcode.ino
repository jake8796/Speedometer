
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA 2, SCL 3 pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

static const int RXPin = 9, TXPin = 8;
const byte interruptPin = 7;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);


class Distance {
  public:
    double Long1;
    double Long2;
    double Lat1;
    double Lat2;
    double R = 3958.8;
    double distance = 0.0;
    unsigned long Time = 0.0;
    
    void calculateDistance() {
      if(Time == 0.0){
        Time = millis();
        Long1 = gps.location.lng();
        Lat1 = gps.location.lat();
      }
      if (millis() > Time + 10000.0) {
        Long2 = gps.location.lng();
        Lat2 = gps.location.lat();
        
        double phi1 = Lat1 * DEG_TO_RAD;
        double phi2 = Lat2 * DEG_TO_RAD;
        double lambda1 = Long1 * DEG_TO_RAD;
        double lambda2 = Long2 * DEG_TO_RAD;


        double deltaPhi = phi2 - phi1;
        double deltaLambda = lambda2 - lambda1;

        double a = sq(sin(deltaPhi / 2.0)) + cos(phi1) * cos(phi2) * sq(sin(deltaLambda / 2.0));
        double b = (sqrt(a) / sqrt(1.0 - a));
        double c = 2.0 * atan(b);
        
        distance = distance + (R * c);
        Time = 0.0;
      }
    }
    void resetDistance() {
      distance = 0.0;
    }

};

Distance D1;

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), distanceISR, FALLING);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally

}

void loop() {
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      if (gps.location.isValid()) {
        D1.calculateDistance();
        Serial.println(D1.distance);
        showDisplay(D1.distance);
      } else {
        error();
      }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
}

void distanceISR() {
  D1.resetDistance();
}

void showDisplay(double distance) {
  double data = gps.speed.mph();
  display.clearDisplay();
  if (data >= 10) display.setCursor(25, 16);
  if (data < 10) display.setCursor(47, 16); // Start at top-left corner
  display.setTextSize(7);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(data);

  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print(distance);


  display.display();
  delay(10);
}

void error(void) {
  display.clearDisplay();
  display.setCursor(0, 16);
  display.setTextSize(3);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.println(F("No Data"));
  display.display();
}
