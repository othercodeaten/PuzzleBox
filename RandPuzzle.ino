#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <bluefruit.h>
/*********** PINOUTS FOR SENSORS *************/
int sensorPin = 2; /* this is for the thermal sensor */
/*************** PHOTO CELL ******************/
int photocellPin = 3;    // the cell and 10K pulldown are connected to a0
int photocellReading;    // the analog reading from the sensor divider
int LEDpin = 30;         // connect Red LED to pin 11 (PWM pin)
int LEDbrightness;
int ledPin = 11;
int buttonBpin = 7; // button status
byte leds = 0;
int light;
int light2;
bool test = HIGH;
// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;
// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 50;    // the debounce time, increase if the output flickers
/**************** Temperature stuff ***********/
double temperatureF;
double temperatureC;
/******************** PARAMS *******************/
int c;
int temp1;
int temp2;
int photo1;
int photo2;
bool tilted;
int wincount = 0;
/*********************TIME STUFF**************************/
unsigned long previousMillis = 0;
unsigned long cleardisplayinterval = 5000;
unsigned long currentMillis = 0;

#define MANUFACTURER_ID 0x0008 /*for bluetooth connectivity*/

#define OLED_RESET 29
Adafruit_SSD1306 display(OLED_RESET);

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

#define SSD1306_LCDHEIGHT 64
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup()   {

  /********************************* FROM BLUETOOTH LOOP ***********************/
  Serial.begin(115200);
  Serial.println("Welcome to Mario and Art's Puzzle Box");
  Serial.println("---------------------------\n");

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done

  display.display();

  // Clear the buffer.
  display.clearDisplay();
  randomSeed(analogRead(28));

  pinMode(ledPin, OUTPUT);
  pinMode(buttonBpin, INPUT_PULLUP);

  initializer();
  initprinter();
}

void loop() {

  currentMillis = millis();

  displayall();

  while (wincount != 1) {
    gettemp();
    getpr();
    displayall();

    if (digitalRead(buttonBpin) == HIGH)
    {
      if (light == 0) {
        light2 = 1;
      }
      digitalWrite(ledPin, LOW);
    }
    if (digitalRead(buttonBpin) == LOW)
    {
      if (light == 1) {
        light2 = 1;
      }
      digitalWrite(ledPin, HIGH);
    }

    if (((temperatureF >= temp1) && (temperatureF <= temp2)) &&
        ((photocellReading >= photo1) && (photocellReading <= photo2))
        && (light2 == 1)) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(12, 15);
      display.println("YOU WON THE GAME!");
      display.display();
      //display.clearDisplay();
      wincount++;
      delay(5000);
      initializer();
      initprinter();
    }
    wincount--;
  }
  //delay(3000);
}

void gettemp() {

  int reading = analogRead(sensorPin);

  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = reading * 3.3;
  voltage /= 1024.0;

  // now print out the temperature
  temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
  //to degrees ((voltage - 500mV) times 100)

  // now convert to Fahrenheit
  temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
  /******* PRINT DEGREE C ****************/
  /* NOTE: there was a weird thing with how this prints on the LCD*/
  // show that its in degree C

  display.clearDisplay();
}

void getpr() {

  photocellReading = analogRead(photocellPin);
  
}

// intialize vars...
void initializer() {

  int val = random(0, 100);

  if (val >= 50) { //darkp
    temp1 = random(65, 70);
    temp2 = random(80, 90);
    photo1 = random(10, 40);
    photo2 = random(50, 70);
    light = 0;

  } else { //light and higher temp
    temp1 = random(70, 75);
    temp2 = random(80, 90);
    photo1 = random(500, 600);
    photo2 = random(750, 800);
    light = 1;
  }
}
// display vars
void initprinter() {
  Serial.println(temp1);
  Serial.println(temp2);
  Serial.println(photo1);
  Serial.println(photo2);
  Serial.println("");
}
void displayall() {

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("Temp:");

  /********* PRINT DEGREE F ****************/
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(60, 10);
  display.println(temperatureF);
  display.display();
  /********* PRINT DEGREE F ****************/
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  display.println("Lumens:");
  display.display();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(60, 20);
  display.println(photocellReading);
  display.display();

  if (light == 0 ) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("TURN ON THE LIGHT");
    display.display();
  }

  if (light == 1) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("TURN OFF THE LIGHT");
    display.display();
  }
  display.clearDisplay();
}
