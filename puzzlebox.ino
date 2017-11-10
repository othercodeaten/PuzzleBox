#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <bluefruit.h>

/*********** PINOUTS FOR SENSORS ***********/
int sensorPin = 2; /* this is for the thermal sensor */

/*************** PHOTO CELL ****************/
int photocellPin = 3;     // the cell and 10K pulldown are connected to a0
int photocellReading;     // the analog reading from the sensor divider
int LEDpin = 30;         // connect Red LED to pin 11 (PWM pin)
int LEDbrightness;
/**************** BALL TILT *****************/ 
int inPin = 5;         // the number of the input pin
int outPin = 30;       // the number of the output pin
 
int LEDstate = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
 
// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 50;   // the debounce time, increase if the output flickers

/********************************************/

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
  Serial.begin(9600);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done

  pinMode(inPin, INPUT);
  digitalWrite(inPin, HIGH);   // turn on the built in pull-up resistor
  pinMode(outPin, OUTPUT);
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();

 
  
  //delay(2000);

  // Clear the buffer.
  display.clearDisplay();
}

void loop() {                // run over and over again

tiltevent();
displaypr();
displaytemp();

}
void displaytemp() {

 int reading = analogRead(sensorPin);  
 
 // converting that reading to voltage, for 3.3v arduino use 3.3
 float voltage = reading * 3.3;
 voltage /= 1024.0; 
 
 // now print out the temperature
float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                               //to degrees ((voltage - 500mV) times 100)
 
   // now convert to Fahrenheit
float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
/******* PRINT DEGREE C ****************/
/* NOTE: there was a weird thing with how this prints on the LCD*/
  // show that its in degree C
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(temperatureC); display.println("Degrees C");
  display.display();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(70,0);
  display.println(temperatureF);
  display.display();
/*****************************************/
/********* PRINT DEGREE F *****************/
 // show that its deg F
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(70, 8);
  display.println("Degrees F");
  display.display();
  
  // show the voltage
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,20);
  display.println("Voltage is: ");
  display.setCursor(70,20);
  display.println(voltage);
  display.display();
/********************************************/
 
  // delay(2000);  //waiting 2 seconds
  
  // this condition true if....
  if(temperatureF >= 95 && temperatureC >= 32.2222) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("TOO HOT!!!");
    display.display();
    //delay(2000);
  }
  
  display.clearDisplay();
}

void displaypr() {

photocellReading = analogRead(photocellPin);  
 
  Serial.print("Analog reading = ");
  Serial.println(photocellReading);     // the raw analog reading
 
  // LED gets brighter the darker it is at the sensor
  // that means we have to -invert- the reading from 0-1023 back to 1023-0
  photocellReading = 1023 - photocellReading;
  //now we have to map 0-1023 to 0-255 since thats the range analogWrite uses
  //LEDbrightness = map(photocellReading, 0, 1023, 0, 255);
  //analogWrite(LEDpin, LEDbrightness);
 
  //delay(100);
}
void tiltevent() {

int switchstate;
 
  reading = digitalRead(inPin);
 
  // If the switch changed, due to bounce or pressing...
  if (reading != previous) {
    // reset the debouncing timer
    time = millis();
  } 
 
  if ((millis() - time) > debounce) {
     // whatever the switch is at, its been there for a long time
     // so lets settle on it!
     switchstate = reading;
 
     // Now invert the output on the pin13 LED
    if (switchstate == HIGH)
      LEDstate = LOW;
    else
      LEDstate = HIGH;
  }
  digitalWrite(outPin, LEDstate);
 
  // Save the last reading so we keep a running tally
  previous = reading;
}
