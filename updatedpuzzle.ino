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
/**************** BALL TILT ******************/
int inPin = 5;         // the number of the input pin
int outPin = 30;       // the number of the output pin
int LEDstate = HIGH;   // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
/**************** BLUETOOTH ******************/
// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;
// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;
// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 50;    // the debounce time, increase if the output flickers
/**************** Temperature stuff ***********/
float temperatureF;
float temperatureC;
/******************** PARAMS *******************/
int temp1;
int temp2;
int photo1;
int photo2;
bool tilted;
int wincount;
/***********************************************/
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
  Serial.println("Welcome to Mario's Puzzle Box");
  Serial.println("---------------------------\n");

  // Initialize blinkTimer for 1000 ms and start it
  blinkTimer.begin(1000, blink_timer_callback);
  blinkTimer.start();

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Marios Puzzle Box");
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");

  //loadparam();

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done

  /************** BALL TILT STUFF ************/
  pinMode(inPin, INPUT);
  digitalWrite(inPin, HIGH);   // turn on the built in pull-up resistor
  pinMode(outPin, OUTPUT);
  /*******************************************/

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();

  // Clear the buffer.
  display.clearDisplay();
  randomSeed(analogRead(28));
  initializer();
  printer();
}

void loop() {
  // this would be our bluetooth loader
  //blue();
  /*while (wincount != 3) {
    tiltevent();
    displaypr();
    displaytemp();
  } */
    tiltevent();
    displaypr();
    displaytemp();
 

  
}

void blue () {

  // Forward data from HW Serial to BLEUART
  while (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);

    uint8_t buf[64];
    int count = Serial.readBytes(buf, sizeof(buf));
    bleuart.write( buf, count );
    Serial.write(count);
  }

  // Forward from BLEUART to HW Serial
  while ( bleuart.available() )
  {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();
  }

  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");

}

void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  digitalToggle(LED_RED);
}

void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}

void displaytemp() {

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
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(temperatureC); display.println("Degrees C");
  display.display();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(70, 0);
  display.println(temperatureF);
  display.display();
  /*****************************************/
  /********* PRINT DEGREE F ****************/
  // show that its deg F
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(70, 8);
  display.println("Degrees F");
  display.display();

  // show the voltage
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  display.println("Voltage is: ");
  display.setCursor(70, 20);
  display.println(voltage);
  display.display();
  /******************************************/

  // delay(2000);  //waiting 2 seconds

  // this condition true if....
  if (temperatureF >= temp1 && temperatureC <= temp2) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("YOU GOT THE RIGHT TEMP!");
    display.display();
    //delay(2000);
    wincount++;
  }

  display.clearDisplay();
}

void displaypr() {

  photocellReading = analogRead(photocellPin);

  // Serial.print("Analog reading = ");
  // Serial.println(photocellReading);     // the raw analog reading

  if ((photocellReading >= photo1) && (photocellReading <= photo2)) {
    Serial.println("The PR value is correct!");
    wincount++;
  }
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
// intialize vars...
void initializer() {
  temp1 = random(40, 60);
  temp2 = random(61, 80);
  photo1 = random(10, 20);
  photo2 = random(30, 40);

  int ballset = random(0, 10);
  if ((ballset % 2) == 0) {
    tilted = HIGH;
  } else {
    tilted = LOW;
  }
}
// display vars
void printer() {

  Serial.println(temp1);
  Serial.println(temp2);
  Serial.println(photo1);
  Serial.println(photo2);
  Serial.println(tilted);

}
void didyouwin() {

  // when wincount = 3, you win.
  c = 1;
  switch (c) {
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    default:
      Serial.println("You didnt win yet!");
  }
}

void tempcheck() {



}
void prcheck() {


}
void tiltcheck() {


}
