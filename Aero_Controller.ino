#include <Wire.h>
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>          // https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/
#include <Cycle_Timer.h>

/*-----( Definitions )-----*/
#define TOTAL_CIRCUITS 2 // <-- CHANGE THIS | set how many I2C circuits are attached to the Tentacle

#define LCD_ADDRESS	0x3F
#define LCD_EN 		2
#define LCD_RW 		1
#define LCD_RS 		0
#define LCD_D4 		4
#define LCD_D5 		5
#define LCD_D6 		6
#define LCD_D7 		7
#define LCD_BL 		3
#define LCD_BL_POL 	POSITIVE
#define LCD_BUFFER_SIZE	5

#define DISPLAY_DECIMALS 1

/*-----( Declare Constants )-----*/
const int     encoderFactor             = 4;
const uint8_t NozzlesPin 		= 9;
const int32_t initialTimerOn 		= 5;
const int32_t initialTimerOff 		= 10;
const uint8_t floatPin                  = 2;
const uint8_t mainsWaterPin             = 8;

const unsigned int baud_host 		= 9600; // set baud rate for host serial monitor(pc/mac/other)
const unsigned int send_readings_every 	= 2000; // set at what intervals the readings are sent to the computer (NOTE: this is not the frequency of taking the readings!)

const unsigned int reading_delay 	= 1000; // time to wait for the circuit to process a read command. datasheets say 1 second.

/*-----( Declare Objects )-----*/
Encoder 		onTimeEncoder(6, 5);
Encoder 		offTimeEncoder(4, 3);

CycleTimer 		Nozzles;
LiquidCrystal_I2C	lcd(LCD_ADDRESS, LCD_EN, LCD_RW, LCD_RS, LCD_D4, LCD_D5, LCD_D6, LCD_D7, LCD_BL, LCD_BL_POL); // LCD module

/*-----( Declare Variables )-----*/
unsigned long next_reading_time; // holds the time when the next reading should be ready from the circuit
boolean request_pending = false; // wether or not we're waiting for a reading

unsigned long next_serial_time;

char sensordata[30]; // A 30 byte character array to hold incoming data from the sensors
byte sensor_bytes_received = 0; // We need to know how many characters bytes have been received
byte code = 0;                            // used to hold the I2C response code.
byte in_char = 0; // used as a 1 byte buffer to store in bound bytes from the I2C Circuit.

int channel_ids[] = { 99, 100 }; // <-- CHANGE THIS. A list of I2C ids that you set your circuits to.
char *channel_names[] = { "PH", "EC" }; // <-- CHANGE THIS. A list of channel names (must be the same order as in channel_ids[]) - only used to designate the readings in serial communications
String readings[TOTAL_CIRCUITS]; // an array of strings to hold the readings of each channel
int channel = 0; // INT pointer to hold the current position in the channel_ids/channel_names array

long positionOnEncoder = -999;
long positionOffEncoder = -999;
long positionLeft = -999;
long positionRight = -999;

int floatLevel = LOW;

void setup() {

  pinMode(NozzlesPin, OUTPUT);
  Nozzles.setPin(NozzlesPin);
  Nozzles.setOnTime(initialTimerOn);
  Nozzles.setOffTime(initialTimerOff);
  Nozzles.update();

  onTimeEncoder.write(initialTimerOn * encoderFactor);
  offTimeEncoder.write(initialTimerOff * encoderFactor);

  Serial.begin(baud_host); // Set the hardware serial port.
  Wire.begin(); // enable I2C port.

  pinMode(floatPin, INPUT_PULLUP);
  pinMode(mainsWaterPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(floatPin), floatPinISR, CHANGE);

  lcd.begin(20, 4);        // initialize the lcd for 20 chars 4 lines, turn on backlight
  intro();

  floatPinISR();

  next_serial_time = millis() + send_readings_every; // calculate the next point in time we should do serial communications
}

void loop() {
  do_cycle_timer();
  do_sensor_readings();
  do_serial();
}

void do_cycle_timer() {

  String displayString;

  long newOn = onTimeEncoder.read() / encoderFactor;
  if (newOn < 0) {
    onTimeEncoder.write(0);
    newOn = 0;
  }
  if (newOn != positionOnEncoder) {
    Nozzles.setOnTime(newOn);
    lcd.setCursor(0, 3);
    Serial.print("Left = ");
    Serial.print(newOn);
    Serial.println();
    lcd.print("On:"); lcd.print(newOn); lcd.print("s");
    positionOnEncoder = newOn;
  }
  long newOff = (offTimeEncoder.read() / encoderFactor);
  if (newOff < 0) {
    offTimeEncoder.write(0);
    newOff = 0;
  }
  if (newOff != positionOffEncoder) {
    Nozzles.setOffTime(newOff);
    lcd.setCursor(10, 3);
    Serial.print("Right = ");
    Serial.print(newOff);
    Serial.println();
    lcd.print("Off:"); lcd.print(newOff); lcd.print("s");
    positionOffEncoder = newOff;
  }
  Nozzles.update();
}

// take sensor readings in a "asynchronous" way
void do_sensor_readings() {
  if (request_pending) {                          // is a request pending?
    if (millis() >= next_reading_time) { // is it time for the reading to be taken?
      receive_reading();                // do the actual I2C communication
    }
  } else {                                        // no request is pending,
    channel = (channel + 1) % TOTAL_CIRCUITS; // switch to the next channel (increase current channel by 1, and roll over if we're at the last channel using the % modulo operator)
    request_reading();                    // do the actual I2C communication
  }
}

// Request a reading from the current channel
void request_reading() {
  request_pending = true;
  Wire.beginTransmission(channel_ids[channel]); // call the circuit by its ID number.
  Wire.write('r');        		        // request a reading by sending 'r'
  Wire.endTransmission();          	       // end the I2C data transmission.
  next_reading_time = millis() + reading_delay; // calculate the next time to request a reading
}

// Receive data from the I2C bus
void receive_reading() {
  sensor_bytes_received = 0;                        // reset data counter
  memset(sensordata, 0, sizeof(sensordata));        // clear sensordata array;

  Wire.requestFrom(channel_ids[channel], 48, 1); // call the circuit and request 48 bytes (this is more then we need).
  code = Wire.read();

  while (Wire.available()) {          // are there bytes to receive?
    in_char = Wire.read();            // receive a byte.

    if (in_char == 0) {  // if we see that we have been sent a null command.
      Wire.endTransmission();         // end the I2C data transmission.
      break;                       // exit the while loop, we're done here
    } else {
      sensordata[sensor_bytes_received] = in_char; // load this byte into our array.
      sensor_bytes_received++;
    }
  }

  switch (code) {           // switch case based on what the response code is.
    case 1:                      // decimal 1  means the command was successful.
      readings[channel] = sensordata;
      break;                        	    // exits the switch case.

    case 2:                        	  // decimal 2 means the command has failed.
      readings[channel] = "error: command failed";
      break;                         	    // exits the switch case.

    case 254: // decimal 254  means the command has not yet been finished calculating.
      readings[channel] = "reading not ready";
      break;                         	    // exits the switch case.

    case 255:             // decimal 255 means there is no further data to send.
      readings[channel] = "error: no data";
      break;                         	    // exits the switch case.
  }
  request_pending = false; // set pending to false, so we can continue to the next sensor
}

// do serial communication in a "asynchronous" way
void do_serial() {

  if (millis() >= next_serial_time) { // is it time for the next serial communication?
    Serial.print("On = ");
    Serial.print(positionOnEncoder);
    Serial.print("    Off = ");
    Serial.println(positionOffEncoder);
    for (int i = 0; i < TOTAL_CIRCUITS; i++) { // loop through all the sensors
      Serial.print(channel_names[i]);                // print channel name
      Serial.print(":\t");
      Serial.println(readings[i]);             // print the actual reading
    }
    // display readings to LCD
    lcd.setCursor((int) ((10 - 3 - readings[0].length()) / 2), 1);
    lcd.print("pH:"); lcd.print(readings[0]);
    lcd.setCursor(10 + (int) ((10 - 3 - readings[1].indexOf(',')) / 2), 1);
    lcd.print("EC:"); lcd.print(readings[1].substring(0, readings[1].indexOf(',')));

    lcd.setCursor(0, 2);
    switch (!floatLevel) {
      case HIGH:
        lcd.print("  Float Level:HIGH");
        break;
      case LOW:
        lcd.print("  Float Level: LOW");
        break;
    }
    next_serial_time = millis() + send_readings_every;
  }
}

void intro() {

  Serial.flush();
  Serial.println(F("Aeroponics Controller"));
  //serialPrintDivider();

  lcd.setBacklight(HIGH); // NOTE: You can turn the backlight off by setting it to LOW instead of HIGH
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Aero Controller");

}

void floatPinISR() {
  floatLevel = !digitalRead(floatPin);

  switch (floatLevel) {
    case HIGH:
      //Full
      digitalWrite(mainsWaterPin, floatLevel);
      break;
    case LOW:
      //Needs Filling
      digitalWrite(mainsWaterPin, floatLevel);
      break;
  }
}
