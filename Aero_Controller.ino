#include <Wire.h>
#define HALF_STEP
#include <Rotary.h>
#include <Cycle_Timer.h>
#include <util/atomic.h>
#include <LiquidCrystal_I2C.h>          // http://members.iinet.net.au/~vanluynm/Down/LiquidCrystal_I2C.zip

/*-----( Definitions )-----*/
#define TOTAL_CIRCUITS 2 // <-- CHANGE THIS | set how many I2C circuits are attached to the Tentacle

/*-----( Declare Constants )-----*/
const int32_t initialTimerOn				= 5;
const int32_t initialTimerOff				= 10;

const uint8_t floatPin					= 2;
const uint8_t mainsWaterPin				= 8;
const uint8_t NozzlesPin				= 9;

const unsigned long READ_ENCODERS_EVERY			= 0;
const unsigned long READ_SENSORS_EVERY			= 800;
const unsigned long UPDATE_NOZZLES_EVERY		= 20;
const unsigned long WRITE_SERIAL_MONITOR_EVERY		= 1000;
const unsigned long WRITE_DISPLAY_EVERY			= 250;

/*-----( Declare Objects )-----*/
Rotary onTimeEncoder(6, 5);
Rotary offTimeEncoder(4, 3);

CycleTimer Nozzles;

LiquidCrystal_I2C lcd(0x3F, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

/*-----( Declare Variables )-----*/
boolean requestPending = false; // wether or not we're waiting for a reading

char sensordata[30]; // A 30 byte character array to hold incoming data from the sensors
byte sensorBytesReceived = 0; // We need to know how many characters bytes have been received
byte code = 0;                            // used to hold the I2C response code.
byte in_char = 0; // used as a 1 byte buffer to store in bound bytes from the I2C Circuit.

int channel_ids[] = { 99, 100 }; // <-- CHANGE THIS. A list of I2C ids that you set your circuits to.
char *channelNames[] = { "PH", "EC" }; // <-- CHANGE THIS. A list of channel names (must be the same order as in channel_ids[]) - only used to designate the readings in serial communications
String readings[TOTAL_CIRCUITS]; // an array of strings to hold the readings of each channel
int channel = 0; // INT pointer to hold the current position in the channel_ids/channelNames array

long positionOnEncoder = initialTimerOn;
long positionOffEncoder = initialTimerOff;

int floatLevel = HIGH;

char lcd_buffer[20];             // LCD buffer used for the better string format to LCD

unsigned long lastSensorRead;
unsigned long lastEncoderRead;
unsigned long lastNozzlesUpdate;
unsigned long lastSerialMonitorWrite;
unsigned long lastDisplayWrite;

void setup() {

  Wire.begin();
  Serial.begin(115200);

  initEncoders();
  initFloat();
  initNozzles();
  initSerial();
  initDisplay();

  // initialise most recent update events
  unsigned long now = millis();
  lastSensorRead = now;
  lastEncoderRead = now;
  lastNozzlesUpdate = now;
  lastSerialMonitorWrite = now;
  lastDisplayWrite = now;

}

void doFunctionAtInterval(void (*callBackFunction)(), unsigned long *lastEvent, unsigned long Interval) {

  unsigned long now = millis();

  if ((now - *lastEvent) >= Interval) {
    callBackFunction();
    *lastEvent = now;
  }

}

void loop() {

  doFunctionAtInterval(readSensors, &lastSensorRead, READ_SENSORS_EVERY);
  readEncoders();
  doFunctionAtInterval(updateNozzles, &lastNozzlesUpdate, UPDATE_NOZZLES_EVERY);
  doFunctionAtInterval(writeDisplay, &lastDisplayWrite, WRITE_DISPLAY_EVERY);
  doFunctionAtInterval(writeSerial, &lastSerialMonitorWrite, WRITE_SERIAL_MONITOR_EVERY);

}

void initSensors() {

  readSensors();

}

void readSensors() {

  if (requestPending) {                          // is a request pending?
    receiveReading();                // do the actual I2C communication
  } else {                                        // no request is pending,
    channel = (channel + 1) % TOTAL_CIRCUITS; // switch to the next channel (increase current channel by 1, and roll over if we're at the last channel using the % modulo operator)
    requestReading();                    // do the actual I2C communication
  }

}

// Request a reading from the current channel
void requestReading() {

  requestPending = true;
  Wire.beginTransmission(channel_ids[channel]); // call the circuit by its ID number.
  Wire.write('r');        		        // request a reading by sending 'r'
  Wire.endTransmission();          	       // end the I2C data transmission.

}

// Receive data from the I2C bus
void receiveReading() {

  sensorBytesReceived = 0;                        // reset data counter
  memset(sensordata, 0, sizeof(sensordata));        // clear sensordata array;

  Wire.requestFrom(channel_ids[channel], 48, 1); // call the circuit and request 48 bytes (this is more then we need).
  code = Wire.read();

  while (Wire.available()) {          // are there bytes to receive?
    in_char = Wire.read();            // receive a byte.

    if (in_char == 0) {  // if we see that we have been sent a null command.
      Wire.endTransmission();         // end the I2C data transmission.
      break;                       // exit the while loop, we're done here
    } else {
      sensordata[sensorBytesReceived] = in_char; // load this byte into our array.
      sensorBytesReceived++;
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
  requestPending = false;

}

void initEncoders() {

  readEncoders();

}

void readEncoders() {

  unsigned char result;

  result = onTimeEncoder.process();
  if (result) {
    switch (result) {
      case DIR_CW:
        positionOnEncoder++;
        break;
      case DIR_CCW:
        positionOnEncoder = (positionOnEncoder == 0) ? 0 : positionOnEncoder - 1;
        break;
    }
    Serial.print("On: "); Serial.println(positionOnEncoder);
  } 
  result = offTimeEncoder.process();
  if (result) {
    switch (result) {
      case DIR_CW:
        positionOffEncoder++;
        break;
      case DIR_CCW:
        positionOffEncoder = (positionOffEncoder == 0) ? 0 : positionOffEncoder - 1;
        break;
    }
    Serial.print("Off: "); Serial.println(positionOffEncoder);
  }

}

void initFloat() {

  pinMode(floatPin, INPUT_PULLUP);
  pinMode(mainsWaterPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(floatPin), floatPinISR, CHANGE);
  floatPinISR();

}

void initNozzles() {

  pinMode(NozzlesPin, OUTPUT);
  Nozzles.setPin(NozzlesPin);
  updateNozzles();

}

void updateNozzles() {

  Nozzles.setOnTime(positionOnEncoder);
  Nozzles.setOffTime(positionOffEncoder);
  Nozzles.update();

}

void initSerial() {

  Serial.println("Aero Controller");
  serialDivider();

}

void writeSerial() {

  int localFloatLevel;
  long localPositionOnEncoder;
  long localPositionOffEncoder;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    localFloatLevel = floatLevel;
    localPositionOnEncoder = positionOnEncoder;
    localPositionOffEncoder = positionOffEncoder;
  }

  for (int i = 0; i < TOTAL_CIRCUITS; i++) { // loop through all the sensors
    Serial.print(channelNames[i]);                // print channel name
    Serial.print(":\t");
    Serial.println(readings[i]);             // print the actual reading
  }
  Serial.print("On:"); Serial.print(localPositionOnEncoder); Serial.print("s ");
  Serial.print("Off:"); Serial.print(localPositionOffEncoder); Serial.println("s");

  Serial.print("Resevoir: "); Serial.println((localFloatLevel) ? "Full" : "Filling");
  serialDivider();

}

void serialDivider() {

  Serial.println("------------------------------------------");

}

void initDisplay() {

  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("Aero Controller");
  delay(1000);
  writeDisplay();

}

void writeDisplay() {

  int localFloatLevel;
  long localPositionOnEncoder;
  long localPositionOffEncoder;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    localFloatLevel = floatLevel;
    localPositionOnEncoder = positionOnEncoder;
    localPositionOffEncoder = positionOffEncoder;
  }

  lcd.setCursor(0, 2);
  lcd.print("Resevoir:");
  lcd.println((localFloatLevel) ? "       Full" : "    Filling");
  lcd.setCursor(0, 1);
  lcd.print(" pH:"); lcd.print(readings[0]);
  lcd.setCursor(10, 1);
  lcd.print(" EC:"); lcd.print(readings[1]);
  lcd.setCursor(2, 3);
  dtostrf((double) localPositionOnEncoder, 3, 0, lcd_buffer);
  lcd.print("On:"); lcd.print(lcd_buffer); lcd.print("s");
  lcd.setCursor(10, 3);
  dtostrf((double) localPositionOffEncoder, 3, 0, lcd_buffer);
  lcd.print("Off:"); lcd.print(lcd_buffer); lcd.print("s");

}

void floatPinISR() {

  floatLevel = digitalRead(floatPin);
  digitalWrite(mainsWaterPin, !floatLevel);
  //turnOnBacklight();

}
