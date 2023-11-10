// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include <bluefruit.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        8 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 2 // Popular NeoPixel ring size

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

// what's the name of the hardware serial port?
#define GPSSerial Serial1

#define TILTSWITCH A1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

#define TILTINCREASE 1
#define TILTDECREASE 2
#define TILTONTHRESHOLD 15
#define TILTLIMIT 20

uint32_t tilt = 0;

BLEService        bleService = BLEService(0x0070);
BLECharacteristic latitude = BLECharacteristic(0x0071);
BLECharacteristic longitude = BLECharacteristic(0x0072);
BLECharacteristic trigger = BLECharacteristic(0x007F);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

void setupTilt()
{
  pinMode(TILTSWITCH, INPUT);
}

void setupBLEService(void)
{
  bleService.begin();

  latitude.setProperties(CHR_PROPS_NOTIFY);
  latitude.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  latitude.setFixedLen(4);
  latitude.setUserDescriptor("latitude");
  latitude.begin();

  longitude.setProperties(CHR_PROPS_NOTIFY);
  longitude.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  longitude.setFixedLen(4);
  longitude.setUserDescriptor("longitude");
  longitude.begin();

  trigger.setProperties(CHR_PROPS_NOTIFY);
  trigger.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  trigger.setFixedLen(1);
  trigger.setUserDescriptor("trigger");
  trigger.begin();
}

void updateTilt()
{
  if (digitalRead(TILTSWITCH) == HIGH && tilt <= TILTLIMIT) {
    tilt+=TILTINCREASE;
  }
  else if (tilt > 0) {
    tilt -= min(tilt, TILTDECREASE);
  }
}

bool isTilt()
{
  return (tilt >= TILTONTHRESHOLD);
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(bleService);

  // Include Name
  Bluefruit.Advertising.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void setup()
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  while ( !Serial ) delay(10); 

  delay(1000);
  Serial.println("Adafruit GPS library basic parsing test!");
  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.setName("Vending Machine");
  Bluefruit.begin();

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  setupTilt();
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(128);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  setupBLEService();
  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();
  Bluefruit.Advertising.start();
  Serial.println("DOne");
}

void transmitLocation() {
  latitude.notify32((uint32_t) GPS.latitude_fixed);
  longitude.notify32((uint32_t) GPS.longitude_fixed);
}

void transmitTrigger(bool value) {
  trigger.notify8((uint8_t) value); 
}

void loop() // run over and over again
{
  pixels.clear();

  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  updateTilt();


  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 200) {
    timer = millis(); // reset the timer
  
    if (GPS.fix) {
      pixels.setPixelColor(0, pixels.Color(0, 150, 0));

      if (Bluefruit.connected()) {
        transmitLocation();
      }
    } else {
      pixels.setPixelColor(0, pixels.Color(150, 0, 0));
    }

    if (isTilt()) {
      pixels.setPixelColor(1, pixels.Color(0,150,0));
      transmitTrigger(true);
    } else {
      transmitTrigger(false);
    }
  }

  pixels.show();
}