/***************************************************
 * 
 * Adafruit IO Dashboard
 *   - https://io.adafruit.com/siletzspey/dashboards
 *   - Create feeds - cg-temp-1, cg-temp-2, ...
 *   - Create dashboards based on feeds
 *   - Use one feed to beam all sensor data in a CSV format
 *   - Use a few more feeds to beam select data that can be displayed in the MQTT service
 * 
 * Adafruit MQTT Library
 *   - Sketch > Include Library - Adafruit MQTT Library by Adafruit
 *   - https://github.com/adafruit/Adafruit_MQTT_Library
 *   - https://learn.adafruit.com/mqtt-adafruit-io-and-you/intro-to-adafruit-mqtt
 *   - https://learn.adafruit.com/adafruit-io-basics-temperature-and-humidity/adafruit-io-setup
 * 
 * Adafruit BME680 Library
 *   - Sketch > Include Library - Adafruit BME680 Library by Adafruit
 *   - Sketch > Include Library - Adafruit Unified Sensor
 *   - https://github.com/adafruit/Adafruit_BME680
 *   - https://learn.adafruit.com/adafruit-bme680-humidity-temperature-barometic-pressure-voc-gas
 *   
 * ESP8266 Board
 *   - Preferences > Additional Boards Manager URL - http://arduino.esp8266.com/stable/package_esp8266com_index.json
 *   - https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi/src
 *   - Multi WAP example - https://tttapa.github.io/ESP8266/Chap07%20-%20Wi-Fi%20Connections.html
 *
 * OneWire Examples
 *   - API DOCS https://www.pjrc.com/teensy/td_libs_OneWire.html
 *   - https://github.com/milesburton/Arduino-Temperature-Control-Library
 *   - https://github.com/milesburton/Arduino-Temperature-Control-Library/blob/master/examples/UserDataDemo/UserDataDemo.ino
 *
 * Description
 *   - Scan DS18B20 temperature devices via onewire every 15 seconds and uploaded via MQTT to io.adafruit.com
 *   - Scan BME680 atmospheric device via I2C every minute and upload via MQTT to io.adafruit.com
 *   - (future) Shower Miser - scan temp on hot water tank and maintain 100F degree temp
 *   - (future) Freeze Guard - scan temps and trigger micro PC fans to force warmer cabin air into crawl spaces with water tanks and lines
 *
 * Debug Notes
 *   - Without 3rd pin 3.3V, a DS18B20 returns tempC of 85 or 127 or other bogus values
 *   - ESP8266 Pin 15 has a built-in pull-down, which counteracts a 4.7K pull-up for parasitic power. Don't use Pin 15 for OneWire!
 *   
 * Debug Bogus Lines Seen
 *   0EADXG90R696TAYETQHZ42WTEH,51.5,49.0,47.7,49.0,38.6,50.1,-197.-6,,66.5,,34.6, , 48.8,66.5,994.0,528.5,113.2,1235641,2019-12-01 03:28:16 UTC
 *   0EADXGDK9RM52MGB21EEEWFTKK,51.3,49.0,47.5,49.0,38.6,50.1,261.2,,66.3,,34.6, , 48.8,66.5,994.0,527.9,113.7,1235641,2019-12-01 03:28:31 UTC
 */

#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// TJG deep sleep
extern "C" {
  #include "user_interface.h"
}

/****************************** DS18B20 OneWire Temp Sensor Setup *******************************/

// An ordered list of owned marker-dot-tagged devices that might be connected to this project
const int db18b20_sensor_count = 12;
DeviceAddress ds18b20_sensors[db18b20_sensor_count] = { // top dots & bottom dots on each sensor
  {0x28, 0xF5, 0xA6, 0xD3, 0x08, 0x00, 0x00, 0x84},     //      x              0  1 Black Vault
  {0x28, 0x15, 0xEA, 0xD3, 0x08, 0x00, 0x00, 0x65},     //     xx              1  2 Black Tank
  {0x28, 0x6C, 0x53, 0xD3, 0x08, 0x00, 0x00, 0xD7},     //    xxx              2  3 Gray Vault
  {0x28, 0x0D, 0xB1, 0xD2, 0x08, 0x00, 0x00, 0xF5},     //  X                  3  4 Gray-area fiberglass surface
  {0x28, 0xB1, 0x8E, 0x76, 0x08, 0x00, 0x00, 0xC3},     //  X   x              4  5 Fresh Vault
  {0x28, 0x16, 0x3B, 0x77, 0x08, 0x00, 0x00, 0x08},     //  X  xx              5  6 Sink Vault
  {0x28, 0x5A, 0xE6, 0xD4, 0x08, 0x00, 0x00, 0x93},     //  X xxx              6  7 Heater Output
  {0x28, 0xB9, 0xEC, 0x77, 0x08, 0x00, 0x00, 0x7F},     // XX                  7  8 Bedroom
  {0x28, 0x2B, 0x3F, 0x78, 0x08, 0x00, 0x00, 0x2A},     // XX   x              8  9 Cabin
  {0x28, 0x1F, 0x00, 0xD3, 0x08, 0x00, 0x00, 0xCD},     // XX  xx              9 10 Floor
  {0x28, 0xF1, 0x9F, 0x99, 0x0A, 0x00, 0x00, 0x9B},     // Waterproof Black   10 11 Outside
  {0x28, 0xCF, 0xD5, 0x17, 0x09, 0x00, 0x00, 0x19}      // Hi Temp White      11 12 Hot water tank frame
};
// To track owned devices determined to be connected at runtime
bool  ds18b20_sensors_valid[db18b20_sensor_count];

// Based on the average of all owned device at the same place/time/temp, an adjustment for each so that they all report the same
float ds18b20_sensors_adjust[db18b20_sensor_count] = {0.4705, 0.5079, -1.1045, -0.9921, 0.5830, 0.5079, -1.0295, 0.3579, 0.2830, 0.3579, 0.0580, 0.0000};

/*
 * Example copy-n-paste-ready adjustment arrays produced by list_and_reconcile_ow_devices()
 * 
 * {0.5932, 0.5557, -1.0193, -1.0568, 0.4807, 0.4057, -1.1318, 0.1807, -0.0443, 0.1057, 0.9307, 0.0000}    2/21/2019
 * {0.3648, 0.5148, -1.0602, -0.9477, 0.5523, 0.5523, -1.0227, 0.2898,  0.2523, 0.4398, 0.0648, 0.0000}    2/22/2019
 * {0.4261, 0.5011, -1.1489, -0.9614, 0.6136, 0.5011, -1.0739, 0.3886,  0.2761, 0.4261, 0.0511, 0.0000}    2/22/2019
 * {0.4705, 0.5079, -1.1045, -0.9921, 0.5830, 0.5079, -1.0295, 0.3579,  0.2830, 0.3579, 0.0580, 0.0000}    2/22/2019  using this one, getting even results
 */

// Pins and precision
#define ONEWIRE_PIN            2  // used to read, use to flash RED led on upload
#define TEMPERATURE_PRECISION 12  // 9 bits, 0.5C increments, 93.75ms; 10, 0.25C, 187.5ms; 11, 0.125C, 375ms; 12 0.0625C 750ms
#define SAMPLE_FREQ           15  // Sample every X seconds. Be careful not to exceed the FREE $0.00 rate limit of the Adafruit MQTT/IO Service

#define BLUE_LED_PIN           2  // same as ONEWIRE_PIN. ONEWIRE causes RED LED flash. Will also use RED LED to convey other good moments
#define RED_LED_PIN            0  // Will use red LED to convey bad moments

/****************************** BME680 I2C SCL/SDA Sensor Setup *******************************/

// Connect the SCK pin to the I2C clock SCL pin on your Arduino compatible
// Connect the SDI pin to the I2C data SDA pin on your Arduino compatible

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;            // init style for I2C. SPI has other init style
bool bme_valid;                 // if not seen, simply eliminate from output
float bme_temp_adjust = -1.3;   // by hand adjust wrt DS18B20 temp sensors


/************************* WiFi Setup ****************************************/

#include "userpass.h"

// #define WIFI_SSID    see userpass.h
// #define WIFI_PASS    see userpass.h

/************************* Adafruit.io MQTT Setup ****************************/

#include "userpass.h"

#define AIO_SERVER        "io.adafruit.com"
#define AIO_SERVERPORT    1883
// #define AIO_USERNAME   see userpass.h
// #define AIO_KEY        see userpass.h
// #define AIO_CLIENT_ID  see userpass.h

/************************* Global Constructs *********************************/

// Create the ESP8266 WiFiClient class to connect to the MQTT server
WiFiClient client;

// Create MQTT feeds
// - using PROGMEM crashes MQTT connection code. Known bug
const char     MQTT_SERVER[]   = AIO_SERVER;
const uint16_t MQTT_SERVERPORT = AIO_SERVERPORT;
const char     MQTT_USERNAME[] = AIO_USERNAME;
const char     MQTT_PASSWORD[] = AIO_KEY;
const char     MQTT_CLIENTID[] = AIO_CLIENT_ID;

// Create the MQTT client class and feeds1
// - cram all sensor values into a single MQTT feed as a CSV
// - also upload a kew key sensor values into an MQTT feed so they are directly usable in the MQTT Service
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

const char FEED_CG_TEMP_CSV[] = AIO_USERNAME "/feeds/cg-temp-csv";
const char FEED_CG_TEMP_1[]   = AIO_USERNAME "/feeds/cg-temp-1";
const char FEED_CG_TEMP_9[]   = AIO_USERNAME "/feeds/cg-temp-9";
const char FEED_CG_TEMP_11[]  = AIO_USERNAME "/feeds/cg-temp-11";
Adafruit_MQTT_Publish feed_cg_temp_csv = Adafruit_MQTT_Publish(&mqtt, FEED_CG_TEMP_CSV);
Adafruit_MQTT_Publish feed_cg_temp_1   = Adafruit_MQTT_Publish(&mqtt, FEED_CG_TEMP_1);
Adafruit_MQTT_Publish feed_cg_temp_9   = Adafruit_MQTT_Publish(&mqtt, FEED_CG_TEMP_9);
Adafruit_MQTT_Publish feed_cg_temp_11  = Adafruit_MQTT_Publish(&mqtt, FEED_CG_TEMP_11);

// Create the generic-onewire and Dallas-sensor classes
OneWire ow(ONEWIRE_PIN);
DallasTemperature sensors(&ow);

void setup() {
  
  Serial.begin(115200);
  Serial.println();
  Serial.println("Setup()");

#if false
  list_esp8266_details();
#endif

  list_and_reconcile_ow_devices();
  
  sensors.begin();
  for (int i = 0; i < db18b20_sensor_count; i++ ) {
    if (ds18b20_sensors_valid[i])
      sensors.setResolution(ds18b20_sensors[i], TEMPERATURE_PRECISION);
  }
  
  list_sensor_differences();

  if (!bme.begin()) {
   bme_valid = true;
   Serial.println("Could not find a valid BME680 sensor, check wiring!");
  }
  else {
    bme_valid = true;

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }
}
 
void loop() {
  float temps[db18b20_sensor_count];
  int itemps[db18b20_sensor_count];
  int itemp2;
  char tmpstr[20];
  char csvbuf[512];
  int feedfailurecnt;
  unsigned long timenow;
  
  Serial.println("loop()");

  while (1) {
    timenow = millis();
    // re-setup connections. If already connected, the calls return immediately
    WIFI_connect();
    MQTT_connect();
    
    // request temps
    // the request on pin 2 will also blink the ESP8266 RED led
    sensors.requestTemperatures();
    // sensors.requestTemperaturesByAddress(ds18b20_sensors[n]);

    // request enviro
    if (bme_valid) {
      if (! bme.performReading()) {
        Serial.println("Failed to perform BME680 reading");
      }
    }

    // put temps into CSV
    strcpy (csvbuf, "" );
    for ( int i = 0; i < db18b20_sensor_count; i++ ) {
      if (i != 0) strcat( csvbuf, "," );
      if (ds18b20_sensors_valid[i]) {
        temps[i] = sensors.getTempF(ds18b20_sensors[i]) + ds18b20_sensors_adjust[i];
        // cheesy trick to print a float to one decimal place, since Arduino's sprintf does not support a float
        itemps[i] = int ( temps[i] * 10 );
        sprintf( tmpstr, "%d.%d", itemps[i]/10, itemps[i]%10 );
        strcat( csvbuf, tmpstr );
      }
    }    

    // put enviros into CSV
    if (bme_valid) {
      // temp F
      itemp2 = (bme.temperature * 9/5 + 32)*10;   // also convert C to F
      itemp2 += bme_temp_adjust*10;
      sprintf( tmpstr, " , %d.%d", itemp2/10, itemp2%10 );   // spaces around , separate DS18B20 from BME680 columns
      strcat( csvbuf, tmpstr );

      // humidity %
      itemp2 = bme.humidity*10;
      sprintf( tmpstr, ",%d.%d", itemp2/10, itemp2%10 );
      strcat( csvbuf, tmpstr );
      
      // pressure hPa
      itemp2 = (bme.pressure/100.0)*10;
      sprintf( tmpstr, ",%d.%d", itemp2/10, itemp2%10 );
      strcat( csvbuf, tmpstr );

      // altitude feet
      itemp2 = bme.readAltitude(SEALEVELPRESSURE_HPA)*3.28084 * 10;   // also convert m to f
      sprintf( tmpstr, ",%d.%d", itemp2/10, itemp2%10 );
      strcat( csvbuf, tmpstr );

      // gas KOhms
      itemp2 = (bme.gas_resistance / 1000.0)*10;
      sprintf( tmpstr, ",%d.%d", itemp2/10, itemp2%10 );
      strcat( csvbuf, tmpstr );
    }

    Serial.print("temp csv: "); Serial.println( csvbuf );

    feedfailurecnt = 0;
    if (! feed_cg_temp_csv.publish(csvbuf)) {
      Serial.println("csv send failed");
      feedfailurecnt++;
    }
    
    if (ds18b20_sensors_valid[0]) {
      if (! feed_cg_temp_1.publish(temps[0])) {
        Serial.println("temp 1 send failed");
        feedfailurecnt++;
      }
    }

    if (ds18b20_sensors_valid[8]) {
      if (! feed_cg_temp_9.publish(temps[8])) {
        Serial.println("temp 9 send failed");
        feedfailurecnt++;
      }
    }

    if (ds18b20_sensors_valid[10]) {
      if (! feed_cg_temp_11.publish(temps[10])) {
        Serial.println("temp 11 send failed");
        feedfailurecnt++;
      }
    }

    delay(1000);
    if ( feedfailurecnt == 0 ) {
      // blink BLUE led x5 for successful sends
      pinMode(BLUE_LED_PIN, OUTPUT);
      delay(250);
      digitalWrite(BLUE_LED_PIN, HIGH); delay(100); digitalWrite(BLUE_LED_PIN, LOW); delay(100);
      digitalWrite(BLUE_LED_PIN, HIGH); delay(100); digitalWrite(BLUE_LED_PIN, LOW); delay(100);
      digitalWrite(BLUE_LED_PIN, HIGH); delay(100); digitalWrite(BLUE_LED_PIN, LOW); delay(100);
      digitalWrite(BLUE_LED_PIN, HIGH); delay(100); digitalWrite(BLUE_LED_PIN, LOW); delay(100);
      digitalWrite(BLUE_LED_PIN, HIGH); delay(100); digitalWrite(BLUE_LED_PIN, LOW); delay(100);
      pinMode(BLUE_LED_PIN, INPUT);
    }
    else {
      Serial.println("One or more sends failed");
      // blink red led x5 for failed sends
      pinMode(RED_LED_PIN, OUTPUT);
      delay(250);
      digitalWrite(RED_LED_PIN, HIGH); delay(100); digitalWrite(RED_LED_PIN, LOW); delay(100);
      digitalWrite(RED_LED_PIN, HIGH); delay(100); digitalWrite(RED_LED_PIN, LOW); delay(100);
      digitalWrite(RED_LED_PIN, HIGH); delay(100); digitalWrite(RED_LED_PIN, LOW); delay(100);
      digitalWrite(RED_LED_PIN, HIGH); delay(100); digitalWrite(RED_LED_PIN, LOW); delay(100);
      digitalWrite(RED_LED_PIN, HIGH); delay(100); digitalWrite(RED_LED_PIN, LOW); delay(100);
      pinMode(RED_LED_PIN, INPUT);
    }

    // On ESP8266, yield() lets system context do a little WiFi and other work if needed
    while( millis() < timenow + SAMPLE_FREQ * 1000 ) {
        yield();
    }
  }

#if false
  // TJG deep sleep
  Serial.println("Deep sleep start, 10 seconds");
  // ESP.deepSleep(microseconds, mode)
  //    WAKE_RF_DEFAULT  - depending on init data, RF_CL or not after wake
  //    WAKE_RFCAL       - RF_CL after wake, large current
  //    WAKE_NO_RFCAL    - no RF_CL after wake, small current
  //    WAKE_RF_DISABLED - disable RF after wake, smallest current
  ESP.deepSleep(10 * 1000000, WAKE_RF_DEFAULT);
  Serial.println("Deep sleep end");
  // delay(3000);
#endif

}



void WIFI_connect() {

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.print("WiFi connecting to: ");  Serial.print(WIFI_SSID);  Serial.print(" ");

  // WIFI_OFF=0, WIFI_STA=1, WIFI_AP=2, WIFI_AP_STA=3
  WiFi.mode(WIFI_STA);

  // WiFi.setOutputPower( float dBm );  20.5dBm max, 0dBm min

  // WIFI_NONE_SLEEP=0 - WiFi is off-off
  // WIFI_LIGHT_SLEEP=2 - CPU may be suspended, no data, receive beacons, 0.9ma
  // WIFI_MODEM_SLEEP=3 - CPU must be on, no data transmissions, receive beacons, 15ma
  // WiFi.setSleepMode( WIFI_NONE_SLEEP );

  // WiFi.forceSleepBegin( uint32 sleepUs );  sets WIFI_OFF and more
  // WiFi.forceSleepWake(); sets last mode and more, and connects()
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");

    // blink red led once to indicate WiFi connecting issue
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, HIGH); delay(200); digitalWrite(RED_LED_PIN, LOW);
    pinMode(RED_LED_PIN, INPUT);
    delay(500);
  }
  Serial.println();

  Serial.print("WiFi connected as: "); Serial.println(WiFi.localIP());
}

// connect to adafruit io via MQTT
void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }
  
  Serial.print("MQTT connecting to: "); Serial.print(MQTT_SERVER); Serial.print(" as "); Serial.println(MQTT_USERNAME);

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("MQTT connection retry in 5 seconds...");
    mqtt.disconnect();

    // blink red led twice to indicate MQTT connecting issue
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, HIGH); delay(200); digitalWrite(RED_LED_PIN, LOW); delay(200);
    digitalWrite(RED_LED_PIN, HIGH); delay(200); digitalWrite(RED_LED_PIN, LOW);
    pinMode(RED_LED_PIN, INPUT);

    delay(5000);
  }

  Serial.println("MQTT connected");
}

/**************************************************************************
 * list_and_reconcile_ow_devices()
 * 
 * - find all devices on the wire 
 * - see if each found device is in the known-ordered list
 */
void list_and_reconcile_ow_devices()
{
  uint8_t address[8];

  // declare all known-ordered devices as not currently present on the wire
  for (int i = 0; i < db18b20_sensor_count; i++) {
    ds18b20_sensors_valid[i] = false;
  }
  
  Serial.println("List of All Devices on Wire");
  ow.reset();
  if (ow.search(address))
  {
    do {
      // List address of device just found on wire
      Serial.print("  {");
      for (int i = 0; i < 8; i++)
      {
        Serial.print("0x");
        if (address[i] < 0x10) Serial.print("0");
        Serial.print(address[i], HEX);
        if (i < 7) Serial.print(", ");
      }
      Serial.println("}");

      // Reconcile with known-ordered list of devices
      for (int i = 0; i < db18b20_sensor_count; i++) {
        int hopeful = 0;
        for (int j = 0; j < 8; j++) {
          if (address[j] == ds18b20_sensors[i][j]) {
            hopeful++;
          }
        }
        if (hopeful == 8) {
          ds18b20_sensors_valid[i] = true;
          break;
        }
      }
    } while (ow.search(address));
  }

  Serial.println("List of Known-Ordered Devices on Wire");
  for (int i = 0; i < db18b20_sensor_count; i++) {
    Serial.print("  ["); Serial.print(i); Serial.print("] {");
    for (int j = 0; j < 8; j++) {
      Serial.print("0x");
      if (ds18b20_sensors[i][j] < 0x10) Serial.print("0");
      Serial.print(ds18b20_sensors[i][j], HEX);
      if (j < 7) Serial.print(", ");
    }
    Serial.print("}");

    if ( ds18b20_sensors_valid[i] ) {
      Serial.println(" online");
    }
    else {
      Serial.println(" not found");
    }
  }
}

void list_sensor_differences() {
  float temps[db18b20_sensor_count];
  float totaltemp;
  int totaltemps;
  int samples = 3;

  Serial.println("List of Temperature Sensor Adjustments");
  
  for (int i = 0; i < db18b20_sensor_count; i++ ) {
    temps[i] = 0.0;
  }
  
  totaltemp = 0.0;
  totaltemps = 0;
  for (int j = 0; j < samples; j++) {
    sensors.requestTemperatures();

    for (int i = 0; i < db18b20_sensor_count; i++ ) {
      if (ds18b20_sensors_valid[i]) {
        float temp = sensors.getTempF(ds18b20_sensors[i]);  
        temps[i] += temp;
        totaltemp += temp;
        totaltemps++;
      }
    }
    delay(2000);
  }

  // float ds18b20_sensors_adjust[db18b20_sensor_count] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1};

  Serial.print("  {");
  for ( int i = 0; i < db18b20_sensor_count; i++ ) {
    if (ds18b20_sensors_valid[i]) {
      if (i > 0)
        Serial.print(", ");
      temps[i] = totaltemp / totaltemps - temps[i]/samples;
      Serial.print(temps[i], 4);
    }
    else {
      if (i > 0)
        Serial.print(", 0.0000");
    }
  }
  Serial.println("}");
}

void list_esp8266_details()
{
  Serial.print( "getVcc(): " ); Serial.println( ESP.getVcc() );
  Serial.print( "getFreeHeap(): " ); Serial.println( ESP.getFreeHeap() );
  Serial.print( "getChipId(): " ); Serial.println( ESP.getChipId() );
  Serial.print( "getSdkVersion(): " ); Serial.println( ESP.getSdkVersion() );
  Serial.print( "getBootVersion(): " ); Serial.println( ESP.getBootVersion() );
  Serial.print( "getBootMode(): " ); Serial.println( ESP.getBootMode() );
  Serial.print( "getCpuFreqMHz(): " ); Serial.println( ESP.getCpuFreqMHz() );
  Serial.print( "getFlashChipId(): " ); Serial.println( ESP.getFlashChipId() );
  Serial.print( "getFlashChipRealSize(): " ); Serial.println( ESP.getFlashChipRealSize() );
  Serial.print( "getFlashChipSize(): " ); Serial.println( ESP.getFlashChipSize() );
  Serial.print( "getFlashChipSpeed(): " ); Serial.println( ESP.getFlashChipSpeed() );
  Serial.print( "getFlashChipMode(): " ); Serial.println( ESP.getFlashChipMode() );
  Serial.print( "getFlashChipSizeByChipId(): " ); Serial.println( ESP.getFlashChipSizeByChipId() );
  Serial.print( "getSketchSize(): " ); Serial.println( ESP.getSketchSize() );
  Serial.print( "getFreeSketchSpace(): " ); Serial.println( ESP.getFreeSketchSpace() );
  Serial.print( "getResetReason(): " ); Serial.println( ESP.getResetReason() );
  Serial.print( "getResetInfo(): " ); Serial.println( ESP.getResetInfo() );
  Serial.print( "getCycleCount(): " ); Serial.println( ESP.getCycleCount() );
  // Serial.print( "__get_rf_mode(): " ); Serial.println( ESP.__get_rf_mode() );
  // Serial.print( "__get_adc_mode(): " ); Serial.println( ESP.__get_adc_mode() );  

  Serial.print( "WiFi.localIP(): " ); Serial.println( WiFi.localIP() );
  Serial.print( "WiFi.subnetMask(): " ); Serial.println( WiFi.subnetMask() );
  Serial.print( "WiFi.gatewayIP(): " ); Serial.println( WiFi.gatewayIP() );

  Serial.print( "WiFi.SSID(): " ); Serial.println( WiFi.SSID() );
  Serial.print( "WiFi.RSSI(): " ); Serial.println( WiFi.RSSI() );
  byte mac[6]; WiFi.macAddress(mac);
  Serial.print("WiFi.macAddress(): ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);

  WiFi.printDiag(Serial);
}
