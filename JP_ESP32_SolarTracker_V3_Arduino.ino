
#include <WiFi.h>
//#include <HTTPClient.h>
#include <WiFiClient.h>

#include <TimeLib.h>
#include <SolarCalculator.h>

//BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//Json
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

//UDP
#include <WiFiUdp.h>

///stepper motors                 //https://www.airspayce.com/mikem/arduino/AccelStepper/
#include <AccelStepper.h>         //http://www.schmalzhaus.com/EasyDriver/index.html

// The Az Stepper pins.
#define STEPPER_AZ_DIR_PIN 32
#define STEPPER_AZ_STEP_PIN 33
// The El stepper pins
#define STEPPER_EL_DIR_PIN 23
#define STEPPER_EL_STEP_PIN 19

#define STEPPER_SLEEP_PIN 25

#define EL_HOME_PIN 5
#define AZ_HOME_PIN 18
#define VCC_PIN 36

#define SOLAR_PANEL_R 540   //expressed in mm
#define EL_HOME 20          //expressed in ° (stop move El axis if sun below this value)
#define EL_LIMIT 20         //expressed in ° (go to next morning if sun below this value)
#define AZ_HOME 180.        //expressed in ° 
#define AZ_LINEAR           //Azimut is driven by a linear actuator

// Define steppers and the pins they will use
AccelStepper stepperEl(AccelStepper::DRIVER, STEPPER_EL_STEP_PIN, STEPPER_EL_DIR_PIN);
AccelStepper stepperAz(AccelStepper::DRIVER, STEPPER_AZ_STEP_PIN, STEPPER_AZ_DIR_PIN);

double sunElevation;
double sunAzimuth;
double TargetElevation;
double TargetAzimuth;
double previousAz, currentAz, previousAzLength, currentAzLength, previousEl, previousElLength, currentEl, currentElLength;
String mode = "s";                                  //"h" for heliostat, "s" for sunTracker
enum {tracking, morning, sleeping, homing, initializing };        // 0 : tracking, 1 : morning, 2 : sleeping, 3 : homing, 4 : initializing (tracking is normal mode. Sleeping is during night)
int heliostatStatus = sleeping;

float wormGearRatioEl = 1.0 / 0.8;                  //for M5 thread, 0.8mm per turn
int stepsPerTurnEl = 200;
#ifdef AZ_LINEAR
float wormGearRatioAz = 1.0 / 0.8;                  //for M5 thread, 0.8mm per turn
int stepsPerTurnAz = 200;                           //steppers in full step mode to increase torque --> 200 steps per turn
#define AZ_R 65
#define OH 264
#else
float wormGearRatioAz = 44.0;                       //number of teeths for the azimut gear
int stepsPerTurnAz = 200;                           //steppers in full step mode to increase torque --> 200 steps per turn
#endif

long stepsToMoveAz = 0;
long stepsToMoveEl = 0;
int repeatInterval = 30;

//SUN
//PUT YOUR LATITUDE, LONGITUDE, AND TIME ZONE HERE
double latitude = 43.6;        //Toulouse
double longitude = 1.433333;
long sunRize;

int timeZone = 0; //set to UTC
const int dst = 0;


String station = "";
String ssid = "";
String password = "";
boolean hasWifiCredentials = false;
boolean hasNtpTime = false;                 //UTC time not acquired from NTP

//these variable remain in RTC memory even in deep sleep or after software reset (https://github.com/espressif/esp-idf/issues/7718)(https://www.esp32.com/viewtopic.php?t=4931)
RTC_NOINIT_ATTR boolean hasRtcTime = false;   //UTC time not acquired from smartphone
//RTC_DATA_ATTR boolean hasRtcTime = false;   //will only survice to deepsleep reset... not software reset
RTC_NOINIT_ATTR int hours;
RTC_NOINIT_ATTR int seconds;
RTC_NOINIT_ATTR int tvsec;
RTC_NOINIT_ATTR int minutes;
RTC_NOINIT_ATTR int days;
RTC_NOINIT_ATTR int months;
RTC_NOINIT_ATTR int years;


//If you live in the northern hemisphere, it would probably be easier
//for you if you make north as the direction where the azimuth equals
//0 degrees. To do so, switch the 0 below with 180.
float northOrSouth = 180;

float pi = 3.14159265;
float elevation;
float azimuth;
float delta;
float h;

String device = "SolarTracker";
String theMAC = "";
long LastBLEnotification;
long BLEconnectionTimeout = 0;

#include <TimeLib.h>

//Preferences
#include <Preferences.h>
Preferences preferences;


#define PIN_LED 22

unsigned long timeout = 0;


boolean touchWake = false;
boolean resetWake = false;

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
int timeToSleep = 60  ;      /* Time ESP32 will go to sleep (in seconds) */

String message = "";

//UDP --------------
unsigned int localPort = 5000;      // local port to listen on
char packetBuffer[64]; //buffer to hold incoming packet
char AndroidConnected = 0;
WiFiUDP Udp;
//end UDP-----------

//watchdog
//#include <esp_task_wdt.h> //for watchdog
//#define WDT_TIMEOUT 3   //3 seconds WDT

#include "rom/rtc.h"
void print_reset_reason(int reason) //Print last reset reason of ESP32
{
  switch ( reason)
  {
    case 1 :                                                    //Vbat power on reset
      Serial.println ("POWERON_RESET");
      resetWake = true;
      hasRtcTime = false;                                       //this is the only reset case where RTC memory persistant variables are wiped
      heliostatStatus = initializing;
      break;
    case 3 : Serial.println ("SW_RESET"); break;                //Software reset digital core
    case 4 : Serial.println ("OWDT_RESET"); break;              //Legacy watch dog reset digital core
    case 5 : Serial.println ("DEEPSLEEP_RESET"); break;         //Deep Sleep reset digital core
    case 6 : Serial.println ("SDIO_RESET"); break;              //Reset by SLC module, reset digital core
    case 7 : Serial.println ("TG0WDT_SYS_RESET"); break;        //Timer Group0 Watch dog reset digital core
    case 8 : Serial.println ("TG1WDT_SYS_RESET"); break;        //Timer Group1 Watch dog reset digital core
    case 9 : Serial.println ("RTCWDT_SYS_RESET"); break;        //RTC Watch dog Reset digital core
    case 10 : Serial.println ("INTRUSION_RESET"); break;        //Instrusion tested to reset CPU
    case 11 : Serial.println ("TGWDT_CPU_RESET"); break;        //Time Group reset CPU
    case 12 : Serial.println ("SW_CPU_RESET"); break;           //Software reset CPU
    case 13 : Serial.println ("RTCWDT_CPU_RESET"); break;       //RTC Watch dog Reset CPU
    case 14 : Serial.println ("EXT_CPU_RESET"); break;          //for APP CPU, reseted by PRO CPU
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET"); break; //Reset when the vdd voltage is not stable
    case 16 : Serial.println ("RTCWDT_RTC_RESET"); break;       //RTC Watch dog reset digital core and rtc module
    default : Serial.println ("NO_MEAN");
  }
}

void print_wakeup_reason()  //deepSleep wake up reason
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); touchWake = true; break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}


/* ===CODE_STARTS_HERE========================================== */


#define DEBUG_WIFI   //debug Wifi 
#define DEBUG_SLEEP
#define DEBUG_UDP    //broadcast info over UDP
#define DEBUG_PREFS  //debug preferences
#define DEBUG_AZ     //debug computeAzSteps
//#define DEBUG_EL    //debug computeElSteps
//#define DEBUG_VCC
//#define DEBUG


//asyncUDP
#if defined DEBUG_UDP
#include <AsyncUDP.h>
AsyncUDP broadcastUDP;
#endif

//BLE declarations
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914e"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26b1"

void BLEnotify(String theString )
{
  if (deviceConnected == true)
  {
    char message[21];
    String small = "";          //BLE notification MTU is limited to 20 bytes
    while (theString.length() > 0)
    {
      small = theString.substring(0, 19); //cut into 20 chars slices
      theString = theString.substring(19);
      small.toCharArray(message, 20);
      pCharacteristic->setValue(message);
      pCharacteristic->notify();
      delay(3);             // bluetooth stack will go into congestion, if too many packets are sent
      LastBLEnotification = millis(); //will prevent to send new notification before this one is not totally sent
    }
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("client connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("client disconnected");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string rxValue = pCharacteristic->getValue();
      String test = "";
      if (rxValue.length() > 0)
      {
        Serial.print("Received : ");
        for (int i = 0; i < rxValue.length(); i++)
        {
          Serial.print(rxValue[i]);
          test = test + rxValue[i];
        }
        Serial.println();
      }
      String Res;
      int i;
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, test);
      // Test if parsing succeeds.
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        Serial.println("deserializeJson() failed");                             //answer with error : {"answer" : "error","detail":"decoding failed"}
      }
      else
      {
        // Fetch values --> {"Cmd":"Wifi"}
        String Cmd = doc["cmd"];
        if (Cmd == "Wifi")
        {
          const char* cpassword =  doc["Password"] ;
          const char* cssid = doc["SSID"];
          String strpassword(cpassword);
          String strssid(cssid);
          preferences.putString("password", strpassword);
          preferences.putString("ssid", strssid);
          BLEnotify("{\"status\" : \"Wifi set\"}");
#ifdef DEBUG
          Serial.println("set wifi");
#endif
          delay(1000);
          ESP.restart();
          delay(1000);
        }
        else if (Cmd == "Beat") BLEconnectionTimeout = millis(); //heartbeat for the bluetooth connection

        else if (Cmd == "Sun")           //we received position of panel
        {
          currentAz = doc["Az"] ;        //consider that Az is calibrated to its curent position
          currentEl = doc["El"];         //consider that El is calibrated to its curent position
          BLEnotify("{\"status\" : \"sun angles set\"}");
          Serial.println("set Az, El pointing to smartphone");
        }

        else if (Cmd == "Time")
        {
          hours =  doc["HH"] ;
          minutes = doc["MM"];
          seconds = doc["SS"];
          days =  doc["DD"] ;
          months = doc["mm"];
          years = doc["YY"];
          timeZone = doc["TZ"];
          preferences.putInt("timeZone", timeZone);
          latitude = doc["LA"];
          longitude = doc["LO"];
          preferences.putDouble("latitude", latitude);
          preferences.putDouble("longitude", longitude);
          //setTime(now.hour(), now.minute(), now.second(), now.day(), now.month(),  now.year());
          hasRtcTime = true;
          hours = hours - timeZone;
          setTime(hours, minutes, seconds, days, months, years);
          struct timeval current_time;
          gettimeofday(&current_time, NULL);
          tvsec  = current_time.tv_sec ;  //seconds since reboot (stored into RTC memory)
          BLEnotify("{\"status\" :\"time and location set\"}");
          Serial.println("set time from smartphone:");
          display_time();
        }
      }
    }
};

void display_time(void)
{
  Serial.print(year());
  Serial.print("-");
  Serial.print(month());
  Serial.print("-");
  Serial.print(day());
  Serial.print(" at ");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.println(second());
}




void setup()
{
  Serial.begin(115200);

  Serial.println(" ");
  Serial.println("*****************************************");
  Serial.print("CPU0 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(0));
  Serial.println("*****************************************");

  //Preferences
  preferences.begin("eSolarTracker", false);
  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only
  preferences.putInt("timeToSleep", 30);                 // reset time to sleep to 10 minutes (usefull after night)
  timeToSleep = preferences.getInt("timeToSleep", 30);
  currentAz = preferences.getFloat("currentAz", AZ_HOME);
  currentEl = preferences.getFloat("currentEl", EL_HOME);
  heliostatStatus = preferences.getInt("heliostatStatus", 1);
  ssid = preferences.getString("ssid", "");         // Get the ssid  value, if the key does not exist, return a default value of ""
  password = preferences.getString("password", "");
  timeZone = preferences.getInt("timeZone", 1);
  latitude = preferences.getDouble ("latitude", 43.6);        //Toulouse
  longitude = preferences.getDouble("longitude", 1.433333);
  //preferences.end();  // Close the Preferences

#if defined DEBUG_PREFS
  Serial.println("_______prefs after boot_______");
  Serial.print("latitude : ");
  Serial.println(latitude);
  Serial.print("longitude : ");
  Serial.println(longitude);
  Serial.print("timeZone : ");
  Serial.println(timeZone);
  Serial.print("timeToSleep : ");
  Serial.println(timeToSleep);
  Serial.print("heliostatStatus : ");
  Serial.println(heliostatStatus);
  Serial.print("currentEl : ");
  Serial.println(currentEl);
  Serial.print("currentAz : ");
  Serial.println(currentAz);
  Serial.println("______________________________");
#endif

  //enable deepsleep for ESP32
  //  esp_sleep_enable_ext1_wakeup(PIR_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); //this will be the code to enter deep sleep and wakeup with pin GPIO2 high
  esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);                 //allow timer deepsleep

  pinMode(PIN_LED, OUTPUT);   // initialize digital pin 22 as an output.(LED and power on for sensors via P mosfet)
  digitalWrite(PIN_LED, LOW);

  pinMode(AZ_HOME_PIN, INPUT_PULLUP);
  pinMode(EL_HOME_PIN, INPUT_PULLUP);


  //VCC voltage sensor
  //analogSetClockDiv(255);
  //analogReadResolution(12);           // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetAttenuation(ADC_11db);        // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
  //analogSetCycles(8);                   // Set number of cycles per sample, default is 8 and provides an optimal result, range is 1 - 255
  float VCC = 0.;
  for (int i = 0; i < 100; i++)
  {
    VCC += analogRead(VCC_PIN);
  }
  VCC = VCC * 13.15 / 2505. / 100;


#ifdef DEBUG_VCC
  Serial.print("VCC : ");
  Serial.println(VCC);
#endif
  if ((VCC < 11) && (resetWake == false))
  {
    Serial.print("VCC : ");
    Serial.print(VCC);
    Serial.println("V --> XXX too low, go to sleep without motion");
    //    delay(200);
    //    GotoSleep();
  }


  //connect to WiFi
  WiFi.begin(ssid.c_str(), password.c_str());
  long start = millis();
  hasWifiCredentials = false;
  hasNtpTime = false;
  while ((WiFi.status() != WL_CONNECTED) && (millis() - start < 10000))
  {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) hasWifiCredentials = true;

  //if you get here you may be connected to the WiFi
  Serial.print("connected to Wifi: ");
  Serial.println(hasWifiCredentials);
  theMAC = WiFi.macAddress();
  theMAC.replace(":", "");

  // init interfaces
  SetIF();

  if (hasWifiCredentials)
  {
    //init and get the time
    Serial.println("trying to get time 1");
    configTime(timeZone * 3600, dst * 0, "pool.ntp.org");
    printLocalTime();

    //init and get the time
    Serial.println("trying to get time 2");   //call it twice to have a well synchronized time on soft reset... Why ? bex=caus eit works...
    delay(2000);
    configTime(timeZone * 3600, dst * 0, "pool.ntp.org");
    printLocalTime();

    //disconnect WiFi as it's no longer needed
    //  WiFi.disconnect(true);
    //  WiFi.mode(WIFI_OFF);

    if (hasNtpTime)   //set the time with NTP info
    {
      time_t now;
      struct tm * timeinfo;
      time(&now);
      timeinfo = localtime(&now);

      years = timeinfo->tm_year + 1900;   //https://mikaelpatel.github.io/Arduino-RTC/d8/d5a/structtm.html
      months = timeinfo->tm_mon + 1;
      days = timeinfo->tm_mday;
      hours = timeinfo->tm_hour - timeZone;
      minutes = timeinfo->tm_min;
      seconds = timeinfo->tm_sec;

      //set time manually (hr, min, sec, day, mo, yr)
      setTime(hours, minutes, seconds, days, months, years);
      Serial.println("time after ntp:");
      display_time();
      struct timeval current_time;        //get RTC time and save it
      gettimeofday(&current_time, NULL);
      tvsec  = current_time.tv_sec ;      //seconds since reboot
      hasRtcTime = true;                  //now RTC time is also initialized
    }
  }


  //BLE
  // Create the BLE Device
  BLEDevice::init("JP Solar Tracker");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  pCharacteristic->addDescriptor(new BLE2902());      // Create a BLE Descriptor
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();                                  // Start the BLE service
  pServer->getAdvertising()->start();                 // Start advertising
  Serial.println("Waiting a BLE client connection to notify...");
  delay(100);
  //vTaskDelay(10);
  //end BLE

  if ((!hasNtpTime) && (!hasRtcTime))
  {
    Serial.println("no time acquired to compute sun position...");
  }
  else  //ready to compute sun angles
  {
    if (!hasNtpTime)
    {
      Serial.println("use time from RTC :");

      struct timeval current_time;
      gettimeofday(&current_time, NULL);
      // Serial.printf("seconds : %ld\nmicro seconds : %ld", current_time.tv_sec, current_time.tv_usec);
      Serial.printf("seconds stored : %ld\nnow seconds : %ld\n", tvsec, current_time.tv_sec);

      int sec  = seconds - tvsec + current_time.tv_sec ;
      sec = hours * 3600 + minutes * 60 + sec;
      int ss = sec % 60;
      sec = sec / 60;
      int mm = sec % 60;
      sec = sec / 60;
      int hh = sec % 24;
      int dd = days + sec / 24;
      //set time manually (hr, min, sec, day, mo, yr)
      setTime(hh, mm, ss, dd, months, years);
      display_time();
    }

    // Calculate the Sun's azimuth and elevation (corrected for atmospheric refraction), in degrees
    calcHorizontalCoordinates(year(), month(), day(), hour(), minute(), second(), latitude, longitude, sunAzimuth, sunElevation);

    Serial.println("Sun angles : ");
    Serial.print("    Elevation : ");
    Serial.println(sunElevation);
    Serial.print("    panel Elevation : ");
    Serial.println(90 - sunElevation);
    Serial.print("    Azimuth   : ");
    Serial.println(sunAzimuth);

    //AccelStepper
    stepperEl.setMaxSpeed(80.0);
    stepperEl.setAcceleration(80.0);

#ifdef AZ_LINEAR
    stepperAz.setMaxSpeed(80.0);
    stepperAz.setAcceleration(80.0);
#else
    stepperAz.setMaxSpeed(25.0);
    stepperAz.setAcceleration(25.0);
#endif

    ComputeMotion();
    //CheckHoming(); //disabled as no home switch is needed
    Serial.println("moving now");
    pinMode(STEPPER_SLEEP_PIN, OUTPUT);   // initialize digital pin 22 as an output.(LED and power on for sensors via P mosfet)
    digitalWrite(STEPPER_SLEEP_PIN, HIGH);//init the steppers in active mode
    timeout = millis();     //arm software watchdog
  }
  BLEconnectionTimeout = millis();                                //will allow smartphone to connect over bluetooth

  //watchdog
  // Serial.println("Configuring WDT...");
  //  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  //  esp_task_wdt_add(NULL); //add current thread to WDT watch
  //
  //  Serial.println("Resetting WDT...");
  //      esp_task_wdt_reset();             //call it periodically into the main loop

}

void printDigits(int digits)
{
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

time_t toUtc(time_t local)
{
  return local - timeZone * 3600;
}

time_t toLocal(time_t utc)
{
  return utc + timeZone * 3600;
}


void SetIF(void)
{
  //Start UDP
  //Udp.begin(localPort);
}

void printLocalTime() //check if ntp time is acquired nd print it
{
  struct tm timeinfo;
  hasNtpTime = true;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    hasNtpTime = false;
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S"); //https://www.ibm.com/docs/en/workload-automation/9.5.0?topic=troubleshooting-date-time-format-reference-strftime
}

double Az2alpha(double Az)  //convert Azimut angle to alpha angle
{
  double alpha;
  //y = -0,0009x2 - 0,1613x + 178,66 (see excel spreadsheet)
  alpha = -.0009 * Az * Az - 0.1613 * Az + 0178.66;
  return alpha;
}
double alpha2HA(double alpha)
{
  alpha = alpha * PI / 180;
  double HA;
  HA = sqrt( pow(2 * AZ_R * cos(alpha) + AZ_R + OH, 2) + pow(2 * AZ_R * sin(alpha), 2));
  return HA;
}

void loop()
{
  //AccelStepper
  if ((stepperEl.distanceToGo() == 0) && (stepperAz.distanceToGo() == 0))
  {
    digitalWrite(STEPPER_SLEEP_PIN, LOW); //sleep the steppers
    pinMode(STEPPER_SLEEP_PIN, INPUT);    //to force pull down resistor

    if (heliostatStatus ==  morning)   heliostatStatus = sleeping ; // motion finished, now go to sleep until tomorrow morning 
       
    if (resetWake == false)                               //when booting from deepsleep, do not wait for BLE connection
    {
      writePrefs();
      GotoSleep();
    }
    else if ((millis() - BLEconnectionTimeout) > 10000)   //else wait 10s, then sleep
    {
      writePrefs();
      esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR); // restart 10s later to check everything ok (do not reset again !)
      GotoSleep();
    }
  }

  stepperEl.run();
  stepperAz.run();

  if ((millis() - timeout) > 5000)  //log every 5s
  {
    timeout = millis();
    Serial.print("    El nbSteps :");
    Serial.println(stepperEl.distanceToGo());
    Serial.print("    Az nbSteps :");
    Serial.println(stepperAz.distanceToGo());
  }
}

void ComputeMotion()
{
  Serial.println("compute motion");
  Serial.print("    current El : ");
  Serial.println(currentEl);
  Serial.print("    current Az : ");
  Serial.println(currentAz);
  previousAz = currentAz;
  previousEl = currentEl;
#if defined DEBUG_UDP
  fbroadcastUDP("sun El " + String(sunElevation) + ", Current El " + String(currentEl)  + ", panel El " + String(90 - sunElevation) + ", Az " + String(sunAzimuth)  + ", Current Az " + String(currentAz));
#endif
  if (sunElevation >= EL_LIMIT) //catch the sun down to EL_LIMIT
  {
    currentAz = sunAzimuth;
    currentEl = max(sunElevation, double(EL_HOME)); //but move EL down to EL_HOME
    if (mode == "h")  //heliostat mode --> then angle in between sun and target
    {
      currentAz = (currentAz + TargetAzimuth) / 2;
      currentEl = (currentEl + TargetElevation) / 2;
    }
    computeAzSteps();
    computeElSteps();

    switch (heliostatStatus)
    {
      case initializing:    // we have pushed the reset button and time wasn't ok
        Serial.println("daytime calibration of panel done, no motion and goto to sleep");
        stepsToMoveEl = 0;
        stepsToMoveAz = 0;

        if ((!hasNtpTime) && (!hasRtcTime))
        {
          heliostatStatus = initializing; //don't progress if still no time
        }
        else
        {
          heliostatStatus = tracking;     //next time will be tracking or sleeping starting from current position
          Serial.println("next time will be tracking");
          currentAz = previousAz;
          currentEl = previousEl;
          esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR); // restart 1s later
        }
        break;
      default:
        // statements
        Serial.println("daytime motion and goto to sleep");
        heliostatStatus = tracking;     //next time will be tracking
        break;
    }
  }
  else
  {
    switch (heliostatStatus) {
      case initializing:    // we have pushed the reset button and time wasn't ok
        Serial.println("night time calibration of panel done, no motion and goto to sleep");
        stepsToMoveEl = 0;
        stepsToMoveAz = 0;
        if (!hasNtpTime && !hasRtcTime) heliostatStatus = initializing; //don't progress if still no time
        else                            heliostatStatus = sleeping;     //next time will be tracking or sleeping
        break;
      case tracking:    // now go to next morning sunrize azimuth
        heliostatStatus = morning ;
        currentAz = 180 - (previousAz - 180.0);
        computeAzSteps();
        Serial.println("going to next morning Az");
        //sunRize = 2 * ((hours - 12) * 3600 + minutes * 60) ;        // sunRize duration symetrical around 12h and "now"
        sunRize = 2 * ((hour() - 12) * 3600 + minute() * 60) ;        // sunRize duration symetrical around 12h and "now"
        timeToSleep = 86400 - sunRize;
        esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
        Serial.print("timeToSleep : ");
        Serial.println(timeToSleep);
        break;
      case morning:    // now go to sleep until tomorrow morning (handle directly into the loop)
        heliostatStatus = sleeping ;
    
        stepsToMoveAz = 0;
        break;
      case sleeping:    // don't move
        stepsToMoveAz = 0;
        break;
    }
    stepsToMoveEl = 0;
  }

  if (stepsToMoveEl != 0) stepperEl.move(stepsToMoveEl);
  if (stepsToMoveAz != 0) stepperAz.move(stepsToMoveAz);
  Serial.print("    El nbSteps : ");
  Serial.println(stepperEl.distanceToGo());
  Serial.print("    Az nbSteps : ");
  Serial.println(stepperAz.distanceToGo());

}

void computeAzSteps()
{
#ifdef AZ_LINEAR

  previousAzLength = alpha2HA(Az2alpha(previousAz));
  currentAzLength = alpha2HA(Az2alpha(currentAz));
  stepsToMoveAz = ((previousAzLength - currentAzLength ) * wormGearRatioAz * stepsPerTurnAz) ; //it's an integer (would need to convert it back...)
#else
  stepsToMoveAz = ((currentAz - previousAz) * wormGearRatioAz * stepsPerTurnAz) / 360;        //it's an integer
  currentAz = previousAz + (float)stepsToMoveAz * 360. / (wormGearRatioAz * stepsPerTurnAz);  //so needs to recompute real Az value
#endif
#ifdef DEBUG_AZ
  Serial.print("previousAz, currentAz : ");
  Serial.print(previousAz);
  Serial.print(",   ");
  Serial.println(currentAz);
  Serial.print("previousAzLength, currentAzLength, stepsToMoveAz : ");
  Serial.print(previousAzLength);
  Serial.print(",   ");
  Serial.print(currentAzLength);
  Serial.print(",   ");
  Serial.println(stepsToMoveAz);
#endif
}

void computeElSteps()
{
  //length = R* sqrt(2 -2sin(téta))
  previousElLength = SOLAR_PANEL_R * sqrt(2 - 2 * sin(previousEl * pi / 180));
  currentElLength = SOLAR_PANEL_R * sqrt(2 - 2 * sin(currentEl * pi / 180));
  stepsToMoveEl = ((currentElLength - previousElLength) * wormGearRatioEl * stepsPerTurnEl) ;     //it's an integer
  currentElLength = previousElLength + (float)stepsToMoveEl / (wormGearRatioEl * stepsPerTurnEl); //so needs to recompute real currentElLength value
  currentEl = 180. / pi * asin(1. - pow(currentElLength / SOLAR_PANEL_R, 2) / 2.);                //and real El value
#ifdef DEBUG_EL
  Serial.print("previousEl, currentEl : ");
  Serial.print(previousEl);
  Serial.print(",   ");
  Serial.println(currentEl);
  Serial.print("previousElLength, currentElLength, stepsToMoveEl : ");
  Serial.print(previousElLength);
  Serial.print(",   ");
  Serial.print(currentElLength);
  Serial.print(",   ");
  Serial.println(stepsToMoveEl);
#endif
}
void CheckHoming()
{
  Serial.println("check homing");
  if (heliostatStatus == homing)
  {
    Serial.println("homing now");
    Serial.println("move El to home switch");
    stepperEl.move(-10000000);
    do
    {
      stepperEl.run();
      delay(0);
    } while (digitalRead(EL_HOME_PIN) == HIGH);
    stepperEl.setCurrentPosition(0);
    delay(1000);

    Serial.println("move El after home switch");
    stepperEl.move(10000);
    do
    {
      stepperEl.run();
      delay(0);
    } while (digitalRead(EL_HOME_PIN) == LOW);
    currentEl = EL_HOME;
    stepperEl.setCurrentPosition(0);
    delay(1000);

    Serial.println("move Az  to home switch");
    stepperAz.move(-1000000);
    do
    {
      stepperAz.run();
      delay(0);
    } while (digitalRead(AZ_HOME_PIN) == HIGH);
    stepperAz.setCurrentPosition(0);
    delay(1000);

    Serial.println("move Az after home switch");
    stepperAz.move(10000);
    do
    {
      stepperAz.run();
      delay(0);
    } while (digitalRead(AZ_HOME_PIN) == LOW);
    currentAz = AZ_HOME;
    stepperAz.setCurrentPosition(0);

    heliostatStatus = sleeping;
    writePrefs();
    GotoSleep();
  }
}

void writePrefs(void)
{
  preferences.putFloat("currentAz", currentAz);
  preferences.putFloat("currentEl", currentEl);
  preferences.putInt("heliostatStatus", heliostatStatus);
}

void GotoSleep()
{
#if defined DEBUG_SLEEP
  Serial.println("Entering DeepSleep");
  delay(100);
#endif
  esp_deep_sleep_start();       //enter deep sleep mode
  delay(1000);
  abort();
}


#if defined DEBUG_UDP
void fbroadcastUDP(String Res)
{
  // Send UDP Broadcast to 255.255.255.255 (default broadcast addr), Port 5000
  broadcastUDP.broadcastTo(Res.c_str(), 5000);
}
#endif
