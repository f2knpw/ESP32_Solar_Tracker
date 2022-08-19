
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>

#include <TimeLib.h>
#include <SolarCalculator.h>


//UDP
#include <ESPmDNS.h>
#include <WiFiUdp.h>

///stepper motors
#include <AccelStepper.h>

// The Az Stepper pins. http://www.schmalzhaus.com/EasyDriver/index.html
#define STEPPER_AZ_DIR_PIN 32
#define STEPPER_AZ_STEP_PIN 33
// The El stepper pins
#define STEPPER_EL_DIR_PIN 23
#define STEPPER_EL_STEP_PIN 19

#define STEPPER_SLEEP_PIN 25

#define EL_HOME_PIN 5
#define AZ_HOME_PIN 18
#define GUI_PIN 12
#define VCC_PIN 36

#define SOLAR_PANEL_R 540   //expressed in mm
#define EL_HOME 20          //expressed in °
#define EL_LIMIT 15         //expressed in °
#define AZ_HOME 180         //expressed in °

// Define steppers and the pins they will use
AccelStepper stepperEl(AccelStepper::DRIVER, STEPPER_EL_STEP_PIN, STEPPER_EL_DIR_PIN);
AccelStepper stepperAz(AccelStepper::DRIVER, STEPPER_AZ_STEP_PIN, STEPPER_AZ_DIR_PIN);

double sunElevation;
double sunAzimuth;
double TargetElevation;
double TargetAzimuth;
double previousAz, currentAz, previousEl, previousElLength, currentEl, currentElLength;
String mode = "s";                                  //"h" for heliostat, "s" for sunTracker
enum {tracking, morning, sleeping, homing };                  // 0 : tracking, 1 : sleeping, 2 : homing
int heliostatStatus = sleeping;
float wormGearRatioAz = 44.0;
float wormGearRatioEl = 1.0 / 0.8;                    //for M5 thread, 0.8mm per turn
int stepsPerTurnAz = 200;
int stepsPerTurnEl = 200;
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
boolean noTime = false;

int hours;
int seconds;
int minutes;
int days;
int months;
int years;


//If you live in the northern hemisphere, it would probably be easier
//for you if you make north as the direction where the azimuth equals
//0 degrees. To do so, switch the 0 below with 180.
float northOrSouth = 180;

float pi = 3.14159265;
float elevation;
float azimuth;
float delta;
float h;

extern "C" int rom_phy_get_vdd33(); //may work if bluetooth or wifi is on
int internalBatReading;


String device = "SolarTracker";
String theMAC = "";


#include <TimeLib.h>


//needed for WifiManager library
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

//Preferences
#include <Preferences.h>
Preferences preferences;



#define PIN_LED 22


unsigned long timeout = 0;


//touchpad
#define Threshold 50 /* Greater the value, more the sensitivity */
RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;
void callback() {
  //placeholder callback function
}
boolean TouchWake = false;
boolean SendMAC = false;

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
int timeToSleep = 60  ;      /* Time ESP32 will go to sleep (in seconds) */

String st;

//UDP --------------
unsigned int localPort = 5000;      // local port to listen on
WiFiUDP Udp;




/* ===CODE_STARTS_HERE========================================== */

#define xDEBUG_OUT
#define W_DEBUG     //debug Wifi 
#define DEBUG_SLEEP
#define DEBUG_UDP   //broadcast info over UDP
#define DEBUG_OUT
#define xDEBUG
#define VCC_DEBUG
#define PREF_DEBUG
//#define DEBUG
//#define TEST

//asyncUDP
#if defined DEBUG_UDP //will broadcast debug messages over UDP
#include <AsyncUDP.h>
AsyncUDP broadcastUDP;
#endif

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD :
      Serial.println("Wakeup caused by touchpad");
      TouchWake = true;
      break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}



void setup()
{
  Serial.begin(115200);

  //Preferences
  preferences.begin("eSolarTracker", false);
  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only
  preferences.putInt("timeToSleep", 600);                 // reset time to sleep to 10 minutes (usefull after night)
  timeToSleep = preferences.getInt("timeToSleep", 600);
  currentAz = preferences.getFloat("currentAz", AZ_HOME);
  currentEl = preferences.getFloat("currentEl", EL_HOME);
  heliostatStatus = preferences.getInt("heliostatStatus", 1);
  //preferences.end();  // Close the Preferences
#if defined PREF_DEBUG
  Serial.println("_________________");
  Serial.print("timeToSleep : ");
  Serial.println(timeToSleep);
  Serial.print("heliostatStatus : ");
  Serial.println(heliostatStatus);
  Serial.print("currentEl : ");
  Serial.println(currentEl);
  Serial.print("currentAz : ");
  Serial.println(currentAz);
  Serial.println("_________________");
#endif

  //Print the wakeup reason for ESP32
  //  esp_sleep_enable_ext1_wakeup(PIR_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); //this will be the code to enter deep sleep and wakeup with pin GPIO2 high
  esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);                 //this is to configure deepsleep duration
  print_wakeup_reason();

  pinMode(PIN_LED, OUTPUT);              // initialize digital pin 22 as an output.(LED and power on for sensors via P mosfet)
  digitalWrite(PIN_LED, LOW);

  pinMode(STEPPER_SLEEP_PIN, OUTPUT);   // initialize digital pin 22 as an output.(LED and power on for sensors via P mosfet)
  digitalWrite(STEPPER_SLEEP_PIN, HIGH);//init the steppers in active mode

  pinMode(AZ_HOME_PIN, INPUT_PULLUP);
  pinMode(EL_HOME_PIN, INPUT_PULLUP);
  pinMode(GUI_PIN, INPUT_PULLUP);

  //VCC voltage sensor
  //analogSetClockDiv(255);
  //analogReadResolution(12);           // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetAttenuation(ADC_11db);       // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
  analogSetCycles(8);                   // Set number of cycles per sample, default is 8 and provides an optimal result, range is 1 - 255
  float VCC = 0.;
  for (int i = 0; i < 100; i++)
  {
    VCC += analogRead(VCC_PIN);         //read the ADC on VCC PIN
  }
  VCC = VCC * 13.15 / 2505. / 100;      //simple calibration and average over 100 values
#ifdef VCC_DEBUG
  Serial.print("VCC : ");
  Serial.println(VCC);
#endif
  if (VCC < 11)                       //don't try to move if VCC is too low
  {
    Serial.print("VCC : ");
    Serial.println(VCC);
    Serial.println(" too low, go to sleep without motion");
    delay(200);
    GotoSleep();
  }

  //WiFiManager --> now try to connect to WIFI --> see doc of tis lib here : https://www.arduino.cc/reference/en/libraries/wifimanager/
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("SolarTrackerConfig")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  theMAC = WiFi.macAddress();
  theMAC.replace(":", "");

  // init interfaces
  SetIF();

 

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

  time_t now;
  struct tm * timeinfo;
  time(&now);
  timeinfo = localtime(&now);

  years = timeinfo->tm_year + 1900;   //https://mikaelpatel.github.io/Arduino-RTC/d8/d5a/structtm.html
  months = timeinfo->tm_mon + 1;
  days = timeinfo->tm_mday;
  hours = timeinfo->tm_hour;
  minutes = timeinfo->tm_min;
  seconds = timeinfo->tm_sec;

  //set time manually (hr, min, sec, day, mo, yr)
  setTime(hours, minutes, seconds, days, months, years);

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
  stepperEl.setMaxSpeed(100.0);
  stepperEl.setAcceleration(100.0);

  stepperAz.setMaxSpeed(25.0);
  stepperAz.setAcceleration(25.0);

  ComputeMotion();
  CheckHoming();
  //  if ((stepsToMoveAz != 0) || (stepsToMoveEl != 0)) WriteFilePref();  //if sunposition has changed then save it
  Serial.println("moving now");

  timeout = millis();     //arm software watchdog
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
  Udp.begin(localPort);
}

void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    noTime = true;
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S"); //https://www.ibm.com/docs/en/workload-automation/9.5.0?topic=troubleshooting-date-time-format-reference-strftime
}

void loop()
{
  //AccelStepper
  // Change direction at the limits
  if ((stepperEl.distanceToGo() == 0) && (stepperAz.distanceToGo() == 0))
  {
    //save values and sleep
    //heliostatStatus = sleeping;
    digitalWrite(STEPPER_SLEEP_PIN, LOW); //sleep the steppers
    pinMode(STEPPER_SLEEP_PIN, INPUT);    //to force pull down resistor
    writePrefs();
    GotoSleep();
  }
  stepperEl.run();                        //perform motion
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
  if (sunElevation >= EL_LIMIT)
  {
    heliostatStatus = tracking;
    currentAz = sunAzimuth;
    currentEl = max(sunElevation, double(EL_HOME));
    if (mode == "h")  //heliostat mode --> then angle in between sun and target
    {
      currentAz = (currentAz + TargetAzimuth) / 2;
      currentEl = (currentEl + TargetElevation) / 2;
    }
    stepsToMoveAz = ((currentAz - previousAz) * wormGearRatioAz * stepsPerTurnAz) / 360;        //it's an integer
    currentAz = previousAz + (float)stepsToMoveAz * 360. / (wormGearRatioAz * stepsPerTurnAz);  //so needs to recompute real Az value

    //length = R* sqrt(2 -2sin(téta))
    previousElLength = SOLAR_PANEL_R * sqrt(2 - 2 * sin(previousEl * pi / 180));
    currentElLength = SOLAR_PANEL_R * sqrt(2 - 2 * sin(currentEl * pi / 180));
    stepsToMoveEl = ((currentElLength - previousElLength) * wormGearRatioEl * stepsPerTurnEl) ;     //it's an integer
    currentElLength = previousElLength + (float)stepsToMoveEl / (wormGearRatioEl * stepsPerTurnEl); //so needs to recompute real currentElLength value
    currentEl = 180. / pi * asin(1. - pow(currentElLength / SOLAR_PANEL_R, 2) / 2.);                      //and real El value
  }
  else
  {
    switch (heliostatStatus) {
      case tracking:    // now go to next morning sunrize azimuth
        heliostatStatus = morning ;
        currentAz = 180 - (previousAz - 180.0);
        stepsToMoveAz = ((currentAz - previousAz) * wormGearRatioAz * stepsPerTurnAz) / 360;        //it's an integer
        currentAz = previousAz + (float)stepsToMoveAz * 360. / (wormGearRatioAz * stepsPerTurnAz);  //so needs to recompute real Az value
        Serial.println("going to next morning Az");
        break;
      case morning:    // now go to sleep until tomorrow morning
        heliostatStatus = sleeping ;
        sunRize = 2 * ((hours - 12) * 3600 + minutes * 60) ;        // sunRize duration symetrical around 12h and "now"
        timeToSleep = 86400 - sunRize;
        esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
        Serial.print("timeToSleep : ");
        Serial.println(timeToSleep);
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
    currentAz = 0;
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

void SendUDP(String Res)
{
  char  ReplyBuffer[Res.length() + 1];     // a string to send back
  Res.toCharArray(ReplyBuffer, Res.length() + 1);
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.print(ReplyBuffer); //was write...
  Udp.endPacket();
#if defined DEBUG_OUT
  Serial.println(Res);
#endif
}

#if defined DEBUG_UDP
void fbroadcastUDP(String Res)
{
  // Send UDP Broadcast to 255.255.255.255 (default broadcast addr), Port 5000
  broadcastUDP.broadcastTo(Res.c_str(), localPort);
}
#endif
