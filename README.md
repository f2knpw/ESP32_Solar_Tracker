# ESP32_Solar_Tracker
low cost solar panel solution (MPPT + sun tracker). Here is the code for the Solar Tracker 

The ESP32 is programmed under Arduino IDE. The code is quite simple but provides the following functionalities :<br />
 Lite version:https://github.com/f2knpw/ESP32_Solar_Tracker/blob/master/JP_ESP32_SolarTracker_lite_Arduino.ino
try to connect to internet<br />
get the UTC time over NTP server<br />
in case of failure open a bluetooth connection with an android App -->to be done<br />
get the time over Bluetooth Low Energy                             -->to be done<br />
in case of failure : use the RTC time                              -->to be done<br />
compute the sun position using the excellent solar calulator library (licensed under the MIT License)<br />
compute the stepper motors motion<br />
read the power voltage<br />
perform the motion if the power level is enough to drive the motors<br />
apply a sleeping strategy (sleep 10 minutes during day or sleep the whole night)<br />
reset to the next morning position if needed<br />
sleep and wake up next time<br />
<br />
refer to my Hackaday's project page for full description : https://hackaday.io/project/185105-low-cost-solar-panel-solution-mppt-sun-tracker<br />

Full version V2: https://github.com/f2knpw/ESP32_Solar_Tracker/blob/master/JP_ESP32_SolarTracker_OTA_Arduino.ino <br />
Same functionalities as lite version + solar Tracker initialization and calibration via Android companion APP "panelOrientation V2".<br />
For details refer to this log: https://hackaday.io/project/185105/log/210034-automating-the-solar-tracker-initialization <br />
Android panelOrientation V2 App is here : https://github.com/f2knpw/solar_Panel_Orientation <br />

Full version V3: https://github.com/f2knpw/ESP32_Solar_Tracker/blob/master/JP_ESP32_SolarTracker_V3_Arduino.ino <br />
Same functionalities as lite version + solar Tracker initialization and calibration via Android companion APP "panelOrientation V3" Implements the new azimut axis mechanics.<br />
For details refer to this log: https://hackaday.io/project/185105/log/211762-a-new-calibration-procedure<br />
Android panelOrientation V3 App is here : https://github.com/f2knpw/solar_Panel_Orientation <br />

