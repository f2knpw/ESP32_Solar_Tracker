# ESP32_Solar_Tracker
low cost solar panel solution (MPPT + sun tracker). Here is the code for the Solar Tracker 

The ESP32 is programmed under Arduino IDE. The code is quite simple but provides the following functionalities :

try to connect to internet
get the UTC time over NTP server
in case of failure open a bluetooth connection with an android App -->to be done
get the time over Bluetooth Low Energy                             -->to be done
in case of failure : use the RTC time                              -->to be done
compute the sun position using the excellent solar calulator library (licensed under the MIT License)
compute the stepper motors motion
read the power voltage
perform the motion if the power level is enough to drive the motors
apply a sleeping strategy (sleep 10 minutes during day or sleep the whole night)
reset to the next morning position if needed
sleep and wake up next time

refer to my Hackaday's project page for full description : https://hackaday.io/project/185105-low-cost-solar-panel-solution-mppt-sun-tracker

