# SpeedGPS
GPS Module for Jeti that has a high navigation rate of up to 10hz (based on work by Tero @ rc-thoughts and some of my original RFID jeti work). I have Powerbox GPS's on my speed helis but the GPS module it uses only gives 1-2 samples per second. This means that there are not many samples in a speed run. The latest generation of GPS modules sample at much higher data rates and have the same doppler shift speed configuration for very accurate speed readings. This code configures a UBlox NEO-M8N module to put out a limited set of NMEA messages to get the high data rate over the very low bandwidth serial connection with an Arduino Pro mini. 

* Ver 1.0: Initial version
* Ver 1.1: Changed Altitude to be relative to start-position and added a 3 second wait after 1st fix to allow GPS to settle. Added a pre-comiled .hex file.
* Ver 1.2: Changed to 5hz as ports were overloaded at 10hz

The GPS Module I got here:
https://www.ebay.co.uk/itm/3-5V-UBlox-NEO-M8N-GPS-Module-For-Pixhawk-Flight-Controller-APM-2-5-W-Antenna/20208032531

The Arduino from here:
http://www.ebay.co.uk/itm/Arduino-3-3v-Pro-mini-compatible-boards-Multipacks-NEW-TESTED-UK-STOCK/162062691852
 
and the best place to find how to put them together and the place I based this work off is:
https://www.rc-thoughts.com/jeti-gps-sensor/

![alt text](https://github.com/AlCormack/SpeedGPS/blob/master/images/GPSModule.jpg "SpeedGPS on Gaui X3L")

Big thanks to Tero at rc-thoughts. I strongly encourage everyone to look at his site. 
