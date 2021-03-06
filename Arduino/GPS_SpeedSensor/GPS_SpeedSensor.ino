/*
  -----------------------------------------------------------
            MHSFA Speed GPS Sensor v 1.7
  -----------------------------------------------------------

   Based on the "Jeti GPS Sensor v 1.4" - Tero Salminen RC-Thoughts.com (c) 2017 www.rc-thoughts.com

   Uses Arduino Pro Mini + Ublox NEO-M8N GPS-module

   Libraries needed
   - NeoGPS by SlashDevin
   - AltSoftSerial by Paul Stoffregen
   - Jeti Sensor EX Telemetry C++ Library by Bernd Wokoeck
   - TinyGPS++ (C) 2008-2013 Mikal Hart - using the distancebetween in code below

  -----------------------------------------------------------
      Shared under MIT-license by Alastair Cormack (c) 2017
  -----------------------------------------------------------
  Ver 1.0 - Initial Release
  Ver 1.1 - Changed Altitude to be relative to start position and added a 3 second wait after 1st fix
  Ver 1.2 - Changed to 5hz as ports were overloaded at 10hz
  Ver 1.3 - Added in "°" (degree) as unit for lat and long
  Ver 1.4 - Added a faster way to calc distance between 2 points as we are only doing this over a few 
            hunded metres. This will allow 9-10hz of gps of Jeti telemitry if no others sensors using the bus.
            For the maths folks it is now using Equirectangular approximation and not the haversine formula. 
            Code for this faster method is mine :).
  Ver 1.5 - Changed DoJetiSend in JetiExProtocol (changed filename to include2) from 150ms to 80ms wait
  Ver 1.6 - Added in support for ATMega32U chip (ie Pro Micro board)
  Ver 1.7 - Added ability to change between 5hz and 10hz. It also changed the GNSS setting for 10hz as only GPS sats are supported at this rate.
  Ver 1.8 - Added pdop as error value to be reported back to tx. Note this needs an updated GPSfix_cfg.h to enable DOP fields. The value sent back is * x1000 eg 1209 is 1.209
  Ver 1.9 - Added version display while we are waiting for GPS to settle after getting a fix (5 seconds) - 18 is 1.8. Added in a reset in the jeti box to reset dist and alt to 0.
  Ver 1.10 - Corrected DOP display
*/
#define GPS_SPEED_VER 110


#include "JetiExSerial.h"
#include "JetiExProtocol2.h" // The has an 80ms wait for sending rather than the stock 150. This lets this do atleast 10hz now.
#include <EEPROM.h>

#if defined(__AVR_ATmega32U4__)
  #define GPSport_h
  #include <AltSoftSerial.h>
  #define SS_TYPE AltSoftSerial
  #define RX_PIN -1  // doesn't matter because it only works...
  #define TX_PIN -1  //    ...on two specific pins
  SS_TYPE gpsPort( RX_PIN, TX_PIN );
  #define GPS_PORT_NAME "AltSoftSerial"
  #define DEBUG_PORT Serial
#else
  #include <GPSport.h>
#endif

#include <NMEAGPS.h>
#include <Streamers.h>

static NMEAGPS  nmgps;
static gps_fix  fix_data;
unsigned long distToHome = 0;
float home_lat;
float home_lon;
int last_good_alt = 0;
boolean homeSet = false;
boolean timerStarted = false;
unsigned long timeOfFix;

int hz = 5;

uint8_t LastSentenceInInterval = 0xFF; // storage for the run-time selection

//-------------------------------------------
// U-blox UBX binary commands

const unsigned char ubxRate1Hz[] PROGMEM = 
  { 0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00 };
const unsigned char ubxRate5Hz[] PROGMEM =
  { 0x06,0x08,0x06,0x00,200,0x00,0x01,0x00,0x01,0x00 };
const unsigned char ubxRate10Hz[] PROGMEM =
  { 0x06,0x08,0x06,0x00,100,0x00,0x01,0x00,0x01,0x00 };
const unsigned char ubxRate16Hz[] PROGMEM =
  { 0x06,0x08,0x06,0x00,50,0x00,0x01,0x00,0x01,0x00 };

// Disable specific NMEA sentences
const unsigned char ubxDisableGGA[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableGLL[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableGSA[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableGSV[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableRMC[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableVTG[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableZDA[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x08,0x00,0x00,0x00,0x00,0x00,0x01 };

const unsigned char ubxCfgGNSSFast[] PROGMEM =
  { 0x06,0x3E,0x2C,0x00,0x00,0x20,0x20,0x05,0x00,0x08,0x10,
    0x00,0x01,0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x00,0x00,
    0x01,0x01,0x03,0x08,0x10,0x00,0x00,0x00,0x01,0x01,0x05,
    0x00,0x03,0x00,0x00,0x00,0x01,0x01,0x06,0x08,0x0E,0x00,
    0x00,0x00,0x01,0x01};

const unsigned char ubxCfgGNSSBetter[] PROGMEM =
  { 0x06,0x3E,0x2C,0x00,0x00,0x20,0x20,0x05,0x00,0x08,0x10,
    0x00,0x01,0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x01,0x00,
    0x01,0x01,0x03,0x08,0x10,0x00,0x00,0x00,0x01,0x01,0x05,
    0x00,0x03,0x00,0x01,0x00,0x01,0x01,0x06,0x08,0x0E,0x00,
    0x01,0x00,0x01,0x01};

const unsigned char ubxSetdm7[] PROGMEM = 
  { 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//--------------------------

void sendUBX( const unsigned char *progmemBytes, size_t len )
{
  gpsPort.write( 0xB5 ); // SYNC1
  gpsPort.write( 0x62 ); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte( progmemBytes++ );
    a += c;
    b += a;
    gpsPort.write( c );
  }

  gpsPort.write( a ); // CHECKSUM A
  gpsPort.write( b ); // CHECKSUM B

} // sendUBX

const uint32_t COMMAND_DELAY = 250;

JetiExProtocol jetiEx;

boolean fix = false;
float glat;
float glng;
int altirel;
int altiabs;
long gerror = 9999;

enum
{
  ID_GPSLAT       = 1,
  ID_GPSLON       = 2,
  ID_GPSSPEEDKM   = 3,
  ID_GPSSPEEDMI   = 3,
  ID_ALTM         = 4,
  ID_DIST         = 5,
  ID_ERROR        = 6,
};

double distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double distanceBetweenFast(double lat1, double long1, double lat2, double long2)
{
  double p1 = radians(long2 - long1) * cos ( 0.5*(lat2+lat1) ); //convert lat/lon to radians
  double p2 = radians(lat2 - lat1);
  return 6372795 * sqrt( p1*p1 + p2*p2);
}

JETISENSOR_CONST sensorsSPEED[] PROGMEM =
{
  // id             name          unit          data type           precision
  { ID_GPSLAT,      "Latitude",   "\xB0",       JetiSensor::TYPE_GPS, 0 },
  { ID_GPSLON,      "Longitude",  "\xB0",       JetiSensor::TYPE_GPS, 0 },
  { ID_GPSSPEEDKM,  "Speed",      "km/h",       JetiSensor::TYPE_14b, 0 },
  { ID_ALTM,        "Altitude",   "m",          JetiSensor::TYPE_14b, 0 },
  { ID_DIST,        "Distance",   "m",          JetiSensor::TYPE_14b, 0 },
  { ID_ERROR,       "Error",      "",           JetiSensor::TYPE_30b, 0 },
  { 0 }
};


// Restart by user
void(* resetFunc) (void) = 0;

void setup()
{
  gpsPort.begin( 9600 );

  hz = EEPROM.read(0);
  if (hz == 255) {
    hz = 5;
  }
  
  sendUBX( ubxDisableGLL, sizeof(ubxDisableGLL) );
  delay( COMMAND_DELAY );
  sendUBX( ubxDisableGSV, sizeof(ubxDisableGSV) );
  delay( COMMAND_DELAY );
  sendUBX( ubxDisableGSA, sizeof(ubxDisableGSA) );
  delay( COMMAND_DELAY );
  sendUBX( ubxDisableVTG, sizeof(ubxDisableVTG) );
  delay( COMMAND_DELAY );
  sendUBX( ubxDisableZDA, sizeof(ubxDisableZDA) );
  delay( COMMAND_DELAY );
  sendUBX( ubxSetdm7, sizeof(ubxSetdm7) );
  delay( COMMAND_DELAY ); 
  
  if (hz == 5) {
      sendUBX( ubxCfgGNSSBetter, sizeof(ubxCfgGNSSBetter) );
      delay( COMMAND_DELAY *10 ); 
      sendUBX( ubxRate5Hz, sizeof(ubxRate5Hz) );
      delay( COMMAND_DELAY );  
  }
  else {
      sendUBX( ubxCfgGNSSFast, sizeof(ubxCfgGNSSFast) );
      delay( COMMAND_DELAY * 10 ); 
      sendUBX( ubxRate10Hz, sizeof(ubxRate10Hz) );  
      delay( COMMAND_DELAY );  
  }
  
  LastSentenceInInterval = NMEAGPS::NMEA_GGA;
  
  jetiEx.SetDeviceId( 0x76, 0x32 );

  jetiEx.Start( "GPS", sensorsSPEED, JetiExProtocol::SERIAL2 );
}


void loop()
{
  int temp_alt = 0;
  
  if (nmgps.available( gpsPort )){
      fix_data = nmgps.read();
      if (!fix) {
        if (fix_data.valid.location){
          if (timerStarted == false) {
            timerStarted = true;
            timeOfFix = millis();
          }
          else {
            if ((millis() - timeOfFix) > 5000) { //allow for a few seconds for fixes to settle
              fix = true;
              altirel = fix_data.altitude();
            }
          }
        } else {
          fix = false;
        }
      }
    
      if (fix) {
      
        glat = (float)fix_data.latitude();
        glng = (float)fix_data.longitude();

        if (!homeSet) {
            homeSet = true;
            home_lat = glat;
            home_lon = glng;
        }
        distToHome = distanceBetweenFast(
            glat,
            glng,
            home_lat,
            home_lon);


        //gerror = (fix_data.lon_err() + fix_data.lat_err()) / 2.0;
        if (fix_data.hdop > 0) {
          gerror = fix_data.hdop;
        }
        jetiEx.SetSensorValue( ID_ERROR, gerror );
        jetiEx.SetSensorValueGPS( ID_GPSLON, true, glng );
        jetiEx.SetSensorValueGPS( ID_GPSLAT, false, glat );
        jetiEx.SetSensorValueGPS( ID_GPSLON, true, glng );
        //getting some bad altitude values that give large negative numbers.. lets clear them out and use last known good value.
        temp_alt = fix_data.altitude() - altirel;
        if (temp_alt > -50) {  //if we have good alt we are ok
            last_good_alt = temp_alt; 
        }
        jetiEx.SetSensorValue( ID_ALTM, (last_good_alt));
        jetiEx.SetSensorValue( ID_DIST, distToHome);
        jetiEx.SetSensorValue( ID_GPSSPEEDKM, fix_data.speed_kph() );
    
      } else { // If Fix end
        glat = 0;
        glng = 0;
  
        jetiEx.SetSensorValueGPS( ID_GPSLAT, false, glat );
        jetiEx.SetSensorValueGPS( ID_GPSLON, true, glng );
        jetiEx.SetSensorValue( ID_ALTM, 0 );
        jetiEx.SetSensorValue( ID_DIST, 0);
        jetiEx.SetSensorValue( ID_GPSSPEEDKM, 0 );
        if (timerStarted == false) {
          jetiEx.SetSensorValue( ID_ERROR, hz); //put out hz while we are waiting for fix
        }
        else {
          jetiEx.SetSensorValue( ID_ERROR, GPS_SPEED_VER); //put version while we are waiting for fix to settle
        }
        
      }
  }

  //just to deal with the possibility of not reading the GPS buffer fast enough
  if (nmgps.overrun()) {
    nmgps.overrun( false );
  }
  HandleMenu(); 
  jetiEx.DoJetiSend();
  
}


void HandleMenu()
{
  static int  _nMenu = 0;
  static bool _bSetDisplay = true;
  uint8_t c = jetiEx.GetJetiboxKey();

  // 224 0xe0 : // RIGHT
  // 112 0x70 : // LEFT
  // 208 0xd0 : // UP
  // 176 0xb0 : // DOWN
  // 144 0x90 : // UP+DOWN
  //  96 0x60 : // LEFT+RIGHT

  // Right
  if ( c == 0xe0 && _nMenu < 3 )
  {
    _nMenu++;
    _bSetDisplay = true;
  }

  // Left
  if ( c == 0x70 &&  _nMenu > 0 )
  {
    _nMenu--;
    _bSetDisplay = true;
  }

  // DN
  if ( c == 0xb0 )
  {
    if ( _nMenu == 1 ) {
      hz = 5;
      EEPROM.write(0, hz);
      _nMenu = 4;
      _bSetDisplay = true;
    }
    if ( _nMenu == 2 ) {
      hz = 10;
      EEPROM.write(0, hz);
      _nMenu = 4;
      _bSetDisplay = true;
    }
    if ( _nMenu == 3 ) {
      fix = false;
      homeSet = false;
      timerStarted = false;
      _nMenu = 4;
      _bSetDisplay = true;
    }
  }

  if ( !_bSetDisplay )
    return;

  switch ( _nMenu )
  {
    case 0:
      jetiEx.SetJetiboxText( JetiExProtocol::LINE1, "  MHSFA Speed" );
      jetiEx.SetJetiboxText( JetiExProtocol::LINE2, "   GPS Sensor" );
      _bSetDisplay = false;
      break;
    case 1:
      jetiEx.SetJetiboxText( JetiExProtocol::LINE1, "  5hz" );
      jetiEx.SetJetiboxText( JetiExProtocol::LINE2, "  Store: DOWN" );
      _bSetDisplay = false;
      break;
    case 2:
      jetiEx.SetJetiboxText( JetiExProtocol::LINE1, "  10hz" );
      jetiEx.SetJetiboxText( JetiExProtocol::LINE2, "  Store: DOWN" );
      _bSetDisplay = false;
      break;
    case 3:
      jetiEx.SetJetiboxText( JetiExProtocol::LINE1, "  Reset" );
      jetiEx.SetJetiboxText( JetiExProtocol::LINE2, "  DOWN" );
      _bSetDisplay = false;
      break;
    case 4:
      jetiEx.SetJetiboxText( JetiExProtocol::LINE1, "Settings stored!" );
      jetiEx.SetJetiboxText( JetiExProtocol::LINE2, " " );
      _bSetDisplay = false;
      break;
      if (_nMenu == 4) {
        delay(1500);
        _nMenu = 0;
        _bSetDisplay = true;
      }
  }
}

