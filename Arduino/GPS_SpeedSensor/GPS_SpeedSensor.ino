/*
  -----------------------------------------------------------
            MHSFA Speed GPS Sensor v 1.1
  -----------------------------------------------------------

   Based on the "Jeti GPS Sensor v 1.4" - Tero Salminen RC-Thoughts.com (c) 2017 www.rc-thoughts.com

   Uses Arduino Pro Mini + Ublox NEO-M8N GPS-module

   Libraries needed
   - NeoGPS by SlashDevin
   - AltSoftSerial by Paul Stoffregen
   - Jeti Sensor EX Telemetry C++ Library by Bernd Wokoeck
   - TinyGPS++ (C) 2008-2013 Mikal Hart - using the distancebetween in my code below

  -----------------------------------------------------------
      Shared under MIT-license by Alastair Cormack (c) 2017
  -----------------------------------------------------------
  Ver 1.0 - Initial Release
  Ver 1.1 - Changed Altitude to be relative to start position and added a 3 second wait after 1st fix
*/

#include <JetiExSerial.h>
#include <JetiExProtocol.h>

#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>

static NMEAGPS  nmgps;
static gps_fix  fix_data;
unsigned long distToHome = 0;
float home_lat;
float home_lon;
boolean homeSet = false;
boolean timerStarted = false;
unsigned long timeOfFix;


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


enum
{
  ID_GPSLAT       = 1,
  ID_GPSLON       = 2,
  ID_GPSSPEEDKM   = 3,
  ID_GPSSPEEDMI   = 3,
  ID_ALTM         = 4,
  ID_DIST         = 5,
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

JETISENSOR_CONST sensorsSPEED[] PROGMEM =
{
  // id             name          unit          data type           precision
  { ID_GPSLAT,      "Latitude",   " ",          JetiSensor::TYPE_GPS, 0 },
  { ID_GPSLON,      "Longitude",  " ",          JetiSensor::TYPE_GPS, 0 },
  { ID_GPSSPEEDKM,  "Speed",      "km/h",       JetiSensor::TYPE_14b, 0 },
  { ID_ALTM,        "Altitude",   "m",          JetiSensor::TYPE_14b, 0 },
  { ID_DIST,        "Distance",   "m",          JetiSensor::TYPE_14b, 0 },
  { 0 }
};


// Restart by user
void(* resetFunc) (void) = 0;

void setup()
{
  gpsPort.begin( 9600 );

  
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
  sendUBX( ubxRate10Hz, sizeof(ubxRate10Hz) );
  delay( COMMAND_DELAY ); 
  LastSentenceInInterval = NMEAGPS::NMEA_GGA;
  
  jetiEx.SetDeviceId( 0x76, 0x32 );

  jetiEx.Start( "GPS", sensorsSPEED, JetiExProtocol::SERIAL2 );
}


void loop()
{
  
  if (nmgps.available( gpsPort )){
      fix_data = nmgps.read();
      if (!fix) {
        if (fix_data.valid.location){
          if (timerStarted == false) {
            timerStarted = true;
            timeOfFix = millis();
          }
          else {
            if ((millis() - timeOfFix) > 3000) { //allow for a few seconds for fixes to settle
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
        distToHome = distanceBetween(
            glat,
            glng,
            home_lat,
            home_lon);

        
        jetiEx.SetSensorValueGPS( ID_GPSLAT, false, glat );
        jetiEx.SetSensorValueGPS( ID_GPSLON, true, glng );
        jetiEx.SetSensorValue( ID_ALTM, (fix_data.altitude() - altirel));
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
      }
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

  // DN
  if ( c == 0xb0 )
  {
    if ( _nMenu == 1 ) {
      _nMenu = 0;
      _bSetDisplay = true;
      resetFunc();
    }
  }

  if ( !_bSetDisplay )
    return;

  switch ( _nMenu )
  {
    case 0:
      jetiEx.SetJetiboxText( JetiExProtocol::LINE1, "   MHSFA Speed" );
      jetiEx.SetJetiboxText( JetiExProtocol::LINE2, "   GPS Sensor" );
      _bSetDisplay = false;
      break;
  }
}