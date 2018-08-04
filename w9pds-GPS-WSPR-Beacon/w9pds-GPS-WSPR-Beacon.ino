//
// Simple WSPR beacon for Arduino, with the Etherkit
// Si5351A Breakout Board, by Jason Milldrum NT7S.
// This code works great an an Arduino Nano,
//
// Uses a Tinygps++ compatible device for scheduling the transmissions &
// configuring your 4 digit maidenhead locator.
//
// Code adopted from various sources, and cobbled together by
// W9PDS - patrick@skerrett.net
//
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

//#define DEBUG     // Comment next 2 lines to reduce memory
#define LED_DEBUG
#include <si5351.h>
#include <JTEncode.h>
#include <rs_common.h>
#include <int.h>
#include <string.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Wire.h"


// Mode defines
#define WSPR_TONE_SPACING       146           // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR

//#define WSPR_DEFAULT_FREQ       14097045UL
#define WSPR_DEFAULT_FREQ       14097000UL  //You may want to adjust after calibrating your Si5351

#define DEFAULT_MODE            MODE_WSPR

// Hardware defines
#define LED_PIN          13
#define RED_LED          12
#define GREEN_LED        11

// Enumerations
enum mode {MODE_WSPR};

// Class instantiation
Si5351 si5351;
JTEncode jtencode;


// Global variables
unsigned long freq;
char call[] = "W9PDS";     //CHANGE AAAA TO YOUR CALLSIGN!!
uint8_t dbm = 10;         //CHANGE for your power setting. 10dbm = 0.01W
uint8_t tx_buffer[255];
enum mode cur_mode = DEFAULT_MODE;
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;
// Variables for Maidenhead locator calculations
float scrap;
float lon;
float lat;

/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


//******************************************
//******************************************
//******************************************




// Loop through the string, transmitting one character at a time.
void encode()
{
  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK0, 1);

  #ifdef DEBUG
  Serial.println(F("Transmitting"));
  #endif
  #ifdef LED_DEBUG
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  #endif

  // Now transmit the channel symbols
  uint8_t i;
  for(i = 0; i < symbol_count; i++)
  {
      si5351.set_freq((freq * 100) + (tx_buffer[i] * tone_spacing), SI5351_CLK0);
      delay(tone_delay);
  }

  // Turn off the output
  si5351.output_enable(SI5351_CLK0, 0);
  #ifdef DEBUG
  Serial.println(F("Transmit End"));
  #endif
  #ifdef LED_DEBUG
    digitalWrite(RED_LED, LOW);
  #endif

  smartdelay(5000);
}




void set_tx_buffer()
{


  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);
    // Find current Maidenhead from GPS
  long loclon=(gps.location.lng()+180)*1000000;
  long loclat=(gps.location.lat()+90)*1000000;

  char MH[5] = {'A', 'A', '0', '0'};     // Initialise our print string
  MH[0] += int(loclon/20000000);        //First Char
  MH[1] += int(loclat/10000000);        //Second Char
  scrap = loclon - (20000000 * int (loclon/20000000));
  MH[2] += int(scrap*10/20/1000000);    //Third Char
  scrap = loclat - (10000000 * int (loclat/10000000));
  MH[3] += int(scrap/1000000);          //Fourth Char

    #ifdef DEBUG
    String MH_txt = "";                              // Build up Maidenhead
    int x = 0;                                       // into a string that's easy to print
    while (x < 4){
      MH_txt += MH[x];
      x++; }
     Serial.print("Found Maidenhead: ");              //Only for debug
     Serial.println(MH_txt);
     #endif

  // Set the proper frequency and timer CTC depending on mode
  switch(cur_mode)
  {
  case MODE_WSPR:
    jtencode.wspr_encode(call, MH, dbm, tx_buffer);
    break;
  }
}




void waitForTime()
{
    //Check for GPS lock
  if ( gps.satellites.value() > 2 )
    {
    #ifdef DEBUG
    Serial.print(F("Valid GPS Lock. Sats:"));
    Serial.println(gps.satellites.value());
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    Serial.println(gps.time.second());
    #endif
    #ifdef LED_DEBUG
    digitalWrite(GREEN_LED, HIGH);
    #endif


    //Configure your transmit schedule here. Transmit should be done on even minute at 0 seconds.
    //Default is every 10 minutes.
    if ((gps.time.second() == 0 ) &&
    ( (gps.time.minute() == 0 ) || (gps.time.second() == 0 ) &&
    (gps.time.minute() == 10) || (gps.time.second() == 0 ) &&
    (gps.time.minute() == 20) || (gps.time.second() == 0 ) &&
    (gps.time.minute() == 30) || (gps.time.second() == 0 ) &&
    (gps.time.minute() == 40) ||   (gps.time.second() == 0 ) &&
    (gps.time.minute() == 50)))

        {
           #ifdef DEBUG
           Serial.print("Scheduled Transmit Start:      ");   //print for debug
           Serial.print(gps.time.hour());
           Serial.print(F(":"));
           Serial.print(gps.time.minute());
           Serial.print(F(":"));
           Serial.print(gps.time.second());
           #endif
           set_tx_buffer();
           encode();
        }
    }

  else
    {
    #ifdef DEBUG
    Serial.println(F("GPS Lock Not aquired: "));
    Serial.println(gps.satellites.value());
    #endif
    #ifdef LED_DEBUG
    digitalWrite(GREEN_LED, LOW);
    #endif


    }

}



//Smartdelay is used to sleep while keeping the GPS going.
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}



//***************************************************************************


void setup()
{
  ss.begin(GPSBaud);

  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println(F("Starting GPS WSPR System"));
  #endif
  #ifdef LED_DEBUG
  //init leds
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(RED_LED, HIGH);
  delay(500);
  digitalWrite(GREEN_LED, HIGH);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  #endif

  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);




  // Set the mode to use
  cur_mode = MODE_WSPR;

  // Set the proper frequency, tone spacing, symbol count, and
  // tone delay depending on mode
  switch(cur_mode)
  {

  case MODE_WSPR:
    freq = WSPR_DEFAULT_FREQ;
    symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
    tone_spacing = WSPR_TONE_SPACING;
    tone_delay = WSPR_DELAY;
    break;

  }

  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially

  // Encode the message in the transmit buffer
  // This is RAM intensive and should be done separately from other subroutines
  //set_tx_buffer();
}





void loop()
{

   while (ss.available() > 0)
    if (gps.encode(ss.read()))
      waitForTime();


  #ifdef DEBUG
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
  #endif

}
