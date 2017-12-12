//
// Simple JT65/JT9/WSPR/FSQ beacon for Arduino, with the Etherkit
// Si5351A Breakout Board, by Jason Milldrum NT7S.
//
// Transmit an abritrary message of up to 13 valid characters
// (a Type 6 message) in JT65 and JT9, or a standard Type 1
// message in WSPR.
//
// Connect a momentary push button to pin 12 to use as the
// transmit trigger. Get fancy by adding your own code to trigger
// off of the time from a GPS or your PC via virtual serial.
//
// Original code based on Feld Hell beacon for Arduino by Mark
// Vandewettering K6HX, adapted for the Si5351A by Robert
// Liesenfeld AK6L <ak6l@ak6l.org>.
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

#include <si5351.h>
#include <JTEncode.h>
#include <rs_common.h>
#include <int.h>
#include <string.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Wire.h"


// Variables for Maidenhead locator calculations
char* FirstCharString[]={"A","B","C","D","E","F","G","H","I","J","K","L","M","N","O","P","Q","R"};
char* MidCharString[]={"0","1","2","3","4","5","6","7","8","9"};
char* LastCharString[]={"a","b","c","d","e","f","g","h","i","j","k","l","m","n","o","p","q","r","s","t","u","v","w","x"};
float scrap;
float loclong;
float loclat;


// Mode defines
#define WSPR_TONE_SPACING       146           // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR

#define WSPR_DEFAULT_FREQ       14097045UL

#define DEFAULT_MODE            MODE_WSPR

// Hardware defines
#define LED_PIN                 13

// Enumerations
enum mode {MODE_WSPR};

// Class instantiation
Si5351 si5351;
JTEncode jtencode;

// Global variables
unsigned long freq;
char message[] = "W9PDS EN61";
char call[] = "W9PDS";
char loc[] = "EN61";
uint8_t dbm = 10;
uint8_t tx_buffer[255];
enum mode cur_mode = DEFAULT_MODE;
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;

/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 2, TXPin = 2;
static const uint32_t GPSBaud = 9600;
// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


//******************************************

// Loop through the string, transmitting one character at a time.
void encode()
{
  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK0, 1);
  digitalWrite(LED_PIN, HIGH);
  Serial.println(F("Transmitting"));

  // Now transmit the channel symbols

  for(i = 0; i < symbol_count; i++)
  {
      si5351.set_freq((freq * 100) + (tx_buffer[i] * tone_spacing), SI5351_CLK0);
      delay(tone_delay);
  }

  // Turn off the output
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(LED_PIN, LOW);
  Serial.println(F("Transmit End"));
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  switch(cur_mode)
  {
  case MODE_WSPR:
    jtencode.wspr_encode(call, loc, dbm, tx_buffer);
    break;
  }
}

void setup()
{

  Serial.begin(115200);
  ss.begin(GPSBaud);

  Serial.println(F("Starting GPS WSPR System"));
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  // Use the Arduino's on-board LED as a keying indicator.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);


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
  set_tx_buffer();
}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}





void printLoc(void)
{
loclong=(gps.location.lng()+180)*1000000;
loclat=(gps.location.lat()+90)*1000000;
// First Character - longitude based (every 20° = 1 gridsq)
Serial.print(FirstCharString[int(loclong/20000000)]);
// Second Character - latitude based (every 10° = 1 gridsq)
Serial.print(FirstCharString[int(loclat/10000000)]);
// Third Character - longitude based (every 2Â° = 1 gridsq)
scrap = loclong - (20000000 * int (loclong/20000000));
Serial.print(MidCharString[int(scrap*10/20/1000000)]);
// Fourth Character - latitude based (every 1Â° = 1 gridsq)
scrap = loclat - (10000000 * int (loclat/10000000));
Serial.print(MidCharString[int(scrap/1000000)]);
// Fifth Character - longitude based (every 5' = 1 gridsq)
//scrap = (loclong / 2000000) - (int (loclong/2000000));
//Serial.print(LastCharString[int(scrap * 24)]);
// Sixth Character - longitude based (every 2.5' = 1 gridsq)
//scrap = (loclat / 1000000) - (int (loclat/1000000));
//Serial.print(LastCharString[int(scrap * 24)]);

}






void loop()
{

  //Check for GPS lock
  if (gps.satellites.value() > 2 && gps.hdop.value() > 100)
    {
    Serial.println(F("Valid GPS Lock"));
    if ((gps.time.second() == 0 ) && ( (gps.time.minute() == 0 ) || (gps.time.minute() == 10) || (gps.time.minute() == 20) || (gps.time.minute() == 30) || (gps.time.minute() == 40) || (gps.time.minute() == 50)))
        {
           Serial.print("Ten MInute STart      ");
           Serial.print(gps.time.hour());
           Serial.print(F(":"));
           Serial.print(gps.time.minute());
           Serial.print(":");
           Serial.print(gps.time.second());
           //printLoc();
           encode();
        }
    }

  else
    {
    Serial.println(F("GPS Lock Not acuqired:"));
    Serial.print(gps.satellites.value());
    Serial.print(F(" "));
    Serial.println(gps.hdop.value());
    }




  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  //encode();
  //delay(129400);
   smartDelay(1000);
}
