# gps-wspr-mept

MEPT beacon for Transmission of WSPR signals using an Si5351 and a GPS timer.

TO use, be sure to change the following values:

char call[] = "AAAA";     //CHANGE AAAA TO YOUR CALLSIGN!!

uint8_t dbm = 10;         //CHANGE for your power setting.
10dbm means 0.01W - This is typical for an Si5351 without an amplifier

BE SURE TO USE A PROPER LOW PASS FILTER!!!
Your signal output will not be clean and you will broadcast spurious emissions
on un-intended bands without a filter.
Performance will also greatly improve if you us a proper filter in-line.


I've had great success using the QRL-Labs kits:
https://www.qrp-labs.com/lpfkit.html



###########################
Wiring -
###########################


On an Arduino Nano, the GPS is wired as follows:
RXPin = 4, TXPin = 3;

The Si5351 is wired per the Arduino Wire library -
(SDA), (SCL)
https://www.arduino.cc/en/Reference/Wire

I am also using LEDs for debugging purposes, but these can be omitted.
If you use them, terminate with a resistor to ground

#define RED_LED          12
#define GREEN_LED        11


73 and happy whispering!

W9PDS

patrick@skerrett.net
