# gps-wspr-mept

MEPT beacon for Transmission of WSPR signals using an Si5351 and a GPS timer.

TO use, be sure to change the following values:

char call[] = "AAAA";     //CHANGE AAAA TO YOUR CALLSIGN!!
uint8_t dbm = 10;         //CHANGE for your power setting. 10dbm = 0.01W


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
