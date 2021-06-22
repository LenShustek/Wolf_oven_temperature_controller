// file: oven_mockup.ino
/**************************************************************************************************

   Wolf oven simulator

   This is the software for a small test board that simulates the operation
   of a Wolf oven and controller knob, for testing our temperature controller.
   It has:

   - Arduino Nano processor board
   - SPDT 3-position momentary contact toggle switch for "temp up/down"
   - 16-position binary encoded rotary switch for "mode"
   - a potentiometer for (maybe) changing the oven temperature
   - a serial-over-power driver for output to the knob display
   - one 10-pin connector that simulates the oven
   - one 10-pin connector that simulates the knob
   - one 4-pin connector that simulates connection to the RTD temperature sensor
   - a 4-line 20-character LCD display with an I2C interface

   It optionally has a second Nano that simulates our temperature controller instead
   of connecting the real PC board to the 10-pin connectors. Connect one or the other,
   but not both, and change the REAL_BOARD switch below.

****** change log ******

   10 Dec 2020, L. Shustek, V1.0
   24 Dec 2020, L. Shustek, V1.1 for V1.1 temp controller hardware
   31 Dec 2020, L. Shustek, update for RTD version, with either the real board or a Nano simulator
   13 Feb 2021, L. Shustek, add debugging using software serial port on D12, if REAL_BOARD
    3 Mar 2021, L. Shustek, don't reset digital pots if temp hasn't changed, to avoid glitching controller
   10 Mar 2021, L. Shustek, add oven fast/slow heat modes
   14 Mar 2021, L. Shustek, signal "digital pots changing" on an output pin
   ----------------------------------------------------------------------------------------------------
   Copyright (c) 2020,2021 Len Shustek; released under The MIT License (MIT)

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
   associated documentation files (the "Software"), to deal in the Software without restriction,
   including without limitation the rights to use, copy, modify, merge, publish, distribute,
   sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or
   substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
   DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
   -------------------------------------------------------------------------------------------------*/

#define REAL_BOARD 1              // are we using the real temp controller board, or a Nano simulator?
#define DIGITAL_POTS 1            // if the real board, are we driving the RTD interface with digital pots?
#define TEMP_ADJUST -3            // experimental adjustment to the temperature we sent via the digital pots

#define DEBUG REAL_BOARD          // debugging output
#define DEBUG_PIN 12
#define DEBUG_BAUDRATE 115200

#define MSECS_PER_OVEN_SIM 1000   // msec between simulation steps
#define PURGE_BUFFER_TIME 50      // msec after which we purge fragments in the receive buffer
#define OVEN_DISPLAY_TIME 100     // how often oven outputs knob display

#define START_TEMP 250            // default starting oven temperature
#define DO_IMPULSE false          // should we goose the temperature with an impulse, like from opening the door?
#define IMPULSE_MSEC (500UL*1000) // at what msec?
#define IMPULSE_AMOUNT -100       // how many degrees plus or minus?

#define DEBOUNCE 50               // msec of debounce delay

#define LED 13           // Nano's LED
#define LED_ON HIGH
#define LED_OFF LOW

#define MODE_S8 8       // inputs from knob simulator
#define MODE_S4 7       // (also intputs to temp controller)
#define MODE_S2 6
#define MODE_S1 5
#define KNOB_UP 3
#define KNOB_DN 4

#if REAL_BOARD
   #define DPOT_CONNECT A0  // digital pot connect to RTD connector
   #define DPOT_CS A1       // digital pot chip select
   #define DPOT_SI A2       // digital pot serial data in
   #define DPOT_SCK A3      // digital pot clock
   #define DPOT_CHANGING 2  // "we are changing the digital pot", active low
#else                   // inputs from the Nano temp controller simulator
   #define RTD_CTLR 9       // "the RTD is connected to our controller"
   #define RTD_OVEN 2       // "the RTD is connected to the oven"
   #define OVEN_LOW 11      // oven is being told temp is low, so is heating
   #define OVEN_HIGH 12     // oven is being told temp is high, so is cooling
#endif

#define POT xx            // our potentiometer (unused presently)
#define TEMP_OUT 10       // the simulated real temperature to our temp controller
#define FAKE_TEMP_IN A7   // the fake temperature from our temp controller

#if REAL_BOARD
   #define TEMP_MIN 50     // oven temp to the real board: PWM 0..66% (3.3V/5V) for TEMP_MIN..TEMP_MAX
   #define TEMP_MAX 600
   #define TEMP_PWM_MAX_mV 3153UL // experimentally measured, theoretically is 3300
#else
   #define TEMP_MIN 50     // oven temp to the Nano simulator: PWM 0..100% for TEMP_MIN..TEMP_MAX
   #define TEMP_MAX 600
   #define TEMP_PWM_MAX_mV 5000UL
#endif

#define INTERPOLATE(x,x0,x1,y0,y1) (y0*(x1-x)+y1*(x-x0))/(x1-x0)  // use casts on y0 and y1 to avoid truncation and overflow!
#define INTERPOLATE_BOUNDED(x,x0,x1,y0,y1) x<x0 ? y0 : x>x1 ? y1 : INTERPOLATE(x,x0,x1,y0,y1)  // ensure x0 < x1

#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"
Adafruit_LiquidCrystal lcd(0);  // default I2C address is 0x70

#if DEBUG
   #include "SoftwareSerial.h"
   SoftwareSerial debug_port (/*RX*/ A7, /*TX*/ DEBUG_PIN); // (receive is not used)
   #define MAXLINE 100              // maximum debugging line
#endif

#if DEBUG
#define debug_print(format,...) debug_printer (F(format),__VA_ARGS__)
void debug_printer(const __FlashStringHelper *format, ...) {
   char buf[MAXLINE];
   unsigned long timenow = millis();
   sprintf_P(buf, PSTR("at %3d.%01d secs: "), (unsigned short)(timenow / 1000), (unsigned short) ((timenow % 1000) / 100));
   debug_port.write(buf);
   va_list argptr;
   va_start(argptr, format);
   vsnprintf_P(buf, MAXLINE, (PGM_P)(format), argptr);
   debug_port.write(buf);
   va_end(argptr); }
#else
#define debug_print(...)
#endif

const char *modenames[16] = {
   "OFF ", "BAKE", "MD02", "MD03", "CRST", "PROF", "CONV", "CLN ",
   "CBAK", "BRL ", "MD10", "MD11", "CBRL", "RST ", "MD14", "MD15" };
enum mode_t {M_OFF, M_BAKE, M_02, M_03, M_CONV_ROAST, M_PROOF, M_CONV, M_CLEAN,
             M_CONV_BAKE, M_BROIL, M_10, M_11, M_CONV_BROIL, M_ROAST, M_14, M_15 };
byte getmode(void) {
   byte mode = digitalRead(MODE_S8);
   mode = (mode << 1) | digitalRead(MODE_S4);
   mode = (mode << 1) | digitalRead(MODE_S2);
   mode = (mode << 1) | digitalRead(MODE_S1);
   return 15 - mode; }

#if !REAL_BOARD
const char *RTDswitchnames[4] = { // who the RTD is connected to
   "    ", "oven", "ctlr", "both" };
byte getRTDswitch(void) {
   return (digitalRead(RTD_CTLR) << 1) | digitalRead(RTD_OVEN); }

const char *ovenfaketempnames[4] = { // what temp we're faking to the oven
   "       ", "heating", "cooling", "*error*" };
byte getovenfaketemp(void) {
   return (digitalRead(OVEN_HIGH) << 1) | digitalRead(OVEN_LOW); }
#endif

const char *updnnames[4] = { // status of up/down switch
   "??", "DN", "UP", "  " };
byte getupdn(void) {
   return (digitalRead(KNOB_DN) << 1) | digitalRead(KNOB_UP); }

bool controlling_temp = false;
byte mode = M_14;
byte RTDswitch = 0xff, ovenfaketemp = 0xff, updn = 0xff;
int fake_temp = 0;

// oven simulator

int temp_current = START_TEMP;
float ftemp_current = temp_current;
unsigned long startsim_time = 0;
unsigned long last_ovensim_time = 0;
unsigned long last_oven_display_time = 0;
char oven_display[4] = {0 };

#if REAL_BOARD
int read_fake_temp(void) { // read the "fake" temp from the oven controller
   int rawdata = analogRead(FAKE_TEMP_IN);  // 0 to 1023
   int Vd = ((unsigned long) rawdata * (5 * 10)) >> 10; // tenths of a volt
   // V < 1.2V means the 1.2K resistor is switched in, simulating 125F
   // V > 1.2V means the 2.2K resistor is switched in, simulating 620F
   return Vd < 12 ? 125 : 620; }
#define OVEN_SETPOINT 350  // what the oven thinks the setpoint is
#endif

#if REAL_BOARD && DIGITAL_POTS
void set_potentiometer (unsigned int setting) { // send configuration word to AD5162 digital pot
   digitalWrite(DPOT_CONNECT, LOW); // disconnect from the RTD reader, else can't set pots!
   lcd.setCursor(10 + (setting & 0x100 ? 5 : 0), 3); //FOR TESTING
   lcd.print("   "); // erase old stuff
   lcd.setCursor(10 + (setting & 0x100 ? 5 : 0), 3); //FOR TESTING
   lcd.print(setting & 255, DEC);
   digitalWrite(DPOT_CS, LOW);  // chip select
   for (int i = 0; i < 9; ++i) { // send pot address A0, then D7..D0
      digitalWrite(DPOT_SI, setting & 0x100 ? 1 : 0);
      digitalWrite(DPOT_SCK, HIGH); // positive clock edge loads data
      setting <<= 1;
      digitalWrite(DPOT_SCK, LOW); }  // (clock high min is only 20 ns)
   digitalWrite(DPOT_CS, HIGH);
   digitalWrite(DPOT_CONNECT, HIGH); } // reconnect to RTD reader

void send_real_temp(unsigned int temp) { //tell the real controller what the oven temp is
   // by simulating the resistance of an R1000 RTD at that temperature

   // A table of 869 pairs of bytes representing optimal digital pot settings,
   // indexed by temperatures from 32F to 900F by 1F,
   // calculated from Rpots=2617.0, R1=356.1, R2=9980.0, Rboth=505.4, and Rwiper=76.7,
   // as generated by the C program DigitalPot.
#define F_LOW 32
#define F_HIGH 900
   static const struct {
      byte pot1, pot2; } PROGMEM temp_to_pot_settings[] = {
      /*   32F, 1000.00 ohms */ 8, 255, 9, 0, 9, 7, 9, 102, 9, 217, 9, 255, 10, 0, 10, 73, 10, 178, 10, 255,
      /*   42F, 1021.69 ohms */ 11, 0, 11, 49, 11, 144, 11, 255, 12, 0, 12, 27, 12, 115, 12, 219, 12, 255, 13, 8,
      /*   52F, 1043.35 ohms */ 13, 89, 13, 185, 13, 255, 14, 0, 14, 67, 14, 155, 14, 255, 15, 0, 15, 47, 15, 129,
      /*   62F, 1064.98 ohms */ 15, 224, 16, 0, 16, 30, 16, 106, 16, 194, 16, 255, 17, 14, 17, 85, 17, 167, 17, 255,
      /*   72F, 1086.57 ohms */ 18, 0, 18, 67, 18, 144, 18, 232, 19, 0, 19, 51, 19, 123, 19, 205, 20, 0, 20, 37,
      /*   82F, 1108.12 ohms */ 20, 104, 20, 181, 20, 255, 21, 24, 21, 87, 21, 159, 21, 241, 22, 12, 22, 72, 22, 140,
      /*   92F, 1129.63 ohms */ 22, 216, 23, 2, 23, 59, 23, 123, 23, 195, 24, 0, 24, 47, 24, 107, 24, 175, 24, 251,
      /*  102F, 1151.12 ohms */ 25, 36, 25, 93, 25, 157, 25, 229, 26, 26, 26, 81, 26, 141, 26, 209, 27, 17, 27, 69,
      /*  112F, 1172.56 ohms */ 27, 127, 27, 191, 28, 9, 28, 59, 28, 114, 28, 175, 28, 242, 29, 49, 29, 102, 29, 160,
      /*  122F, 1193.97 ohms */ 29, 224, 30, 41, 30, 91, 30, 146, 30, 207, 31, 33, 31, 81, 31, 134, 31, 192, 32, 26,
      /*  132F, 1215.35 ohms */ 32, 72, 32, 123, 32, 178, 32, 238, 33, 64, 33, 112, 33, 165, 33, 223, 34, 56, 34, 103,
      /*  142F, 1236.68 ohms */ 34, 153, 35, 8, 35, 50, 35, 94, 35, 143, 35, 196, 35, 253, 36, 87, 36, 133, 36, 184,
      /*  152F, 1257.99 ohms */ 36, 239, 37, 79, 37, 124, 37, 173, 37, 225, 38, 73, 38, 116, 38, 163, 38, 213, 39, 67,
      /*  162F, 1279.25 ohms */ 39, 108, 39, 153, 39, 202, 40, 61, 40, 102, 40, 145, 40, 191, 40, 242, 41, 95, 41, 137,
      /*  172F, 1300.49 ohms */ 42, 16, 41, 230, 42, 89, 42, 130, 42, 173, 42, 220, 43, 84, 43, 123, 44, 10, 43, 210,
      /*  182F, 1321.68 ohms */ 44, 79, 44, 117, 44, 157, 45, 40, 44, 247, 45, 111, 46, 5, 45, 192, 45, 237, 46, 106,
      /*  192F, 1342.84 ohms */ 46, 144, 47, 34, 46, 228, 47, 101, 47, 138, 47, 177, 47, 219, 48, 97, 48, 132, 49, 29,
      /*  202F, 1363.97 ohms */ 48, 211, 48, 254, 49, 127, 49, 164, 50, 57, 49, 245, 50, 123, 51, 25, 51, 54, 50, 237,
      /*  212F, 1385.05 ohms */ 51, 118, 51, 153, 51, 190, 51, 229, 52, 114, 52, 148, 52, 184, 52, 222, 53, 111, 54, 20,
      /*  222F, 1406.11 ohms */ 54, 48, 53, 215, 53, 254, 55, 19, 55, 46, 54, 209, 55, 104, 56, 18, 55, 168, 55, 203,
      /*  232F, 1427.13 ohms */ 55, 240, 56, 132, 56, 164, 57, 70, 56, 233, 57, 128, 58, 42, 57, 193, 58, 96, 59, 16,
      /*  242F, 1448.11 ohms */ 59, 41, 58, 188, 59, 94, 60, 16, 60, 40, 59, 184, 60, 92, 59, 251, 60, 149, 61, 64,
      /*  252F, 1469.05 ohms */ 61, 90, 60, 245, 61, 146, 62, 63, 61, 207, 62, 115, 62, 143, 62, 172, 63, 87, 63, 113,
      /*  262F, 1489.96 ohms */ 64, 38, 63, 169, 64, 86, 63, 230, 64, 138, 64, 166, 64, 195, 64, 225, 65, 136, 66, 60,
      /*  272F, 1510.84 ohms */ 65, 191, 67, 16, 66, 134, 66, 160, 66, 188, 66, 217, 66, 248, 67, 158, 68, 82, 69, 17,
      /*  282F, 1531.68 ohms */ 68, 130, 69, 59, 69, 81, 68, 210, 68, 239, 70, 59, 69, 180, 69, 207, 71, 38, 70, 152,
      /*  292F, 1552.48 ohms */ 70, 177, 70, 204, 71, 126, 71, 150, 72, 80, 72, 102, 73, 39, 73, 59, 72, 173, 74, 20,
      /*  302F, 1573.25 ohms */ 73, 124, 73, 147, 73, 171, 75, 21, 74, 123, 74, 146, 76, 4, 76, 22, 75, 122, 74, 246,
      /*  312F, 1593.98 ohms */ 75, 168, 75, 192, 75, 217, 75, 243, 77, 80, 77, 100, 78, 42, 76, 240, 78, 80, 79, 25,
      /*  322F, 1614.68 ohms */ 79, 43, 78, 142, 80, 9, 80, 26, 79, 120, 78, 235, 79, 163, 80, 100, 80, 120, 79, 233,
      /*  332F, 1635.34 ohms */ 81, 81, 81, 100, 80, 207, 81, 140, 81, 161, 82, 100, 81, 205, 83, 64, 83, 82, 84, 31,
      /*  342F, 1655.97 ohms */ 82, 204, 83, 139, 82, 250, 85, 32, 83, 202, 84, 139, 84, 159, 85, 101, 84, 201, 87, 4,
      /*  352F, 1676.56 ohms */ 86, 84, 87, 35, 87, 51, 88, 6, 86, 158, 86, 178, 87, 120, 86, 220, 86, 242, 89, 38,
      /*  362F, 1697.11 ohms */ 87, 198, 87, 219, 90, 24, 89, 103, 88, 197, 91, 11, 90, 87, 90, 104, 91, 56, 92, 13,
      /*  372F, 1717.63 ohms */ 90, 157, 90, 176, 93, 1, 91, 139, 90, 236, 92, 105, 92, 122, 93, 74, 92, 157, 93, 106,
      /*  382F, 1738.11 ohms */ 94, 60, 94, 75, 93, 157, 92, 254, 96, 7, 94, 140, 95, 92, 93, 253, 97, 9, 94, 212,
      /*  392F, 1758.56 ohms */ 97, 36, 97, 50, 98, 11, 96, 141, 97, 94, 96, 175, 99, 13, 99, 26, 96, 230, 99, 53,
      /*  402F, 1778.97 ohms */ 98, 126, 100, 28, 97, 229, 100, 55, 101, 17, 100, 83, 101, 43, 102, 7, 102, 19, 102, 32,
      /*  412F, 1799.35 ohms */ 102, 45, 103, 9, 102, 72, 101, 144, 102, 100, 100, 246, 101, 193, 103, 87, 103, 101, 101, 245,
      /*  422F, 1819.69 ohms */ 104, 75, 102, 210, 102, 227, 103, 177, 103, 193, 105, 90, 107, 6, 106, 65, 106, 78, 105, 147,
      /*  432F, 1839.99 ohms */ 107, 54, 106, 119, 107, 80, 106, 148, 108, 56, 109, 22, 106, 194, 106, 210, 109, 58, 106, 243,
      /*  442F, 1860.26 ohms */ 109, 83, 109, 96, 111, 15, 110, 72, 111, 38, 108, 210, 109, 165, 110, 124, 112, 40, 111, 99,
      /*  452F, 1880.50 ohms */ 110, 166, 113, 31, 112, 88, 110, 211, 114, 22, 114, 33, 112, 140, 111, 211, 113, 115, 112, 182,
      /*  462F, 1900.69 ohms */ 113, 141, 116, 16, 112, 227, 115, 81, 115, 93, 113, 212, 113, 227, 116, 83, 114, 198, 116, 107,
      /*  472F, 1920.85 ohms */ 117, 73, 116, 132, 116, 145, 119, 23, 118, 75, 119, 44, 116, 199, 117, 159, 119, 77, 117, 186,
      /*  482F, 1940.98 ohms */ 119, 100, 117, 214, 121, 38, 117, 243, 121, 59, 122, 30, 123, 3, 121, 92, 120, 150, 119, 215,
      /*  492F, 1961.07 ohms */ 121, 127, 121, 139, 120, 202, 123, 74, 125, 8, 124, 55, 123, 107, 122, 165, 123, 130, 126, 20,
      /*  502F, 1981.13 ohms */ 123, 154, 125, 78, 127, 13, 122, 245, 127, 32, 126, 80, 124, 180, 128, 25, 129, 0, 127, 82,
      /*  512F, 2001.15 ohms */ 125, 181, 127, 103, 126, 158, 126, 170, 129, 56, 125, 246, 129, 76, 126, 220, 128, 138, 129, 107,
      /*  522F, 2021.13 ohms */ 130, 78, 131, 51, 127, 234, 127, 247, 132, 44, 129, 174, 130, 141, 133, 37, 129, 210, 131, 132,
      /*  532F, 2041.08 ohms */ 133, 65, 130, 199, 132, 123, 134, 58, 130, 236, 131, 200, 135, 51, 132, 178, 133, 146, 135, 79,
      /*  542F, 2060.99 ohms */ 133, 168, 134, 137, 137, 38, 138, 15, 133, 214, 133, 226, 137, 74, 135, 160, 138, 58, 138, 67,
      /*  552F, 2080.87 ohms */ 137, 112, 137, 122, 137, 132, 135, 228, 138, 114, 137, 163, 141, 31, 136, 229, 139, 116, 136, 253,
      /*  562F, 2100.71 ohms */ 143, 4, 137, 230, 138, 197, 138, 208, 141, 101, 138, 231, 142, 85, 143, 61, 142, 103, 139, 232,
      /*  572F, 2120.51 ohms */ 144, 55, 143, 96, 142, 141, 140, 233, 141, 201, 142, 171, 141, 223, 141, 234, 147, 30, 146, 68,
      /*  582F, 2140.28 ohms */ 142, 224, 146, 85, 149, 5, 145, 137, 143, 225, 145, 156, 144, 205, 150, 15, 148, 81, 150, 30,
      /*  592F, 2160.02 ohms */ 148, 98, 149, 75, 147, 150, 150, 61, 152, 13, 146, 218, 148, 152, 152, 35, 147, 209, 150, 111,
      /*  602F, 2179.72 ohms */ 152, 58, 149, 163, 153, 45, 151, 113, 152, 90, 154, 40, 150, 174, 152, 115, 154, 63, 150, 203,
      /*  612F, 2199.38 ohms */ 152, 141, 152, 150, 152, 159, 155, 73, 153, 143, 158, 8, 153, 161, 151, 245, 155, 113, 156, 91,
      /*  622F, 2219.01 ohms */ 157, 70, 160, 0, 156, 115, 154, 190, 157, 101, 158, 80, 159, 60, 161, 16, 155, 201, 156, 175,
      /*  632F, 2238.60 ohms */ 162, 12, 155, 230, 157, 168, 156, 212, 162, 39, 157, 195, 162, 53, 159, 146, 157, 223, 159, 163,
      /*  642F, 2258.16 ohms */ 165, 7, 157, 252, 165, 20, 164, 51, 162, 112, 160, 182, 164, 72, 159, 235, 161, 175, 163, 122,
      /*  652F, 2277.68 ohms */ 160, 227, 166, 56, 160, 246, 164, 124, 163, 162, 165, 111, 166, 91, 169, 24, 170, 8, 164, 172,
      /*  662F, 2297.16 ohms */ 163, 213, 164, 189, 170, 33, 165, 174, 171, 23, 169, 77, 167, 138, 171, 42, 166, 184, 173, 10,
      /*  672F, 2316.61 ohms */ 173, 16, 173, 22, 166, 218, 167, 194, 172, 64, 173, 47, 170, 129, 174, 37, 167, 237, 173, 73,
      /*  682F, 2336.02 ohms */ 167, 255, 176, 18, 170, 175, 170, 183, 176, 36, 173, 114, 173, 121, 171, 185, 170, 224, 172, 171,
      /*  692F, 2355.40 ohms */ 171, 209, 170, 250, 179, 20, 173, 173, 176, 100, 172, 219, 177, 89, 178, 72, 172, 244, 179, 62,
      /*  702F, 2374.74 ohms */ 174, 198, 174, 206, 174, 214, 175, 192, 176, 171, 177, 151, 174, 247, 181, 67, 180, 96, 178, 153,
      /*  712F, 2394.05 ohms */ 183, 42, 181, 92, 184, 33, 179, 155, 180, 136, 185, 30, 186, 16, 180, 157, 186, 27, 181, 145,
      /*  722F, 2413.32 ohms */ 181, 152, 181, 159, 181, 166, 179, 230, 179, 238, 182, 161, 186, 73, 186, 79, 186, 85, 183, 163,
      /*  732F, 2432.56 ohms */ 183, 170, 183, 177, 191, 8, 181, 249, 182, 227, 184, 179, 187, 112, 186, 142, 192, 21, 188, 108,
      /*  742F, 2451.76 ohms */ 190, 71, 192, 37, 191, 62, 184, 238, 185, 217, 194, 21, 189, 129, 192, 70, 188, 166, 196, 6,
      /*  752F, 2470.92 ohms */ 196, 11, 195, 34, 192, 99, 192, 105, 192, 111, 192, 117, 191, 146, 196, 47, 194, 92, 197, 39,
      /*  762F, 2490.05 ohms */ 199, 9, 189, 231, 200, 2, 195, 100, 191, 199, 196, 91, 199, 39, 192, 194, 191, 227, 196, 114,
      /*  772F, 2509.14 ohms */ 197, 99, 201, 29, 201, 34, 201, 39, 196, 144, 198, 107, 197, 134, 197, 140, 196, 169, 201, 70,
      /*  782F, 2528.20 ohms */ 204, 22, 195, 213, 197, 171, 203, 54, 201, 97, 206, 13, 198, 173, 195, 255, 205, 44, 198, 192,
      /*  792F, 2547.22 ohms */ 204, 72, 202, 116, 197, 237, 197, 244, 197, 251, 199, 207, 207, 49, 208, 37, 207, 59, 203, 141,
      /*  802F, 2566.20 ohms */ 207, 69, 207, 74, 201, 204, 202, 187, 209, 54, 210, 42, 207, 100, 213, 4, 205, 151, 206, 136,
      /*  812F, 2585.15 ohms */ 212, 33, 209, 89, 204, 197, 212, 47, 214, 20, 209, 110, 205, 199, 215, 18, 214, 38, 212, 76,
      /*  822F, 2604.07 ohms */ 205, 224, 208, 163, 217, 10, 212, 96, 218, 4, 214, 71, 216, 43, 217, 32, 212, 122, 210, 167,
      /*  832F, 2622.95 ohms */ 218, 30, 217, 50, 215, 88, 211, 169, 208, 242, 220, 22, 209, 231, 215, 113, 220, 35, 211, 204,
      /*  842F, 2641.79 ohms */ 211, 210, 214, 153, 215, 139, 222, 27, 218, 95, 223, 21, 222, 40, 215, 166, 212, 236, 217, 138,
      /*  852F, 2660.60 ohms */ 219, 107, 224, 32, 218, 135, 213, 244, 219, 127, 221, 97, 218, 156, 217, 181, 228, 5, 228, 9,
      /*  862F, 2679.37 ohms */ 219, 158, 224, 76, 216, 231, 217, 215, 220, 160, 216, 249, 223, 116, 219, 196, 230, 18, 229, 36,
      /*  872F, 2698.11 ohms */ 219, 213, 222, 159, 224, 128, 226, 99, 228, 72, 222, 180, 226, 113, 225, 135, 231, 45, 223, 182,
      /*  882F, 2716.81 ohms */ 230, 68, 227, 120, 225, 160, 226, 147, 234, 28, 226, 157, 224, 200, 222, 247, 227, 154, 232, 77,
      /*  892F, 2735.47 ohms */ 224, 222, 236, 29, 233, 75, 224, 239, 232, 99, 233, 88, 227, 195, 240, 1, 238, 30, 255, 255 };

   static unsigned int last_temp_sent = 0xffff;
   if (last_temp_sent == temp) return; // don't resend the same temp again
   last_temp_sent = temp;

   if (temp < F_LOW) temp = F_LOW;
   if (temp > F_HIGH) temp = F_HIGH;
   int ndx = (temp - F_LOW) << 1;
   //lcd.setCursor(15, 2); lcd.print("    "); //FOR TESTING
   //lcd.setCursor(15, 2); lcd.print(ndx); //FOR TESTING
   digitalWrite(DPOT_CHANGING, LOW); // signal to the controller that the pots are changing
   delay(200); // long delay in case it had already starting reading them
   set_potentiometer ((unsigned int)pgm_read_byte_near((byte *)temp_to_pot_settings + ndx)); // pot 1 (most significant)
   delayMicroseconds(50);
   set_potentiometer ((unsigned int)pgm_read_byte_near((byte *)temp_to_pot_settings + ndx + 1) | 0x100); // pot 2 (least significant)
   digitalWrite(DPOT_CHANGING, HIGH); }

#else // !DIGITAL_POTS  not using digital pots: send an analog signal indicating the simulated real temp
void send_real_temp(int temp) {
   // we're running at 5V, so analogWrite generates 0 to 5V for 0 to 255
   int rawdata = INTERPOLATE(temp, TEMP_MIN, TEMP_MAX, 0UL, ((unsigned long) TEMP_PWM_MAX_mV * 255UL) / 5000);
   analogWrite(TEMP_OUT, rawdata); }
#endif

void send_knob_data_to_ctlr(byte * digits) {
   char msg[20];
   sprintf(msg, "\"%s\"", (char *) digits);
   lcd.setCursor(15, 2); lcd.print(msg);
   Serial.write(digits[0]);
   Serial.write(digits[1]);
   Serial.write(digits[2]);
   byte checksum = 255 - (digits[0] + digits[1] + digits[2]);
   Serial.write(checksum); }

float delta_now;
bool oven_heat_fast, oven_heated_once;

void simulate_oven(void) {
   char msg[20];
   if (!controlling_temp) {
      oven_heat_fast = true; // start off in fast heat mode
      oven_heated_once = false; // until a heat/noheat cycle happens
      return; }
   unsigned long timenow_msec = millis();
   if (timenow_msec - last_ovensim_time >= MSECS_PER_OVEN_SIM) {  // do an oven simulation cycle
      digitalWrite(LED, LED_ON);

      bool oven_heating =
         #if REAL_BOARD
         read_fake_temp() < OVEN_SETPOINT;
         #else
         digitalRead(OVEN_LOW) == HIGH;
         #endif

      #if DO_IMPULSE
      static bool did_impulse = false;
      if (!did_impulse && (timenow_msec - startsim_time) > IMPULSE_MSEC) {
         ftemp_current += IMPULSE_AMOUNT;
         did_impulse = true; }
      #endif

      //****** START OF SECTION THAT IS IDENTICAL BETWEEN PC SIMULATOR AND ARDUINO MOCK OVEN *********
      // Our oven model:
      //  When fast heating, the temperature change goes from 20 degrees per minute at 100F
      //  to 0 degrees per minute at 900F.
      //  When slow heating, the temperature change goes from 10 degrees per minute at 100F
      //  to 0 degrees per minutes at 900F.
      //  When cooling, the temperature change goes from -5 degrees per minute at 900F
      //  to 0 degree per minute at 70F
      float target_delta; // degrees per minute
      if (oven_heating) {
         oven_heated_once = true;
         if (oven_heat_fast)
            target_delta = INTERPOLATE_BOUNDED(ftemp_current, 100.f, 900.f, 20, 0);
         else
            target_delta = INTERPOLATE_BOUNDED(ftemp_current, 100.f, 900.f, 10, 0); }
      else { // oven not heating
         if (oven_heated_once) oven_heat_fast = false; // stop fast heat mode the first time the oven goes off
         target_delta = INTERPOLATE_BOUNDED(ftemp_current, 70.f, 900.f, 0, -5); }

      // But that's the target rate; we also put a low-pass filter on changing the rate.
#define DELTA_UP   (3*60.f)  // max increase of delta: 3F/sec, or 180F/min (ie bounded second derivative) 
#define DELTA_DOWN (1*60.f)  // max decrease of delta: 1F/sec, or 60F/min (ie bounded second derivative) 
      float minute_fraction = (float)(timenow_msec - last_ovensim_time) / (1000.0f * 60); // scale to the simulation step time
      if (delta_now < target_delta) {
         delta_now += DELTA_UP * minute_fraction;
         if (delta_now > target_delta) delta_now = target_delta; }
      if (delta_now > target_delta) {
         delta_now -= DELTA_DOWN * minute_fraction;
         if (delta_now < target_delta) delta_now = target_delta; }

      // accumulate temp changes in floating point fractions
      ftemp_current += delta_now * minute_fraction;
      temp_current = (int) (ftemp_current + .5);
      //****** END OF IDENTICAL SECTION *******

      last_ovensim_time = timenow_msec; // do it early, because sending temp takes a variable time
      sprintf(msg, "%3dF", temp_current); // display it locally
      lcd.setCursor(5, 1); lcd.print(msg);
      sprintf(oven_display, "%3d",
              temp_current >= 0 && temp_current < 999 ? temp_current : 999); // send it as the knob data
      send_real_temp(temp_current + TEMP_ADJUST); // send the controller the real temperature as changed
      digitalWrite(LED, LED_OFF);
#define INTF(f) f<0? "-":"", (int)fabs(f), (int)(((long)(fabs(f)*1000.))%1000L) // sprintf doesn't do floats!
#define STRF "%s%d.%03d"
      debug_print("sim: temp %dF %s, dutycycle ..., delta " STRF "F/min, target " STRF ", minute frac " STRF "\n",
                  temp_current, oven_heating ? (oven_heat_fast ? "fastheat" : "slowheat") : "cooling ",
                  INTF(delta_now), INTF(target_delta), INTF(minute_fraction)); } }

// knob and controller interface

void update_display(void) {
   if (mode != getmode()) { // mode is changing
      delay(DEBOUNCE); // wait for it to settle
      mode = getmode(); // then read again
      controlling_temp = mode == M_BAKE || mode == M_CONV_BAKE || mode == M_ROAST || mode == M_CONV_ROAST ;
      if (controlling_temp) {
         delta_now = 0.0f;
         startsim_time = millis(); }
      strcpy (oven_display,
              controlling_temp ? "---" : "   ");
      lcd.setCursor(11, 0); lcd.print(modenames[mode]); }

   #if REAL_BOARD
   if (fake_temp != read_fake_temp()) { // fake temp to oven is changing
      fake_temp = read_fake_temp();
      lcd.setCursor(10, 1); lcd.print(fake_temp < OVEN_SETPOINT ? "heating" : "cooling");
      char msg[20];
      sprintf(msg, "fake %3dF", fake_temp);
      lcd.setCursor (0, 2); lcd.print(msg); }
   #else // Nano simulator of our board
   if (ovenfaketemp != getovenfaketemp()) { // fake temp to oven is changing
      delay(DEBOUNCE);
      ovenfaketemp = getovenfaketemp();
      lcd.setCursor(10, 1); lcd.print(ovenfaketempnames[ovenfaketemp]); }
   if (RTDswitch != getRTDswitch()) { // RTD switch is changing
      delay(DEBOUNCE);
      RTDswitch = getRTDswitch();
      lcd.setCursor(5, 2); lcd.print(RTDswitchnames[RTDswitch]); }
   #endif

   if (updn != getupdn()) { // up or down switch changed
      delay(DEBOUNCE);
      updn = getupdn();
      lcd.setCursor(16, 0); lcd.print(updnnames[updn]); } }

void setup(void) {
   for (int ndx = 0;; ) { // configure input pins
      static const byte input_pins[] = {
         MODE_S8, MODE_S4, MODE_S2, MODE_S1,
         KNOB_UP, KNOB_DN,
         #if !REAL_BOARD
         RTD_CTLR, RTD_OVEN, OVEN_LOW, OVEN_HIGH,
         #endif
         0xff };
      byte pin = input_pins[ndx++];
      if (pin == 0xff) break;
      pinMode(pin, INPUT_PULLUP); }
   for (int ndx = 0;;) { // configure output pins
      static const byte output_pins[] = {
         LED, LED_OFF,
         #if REAL_BOARD
         DPOT_CS, HIGH, DPOT_SI, HIGH, DPOT_SCK, LOW, DPOT_CONNECT, HIGH,
         DPOT_CHANGING, HIGH,
         #else
         TEMP_OUT, LOW,
         #endif
         #if DEBUG
         DEBUG_PIN, HIGH,
         #endif
         0xff };
      byte pin = output_pins[ndx++];
      if (pin == 0xff) break;
      pinMode(pin, OUTPUT);
      digitalWrite(pin, output_pins[ndx++]); }

   lcd.begin(20, 4);
   lcd.setBacklight(HIGH);
   lcd.print("Wolf oven simulator");
   delay(2000);
   lcd.clear();
   lcd.setCursor(0, 0); lcd.print("knob");
   lcd.setCursor(0, 1); lcd.print("oven");
   lcd.setCursor(0, 2); lcd.print("RTD: ");
   #if DEBUG
   debug_port.begin(DEBUG_BAUDRATE);
   debug_port.println("*** Oven simulator ***");
   #endif
   Serial.begin(2400, SERIAL_8N2);
   while (Serial.available()) Serial.read(); // flush garbage left over from boot
   send_real_temp(temp_current + TEMP_ADJUST); }

void test_sendtemp(int temp) { //TEMPORARY for testing
   char msg[21];
   sprintf(msg, "send %3dF", temp);
   lcd.setCursor(0, 3); lcd.print(msg);
   send_real_temp(temp);
   delay(250); // in case receiver had the bias voltage on, do it again
   send_real_temp(temp);
   /*update_display();*/
}

#if DIGITAL_POTS
void setpots (unsigned int pot1, unsigned int pot2) {
   set_potentiometer(pot1); // MSB
   delayMicroseconds(50);
   set_potentiometer(pot2 | 0x100); //LSB
   delay(10000); }
#endif

void loop(void) {

   #if 0 //TEST CODE various code for sending temperature
   if (0) { // setup to measure wiper and/or total resistance
      digitalWrite(DPOT_CONNECT, LOW); // disconnect from the RTD connector, else can't set!
      setpots(255, 255);
      while (1) ; }
   for (unsigned int temp = TEMP_MIN; temp <= TEMP_MAX; temp += 50) {
      test_sendtemp(temp);
      delay(5000); }
   while (0) {
      setpots(0, 0);
      setpots(0, 255);
      setpots(255, 0);
      setpots(255, 255); }
   #endif

   update_display();
   simulate_oven();

   if (millis() - last_oven_display_time >= OVEN_DISPLAY_TIME) {
      send_knob_data_to_ctlr((byte *) oven_display);
      last_oven_display_time = millis(); }

   static unsigned long first_partial_time = 0;
   int navail;
   while ((navail = Serial.available()) >= 4) { // display packet for knob is available from controller
      byte buf[4];
      char msg[20];
      Serial.readBytes(buf, 4);
      if ((byte)(buf[0] + buf[1] + buf[2] + buf[3]) == 255)
         buf[3] = 0;
      else strcpy((char *)buf, "-CH");
      sprintf(msg, "\"%s\"", (char *)buf);
      lcd.setCursor(5, 0); lcd.print(msg);
      first_partial_time = 0; }
   if (navail > 0) { // if we have 1 to 3 bytes
      if (first_partial_time == 0) first_partial_time = millis(); // record first appearance of a fragment
      else if (millis() - first_partial_time > PURGE_BUFFER_TIME) { // it's hung around too long
         while (Serial.available()) Serial.read(); // so flush it
         first_partial_time = 0; } }

}
//*
