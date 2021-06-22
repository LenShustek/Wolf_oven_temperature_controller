// file: Wolf_temp_controller.ino
/*****************************************************************************

   Wolf oven temperature controller

   This is the software for a small device that attaches behind the oven
   control knob of a Wolf dual-fuel range and uses one of several feedback
   algorithms to achieve temperature control within 2-3 degrees Fahrenheit.
   With the standard Wolf control, the temperature oscillates about 25
   degrees with a cycle time of 5-6 minutes.

   The device is a 1.5" x 2.5" printed circuit board that plugs into the
   back of the control, replacing the 10-conductor cable that comes from
   the oven's "ECH" board. The original cable then plugs into the back of
   our board. We also interrupt the cable that goes to the "Oven Controller"
   board with a special extension cable that diverts the RTD ("resistance
   temperature detector") inputs to our board and allows us to simulate
   a different RTD signal back to the oven.

   The Wolf oven selector switch has three functions: a 9-position mode
   selection bezel, an "up/down" switch for changing the temperature,
   and a 3-digit 7-segment display for temperature and some messages

   The knob is normally connected to the "ECH" board that has the fancy
   display and buttons. With our board in place, we leave the mode switch
   connected as normal, but control whether the oven see the up/down switch.
   We monitor what the oven wants to display on the knob, and either
   display that or our own information.

   We use the up/down switch when the mode is "OFF" to access our
   configuration information. When we are controlling the oven, we read
   the current temperature from the RFD sensor, and periodically tell the
   oven to turn the heater on or off by faking to it that the temperature
   is either very low or very high.

   Our board contains an 8 Mhz ATMega328P processor with 32K of Flash
   program memory, 2K of RAM, and 1K of non-volatile EEPROM.
   It also has:
    - a MAXIM MAX31865 RTD temperature sensor interface
    - eight Toshiba TLP3306 solid-state relays for switching RTD inputs
      and connecting or disconnecting the up/down switch to the oven
    - a power-harvesting circuit to get power from the knob serial data line
    - a serial-over-power circuit to provide power to the knob display

   Tips for programming the "bare" ATMega328P on our PC board:
    - There is only 2K of RAM, so be frugal allocating array variables!
      Put constant data in flash memory by using PROGMEM.
    - A 2x3 pin header mates to the Pololu USB AVR Programmer v2.1,
      configured with their utility program for 3.3V, but not to supply power
      to the board. The red cable stripe goes next to the dot.
    - Power the board from the oven serial port power harvester.
      (The Pololu won't power the board. Why??)
    - Select board "Arduino/Genuino Uno".
    - The boards.txt file in C:\Users\len\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.6.21\...
      has to be changed to allow the frequency to be set to other than 16 Mhz.
        #uno.build.f_cpu=1000000L
        ...(then at the end of uno section)...
        menu.speed=CPU Speed
        uno.menu.speed.1=1 MHz
        uno.menu.speed.2=2 MHz
        uno.menu.speed.4=4 MHz
        uno.menu.speed.8=8 MHz
        uno.menu.speed.16=16 MHz
        uno.menu.speed.1.build.f_cpu=1000000L
        uno.menu.speed.2.build.f_cpu=2000000L
        uno.menu.speed.4.build.f_cpu=4000000L
        uno.menu.speed.8.build.f_cpu=8000000L
        uno.menu.speed.16.build.f_cpu=16000000L
      (Might need to uncheck "aggressively cache compiled core" in files/preferences after changing it?)
      See https://tttapa.github.io/Pages/Arduino/Bootloaders/ATmega328P-custom-frequency.html
   - While you're at it, to get sprintf() and friends to do floating-point formats for debugging output,
     add this too:
        menu.printf=AVR printf Version
        uno.menu.printf.full=Full printf
        uno.menu.printf.full.compiler.c.elf.extra_flags=-Wl,-u,vfprintf -lprintf_flt
        uno.menu.printf.default=Default printf
        uno.menu.printf.default.compiler.c.elf.extra_flags=
        uno.menu.printf.minimal=Minimal printf
        uno.menu.printf.minimal.compiler.c.elf.extra_flags=-Wl,-u,vfprintf -lprintf_mi
      See https://forum.arduino.cc/index.php?topic=344206.new#new
   - Use AVRDUDE to check and modify the clock configuration fuse settings.
       avrdude -c stk500v2 -P COM9 -p m328p  (shows signature and current fuses)
          avrdude: Device signature = 0x1e950f (probably m328p)
          avrdude: safemode: Fuses OK (E:FF, H:D9, L:62)
      The ATMega328p comes with the low fuse preprogrammed as 0x62, which says to use
      the internal RC 8 Mhz ocillator divided by 8, producing a system clock of 1 Mhz.
      We change to a system clock of 8 Mhz, by programming the low fuse to be 0xE2 instead.
      (If you don't change to 8 Mhz, the serial port clock will not be accurate enough.)
      Also, the default for the high fuse is 0xD9, which causes EEPROM to be erased during programming.
      We change EESAVE (D3) to 0 by programming the high fuse to be 0xD1 instead.
      All those changes are effected by this command:
         avrdude -c stk500v2 -P COM9 -p m328p -U hfuse:w:0xd1:m -U lfuse:w:0xe2:m
   - To download the program, elect the "Atmel STK500 development board" programmer in the Aduino IDE
      tools/programmer menu, then "Upload using programmer". It takes about 10 seconds.

****** change log ******

   10 Dec 2020, L. Shustek, V1.0
   24 Dec 2020, L. Shustek, V1.1 for V1.1 hardware with RTD
   10 Feb 2021, L. Shustek, V1.2 for V1.2 hardware with UP/DOWN disconnects
   24 Feb 2021, L. Shustek, V1.2.1, add processor speed calibration, "coast" mode heating
    5 Mar 2021, L. Shustek, V1.2.2, change coast mode to "temporary shutoff to cause slow heating".
                            Change to Abderraouf Adjal's Embedded-PID library instead of FastPID.
                            Keep track of reboots in EEPROM and display on startup.
   13 Mar 2021, L. Shustek, V1.2.3, change to Brett Beauregard's PID library, version 1.2.1,
                              with a mod so that compute() is called only when calculation is to be done.
                            Add Kp, Ki, and Kd to config info, and generalize how config data is handled.
                            Use input low on debug port R to tell us that digital pots on the mockup
                              board are changing and we should delay reading them.
                            Try to preserve oscillator calibration when the config data format changes.
    4 Apr 2021, L. Shustek, V1.2.4
                            - fix do_shutdown: was set false always, not just for initialization
                            - for PID, debug output of I is now cumulative value
                            - make double sure we own the RTD when we're faking temps to the oven
                            - have turn_oven_on() _off() always set fake temp resistors, regardless of oven_on
                            - call turn_over_off() when mode changes to one we don't control
                            - put a "low pass filter" on mode changes: don't act for a couple of seconds
                            - add "InF" command to show info, for now just the firmware version
   ----------------------------------------------------------------------------------------------------
   Copyright (c) 2020, 2021, Len Shustek; released under The MIT License (MIT)

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
 ********************************************************************************************************/

/* TODO
   - for the H1 algorithm: record stable duty cycle in non-voltatile memory? Make it a function of temperature?
   - allow export of temp data even when we aren't controlling temp <--- CAN'T DO WITH NEW RTD SCHEME!
*/

#define VERSION "124"           // (must be 3 chars for knob "Info" display)
#define DEBUG 1                 // generate debugging output on a serial port?
#define DEBUG_BAUDRATE 115200   // (seems to work ok for transmit with the internal 8Mhz ATMega328P clock)
#define DEBUG_MOSI 1            // debugging output on MOSI (pin 11), so on the front panel of the oven? 
//                              // otherwise it's on pin A3 to the debug header the board.
#define OSCILLATOR_DEFAULT 170  // the default OSCCAL (oscillator calibration) value to get us close to 8Mh
#define OSCILLATOR_MIN 145
#define OSCILLATOR_MAX 200

#define REAL_BOARD 1            // are we running on the real temp controller board, or a Nano simulator?
#define ANALOG_TEMP_IN 0        // using analog temp in from the mockup board, not a real RTD    

#define EXPORT_MOSI 0           // export temperature data on MOSI (pin 11) at EXPORT_BAUDRATE?
#define EXPORT_DEBUG 1          // export temperature data on whereever the debug serial port is, at the DEBUG_BAUDRATE?
#define EXPORT_KNOB 0           // export temperature data via the knob phono jack (pin 1) at 2400 baud?
#define EXPORT_BAUDRATE 115200  // (for EXPORT_MOSI only)

#define KNOB_DISPLAY_REPEAT_TIME 150  // msec after which to repeat the knob display
#define FAST_KNOB_TIME 1000UL         // msec that defines "fast knob up/down release"
#define SLOW_KNOB_TIME 5*1000UL       // msec that defines "slow knob up/down release"
#define DISPLAY_CHANGE_TIME 1500UL    // msec between changes of temperature display on knob
#define DISPLAY_HOLDOFF_TIME 3000UL   // msec to hold off display changes after setpoint change
#define MENU_DSP_TIME 2000UL          // msec to display each menu item
#define REPEAT_START_TIME 1000UL      // msec before knob repeat starts
#define REPEAT_DELTA_TIME 250UL       // msec between knob repeats
#define NUM_DONE_TIME 3000UL          // msec after which number entry ends
#define DISPLAY_INFO_TIME 2000UL      // msec to show informational display
#define MODE_PERSIST_TIME 1500UL      // msec that mode change has to persist before we act on it
#define PURGE_BUFFER_TIME 50UL        // msec after which we purge fragments in the receive buffer
#define EMPTY_BUFFER_TIME 20UL        // msec of quiet after which we think we're between packets
#define DEBOUNCE 50                   // msec of switch debounce delay

#define DEFAULT_DUTY_CYCLE 50            // default duty cycle (0 to 100%) for testing
#define DEFAULT_CYCLE_PERIOD (60*1000UL) // msec default algorithm cycle period (at most one oven relay cycle each time)
#define DEFAULT_PID_P_WEIGHT 4.0f        // default PID weights 
#define DEFAULT_PID_I_WEIGHT 0.50f
#define DEFAULT_PID_D_WEIGHT 0.10f
#define OVEN_MIN_ON_TIME 10*1000UL       // msec minimum oven on and off times
#define OVEN_MIN_OFF_TIME 10*1000UL      // (don't turn the oven heating relays off or on for too short a time)
#define MIN_CYCLE_PERIOD (OVEN_MIN_ON_TIME + OVEN_MIN_OFF_TIME + 1000)
#define MAX_CYCLE_PERIOD 5*60*1000UL
#define MAX_TEMP_SETPOINT 550
#define MIN_TEMP_SETPOINT 100
#define DEFAULT_TEMP_HISTORY_DELTA (10*1000UL) // default msec between temperature history samples 
#define TEMP_HISTORY_NUM 450             // number of samples (approximately) to keep; 10 sec default implies 1h15m
//                                       // in changing, keep an eye on the local variable space left of our 2K

#include "PID_v1.h"  // Brett Beauregard's PID library, version 1.2.1
#define INTERPOLATE(x,x0,x1,y0,y1) ((long)y0*(x1-x)+(long)y1*(x-x0))/(x1-x0)  // watch for bounds and overflow!

#include <EEPROM.h>
#if DEBUG || EXPORT_MOSI
   #include "SoftwareSerial.h"  // our local copy that has a minimal receive buffer
#endif

#define MOSI 11
#if DEBUG
SoftwareSerial debug_port (/*RX*/ A7, /*TX*/ DEBUG_MOSI ? MOSI : A3); // (receive is not used)
#define MAXLINE 80              // maximum debugging line
#endif

#if EXPORT_MOSI
   #if DEBUG_MOSI
      #define export_port debug_port
   #else
      SoftwareSerial export_port (/*RX*/ 12, /*TX*/ MOSI);
   #endif
#elif EXPORT_DEBUG
   #define export_port debug_port
#elif EXPORT_KNOB
   #define export_port Serial
#endif

// hardware configuration
// (the ATMega328P bit definitions are in iom328p.h)

/* The multiple schemes for pin numbering is confusing!
   IDE  port  TQFP pin  use
   A0  PC0      23     -CS to RTD reader IC
   A1  PC1      24     our LED
   A2  PC2      25     analog temp in from mockup board
   A3  PC3      26     TxD to debug port, and sometimes exported data
   A4  PC4      27     connect RTD to us
   A5  PC5      28     connect RTD to oven
   --  ADC6     19     (unusable for digital I/O)
   --  ADC7     22     (unusable for digital I/O)
   0   PD0      30     RxD oven's knob display info
   1   PD1      31     TxD output to knob display
   2   PD2      32     temp down from switch
   3   PD3      1      temp up from switch
   4   PD4      2      switch mode bit 0
   5   PD5      9      switch mode bit 1
   6   PD6      10     switch mode bit 2
   7   PD7      11     switch mode bit 3
   8   PB0      12     send oven a low temperature
   9   PB1      13     send oven a high temperature
   10  PB2      14     serial data to RTD reader
   11  PB3      15     programming port MOSI, and sometimes exported data and/or debugging info, and OCR2A
   12  PB4      16     programming port and RTD reader MISO
   13  PB5      17     programming port and RTD reader SCK (or LED on Nano mockup board)
   --  PC6      29     programming port CPU reset  */

#define RxD 0   // serial receive data port for knob data
#define TxD 1   // serial transmit data port to knob
#define KNOB_BAUDRATE 2400

#if REAL_BOARD
   #define LED A1           // the LED on our board
   #if ANALOG_TEMP_IN
      #define OVEN_TEMP_IN A2   // use the debug R pin to sense the analog temp from the test board
   #else
      #define DIGITAL_POTS_CHANGING A2 // use the debug R pin low to know when the digital pots are changing
   #endif
#else
   #define LED 13           // Arduino Nano's LED
   #define OVEN_TEMP_IN A7  // the simulated temperature: 0..1023 for TEMP_IN_MIN..TEMP_IN_MAX
#endif
#if !REAL_BOARD || ANALOG_TEMP_IN
   #define TEMP_IN_MIN 50
   #define TEMP_IN_MAX 664  // theoretically 600, but this calibrates for power supply differences
#endif

#define LED_ON HIGH
#define LED_OFF LOW


#define MODE_S8 7    // inputs that give switch info
#define MODE_S4 6
#define MODE_S2 5
#define MODE_S1 4
#define KNOB_UP 3
#define KNOB_DN 2
enum updown_t {UP, DOWN};

#define MOSI 11        // standard SPI signals
#define MISO 12
#define SCK 13

#define RTD_CS A0      // chip select to RTD reader
#define RTD_SDI 10     // serial data to RTD reader
#define RTD_SDO MISO   // serial data from RTD reader (MISO)
#define RTD_CLK SCK    // clock to RTD reader (SCK)

#define UPDOWN_TO_OVEN A5 // output high to connect up/down switch to the oven
#define RTD_TO_US A4      // output high to connect RTD to us
#define OVEN_LOW_TEMP 8   // output high to send the oven a low fake temperature
#define OVEN_HIGH_TEMP 9  // output high to send the oven a high fake temperature

enum ovenmode_t {M_OFF, M_BAKE, M_02, M_03, M_CONV_ROAST, M_PROOF, M_CONV, M_CLEAN,
                 M_CONV_BAKE, M_BROIL, M_10, M_11, M_CONV_BROIL, M_ROAST, M_14, M_15 };
const bool oven_mode_heating[16] = {0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0 }; // which modes are heating

bool knob_up = false, knob_dn = false; // what our physical knob does
bool controlling_temp = false;
bool repeating = false;
int temp_setpoint = 300, temp_current = 115;
byte ovenmode = M_14;
byte current_duty_cycle = DEFAULT_DUTY_CYCLE; // 0 to 100%, except not close to either:
byte min_duty_cycle = (OVEN_MIN_ON_TIME * 100) / DEFAULT_CYCLE_PERIOD; // could be 0 too, of course
byte max_duty_cycle = 100 - (OVEN_MIN_OFF_TIME * 100) / DEFAULT_CYCLE_PERIOD; // could be 100 too, of course
unsigned long knob_change_time;
unsigned long last_cycle_time;
unsigned long last_display_change_time = 0;
unsigned long last_knob_display_time = 0;
enum {DSP_START, DSP_TEMP_NOW, DSP_TEMP_SET, DSP_DUTY_CYCLE, DSP_END };
byte next_display = DSP_TEMP_SET; // (can't increment enums in C++)
bool display_holdoff = false;
char oven_display[4]; // what the oven is asking to display
char knob_display[4]; // what we are displaying on the knob
enum config_state_t {CS_IDLE, CS_RIGHT1, CS_LEFT2, CS_RIGHT2, CS_DOIT } config_state;

enum calc_mode_t {CALC_PID, CALC_H1, CALC_H2, CALC_NONE, CALC_DUTYCYCLE };
#define CALC_DEFAULT CALC_PID;
#if DEBUG
const char* calc_mode_names[] = {"PID", "H1", "H2", "none", "const duty"};
#endif

#if DEBUG
#define debug_print(format,...) debug_printer (F(format),__VA_ARGS__)
void debug_printer(const __FlashStringHelper *format, ...) {
   char buf[MAXLINE];
   unsigned long timenow = millis();
   snprintf_P(buf, sizeof(buf), PSTR("at %3d.%01d secs: "), (unsigned short)(timenow / 1000), (unsigned short) ((timenow % 1000) / 100));
   debug_port.write(buf);
   va_list argptr;
   va_start(argptr, format);
   vsnprintf_P(buf, MAXLINE, (PGM_P)(format), argptr);
   debug_port.write(buf);
   va_end(argptr); }
#else
#define debug_print(...)
#endif

void show_displays() {
   debug_print("oven \"%s\", knob \"%s\"\n", oven_display, knob_display); }

void output_to_knob(void) {
   Serial.write(knob_display[0]); // at 2400 baud, each character takes 4.2 msec
   //delay(7); // delay a bit more, in case the knob  can't cope with back-to-back characters
   Serial.write(knob_display[1]);
   //delay(7);
   Serial.write(knob_display[2]);
   //delay(7);
   byte checksum = 255 - (knob_display[0] + knob_display[1] + knob_display[2]);
   Serial.write(checksum);
   //show_displays();
   last_knob_display_time = millis(); }

void set_knob_display(const byte *digits) { // setup for periodic display to the knob display
   memcpy(knob_display, digits, 3);
   output_to_knob(); } // do the first one; then it's done periodically

void flash3times (const char * msg) {
   for (byte i = 0; i < 3; ++i) {
      digitalWrite(LED, LED_ON);
      set_knob_display((byte *)msg);
      delay(400);
      set_knob_display((byte *)"   ");
      digitalWrite(LED, LED_OFF);
      delay(400); } }

void fatal(byte errcode) {
   char msg[4];
   debug_print("FATAL ERR %d\n", errcode);
   flash3times("Err");
   variable_to_knob (errcode, false);
   delay(1000); }

byte getovenmode() {  // this could return momentary garbage because of noise glitches
   byte ovenmode = digitalRead(MODE_S8);
   ovenmode = (ovenmode << 1) | digitalRead(MODE_S4);
   ovenmode = (ovenmode << 1) | digitalRead(MODE_S2);
   ovenmode = (ovenmode << 1) | digitalRead(MODE_S1);
   return 15 - ovenmode; }
#if DEBUG
const char *ovenmodenames[16] = {
   "OFF ", "BAKE", "MD02", "MD03", "CRST", "PROF", "CONV", "CLN ",
   "CBAK", "BRL ", "MD10", "MD11", "CBRL", "RST ", "MD14", "MD15" };
#endif

bool knob_turned(enum updown_t dir) { // see if the knob is solidly turned in the specified direction
   byte dn_dir = 1, up_dir = 1;
   if (dir == UP) up_dir = 0;
   if (dir == DOWN) dn_dir = 0;
   if (digitalRead(KNOB_DN) != dn_dir || digitalRead(KNOB_UP) != up_dir)
      return false; // not even momentarily there
   delay(DEBOUNCE); // it's a possible: wait for a while
   return digitalRead(KNOB_DN) == dn_dir && digitalRead(KNOB_UP) == up_dir; // return TRUE if it persisted
}
void knob_waitrelease(void) { // wait for knob to be released
   while (digitalRead(KNOB_UP) == 0 || digitalRead(KNOB_DN) == 0) ;
   delay(DEBOUNCE); }

//
// temperature routines
//
#if !REAL_BOARD || ANALOG_TEMP_IN
int read_temp(void) { // read the temp from the oven simulator on the Nano
   int rawdata = analogRead(OVEN_TEMP_IN);
   return INTERPOLATE(rawdata, 0, 1023, TEMP_IN_MIN, TEMP_IN_MAX); }

#else // read from the Maxim MAX31865 RTD-to-Digital Converter

void RTD_write_byte(byte value) {
   digitalWrite(RTD_CS, 0); // chip select
   for (byte b = 0; b < 8; ++b) {
      digitalWrite(RTD_CLK, 0);
      digitalWrite(RTD_SDI, value & 0x80 ? 1 : 0);
      digitalWrite(RTD_CLK, 1);
      value <<= 1; } }

void RTD_write_reg(byte regno, byte value) {
   RTD_write_byte(regno);
   RTD_write_byte(value);
   digitalWrite(RTD_CS, 1); // unselect
}
byte RTD_read_reg(byte regno) {
   RTD_write_byte(regno);
   byte value = 0;
   for (byte b = 0; b < 8; ++b) {
      digitalWrite(RTD_CLK, 0);
      value = (value << 1) | digitalRead(RTD_SDO);
      digitalWrite(RTD_CLK, 1); }
   digitalWrite(RTD_CS, 1); // unselect
   return value; }

#define RTD_CONFIG 0b10000100  // Vbias on, one-shot, 2-wire, auto fault detect, 60 Hz filter
//#define RTD_CONFIG 0b00000000  // Vbias off, one-shot, 2-wire, no fault detect, 60 Hz filter
#define RTD_CLEAR_FAULT (RTD_CONFIG | 0b10)  // config, plus "clear fault"
#define RTD_BIAS_ON 0b10000000  // turn the bias voltage on
#define RTD_FAULT_DETECT 0b10000100  // start fault detect with auto timing
#define RTD_START  0b10100000   // start conversion

#define RTD_REF 3000        // our reference resistor
#define RTD_RETRIES 5       // how many times to retry if we get faults
#define RTD_FAULT  0x8000   // a code for "fault"

#define RTD_MINRAW 10912    // our minimum 15-bit raw data, representing 32F
#define RTD_SHIFTBITS 4     // how many bits of the 15 to discard
#define RTD_TABLESIZE 1024  // which then indexes into a table of this size
#define RTD_MAXRAW ((RTD_TABLESIZE-1 << RTD_SHIFTBITS) + RTD_MINRAW) // our maximum 15-bit raw data, representing 766F
static const int16_t temp_table[RTD_TABLESIZE] PROGMEM = {
   32, 32, 33, 34, 34, 35, 36, 36, 37, 38, 38, 39, 40, 40, 41, 42, 42, 43, 44, 44, 45, 46, 46, 47, 48, 48, 49, 50, 50, 51,
   52, 52, 53, 54, 55, 55, 56, 57, 57, 58, 59, 59, 60, 61, 61, 62, 63, 63, 64, 65, 65, 66, 67, 67, 68, 69, 69, 70, 71, 71,
   72, 73, 74, 74, 75, 76, 76, 77, 78, 78, 79, 80, 80, 81, 82, 82, 83, 84, 84, 85, 86, 86, 87, 88, 88, 89, 90, 91, 91, 92,
   93, 93, 94, 95, 95, 96, 97, 97, 98, 99, 99, 100, 101, 101, 102, 103, 103, 104, 105, 106, 106, 107, 108, 108, 109, 110,
   110, 111, 112, 112, 113, 114, 114, 115, 116, 116, 117, 118, 119, 119, 120, 121, 121, 122, 123, 123, 124, 125, 125, 126,
   127, 127, 128, 129, 129, 130, 131, 132, 132, 133, 134, 134, 135, 136, 136, 137, 138, 138, 139, 140, 140, 141, 142, 143,
   143, 144, 145, 145, 146, 147, 147, 148, 149, 149, 150, 151, 151, 152, 153, 154, 154, 155, 156, 156, 157, 158, 158, 159,
   160, 160, 161, 162, 162, 163, 164, 165, 165, 166, 167, 167, 168, 169, 169, 170, 171, 171, 172, 173, 174, 174, 175, 176,
   176, 177, 178, 178, 179, 180, 180, 181, 182, 183, 183, 184, 185, 185, 186, 187, 187, 188, 189, 189, 190, 191, 192, 192,
   193, 194, 194, 195, 196, 196, 197, 198, 198, 199, 200, 201, 201, 202, 203, 203, 204, 205, 205, 206, 207, 207, 208, 209,
   210, 210, 211, 212, 212, 213, 214, 214, 215, 216, 216, 217, 218, 219, 219, 220, 221, 221, 222, 223, 223, 224, 225, 226,
   226, 227, 228, 228, 229, 230, 230, 231, 232, 233, 233, 234, 235, 235, 236, 237, 237, 238, 239, 240, 240, 241, 242, 242,
   243, 244, 244, 245, 246, 246, 247, 248, 249, 249, 250, 251, 251, 252, 253, 253, 254, 255, 256, 256, 257, 258, 258, 259,
   260, 260, 261, 262, 263, 263, 264, 265, 265, 266, 267, 268, 268, 269, 270, 270, 271, 272, 272, 273, 274, 275, 275, 276,
   277, 277, 278, 279, 279, 280, 281, 282, 282, 283, 284, 284, 285, 286, 286, 287, 288, 289, 289, 290, 291, 291, 292, 293,
   294, 294, 295, 296, 296, 297, 298, 298, 299, 300, 301, 301, 302, 303, 303, 304, 305, 306, 306, 307, 308, 308, 309, 310,
   310, 311, 312, 313, 313, 314, 315, 315, 316, 317, 318, 318, 319, 320, 320, 321, 322, 323, 323, 324, 325, 325, 326, 327,
   327, 328, 329, 330, 330, 331, 332, 332, 333, 334, 335, 335, 336, 337, 337, 338, 339, 340, 340, 341, 342, 342, 343, 344,
   345, 345, 346, 347, 347, 348, 349, 349, 350, 351, 352, 352, 353, 354, 354, 355, 356, 357, 357, 358, 359, 359, 360, 361,
   362, 362, 363, 364, 364, 365, 366, 367, 367, 368, 369, 369, 370, 371, 372, 372, 373, 374, 374, 375, 376, 377, 377, 378,
   379, 379, 380, 381, 382, 382, 383, 384, 384, 385, 386, 387, 387, 388, 389, 389, 390, 391, 392, 392, 393, 394, 395, 395,
   396, 397, 397, 398, 399, 400, 400, 401, 402, 402, 403, 404, 405, 405, 406, 407, 407, 408, 409, 410, 410, 411, 412, 412,
   413, 414, 415, 415, 416, 417, 418, 418, 419, 420, 420, 421, 422, 423, 423, 424, 425, 425, 426, 427, 428, 428, 429, 430,
   430, 431, 432, 433, 433, 434, 435, 436, 436, 437, 438, 438, 439, 440, 441, 441, 442, 443, 443, 444, 445, 446, 446, 447,
   448, 449, 449, 450, 451, 451, 452, 453, 454, 454, 455, 456, 457, 457, 458, 459, 459, 460, 461, 462, 462, 463, 464, 465,
   465, 466, 467, 467, 468, 469, 470, 470, 471, 472, 473, 473, 474, 475, 475, 476, 477, 478, 478, 479, 480, 481, 481, 482,
   483, 483, 484, 485, 486, 486, 487, 488, 489, 489, 490, 491, 491, 492, 493, 494, 494, 495, 496, 497, 497, 498, 499, 499,
   500, 501, 502, 502, 503, 504, 505, 505, 506, 507, 508, 508, 509, 510, 510, 511, 512, 513, 513, 514, 515, 516, 516, 517,
   518, 519, 519, 520, 521, 521, 522, 523, 524, 524, 525, 526, 527, 527, 528, 529, 530, 530, 531, 532, 532, 533, 534, 535,
   535, 536, 537, 538, 538, 539, 540, 541, 541, 542, 543, 543, 544, 545, 546, 546, 547, 548, 549, 549, 550, 551, 552, 552,
   553, 554, 555, 555, 556, 557, 558, 558, 559, 560, 560, 561, 562, 563, 563, 564, 565, 566, 566, 567, 568, 569, 569, 570,
   571, 572, 572, 573, 574, 575, 575, 576, 577, 577, 578, 579, 580, 580, 581, 582, 583, 583, 584, 585, 586, 586, 587, 588,
   589, 589, 590, 591, 592, 592, 593, 594, 595, 595, 596, 597, 598, 598, 599, 600, 600, 601, 602, 603, 603, 604, 605, 606,
   606, 607, 608, 609, 609, 610, 611, 612, 612, 613, 614, 615, 615, 616, 617, 618, 618, 619, 620, 621, 621, 622, 623, 624,
   624, 625, 626, 627, 627, 628, 629, 630, 630, 631, 632, 633, 633, 634, 635, 636, 636, 637, 638, 639, 639, 640, 641, 642,
   642, 643, 644, 645, 645, 646, 647, 648, 648, 649, 650, 651, 651, 652, 653, 654, 654, 655, 656, 657, 657, 658, 659, 660,
   660, 661, 662, 663, 663, 664, 665, 666, 666, 667, 668, 669, 669, 670, 671, 672, 672, 673, 674, 675, 675, 676, 677, 678,
   678, 679, 680, 681, 681, 682, 683, 684, 684, 685, 686, 687, 687, 688, 689, 690, 691, 691, 692, 693, 694, 694, 695, 696,
   697, 697, 698, 699, 700, 700, 701, 702, 703, 703, 704, 705, 706, 706, 707, 708, 709, 709, 710, 711, 712, 713, 713, 714,
   715, 716, 716, 717, 718, 719, 719, 720, 721, 722, 722, 723, 724, 725, 725, 726, 727, 728, 728, 729, 730, 731, 732, 732,
   733, 734, 735, 735, 736, 737, 738, 738, 739, 740, 741, 741, 742, 743, 744, 744, 745, 746, 747, 748, 748, 749, 750, 751,
   751, 752, 753, 754, 754, 755, 756, 757, 758, 758, 759, 760, 761, 761, 762, 763, 764, 764, 765, 766 };

void RTD_init(void) {
   RTD_write_reg(0x80, RTD_CONFIG); }

uint16_t RTD_read(int trynumber) { // try once to read the raw RTD value; return RTD_FAULT if fault
   #if !ANALOG_TEMP_IN
   if (digitalRead(DIGITAL_POTS_CHANGING) == 0) { // if the mockup controller is telling us...
      debug_print("---digital pots busy\n", 0);
      while (digitalRead(DIGITAL_POTS_CHANGING) == 0) ; }
   #endif
   //RTD_write_reg(0x80, RTD_BIAS_ON);
   //delay(5); // wait for it (min 10 usec time constant x 10 + 1msec
   RTD_write_reg(0x80, RTD_FAULT_DETECT); // start a fault detect cycle
   while (RTD_read_reg(0x00) & 0x0c) ;    // wait for "fault detect finished"
   delay(10);  // why? can't hurt...
   RTD_write_reg(0x80, RTD_START);  // now start a conversion
   delay(/*60*/ 100); // typical 52 msec, max 55 msec; BUT NEEDS MORE!
   byte msb = RTD_read_reg(0x01);
   byte lsb = RTD_read_reg(0x02);
   uint16_t rawdata = ((uint16_t)msb << 7) | (lsb >> 1); // 15 bits representing R*32768/REF
   if (lsb & 1) { // fault!
      byte fault_status = RTD_read_reg(0x07);
      debug_print("RTD fault on try %d: %02X\n", trynumber, fault_status);
      RTD_write_reg(0x80, RTD_CLEAR_FAULT);
      return RTD_FAULT; }
   if (rawdata < RTD_MINRAW) {
      debug_print("RTD raw temp too small on try %d: %u\n", trynumber, rawdata);
      return RTD_FAULT; }
   if (rawdata > RTD_MAXRAW) {
      debug_print("RTD raw temp too big on try %d: %u\n", trynumber, rawdata);
      return RTD_FAULT; }
   //RTD_write_reg(0x80, RTD_CONFIG); // turn bias voltage off
   return rawdata; }

int read_temp(void) {
   uint16_t rawdata;
   for (byte trycount = 0; trycount < RTD_RETRIES; ++trycount)
      if ((rawdata = RTD_read(trycount + 1)) != RTD_FAULT) break;
   if (rawdata == RTD_FAULT) {
      debug_print("fail: %d consecutive faults\n", RTD_RETRIES);
      return temp_current; } // too many tries: return current temperature
   //debug_print("RTD raw data %04x\n", rawdata);
   // convert the raw data into temperature in Fahrenheit (32-1035) using an Excel-generated table
   // that was computed using the inverse of the Callendar-VanDusen formula
   if (rawdata < RTD_MINRAW)
      rawdata = RTD_MINRAW; // below min temp of 32F
   int index = (rawdata - RTD_MINRAW) >> RTD_SHIFTBITS;  // normalize and discard bits we don't use
   if (index >= RTD_TABLESIZE)
      index = RTD_TABLESIZE - 1; // pin at the max temp in the table
   //debug_print("table index %d\n", index);
   int temp = pgm_read_word_near(temp_table + index);
   //debug_print("temp is %dF\n", temp);
   return temp; }
#endif
//
// oven control routines
//
bool read_oven_display() {
   static unsigned long first_partial_time = 0;
   int navail = Serial.available();
   if (navail >= 4) { // a display packet for the knob is coming from the oven
      Serial.readBytes(oven_display, 4);
      //debug_print(" %02X %02X %02X %02X, %d\n", //TEMPORARY
      //            (byte)oven_display[0], (byte)oven_display[1], (byte)oven_display[2], (byte)oven_display[3], navail);
      if ((byte)(oven_display[0] + oven_display[1] + oven_display[2] + oven_display[3]) == 255)
         oven_display[3] = 0;
      else strcpy((char *)oven_display, "-CH");
      //show_displays();
      first_partial_time = 0;
      return true; }
   if (navail > 0) { // if we have 1 to 3 bytes
      if (first_partial_time == 0) first_partial_time = millis(); // record first appearance of a fragment
      else if (millis() - first_partial_time > PURGE_BUFFER_TIME) { // it's hung around too long
         while (Serial.available()) Serial.read(); // so flush it
         first_partial_time = 0; } }
   return false; }

// purge knob data until we think that we're in between packets of 4 characters
void purge_oven_display(void) {
   unsigned long last_char_time = millis();
   do if (Serial.available()) {
         Serial.read();
         last_char_time = millis(); }
   while (millis() - last_char_time < EMPTY_BUFFER_TIME); }

#define OVEN_SHUTDOWN_TRIGGER_F 50  // starting delta degrees that trigger "slow heat" shutdown
#define OVEN_SHUTDOWN_PCT 90        // percentage of target temp at which we temporarily shut down the oven 
//.                                    to cause it to enter its "slower heat" mode
#define OVEN_SHUTODWN_TIME_MSEC 10*1000UL // how long to shut it down for

bool oven_on = false;
bool oven_do_shutdown = false;
bool oven_doing_shutdown = false;
unsigned long oven_off_time;

void turn_oven_on(void) { // turn oven on by setting a low faked temperature
   digitalWrite(RTD_TO_US, HIGH); // make sure the RTD is ours (shouldn't be necessary...)
   digitalWrite(OVEN_HIGH_TEMP, 0);
   digitalWrite(OVEN_LOW_TEMP, 1);
   if (!oven_on) {
      debug_print("oven on  temp %d setpoint %d\n", temp_current, temp_setpoint);
      extern byte temp_history_oven_on;
      oven_on = true; temp_history_oven_on = true; } }

void turn_oven_off(void) { // turn oven off by setting a high faked temperature
   digitalWrite(RTD_TO_US, HIGH); // make sure the RTD is ours (shouldn't be necessary...)
   digitalWrite(OVEN_LOW_TEMP, 0);
   digitalWrite(OVEN_HIGH_TEMP, 1);
   if (oven_on) {
      debug_print("oven off temp %d setpoint %d\n", temp_current, temp_setpoint);
      extern byte temp_history_oven_off;
      oven_on = false; temp_history_oven_off = true;
      oven_doing_shutdown = false;
      oven_off_time = millis(); } }

// We do a temporary oven shutdown when we're at 90% of the target.
// That switches the oven from its fast heat mode to its slow heat mode.
void temporary_oven_shutdown(void) {
   debug_print("TEMPORARY OVEN SHUTOFF\n", 0);
   turn_oven_off();
   oven_doing_shutdown = true; }

void show_oven_shutdown(const char *msg) {
   debug_print("%s: oven_do_shutdown %d, oven_doing_shutdown %d, temp %d target %d\n",
               msg, oven_do_shutdown, oven_doing_shutdown, temp_current, temp_setpoint); }

//
// process knob up/down movements for:
//  (a) entering config menu when the selector knob mode is "off"
//  (b) changing the value of numeric or yes/no configuration parameters
//  (c) adjusting the temperature setpoint when controlling_temp
//
void variable_to_knob (int value, bool yesno) {
   char msg[4];
   if (yesno)
      sprintf(msg, value ? "yE5" : "no ");
   else snprintf(msg, sizeof(msg), "%3d", value);
   set_knob_display((byte *) msg); }

void change_variable(int delta, int *variable, int minvar, int maxvar) {
   if (delta > 0 && *variable <= maxvar - delta) {
      int remainder = *variable % delta;
      if (remainder == 0) *variable += delta;
      else *variable += delta - remainder; }
   else if (delta < 0 && *variable >= minvar - delta) {
      int remainder = *variable % -delta;
      if (remainder == 0) *variable += delta;
      else *variable -= remainder; }
   variable_to_knob(*variable, minvar == 0 && maxvar == 1); }

// use up/down knob motion to make a change to a variable
// return true if it was changed
bool process_knob_updn(int *variable, int minvar, int maxvar) {
   bool changed = false;
   // (could refactor and extract commonality in these two sections, but code space isn't an issue...)
   if (knob_turned(DOWN)) { // knob "down", ie "left"
      if (!knob_dn) { // initial push
         knob_dn = true;
         if (!variable) { // check for entering config mode
            if (millis() - knob_change_time > FAST_KNOB_TIME) // if it's been a while since last knob movement
               config_state = CS_RIGHT1; // treat this as the first left and then wait for the first right
            else {
               if (config_state == CS_LEFT2) config_state = CS_RIGHT2; // await second right
               else config_state = CS_IDLE; } }
         else {
            change_variable(-1, variable, minvar, maxvar);
            changed = true; }
         knob_change_time = millis();
         delay(DEBOUNCE); }
      else  if (variable) { // we already know it is pushed
         if (!repeating) { // check for repeat start time
            if (millis() - knob_change_time > REPEAT_START_TIME) {
               repeating = true;
               knob_change_time = millis();
               change_variable(-5, variable, minvar, maxvar);
               changed = true; } }
         else { // repeating
            if (millis() - knob_change_time > REPEAT_DELTA_TIME) {
               knob_change_time = millis();
               change_variable(-5, variable, minvar, maxvar);
               changed = true; } } } }
   else if (knob_dn) { // released
      knob_dn = repeating = false;
      display_holdoff = true; // delay the normal display
      last_display_change_time = millis();
      delay(DEBOUNCE); }

   if (knob_turned(UP)) { // knob "up", ie "right"
      if (!knob_up) { // first push
         knob_up = true;
         if (!variable) { // check for entering config mode
            if (millis() - knob_change_time > FAST_KNOB_TIME) // if it's been a while since last knob movement
               config_state = CS_IDLE; // then restart the search for the config mode sequence
            else {
               if (config_state == CS_RIGHT1) config_state = CS_LEFT2; // await second left
               else if (config_state == CS_RIGHT2)
                  config_state = CS_DOIT; // do config (or calibration) when the knob is released
               else config_state = CS_IDLE; } }
         else {
            change_variable(+1, variable, minvar, maxvar);
            changed = true; }
         knob_change_time = millis();
         delay(DEBOUNCE); }
      else if (variable) { // we already know it is pushed
         if (!repeating) { // check for repeat start time
            if (millis() - knob_change_time > REPEAT_START_TIME) {
               repeating = true;
               knob_change_time = millis();
               change_variable(+5, variable, minvar, maxvar);
               changed = true; } }
         else { // repeating
            if (millis() - knob_change_time > REPEAT_DELTA_TIME) {
               knob_change_time = millis();
               change_variable(+5, variable, minvar, maxvar);
               changed = true; } } } }
   else if (knob_up) { // released
      knob_up = repeating = false;
      if (config_state == CS_DOIT) { // successfully did fast config sequence: left/right/left/right
         config_state = CS_IDLE;
         if (millis() - knob_change_time < FAST_KNOB_TIME) // if it was released fast,
            do_config();  // show the configuration menu
         else if (millis() - knob_change_time > SLOW_KNOB_TIME) // if it was released really, really slowly
            do_oscillator_calibration(); // do the processor clock calibration
         config_state = CS_IDLE; }
      display_holdoff = true; // delay the normal display
      last_display_change_time = millis();
      delay(DEBOUNCE); }
   return changed; }

//
// Routines for non-volatile storage of configuration information
//
#define EEPROM_SIZE 1024
#define REBOOT_COUNTER_LOC EEPROM_SIZE-2 // last two bytes 

// keep a count of reboots at the end of the EEPROM
uint16_t increment_reboot_counter(void) {
   uint16_t count;
   *((byte *)&count) = EEPROM.read(REBOOT_COUNTER_LOC);
   *((byte *)&count + 1) = EEPROM.read(REBOOT_COUNTER_LOC + 1);
   if (count == 0xffff) count = 0; // unprogrammed EEPROM
   if (count < 9999) { // pin at 4 digits, rather than wraparound
      ++count;
      EEPROM.write(REBOOT_COUNTER_LOC, *((byte*)&count));
      EEPROM.write(REBOOT_COUNTER_LOC + 1, *((byte*)&count + 1)); }
   return count; }

#define CONFIG_VERSION 3
#define CONFIG_ID "Cfg"
struct config_data_t { // configuration data at the start of the EEPROM
   char id[4];                      // CONFIG_ID
   byte version;                    // configuration format version
   byte options;                    // option bits
#define OPT_SHOW_DUTY_CYCLE 0x01
#define OPT_DEFAULT 0
   byte oscillator_calibration;     //  OSCCAL hardware register; DON'T MOVE THIS!
   byte duty_cycle;                 // 0 to 100%, for some algorithms
   double Kp, Ki, Kd;               // coefficients for the PID algorithm
   enum calc_mode_t calc_mode;      // what is the current calculation algorithm
   unsigned long cycle_period_msec; // time between algorithm computations
   unsigned long temp_history_delta_msec; // time between temperature log entries
} config;

void show_config(const char *msg) {
   debug_print("config V%d %s:\n", config.version, msg);
   debug_print("...oscillator %d\n", config.oscillator_calibration);
   debug_print("...fixed duty cycle %d%%\n", config.duty_cycle);
   debug_print("...PID coefficients Kp=%.1f Ki=%.1f Kd=%.1f\n", config.Kp, config.Ki, config.Kd);
   debug_print("...do \"%s\" every %lu msec\n", calc_mode_names[config.calc_mode], config.cycle_period_msec);
   debug_print("...record temp every %lu msec\n", config.temp_history_delta_msec); }

void reset_config(byte oscillator_candidate) {
   strcpy(config.id, CONFIG_ID);
   config.version = CONFIG_VERSION;
   config.options = OPT_DEFAULT;
   config.oscillator_calibration = // try to preserve an existing calibration
      oscillator_candidate >= OSCILLATOR_MIN && oscillator_candidate <= OSCILLATOR_MAX ?
      oscillator_candidate : OSCILLATOR_DEFAULT;
   config.duty_cycle = DEFAULT_DUTY_CYCLE;
   config.Kp = DEFAULT_PID_P_WEIGHT;
   config.Ki = DEFAULT_PID_I_WEIGHT;
   config.Kd = DEFAULT_PID_D_WEIGHT;
   config.calc_mode = CALC_DEFAULT;
   config.cycle_period_msec = DEFAULT_CYCLE_PERIOD;
   config.temp_history_delta_msec = DEFAULT_TEMP_HISTORY_DELTA; }

void write_config () {
   for (byte i = 0; i < sizeof(config); ++i)
      EEPROM.update(i, ((byte *) &config)[i]);
   show_config("written"); }

void read_config (void) {
   for (byte i = 0; i < sizeof(config); ++i) { // read the EEPROM
      ((byte *) &config)[i] = EEPROM.read(i); }
   bool id_ok = strcmp(config.id, CONFIG_ID) == 0;
   if (!id_ok || config.version != CONFIG_VERSION) {  // must initialize
      reset_config(id_ok ? config.oscillator_calibration : 0);
      write_config(); } }

//
// temperature history log routines
//
/*  We log the current oven temperature, plus two bits that indicate whether the oven heater went
    on or off since the last history entry was made. All three are combined into a "delta".

    In order to get the most out of our paltry 2K of RAM, we do delta-encoding of the
    temperature history. Deltas from -63F to +63F, the common case, are stored in one byte.
    If it is outside that range, it's stored as two bytes in big-endian order.
    Big positive deltas are stored as delta+64*256, so the first byte is 64 to 127.
    Big negative deltas are stored as delta-64*256, so the first byte is -64 to -128.
    The tricky part is removing old entries as the history wraps around in the buffer,
    because each might be one or two bytes long, and we might need one or two for the
    new entry of one or two bytes.
*/

int8_t temp_history[TEMP_HISTORY_NUM];    // the temperature deltas, 1 or 2 bytes each
int temp_history_next = 0;                // index of next slot to use
int temp_history_first = 0;               // index of the first (oldest) slot in use
int temp_history_prior = 0;               // temperature before the deltas in the history log
int temp_history_latest = 0;              // temperature of the newest entry
byte temp_history_oven_on = false;        // was the oven ever on since the last history entry?
byte temp_history_oven_off = false;       // was the oven ever off since the last history entry?
byte temp_history_fullness = 0;           // how full the log is: 0 to 100%
unsigned long temp_history_last_time = 0; // when the last sample was done

void temp_history_clear(void) {
   temp_history_next = temp_history_first = 0;
   temp_history_fullness = 0;
   temp_history_last_time = 0;
   temp_history_oven_on = oven_on;
   temp_history_oven_off = !oven_on;
   temp_history_prior = temp_history_latest = (temp_current << 2) | (temp_history_oven_off << 1) | temp_history_oven_on; }

void temp_history_need(int count) { // make room for "count" bytes starting at temp_history_next
   int next = temp_history_next;
   while (count-- >= 0) {
      if (++next >= TEMP_HISTORY_NUM) next = 0; // imagine we use up the entry at next
      if (next == temp_history_first) {  // if history is then full, we have to remove an entry at the beginning
         temp_history_fullness = 100;  // note that we've reached capacity
         int delta = temp_history[temp_history_first]; // get the oldest delta and remove it
         if (++temp_history_first >= TEMP_HISTORY_NUM) temp_history_first = 0;
         if (delta >= 64 || delta <= -64) { // it's a two-byte delta
            byte delta_LSB = temp_history[temp_history_first]; // get and remove the second delta byte
            if (++temp_history_first >= TEMP_HISTORY_NUM) temp_history_first = 0;
            if (delta > 0) delta -= 64; else delta += 64; // undo the big delta adjustments
            delta = (delta << 8) | delta_LSB;
            --count; } // note that we took away two bytes
         temp_history_prior += delta; } // adjust the temp prior to the history based on the entry we removed
   } }

void temp_history_add (int temp) {
   temp = (temp << 2) | (temp_history_oven_off << 1) | temp_history_oven_on; // combine with "oven was on or off at some point" bits
   temp_history_oven_on = oven_on; // initial condition for next interval
   temp_history_oven_off = !oven_on;
   int delta = temp - temp_history_latest;
   temp_history_latest = temp;
   if (delta < 64 && delta > -64) { // single-byte delta
      temp_history_need(1); // make room for it
      temp_history[temp_history_next] = delta;
      if (++temp_history_next >= TEMP_HISTORY_NUM) temp_history_next = 0;
      //debug_print("short delta %d, first %d next %d\n", delta, temp_history_first, temp_history_next);
   }
   else {  // we need to store a two-byte adjusted delta
      temp_history_need(2); // make room for 2 bytes
      //debug_print("long real delta %d adjust to ", delta);
      if (delta >= 0) delta += 64 * 256;
      else delta -= 64 * 256;
      temp_history[temp_history_next] = delta >> 8; // MSB
      if (++temp_history_next >= TEMP_HISTORY_NUM) temp_history_next = 0;
      temp_history[temp_history_next] = delta & 0xff; // LSB
      if (++temp_history_next >= TEMP_HISTORY_NUM) temp_history_next = 0;
      //debug_print("%d, first %d next %d\n", delta, temp_history_first, temp_history_next);
   }
   if (temp_history_fullness < 100)
      temp_history_fullness = 100UL * temp_history_next / (TEMP_HISTORY_NUM - 1);  }

void temp_history_export1(unsigned long timenow, int temp) { // export one temperature value in CSV format
   char buf[25];
   snprintf_P(buf, sizeof(buf), PSTR("%3d.%01d, %d, %d\n"),
              (unsigned short)(timenow / 1000), (unsigned short)((timenow % 1000) / 100),
              temp >> 2, // the temperature
              (temp & 3) == 0 ? -100   // 2 heater status bits (00 shouldn't happen)
              : (temp & 3) == 1 ? 100  // oven was only on in this interval
              : (temp & 3) == 2 ? 0    // oven was only off
              : 50);                   // oven was both on and off
   export_port.print(buf);
   delay(100); // don't overrun serial buffers
}
void temp_history_export() {
   #if EXPORT_MOSI && !DEBUG_MOSI
   export_port.begin(EXPORT_BAUDRATE);
   #endif
   #if EXPORT_KNOB
   while (getovenmode() == ovenmode) ; // wait for mode selector switch to jiggle
   #endif
   digitalWrite(LED, LED_ON);
   export_port.println("secs,temp,heating");
   temp_history_export1(0, temp_history_prior); // output the temp prior to the history
   unsigned long timenow = 0;
   int tempnow = temp_history_prior;
   int ndx = temp_history_first; // start with the oldest entry
   if (temp_history_last_time) do { // if we made any entries, dump them all
         timenow += config.temp_history_delta_msec;
         int delta = temp_history[ndx];
         if (delta >= 64 || delta <= -64) { // it's a two-byte delta
            if (++ndx >= TEMP_HISTORY_NUM) ndx = 0; // point to the second delta byte
            byte delta_LSB = temp_history[ndx]; // get it
            //debug_print("long delta MSB %d LSB %d, recomputed ", delta, delta_LSB);
            if (delta > 0) delta -= 64; else delta += 64; // undo the big delta adjustments
            delta = (delta << 8) | delta_LSB;
            //debug_print("%d\n", delta);
         }
         else {
            //debug_print("short delta %d\n", delta);
         }
         tempnow += delta;
         temp_history_export1(timenow, tempnow);
         if (++ndx >= TEMP_HISTORY_NUM) ndx = 0; // point to the next entry's delta byte
      }
      while (ndx != temp_history_next);
   digitalWrite(LED, LED_OFF); }
//
// one-time calibration of the processor clock
//
/* Try a range of clock values, waiting for the user to tell us which is good, either:
   - by watching the knob display and/or debugging serial output, and using the DOWN knob motion
     to identify first the the lowest and then the highest frequencies that work, which we average
   - if MOSI is not being used for debugging, by watching an oscilloscope connected to MOSI,
     and using the UP knob motion to identify the perfect 1 Khz frequency
   We then store that in EEPROM, and use it from then on at every reboot.
*/
void do_oscillator_calibration() {
   byte osccal_low = 0, osccal_high = 0;  // lowest and highest good values
   byte osccal_candidate;
   #if !DEBUG_MOSI
   TCCR2A = 0; // (needed why? nobody knows, but it is!)
   TCCR2B = 0; // setup timer2 for 1 Khz square wave on OC2A (pin 11, MOSI)
   TCNT2 = 0;
   TIMSK2 = 0;
   // The following only works at 8Mhz or less; otherwise OCR2A overflows and prescaler has to change.
   OCR2A = F_CPU / (1000UL * 2 * 32)  - 1; // (clk/(freq*2*prescale)) - 1
   TCCR2A = 0b01000010;    // "toggle OC2A on match, CTC mode"
   TCCR2B = 0b00000011;    // "CTC mode, clk/32 prescaling, GO!"
   #endif
   flash3times("CLO");
   for (osccal_candidate = OSCILLATOR_MIN; osccal_candidate <= OSCILLATOR_MAX; ++osccal_candidate) { // for each candidate +-10%
      digitalWrite(LED, LED_ON);
      delay(100);
      debug_print("clock calibration: OSCCAL %d\n", osccal_candidate); // might be visible, or not
      digitalWrite(LED, LED_OFF);
      unsigned long trial_start = millis();
      OSCCAL = osccal_candidate; // try this one
      // drop power here to reset the knob to clear garbage?
      while (millis() - trial_start < 1000) { // keep displaying this one for a second
         variable_to_knob (osccal_candidate, false); // might be visible, or not
         delay(100);
         if (knob_turned(DOWN)) { // knob down: set lower or upper bound
            knob_waitrelease();
            if (osccal_low == 0) {
               osccal_low = osccal_candidate; // record the lowest good value
               debug_print("low: %d\n", osccal_low); }
            else {
               osccal_high = osccal_candidate - 1; // record the highest good vaue
               debug_print("high: %d\n", osccal_high);
               osccal_candidate = 254; // and exit
            } }
         if (knob_turned(UP)) { // knob up: set the exact value
            knob_waitrelease();
            osccal_low = osccal_high = osccal_candidate; // record exact value
            osccal_candidate = 254; // and exit
         } } }
   #if !DEBUG_MOSI
   TCCR2B = 0; // turn off timer1
   #endif
   if (osccal_high != 0) { // got both values of the range
      osccal_candidate = ((uint16_t)osccal_low + (uint16_t)osccal_high) >> 1; // average
      OSCCAL = osccal_candidate; // start using it now, so output is seen
      debug_print("picked %d\n", osccal_candidate);
      snprintf(knob_display, sizeof(knob_display), "%3d", osccal_candidate);
      flash3times(knob_display);
      config.oscillator_calibration = osccal_candidate; // write it to EEPROM
      write_config(); }
   else { // failed
      OSCCAL = config.oscillator_calibration; // restore old value
      debug_print("\nbad range: %d to %d\n", osccal_low, osccal_high); } }
//
// configuration menu routines
//
struct menu_t {
   const char name[4];
   void (*menu_rtn)(byte);
   byte arg; };
bool config_changed;

byte menu_choose(struct menu_t *menu_ptr) {
   byte item_num = 0;
   do { // try each menu item in turn
      // return the item number 1..n chosen and executed, or zero if none
      set_knob_display((byte *)menu_ptr->name);
      unsigned long start_time = millis();
      ++item_num;
      while (millis() - start_time < MENU_DSP_TIME) {
         if (knob_turned(UP)) { // this menu item was chosen with "UP"
            knob_waitrelease();
            (*menu_ptr->menu_rtn)(menu_ptr->arg); // execute the menu function
            return item_num; }
         if (knob_turned(DOWN)) { // this menu item was rejected with "DN"
            knob_waitrelease();
            break; } }
      ++menu_ptr; } // go on to next menu item
   while (menu_ptr->menu_rtn);
   return 0; }

// allow a few seconds for a number to be changed using the up/down knob
int getnum(int value, int lowbound, int upbound) {
   unsigned long last_updown_time = millis();
   bool yesno = lowbound == 0 && upbound == 1;
   variable_to_knob(value, yesno);
   do {
      if (process_knob_updn(&value, lowbound, upbound)) {
         variable_to_knob(value, yesno);
         last_updown_time = millis(); } }
   while (millis() - last_updown_time < NUM_DONE_TIME);
   return value; }

struct menu_t config_menu[] = { // top level menus
   {"CAL", menu_calcmode, 0 },
   {"dAt", menu_dataexport, 0 },
   {"dEL", menu_tempdelta, 0 },
   {"CyC", menu_cycleperiod, 0 },
   {"oPt", menu_set_options, 0 },
   {"InF", menu_info, 0 },
   {"rE5", menu_reset, 0 },
   {"rEb", menu_reboot, 0 },
   {"", NULL } };

struct menu_t calc_menu[] = { // calculation mode menu
   {"pId", set_calc_mode, CALC_PID },
   {"H1 ", set_calc_mode, CALC_H1 },
   {"H2 ", set_calc_mode, CALC_H2 },
   {"non", set_calc_mode, CALC_NONE },
   {"dut", set_calc_mode, CALC_DUTYCYCLE },
   {"", NULL } };

struct menu_t pid_coefficient_menu[] {
   {" p ", set_pid_coefficient, 0 },
   {" I ", set_pid_coefficient, 1 },
   {" d ", set_pid_coefficient, 2 },
   {"", NULL } };

struct menu_t options_menu[] = { // options menu
   {"dut", set_option_show_duty_cycle, 0 },
   {"", NULL } };

void menu_set_options (byte arg) {
   menu_choose(options_menu); }

void set_option_show_duty_cycle (byte arg) {
   int oldval = config.options & OPT_SHOW_DUTY_CYCLE ? 1 : 0;
   int newval = getnum(oldval, 0, 1);
   if (newval != oldval) {
      if (newval) config.options |= OPT_SHOW_DUTY_CYCLE;
      else config.options &= ~OPT_SHOW_DUTY_CYCLE;
      config_changed = true; } }

void menu_calcmode(byte arg) {
   menu_choose(calc_menu); }

void menu_pid_coefficients(void) { // present all PID coefficients for change
   byte item = 0;
   do { // if a coefficient is chosen, do it, then try the next}
      byte item_incr = menu_choose(&pid_coefficient_menu[item]);
      if (item_incr == 0) break;
      item += item_incr; }
   while (pid_coefficient_menu[item].menu_rtn) ; }

void set_pid_coefficient (byte arg) { // adjust a PID coefficient
   double *coef;
   if (arg == 0) coef = &config.Kp;
   if (arg == 1) coef = &config.Ki;
   if (arg == 2) coef = &config.Kd;
   // implied decimal point between 2nd and 3rd digits: dd.d
   double newval = (double)getnum((int)(*coef * 10.), 0, 999) / 10.;
   if (newval != *coef) {
      *coef = newval;
      config_changed = true; } }

void set_calc_mode (byte mode) {
   if (config.calc_mode != mode) {
      config.calc_mode = (calc_mode_t) mode;
      config_changed = true; }
   if (mode == CALC_DUTYCYCLE || mode == CALC_H1) {
      int new_duty_cycle = getnum(config.duty_cycle, 0, 100);
      if (config.duty_cycle != new_duty_cycle) {
         config.duty_cycle = new_duty_cycle;
         config_changed = true; } }
   if (mode == CALC_PID) menu_pid_coefficients(); }

void menu_reboot(byte arg) {
   asm volatile ("  jmp 0"); }

void menu_reset(byte arg) {
   reset_config(config.oscillator_calibration);
   config_changed = true; }

void menu_info(byte arg) {
   set_knob_display((byte *)"rEL");
   delay(DISPLAY_INFO_TIME);
   set_knob_display((byte *)VERSION);
   delay(DISPLAY_INFO_TIME);
   set_knob_display((byte *)"   "); }

void menu_cycleperiod(byte arg) {
   unsigned long new_cycle_period = 1000UL * getnum((int) (config.cycle_period_msec / 1000),
                                    (int)(MIN_CYCLE_PERIOD / 1000), (int)(MAX_CYCLE_PERIOD / 1000));
   if (config.cycle_period_msec != new_cycle_period) {
      config.cycle_period_msec = new_cycle_period;
      adjust_duty_cycle_minmax();
      config_changed = true; } }

void menu_tempdelta(byte arg) {
   unsigned long new_delta = 1000UL * getnum((int) (config.temp_history_delta_msec / 1000UL), 1, 1000);
   if (config.temp_history_delta_msec != new_delta) {
      config.temp_history_delta_msec = new_delta;
      config_changed = true; } }

void menu_dataexport(byte arg) {
   temp_history_export(); };

void do_config(void) {
   debug_print("CONFIG\n", 0);
   flash3times("Con");
   config_changed = false;
   menu_choose(config_menu);
   if (config_changed) {
      write_config();
      set_knob_display((byte *)"yAy");
      delay(DISPLAY_INFO_TIME); }
   set_knob_display((byte *)"   "); }

byte fix_duty_cycle(byte duty_cycle) { // make sure a duty cycle conforms to min/max constaints
   if (duty_cycle > 0 && duty_cycle < min_duty_cycle)
      duty_cycle = min_duty_cycle;
   else if (duty_cycle < 100 && duty_cycle > max_duty_cycle)
      duty_cycle = max_duty_cycle;
   return duty_cycle; }

void adjust_duty_cycle_minmax(void) { // whenever cycle_period changes, have to change duty cycle min or max
   if (config.cycle_period_msec < MIN_CYCLE_PERIOD)
      config.cycle_period_msec = MIN_CYCLE_PERIOD;
   min_duty_cycle = (OVEN_MIN_ON_TIME * 100) / config.cycle_period_msec; // could be 0 too, of course
   max_duty_cycle = 100 - (OVEN_MIN_OFF_TIME * 100) / config.cycle_period_msec; // could be 100 too, of course
}
//
//   PID control algorithm using Brett Beauregard's PID library,
//   modified so compute() assumes it is called when a calculation is to be done
//
#define PID_LIMIT_MIN 0.0f     // output low limit
#define PID_LIMIT_MAX 100.0f   // output high limit
#define PID_RUN_RANGE_F 15     // only run PID if we're within these degrees of the setpoint

double pid_setpoint, pid_input, pid_output;

PID myPID(&pid_input, &pid_output, &pid_setpoint, DEFAULT_PID_P_WEIGHT, DEFAULT_PID_I_WEIGHT, DEFAULT_PID_D_WEIGHT, DIRECT);
bool PID_running = false;
int last_err;

void PID_start(void) {
   if (!PID_running) {
      myPID.SetMode(AUTOMATIC);
      PID_running = true; } }

void PID_stop(void) {
   if (PID_running) {
      myPID.SetMode(MANUAL);
      PID_running = false; } }

void do_PID(bool initalg) {   // call this every cycle_period milliseconds
   if (initalg) {
      myPID.SetTunings(config.Kp, config.Ki, config.Kd);
      myPID.SetSampleTime(config.cycle_period_msec);
      myPID.SetOutputLimits(PID_LIMIT_MIN, PID_LIMIT_MAX);
      PID_stop();
      last_err = temp_setpoint - temp_current;
      oven_do_shutdown = last_err > OVEN_SHUTDOWN_TRIGGER_F; // if way below setpoint, do the oven shutdown thing
      show_oven_shutdown("init"); }
   else { // do calculation
      int err = temp_setpoint - temp_current;
      show_oven_shutdown("before");
      if (err < -PID_RUN_RANGE_F) { // above the range where PID works
         PID_stop();
         oven_do_shutdown = false;
         current_duty_cycle = 0; }
      else if (err > PID_RUN_RANGE_F) { // below the range where PID works
         PID_stop();
         if (err > last_err) oven_do_shutdown = false; // if temp is decreasing, cancel shutdown mode
         current_duty_cycle = 100; }
      else { // finally, do the real PID algorithm
         oven_do_shutdown = false;
         PID_start();
         pid_setpoint = (double) temp_setpoint;
         pid_input = (double) temp_current;
         myPID.Compute();
         current_duty_cycle = (byte) pid_output;
         debug_print("PID err=%d dutycycle=%d%% P=%.3f I=%.3f D=%.3f\n",
                     err, current_duty_cycle,
                     myPID.Get_p(), myPID.Get_i(), myPID.Get_d()); }
      show_oven_shutdown("after");
      last_err = err; } }

//
// Heuristic #1 control algorithm
//
#define H1_PHASE1 -30
#define H1_PHASE2 -5
#define H1_PHASE3 5
#define H1_PHASE4 30
byte stable_duty_cycle;
int adjustment_countdown;
#define ADJUST_TIME 5 // in the central sections, adjust stable_duty_cycle every this many cycles
#define ADJUST_AMT 2 // adjust by this percent amount 
unsigned long neg_errs, pos_errs;
int errcnt_countdown;
#define ERRCNT_TIME 5 // reset error counters every this many times in central area

void do_H1(bool initalg) {
   if (initalg) {
      stable_duty_cycle = config.duty_cycle;
      adjustment_countdown = ADJUST_TIME;
      errcnt_countdown = ERRCNT_TIME;
      neg_errs = pos_errs = 0; }
   else {
      long int err = temp_current - temp_setpoint;
      if (err < H1_PHASE1)  current_duty_cycle = 100;
      else if (err < H1_PHASE2) // linear interpolation from max duty cycle to the stable zone
         //current_duty_cycle = 100 - (err - H1_PHASE1) * (100 - stable_duty_cycle) / (H1_PHASE2 - H1_PHASE1);
         current_duty_cycle = INTERPOLATE(err, H1_PHASE1, H1_PHASE2, max_duty_cycle, stable_duty_cycle);
      else if (err < H1_PHASE3)
         current_duty_cycle = stable_duty_cycle;
      else if (err < H1_PHASE4) // linear interpolation from the stable zone to the min duty cycle
         //current_duty_cycle = stable_duty_cycle - (err - H1_PHASE3) * stable_duty_cycle / (H1_PHASE4 - H1_PHASE3);
         current_duty_cycle = INTERPOLATE(err, H1_PHASE3, H1_PHASE4, stable_duty_cycle, min_duty_cycle);
      else current_duty_cycle = 0;
      // keep track of how often we're high vs low
      if (err < 0) ++neg_errs;
      if (err > 0) ++pos_errs;
      // if we're in the central area, gradually adjust the stable duty cycle
      if (err > H1_PHASE1 && err < H1_PHASE4 && --adjustment_countdown <= 0) {
         if (neg_errs > pos_errs) stable_duty_cycle += ADJUST_AMT;
         if (neg_errs < pos_errs) stable_duty_cycle -= ADJUST_AMT;
         debug_print("adjust stable duty cycle to %d\n", stable_duty_cycle);
         if (--errcnt_countdown <= 0) { // every so often, reset error counts
            debug_print("reset error counts\n", 0);
            neg_errs = pos_errs = 0;
            errcnt_countdown = ERRCNT_TIME; }
         adjustment_countdown = ADJUST_TIME; } } }
//
// Heuristic #2: anticipated target
//
#define H2_CLOSE_DEGREES_LOW 15
#define H2_CLOSE_DEGREES_HIGH 5
#define H2_HYSTERESIS 1
#define H2_UP_DUTY_CYCLE 40
#define H2_DOWN_DUTY_CYCLE 15

//**** start: identical to simulator code
void do_H2(bool init) {
   if (init)
      last_err = 0;
   else {
      int err = temp_current - temp_setpoint;  // the current signed error
      if (err < -H2_CLOSE_DEGREES_LOW) current_duty_cycle = 100; // much too cool
      else if (err > H2_CLOSE_DEGREES_HIGH) current_duty_cycle = 0;  // much too hot
      else if (err < 0) {  // below but close to target
         if (last_err < err) // if we're headed up
            current_duty_cycle = H2_DOWN_DUTY_CYCLE; // turn down the oven
         if (last_err > err && err < -H2_HYSTERESIS) //if we're headed down and far enough below
            current_duty_cycle = H2_UP_DUTY_CYCLE; // turn up the oven
      }
      else if (err >= 0) { // above but close to target
         if (last_err < err && err > H2_HYSTERESIS) // if we're headed down and far enough above
            current_duty_cycle = H2_DOWN_DUTY_CYCLE; // turn downn the oven
         if (last_err > err) // if we're headed up
            current_duty_cycle = 0; // turn off the oven
      }
      last_err = err; } }
//**** end: identical to simulator code

//
// Constant duty-cycle mode
// We use this to discover stable duty cycles at various temperatures
//
void do_dutycycle(bool initalg) {
   if (initalg)
      current_duty_cycle = fix_duty_cycle(config.duty_cycle); // use the configuration duty cycle
   // else do nothing: just keep cycling on and off at the specified duty cycle
}
//
// execute one of the algorithms
//
void do_algorithm (bool initalg) {
   temp_current = read_temp(); // read the real temperature now
   if (config.calc_mode == CALC_PID) do_PID(initalg);
   else if (config.calc_mode == CALC_H1) do_H1(initalg);
   else if (config.calc_mode == CALC_H2) do_H2(initalg);
   else if (config.calc_mode == CALC_DUTYCYCLE) do_dutycycle(initalg);
   // else: CALC_NONE, in which case we don't control heating
   if (!initalg) {
      current_duty_cycle = fix_duty_cycle(current_duty_cycle); // make sure it obeys constraints
      debug_print("did calc: set %dF temp %dF duty cycle %d%%, oven dsp \"%s\", log %d%%\n",
                  temp_setpoint, temp_current, current_duty_cycle, oven_display, temp_history_fullness); } }
//
// initialization
//
void setup(void) {
   static const byte input_pins[] PROGMEM = {
      RxD, INPUT_PULLUP, // pullup because there's a diode in series blocking high voltage
      MODE_S8, INPUT, MODE_S4, INPUT,  MODE_S2, INPUT, MODE_S1, INPUT,
      KNOB_UP, INPUT, KNOB_DN, INPUT,
      RTD_SDO, INPUT_PULLUP,
      #if !ANALOG_TEMP_IN
      DIGITAL_POTS_CHANGING, INPUT_PULLUP, // for testing with the oven mockup board with digital pots
      #endif
      0xff };
   static const byte output_pins[] PROGMEM = {
      TxD, LOW,  // start with knob data down to reset the knob
      LED, LED_OFF,
      RTD_CS, HIGH, RTD_SDI, HIGH, RTD_CLK, HIGH, // clk normally high: CPOL=1
      UPDOWN_TO_OVEN, HIGH, RTD_TO_US, LOW,
      OVEN_LOW_TEMP, LOW, OVEN_HIGH_TEMP, LOW,
      //   #if DEBUG_MOSI || EXPORT_MOSI
      MOSI, HIGH,
      //   #endif
      0xff };

   for (int ndx = 0;;) { // configure input pins
      byte pin = pgm_read_byte_near(input_pins + ndx++);
      if (pin == 0xff) break;
      pinMode(pin, pgm_read_byte_near(input_pins + ndx++)); }

   for (int ndx = 0;;) { // configure output pins
      byte pin = pgm_read_byte_near(output_pins + ndx++);
      if (pin == 0xff) break;
      pinMode(pin, OUTPUT);
      digitalWrite(pin, pgm_read_byte_near(output_pins + ndx++)); }

   read_config();  // read (or initialize) non-volatile config memory
   byte old_osccal = OSCCAL;
   OSCCAL = config.oscillator_calibration;  // set the processor speed as indicated
   uint16_t reboot_count = increment_reboot_counter();

   #if DEBUG
   debug_port.begin(DEBUG_BAUDRATE);
   debug_print("*** controller started, boot %d\n", reboot_count);
   debug_print("V" VERSION "; oscillator %d changed to %d\n", old_osccal, OSCCAL);
   show_config("read");
   #endif

   delay(2000); // wait for knob to lose power and reset itself
   Serial.begin(KNOB_BAUDRATE, SERIAL_8N2); // RxD,TxD: for display data received from the oven and sent to the knob
   delay(REAL_BOARD ? 1000 : 4000);
   flash3times ("5n5");
   variable_to_knob (reboot_count, false);  // flash the number of reboots
   delay(1000); // leave it showing for a bit
   #if REAL_BOARD && !ANALOG_TEMP_IN
   RTD_init();
   #endif
   digitalWrite(RTD_TO_US, HIGH);  // take over the RTD
   temp_current = read_temp();  // initialize current temp
   debug_print("initial temp is %dF\n", temp_current);
   digitalWrite(RTD_TO_US, LOW);  // return the RTD
   ovenmode = M_14; // start with an impossible oven mode
   debug_print("setup done\n", 0);
   purge_oven_display();  // empty and resync incoming knob data

   #if 0 //TEST CODE
   while (0) { //test reading "knob down"
      static int value = 0;
      int newval = digitalRead(KNOB_DN);
      if (newval != value) debug_print("val=%d, ovenmode=%d\n", newval, getovenmode());
      value = newval; }
   bool ovenlow = true;
   while (0) { // test RTD switching
      ovenlow = !ovenlow;
      debug_print("RTD to us, oven %s\n", ovenlow ? "low" : "high");
      digitalWrite(RTD_TO_US, HIGH);
      digitalWrite(ovenlow ? OVEN_LOW_TEMP : OVEN_HIGH_TEMP, HIGH);
      delay(5000);
      digitalWrite(ovenlow ? OVEN_LOW_TEMP : OVEN_HIGH_TEMP, LOW);
      debug_print("RTD to oven\n", 0);
      digitalWrite(RTD_TO_US, LOW);
      delay(5000); }
   while (0) {   //for testing receiving analog temperature
      debug_print("temp %dF\n", read_temp()); // analog temp in
      delay(1000); }
   while (0) {  //test receiving digital pot temperature
      digitalWrite(RTD_TO_US, HIGH);  // take over the RTD
      int temp = read_temp(); // read the RTD
      debug_print("temp is %dF\n", temp);
      delay(2000); }
   #endif
}
//
// main loop
//
void loop(void) {

   if (millis() - last_knob_display_time >= KNOB_DISPLAY_REPEAT_TIME)
      output_to_knob(); // periodic resend of what's being displayed on the knob

   // process mode selector switch changes

   static bool mode_change_pending = false;
   static unsigned long mode_change_started_msec = 0;
   static byte mode_change_newmode = M_14;
   byte mode_current = mode_change_pending ? mode_change_newmode : ovenmode; // where we think the switch is
   if (mode_current != getovenmode()) { // if mode switch is changing
      delay(DEBOUNCE);      // wait for it to settle; there are noise glitches
      if (mode_current != getovenmode()) { // if it really is changing
         mode_change_newmode = getovenmode(); // remember the new mode
         mode_change_started_msec = millis(); // but wait for it to persist for a while before acting on it
         mode_change_pending = true; } }

   if (mode_change_pending && millis() - mode_change_started_msec > MODE_PERSIST_TIME) {
      mode_change_pending = false; // a mode change has persisted for long enough, so act on it
      if (ovenmode != mode_change_newmode) { // but only if it's really a change
         ovenmode = mode_change_newmode;
         last_display_change_time = 0;
         debug_print("ovenmode: %s\n", ovenmodenames[ovenmode]);
         if (ovenmode == M_OFF) config_state = CS_IDLE;
         bool were_controlling_temp = controlling_temp;
         controlling_temp = (config.calc_mode != CALC_NONE) &&
                            (ovenmode == M_BAKE || ovenmode == M_CONV_BAKE || ovenmode == M_ROAST || ovenmode == M_CONV_ROAST);
         if (controlling_temp && !were_controlling_temp) { // starting to control temp
            temp_setpoint =  ovenmode == M_BAKE ? 350 : 375; // its starting setpoint
            digitalWrite(RTD_TO_US, HIGH); // take over the RTD
            digitalWrite(UPDOWN_TO_OVEN, LOW); // don't let the oven see up/down switch movement
            temp_history_clear();
            turn_oven_off(); // start with the oven off
            oven_do_shutdown = oven_doing_shutdown = false;
            do_algorithm(true); // initialize algorithm
            debug_print("starting \"%s\"\n", calc_mode_names[config.calc_mode]);
            last_cycle_time = millis() - config.cycle_period_msec; // set up to start a cycle now
         }
         if (!controlling_temp && were_controlling_temp) { // stopping to control temp
            turn_oven_off(); // turn oven off
            digitalWrite(UPDOWN_TO_OVEN, HIGH); // let the oven see up/down switch movements
            digitalWrite(RTD_TO_US, LOW); // take RTD away from us and back to the oven
            digitalWrite(OVEN_LOW_TEMP, LOW); // remove fake temp to oven
            digitalWrite(OVEN_HIGH_TEMP, LOW); } } }

   // process up/down knob toggle switch changes

   if (ovenmode == M_OFF) // in off mode
      process_knob_updn(NULL, 0, 0); // just look for entering config mode

   if (controlling_temp) { // we are controlling the oven heater
      read_oven_display();  // for debugging and to keep the buffer empty, grab what the oven wants to display
      // now maybe adjust our temperature setpoint
      if (process_knob_updn(&temp_setpoint, MIN_TEMP_SETPOINT, MAX_TEMP_SETPOINT))
         debug_print("setpoint chg to %d\n", temp_setpoint); }
   else { // the oven is in charge
      if (read_oven_display()) { // if the oven is sending characters to the knob
         set_knob_display ((byte *)oven_display); // just display them
         //debug_print("got %s sent %s\n", oven_display, knob_display);
      } }

   if (controlling_temp) {
      if (millis() - last_cycle_time >= config.cycle_period_msec) { // time for an algorithm cycle
         do_algorithm(false);
         if (current_duty_cycle > 0 && !oven_doing_shutdown) turn_oven_on();
         last_cycle_time = millis(); }

      if (current_duty_cycle < 100 && oven_on // time to turn off mid-cycle?
            && millis() - last_cycle_time >= (config.cycle_period_msec * current_duty_cycle) / 100)
         turn_oven_off();

      if (oven_do_shutdown  // if we should start a temporary shutdown
            && !oven_doing_shutdown // and we're not doing it yet
            && temp_setpoint - temp_current < ((100 - OVEN_SHUTDOWN_PCT) * (long int)temp_setpoint) / 100) { // and we've passed the "do shutdown" point
         temporary_oven_shutdown(); // this changes the oven from fast-heat to slow-heat mode
         oven_do_shutdown = false; }

      if (oven_doing_shutdown // if we're doing a temporary shutdown
            && millis() - oven_off_time >= OVEN_SHUTODWN_TIME_MSEC) { // and it's time to end it
         oven_doing_shutdown = false;
         if (current_duty_cycle > 0) turn_oven_on(); }

      if (!knob_dn && !knob_up // knob not moving: cycle through our displays for the knob
            && millis() - last_display_change_time >=
            (display_holdoff ? DISPLAY_HOLDOFF_TIME : DISPLAY_CHANGE_TIME)) {
         char msg[4];
         display_holdoff = false;
         if (next_display == DSP_TEMP_NOW)
            snprintf(msg, sizeof(msg), "%3d", temp_current = read_temp());
         else if (next_display == DSP_TEMP_SET)
            snprintf(msg, sizeof(msg), "%3d", temp_setpoint);
         else if (next_display == DSP_DUTY_CYCLE)
            snprintf(msg, sizeof(msg), "d%2d", current_duty_cycle >= 100 ? 99 : current_duty_cycle);
         else sprintf(msg, "---"); // internal error
         set_knob_display((byte *) msg);
         do if (++next_display >= DSP_END) next_display = DSP_START + 1; // setup for the next display item
         while (next_display == DSP_DUTY_CYCLE && !(config.options & OPT_SHOW_DUTY_CYCLE));
         last_display_change_time = millis(); } //

      if (controlling_temp && millis() - temp_history_last_time >= config.temp_history_delta_msec) { // accmulate temperature history
         temp_history_add(temp_current = read_temp()); // make an entry with the current temperature
         temp_history_last_time = millis(); }

   } // controlling_temp

}
//*
