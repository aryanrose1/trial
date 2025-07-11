#include <Wire.h> // I2C comms library
#include <LiquidCrystal_I2C.h> // LCD over I2C
#include <assert.h> // For runtime asserts
#include <EEPROM.h> // Non-volatile storage
#include <limits.h> // Data type limits

#define SPARGE_DELAY 3 // delay to prevent sparging from valve cycling
#define TANK_2_LOW 10 // time delay for valve to stop draining tank 2
#define MAX_STATE 100000 // time until a tank assumes level control is lost

#define PRESSURE_PIN A9 // analog pin for pressure sensor
#define FLOW_PIN A1 // analog pin for flow sensor
#define TEMP_PIN A2 // analog pin for temperature sensor
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;     // whether the string is complete


void printHex(int num, int precision) // print number as fixed-width hex
{
  char tmp[16]; // temporary buffer
  char format[128]; // printf format buffer
  sprintf(format, "%%.%dX", precision); // build format like "%.4X"
  sprintf(tmp, format, num); // format number
  Serial.print(tmp); // send to serial
}

void EEPROMWritelong(int address, long value) // write 32-bit value to EEPROM
{
	byte four = (value & 0xFF); // least-significant byte
	byte three = ((value >> 8) & 0xFF);
	byte two = ((value >> 16) & 0xFF);
	byte one = ((value >> 24) & 0xFF); // most-significant byte

	EEPROM.write(address, four); // store bytes
	EEPROM.write(address + 1, three);
	EEPROM.write(address + 2, two);
	EEPROM.write(address + 3, one);
}

class Timer // simple second-resolution timer
{
   public:
  unsigned long started_at, set_point; // start time and duration
  bool set; // is timer active

  void start(int len) // start timer for len seconds
  {
    set = true; // mark active
    started_at = millis()/1000; // record start in seconds
    set_point = len; // record duration
  }
  bool is_set() // check active
  {
    return set;
  }
  void reset() // deactivate
  {
    set = false;
  }
  bool elapsed() // has duration passed?
  {
    return millis()/1000 - started_at >= set_point;
  }
};

long EEPROMReadlong(long address) // read 32-bit from EEPROM
{
	long four = EEPROM.read(address); // LSB
	long three = EEPROM.read(address + 1);
	long two = EEPROM.read(address + 2);
	long one = EEPROM.read(address + 3); // MSB

	return ((four << 0) & 0xFF)
	     + ((three << 8) & 0xFFFF)
	     + ((two << 16) & 0xFFFFFF)
	     + ((one << 24) & 0xFFFFFFFF); // recombine
}

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4); // 20×4 LCD at 0x27

enum POSITION // valve positions and on/off
{
  CCW = 0, // counter-clockwise (ON)
  CW = 1,  // clockwise (OFF)
  ON = 1,  // generic ON
  OFF = 0  // generic OFF
}; 

class Bistable { // two-state timer controller

bool is_state_1; // current state flag
long state_start; // millis timestamp when state began
int state_1_addr; // EEPROM addr for state-1 duration
int state_2_addr; // EEPROM addr for state-2 duration
public:
unsigned long state_1_len; // seconds in state 1
unsigned long state_2_len; // seconds in state 2
Bistable(long state_1_len, long state_2_len,bool is_EEPROM=false,int state_1_addr=0, int state_2_addr=0)
{
  this-> state_1_len = state_1_len; // init durations
  this-> state_2_len = state_2_len;
  is_state_1 = true; // start in state 1
  state_start = millis(); // record now
  if (is_EEPROM) // load saved durations?
  {
    this-> state_1_len = EEPROMReadlong(state_1_addr); // from EEPROM
    this-> state_2_len = EEPROMReadlong(state_2_addr);
    this-> state_1_addr = state_1_addr; // store addresses
    this-> state_2_addr = state_2_addr;
  }
}
void toggle() // flip immediately
{
  is_state_1 = !is_state_1; // toggle state
  state_start = millis(); // reset timer
}
void set_length(bool setting_1,long length) // update duration
{
  if (setting_1)
  {
    state_1_len = length; // change state1 duration
    EEPROMWritelong(state_1_addr,length); // save
  }
  else 
  {
    state_2_len = length; // change state2 duration
    EEPROMWritelong(state_2_addr,length); // save
  }
}

bool update() // call each loop; returns true for state1
{
  unsigned long elapsed_ms = millis() - state_start; // ms since change
  if (is_state_1)
  {
     if (elapsed_ms > state_1_len * 1000 || elapsed_ms < 0) // expired or rolled
     {
       is_state_1 = false; // switch state
       state_start = millis(); // reset timer
     }
    return true; // in state1
  }
  else
  {
     if (elapsed_ms > state_2_len * 1000 || elapsed_ms < 0) // expired or rolled
     {
       is_state_1 = true; // switch back
       state_start = millis();
     }
    return false; // in state2
  }
}
};

class ThreeWayValve { // drives two coils for 3-way valve
POSITION position; // current pos
int ccw_pin, cw_pin; // digital pins
int id; // valve ID
Timer *timer; // optional delay timer
public:
set(POSITION pos) // fire coils to pos
{
  position = pos; // record pos
  digitalWrite(ccw_pin, pos); // energize CCW coil
 delay(1); // slight pause
  digitalWrite(cw_pin, !pos); // energize other
}
ThreeWayValve(int id, POSITION pos, int ccw_pin, int cw_pin, Timer * timer = nullptr)
{
  this->timer = timer; // attach timer if any
  assert(ccw_pin != cw_pin); // ensure pins differ
  this->id = id;  
  this->ccw_pin = ccw_pin; // store pins
  this->cw_pin = cw_pin;
  pinMode(ccw_pin,OUTPUT); // set pin modes
  pinMode(cw_pin,OUTPUT);
}

~ThreeWayValve() // cleanup on deletion
{
  delete timer; // free timer memory
}

bool get_pos() // return current valve position
{
  return position; // true=CW/ON, false=CCW/OFF
}

printLCD(int col, int row ) // display ID and position on LCD
{
  lcd.setCursor(col, row); // move cursor
  lcd.print(id);           // show valve ID
  lcd.print(":");          // separator
  lcd.print(position ? "CW " : "CCW"); // show text
}

void update(POSITION new_pos,int sec_delay) // schedule a position change
{
  if (!timer) return; // if no timer, do nothing
  if (!timer->is_set() && new_pos != position) // start delay if needed
    timer->start(sec_delay);
  else if (timer->is_set() && new_pos == position) // restart if cancelled
    timer->start(sec_delay);
  else if (new_pos != position && timer->elapsed()) // after delay, commit
  {
    set(new_pos);       // change valve
    timer->reset();     // clear timer
  }
}
}; // end ThreeWayValve class

class Valve { // simple on/off valve
  POSITION position; // ON/OFF state
  int pin;           // control pin
  int id;            // valve identifier
public:
  set(POSITION pos) // apply new state
  {
    position = pos;      // store state
    digitalWrite(pin, pos); // write digital output
  }
  Valve(int id, POSITION pos, int pin) // constructor
  {
    this->id = id;           // store ID
    this->pin = pin;         // store pin
    pinMode(pin, OUTPUT);    // set as output
  }
  bool get_pos() // return current state
  {
    return position;
  }
  printLCD(int col, int row ) // display on LCD
  {
    lcd.setCursor(col, row);
    lcd.print(id);
    lcd.print(":");
    lcd.print(position ? "ON " : "OFF");
  }
}; // end Valve class

class Barrel { // float-sensor tank monitor
  int pin;          // sensor pin
  int id;           // barrel ID
  int min_low, min_high; // unused placeholders
  bool filled;      // last known state
  bool active_low;  // sensor logic invert
  bool fail;        // stuck flag
  Timer * timer;    // fail-detection timer
public:
  Barrel(int id,int pin,bool active_low = false,Timer * timer = nullptr)
  {
    fail = false;            // initial no-fail
    this->timer = timer;     // store timer
    timer->start(MAX_STATE); // start countdown
    this->id = id;           // store ID
    this->pin = pin;         // store pin
    this->min_low = min_low; // unused
    this->min_high = min_high; // unused
    this->active_low = active_low; // store logic
    this->filled = is_full(); // initial read
    pinMode(pin, INPUT_PULLUP); // enable pullup
  }

  bool is_full() // read float sensor
  {
    bool tank_full = active_low ? !digitalRead(pin) : digitalRead(pin);
    if (tank_full != filled) // on change
    {
      fail = false;        // reset fail
      timer->start(MAX_STATE); // restart timer
      filled = tank_full;  // update state
    }
    if (timer->elapsed()) // if stuck
    {
      fail = true;         // mark fail
      return true;         // treat as full
    }
    return tank_full;      // report normal
  }

  void printLCD(int col, int row) // display tank status
  {
    lcd.setCursor(col, row);
    lcd.print("B");
    lcd.print(id);
    lcd.print(":");
    if (fail) lcd.print("FAIL");
    else if (is_full()) lcd.print("FULL");
    else lcd.print("VOID");
  }
}; // end Barrel class

void serialStatus(bool tw1, bool tw2, bool tw3, bool tw4, bool tw5,
                  bool v6, bool v7,
                  bool b1, bool b2, bool b3,
                  long v4_1, long v4_2,
                  long v5_1, long v5_2,
                  long v6_1, long v6_2,
                  long v7_1,long v7_2)
{
  Serial.print("#"); // start marker
  Serial.print(tw1); Serial.print(tw2); Serial.print(tw3); // valves 1–3
  Serial.print(b1);  Serial.print(b2);  Serial.print(b3);  // barrels 1–3
  Serial.print(tw4); Serial.print(tw5); // valves 4–5
  Serial.print(v6);  Serial.print(v7);  // valves 6–7
  printHex(v4_1,4); printHex(v4_2,4); // controller 4 timings
  printHex(v5_1,4); printHex(v5_2,4); // controller 5 timings
  printHex(v6_1,4); printHex(v6_2,4); // controller 6 timings
  printHex(v7_1,4); printHex(v7_2,4); // controller 7 timings
  printHex(analogRead(PRESSURE_PIN),4); // pressure ADC
  printHex(analogRead(FLOW_PIN),4);     // flow ADC
  printHex(analogRead(TEMP_PIN),4);     // temp ADC
  Serial.print("!\r\n"); // end marker
}

ThreeWayValve three_way_valve1 = ThreeWayValve(1,CCW,2,3,new Timer());
ThreeWayValve three_way_valve2 = ThreeWayValve(2,CCW,4,5,new Timer());
ThreeWayValve three_way_valve3 = ThreeWayValve(3,CCW,7,6,new Timer());
ThreeWayValve three_way_valve4 = ThreeWayValve(4,CCW,8,9);
ThreeWayValve three_way_valve5 = ThreeWayValve(5,CCW,11,12);
Valve valve6 = Valve(6,OFF,10);
Valve valve7 = Valve(7,OFF,13);

Barrel barrel1 = Barrel(1,24,false,new Timer());
Barrel barrel2 = Barrel(2,23,false,new Timer()); // to flip logic, change 3rd arg
Barrel barrel3 = Barrel(3,22,false,new Timer()); // all sensors are floats
Bistable valve4_controller = Bistable(5,5,true,0,4);
Bistable valve5_controller = Bistable(5,5,true,8,12);
Bistable valve6_controller = Bistable(5,5,true,16,20);
Bistable valve7_controller = Bistable(5,5,true,24,28);

void setup()
{
  Serial.begin(115200); // open serial port
  lcd.begin();          // initialize LCD
  inputString.reserve(200); // reserve serial buffer
  lcd.backlight();      // turn on LCD backlight
  //EEPROMWritelong(0,1);
  //EEPROMWritelong(4,2);
  //EEPROMWritelong(8,3);
  //EEPROMWritelong(12,4);
  //EEPROMWritelong(16,5);
  //EEPROMWritelong(20,6);
  //EEPROMWritelong(24,7);
  //EEPROMWritelong(28,8);
}

void loop()
{
  if (stringComplete) { // if a complete command arrived
    if (inputString[0]=='U') { // user command
      if (inputString[1]=='4' && inputString[2]=='3') valve4_controller.toggle();
      if (inputString[1]=='5' && inputString[2]=='3') valve5_controller.toggle();
      if (inputString[1]=='6' && inputString[2]=='3') valve6_controller.toggle();
      if (inputString[1]=='7' && inputString[2]=='3') valve7_controller.toggle();
      if (inputString[1]=='4' && inputString[2]=='1')
        valve4_controller.set_length(true, atol(inputString.substring(3).c_str()));
      if (inputString[1]=='4' && inputString[2]=='2')
        valve4_controller.set_length(false, atol(inputString.substring(3).c_str()));
      if (inputString[1]=='5' && inputString[2]=='1')
        valve5_controller.set_length(true, atol(inputString.substring(3).c_str()));
      if (inputString[1]=='5' && inputString[2]=='2')
        valve5_controller.set_length(false, atol(inputString.substring(3).c_str()));
      if (inputString[1]=='6' &&	inputString[2]=='1')
        valve6_controller.set_length(true, atol(inputString.substring(3).c_str()));
      if (inputString[1]=='6' && inputString[2]=='2')
        valve6_controller.set_length(false, atol(inputString.substring(3).c_str()));
      if (inputString[1]=='7' &&	inputString[2]=='1')
        valve7_controller.set_length(true, atol(inputString.substring(3).c_str()));
      if (inputString[1]=='7' && inputString[2]=='2')
        valve7_controller.set_length(false, atol(inputString.substring(3).c_str()));
    }
    inputString = "";       // clear buffer
    stringComplete = false; // reset flag
  }

  if (barrel1.is_full())           three_way_valve1.update(CCW, SPARGE_DELAY);
  else                             three_way_valve1.update(CW,  SPARGE_DELAY);
  if (barrel2.is_full())           three_way_valve2.update(CCW, SPARGE_DELAY);
  else                             three_way_valve2.update(CW,  TANK_2_LOW);
  if (barrel3.is_full())           three_way_valve3.update(CCW, SPARGE_DELAY);
  else                             three_way_valve3.update(CW,  SPARGE_DELAY);
  if (valve4_controller.update())  three_way_valve4.set(CW);
  else                             three_way_valve4.set(CCW);
  if (valve5_controller.update())  three_way_valve5.set(CW);
  else                             three_way_valve5.set(CCW);
  if (valve6_controller.update())  valve6.set(ON);
  else                             valve6.set(OFF);
  if (valve7_controller.update())  valve7.set(ON);
  else                             valve7.set(OFF);

  three_way_valve1.printLCD(0, 0);
  three_way_valve2.printLCD(0, 1);
  three_way_valve3.printLCD(0, 2);
  three_way_valve4.printLCD(0, 3);
  three_way_valve5.printLCD(6, 0);
  valve6.printLCD(6, 1);
  valve7.printLCD(6, 2);
  barrel1.printLCD(12,0);
  barrel2.printLCD(12,1);
  barrel3.printLCD(12,2);

  serialStatus(
    three_way_valve1.get_pos(), three_way_valve2.get_pos(), three_way_valve3.get_pos(),
    three_way_valve4.get_pos(), three_way_valve5.get_pos(),
    valve6.get_pos(), valve7.get_pos(),
    barrel1.is_full(), barrel2.is_full(), barrel3.is_full(),
    valve4_controller.state_1_len, valve4_controller.state_2_len,
    valve5_controller.state_1_len, valve5_controller.state_2_len,
    valve6_controller.state_1_len, valve6_controller.state_2_len,
    valve7_controller.state_1_len, valve7_controller.state_2_len
  );
}

void serialEvent() { // triggered by incoming serial data
  while (Serial.available()) {
    char inChar = (char)Serial.read(); // read byte
    inputString += inChar;             // append to buffer
    if (inChar == '\n') {              // end-of-line?
      stringComplete = true;           // mark ready
    }
  }
}
