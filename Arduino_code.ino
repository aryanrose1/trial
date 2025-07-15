#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <assert.h>
#include <EEPROM.h>
#include <limits.h>
#define SPARGE_DELAY 3 //delay to prevent sparging from valve cycling
#define TANK_2_LOW 10 //time delay for valve to stop draining tank 2
#define MAX_STATE 100000 //time until a tank assumes level control is lost

#define PRESSURE_PIN A9
#define FLOW_PIN A1
#define TEMP_PIN A2
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;



void printHex(int num, int precision)
{
  char tmp[16];
  char format[128];
  sprintf(format, "%%.%dX", precision);
  sprintf(tmp, format, num);
  Serial.print(tmp);
}

void EEPROMWritelong(int address, long value)
{
	//Decomposition from a long to 4 bytes by using bitshift.
	//One = Most significant -> Four = Least significant byte
	byte four = (value & 0xFF);
	byte three = ((value >> 8) & 0xFF);
	byte two = ((value >> 16) & 0xFF);
	byte one = ((value >> 24) & 0xFF);


	EEPROM.write(address, four);
	EEPROM.write(address + 1, three);
	EEPROM.write(address + 2, two);
	EEPROM.write(address + 3, one);
}

class Timer
{
   public:
  unsigned long started_at, set_point;
  bool set;
 

  void start(int len)
  {
    set = true;
    started_at = millis()/1000;
    set_point = len;
  }
  bool is_set()
  {
    return set;
  }
  void reset()
  {
    set = false;
  }
  bool elapsed()
  {
    return millis()/1000 - started_at >= set_point;
  }
};

long EEPROMReadlong(long address)
{
	//Read the 4 bytes from the eeprom memory.
	long four = EEPROM.read(address);
	long three = EEPROM.read(address + 1);
	long two = EEPROM.read(address + 2);
	long one = EEPROM.read(address + 3);

	//Return the recomposed long by using bitshift.
	return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);
enum POSITION
{
  CCW = 0, //ON 
  CW = 1, //OFF
  ON = 1,
  OFF = 0
}; 



class Bistable {

bool is_state_1;
long state_start;
int state_1_addr;
int state_2_addr;
public:
unsigned long state_1_len;
unsigned long state_2_len;
Bistable(long state_1_len, long state_2_len,bool is_EEPROM=false,int state_1_addr=0, int state_2_addr=0)
{
  this-> state_1_len = state_1_len;
  this-> state_2_len = state_2_len;
  is_state_1 = true;
  state_start = millis();
  if (is_EEPROM)
  {
    this-> state_1_len = EEPROMReadlong(state_1_addr);
  this-> state_2_len = EEPROMReadlong(state_2_addr);
  this-> state_1_addr = state_1_addr;
  this-> state_2_addr = state_2_addr;
  }
}
void toggle()
{
  if (is_state_1) is_state_1 = false;
  else is_state_1 = true;
  state_start = millis();
}
void set_length(bool setting_1,long length)
{
  if (setting_1)
  {
    state_1_len = length;
    EEPROMWritelong(state_1_addr,length);
  }
    else 
    {
      state_2_len = length;
      EEPROMWritelong(state_2_addr,length);
    }
    
}

bool update()
{
  if (is_state_1)
  {
     if ((millis()-state_start)>state_1_len*1000 ||millis()-state_start < 0) //negative check for millis roll over
     {
       is_state_1 = false;
       state_start = millis();
     }
    return true;
  }
  else
  {
     if ((millis()-state_start)>state_2_len*1000 ||millis()-state_start < 0) //negative check for millis roll over
     {
       is_state_1 = true;
       state_start = millis();
     }
    return false;
  }
}
};
class ThreeWayValve {
POSITION position;
int ccw_pin, cw_pin;
int id;
Timer *timer;
public:
set(POSITION pos)
{
  position = pos;
  digitalWrite(ccw_pin, pos);
 delay(1);
  digitalWrite(cw_pin, !pos);
} 
ThreeWayValve(int id, POSITION pos, int ccw_pin, int cw_pin, Timer * timer = nullptr)
{
  this->timer = timer;
assert(ccw_pin != cw_pin);
  this->id = id;
  

  this->ccw_pin = ccw_pin;
  this->cw_pin = cw_pin;


  pinMode(ccw_pin,OUTPUT);
  pinMode(cw_pin,OUTPUT);

  
 // set(pos);
  
}

~ThreeWayValve()
{
  delete timer;
}

bool get_pos()
{
  return position;
}
printLCD(int col, int row )
{
  lcd.setCursor(col, row);
  lcd.print(id);
   lcd.print(":");
  if (position) lcd.print("CW ");
  else lcd.print("CCW");
 
}

void update(POSITION new_pos,int sec_delay)
{
  if (timer)
  {
   
    if (!timer->is_set() && new_pos != position)
    {
      timer->start(sec_delay);
    }
    else if (timer-> is_set() && new_pos == position)
    {
      timer -> start(sec_delay);
    }
    else if (new_pos != position)
    {
      if (timer->elapsed())
      {
        set(new_pos);
        timer->reset();        
      }
    }
  }
}
};


class Valve {
POSITION position;
int pin;
int id;
public:
set(POSITION pos)
{
  position = pos;
  digitalWrite(pin, pos);
 
} 
Valve(int id, POSITION pos, int pin)
{
  

  this->id = id;
  

  this->pin = pin;
  

  pinMode(pin,OUTPUT);
 
  
}

bool get_pos()
{
  return position;
}
printLCD(int col, int row )
{
  lcd.setCursor(col, row);
  lcd.print(id);
   lcd.print(":");
  if (position) lcd.print("ON ");
  else lcd.print("OFF");
 
}


};

class Barrel {
int pin;
int id;
int min_low, min_high;
bool filled;
bool active_low;
bool fail;
Timer * timer;
public:
Barrel(int id,int pin,bool active_low = false,Timer * timer = nullptr)
{
  fail = false;
  this-> timer = timer;
  timer->start(MAX_STATE);
  this->id = id;
  this->pin = pin;
  this-> min_low = min_low;
  this-> min_high = min_high;
  this->active_low =  active_low;
  this->filled = is_full();
  pinMode(pin, INPUT_PULLUP);
}

bool is_full()
{
  bool tank_full;
  if (active_low) tank_full = !digitalRead(pin);
  else tank_full = digitalRead(pin);
  if (tank_full != filled)
  {
    fail = false;
    timer->start(MAX_STATE);
    filled = tank_full;
  }
  if (timer->elapsed())
  {
    fail = true;
    return true;
  }
  return tank_full;
}
void printLCD(int col, int row)
{
 lcd.setCursor(col, row);
  lcd.print("B");
  lcd.print(id);
   lcd.print(":");
  if(fail) lcd.print("FAIL");
  else if (is_full()) lcd.print("FULL");
  else lcd.print("VOID"); 
}
};
void serialStatus(bool tw1, bool tw2, bool tw3, bool tw4, bool tw5,
                  bool v6, bool v7, 
                  bool b1, bool b2, bool b3, 
                  long v4_1, long v4_2,
                  long v5_1, long v5_2,
                  long v6_1, long v6_2,
                  long v7_1,long v7_2)
{
  Serial.print("#");
  Serial.print(tw1);
  Serial.print(tw2);
  Serial.print(tw3);
  Serial.print(b1);
  Serial.print(b2);
  Serial.print(b3);
  Serial.print(tw4);
  Serial.print(tw5);
  Serial.print(v6);
  Serial.print(v7);
  printHex(v4_1,4);
  printHex(v4_2,4);
  printHex(v5_1,4);
  printHex(v5_2,4);
  printHex(v6_1,4);
  printHex(v6_2,4);
  printHex(v7_1,4);
  printHex(v7_2,4);
  printHex(analogRead(PRESSURE_PIN),4);
  printHex(analogRead(FLOW_PIN),4);
  printHex(analogRead(TEMP_PIN),4);
  Serial.print("!\r\n");
};

  ThreeWayValve three_way_valve1 = ThreeWayValve(1,CCW,2,3,new Timer());
  ThreeWayValve three_way_valve2 = ThreeWayValve(2,CCW,4,5,new Timer());
  ThreeWayValve three_way_valve3 = ThreeWayValve(3,CCW,7,6,new Timer());
  ThreeWayValve three_way_valve4 = ThreeWayValve(4,CCW,8,9);
  ThreeWayValve three_way_valve5 = ThreeWayValve(5,CCW,11,12);
  Valve valve6 = Valve(6,OFF,10);
  Valve valve7 = Valve(7,OFF,13);
 
  Barrel barrel1 = Barrel(1,24,false,new Timer());
  Barrel barrel2 = Barrel(2,23,false, new Timer()); //to flip level logic, change 3rd argument from true to false, currently for float sensor
  Barrel barrel3 = Barrel(3,22,false, new Timer()); //All three sensors are float sensors
  Bistable valve4_controller = Bistable(5,5,true,0,4);
  Bistable valve5_controller = Bistable(5,5,true,8,12);
  Bistable valve6_controller = Bistable(5,5,true,16,20);
  Bistable valve7_controller = Bistable(5,5,true,24,28);
  
  

void setup()
{
	// initialize the LCD
    Serial.begin(115200);
	lcd.begin();
 inputString.reserve(200);
lcd.backlight();
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

if (stringComplete) {
    // clear the string:
    //Serial.println(inputString[0]);
    if (inputString[0]=='U')
    {
      if (inputString[1] =='4' && inputString[2] == '3') valve4_controller.toggle();
      if (inputString[1] =='5' && inputString[2] == '3') valve5_controller.toggle();
      if (inputString[1] =='6' && inputString[2] == '3') valve6_controller.toggle();
      if (inputString[1] =='7' && inputString[2] == '3') valve7_controller.toggle();
      if (inputString[1] =='4' && inputString[2] == '1')valve4_controller.set_length(true,atol(inputString.substring(3).c_str()));
      if (inputString[1] =='4' && inputString[2] == '2')valve4_controller.set_length(false,atol(inputString.substring(3).c_str()));
      if (inputString[1] =='5' && inputString[2] == '1')valve5_controller.set_length(true,atol(inputString.substring(3).c_str()));
      if (inputString[1] =='5' && inputString[2] == '2')valve5_controller.set_length(false,atol(inputString.substring(3).c_str()));
      if (inputString[1] =='6' && inputString[2] == '1')valve6_controller.set_length(true,atol(inputString.substring(3).c_str()));
      if (inputString[1] =='6' && inputString[2] == '2')valve6_controller.set_length(false,atol(inputString.substring(3).c_str()));
      if (inputString[1] =='7' && inputString[2] == '1')valve7_controller.set_length(true,atol(inputString.substring(3).c_str()));
      if (inputString[1] =='7' && inputString[2] == '2')valve7_controller.set_length(false,atol(inputString.substring(3).c_str()));
    }
    
    inputString = "";
    stringComplete = false;
}
    

  if (barrel1.is_full())           three_way_valve1.update(CCW,SPARGE_DELAY);
  else                             three_way_valve1.update(CW,SPARGE_DELAY);
  if (barrel2.is_full())           three_way_valve2.update(CCW,SPARGE_DELAY);
  else                             three_way_valve2.update(CW,TANK_2_LOW);  
  if (barrel3.is_full())           three_way_valve3.update(CCW,SPARGE_DELAY);
  else                             three_way_valve3.update(CW,SPARGE_DELAY);
  if (valve4_controller.update())  three_way_valve4.set(CW);
  else                             three_way_valve4.set(CCW);
  if (valve5_controller.update())  three_way_valve5.set(CW);
  else                             three_way_valve5.set(CCW);
  /* ---------- MUTUAL-EXCLUSION FOR MIXING & SCOURING ------------ */
  bool scour_on = valve7_controller.update();   // state-1 = ON, state-2 = OFF
  bool mix_on   = valve6_controller.update();   // state-1 = ON, state-2 = OFF

  if (scour_on) mix_on = false;                 // ***key line: scouring wins***

  valve6.set( mix_on  ? ON  : OFF );
  valve7.set( scour_on? ON  : OFF );
/* -------------------------------------------------------------- */


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
    three_way_valve1.get_pos(),
    three_way_valve2.get_pos(),
    three_way_valve3.get_pos(),
    three_way_valve4.get_pos(),
    three_way_valve5.get_pos(),
      mix_on, scour_on,    // changed to reflect new variables
    barrel1.is_full(),
    barrel2.is_full(),
    barrel3.is_full(),
    valve4_controller.state_1_len, valve4_controller.state_2_len,
    valve5_controller.state_1_len, valve5_controller.state_2_len,
    valve6_controller.state_1_len, valve6_controller.state_2_len,
    valve7_controller.state_1_len, valve7_controller.state_2_len
  );
 

}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
