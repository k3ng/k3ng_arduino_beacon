#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
//#include <SoftwareSerial.h>
#include <dds.h>
#include <EEPROM.h>

// K3NG Beacon Controller
//
// Copyright 2012 Anthony Good, K3NG
// All trademarks referred to in source code and documentation are copyright their respective owners.

    /*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

// Are you a radio artisan ?

// Supported devices:
//  Yaesu FT-817
//  DDS-60 / AD9851

#define CODE_VERSION "2012082901"

#define FEATURE_SERIAL
//#define FEATURE_RIG_CONTROL
//#define RIG_YAESU
#define FEATURE_DDS_TX
//#define DEBUG
//#define FEATURE_DDS_KEYING

#ifdef FEATURE_RIG_CONTROL
#define rig_control_rx 2     // rig serial port RX line
#define rig_control_tx 3     // rig serial port TX line
#endif //FEATURE_RIG_CONTROL

#define tx_1 2              // transmitter 1 keying line (high = key down/tx on)
#define tx_2 3              // transmitter 2 keying line (0 = unused)
#define tx_3 0
#define tx_4 0
#define tx_5 0
#define tx_6 0
#define sidetone_line 4         // connect a speaker for sidetone
#define ptt_tx_1 11             // PTT ("push to talk") lines - these are optional - set to 0 if unused
#define ptt_tx_2 12             
#define ptt_tx_3 0              
#define ptt_tx_4 0
#define ptt_tx_5 0
#define ptt_tx_6 0
#define analog_vin A0           // under development - power and SWR measurement
#define analog_tx_1_pwr A1      // under development
#define analog_tx_2_pwr A2      // under development

#define initial_speed_wpm 20             // keyer speed setting
#define initial_sidetone_freq 600        // sidetone frequency setting
#define char_send_buffer_size 50
#define element_send_buffer_size 20
#define default_length_letterspace 3
#define default_length_wordspace 7
#define initial_ptt_lead_time 10          // PTT lead time in mS
#define initial_ptt_tail_time 200         // PTT tail time in mS
#define default_ptt_hang_time_wordspace_units 0.0 
#define default_serial_baud_rate 115200
#define analog_vin_calibration 50.19 //50.76
#define rig_control_baud 38400
#define dds_calibration_value 0.0000495


enum unit_mode_type {OFFLINE, MAINTENANCE, BEACON, TEST};
enum key_scheduler_type {IDLE, PTT_LEAD_TIME_WAIT, KEY_DOWN, KEY_UP};
enum sidetone_mode_type {SIDETONE_OFF, SIDETONE_ON};
enum char_send_mode_type {CW, HELL};
enum sending_tye {AUTOMATIC_SENDING, MANUAL_SENDING};
enum element_buffer_type {HALF_UNIT_KEY_UP, ONE_UNIT_KEY_DOWN_1_UNIT_KEY_UP, THREE_UNITS_KEY_DOWN_1_UNIT_KEY_UP,ONE_UNIT_KEYDOWN_3_UNITS_KEY_UP, THREE_UNIT_KEYDOWN_3_UNITS_KEY_UP,
  ONE_UNIT_KEYDOWN_7_UNITS_KEY_UP, THREE_UNIT_KEYDOWN_7_UNITS_KEY_UP, SEVEN_UNITS_KEY_UP, KEY_UP_LETTERSPACE_MINUS_1, KEY_UP_WORDSPACE_MINUS_4, KEY_UP_WORDSPACE};

#define SERIAL_SEND_BUFFER_WPM_CHANGE 200
#define SERIAL_SEND_BUFFER_PTT_ON 201
#define SERIAL_SEND_BUFFER_PTT_OFF 202
#define SERIAL_SEND_BUFFER_TIMED_KEY_DOWN 203
#define SERIAL_SEND_BUFFER_TIMED_WAIT 204
#define SERIAL_SEND_BUFFER_NULL 205
#define SERIAL_SEND_BUFFER_PROSIGN 206
#define SERIAL_SEND_BUFFER_HOLD_SEND 207
#define SERIAL_SEND_BUFFER_HOLD_SEND_RELEASE 208
#define SERIAL_SEND_BUFFER_MEMORY_NUMBER 210

#define SERIAL_SEND_BUFFER_NORMAL 0
#define SERIAL_SEND_BUFFER_TIMED_COMMAND 1
#define SERIAL_SEND_BUFFER_HOLD 2

#define NORMAL 0
#define OMIT_LETTERSPACE 1

// chip, data_pin, load_pin, clock_pin, clock_mhz
//dds ddschip(DDS9850, 8, 6, 7, 100000000LL);     // DDS-50
dds ddschip(DDS9851, 8, 6, 7, 180000000LL);         // DDS-60
//dds ddschip(DDS9834, 8, 6, 7, 25000000LL);

//SoftwareSerial rig(rig_control_rx,rig_control_tx);

byte key_scheduler_state = IDLE;
unsigned long next_key_scheduler_transition_time = 0;
unsigned int key_scheduler_keyup_ms;
unsigned int key_scheduler_keydown_ms;
unsigned int wpm = initial_speed_wpm;
byte current_tx = 1;
unsigned long ptt_time;
byte ptt_line_activated = 0;
byte key_state = 0;
byte key_tx = 1;
byte sidetone_mode = SIDETONE_ON;
unsigned int hz_sidetone = initial_sidetone_freq;
byte char_send_mode = CW;
byte length_letterspace = default_length_letterspace;
byte length_wordspace = default_length_wordspace;
byte manual_ptt_invoke = 0;
byte last_sending_type = MANUAL_SENDING;
unsigned int ptt_tail_time = initial_ptt_tail_time;
unsigned int ptt_lead_time = initial_ptt_lead_time;
float ptt_hang_time_wordspace_units = default_ptt_hang_time_wordspace_units;
byte unit_mode;
unsigned long current_frequency;

#ifdef FEATURE_SERIAL
byte incoming_serial_byte;
unsigned long serial_baud_rate;
byte serial_backslash_command;
#endif //FEATURE_SERIAL

byte pause_sending_buffer = 0;
byte char_send_buffer_array[char_send_buffer_size];
byte char_send_buffer_bytes = 0;
byte char_send_buffer_status = SERIAL_SEND_BUFFER_NORMAL;

byte element_send_buffer_array[element_send_buffer_size];
byte element_send_buffer_bytes = 0;

unsigned long beacon_cycle_count = 0;
unsigned int millis_rollover = 0;

byte sequence = 1;


void setup() {
  
  initialize_pins();   
  initialize_serial();
  //wdt_enable(WDTO_120MS);
  initialize_transmitters();
  switch_to_tx(1);
  delay(500);
  unit_mode = BEACON;
  sidetone_mode = SIDETONE_ON;
}

void loop() {
  
    
  // useful subroutines:
  //   switch_to_tx(byte transmitter_number)
  //   send_character_string(char* string_to_send)
  //   rig_ptt(byte ptt_on)   
  ///  change_frequency(byte rig_number, unsigned long set_frequency)
    
 
//  if (EEPROM.read(250) == 1) {
//    EEPROM.write(250,255);
//    switch_to_tx(1);                   // switch to transmitter one
//    change_frequency(1,50010000LL);  
//    tx(HIGH);
//    while (1) {}
//  } else {
//    EEPROM.write(250,1);
//    switch_to_tx(2);
//    change_frequency(1,28291500LL);
//    tx(HIGH);
//    while (1) {}
//  }  
    
  // message sequencer - customize this area
  if (keyer_is_idle() && unit_mode == BEACON) {
    delay(50);                           
    switch (sequence) {
      case 1:
        switch_to_tx(1);                   // switch to transmitter one
        //change_frequency(1,50010000LL);
        change_frequency(1,400LL);
        //change_frequency(1,25005000LL);
        send_character_string("K3NG/B");   // send this character string in CW
        sequence++;                        // increment for the next sequence
        break;
      case 2:
        switch_to_tx(2);
        change_frequency(1,28291500LL);
        send_character_string("K3NG/B");
        sequence++;
        break;    
      case 3:
        switch_to_tx(1);
        change_frequency(1,50010000LL);
        //change_frequency(1,25005000LL);
        send_character_string("FN20");
        sequence++;
        break;
      case 4:
        switch_to_tx(2);
        change_frequency(1,28291500LL);
        send_character_string("FN20");
        sequence++;
        break;
      default:
        sequence = 1;  
        break;
    }    
    beacon_cycle_count++;          
  }

  // end of sequencer

  // below here is all state machine / scheduler stuff... what makes it work
  
  if (unit_mode == BEACON) {       
    service_key_scheduler(); 
    check_ptt_tail();
    service_element_send_buffer();
    service_char_send_buffer();
  }
      
  millis_rollover_check();
  
  #ifdef FEATURE_SERIAL
  check_serial();
  #endif
  
  wdt_reset();
  
}

//-------------------------------------------------------------------------------------------------------

void change_frequency(byte rig_number, unsigned long set_frequency){
  
  #ifdef FEATURE_DDS_TX
  ddschip.setfrequency(set_frequency);
  #endif 
  
  current_frequency = set_frequency;
  
}


//-------------------------------------------------------------------------------------------------------

#ifdef FEATURE_RIG_CONTROL
void rig_ptt(byte ptt_on) {

  #ifdef RIG_YAESU
  rig.write((byte)0x00);
  rig.write((byte)0x00);
  rig.write((byte)0x00);
  rig.write((byte)0x00);
  if (ptt_on) {
    rig.write((byte)0x08);
  } else {
    rig.write((byte)0x88);
  }
  delay(100);
  #endif //RIG_YAESU

}    
#endif   

//-------------------------------------------------------------------------------------------------------
void switch_to_tx(byte transmitter)
{
  
//  #ifdef FEATURE_RIG_CONTROL
//  rig_ptt(0);
//  #endif
  
  switch(transmitter) {
    case 1:
      #ifdef FEATURE_RIG_CONTROL
      #ifdef RIG_YAESU
      rig.write(0x05);                 // switch to frequency 50.001000 Mhz
      rig.write((byte)0x00);
      rig.write(0x10);
      rig.write((byte)0x00);
      rig.write(1);
      delay(100);
      #endif //RIG_YAESU
      #endif //FEATURE_RIG_CONTROL          
      current_tx = 1;
      break;
    case 2:                            // switch to frequency 28.291500 Mhz
      #ifdef FEATURE_RIG_CONTROL
      #ifdef RIG_YAESU
      rig.write(0x02);
      rig.write(0x82);
      rig.write(0x91);
      rig.write(0x50);
      rig.write(1);
      delay(100);
      #endif //RIG_YAESU
      #endif //FEATURE_RIG_CONTROL     
      current_tx = 2;
      break;
  }  
  
  
//  #ifdef FEATURE_RIG_CONTROL
//  rig_ptt(1);
//  #endif  
  
}



//-------------------------------------------------------------------------------------------------------
byte keyer_is_idle() {
  
  if ((!char_send_buffer_bytes) && (!element_send_buffer_bytes) && (!ptt_line_activated) && (key_scheduler_state == IDLE)) {
    return 1;
  } else {
    return 0;
  }
  
}


//-------------------------------------------------------------------------------------------------------
void send_character_string(char* string_to_send) {
  
  for (int x = 0;x < 32;x++) {
    if (string_to_send[x] != 0) {
      add_to_char_send_buffer(string_to_send[x]);
    } else {
      x = 33;
    }
  }
}
//-------------------------------------------------------------------------------------------------------
void tx(byte state)
{
  switch (current_tx){
    case 1: digitalWrite (tx_1, state); break;
    case 2: digitalWrite (tx_2, state); break;
    case 3: digitalWrite (tx_3, state); break;
    case 4: digitalWrite (tx_4, state); break;
    case 5: digitalWrite (tx_5, state); break;
    case 6: digitalWrite (tx_6, state); break;
  }  
  
  #ifdef FEATURE_DDS_KEYING
  if (state == HIGH) {
    ddschip.setfrequency(current_frequency);
  } else {
    ddschip.setfrequency(0);
  }  
  #endif
  
  
}


//-------------------------------------------------------------------------------------------------------

void ptt(byte state)
{
  switch (current_tx){
    case 1: if (ptt_tx_1) {digitalWrite (ptt_tx_1, state);} break;
    case 2: if (ptt_tx_2) {digitalWrite (ptt_tx_2, state);} break;
    case 3: if (ptt_tx_3) {digitalWrite (ptt_tx_3, state);} break;
    case 4: if (ptt_tx_4) {digitalWrite (ptt_tx_4, state);} break;
    case 5: if (ptt_tx_5) {digitalWrite (ptt_tx_5, state);} break;
    case 6: if (ptt_tx_6) {digitalWrite (ptt_tx_6, state);} break;
  }  
}

//-------------------------------------------------------------------------------------------------------

void tx_and_sidetone_key (int state)
{
  if ((state) && (key_state == 0)) {
    if (key_tx) {
      ptt_key();      
      tx(HIGH);
    }
    if (sidetone_mode == SIDETONE_ON){
      tone(sidetone_line, hz_sidetone);
    }
    key_state = 1;
  } else {
    if ((state == 0) && (key_state)) {
      if (key_tx) {
        tx(LOW);
      }
      if (sidetone_mode == SIDETONE_ON) {
        noTone(sidetone_line);
      }
      key_state = 0;
    }          
  }
}  

//-------------------------------------------------------------------------------------------------------
void check_ptt_tail()
{ 
  if ((key_state) || (key_scheduler_state == PTT_LEAD_TIME_WAIT)) {
    ptt_time = millis();
  } else {
    if ((ptt_line_activated) && (manual_ptt_invoke == 0) && ((millis() - ptt_time) > ptt_tail_time)){
      ptt_unkey();
    }
  }
}

//-------------------------------------------------------------------------------------------------------


void ptt_key()
{
  if (ptt_line_activated == 0) {   // if PTT is currently deactivated, bring it up and insert PTT lead time delay
    ptt(HIGH);
    ptt_line_activated = 1;      
  }
  ptt_time = millis();
}

//-------------------------------------------------------------------------------------------------------

void ptt_unkey()
{
  if (ptt_line_activated) {
    ptt(LOW);
    ptt_line_activated = 0;      
  }  
}



//-------------------------------------------------------------------------------------------------------

void schedule_keydown_keyup (unsigned int keydown_ms, unsigned int keyup_ms)
{
  if (keydown_ms) {
    if ((ptt_lead_time) && (!ptt_line_activated)) {
      ptt_key();
      key_scheduler_state = PTT_LEAD_TIME_WAIT;
      next_key_scheduler_transition_time = millis() + ptt_lead_time;
      key_scheduler_keydown_ms = keydown_ms;
      key_scheduler_keyup_ms = keyup_ms;
    } else {
      tx_and_sidetone_key(1);
      key_scheduler_state = KEY_DOWN;
      next_key_scheduler_transition_time = millis() + keydown_ms;
      key_scheduler_keyup_ms = keyup_ms;      
    }
  } else {
    tx_and_sidetone_key(0);
    key_scheduler_state = KEY_UP;
    next_key_scheduler_transition_time = millis() + keyup_ms;
  }
  
  
}

//-------------------------------------------------------------------------------------------------------

void service_key_scheduler()
{
  
  switch (key_scheduler_state) {
    case PTT_LEAD_TIME_WAIT:
      if (millis() >= next_key_scheduler_transition_time) {
        tx_and_sidetone_key(1);
        key_scheduler_state = KEY_DOWN;
        next_key_scheduler_transition_time = (millis() + key_scheduler_keydown_ms);
      }
      break;        
    case KEY_DOWN:
      if (millis() >= next_key_scheduler_transition_time) {
        tx_and_sidetone_key(0);
        key_scheduler_state = KEY_UP;
        if (key_scheduler_keyup_ms) {
          next_key_scheduler_transition_time = (millis() + key_scheduler_keyup_ms);
        } else {
          key_scheduler_state = IDLE;
        }
      }
      break;
    case KEY_UP:
      if (millis() >= next_key_scheduler_transition_time) {
        key_scheduler_state = IDLE;
      }
      break;    
  }
}

//-------------------------------------------------------------------------------------------------------

void service_char_send_buffer() {

  if ((char_send_buffer_bytes > 0) && (pause_sending_buffer == 0) && (element_send_buffer_bytes == 0)) {
    send_char(char_send_buffer_array[0],NORMAL);
    remove_from_char_send_buffer();    
  }
  
}

//-------------------------------------------------------------------------------------------------------

void remove_from_char_send_buffer()
{
  if (char_send_buffer_bytes > 0) {
    char_send_buffer_bytes--;
  }
  if (char_send_buffer_bytes > 0) {
    for (int x = 0;x < char_send_buffer_bytes;x++) {
      char_send_buffer_array[x] = char_send_buffer_array[x+1];
    }
  }
}

//-------------------------------------------------------------------------------------------------------

void add_to_char_send_buffer(byte incoming_serial_byte) {

    if (char_send_buffer_bytes < char_send_buffer_size) {
      if (incoming_serial_byte != 127) {
        char_send_buffer_bytes++;
        char_send_buffer_array[char_send_buffer_bytes - 1] = incoming_serial_byte;
      } else {  // we got a backspace
        char_send_buffer_bytes--;
      }
    } 

}


//-------------------------------------------------------------------------------------------------------

void send_char(char cw_char, byte omit_letterspace)
{
  #ifdef DEBUG
  Serial.write("\nsend_char: called with cw_char:");
  Serial.print(cw_char);
  if (omit_letterspace) {
    Serial.print (" OMIT_LETTERSPACE");
  }
  Serial.write("\n\r");
  #endif
  
  if ((cw_char == 10) || (cw_char == 13)) { return; }  // don't attempt to send carriage return or line feed
  
  if (char_send_mode == CW) {
    switch (cw_char) {
      case 'A': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case 'B': send_dah(AUTOMATIC_SENDING); send_dits(3); break;
      case 'C': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case 'D': send_dah(AUTOMATIC_SENDING); send_dits(2); break;
      case 'E': send_dit(AUTOMATIC_SENDING); break;
      case 'F': send_dits(2); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case 'G': send_dahs(2); send_dit(AUTOMATIC_SENDING); break;
      case 'H': send_dits(4); break;
      case 'I': send_dits(2); break;
      case 'J': send_dit(AUTOMATIC_SENDING); send_dahs(3); break;
      case 'K': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case 'L': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dits(2); break;
      case 'M': send_dahs(2); break;
      case 'N': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case 'O': send_dahs(3); break;
      case 'P': send_dit(AUTOMATIC_SENDING); send_dahs(2); send_dit(AUTOMATIC_SENDING); break;
      case 'Q': send_dahs(2); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case 'R': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case 'S': send_dits(3); break;
      case 'T': send_dah(AUTOMATIC_SENDING); break;
      case 'U': send_dits(2); send_dah(AUTOMATIC_SENDING); break;    
      case 'V': send_dits(3); send_dah(AUTOMATIC_SENDING); break;
      case 'W': send_dit(AUTOMATIC_SENDING); send_dahs(2); break;
      case 'X': send_dah(AUTOMATIC_SENDING); send_dits(2); send_dah(AUTOMATIC_SENDING); break;
      case 'Y': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dahs(2); break;
      case 'Z': send_dahs(2); send_dits(2); break;
          
      case '0': send_dahs(5); break;
      case '1': send_dit(AUTOMATIC_SENDING); send_dahs(4); break;
      case '2': send_dits(2); send_dahs(3); break;
      case '3': send_dits(3); send_dahs(2); break;
      case '4': send_dits(4); send_dah(AUTOMATIC_SENDING); break;
      case '5': send_dits(5); break;
      case '6': send_dah(AUTOMATIC_SENDING); send_dits(4); break;
      case '7': send_dahs(2); send_dits(3); break;
      case '8': send_dahs(3); send_dits(2); break;
      case '9': send_dahs(4); send_dit(AUTOMATIC_SENDING); break;
      
      case '=': send_dah(AUTOMATIC_SENDING); send_dits(3); send_dah(AUTOMATIC_SENDING); break;
      case '/': send_dah(AUTOMATIC_SENDING); send_dits(2); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case ' ': add_to_element_send_buffer(KEY_UP_WORDSPACE_MINUS_4); break;
      case '*': send_dah(AUTOMATIC_SENDING); send_dits(3); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;    // using asterisk for BK
      case '.': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case ',': send_dahs(2); send_dits(2); send_dahs(2); break;
      case '\'': send_dit(AUTOMATIC_SENDING); send_dahs(4); send_dit(AUTOMATIC_SENDING); break;                   // apostrophe
      case '!': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dahs(2); break;
      case '(': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dahs(2); send_dit(AUTOMATIC_SENDING); break;
      case ')': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dahs(2); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case '&': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dits(3); break;
      case ':': send_dahs(3); send_dits(3); break;
      case ';': send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case '+': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case '-': send_dah(AUTOMATIC_SENDING); send_dits(4); send_dah(AUTOMATIC_SENDING); break;
      case '_': send_dits(2); send_dahs(2); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;
      case '"': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dits(2); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case '$': send_dits(3); send_dah(AUTOMATIC_SENDING); send_dits(2); send_dah(AUTOMATIC_SENDING); break;
      case '@': send_dit(AUTOMATIC_SENDING); send_dahs(2); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;
      case '<': send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); break;     // AR
      case '>': send_dits(3); send_dah(AUTOMATIC_SENDING); send_dit(AUTOMATIC_SENDING); send_dah(AUTOMATIC_SENDING); break;               // SK
      case '\n': break;
      case '\r': break;
      case '|': add_to_element_send_buffer(HALF_UNIT_KEY_UP); return; break;  
      default: send_dits(2); send_dahs(2); send_dits(2); break;
    }  
    if (omit_letterspace != OMIT_LETTERSPACE) {
      add_to_element_send_buffer(KEY_UP_LETTERSPACE_MINUS_1); //this is minus one because send_dit and send_dah have a trailing element space
    }
  } else {
    #ifdef FEATURE_HELL
      transmit_hell_char(cw_char);
    #endif 
  }
  
}

//-------------------------------------------------------------------------------------------------------
void send_dit(byte sending_type) {
  add_to_element_send_buffer(ONE_UNIT_KEY_DOWN_1_UNIT_KEY_UP);
}

//-------------------------------------------------------------------------------------------------------
void send_dah(byte sending_type) {
  add_to_element_send_buffer(THREE_UNITS_KEY_DOWN_1_UNIT_KEY_UP);
}

//-------------------------------------------------------------------------------------------------------

void send_dits(int dits)
{
  for (;dits > 0;dits--) {
    send_dit(AUTOMATIC_SENDING);
  } 
  
}

//-------------------------------------------------------------------------------------------------------

void send_dahs(int dahs)
{
  for (;dahs > 0;dahs--) {
    send_dah(AUTOMATIC_SENDING);
  } 
  
}

//-------------------------------------------------------------------------------------------------------

void add_to_element_send_buffer(byte element_byte)
{
  if (element_send_buffer_bytes < element_send_buffer_size) {
    element_send_buffer_array[element_send_buffer_bytes] = element_byte;
    element_send_buffer_bytes++;
  } 

}

//-------------------------------------------------------------------------------------------------------

void remove_from_element_send_buffer()
{
  if (element_send_buffer_bytes > 0) {
    element_send_buffer_bytes--;
  }
  if (element_send_buffer_bytes > 0) {
    for (int x = 0;x < element_send_buffer_bytes;x++) {
      element_send_buffer_array[x] = element_send_buffer_array[x+1];
    }
  }
}

//-------------------------------------------------------------------------------------------------------


void service_element_send_buffer(){
  
  /*
  enum element_buffer_type {HALF_UNIT_KEY_UP, ONE_UNIT_KEY_DOWN_1_UNIT_KEY_UP, THREE_UNITS_KEY_DOWN_1_UNIT_KEY_UP,ONE_UNIT_KEYDOWN_3_UNITS_KEY_UP, THREE_UNIT_KEYDOWN_3_UNITS_KEY_UP,
  ONE_UNIT_KEYDOWN_7_UNITS_KEY_UP, THREE_UNIT_KEYDOWN_7_UNITS_KEY_UP, SEVEN_UNITS_KEY_UP, KEY_UP_LETTERSPACE_MINUS_1, KEY_UP_WORDSPACE_MINUS_1};
  */
  
  if ((key_scheduler_state == IDLE) && (element_send_buffer_bytes > 0)) {
    switch(element_send_buffer_array[0]) {
 
       case HALF_UNIT_KEY_UP:
         schedule_keydown_keyup(0,0.5*(1200/wpm));
         remove_from_element_send_buffer();
         break;
      
       case ONE_UNIT_KEY_DOWN_1_UNIT_KEY_UP:
         schedule_keydown_keyup(1200/wpm,1200/wpm);
         remove_from_element_send_buffer();
         break;
       
       case THREE_UNITS_KEY_DOWN_1_UNIT_KEY_UP:
         schedule_keydown_keyup(3*(1200/wpm),1200/wpm);
         remove_from_element_send_buffer();
         break;
         
       case KEY_UP_LETTERSPACE_MINUS_1:
         schedule_keydown_keyup(0,(length_letterspace-1)*(1200/wpm));
         remove_from_element_send_buffer();
         break;         
         
       case KEY_UP_WORDSPACE_MINUS_4:
         schedule_keydown_keyup(0,(length_wordspace-4)*(1200/wpm));
         remove_from_element_send_buffer();
         break;  
 
       case KEY_UP_WORDSPACE:
         schedule_keydown_keyup(0,length_wordspace*(1200/wpm));
         remove_from_element_send_buffer();
         break;         
      
    }
  }
  
}

//---------------------------------------------------------------------

#ifdef FEATURE_SERIAL
void check_serial()
{
  
  while (Serial.available() > 0) {  
    if (serial_backslash_command == 0) {
      incoming_serial_byte = Serial.read(); 
      incoming_serial_byte = uppercase(incoming_serial_byte);
      if (incoming_serial_byte != 92) {
      } else {                                // we're getting a backslash command
        serial_backslash_command = 1;
        Serial.write(incoming_serial_byte);          
      }
    } else {       
        incoming_serial_byte = Serial.read();
        Serial.write(incoming_serial_byte);
        incoming_serial_byte = uppercase(incoming_serial_byte);
        process_serial_command(incoming_serial_byte);
        serial_backslash_command = 0; 
        Serial.write("\n\r");        
    }
  }  //while (Serial.available() > 0)
}
#endif


//---------------------------------------------------------------------

#ifdef FEATURE_SERIAL

void process_serial_command(byte incoming_serial_byte) {

  Serial.print(F("\n\r"));
  
  switch (incoming_serial_byte) {   
    case 83: serial_status(); break;
    
    case 126: wdt_enable(WDTO_30MS); while(1) {} ; break;  // ~ - reset unit
    default: Serial.write("\nUnknown command\n\r"); break;
  }
  
          
}
#endif //FEATURE_SERIAL

//---------------------------------------------------------------------

#ifdef FEATURE_SERIAL

int serial_get_number_input(byte places,int lower_limit, int upper_limit)
{
  byte incoming_serial_byte = 0;
  byte looping = 1;
  byte error = 0;
  String numberstring = "";
  byte numberindex = 0;
  int numbers[6];
  
  while (looping) {
    if (Serial.available() == 0) {        // wait for the next keystroke

    } else {  
      incoming_serial_byte = Serial.read();
      if ((incoming_serial_byte > 47) && (incoming_serial_byte < 58)) {    // ascii 48-57 = "0" - "9")
        numberstring = numberstring + incoming_serial_byte;
        numbers[numberindex] = incoming_serial_byte;
        numberindex++;
        if (numberindex > places){
            looping = 0;
            error = 1;
        }
      } else {
        if (incoming_serial_byte == 13) {   // carriage return - get out
          looping = 0;
        } else {                 // bogus input - error out
          looping = 0;
          error = 1;
        }
      }
    }
  }
  if (error) {
    Serial.write("Error...\n\r");
    while (Serial.available() > 0) { incoming_serial_byte = Serial.read(); }  // clear out buffer
    return(-1);
  } else {
    int y = 1;
    int return_number = 0;
    for (int x = (numberindex - 1); x >= 0 ; x = x - 1) {
      return_number = return_number + ((numbers[x]-48) * y);
      y = y * 10;
    }
    if ((return_number > lower_limit) && (return_number < upper_limit)) {
      return(return_number); 
    } else {
      Serial.write("Error...\n\r");  
      return(-1);
    }
  }
}

#endif

//---------------------------------------------------------------------

#ifdef FEATURE_SERIAL

void serial_status() {
    
  serial_uptime_stamp();
  memorycheck();
  Serial.print(F(" beacon_cycle_count="));
  Serial.print(beacon_cycle_count);
  Serial.print(F(" millis="));
  Serial.print(millis());
  Serial.print(F(" millis_rollover="));
  Serial.print(millis_rollover);
  Serial.print(F(" Vin="));
  Serial.println((analogRead(analog_vin)/analog_vin_calibration));
  
}

#endif
//---------------------------------------------------------------------
void serial_uptime_stamp() {
  
  unsigned long days = 0;
  unsigned long hours = 0;
  unsigned long minutes = 0;
  unsigned long seconds = 0;
  
  seconds = (millis() /1000);
  minutes = seconds /60;
  hours = minutes /60;
  days = hours /24;
  seconds = seconds - ( minutes * 60);
  minutes = minutes - ( hours * 60);
  hours = hours - ( days * 24);
  
  // add rollovers
  days = days + (millis_rollover * 49);
  hours = hours + (millis_rollover * 17);
  minutes = minutes + (millis_rollover * 2);
  seconds = seconds + (millis_rollover * 47);
  
  if (seconds > 59) {
    minutes = minutes + int(seconds / 60);
    seconds = seconds - (int(seconds/60) * 60);
  }
    
  if (minutes > 59) {
    hours = hours + int(minutes / 60);
    minutes = minutes - (int(minutes/60) * 60);
  }
   
  if (hours > 24) { 
    days = days + (int(hours / 24));
    hours = hours - (int(hours/24) * 24);
  }
  
  Serial.print(days);
  Serial.print(F(":"));
  if (hours < 10) {
    Serial.print(F("0"));
  }
  Serial.print(hours);
  Serial.print(F(":"));
  if (minutes < 10) {
    Serial.print(F("0"));
  }
  Serial.print(minutes);
  Serial.print(F(":"));
  if (seconds < 10) {
    Serial.print(F("0"));
  }  
  Serial.print(seconds);
  Serial.print(F(" - "));
  
}

//---------------------------------------------------------------------
void memorycheck()
{
  void* HP = malloc(4);
  if (HP)
    free (HP);
    
  unsigned long free = (unsigned long)SP - (unsigned long)HP;
  
  Serial.print(F("heap="));
  Serial.print((unsigned long)HP,HEX);
  Serial.print(F(" stack="));
  Serial.print((unsigned long)SP,HEX);
  Serial.print(F(" free_memory_bytes="));
//  Serial.print((unsigned long)free,HEX);
//  Serial.print("  ");
  if (free > 2048) {
    free = 0;
  }
  Serial.print((unsigned long)free,DEC);
}

//---------------------------------------------------------------------

void millis_rollover_check() {
  
  static unsigned long last_millis = 0;
  
  if (millis() < last_millis) {
    millis_rollover++;
  }
  last_millis = millis();
}

//-------------------------------------------------------------------------------------------------------

int uppercase (int charbytein)
{
  if ((charbytein > 96) && (charbytein < 123)) {
    charbytein = charbytein - 32;
  }
  return charbytein;
}

//-------------------------------------------------------------------------------------------------------

void initialize_pins(){
  
  if (tx_1) {
    pinMode (tx_1, OUTPUT);
    digitalWrite (tx_1, LOW);
  }  
  if (tx_2) {    
    pinMode (tx_2, OUTPUT);
    digitalWrite (tx_2, LOW);
  }
  if (tx_3) {
    pinMode (tx_3, OUTPUT);
    digitalWrite (tx_3, LOW);
  }  
  if (tx_4) {
    pinMode (tx_4, OUTPUT);
    digitalWrite (tx_4, LOW);
  }  
  if (tx_5) {
    pinMode (tx_5, OUTPUT);
    digitalWrite (tx_5, LOW);
  }  
  if (tx_6) {
    pinMode (tx_6, OUTPUT);
    digitalWrite (tx_6, LOW);
  }   

  
  if (ptt_tx_1) {
    pinMode (ptt_tx_1, OUTPUT);
    digitalWrite (ptt_tx_1, LOW);
  }  
  if (ptt_tx_2) {    
    pinMode (ptt_tx_2, OUTPUT);
    digitalWrite (ptt_tx_2, LOW);
  }
  if (ptt_tx_3) {
    pinMode (ptt_tx_3, OUTPUT);
    digitalWrite (ptt_tx_3, LOW);
  }  
  if (ptt_tx_4) {
    pinMode (ptt_tx_4, OUTPUT);
    digitalWrite (ptt_tx_4, LOW);
  }  
  if (ptt_tx_5) {
    pinMode (ptt_tx_5, OUTPUT);
    digitalWrite (ptt_tx_5, LOW);
  }  
  if (ptt_tx_6) {
    pinMode (ptt_tx_6, OUTPUT);
    digitalWrite (ptt_tx_6, LOW);
  }  
  pinMode (sidetone_line, OUTPUT);
  digitalWrite (sidetone_line, LOW);  

}  

//-------------------------------------------------------------------------------------------------------

void initialize_transmitters() {
  
  #ifdef FEATURE_DDS_TX
  ddschip.set_clock_multiplier(1);
  ddschip.calibrate(dds_calibration_value);
  #endif
  
  #ifdef FEATURE_RIG_CONTROL
  rig.begin(rig_control_baud);
  #endif
  
}
//-------------------------------------------------------------------------------------------------------

void initialize_serial() {
  
  #ifdef SERIAL
  serial_baud_rate = default_serial_baud_rate;
  Serial.begin(serial_baud_rate);
  serial_uptime_stamp();
  Serial.print(F("K3NG Beacon Controller Version "));
  Serial.write(CODE_VERSION);
  Serial.println();
  #ifdef DEBUG_MEMORYCHECK
  memorycheck();
  #endif DEBUG_MEMORYCHECK
  #endif //SERIAL
}
//-------------------------------------------------------------------------------------------------------
