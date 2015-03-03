#include <stdio.h>
//#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

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

#define CODE_VERSION "2012012701"

// Command Line Interface ("CLI") (USB Port) (Note: turn on carriage return if using Arduino Serial Monitor program)
//    \s     Status
//    \~     Reset unit


// compile time features and options - comment or uncomment to add or delete features
#define FEATURE_SERIAL
//#define FEATURE_HELL

// don't touch these unless you know what the hell you are doing
//#define DEBUG_SEND_CHAR
//#define DEBUG_HELL_TEST
//#define DEBUG_CHECK_SERIAL


// Pins

#define tx_1 2              // transmitter 1 keying line (high = key down/tx on)
#define tx_2 3
#define tx_3 0
#define tx_4 0
#define tx_5 0
#define tx_6 0
#define sidetone_line 4         // connect a speaker for sidetone
#define ptt_tx_1 13             // PTT ("push to talk") lines
#define ptt_tx_2 12             //   Can be used for keying fox transmitter, T/R switch, or keying slow boatanchors
#define ptt_tx_3 0              //   These are optional - set to 0 if unused
#define ptt_tx_4 0
#define ptt_tx_5 0
#define ptt_tx_6 0

// Initial and hardcoded settings
#define initial_speed_wpm 26             // "factory default" keyer speed setting
#define initial_sidetone_freq 600        // "factory default" sidetone frequency setting
#define hz_high_beep 1500                // frequency in hertz of high beep
#define hz_low_beep 400                  // frequency in hertz of low beep
#define initial_dah_to_dit_ratio 300     // 300 = 3 / normal 3:1 ratio
#define initial_ptt_lead_time 10          // PTT lead time in mS
#define initial_ptt_tail_time 10         // PTT tail time in mS
#define initial_qrss_dit_length 1        // QRSS dit length in seconds
#define default_serial_baud_rate 115200
#define default_length_letterspace 3
#define default_length_wordspace 7
#define default_weighting 50             // 50 = weighting factor of 1 (normal)
#define default_ptt_hang_time_wordspace_units 0.0 
#define hell_pixel_microseconds 4025

// Variable macros
#define OMIT_LETTERSPACE 1

#define SIDETONE_OFF 0
#define SIDETONE_ON 1

#define SENDING_NOTHING 0
#define SENDING_DIT 1
#define SENDING_DAH 2

#define SPEED_NORMAL 0
#define SPEED_QRSS 1

#define CW 0
#define HELL 1

// Variables and stuff
unsigned int wpm;
byte command_mode_disable_tx = 0;
unsigned int hz_sidetone = initial_sidetone_freq;
unsigned int dah_to_dit_ratio = initial_dah_to_dit_ratio;
//byte current_ptt_line = ptt_tx_1;
byte current_tx = 1;
unsigned int ptt_tail_time = initial_ptt_tail_time;
unsigned int ptt_lead_time = initial_ptt_lead_time;
byte manual_ptt_invoke = 0;
byte qrss_dit_length = initial_qrss_dit_length;
byte sidetone_mode;  // SIDETONE_OFF, SIDETONE_ON
byte char_send_mode; // CW, HELL
byte key_tx;         // 0 = tx_line control suppressed
byte being_sent;     // SENDING_NOTHING, SENDING_DIT, SENDING_DAH
byte key_state;      // 0 = key up, 1 = key down
unsigned long ptt_time;
byte ptt_line_activated = 0;
byte speed_mode = SPEED_NORMAL;
byte length_letterspace = default_length_letterspace;
byte length_wordspace = default_length_wordspace;
byte weighting = default_weighting;
unsigned long beacon_cycle_count = 0;
unsigned int millis_rollover = 0;

#ifdef FEATURE_SERIAL
byte incoming_serial_byte;
long serial_baud_rate;
byte serial_backslash_command;
#endif //FEATURE_SERIAL


#ifdef FEATURE_HELL
prog_uchar hell_font1[] PROGMEM = {B00111111, B11100000, B00011001, B11000000, B01100011, B00000001, B10011100, B00111111, B11100000,    // A
                                   B00110000, B00110000, B11111111, B11000011, B00110011, B00001100, B11001100, B00011100, B11100000,    // B
                                   B00111111, B11110000, B11000000, B11000011, B00000011, B00001100, B00001100, B00110000, B00110000,    // C
                                   B00110000, B00110000, B11111111, B11000011, B00000011, B00001100, B00001100, B00011111, B11100000,    // D
                                   B00111111, B11110000, B11001100, B11000011, B00110011, B00001100, B00001100, B00110000, B00110000,
                                   B00111111, B11110000, B00001100, B11000000, B00110011, B00000000, B00001100, B00000000, B00110000,
                                   B00111111, B11110000, B11000000, B11000011, B00000011, B00001100, B11001100, B00111111, B00110000,
                                   B00111111, B11110000, B00001100, B00000000, B00110000, B00000000, B11000000, B00111111, B11110000,
                                   B00000000, B00000000, B00000000, B00000011, B11111111, B00000000, B00000000, B00000000, B00000000,
                                   B00111100, B00000000, B11000000, B00000011, B00000000, B00001100, B00000000, B00111111, B11110000,
                                   B00111111, B11110000, B00001100, B00000000, B01110000, B00000011, B00110000, B00111000, B11100000,
                                   B00111111, B11110000, B11000000, B00000011, B00000000, B00001100, B00000000, B00110000, B00000000,
                                   B00111111, B11110000, B00000001, B10000000, B00001100, B00000000, B00011000, B00111111, B11110000,
                                   B00111111, B11110000, B00000011, B10000000, B00111000, B00000011, B10000000, B00111111, B11110000,
                                   B00111111, B11110000, B11000000, B11000011, B00000011, B00001100, B00001100, B00111111, B11110000,
                                   B00110000, B00110000, B11111111, B11000011, B00110011, B00000000, B11001100, B00000011, B11110000,
                                   B00111111, B11110000, B11000000, B11000011, B11000011, B00001111, B11111100, B11110000, B00000000,
                                   B00111111, B11110000, B00001100, B11000000, B00110011, B00000011, B11001100, B00111001, B11100000,
                                   B00110001, B11100000, B11001100, B11000011, B00110011, B00001100, B11001100, B00011110, B00110000,
                                   B00000000, B00110000, B00000000, B11000011, B11111111, B00000000, B00001100, B00000000, B00110000,
                                   B00111111, B11110000, B11000000, B00000011, B00000000, B00001100, B00000000, B00111111, B11110000,
                                   B00111111, B11110000, B01110000, B00000000, B01110000, B00000000, B01110000, B00000000, B01110000,
                                   B00011111, B11110000, B11000000, B00000001, B11110000, B00001100, B00000000, B00011111, B11110000,
                                   B00111000, B01110000, B00110011, B00000000, B01111000, B00000011, B00110000, B00111000, B01110000,
                                   B00000000, B01110000, B00000111, B00000011, B11110000, B00000000, B01110000, B00000000, B01110000,
                                   B00111000, B00110000, B11111000, B11000011, B00110011, B00001100, B01111100, B00110000, B01110000};   // Z
                                   
prog_uchar hell_font2[] PROGMEM = {B00011111, B11100000, B11000000, B11000011, B00000011, B00001100, B00001100, B00011111, B11100000,   // 0
                                   B00000000, B00000000, B00000011, B00000000, B00000110, B00001111, B11111100, B00000000, B00000000,
                                   B00111000, B01100000, B11110000, B11000011, B00110011, B00001100, B01111000, B00110000, B00000000,                            
                                   B11000000, B00000011, B00000000, B11000110, B00110011, B00001100, B11111100, B00011110, B00000000,
                                   B00000111, B11111000, B00011000, B00000000, B01100000, B00001111, B11111100, B00000110, B00000000,
                                   B00110000, B00000000, B11000000, B00000011, B00011111, B10000110, B01100110, B00001111, B00011000,
                                   B00011111, B11110000, B11001100, B01100011, B00011000, B11001100, B01100000, B00011111, B00000000,
                                   B01110000, B00110000, B01110000, B11000000, B01110011, B00000000, B01111100, B00000000, B01110000,
                                   B00111100, B11110001, B10011110, B01100110, B00110001, B10011001, B11100110, B00111100, B11110000,
                                   B00000011, B11100011, B00011000, B11000110, B01100011, B00001100, B00001100, B00011111, B11100000};  // 9
                                   
prog_uchar hell_font3[] PROGMEM = {B00000011, B00000000, B00001100, B00000001, B11111110, B00000000, B11000000, B00000011, B00000000,
                                   B00000011, B00000000, B00001100, B00000000, B00110000, B00000000, B11000000, B00000011, B00000000,
                                   B00000000, B00110000, B00000000, B11001110, B01110011, B00000000, B01111100, B00000000, B00000000,
                                   B01110000, B00000000, B01110000, B00000000, B01110000, B00000000, B01110000, B00000000, B01110000,
                                   B00111000, B00000000, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
                                   B00001100, B00000001, B11110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
                                   B00000000, B00111000, B00000011, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000,
                                   B00001100, B11000000, B00110011, B00000000, B11001100, B00000011, B00110000, B00001100, B11000000,
                                   B01110000, B00111000, B01110011, B10000000, B01111000, B00000000, B00000000, B00000000, B00000000,
                                   B00000000, B00000000, B00000000, B00000000, B01111000, B00000111, B00111000, B01110000, B00111000,
                                   B00000000, B00000000, B01110011, B10000001, B11001110, B00000000, B00000000, B00000000, B00000000,
                                   0, 0, 0, 0, 0, 0, 0, 0, 0};
                                   
#endif //FEATURE_HELL


//---------------------------------------------------------------------------------------------------------

void setup()
{
  
// setup pins

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
  key_state = 0;
  key_tx = 1;
  wpm = initial_speed_wpm;
  
  // setup default modes
  sidetone_mode = SIDETONE_ON;
  char_send_mode = CW;
  
  #ifdef DEBUG_HELL_TEST
  hell_test();
  #endif

  serial_baud_rate = default_serial_baud_rate;
  Serial.begin(serial_baud_rate);
   
  #ifdef SERIAL
  serial_uptime_stamp();
  Serial.print(F("K3NG Beacon Controller Version "));
  Serial.write(CODE_VERSION);
  Serial.println();
  #ifdef DEBUG_MEMORYCHECK
  memorycheck();
  #endif DEBUG_MEMORYCHECK
  #endif //SERIAL 
  
  delay(201);
}

void loop()
{ 
//  current_ptt_line = ptt_tx_1;
//  ptt_key();
//  sidetone_mode = SIDETONE_ON;
//  send_char('V',0);
//  send_char('V',0);
//  send_char('V',0);
//  ptt_unkey();
//  current_ptt_line = ptt_tx_2;
//  ptt_key();
//  sidetone_mode = SIDETONE_OFF;
//  send_char('V',0);
//  send_char('V',0);
//  send_char('V',0);
//  ptt_unkey();
//  
//  
//  current_ptt_line = ptt_tx_1;
//  ptt_key();
//  sidetone_mode = SIDETONE_ON;
//  send_char('D',0);
//  send_char('E',0);
//  ptt_unkey();
//  current_ptt_line = ptt_tx_2;
//  ptt_key();
//  sidetone_mode = SIDETONE_OFF;
//  send_char('D',0);
//  send_char('E',0);
//  ptt_unkey();
//  
//  current_ptt_line = ptt_tx_1;
//  ptt_key();
//  sidetone_mode = SIDETONE_ON;
//  send_char('K',0);
//  send_char('3',0);
//  send_char('N',0);
//  send_char('G',0);  
//  ptt_unkey();
//  current_ptt_line = ptt_tx_2;
//  ptt_key();
//  sidetone_mode = SIDETONE_OFF;
//  send_char('K',0);
//  send_char('3',0);
//  send_char('N',0);
//  send_char('G',0); 
//  ptt_unkey();
//  
//  current_ptt_line = ptt_tx_1;
//  ptt_key();
//  sidetone_mode = SIDETONE_ON;
//  send_char('F',0);
//  send_char('N',0);
//  send_char('2',0);
//  send_char('0',0);  
//  ptt_unkey();
//  current_ptt_line = ptt_tx_2;
//  ptt_key();
//  sidetone_mode = SIDETONE_OFF;
//  send_char('F',0);
//  send_char('N',0);
//  send_char('2',0);
//  send_char('0',0); 
//  ptt_unkey();  



  serial_status();

  send_char_to_txs('V');
  send_char_to_txs('V');
  send_char_to_txs('V');
  send_char_to_txs('D');
  send_char_to_txs('E');
  send_char_to_txs('K');
  send_char_to_txs('3');
  send_char_to_txs('N');
  send_char_to_txs('G');  
  send_char_to_txs('F');
  send_char_to_txs('N');
  send_char_to_txs('2');
  send_char_to_txs('0');  
  
  beacon_cycle_count++;
  
  millis_rollover_check();
  
  #ifdef FEATURE_SERIAL
  check_serial();
  #endif
}

// Subroutines --------------------------------------------------------------------------------------------

void send_char_to_txs(char cw_char) {
  
  current_ptt_line = ptt_tx_1;
  ptt_key();
  sidetone_mode = SIDETONE_ON;
  send_char(cw_char,0);
  ptt_unkey();
  current_ptt_line = ptt_tx_2;
  ptt_key();
  sidetone_mode = SIDETONE_OFF;
  send_char(cw_char,0);
  ptt_unkey();  
  
  
}

#ifdef FEATURE_HELL
void hell_test ()
{
  for (byte h = 65; h < (65+26); h++) { transmit_hell_char(h); }
  transmit_hell_char('0');
  transmit_hell_char('1');
  transmit_hell_char('2');
  transmit_hell_char('3');
  transmit_hell_char('4');  
  transmit_hell_char('5');
  transmit_hell_char('6');
  transmit_hell_char('7');
  transmit_hell_char('8');
  transmit_hell_char('9');
  transmit_hell_char('+');
  transmit_hell_char('-');
  transmit_hell_char('?');
  transmit_hell_char('/');
  transmit_hell_char('.');
  transmit_hell_char(','); 
  transmit_hell_char('‘');
  transmit_hell_char('='); 
  transmit_hell_char(')'); 
  transmit_hell_char('('); 
  transmit_hell_char(':');
}
#endif

//-------------------------------------------------------------------------------------------------------

#ifdef FEATURE_HELL
void transmit_hell_char (byte hellchar)
{

  // blank column
  for (byte w = 0; w < 14; w++) {
    transmit_hell_pixel(0);
  }


  
  if ((hellchar > 64) && (hellchar < 91)) {    // A - Z
    hellchar = ((hellchar - 65) * 9);
    transmit_hell_pixels(hell_font1, hellchar);
  } else {
    if ((hellchar > 47) && (hellchar < 58)) {  // 0 - 9
      hellchar = ((hellchar - 48) * 9); 
      transmit_hell_pixels(hell_font2, hellchar);
    } else {
      switch (hellchar) {
        case '+': hellchar = 0; break;
        case '-': hellchar = 1; break;
        case '?': hellchar = 2; break;
        case '/': hellchar = 3; break;
        case '.': hellchar = 4; break;
        case ',': hellchar = 5; break;
        case '‘': hellchar = 6; break;
        case '=': hellchar = 7; break;
        case ')': hellchar = 8; break;
        case '(': hellchar = 9; break;
        case ':': hellchar = 10; break;
        default : hellchar = 11; break;
      }
      hellchar = hellchar * 9;
      transmit_hell_pixels(hell_font3, hellchar);
      
    }
  }
  
  // blank column
  for (byte w = 0; w < 14; w++) {
    transmit_hell_pixel(0);
  }

}
#endif

//-------------------------------------------------------------------------------------------------------

#ifdef FEATURE_HELL
void transmit_hell_pixels (prog_uchar* hell_pixels, byte hellchar)
{
  
  for (byte x = 0; x < 9; x++) {
    for (int y = 7; y > -1; y--) {
      if ((x < 8) || ((x == 8) && (y > 1))) {  // drop the last 2 bits in byte 9
        if (bitRead(pgm_read_byte(hell_pixels + hellchar + x ),y)) {   
          transmit_hell_pixel(1);
        } else {
          transmit_hell_pixel(0);
        }
      }
    }
  }
  
}
#endif

//-------------------------------------------------------------------------------------------------------

#ifdef FEATURE_HELL
void transmit_hell_pixel (byte hellbit)
{
  if (hellbit) {
    tx_and_sidetone_key(1);
  } else {
    tx_and_sidetone_key(0);
  }
  delayMicroseconds(hell_pixel_microseconds);
}
#endif

//-------------------------------------------------------------------------------------------------------


void ptt_key()
{
  if (ptt_line_activated == 0) {   // if PTT is currently deactivated, bring it up and insert PTT lead time delay
    if (current_ptt_line) {
      digitalWrite (current_ptt_line, HIGH);
      delay(ptt_lead_time);
    }
    ptt_line_activated = 1;      
  }
  ptt_time = millis();
}

//-------------------------------------------------------------------------------------------------------
void ptt_unkey()
{
  if (ptt_line_activated) {
    if (current_ptt_line) {
      digitalWrite (current_ptt_line, LOW);
    }
    ptt_line_activated = 0;      
  }  
}

//-------------------------------------------------------------------------------------------------------

void send_dit()
{
  unsigned int character_wpm = wpm;
 
  being_sent = SENDING_DIT;
  tx_and_sidetone_key(1);
  loop_element_lengths((1.0*(float(weighting)/50)),0,character_wpm);  
  tx_and_sidetone_key(0);
  loop_element_lengths((2.0-(float(weighting)/50)),0,character_wpm);  
  being_sent = SENDING_NOTHING;
  //last_sending_type = sending_type;
  
}

//-------------------------------------------------------------------------------------------------------

void send_dah()
{  
  unsigned int character_wpm = wpm;
  
  being_sent = SENDING_DAH;
  tx_and_sidetone_key(1);
  loop_element_lengths((float(dah_to_dit_ratio/100.0)*(float(weighting)/50)),0,character_wpm);
  tx_and_sidetone_key(0);  
  loop_element_lengths((4.0-(3.0*(float(weighting)/50))),0,character_wpm);
  being_sent = SENDING_NOTHING;
  //last_sending_type = sending_type;
  

  
}

//-------------------------------------------------------------------------------------------------------

void tx_and_sidetone_key (int state)
{
  if ((state) && (key_state == 0)) {
    if (key_tx) {
      byte previous_ptt_line_activated = ptt_line_activated;
      ptt_key();      
      digitalWrite (tx_line, HIGH);
    }
    if (sidetone_mode == SIDETONE_ON){
      tone(sidetone_line, hz_sidetone);
    }
    key_state = 1;
  } else {
    if ((state == 0) && (key_state)) {
      if (key_tx) {
        digitalWrite (tx_line, LOW);
        ptt_key();
      }
      if (sidetone_mode == SIDETONE_ON) {
        noTone(sidetone_line);
      }
      key_state = 0;
    }          
  }
}  

//-------------------------------------------------------------------------------------------------------

void loop_element_lengths(float lengths, float additional_time_ms, int speed_wpm_in)
{
 
  if ((lengths == 0) or (lengths < 0)) {
    return;
  }
  
 float element_length;
  
 if (speed_mode == SPEED_NORMAL) {
   element_length = 1200/speed_wpm_in;
 } else {
   element_length = qrss_dit_length * 1000;
 }

 long endtime = millis() + long(element_length*lengths) + long(additional_time_ms);

 while ((millis() < endtime) && (millis() > 200)) {  // the second condition is to account for millis() rollover
   #ifdef SERIAL
   check_serial();
   #endif
 }  
}


//-------------------------------------------------------------------------------------------------------

void beep()
{
 tone(sidetone_line, hz_high_beep, 200); 
}

//-------------------------------------------------------------------------------------------------------

void boop()
{
  tone(sidetone_line, hz_low_beep);
  delay(100);
  noTone(sidetone_line);
}

//-------------------------------------------------------------------------------------------------------

void beep_boop()
{
  tone(sidetone_line, hz_high_beep);
  delay(100);
  tone(sidetone_line, hz_low_beep);
  delay(100);
  noTone(sidetone_line);
}

//-------------------------------------------------------------------------------------------------------

void boop_beep()
{
  tone(sidetone_line, hz_low_beep);
  delay(100);
  tone(sidetone_line, hz_high_beep);
  delay(100);
  noTone(sidetone_line);
}


//-------------------------------------------------------------------------------------------------------

void send_dits(int dits)
{
  for (;dits > 0;dits--) {
    send_dit();
  } 
  
}

//-------------------------------------------------------------------------------------------------------

void send_dahs(int dahs)
{
  for (;dahs > 0;dahs--) {
    send_dah();
  } 
  
}

//-------------------------------------------------------------------------------------------------------

void send_char(char cw_char, byte omit_letterspace)
{
  #ifdef DEBUG_SEND_CHAR
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
      case 'A': send_dit(); send_dah(); break;
      case 'B': send_dah(); send_dits(3); break;
      case 'C': send_dah(); send_dit(); send_dah(); send_dit(); break;
      case 'D': send_dah(); send_dits(2); break;
      case 'E': send_dit(); break;
      case 'F': send_dits(2); send_dah(); send_dit(); break;
      case 'G': send_dahs(2); send_dit(); break;
      case 'H': send_dits(4); break;
      case 'I': send_dits(2); break;
      case 'J': send_dit(); send_dahs(3); break;
      case 'K': send_dah(); send_dit(); send_dah(); break;
      case 'L': send_dit(); send_dah(); send_dits(2); break;
      case 'M': send_dahs(2); break;
      case 'N': send_dah(); send_dit(); break;
      case 'O': send_dahs(3); break;
      case 'P': send_dit(); send_dahs(2); send_dit(); break;
      case 'Q': send_dahs(2); send_dit(); send_dah(); break;
      case 'R': send_dit(); send_dah(); send_dit(); break;
      case 'S': send_dits(3); break;
      case 'T': send_dah(); break;
      case 'U': send_dits(2); send_dah(); break;    
      case 'V': send_dits(3); send_dah(); break;
      case 'W': send_dit(); send_dahs(2); break;
      case 'X': send_dah(); send_dits(2); send_dah(); break;
      case 'Y': send_dah(); send_dit(); send_dahs(2); break;
      case 'Z': send_dahs(2); send_dits(2); break;
          
      case '0': send_dahs(5); break;
      case '1': send_dit(); send_dahs(4); break;
      case '2': send_dits(2); send_dahs(3); break;
      case '3': send_dits(3); send_dahs(2); break;
      case '4': send_dits(4); send_dah(); break;
      case '5': send_dits(5); break;
      case '6': send_dah(); send_dits(4); break;
      case '7': send_dahs(2); send_dits(3); break;
      case '8': send_dahs(3); send_dits(2); break;
      case '9': send_dahs(4); send_dit(); break;
      
      case '=': send_dah(); send_dits(3); send_dah(); break;
      case '/': send_dah(); send_dits(2); send_dah(); send_dit(); break;
      case ' ': loop_element_lengths((length_wordspace-length_letterspace-2),0,wpm); break;
      case '*': send_dah(); send_dits(3); send_dah(); send_dit(); send_dah(); break;    // using asterisk for BK
      //case '&': send_dit(); loop_element_lengths(3); send_dits(3); break;
      case '.': send_dit(); send_dah(); send_dit(); send_dah(); send_dit(); send_dah(); break;
      case ',': send_dahs(2); send_dits(2); send_dahs(2); break;
      case '\'': send_dit(); send_dahs(4); send_dit(); break;                   // apostrophe
      case '!': send_dah(); send_dit(); send_dah(); send_dit(); send_dahs(2); break;
      case '(': send_dah(); send_dit(); send_dahs(2); send_dit(); break;
      case ')': send_dah(); send_dit(); send_dahs(2); send_dit(); send_dah(); break;
      case '&': send_dit(); send_dah(); send_dits(3); break;
      case ':': send_dahs(3); send_dits(3); break;
      case ';': send_dah(); send_dit(); send_dah(); send_dit(); send_dah(); send_dit(); break;
      case '+': send_dit(); send_dah(); send_dit(); send_dah(); send_dit(); break;
      case '-': send_dah(); send_dits(4); send_dah(); break;
      case '_': send_dits(2); send_dahs(2); send_dit(); send_dah(); break;
      case '"': send_dit(); send_dah(); send_dits(2); send_dah(); send_dit(); break;
      case '$': send_dits(3); send_dah(); send_dits(2); send_dah(); break;
      case '@': send_dit(); send_dahs(2); send_dit(); send_dah(); send_dit(); break;
      case '<': send_dit(); send_dah(); send_dit(); send_dah(); send_dit(); break;     // AR
      case '>': send_dits(3); send_dah(); send_dit(); send_dah(); break;               // SK
      case '\n': break;
      case '\r': break;
      case '|': loop_element_lengths(0.5,0,wpm); return; break;  
      default: send_dits(2); send_dahs(2); send_dits(2); break;
    }  
    if (omit_letterspace != OMIT_LETTERSPACE) {
      loop_element_lengths((length_letterspace-1),0,wpm); //this is minus one because send_dit and send_dah have a trailing element space
    }
  } else {
    #ifdef FEATURE_HELL
      transmit_hell_char(cw_char);
    #endif 
  }
  
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
#ifdef FEATURE_SERIAL
void serial_print(prog_uchar str[])
{
  char c;
  while((c = pgm_read_byte(str++))) {
    Serial.write(c);
  }
}
#endif

//-------------------------------------------------------------------------------------------------------

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
  

  
//  unsigned long uptime = (millis()/1000);
//  
//  Serial.println();
//  Serial.print(F("\n\rUptime: "));
//  
//  if (uptime > 86400) {
//    uptime = uptime / 86400;
//    Serial.print(uptime);
//    Serial.print(F(" Days\n\r"));
//  } else {
//    if (uptime > 3600) {
//      uptime = uptime / 3600;
//      Serial.print(uptime);
//      Serial.print(F(" Hours\n\r"));
//    } else {
//      if (uptime > 60) {
//        uptime = uptime / 60;
//        Serial.print(uptime);
//        Serial.print(F(" Minutes\n\r"));
//      } else {
//        Serial.print(uptime);
//        Serial.print(F(" Seconds\n\r"));
//      }
//    }
//  }
  


  serial_uptime_stamp();
  memorycheck();
  Serial.print(F(" beacon_cycle_count="));
  Serial.print(beacon_cycle_count);
  Serial.print(F(" millis="));
  Serial.print(millis());
  Serial.print(F(" millis_rollover="));
  Serial.println(millis_rollover);
  
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


