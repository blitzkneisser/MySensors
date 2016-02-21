/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik EKblad
 * 
 * DESCRIPTION
 * Example sketch showing how to control ir devices
 * An IR LED must be connected to Arduino PWM pin 3.
 * An optional ir receiver can be connected to PWM pin 8. 
 * All receied ir signals will be sent to gateway device stored in VAR_1.
 * When binary light on is clicked - sketch will send volume up ir command
 * When binary light off is clicked - sketch will send volume down ir command
 * http://www.mysensors.org/build/ir
 */

#include <MySensor.h>
#include <SPI.h>
#include <IRLib.h>
#include <avr/pgmspace.h>

int RECV_PIN = 8;

#define CHILD_1  3  // childId

IRsend irsend;
IRrecv irrecv(RECV_PIN);
IRdecode decoder;
IRdecodeHash Hash_Decoder;

//As some remotes could have different hashes for the same button (when they are not good received or other reasons)
//we only let the following hashes go through

#define MYCODES_NUM 6
#define MYCODES_LENGTH 24
const unsigned long myHashes[MYCODES_NUM] = {
  0x31538D76,  // Licht an
  0x6DEB31B6,  // Licht aus
  0x8A8F5F86,  // Stufe0
  0xF3E5FAF1,  // Stufe1 
  0x87D57E46,  // Stufe2
  0x9F5D826E,  // Stufe3
};
/*
const unsigned int myCodes[MYCODES_NUM][MYCODES_LENGTH] PROGMEM ={
  {0x19,0x9,0x18,0x9,0x8,0x19,0x8,0x1A,0x8,0x19,0x8,0x19,0x9,0x19,0x8,0x19,0x19,0x9,0x8,0x19,0x8,0x19,0x9,0x0},  //Licht an
  {0x19,0x8,0x1A,0x8,0x8,0x19,0x9,0x18,0x9,0x19,0x8,0x19,0x1A,0x7,0x9,0x19,0x9,0x18,0x9,0x19,0x8,0x19,0x9,0x0},  //Licht aus
  {0x18,0x9,0x18,0x9,0x8,0x1A,0x7,0x1A,0x8,0x19,0x8,0x1A,0x8,0x19,0x19,0x9,0x7,0x1A,0x8,0x19,0x8,0x1A,0x7,0x0},  //Stufe0
  {0x19,0x9,0x18,0x9,0x8,0x1A,0x7,0x1A,0x8,0x19,0x8,0x1A,0x7,0x1A,0x8,0x19,0x8,0x1A,0x8,0x19,0x8,0x1A,0x18,0x0}, //Stufe1
  {0x19,0x9,0x18,0x9,0x8,0x19,0x8,0x1A,0x8,0x19,0x8,0x1A,0x7,0x1A,0x8,0x19,0x8,0x1A,0x18,0x9,0x8,0x1A,0x7,0x0},  //Stufe2
  {0x18,0x9,0x19,0x9,0x7,0x1A,0x8,0x19,0x8,0x1A,0x18,0x9,0x8,0x1A,0x7,0x1A,0x8,0x19,0x8,0x1A,0x18,0x9,0x18,0x0}  //Stufe3
};
*/
const unsigned int myCodes[MYCODES_NUM][MYCODES_LENGTH] PROGMEM ={
{0x44C,0x226,0x47E,0x226,0x12C,0x546,0x12C,0x578,0xFA,0x578,0x12C,0x546,0x12C,0x578,0xFA,0x578,0x47E,0x226,0xFA,0x578,0x12C,0x546,0x12C,0x0},
{0x4B0,0x226,0x44C,0x226,0x12C,0x546,0x12C,0x546,0x12C,0x546,0x15E,0x546,0x47E,0x226,0x12C,0x546,0x12C,0x546,0x12C,0x546,0x15E,0x546,0x12C,0x0},
{0x4B0,0x1F4,0x47E,0x1F4,0x15E,0x514,0x15E,0x546,0x15E,0x514,0x15E,0x546,0x12C,0x546,0x47E,0x1F4,0x15E,0x546,0x15E,0x514,0x15E,0x546,0x12C,0x0},
{0x47E,0x226,0x44C,0x226,0x12C,0x578,0xFA,0x578,0x12C,0x578,0xFA,0x578,0x12C,0x546,0x12C,0x578,0xFA,0x578,0x12C,0x546,0x12C,0x578,0x44C,0x0},
{0x47E,0x226,0x44C,0x226,0x12C,0x546,0x12C,0x578,0xFA,0x578,0x12C,0x578,0xFA,0x578,0x12C,0x546,0x12C,0x578,0x44C,0x226,0x12C,0x546,0x12C,0x0},
{0x44C,0x226,0x4B0,0x1F4,0xFA,0x578,0x12C,0x546,0x12C,0x578,0x44C,0x226,0x12C,0x578,0xFA,0x578,0x12C,0x546,0x12C,0x578,0x44C,0x226,0x47E,0x0}
};

int idx; // has index in table

//decode_results results;
MySensor gw;
MyMessage msgir(CHILD_1, V_IR_RECEIVE);

void setup()  
{  
  irrecv.enableIRIn(); // Start the ir receiver
  gw.begin(incomingMessage,AUTO,true);

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("IR Transceiver", "1.1");

  // Register a sensors to gw. Use binary light for test purposes.
  gw.present(CHILD_1, S_IR);

  Serial.println("Ready to receive...");
}


void loop() 
{
  gw.process();
  if (irrecv.GetResults(&decoder)) {
    Hash_Decoder.copyBuf(&decoder);//copy the results to the hash decoder
    Hash_Decoder.decode();
    if (!FindHashInOurTable(Hash_Decoder.hash, &idx)) {
      Serial.print("Unknown IR hash received: ");
      Serial.println(Hash_Decoder.hash, HEX);
    }
    else
    {
      Serial.print("Known IR hash received: ");
      Serial.println(Hash_Decoder.hash, HEX);
      gw.send(msgir.set(Hash_Decoder.hash));
    }    
    delay(1000);
    irrecv.resume(); 
    }
}

bool FindHashInOurTable(const unsigned long hash, int * idx)
{
  for (int ii=0; ii<MYCODES_NUM; ii++)
  {
    if (myHashes[ii]==hash)
    {
      * idx = ii; // store index value
      return true;   
    }      
  }
  return false;
}

void incomingMessage(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type==V_IR_SEND) {
     unsigned long ir_code=(unsigned long)message.getLong();
     Serial.print("Gateway received code: ");
     Serial.println(ir_code, HEX);
     if (FindHashInOurTable(ir_code, &idx)) {
      unsigned int tempdata[24];
      Serial.print("Data to send(");
      Serial.print(idx, HEX);
      Serial.print("): {0x");
      for (int i=0;i<24;i++)
        {
          tempdata[i] = pgm_read_word_near(myCodes[idx] + i);
          Serial.print(tempdata[i], HEX);
          if (i < 23)
            Serial.print(",0x");
          else
            Serial.println("}");
        }
        //irsend.IRsendRaw::send((unsigned int *)myCodes[idx], 24, 38);
        for (int j=1;j<=4;j++) {
          irsend.IRsendRaw::send((unsigned int *)tempdata, 24, 38);
          delay(15); //wait 15ms
        }
        irrecv.enableIRIn(); 
     }
  }
}
    
// Dumps out the decode_results structure.
// Call this after IRrecv::decode()
// void * to work around compiler issue
//void dump(void *v) {
//  decode_results *results = (decode_results *)v
/*void dump(decode_results *results) {
  int count = results->rawlen;
  if (results->decode_type == UNKNOWN) {
    Serial.print("Unknown encoding: ");
  } 
  else if (results->decode_type == NEC) {
    Serial.print("Decoded NEC: ");
  } 
  else if (results->decode_type == SONY) {
    Serial.print("Decoded SONY: ");
  } 
  else if (results->decode_type == RC5) {
    Serial.print("Decoded RC5: ");
  } 
  else if (results->decode_type == RC6) {
    Serial.print("Decoded RC6: ");
  }
  else if (results->decode_type == PANASONIC) {   
    Serial.print("Decoded PANASONIC - Address: ");
    Serial.print(results->panasonicAddress,HEX);
    Serial.print(" Value: ");
  }
  else if (results->decode_type == JVC) {
     Serial.print("Decoded JVC: ");
  }
  Serial.print(results->value, HEX);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
  Serial.print("Raw (");
  Serial.print(count, DEC);
  Serial.print("): ");

  for (int i = 0; i < count; i++) {
    if ((i % 2) == 1) {
      Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
    } 
    else {
      Serial.print(-(int)results->rawbuf[i]*USECPERTICK, DEC);
    }
    Serial.print(" ");
  }
  Serial.println("");
}
*/
