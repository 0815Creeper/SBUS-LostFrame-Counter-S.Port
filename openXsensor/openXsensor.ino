#include "oXs_config_basic.h"
#include "oXs_config_advanced.h"
#include "oXs_out_frsky.h"
#include "oXs_general.h"

#if  ! defined(PROTOCOL)
#error The parameter PROTOCOL in config_basic.h is not defined
#elif ! (PROTOCOL == FRSKY_SPORT)
#error The parameter PROTOCOL in config_basic.h is NOT valid
#endif

extern unsigned long micros( void ) ;
extern unsigned long millis( void ) ;

// to read SPORT (for Frsky protocol
extern uint8_t  volatile TxData[8] ;
extern uint8_t  volatile TxDataIdx ;

#define FORCE_INDIRECT(ptr) __asm__ __volatile__ ("" : "=e" (ptr) : "0" (ptr))

OXS_OUT oXs_Out(PIN_SERIALTX);

//******************************************* Setup () *******************************************************

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial.begin(100000, SERIAL_8E2);
  oXs_Out.setup();
  //SBUS sync: discard old frames
  while (!Serial.available()) {}
  while (Serial.read() != -1) {
    while (Serial.read() != -1) {}
    delay(1);
  }
} // ******************** end of Setup *****************************************************


//*******************************************************************************************
//                                Main loop                                               ***
//*******************************************************************************************
void loop() {
  oXs_Out.sendData(); //SBUS analysis is done in this function too
}          // ****************   end of main loop *************************************
