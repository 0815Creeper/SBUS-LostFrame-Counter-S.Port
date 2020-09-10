#ifndef OXS_CONFIG_ADVANCED_h
#define OXS_CONFIG_ADVANCED_h

#define PIN_SERIALTX      4                 // The pin which transmits the serial data to the telemetry receiver. pin 4 or 2 are possible

// ***** 1.2 - SPORT_SENSOR_ID used for Frsky Sport protocol  *****   See list of available values in oXs_config_descripion.h 
#define         DATA_ID_LF  0x2F  // = sensor used for Lost Frames and Lost Frames per 3 seconds
#define         DATA_ID_B   0x01  //          unused
#define         DATA_ID_C   0x02  //          unused
#define         DATA_ID_D   0x03  //          unused
#define         DATA_ID_E   0x04  //          unused
#define         DATA_ID_F   0x07  //          unused
#define         DATA_ID_TX  0x08  //          unused (to read data sent by Tx)


#include <Arduino.h>
struct ONE_MEASUREMENT {
  uint8_t available ;
  int32_t value ;
} ;

#endif// End define OXS_CONFIG_ADVANCED_h
