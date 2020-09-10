// For FRSKY
#ifndef OXS_OUT_FRSKY_h
#define OXS_OUT_FRSKY_h

#include "oXs_config_basic.h"
#include "oXs_config_advanced.h"
#include "oXs_general.h"
//#include "Converter.c"
// this file is used only for FRSKY
#if defined(PROTOCOL) && PROTOCOL == FRSKY_SPORT

// FrSky new DATA IDs (2 bytes) (copied from openTX telemetry/frsky_sport.cpp on 11 jul 2014) // those values are not used directly but bits 4 up to 11 are stored in an array in oXs_out_frsky.cpp 
#define ALT_FIRST_ID            0x0100
#define ALT_LAST_ID             0x010f
#define VARIO_FIRST_ID          0x0110
#define VARIO_LAST_ID           0x011f
#define CURR_FIRST_ID           0x0200
#define CURR_LAST_ID            0x020f
#define VFAS_FIRST_ID           0x0210
#define VFAS_LAST_ID            0x021f
#define CELLS_FIRST_ID          0x0300
#define CELLS_SECOND_ID         0x0301
#define CELLS_THIRD_ID          0x0302
#define CELLS_LAST_ID           0x030f
#define T1_FIRST_ID             0x0400
#define T1_LAST_ID              0x040f
#define T2_FIRST_ID             0x0410
#define T2_LAST_ID              0x041f
#define RPM_FIRST_ID            0x0500
#define RPM_LAST_ID             0x050f
#define FUEL_FIRST_ID           0x0600
#define FUEL_LAST_ID            0x060f
#define ACCX_FIRST_ID           0x0700
#define ACCX_LAST_ID            0x070f
#define ACCY_FIRST_ID           0x0710      
#define ACCY_LAST_ID            0x071f
#define ACCZ_FIRST_ID           0x0720
#define ACCZ_LAST_ID            0x072f
#define GPS_LONG_LATI_FIRST_ID  0x0800
#define GPS_LONG_LATI_LAST_ID   0x080f
#define GPS_ALT_FIRST_ID        0x0820
#define GPS_ALT_LAST_ID         0x082f
#define GPS_SPEED_FIRST_ID      0x0830
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840
#define GPS_COURS_LAST_ID       0x084f
#define GPS_TIME_DATE_FIRST_ID  0x0850
#define GPS_TIME_DATE_LAST_ID   0x085f
#define A3_FIRST_ID             0x0900
#define A3_LAST_ID              0x090f
#define A4_FIRST_ID             0x0910
#define A4_LAST_ID              0x091f
#define AIR_SPEED_FIRST_ID      0x0a00
#define AIR_SPEED_LAST_ID       0x0a0f
#define RSSI_ID                 0xf101  // please do not use this code because it is already used by the receiver
#define ADC1_ID                 0xf102  // please do not use this code because it is already used by the receiver
#define ADC2_ID                 0xf103
#define BATT_ID                 0xf104
#define SWR_ID                  0xf105  // please do not use this code because it is already used by the receiver
// End of list of all telemetry fields supported by SPORT  (defined by Frsky)   
 
/***************************************************************************************/
/* Transmission status                                                                 */ 
/***************************************************************************************/
#define TO_LOAD     0
#define LOADED      1
#define SENDING     2
#define SEND        3


class OXS_OUT {
  public:
    OXS_OUT(uint8_t pinTx);
 //   uint8_t currentValueType ; //e.g. = ALTIMETER, VERTICAL_SPEED, = field_Id to transmit  
    void setup();
    void sendData();
    void dbg();
  private:
// used by both protocols  
    uint8_t _pinTx;
 
    void sendSportData() ;
    uint8_t readStatusValue( uint8_t currentValueType) ;
    void loadSportValueToSend(  uint8_t ValueTypeToLoad) ;
    uint8_t nextFieldToSend(  uint8_t indexField) ;
    void FrSkySportSensorGpsSend() ;
};


// used by FRSKY_SPORT protocol
void setSportNewData( uint16_t id, uint32_t value ) ;
void initSportUart() ;
void initMeasurement() ;

// used in all protocol   // not sure it is used here
extern volatile bool RpmSet ;
extern volatile uint16_t RpmValue ;

extern volatile bool sportAvailable ;

uint32_t micros( void ) ;
uint32_t millis( void ) ;

// UART's state.
#define   IDLE               0        // Idle state, both transmit and receive possible.
#define   TRANSMIT           1        // Transmitting byte.
#define   TRANSMIT_STOP_BIT  2        // Transmitting stop bit.
#define   RECEIVE            3        // Receiving byte.
#define    TxPENDING          4
#define   WAITING            5

// 57600 = Desired baudrate for Sport protocol = 17 micro sec per bit.
// 9600   =  Desired baudrate for Hub protocol
//This section chooses the correct timer values for the Sport protocol = 57600 baud.
// Assumes a 16MHz clock
//#define TICKS2COUNT         278  // Ticks between two bits.
//#define TICKS2WAITONE       278  // Wait one bit period.
//#define TICKS2WAITONE_HALF  416  // Wait one and a half bit period.
  #if F_CPU == 20000000L   // 20MHz clock 
    // Sinan: Not tested                                                     
    #define TICKS2COUNTSPORT         348  // Ticks between two bits.
    #define TICKS2WAITONESPORT       348  // Wait one bit period.
    #define TICKS2WAITONE_HALFSPORT  520    // Wait one and a half bit period.
  #elif F_CPU == 16000000L  // 16MHz clock                                                  
    #define TICKS2COUNTSPORT         278  // Ticks between two bits.
    #define TICKS2WAITONESPORT       278  // Wait one bit period.
    #define TICKS2WAITONE_HALFSPORT  416    // Wait one and a half bit period.
  #elif F_CPU == 8000000L   // 8MHz clock
    // Assumes a 8MHz clock                                                   
    #define TICKS2COUNTSPORT         139  // Ticks between two bits.
    #define TICKS2WAITONESPORT       139  // Wait one bit period.
    #define TICKS2WAITONE_HALFSPORT  208    // Wait one and a half bit period.
  #else
    #error Unsupported clock speed
  #endif

//This section chooses the correct timer values for Hub protocol = 9600 baud.
// Assumes a 16MHz clock
//#define TICKS2COUNT         (278*6)  // Ticks between two bits.
//#define TICKS2WAITONE       (278*6)  // Wait one bit period.
//#define TICKS2WAITONE_HALF  (416*6)  // Wait one and a half bit period.
  #if F_CPU == 20000000L     // 20MHz clock                                                  
    // Sinan: Not tested
    #define TICKS2COUNTHUB         (348*6)  // Ticks between two bits.
    #define TICKS2WAITONEHUB       (348*6)  // Wait one bit period.
    #define TICKS2WAITONE_HALFHUB  (520*6)    // Wait one and a half bit period.
  #elif F_CPU == 16000000L   // 16MHz clock                                                  
    #define TICKS2COUNTHUB         (278*6)  // Ticks between two bits.
    #define TICKS2WAITONEHUB       (278*6)  // Wait one bit period.
    #define TICKS2WAITONE_HALFHUB  (416*6)    // Wait one and a half bit period.
  #elif F_CPU == 8000000L    // 8MHz clock                                                   
    #define TICKS2COUNTHUB         (139*6)  // Ticks between two bits.
    #define TICKS2WAITONEHUB       (139*6)  // Wait one bit period.
    #define TICKS2WAITONE_HALFHUB  (208*6)    // Wait one and a half bit period.
  #else
    #error Unsupported clock speed
  #endif

//#define INTERRUPT_EXEC_CYCL   90       // Cycles to execute interrupt routines from interrupt.
//#define INTERRUPT_EARLY_BIAS  32       // Cycles to allow of other interrupts.
// INTERRUPT_EARLY_BIAS is to bias the sample point a bit early in case
// the Timer 0 interrupt (5.5uS) delays the start bit detection
  #if F_CPU == 20000000L     // 20MHz clock                                                  
    #define INTERRUPT_EXEC_CYCL   112       // Cycles to execute interrupt routines from interrupt.
    #define INTERRUPT_EARLY_BIAS  40       // Cycles to allow of other interrupts.
  #elif F_CPU == 16000000L   // 16MHz clock                                                  
    #define INTERRUPT_EXEC_CYCL   90       // Cycles to execute interrupt routines from interrupt.
    #define INTERRUPT_EARLY_BIAS  32       // Cycles to allow of other interrupts.
  #elif F_CPU == 8000000L    // 8MHz clock                                                   
    #define INTERRUPT_EXEC_CYCL   90       // Cycles to execute interrupt routines from interrupt.
    #define INTERRUPT_EARLY_BIAS  32       // Cycles to allow of other interrupts.
  #else
    #error Unsupported clock speed
  #endif

    #define INTERRUPT_ENTRY_TRANSMIT  59   // Cycles in ISR before sending first bit from first byte; Without this correction, first bit is sent 7.4 usec to late at 8 Mhz (so it takes 59 cycles = 7.4 usec * 8)
    #define INTERRUPT_BETWEEN_TRANSMIT  64   // Cycles in ISR before sending first bit from 2d, 3rd... bytes; Without this correction, first bit is sent 4 usec to late at 16 Mhz (so it takes 64 cycles = 4 usec * 16)
    
// this section define some delays used in Aserial; values can be used by any protocol
  #if F_CPU == 20000000L     // 20MHz clock                                                  
    #define DELAY_4000  ((uint16_t)4000.0 * 20.0 /16.0 )
    #define DELAY_3500  ((uint16_t)3500.0 * 20.0 /16.0 )    
    #define DELAY_2000  ((uint16_t)2000.0 * 20.0 /16.0 )
    #define DELAY_1600  ((uint16_t)1600.0 * 20.0 /16.0 )    
    #define DELAY_400  ((uint16_t)400.0 * 20.0 /16.0 )
    #define DELAY_100  ((uint16_t)100.0 * 20.0 /16.0 )
  #elif F_CPU == 16000000L   // 16MHz clock                                                  
    #define DELAY_4000 ((uint16_t) (3999L * 16) )     
    #define DELAY_3500 ((uint16_t) (3500L * 16) )         
    #define DELAY_2000 ((uint16_t) (2000L * 16) )     
    #define DELAY_1600 ((uint16_t) (1600L * 16) )     
    #define DELAY_400 ((uint16_t) (400 * 16) )     
    #define DELAY_100 ((uint16_t) (100 * 16) )     
  #elif F_CPU == 8000000L    // 8MHz clock                                                   
    #define  DELAY_4000 ((uint16_t)4000L * 8 )
    #define  DELAY_3500 ((uint16_t)3500L * 8 )    
    #define  DELAY_2000 ((uint16_t)2000 * 8 )
    #define  DELAY_1600 ((uint16_t)1600 * 8 )    
    #define  DELAY_400 ((uint16_t)400 * 8 )
    #define  DELAY_100 ((uint16_t)100 * 8 )    
  #else
    #error Unsupported clock speed
  #endif

#define TCCR             TCCR1A             //!< Timer/Counter Control Register
#define TCCR_P           TCCR1B             //!< Timer/Counter Control (Prescaler) Register
#define OCR              OCR1A              //!< Output Compare Register
#define EXT_IFR          EIFR               //!< External Interrupt Flag Register
#define EXT_ICR          EICRA              //!< External Interrupt Control Register

#define TRXDDR  DDRD
#define TRXPORT PORTD
#define TRXPIN  PIND

#define SET_TX_PIN( )    ( TRXPORT |= ( 1 << PIN_SERIALTX ) )
#define CLEAR_TX_PIN( )  ( TRXPORT &= ~( 1 << PIN_SERIALTX ) )
#define GET_RX_PIN( )    ( TRXPIN & ( 1 << PIN_SERIALTX ) )

//******************* End of part used for handling of UART with Receiver

#endif // Enf of ndef MULTIPLEX

#endif // OXS_OUT_h
