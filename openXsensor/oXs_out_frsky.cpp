// file for FRSKY telemetry (SPORT and HUB)

#include "oXs_out_frsky.h"
#if defined(PROTOCOL) && (PROTOCOL == FRSKY_SPORT)  //if Frsky protocol is used

extern unsigned long micros( void ) ;
extern unsigned long millis( void ) ;
extern void delay(unsigned long ms) ;

//used only by Sport protocol
extern uint8_t  volatile  sportData[7] ;
uint8_t  volatile TxData[8] ;
uint8_t  volatile TxDataIdx ;
uint8_t rxStuff ;
extern uint8_t LastRx ;
static volatile uint8_t prevLastRx ;           // just for testing

uint8_t volatile sportDataLock ;
extern uint8_t volatile sendStatus ;

//SBUS data
uint32_t  volatile SBUS_lost_Frames_total = 0;
uint16_t  volatile SBUS_lost_Frames_3sec_number = 0;
uint8_t SBUS_FL_RING_BUFFER[333];
uint16_t SBUS_FL_RING_BUFFER_IDX=0;
uint8_t flags;
uint8_t sbus_data[22];
uint16_t channel_data[18];

//Used by both protocols
volatile bool sportAvailable = false ;
//int fieldContainsData[][5]  = {  SETUP_FRSKY_DATA_TO_SEND } ; // contains the set up of field to be transmitted
//int numberOfFields = sizeof(fieldContainsData) / sizeof(fieldContainsData[0]) ;
//static uint16_t convertToSportId[15] = { FRSKY_SPORT_ID } ; // this array is used to convert an index inside fieldContainsData[][0] into the SPORT field Id (or defaultfield)
//static uint8_t convertToHubId[15] = { FRSKY_HUB_ID } ; //// this array is used to convert an index inside fieldContainsData[][0] into the Hub field Id (or defaultfield)
//static uint8_t currentFieldToSend = 0 ;
extern volatile uint8_t state ;                  //!< Holds the state of the UART.


OXS_OUT::OXS_OUT(uint8_t pinTx)
{
  _pinTx = pinTx ;
} // end constructor


// **************** Setup the FRSky OutputLib *********************
void OXS_OUT::setup() {
  // here we look if sport is available or not; when available, sport protocol must be activated otherwise hub protocol
  //initilalise PORT
  TRXDDR &= ~( 1 << PIN_SERIALTX ) ;       // PIN is input, tri-stated.
  TRXPORT &= ~( 1 << PIN_SERIALTX ) ;      // PIN is input, tri-stated.

  initMeasurement() ;
  initSportUart(  ) ;
  sportAvailable = true ;     // force the SPORT protocol
  // Activate pin change interupt 2 on Tx pin
  #if PIN_SERIALTX == 4
  PCMSK2 |= 0x10 ;			            // IO4 (PD4) on Arduini mini
  #elif PIN_SERIALTX == 2
  PCMSK2 |= 0x04 ;                  // IO2 (PD2) on Arduini mini
  #else
  #error "This PIN is not supported"
  #endif
  sportAvailable = true ;
  initMeasurement() ;
  initSportUart() ;
}  // end of setup

struct ONE_MEASUREMENT Converter_SBUS_3secLost = {false, 0};
struct ONE_MEASUREMENT Converter_SBUS_totalLost = {false, 0};

void Converter_put(ONE_MEASUREMENT * om, int32_t v) {
  om->available = true;
  om->value = v;
}

void Converter_convert() {
  while (Serial.available() >= 25) {
    if(Serial.read()==0x0F){
      Serial.readBytes(sbus_data, 22);
      flags = Serial.read();
      if ((!(flags&0b11110000))&&Serial.read() == 0) {
        int db = 0;
        int di = 0;
        for (int i = 0; i < 16; i++) {
          channel_data[i] = 0;
          for (int b = 0; b < 11; b++) {
            channel_data[i] |= (((sbus_data[di]) >> db) & 1) << b;
            db++;
            if (db == 8) {
              db = 0;
              di++;
            }
          }
        }
        channel_data[16] = (flags & 0b1) ? 2000 : 1000;
        channel_data[17] = (flags & 0b10) ? 2000 : 1000;
        SBUS_lost_Frames_total += ((flags>>2) & 0b1);
        SBUS_FL_RING_BUFFER[SBUS_FL_RING_BUFFER_IDX++]=((flags>>2) & 0b1);
        if(SBUS_FL_RING_BUFFER_IDX==333){
          SBUS_FL_RING_BUFFER_IDX=0;
        }
        SBUS_lost_Frames_3sec_number = 0;
        for(int i=0;i<333;i++){
          SBUS_lost_Frames_3sec_number+=SBUS_FL_RING_BUFFER[i];
        }
        Converter_put(&Converter_SBUS_3secLost, (uint16_t)(((float)SBUS_lost_Frames_3sec_number)/3.33));
        Converter_put(&Converter_SBUS_totalLost, SBUS_lost_Frames_total);
      }
    }
  }
}

void OXS_OUT::sendData() {
  Converter_convert();
  sendSportData() ;
}

volatile uint8_t idToReply ;                                                     // each bit (0..5) reports (set to 1) if oXs has to reply to a pooling on a specific Id (this has been added because oXs has to reply with all 0x00 when he has not yet data available
volatile uint8_t frskyStatus = 0x3F  ;                                                   //Status of SPORT protocol saying if data is to load in work field (bit 0...5 are used for the 6 sensorId), initially all data are to be loaded
uint8_t currFieldIdx[6] = { 0 , 2, 5 , 10 , 15 , 19 } ;                          // per sensor, say which field has been loaded the last time (so next time, we have to search from the next one)
const uint8_t fieldMinIdx[7]  = { 0 , 2, 5 , 10 , 15 , 19 , 22 } ;                     // per sensor, say the first field index ; there is one entry more in the array to know the last index
const uint8_t fieldId[22] = {
  0x60 , //0
  0x61 , //1
  0x60 , //2
  0x50 , //3
  0x10 , //4
  0x21 , //5
  0x50 , //6
  0x60 , //7
  0x83 , //8
  0x84 , //9
  0x80 , //10
  0x80 , //11
  0x82 , //12
  0x50 , //13
  0x84 , //14
  0x84 , //15
  0x40 , //16
  0x50 , //17
  0x00 , //18
  0x84 , //19
  0x50 , //20
  0x00   //21
} ; //fieldID to send to Tx (first 4 bit to shift 4 bit left, second 4 bit to stay and be "subID") Different from Standart openXsensor!!! See //MODIFIED!!!! in Timer ISR

struct ONE_MEASUREMENT *p_measurements[22] ;      // array of 22 pointers (each pointer point to a structure containing a byte saying if a value is available and to the value.
int32_t dataValue[6] ;
uint8_t dataId[6] ;
uint8_t sensorSeq  ;
uint8_t sensorIsr  ;
struct ONE_MEASUREMENT no_data = { 0, 0 } ;

void initMeasurement() {
  idToReply |= 0x01 ;/*
  idToReply |= 0x02 ;
  idToReply |= 0x04 ;
  idToReply |= 0x08 ;
  idToReply |= 0x10 ;
  idToReply |= 0x20 ;*/
  p_measurements[0] = &Converter_SBUS_3secLost;
  p_measurements[1] = &Converter_SBUS_totalLost ;
  p_measurements[2] = &no_data  ;
  p_measurements[3] = &no_data  ;
  p_measurements[4] = &no_data  ;
  p_measurements[5] = &no_data  ;
  p_measurements[6] = &no_data  ;
  p_measurements[7] = &no_data  ;
  p_measurements[8] = &no_data  ;
  p_measurements[9] = &no_data  ;
  p_measurements[10] = &no_data ;
  p_measurements[11] = &no_data ;
  p_measurements[12] = &no_data ;
  p_measurements[13] = &no_data ;
  p_measurements[14] = &no_data ;
  p_measurements[15] = &no_data ;
  p_measurements[16] = &no_data ;
  p_measurements[17] = &no_data ;
  p_measurements[18] = &no_data ;
  p_measurements[19] = &no_data ;
  p_measurements[20] = &no_data ;
  p_measurements[21] = &no_data ;

}

void OXS_OUT::sendSportData()
{

  if ( frskyStatus ) {                                                                                  // if at least one data has to be loaded
    for (uint8_t sensorSeq = 0 ; sensorSeq < 6 ; sensorSeq++ ) {                                        // for each sensor (currently 6)
      if ( frskyStatus & (1 << sensorSeq ) )  {                          //if frskyStatus says that a data must be loaded for this sensor
        uint8_t currFieldIdx_ = currFieldIdx[sensorSeq] ;                // retrieve the last field being loaded for this sensor
        for (uint8_t iCount = fieldMinIdx[sensorSeq] ; iCount < fieldMinIdx[sensorSeq + 1] ; iCount++ ) {      // we will not seach more than the number of fields for the selected sensor
          currFieldIdx_++ ;                                                                          // search with next field

          if ( currFieldIdx_ >= fieldMinIdx[sensorSeq + 1] ) currFieldIdx_ = fieldMinIdx[sensorSeq] ;      // if overlap within sensor range, set idx to first idx for this sensorSeq

          if ( p_measurements[currFieldIdx_]->available  ) {

            // if data of current index of sensor is available
            p_measurements[currFieldIdx_]->available = 0 ;                                                         // mark the data as not available
            dataValue[sensorSeq] =  p_measurements[currFieldIdx_]->value ;                                         // store the value in a buffer
            dataId[sensorSeq] = fieldId[currFieldIdx_] ;   // mark the data from this sensor as available
            uint8_t oReg = SREG ; // save status register
            cli() ;
            frskyStatus &= ~(1 << sensorSeq) ;                                              // says that data is loaded by resetting one bit
            SREG = oReg ; // restore the status register
            break ;                                                                         // exit inner for
          }     /*else if(       currFieldIdx_==0){
                  (&Converter_relativeAlt)->available = 0 ;
                  dataValue[sensorSeq] =  (&Converter_relativeAlt)->value ;                                         // store the value in a buffer
                  dataId[sensorSeq] = fieldId[currFieldIdx_] ;                                                   // mark the data from this sensor as available
                  uint8_t oReg = SREG ; // save status register
                  cli() ;
                  frskyStatus &= ~(1<< sensorSeq) ;                                               // says that data is loaded by resetting one bit
                  SREG = oReg ; // restore the status register
                }*/
        }
        currFieldIdx[sensorSeq] = currFieldIdx_   ;                                            // save currentFieldIdx for this
      }
    } // End for one sensorSeq
  }   // End of if (frskystatus)
  else{
    digitalWrite(13,LOW);
  }
}


#define FORCE_INDIRECT(ptr) __asm__ __volatile__ ("" : "=e" (ptr) : "0" (ptr))

volatile uint8_t state ;                  //!< Holds the state of the UART.
static volatile unsigned char SwUartTXData ;     //!< Data to be transmitted.
static volatile unsigned char SwUartTXBitCount ; //!< TX bit counter.
static volatile uint8_t SwUartRXData ;           //!< Storage for received bits.
static volatile uint8_t SwUartRXBitCount ;       //!< RX bit counter.
static volatile uint8_t TxCount ;

volatile uint8_t debugUartRx ;

volatile uint8_t ppmInterrupted ;
uint8_t sensorId ;
uint8_t ByteStuffByte = 0 ;
// only for Sport
uint8_t LastRx ;
uint8_t TxSportData[7] ;
uint16_t Crc ;
uint8_t  volatile  sportData[7] ;
uint8_t volatile sendStatus ;
//uint8_t volatile gpsSendStatus ;
//uint8_t volatile gpsSportDataLock ;
//uint8_t volatile gpsSportData[7] ;
uint8_t currentSensorId ; // save the sensor id being received and on which oXs will reply (can be the main sensor id or GPS sensor id)


ISR(TIMER1_COMPA_vect)
{
  if ( sportAvailable ) {    // ++++++++ here only for SPORT protocol ++++++++++++++++++++++++++++++++++
    switch (state)
    {
      // Transmit Byte.
      case TRANSMIT :   // Output the TX buffer in SPORT ************ we are sending each bit of data
        if ( SwUartTXBitCount < 8 )
        {
          if ( SwUartTXData & 0x01 ) // If the LSB of the TX buffer is 1:
          {
            CLEAR_TX_PIN() ;                    // Send a logic 1 on the TX_PIN.
          }
          else
          { // Otherwise:
            SET_TX_PIN() ;                      // Send a logic 0 on the TX_PIN.
          }
          SwUartTXData = SwUartTXData >> 1 ;    // Bitshift the TX buffer and
          SwUartTXBitCount += 1 ;               // increment TX bit counter.
        }
        else    //Send stop bit.
        {
          CLEAR_TX_PIN();                         // Output a logic 1.
          state = TRANSMIT_STOP_BIT;
          //ENABLE_TIMER0_INT() ;                   // Allow this in now.
        }
        OCR1A += TICKS2WAITONESPORT ;  // Count one period into the future.
        break ;

      // Go to idle after stop bit was sent.
      case TRANSMIT_STOP_BIT: // SPORT ************************************* We have sent a stop bit, we look now if there is other byte to send
        if ( ByteStuffByte || (++TxCount < 8 ) )    // Have we sent 8 bytes?
        {
          if ( ByteStuffByte )
          {
            SwUartTXData = ByteStuffByte ;
            ByteStuffByte = 0 ;
          }
          else
          {
            if ( TxCount < 7 )    // Data (or crc)?
            {
              SwUartTXData = TxSportData[TxCount] ;
              Crc += SwUartTXData ; //0-1FF
              Crc += Crc >> 8 ; //0-100
              Crc &= 0x00ff ;
            }
            else
            {
              SwUartTXData = 0xFF - Crc ; // prepare sending check digit
            }
            if ( ( SwUartTXData == 0x7E ) || ( SwUartTXData == 0x7D ) )
            {
              ByteStuffByte = SwUartTXData ^ 0x20 ;
              SwUartTXData = 0x7D ;
            }
          }
          SET_TX_PIN() ;                    // Send a logic 0 on the TX_PIN. (= start bit)
          OCR1A = TCNT1 + TICKS2WAITONESPORT - INTERRUPT_BETWEEN_TRANSMIT;   // Count one period into the future. Compensate the time for ISR
          SwUartTXBitCount = 0 ;
          state = TRANSMIT ;
          //DISABLE_TIMER0_INT() ;      // For the byte duration
        }
        else  // 8 bytes have been send
        {
          frskyStatus |=  1 << sensorIsr ;              // set the bit relative to sensorIsr to say that a new data has to be loaded for sensorIsr.
          state = WAITING ;
          OCR1A += DELAY_3500 ;   // 3.5mS gap before listening
          TRXDDR &= ~( 1 << PIN_SERIALTX ) ;            // PIN is input
          TRXPORT &= ~( 1 << PIN_SERIALTX ) ;           // PIN is tri-stated.
        }
        break ;

      case RECEIVE :  // SPORT ****  Start bit has been received and we will read bits of data receiving, LSB first.
        OCR1A += TICKS2WAITONESPORT ;                    // Count one period after the falling edge is trigged.
        {
          uint8_t data ;        // Use a temporary local storage
          data = SwUartRXBitCount ;
          if ( data < 8 ) {                        // If 8 bits are not yet read
            SwUartRXBitCount = data + 1 ;
            data = SwUartRXData ;
            data >>= 1 ;                         // Shift due to receiving LSB first.
            if ( GET_RX_PIN( ) == 0 ) {
              data |= 0x80 ;                    // If a logical 1 is read, let the data mirror this.
            }
            SwUartRXData = data ;
          }
          else { //Done receiving =  8 bits are in SwUartRXData
            if ( LastRx == 0x7E ) {
              switch (SwUartRXData ) {

                #define  LF_ID        DATA_ID_LF       // replace those values by the right on
                #define  B_ID         DATA_ID_B
                #define  C_ID         DATA_ID_C
                #define  D_ID         DATA_ID_D
                #define  E_ID         DATA_ID_E
                #define  F_ID         DATA_ID_F
                #define  TX_ID        DATA_ID_TX          // this ID is used when TX sent data to RX with a LUA script ; it requires that LUA script uses the same parameters 
                #define SENSOR_ISR_FOR_TX_ID 0XF0          // this value says that we already received a byte == TX_ID

                case LF_ID :
                  sensorIsr = 0 ; break ;
                case B_ID :
                  sensorIsr = 1 ; break ;
                case C_ID :
                  sensorIsr = 2 ; break ;
                case D_ID :
                  sensorIsr = 3 ; break ;
                case E_ID :
                  sensorIsr = 4 ; break ;
                case F_ID :
                  sensorIsr = 5 ; break ;
                case TX_ID :
                  TxDataIdx = 0 ; // reset the counter used to register all bytes received from Tx
                  sensorIsr = SENSOR_ISR_FOR_TX_ID ; break ;  // this value says that an ID related to a frame sent by Tx has been received; take care that it is perhaps just a pulling from RX without Tx frame.
                default :
                  sensorIsr = 128 ;
              }
              if ( ( sensorIsr < 6 ) && ( idToReply & (1 << sensorIsr ) ) ) { // If this sensor ID is supported by oXs and it has been configured in order to send some data
                if  ( ( frskyStatus & ( 1 << sensorIsr ) ) == 0 )  {    // If this sensor ID is supported by oXs and oXs has prepared data to reply data in dataValue[] for this sensorSeq
                  // if ( sportDataLock == 0 ) {
                  TxSportData[0] = 0x10 ;
                  TxSportData[1] = dataId[sensorIsr] & 0x0F; //MODIFIED!!!!
                  TxSportData[2] = dataId[sensorIsr] >> 4 ;
                  TxSportData[3] = dataValue[sensorIsr] ;
                  TxSportData[4] = dataValue[sensorIsr] >> 8 ;
                  TxSportData[5] = dataValue[sensorIsr] >> 16 ;
                  TxSportData[6] = dataValue[sensorIsr] >> 24 ;
                } else {
                  TxSportData[0] = 0x10 ;
                  TxSportData[1] = 0 ;
                  TxSportData[2] = 0 ;
                  TxSportData[3] = 0 ;
                  TxSportData[4] = 0 ;
                  TxSportData[5] = 0 ;
                  TxSportData[6] = 0 ;
                }
                state = TxPENDING ;
                OCR1A += ( DELAY_400 - TICKS2WAITONESPORT) ;    // 400 uS gap before sending (remove 1 tick time because it was already added before

              } else if ( sensorIsr == SENSOR_ISR_FOR_TX_ID )  {       // we received an ID that could be used by TX to send data to the sensor; so we have to continue reading bytes
                state = IDLE ;                      // Go back to idle
              } else  { // No data are loaded (so there is no data yet available or oXs does not have to reply to this ID)
                state = WAITING ;       // Wait for idle time
                OCR1A += DELAY_3500 ;   // 3.5mS gap before listening
              }
            }    // end receive 1 byte and previous was equal to 0x7E
            else if ( SwUartRXData == 0x7E) {      // reset sensorIsr when 0X7E is received (stop receiving data from Tx) and listen to next byte
              sensorIsr = 128 ;                  // reset sensorIsr when 0X7E is received (stop receiving data from Tx)
              rxStuff = 0;                       // and reset the stuff flag
              state =  IDLE ;                      // Go back to idle.
            } else if ((sensorIsr == SENSOR_ISR_FOR_TX_ID) && (TxDataIdx < 8) ) {                 // we receive one byte that is not 0x7E. We check if it follow a sequence 0X7E and the Tx_ID which means it is sent by Tx to oXs
              // Note: if all bytes have been received, then TxDataIdx = 8 and we do not store the data anymore; test on TxDataIdx = 8 is done in .ino file
              if (SwUartRXData == 0x7D)                 // byte stuffing indicator
                rxStuff = 1;                      // set the flag and discard byte
              else if (rxStuff == 0)
                TxData[TxDataIdx++] = SwUartRXData ;                         // we save the received byte in a buffer
              else {
                TxData[TxDataIdx++] = SwUartRXData | 0x20 ;                         // we save the received byte in a buffer taking into account the stuff bit
                rxStuff = 0;                    // and reset the flag
              }
              state = IDLE ;                      // Go back to idle.
            } else {
              state = IDLE ;                      // Go back to idle.
            }        // end of test on receiving one byte
            LastRx = SwUartRXData ;                 // save the current byte
            if (state == IDLE ) {                    // when Go back to idle.
              DISABLE_TIMER_INTERRUPT() ;         // Stop the timer interrupts.
              PCIFR = ( 1 << PCIF2 ) ;            // clear pending interrupt
              PCICR |= ( 1 << PCIE2 ) ;           // pin change interrupt enabled (so we can receive another byte)
            }

          } // End receiving  1 bit or 1 byte (8 bits)
        }
        break ;

      case TxPENDING :   // SPORT ****** we will here send a start bit before sending a byte
        TRXDDR |= ( 1 << PIN_SERIALTX ) ;       // PIN is output
        SET_TX_PIN() ;                          // Send a logic 0 on the TX_PIN. = start bit
        OCR1A = TCNT1 +  ( TICKS2WAITONESPORT - INTERRUPT_ENTRY_TRANSMIT );    // Count one period into the future (less the time to execute ISR) .
        SwUartTXBitCount = 0 ;
        Crc = SwUartTXData = TxSportData[0] ;
        TxCount = 0 ;
        state = TRANSMIT ;
        break ;
      //#endif // end of Frsky_Port

      case WAITING :       // SPORT ******** we where waiting for some time before listening for an start bit; we can now expect a start bit again
        DISABLE_TIMER_INTERRUPT() ;  // Stop the timer interrupts.
        state = IDLE ;               // Go back to idle.
        PCIFR = ( 1 << PCIF2 ) ;     // clear pending interrupt
        PCICR |= ( 1 << PCIE2 ) ;    // pin change interrupt enabled
        break ;

      // Unknown state.
      default:
        state = IDLE;                               // Error, should not occur. Going to a safe state.
    } // End CASE
  } // end sportAvailable == true

} // End of ISR

// End of the code for both Frsky protocols +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void initSportUart(  )           //*************** initialise UART pour SPORT
{

  //PORT
  TRXDDR &= ~( 1 << PIN_SERIALTX ) ;       // PIN is input.
  TRXPORT &= ~( 1 << PIN_SERIALTX ) ;      // PIN is tri-stated.

  // External interrupt

#if PIN_SERIALTX == 4
  PCMSK2 |= 0x10 ;                    // IO4 (PD4) on Arduini mini
#elif PIN_SERIALTX == 2
  PCMSK2 |= 0x04 ;                    // IO2 (PD2) on Arduini mini
#else
#error "This PIN is not supported"
#endif

  PCIFR = (1 << PCIF2) ; // clear pending interrupt
  PCICR |= (1 << PCIE2) ; // pin change interrupt enabled

  // Internal State Variable
  state = IDLE ;
}

ISR(PCINT2_vect)
{
  if ( TRXPIN & ( 1 << PIN_SERIALTX ) ) {     // if Pin is high = start bit (inverted)
    DISABLE_PIN_CHANGE_INTERRUPT()  ;     // pin change interrupt disabled
    //PORTC &= ~2 ;
    state = RECEIVE ;                 // Change state
    DISABLE_TIMER_INTERRUPT() ;       // Disable timer to change its registers.
    OCR1A = TCNT1 + TICKS2WAITONE_HALFSPORT - INTERRUPT_EXEC_CYCL - INTERRUPT_EARLY_BIAS ; // Count one and a half period into the future.
    SwUartRXBitCount = 0 ;            // Clear received bit counter.
    CLEAR_TIMER_INTERRUPT() ;         // Clear interrupt bits
    ENABLE_TIMER_INTERRUPT() ;        // Enable timer1 interrupt on again
  }
}

#endif   //End of FRSKY protocols
