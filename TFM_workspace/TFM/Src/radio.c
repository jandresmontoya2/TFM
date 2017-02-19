/*
 * radio.c
 *
 *  Created on: Mar 9, 2016
 *      Author: frta
 */
#include "radio.h"
#include "sx1276.h"
#include "string.h"
#include "spi.h"

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];
int8_t RssiValue = 0;
int8_t SnrValue = 0;

uint8_t txDoneFlag = 0;
uint8_t rxDoneFlag = 0;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer
};

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

void sx1276_init()
{
//  SX1276_NSS_HIGH();
//  SX1276_RESET_HIGH();
//  HAL_Delay(20);

	// Radio initialization
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init( &RadioEvents );	// FRTA. Apunta a SX1276Init()

  Radio.SetChannel( RF_FREQUENCY );	// FRTA. Apunta a SX1276SetChannel()

#if defined( USE_MODEM_LORA )

  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                 LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 true, 0, 0, LORA_IQ_INVERSION_ON, 100000 );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

  // Apunta a SX1276SetTxConfig
//  Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
//                                FSK_DATARATE, 0,
//                                FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
//                                true, 0, 0, 0, 3000000 );
//
//  Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
//                                0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
//                                0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
//                                0, 0, 0, false );
  Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                FSK_DATARATE, 0,
                                FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                true, 0, 0, 0, 100000 );

  Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                0, 0,false, false );
#else
  #error "Please define a frequency band in the compiler options."
#endif

  __HAL_GPIO_EXTI_CLEAR_IT(SX1276_DIO0_Pin);  	//I DO NOT KNOW WHAT ARE THIS INTERRUPTIONS FOR
//  __HAL_GPIO_EXTI_CLEAR_IT(SX1276_DIO1_Pin);
//  __HAL_GPIO_EXTI_CLEAR_IT(SX_DIO2_Pin);
//  __HAL_GPIO_EXTI_CLEAR_IT(SX_DIO3_Pin);
//  __HAL_GPIO_EXTI_CLEAR_IT(SX_DIO4_Pin);
//  __HAL_GPIO_EXTI_CLEAR_IT(SX_DIO5_Pin);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  Radio.Rx( RX_TIMEOUT_VALUE );
}

void OnTxDone( void )
{
	SX1276SetStby();
	txDoneFlag = 1;
	HAL_GPIO_WritePin(GPIOD, LD4_Pin, SET);
	HAL_Delay(100);
	   				HAL_GPIO_WritePin(GPIOD, LD4_Pin, RESET);
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
	SX1276SetStby();
	rxDoneFlag = 1;
}

void OnTxTimeout( void )
{
//    Radio.Sleep( );
//    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
//    Radio.Sleep( );
//    State = RX_TIMEOUT;
}

void OnRxError( void )
{
//    Radio.Sleep( );
//    State = RX_ERROR;
}

