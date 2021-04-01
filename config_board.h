
#ifndef CONF_BOARD_H_INCLUDED


#define APP_CONF_DEBUG_ENABLED 1

// LoRa network carrier
// 1: Multitech standard
// 2: Senet
#define APP_LORA_NETWORK_CARRIER 1

// Gateway id
// 1: blue box: testing
// 2: white box: production
#define GATEWAY_ID 1


#if APP_LORA_NETWORK_CARRIER == 1
// If multitech gateway is used:
#if GATEWAY_ID == 1

#define APP_NETWORK_ID "ussgateway"  //"isenseconduit"
#define APP_NETWORK_KEY "ussgateway" //"testtest"
#define APP_NETWORK_BAND "7"

//#define APP_NETWORK_ID "19526023"  //"isenseconduit"
//#define APP_NETWORK_KEY "19526023" //"testtest"
//#define APP_NETWORK_BAND "7"
#else
#define APP_NETWORK_ID "19618447"
#define APP_NETWORK_KEY "19618447"
#define APP_NETWORK_BAND "3"
#endif

#elif APP_LORA_NETWORK_CARRIER == 2

#define APP_NETWORK_ID "00250C0000010001"
// Netwok key for node 00-80-00-00-04-00-90-4A
//#define		APP_NETWORK_KEY													"A1C717A489AC8BFAC088F07F07D6FF2E"
// Netwok key for node 00-80-00-00-04-00-90-4D
#define APP_NETWORK_KEY "FA38F955ABE722BCBEBE135D6B2C4962"
#define APP_NETWORK_BAND "1"

#endif

#if APP_LORA_NETWORK_CARRIER == 1
#define APP_LORA_PUBLIC_NETWORK "0"
#elif LORA_NETWORK_CARRIER == 2
#define APP_LORA_PUBLIC_NETWORK "1"
#endif

#define APP_LORA_TX_DATA_RATE "DR0"

#define APP_LORA_APP_PORT_SYNC "1"
#define APP_LORA_APP_PORT_SYNC_VALUE 1

#define APP_LORA_APP_PORT_HW1_1 "22"
#define APP_LORA_APP_PORT_HW1_1_VALUE 22

#define APP_LORA_APP_PORT_HW1_2 "23"
#define APP_LORA_APP_PORT_HW1_2_VALUE 23

#define APP_LORA_APP_PORT_HW2_1 "24"
#define APP_LORA_APP_PORT_HW2_1_VALUE 24

#define APP_LORA_APP_PORT_HW2_2 "25"
#define APP_LORA_APP_PORT_HW2_2_VALUE 25

#define APP_LORA_TX_POWER "20"

#define APP_LORA_ANTENNA_GAIN "3"

#define APP_LORA_ADAPTIVE_DATA_RATE "1"

#define APP_LORA_TYPE_MESSAGE_COUNT 1 // use 1 right now because it'll send bat everytime until there is a change required

#if APP_LORA_NETWORK_CARRIER == 1
#define APP_LORA_JOIN_DELAY "1"
#elif APP_LORA_NETWORK_CARRIER == 2
#define APP_LORA_JOIN_DELAY "5"
#endif

#define APP_NUMBER_OF_SEND_FAILURES_BEFORE_MAINTENANCE 3
#define APP_RADIO_MAINTENANCE_RETRY 2
#define APP_SEND_INTERVAL_PACKET_DIAG_SIGNAL_MINUTES 11

#define APP_BLOCK_TIME_MAINTENANCE_DELAY_BASE_MS 300000


//////////////////////////////////////////////////////////////////////////
// The settings here are related.
// Refer to xdot AT command document P72 for more information.
#define APP_SEND_RETRIES 3
#define APP_BLOCK_TIME_SEND 5100
#define APP_ACKNOWLEDGE_ATTEMPTS "3"
//////////////////////////////////////////////////////////////////////////


// #define for US-sugar V1.0 
/////////////////////////////
// firmware/hardware define
#define US_SUGAR_HARDWARE_NUMBER      1
#define US_SUGAR_FIRMWARE_NUMBER      1
#define US_SUGAR_FIRMWARE_VERSION     "US_SUGAR_0.7.0.0\r\n"



// pin #define
#define DO_SENSOR_AIN_CHANNEL 0
#define US_SUGAR_TEMP_SENSOR_PIN 2
#define US_SUGAR_RAIN_SENSOR_PIN 3
#define US_SUGAR_BATT_VOLTAGE_EN_PIN 4
#define US_SUGAR_BATT_VOLTAGE_AIN_CHANNEL 3
#define US_SUGAR_BATT_OK_PIN     29
#define US_SUGAR_PGOOD_REG_PIN   28

#define US_SUGAR_JP1_HW_PIN      11 
#define US_SUGAR_JP2_HW_PIN      12

#define US_SUGAR_XDOT_RX_PIN     14 // NRF-TX
#define US_SUGAR_XDOT_TX_PIN     16 // NRF-RX
#define US_SUGAR_XDOT_WAKE_PIN   15
#define US_SUGAR_XDOT_RESET_PIN  13

#define US_SUGAR_SONAR_EN_PIN    6
#define US_SUGAR_SONAR_TX_PIN    7  // NRF-RX

// define value
#define COMPARE_COUNTERTIME (1UL)  // RTC tick interval // 30 s 
#define BLE_INTERVAL 30
#define LORA_INTERVAL_SYNC_FIRST  45             // LORA message send interval short in second
#define LORA_INTERVAL_SYNC_AFTER  7200             // LORA message send interval short in second
#define LORA_INTERVAL_SHORT 1800             // LORA message send interval short in second
#define LORA_INTERVAL_LONG 7200 - 1            // LORA message send interval long in second

#define HW1_VALUE 1
#define HW2_VALUE 2
#define TEST_VALUE 0 

#define TEMPERATURE_THRESHOLD -10

#define US_SUGAR_MINIMUM_VOLTAGE_THRESHOLD 27

// delay define 
#define US_SUGAR_TEMPERATURE_SENSOR_READ_DELAY_MS   800
#define US_SUGAR_OP_AMP_ENABLE_DELAY_MS             15
#define US_SUGAR_ADC_READ_DELAY_MS                  5
#define US_SUGAR_BLE_DISABLE_DELAY_MS               15

#define US_SUGAR_BATT_CHARGING_WAIT_DELAY_MS        1500
#define US_SUGAR_BATT_CHARGING_WAIT_CYCLE           1800000/US_SUGAR_BATT_CHARGING_WAIT_DELAY_MS  // have to wait for x miliseconds/ US_SUGAR_BATT_CHARGING_WAIT_DELAY_MS

#define US_SUGAR_SONAR_WAIT_INIT_DELAY_MS           1000
#define US_SUGAR_SONAR_READ_DELAY_MS                2000


#define CONF_BOARD_H_INCLUDED

#endif /* CONF_BOARD_H_INCLUDED */