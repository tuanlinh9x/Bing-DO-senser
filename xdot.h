/*
 * xdot.h
 *
 * Created: 2/27/2018 10:48:03 PM
 *  Author: jzhai
 */

#ifndef XDOT_H_
#define XDOT_H_

#include <string.h>
#include "config_board.h"

enum board_init_code {
	APP_BOARD_INIT_OK						= 0x00,
	APP_BOARD_INIT_GPIO_FAILED				= 0x01,	
	APP_BOARD_INIT_I2C_FAILED				= 0x02,	
	APP_BOARD_INIT_UART_FAILED				= 0x03,	
	APP_BOARD_INIT_ADC_FAILED				= 0x04,
	APP_BOARD_INIT_RTC_FAILED				= 0x05,	
	APP_BOARD_INIT_I2C_2_FAILED				= 0x06,	
	APP_BOARD_INIT_UART_2_FAILED			= 0x07,
};
typedef enum board_init_code board_init_code_t;

enum app_code {
	APP_XDOT_OK								= 0x00,
	APP_XDOT_RX_TIMEOUT						= 0x01,
	APP_XDOT_RX_CORRUPTED_MESSAGE			= 0x02,
	APP_XDOT_RX_LENGTH_ERROR				= 0x03,
	APP_XDOT_COMMAND_EXE_ERROR				= 0x04,
	APP_XDOT_OTHER_ERROR					= 0x05,
};
typedef enum app_code app_code_t;

typedef struct app_diagnostics_radio_signal{
	// The extra byte is used to store the NULL, so sprintf knows where the end of the string is.
	char radio_rx_rssi[4 + 1];
	char radio_rx_snr[5 + 1];
} app_diagnostics_radio_signal_t;

#define		APP_XDOT_COMMUNICATION_CLASS									"A"
#define		APP_XDOT_RX_BUFFER_SIZE											128
#define		APP_XDOT_SMALLEST_RESPONSE_LENGTH								9
#define		APP_XDOT_WAKE_PIN												PIN_PB10
#define		APP_XDOT_WAKE_DELAY_MS											375

#define		APP_XDOT_CONFIGURE_COMMAND_DELAY_MS								15
#define		APP_XDOT_SAVE_CONFIGURATION_DELAY_MS							200
#define		APP_XDOT_NETWORK_JOIN_DELAY_MS									8000
#define		APP_XDOT_SAVE_SESSION_DELAY_MS									150

#define XDOT_COMMAND_END "\r"
#define XDOT_COMMAND_ECHO_OFF "ATE0"
#define XDOT_COMMAND_FACTORY_RESET "AT&F"
#define XDOT_COMMAND_SLEEP "AT+SLEEP=0"
#define XDOT_COMMAND_SET_PUBLIC_NETWORK "AT+PN="
#if APP_LORA_NETWORK_CARRIER == 1
#define XDOT_COMMAND_SET_NETWORK_NAME "AT+NI=1,"
#elif APP_LORA_NETWORK_CARRIER == 2
#define XDOT_COMMAND_SET_NETWORK_NAME "AT+NI=0,"
#endif

//#define XDOT_COMMAND_SET_WAKE_MODE "AT+WM=1"

#if APP_LORA_NETWORK_CARRIER == 1
#define XDOT_COMMAND_SET_NETWORK_KEY "AT+NK=1,"
#elif APP_LORA_NETWORK_CARRIER == 2
#define XDOT_COMMAND_SET_NETWORK_KEY "AT+NK=0,"
#endif

#define XDOT_COMMAND_SET_NETWORK_BAND "AT+FSB="
#define XDOT_COMMAND_SET_TX_POWER "AT+TXP="
#define XDOT_COMMAND_SET_ANTENNA_GAIN "AT+ANT="
#define XDOT_COMMAND_SET_JOIN_DELAY "AT+JD="
#define XDOT_COMMAND_SET_WAKE_MODE "AT+WM=1"
#define XDOT_COMMAND_SET_TX_DATA_RATE "AT+TXDR="
#define XDOT_COMMAND_SET_LISTEN_BEFORE_TALK "AT+LBT="
#define XDOT_COMMAND_SET_ACKNOWLEDGEMENT "AT+ACK="
#define XDOT_COMMAND_SET_COMMUNICATION_CLASS "AT+DC="
#define XDOT_COMMAND_SAVE_CONFIGURATION "AT&W"
#define XDOT_COMMAND_CHECK_NETWORK_JOIN_STATUS "AT+NJS"
#define XDOT_COMMAND_NETWORK_JOIN "AT+JOIN"
#define XDOT_COMMAND_SAVE_SESSION "AT+SS"
#define XDOT_COMMAND_RESTORE_SESSION "AT+RS"
#define XDOT_COMMAND_RSSI "AT+RSSI"
#define XDOT_COMMAND_SNR "AT+SNR"
#define XDOT_COMMAND_SEND_DATA "AT+SEND="
#define XDOT_COMMAND_SEND_BINARY_DATA "AT+SENDB="
#define XDOT_COMMAND_ADAPTIVE_DATA_RATE "AT+ADR="
#define XDOT_COMMAND_SET_APP_PORT "AT+AP="
//#define			XDOT_COMMAND_DATA_PENDING				"AT+DP"
#define XDOT_COMMAND_RECEIVE_DATA "AT+RECV"
#define XDOT_COMMAND_RECEIVE_OUTPUT_FORMAT "AT+RXO="
#define XDOT_RESPONSE_OK "OK\r\n"
#define XDOT_RESPONSE_ERROR "ERROR\r\n"

void xdot_init(void);

void xdot_sleep();

void xdot_wake();

app_code_t xdot_check_command_result();

void xdot_set_echo_off();

void xdot_set_network_id(char *network_id);

void xdot_set_network_key(char *network_key);

void xdot_set_network_band(char *network_band);

void xdot_set_tx_data_rate(char *data_rate);

void xdot_set_tx_power(char *tx_power);

void xdot_set_tx_communication_class(char *communication_class);

void xdot_set_listen_before_talk(char *listen_time_us, char *threshold);

void xdot_set_wake_mode();

void xdot_set_public_network(char *public_network);

void xdot_set_antenna_gain(char *antenna_gain);

void xdot_set_join_delay(char *join_delay);

void xdot_set_acknowledgement(char *ack);

void xdot_set_adaptive_data_rate(char *adr);

void xdot_set_receive_output_format(char *receive_output_format);

void xdot_save_configuration();

void xdot_check_network_join_status();
bool xdot_is_network_joined();

void xdot_network_join();

void xdot_set_app_port(char *app_port);

void xdot_rssi();
uint8_t xdot_parse_rssi(char *rssi_buffer);
void xdot_snr();
uint8_t xdot_parse_snr(char *snr_buffer);

void xdot_save_session();

void xdot_restore_session();

void xdot_factory_reset();

void xdot_send_data(char *data);

void xdot_send_binary_data(char *data);
/*
void xdot_data_pending();
bool xdot_is_data_pending();
	
void xdot_receive_data();
*/
void xdot_send_command(uint8_t* str);

uint8_t xdot_get_received_data(char *receive_data_buffer);

void xdot_uninit(void);

void xdot_clean_buffer();

void xdot_seed_read();

#endif /* XDOT_H_ */