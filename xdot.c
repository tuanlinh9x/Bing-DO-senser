   /*
 * xdot.c
 *
 * Created: 2/27/2018 10:49:07 PM
 *  Author: jzhai
 */

#include "app_error.h"
#include "bsp.h"
#include "nrf.h"
#include "nrf_delay.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "nrf_delay.h"
#include "nrf_drv_uart.h"
#include "sdk_common.h"
#include "xdot.h"

#define APP_XDOT_RX_BUFFER_SIZE 100 
static char xdot_rx_buffer[APP_XDOT_RX_BUFFER_SIZE];
static uint16_t xdot_rx_count = 0;
static uint16_t xdot_useful_content_start_index;
static uint16_t xdot_useful_content_end_index;

void xdot_send_command(uint8_t *str);
static app_code_t xdot_read_callback();
static app_code_t xdot_parse_response();

nrf_drv_uart_t test_uart_inst = NRF_DRV_UART_INSTANCE(0);

uint8_t rx_buffer[1];

void xdot_uninit(void)
{
    nrf_drv_uart_rx_abort(&test_uart_inst);

    nrf_drv_uart_uninit(&test_uart_inst);

    __WFI();
}

void uart_event_handler(nrf_drv_uart_event_t *p_event, void *p_context)
{
    switch (p_event->type)
    {
    case NRF_DRV_UART_EVT_RX_DONE:
    {        
        //printf("%c",xdot_rx_buffer[xdot_rx_count]);
        xdot_rx_buffer[xdot_rx_count] = p_event->data.rxtx.p_data[0];
        xdot_rx_count++;
        //nrf_drv_uart_tx(&test_uart_inst, &p_event->data.rxtx.p_data[0], 1);
        nrf_drv_uart_rx(&test_uart_inst, &rx_buffer[0], 1);
        break;
    }
    default:
        break;
    }
}


void xdot_init(void)
{
    nrf_gpio_cfg_input(US_SUGAR_XDOT_WAKE_PIN, NRF_GPIO_PIN_PULLDOWN); // US_SUGAR_XDOT_WAKE_PIN 
    
    uint32_t err_code;
    nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;
    config.baudrate = NRF_UART_BAUDRATE_115200;
    config.hwfc = NRF_UART_HWFC_DISABLED;
    config.interrupt_priority = APP_IRQ_PRIORITY_LOWEST;
    config.parity = NRF_UART_PARITY_EXCLUDED;
    config.pselrxd = US_SUGAR_XDOT_TX_PIN;
    config.pseltxd = US_SUGAR_XDOT_RX_PIN;

    err_code = nrf_drv_uart_init(&test_uart_inst, &config, uart_event_handler);
    VERIFY_SUCCESS(err_code);

    nrf_drv_uart_rx(&test_uart_inst, &rx_buffer[0], 1);
}

void xdot_seed_read()
{
    xdot_rx_count = 0;
    nrf_drv_uart_rx(&test_uart_inst, &rx_buffer[0], 1);
}

void xdot_sleep()
{
    xdot_send_command(XDOT_COMMAND_SLEEP);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_clean_buffer()
{
    memcpy(xdot_rx_buffer, 0, APP_XDOT_RX_BUFFER_SIZE);
}

void xdot_wake()
{
    nrf_gpio_cfg_output(US_SUGAR_XDOT_WAKE_PIN); // BATT_VOLTAGE_EN
    nrf_gpio_pin_write(US_SUGAR_XDOT_WAKE_PIN, 1);
    nrf_gpio_pin_write(US_SUGAR_XDOT_WAKE_PIN, 0);
    nrf_delay_ms(5);
    nrf_gpio_cfg_input(US_SUGAR_XDOT_WAKE_PIN, NRF_GPIO_PIN_NOPULL); 
    //	port_pin_set_output_level(APP_XDOT_WAKE_PIN, APP_GPIO_HIGH);
    //      port_pin_set_output_level(APP_XDOT_WAKE_PIN, APP_GPIO_LOW);
}


app_code_t xdot_check_command_result()
{
	return xdot_parse_response();
}

void xdot_set_echo_off()
{
    xdot_send_command(XDOT_COMMAND_ECHO_OFF);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_wake_mode()
{
    xdot_send_command(XDOT_COMMAND_SET_WAKE_MODE);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_public_network(char *public_network)
{
    xdot_send_command(XDOT_COMMAND_SET_PUBLIC_NETWORK);
    xdot_send_command(public_network);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_antenna_gain(char *antenna_gain)
{
    xdot_send_command(XDOT_COMMAND_SET_ANTENNA_GAIN);
    xdot_send_command(antenna_gain);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_join_delay(char *join_delay)
{
    xdot_send_command(XDOT_COMMAND_SET_JOIN_DELAY);
    xdot_send_command(join_delay);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_app_port(char *app_port)
{
    xdot_send_command(XDOT_COMMAND_SET_APP_PORT);
    xdot_send_command(app_port);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_network_id(char *network_id)
{
    xdot_send_command( XDOT_COMMAND_SET_NETWORK_NAME );
    xdot_send_command(network_id);
    nrf_delay_ms(5);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_network_key(char *network_key)
{
    xdot_send_command( XDOT_COMMAND_SET_NETWORK_KEY );
    xdot_send_command(network_key);
    nrf_delay_ms(5);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_network_band(char *network_band)
{
    xdot_send_command(XDOT_COMMAND_SET_NETWORK_BAND);
    xdot_send_command(network_band);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_tx_communication_class(char *communication_class)
{
    xdot_send_command(XDOT_COMMAND_SET_COMMUNICATION_CLASS);
    xdot_send_command(communication_class);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_listen_before_talk(char *listen_time_us, char *threshold)
{
    xdot_send_command(XDOT_COMMAND_SET_LISTEN_BEFORE_TALK);
    xdot_send_command(listen_time_us);
    xdot_send_command(",");
    xdot_send_command(threshold);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_acknowledgement(char *ack)
{
    xdot_send_command(XDOT_COMMAND_SET_ACKNOWLEDGEMENT);
    xdot_send_command(ack);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_receive_output_format(char *receive_output_format)
{
    xdot_send_command(XDOT_COMMAND_RECEIVE_OUTPUT_FORMAT);
    xdot_send_command(receive_output_format);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_tx_data_rate(char *data_rate)
{
    xdot_send_command(XDOT_COMMAND_SET_TX_DATA_RATE);
    xdot_send_command(data_rate);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_tx_power(char *tx_power)
{
    xdot_send_command(XDOT_COMMAND_SET_TX_POWER);
    xdot_send_command(tx_power);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_save_configuration()
{
    xdot_send_command(XDOT_COMMAND_SAVE_CONFIGURATION);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_check_network_join_status()
{
    xdot_send_command(XDOT_COMMAND_CHECK_NETWORK_JOIN_STATUS);
    xdot_send_command(XDOT_COMMAND_END);
}

bool xdot_is_network_joined()
{   
    return xdot_rx_buffer[xdot_useful_content_start_index] == '1';
}

void xdot_network_join()
{
    xdot_send_command(XDOT_COMMAND_NETWORK_JOIN);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_save_session()
{
    xdot_send_command(XDOT_COMMAND_SAVE_SESSION);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_factory_reset()
{
    xdot_send_command(XDOT_COMMAND_FACTORY_RESET);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_restore_session()
{
    xdot_send_command(XDOT_COMMAND_RESTORE_SESSION);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_rssi()
{
    xdot_send_command(XDOT_COMMAND_RSSI);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_snr()
{
    xdot_send_command(XDOT_COMMAND_SNR);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_set_adaptive_data_rate(char *adr)
{
    xdot_send_command(XDOT_COMMAND_ADAPTIVE_DATA_RATE);
    xdot_send_command(adr);
    xdot_send_command(XDOT_COMMAND_END);
}

uint8_t xdot_parse_rssi(char *rssi_buffer)
{
    uint8_t temp_rx_buffer_index = xdot_useful_content_start_index;
    uint8_t temp_output_buffer_index = 0;
    memset(rssi_buffer, 0, strlen(rssi_buffer));
    while (xdot_rx_buffer[temp_rx_buffer_index] != ',')
    {
        *(rssi_buffer + temp_output_buffer_index) = xdot_rx_buffer[temp_rx_buffer_index];
        temp_rx_buffer_index++;
        temp_output_buffer_index++;
    }
    return temp_output_buffer_index;
}

uint8_t xdot_parse_snr(char *snr_buffer)
{
    uint8_t temp_rx_buffer_index = xdot_useful_content_start_index;
    uint8_t temp_output_buffer_index = 0;
    memset(snr_buffer, 0, strlen(snr_buffer));
    while (xdot_rx_buffer[temp_rx_buffer_index] != ',')
    {
        *(snr_buffer + temp_output_buffer_index) = xdot_rx_buffer[temp_rx_buffer_index];
        temp_rx_buffer_index++;
        temp_output_buffer_index++;
    }
    return temp_output_buffer_index;
}

void xdot_send_data(char *data)
{
    xdot_send_command(XDOT_COMMAND_SEND_DATA);
    xdot_send_command(data);
    nrf_delay_ms(5);
    xdot_send_command(XDOT_COMMAND_END);
}

void xdot_send_binary_data(char *data)
{
    xdot_send_command(XDOT_COMMAND_SEND_BINARY_DATA);
    xdot_send_command(data);
    nrf_delay_ms(5);
    //xdot_send_command(XDOT_COMMAND_END);
}

uint8_t xdot_get_received_data(char *receive_data_buffer)
{
    memset(receive_data_buffer, 0, strlen(receive_data_buffer));
    if (xdot_useful_content_start_index < xdot_useful_content_end_index)
    {
        memcpy(
            receive_data_buffer,
            xdot_rx_buffer + xdot_useful_content_start_index,
            xdot_useful_content_end_index - xdot_useful_content_start_index + 1);
    }
}


void xdot_send_command(uint8_t *str)
{
    xdot_rx_count = 0;
    uint8_t test[100];
    memcpy(test, (uint8_t *)str, strlen(str));
    nrf_drv_uart_tx(&test_uart_inst, test, strlen(str));
    nrf_delay_ms(1);
}



static app_code_t xdot_parse_response()
{
    // ???? Disable receive interrupt ????
    xdot_rx_buffer[xdot_rx_count] = NULL;
#if APP_CONF_DEBUG_ENABLED == 1
   SEGGER_RTT_WriteString(0, xdot_rx_buffer);
#endif
    if (xdot_rx_count == 0)
    {
        return APP_XDOT_RX_TIMEOUT;
    }
    else if (xdot_rx_count < APP_XDOT_SMALLEST_RESPONSE_LENGTH)
    {
        return APP_XDOT_RX_CORRUPTED_MESSAGE;
    }
    else
    {
        // Compare uart_rx_buffer[uart_rx_count - strlen(XDOT_RESPONSE_OK)]
        // with XDOT_RESPONSE_OK.
        if (strcmp(&(xdot_rx_buffer[xdot_rx_count - 4]), XDOT_RESPONSE_OK) == 0)
        {
            // Response is OK
            // Case 1: AT+DI\0x0A
            // Response:
            //			//
            // 00-80-00-00-04-00-4b-a0
            //
            // OK
            //
            // Hex:
            // 0D 0D 0A 30 30 2D 38 30 2D 30 30 2D 30 30 2D 30 34 2D 30 30 2D 34 62 2D 61 30 0D 0A 0D 0A 4F 4B 0D 0A

            // Case 2: AT\0x0A
            // Response:
            //
            //
            // OK
            //
            // Hex: 0D 0D 0A 0D 0A 4F 4B 0D 0A

            // This is when the AT command is send and the response is just OK.
            if (xdot_rx_count == APP_XDOT_SMALLEST_RESPONSE_LENGTH)
            {
                xdot_useful_content_start_index = 0;
                xdot_useful_content_end_index = 0;
            }
            else
            {
                xdot_useful_content_start_index = 3;
                // 9 is the length of "OK" + 1: 0D 0A 0D 0A 4F 4B 0D 0A
                xdot_useful_content_end_index = xdot_rx_count - 9;
            }

            if (xdot_useful_content_start_index > xdot_useful_content_end_index)
            {
                return APP_XDOT_RX_LENGTH_ERROR;
            }
            else
            {
                return APP_XDOT_OK;
            }
        }
        // Compare uart_rx_buffer[uart_rx_count - strlen(XDOT_RESPONSE_ERROR)]
        // with XDOT_RESPONSE_ERROR
        else if (strcmp(&(xdot_rx_buffer[xdot_rx_count - 7]), XDOT_RESPONSE_ERROR) == 0)
        {
            // Response is ERROR
            // Case 1: AT+D\0x0A
            // Response:
            //
            //
            // Command not found!
            //
            // ERROR
            // Hex:
            // 0D 0D 0A 43 6F 6D 6D 61 6E 64 20 6E 6F 74 20 66 6F 75 6E 64 21 0D 0A 0D 0A 45 52 52 4F 52 0D 0A

            xdot_useful_content_start_index = 3;
            // 12 is the length of "ERROR" + 1: 0D 0A 0D 0A 45 52 52 4F 52 0D 0A
            xdot_useful_content_end_index = xdot_rx_count - 12;

            if (xdot_useful_content_start_index > xdot_useful_content_end_index)
            {
                return APP_XDOT_RX_LENGTH_ERROR;
            }
            else
            {
                return APP_XDOT_COMMAND_EXE_ERROR;
            }
        }
        else
        {
            return APP_XDOT_OTHER_ERROR;
        }
    }
}