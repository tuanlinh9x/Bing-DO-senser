#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_button.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "nrf_sdh_soc.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "sensorsim.h"

#include "FreeRTOS.h"
#include "config_board.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_uart.h"
#include "nrf_drv_wdt.h"
#include "nrf_rtc.h"
#include "nrf_saadc.h"
#include "semphr.h"
#include "task.h"
#include "xdot.h"


/////////
nrf_drv_wdt_channel_id m_channel_id;

uint32_t message_count = 0;
uint32_t message_count_sent_OK = 0;
uint8_t message_send_rate = 0;
uint16_t time_tick_sec = 0;
uint16_t time_tick_hour = 0;
uint8_t BLE_block = 0;
////////////////////////////////////////
// const and declairation
const TickType_t block_time_xdot_configure = APP_XDOT_CONFIGURE_COMMAND_DELAY_MS / portTICK_PERIOD_MS;
const TickType_t block_time_save_configuration = APP_XDOT_SAVE_CONFIGURATION_DELAY_MS / portTICK_PERIOD_MS;
const TickType_t block_time_join = APP_XDOT_NETWORK_JOIN_DELAY_MS / portTICK_PERIOD_MS;
const TickType_t block_time_xdot_save_session = APP_XDOT_SAVE_SESSION_DELAY_MS / portTICK_PERIOD_MS;
const TickType_t block_time_send = APP_BLOCK_TIME_SEND / portTICK_PERIOD_MS;
const TickType_t block_time_maintenance_delay_base = APP_BLOCK_TIME_MAINTENANCE_DELAY_BASE_MS / portTICK_PERIOD_MS;

const TickType_t block_time_xdot_wake_delay = APP_XDOT_WAKE_DELAY_MS / portTICK_PERIOD_MS;

static volatile bool radio_maintaining = true;
///////////////////////////////////////////

static app_diagnostics_radio_signal_t radio_signal_instance;
static uint8_t number_of_failures_send = 0;
uint8_t batt_ok = 0;

ble_gap_addr_t ble_mac_address;

SemaphoreHandle_t LORA_send_sem;
SemaphoreHandle_t dummy_sem;
SemaphoreHandle_t rain_count_sem;
SemaphoreHandle_t rain_count_mutex;
SemaphoreHandle_t BLE_name_sem;
SemaphoreHandle_t RTC_tick_sem;

static TaskHandle_t task_handle_send = NULL;
static TaskHandle_t task_handle_radio_maintenance = NULL;

app_code_t maintenance_result = APP_XDOT_OTHER_ERROR;

uint8_t BLE_name[12];

void BLE_name_change_task(void *p);
void task_send(void *p);
void task_radio_maintenance(void *p);
void rain_count_task(void *arg);
void report_task(void *arg);
void RTC_tick_task(void *arg);

static app_code_t execute_xdot_operation(void (*xdot_op)(), char *xdot_param, TickType_t block_time);

#define APP_ADV_INTERVAL 2000 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO 3                                /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG 1                                 /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 0                               /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                /**< Handle of the current connection. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];            /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded scan data. */

uint8_t sync_sent = 0;
uint8_t HW_type = 0;

uint16_t mb7388_depth = 0;
int16_t temppp = 0;
uint16_t rain_count = 0;
uint8_t rain_seq = 0;
uint8_t message_count_time = 0;
uint16_t batt_val;
uint16_t interval_for_lora = 0;
uint8_t sonar_uart_check;
int16_t send_temp_val; // temperature read value to send to LoRaWAN
//////////////////////////////////////////////
// Base BLE code from NRF // DO not touch
static void advertising_start(void *p_erase_bonds);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        advertising_start(false);
        break;

    default:
        break;
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
// modify this function only
ble_gap_addr_t ble_mac_address;

static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    sd_ble_gap_addr_get(&ble_mac_address);
    BLE_name[0] = 'E';
    BLE_name[1] = 'S';

    sprintf((char *)(BLE_name + 2), "%02X%02X%02X%02X%02X", ble_mac_address.addr[0], ble_mac_address.addr[1], ble_mac_address.addr[2], ble_mac_address.addr[3], ble_mac_address.addr[4]);
    err_code = sd_ble_gap_device_name_set(&sec_mode, BLE_name, 12);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_FAST:
        NRF_LOG_INFO("Fast advertising.");
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_ADV_EVT_IDLE:
        sleep_mode_enter();
        break;

    default:
        break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected.");
        // LED indication will be changed when advertising starts.
        break;

    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected.");
        err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        APP_ERROR_CHECK(err_code);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond = SEC_PARAM_BOND;
    sec_param.mitm = SEC_PARAM_MITM;
    sec_param.lesc = SEC_PARAM_LESC;
    sec_param.keypress = SEC_PARAM_KEYPRESS;
    sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob = SEC_PARAM_OOB;
    sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc = 1;
    sec_param.kdist_own.id = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
// modify this function to change the data
static void advertising_init(void)
{
    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    ble_advdata_manuf_data_t manuf_data; //Variable to hold manufacturer specific data
    uint8_t data[9];                     //Our data to advertise
    data[0] = 0x0A;                      //Our data to advertise
    data[1] = 0x01;                      //Our data to advertise
    data[2] = 0x01;                      //Our data to advertise
    data[3] = HW_type;                   //Our data to advertise
    if (message_count == 0)
    {
        data[4] = 0; //Pending
    }
    else
    {
        message_send_rate = message_count_sent_OK * 100 / message_count;
        data[4] = message_send_rate; //Pending
    }
    data[5] = batt_val; //Our data to advertise
    data[6] = time_tick_hour >> 8;
    data[7] = time_tick_hour & 0xFF;
    data[8] = 0;
    if (radio_maintaining == true)
    {
        data[8] = (0 << 1) | (batt_ok); //Our data to advertise
    }
    else
    {
        data[8] = (1 << 1) | (batt_ok); //Our data to advertise
        printf("Joined LORA , %d\r\n", data[8]);
    }

    //manuf_data.company_identifier = 0x0060; //Nordics company ID
    manuf_data.data.p_data = data;
    manuf_data.data.size = 9;
    init.advdata.p_manuf_specific_data = &manuf_data;

    //init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.name_type = BLE_ADVDATA_FULL_NAME; // Use a shortened name
    //init.advdata.short_name_len = 15; // Advertise only first 6 letters of name
    init.advdata.include_appearance = false;
    init.advdata.include_ble_device_addr = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = 0;
    init.advdata.uuids_complete.p_uuids = NULL;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    nrf_pwr_mgmt_run();
}

/**@brief Function for starting advertising. */
static void advertising_start(void *p_erase_bonds)
{
    bool erase_bonds = *(bool *)p_erase_bonds;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        SEGGER_RTT_WriteString(0, "Started finally\r\n");
        APP_ERROR_CHECK(err_code);
    }
}

static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}
// end of BLE code block
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
// Harware ISR Handler code
// EXT_ISR handler
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    SEGGER_RTT_WriteString(0, "switch!!!!\r\n");
    xSemaphoreGiveFromISR(rain_count_sem, NULL);
    /// this section requires a timer to see if this is not a wrong read
}

// SAADC Handler
void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        //err_code = nrf_drv_saadc_buffer_convert
        // APP_ERROR_CHECK(err_code);
        //        printf("ADC : %d", temp_adc);
    }
}
// RTC handler
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        nrfx_rtc_counter_clear(&rtc);
        nrf_drv_rtc_int_enable(&rtc, NRF_RTC_INT_COMPARE0_MASK);
        xSemaphoreGiveFromISR(RTC_tick_sem, pdFALSE);
    }
}
/////////////////////////////////////////////////////
// regular function
uint16_t battery_voltage_get()
{
    uint16_t temp_adc_value;
    nrf_gpio_cfg_output(US_SUGAR_BATT_VOLTAGE_EN_PIN);   // BATT_VOLTAGE_EN
    nrf_gpio_pin_write(US_SUGAR_BATT_VOLTAGE_EN_PIN, 1); // enable first
    vTaskDelay(US_SUGAR_OP_AMP_ENABLE_DELAY_MS / portTICK_RATE_MS);
    nrfx_saadc_sample_convert(US_SUGAR_BATT_VOLTAGE_AIN_CHANNEL, &temp_adc_value);
    vTaskDelay(US_SUGAR_ADC_READ_DELAY_MS / portTICK_RATE_MS);
    printf("Batt voltage raw ADC: %d\r\n", temp_adc_value);
    temp_adc_value = (temp_adc_value * 10) >> 8;
    nrf_gpio_pin_write(US_SUGAR_BATT_VOLTAGE_EN_PIN, 0);                   // disable the module
    nrf_gpio_cfg_input(US_SUGAR_BATT_VOLTAGE_EN_PIN, NRF_GPIO_PIN_NOPULL); // VBATT_OK
    return temp_adc_value;
}

uint8_t battery_status_get()
{
    uint8_t temp_status;

    uint16_t temp_value = nrf_gpio_pin_read(US_SUGAR_BATT_OK_PIN);
    if (temp_value == 0)
    {
        temp_status = 1;
    }
    else
    {
        temp_status = 0;
    }

    return temp_status;
}

app_code_t lora_send_message(int16_t value)
{
    app_code_t lora_result;
    message_count++; // increase message count
    uint8_t retries = APP_SEND_RETRIES;
    // block the BLE and init the Xdot again
    printf("Blocking BLE\r\n");
    BLE_block = 1;
    sd_ble_gap_adv_stop(m_advertising.adv_handle);
    vTaskDelay(US_SUGAR_BLE_DISABLE_DELAY_MS / portTICK_RATE_MS);
    xdot_init();

    uint8_t temp_string[14];
    uint8_t temp_uint8[2];
      
    uint8_t do_voltage = (value * 3)/5;
        // send the sync message
        SEGGER_RTT_WriteString(0, "SEND: data send\r\n");
        // parse the message        
        temp_uint8[0] = do_voltage;
        sprintf(temp_string, "%02hhX\n", temp_uint8[0]);
        printf(temp_string);
        xdot_send_binary_data(temp_string);
  

    // send message
    xdot_send_binary_data(temp_string);
    // check the send result
    do
    {
        vTaskDelay(block_time_send);
        lora_result = xdot_check_command_result();
        retries--;
#if APP_CONF_DEBUG_ENABLED == 1
        SEGGER_RTT_WriteString(0, "SEND11: send check\r\n");
#endif
    } while (lora_result != APP_XDOT_OK && retries > 0);
    // finished the send => unblock BLE and uninit Xdot
    xdot_seed_read();
    xdot_uninit();
    printf("Unlocking BLE\r\n");
    BLE_block = 0;

    return lora_result;
}
/////////////////////////////////////////////////////
// hardware init code
// Baterry monitor module init
void batt_monitor_init(void)
{
    nrf_gpio_cfg_output(US_SUGAR_BATT_VOLTAGE_EN_PIN); // BATT_VOLTAGE_EN
    nrf_gpio_pin_write(US_SUGAR_BATT_VOLTAGE_EN_PIN, 0);

    nrf_gpio_cfg_input(US_SUGAR_BATT_VOLTAGE_EN_PIN, NRF_GPIO_PIN_NOPULL); // VBATT_OK

    nrf_gpio_cfg_input(US_SUGAR_BATT_OK_PIN, NRF_GPIO_PIN_NOPULL); // VBATT_OK

    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);

    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(US_SUGAR_BATT_VOLTAGE_AIN_CHANNEL, &channel_config);
    APP_ERROR_CHECK(err_code);
}
/////////////////////////////////////////////////////
// DO sensor module init
void do_sensor_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
    channel_config.gain = NRF_SAADC_GAIN1  ;
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}
// init hardware sequence: ADC => temp => Sonar => read voltage + batt status
int16_t do_val = 0;
void init_hardware(void)
{
    
    // Configuration print out to make sure everything is in good config
    printf(US_SUGAR_FIRMWARE_VERSION);
    printf("Retry delay: %d minutes\r\n",APP_BLOCK_TIME_MAINTENANCE_DELAY_BASE_MS / 60000);
    printf("Send interval long: %d minutes\r\n",(LORA_INTERVAL_LONG + 1 ) / 60);
    printf("Send interval short: %d minutes\r\n",LORA_INTERVAL_SHORT  / 60);
    printf("Send sync first: %d seconds\r\n",LORA_INTERVAL_SYNC_FIRST );
    printf("Send sync after: %d minutes\r\n",LORA_INTERVAL_SYNC_AFTER / 60 );
    printf("Hardware number: %d\r\n", US_SUGAR_HARDWARE_NUMBER);
    printf("Firmwware number: %d\r\n",US_SUGAR_FIRMWARE_NUMBER);

    do_sensor_init();    

    // read the DO value
    
    nrfx_saadc_sample_convert(DO_SENSOR_AIN_CHANNEL, &do_val);
    printf("DO sensor raw ADC read: %d\r\n", do_val);   
}
// RTC init
void rtc_tick_init(void)
{
    uint32_t err_code;

    // Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc, 0, COMPARE_COUNTERTIME * 8, true);

    nrf_drv_rtc_enable(&rtc);
}
// reinit BLE module
void reinit_ble()
{
    uint32_t err_code;
    gap_params_init();
    //gatt_init();
    advertising_init();
    //conn_params_init();
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    SEGGER_RTT_WriteString(0, "Started \r\n");
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */

void wdt_event_handler(void)
{
    printf("WTD test\r\n");
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

static void System_task_initWatchdog(void)
{
    uint32_t err_code = NRF_SUCCESS;
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}

int main(void)

{
    bool erase_bonds;
    // Initialize.
    log_init();
    System_task_initWatchdog();
    
    init_hardware();

    radio_maintaining = true;
    // init the Jumper
    LORA_send_sem = xSemaphoreCreateBinary();
    rain_count = 0;
    sync_sent = 0;
    printf("Init the program type: %d\r\n", HW_type);

    timers_init();
    power_management_init();

    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();

    // Start execution.
    SEGGER_RTT_WriteString(0, "Start!\r\n");
    application_timers_start();

    nrf_sdh_freertos_init(advertising_start, &erase_bonds);

    SEGGER_RTT_WriteString(0, "FreeRTOS example started\r\n");
    
    
    xTaskCreate(RTC_tick_task, "tick task", 256, NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();

    // Enter main loop.
    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}
////////////////////////////////////////////////////////////////////////////////////////
// Task functions
// LORA short/Long interval check task
void RTC_tick_task(void *arg)
{
    uint8_t interval_for_sync = 0;
    uint8_t interval_for_BLE = 0;
    uint16_t interval_lora_send_long = 0;
    uint16_t sync_time = LORA_INTERVAL_SYNC_FIRST;

    RTC_tick_sem = xSemaphoreCreateBinary();
    xSemaphoreTake(RTC_tick_sem, (TickType_t)10);

    nrf_drv_wdt_channel_feed(m_channel_id); // feed WDT

    xdot_wake();
    vTaskDelay(block_time_xdot_wake_delay);

    execute_xdot_operation(xdot_factory_reset, NULL, block_time_xdot_configure);
    // Disable echo
    execute_xdot_operation(xdot_set_echo_off, NULL, block_time_xdot_configure);
    // Set wake mode
    execute_xdot_operation(xdot_set_wake_mode, NULL, block_time_xdot_configure);
    // Save configuration
    execute_xdot_operation(xdot_save_configuration, NULL, block_time_save_configuration);
    // Sleep
    execute_xdot_operation(xdot_sleep, NULL, block_time_xdot_configure);


    xTaskCreate(task_send, "Task send", 1024, NULL, (tskIDLE_PRIORITY + 1), &task_handle_send); //This uses sprintf, watch the stack size
    xTaskCreate(task_radio_maintenance, "Task Maintenance", 100, NULL, (tskIDLE_PRIORITY + 2), &task_handle_radio_maintenance);
    rtc_tick_init();
    while (1)
    {
        if (xSemaphoreTake(RTC_tick_sem, (TickType_t)1000))
        {
            nrf_drv_wdt_channel_feed(m_channel_id);            
            interval_for_BLE++;
            if (time_tick_sec == 3600)  // 3600 seconds = 1 hour
            {
                time_tick_sec = 0;
                time_tick_hour++;
            }
            
                if (interval_for_BLE < BLE_INTERVAL)
                {
                }
                else
                {
                    xSemaphoreGive(LORA_send_sem);                     
                    interval_for_BLE = 0;
                }
            
        }
    }
}



///////////////////////
// Send task
void task_send(void *p)
{
    app_code_t result = APP_XDOT_OK;
    uint8_t retries = 1;
    char temp_packet[] = "1234567891234567891234";
    uint16_t water_level_val;
    uint16_t lora_send_tick = 0;
    //send the data
    SEGGER_RTT_WriteString(0, "SEND task is on\r\n");

    xSemaphoreTake(LORA_send_sem, (TickType_t)10);
    for (;;)
    {
        if (xSemaphoreTake(LORA_send_sem, (TickType_t)1000))
        {
            // check if this is the interval ???
            // do the reading
            // sensor read for HW1: sonar, batt voltage
            
            // disable BLE

            xdot_wake();
            vTaskDelay(block_time_xdot_wake_delay);
            //execute_xdot_operation(xdot_set_echo_off, NULL, block_time_xdot_configure);

            printf("restore session\r\n");
            if (execute_xdot_operation(xdot_restore_session, NULL, block_time_xdot_configure) == APP_XDOT_OK)
            {
                printf("check join status\r\n");
                if (execute_xdot_operation(xdot_check_network_join_status, NULL, block_time_xdot_configure) == APP_XDOT_OK)
                {
                    if (xdot_is_network_joined() == true)
                    {
#if APP_CONF_DEBUG_ENABLED == 1
                        SEGGER_RTT_WriteString(0, "SEND: start send\r\n");
#endif
                            nrfx_saadc_sample_convert(DO_SENSOR_AIN_CHANNEL, &do_val);
                            printf("DO sensor raw ADC read: %d\r\n", do_val);  
                            // choose port
                            printf("choose app port sync\r\n");
                            if (execute_xdot_operation(xdot_set_app_port, APP_LORA_APP_PORT_SYNC, block_time_xdot_configure) == APP_XDOT_OK)
                            {

                                result = lora_send_message(do_val);
                            }
                            else
                            {
                                printf("failed to choose app port\r\n");
                                result == APP_XDOT_OTHER_ERROR;
                            }
                                                
                        if (result == APP_XDOT_OK)
                        {
                            message_count_sent_OK++; // sent out OK
                            if (sync_sent == 0)      // For sync message
                            {
                                sync_sent = 1;
                                SEGGER_RTT_WriteString(0, "SEND: send sync done\r\n");
                                if (execute_xdot_operation(xdot_save_session, NULL, block_time_xdot_save_session) != APP_XDOT_OK)
                                {
#if APP_CONF_DEBUG_ENABLED == 1
                                    SEGGER_RTT_WriteString(0, "SEND: save session failed\r\n");
#endif
                                }

                                if (execute_xdot_operation(xdot_sleep, NULL, block_time_xdot_configure) != APP_XDOT_OK)
                                {
#if APP_CONF_DEBUG_ENABLED == 1
                                    SEGGER_RTT_WriteString(0, "SEND: sleep failed\r\n");
#endif
                                }
                            }
                            else // for regular message
                            {
                                // if HW2 => reset the rain count and release the mutex
                                if (HW_type == HW1_VALUE)
                                {
                                    rain_count = 0;
                                    xSemaphoreGive(rain_count_mutex);
                                    if (rain_seq == 0)
                                    {
                                        rain_seq = 1;
                                    }
                                    else
                                    {
                                        rain_seq = 0;
                                    }
                                }

                                //
#if APP_CONF_DEBUG_ENABLED == 1
                                SEGGER_RTT_WriteString(0, "SEND: send done\r\n");
#endif
                                if (execute_xdot_operation(xdot_rssi, NULL, block_time_xdot_configure) == APP_XDOT_OK)
                                {
                                    xdot_parse_rssi(radio_signal_instance.radio_rx_rssi);
#if APP_CONF_DEBUG_ENABLED == 1
                                    SEGGER_RTT_WriteString(0, "SEND: rssi\r\n");
#endif
                                }
                                if (execute_xdot_operation(xdot_snr, NULL, block_time_xdot_configure) == APP_XDOT_OK)
                                {
                                    xdot_parse_snr(radio_signal_instance.radio_rx_snr);
#if APP_CONF_DEBUG_ENABLED == 1
                                    SEGGER_RTT_WriteString(0, "SEND: snr\r\n");
#endif
                                }

                                if (execute_xdot_operation(xdot_save_session, NULL, block_time_xdot_save_session) != APP_XDOT_OK)
                                {
#if APP_CONF_DEBUG_ENABLED == 1
                                    SEGGER_RTT_WriteString(0, "SEND: save session failed\r\n");
#endif
                                }

                                if (execute_xdot_operation(xdot_sleep, NULL, block_time_xdot_configure) != APP_XDOT_OK)
                                {
#if APP_CONF_DEBUG_ENABLED == 1
                                    SEGGER_RTT_WriteString(0, "SEND: sleep failed\r\n");
#endif
                                }
                            }
                        }
                        else
                        {
                            // if HW1 => release the mutex
                            if (HW_type == HW1_VALUE)
                            {
                                xSemaphoreGive(rain_count_mutex);
                            }

                            // make it sleep
                            if (execute_xdot_operation(xdot_save_session, NULL, block_time_xdot_save_session) != APP_XDOT_OK)
                            {
#if APP_CONF_DEBUG_ENABLED == 1
                                SEGGER_RTT_WriteString(0, "SEND: save session failed\r\n");
#endif
                            }

                            if (execute_xdot_operation(xdot_sleep, NULL, block_time_xdot_configure) != APP_XDOT_OK)
                            {
#if APP_CONF_DEBUG_ENABLED == 1
                                SEGGER_RTT_WriteString(0, "SEND: sleep failed\r\n");
#endif
                            }
                            number_of_failures_send++;
                            if (number_of_failures_send == APP_NUMBER_OF_SEND_FAILURES_BEFORE_MAINTENANCE)
                            {
#if DCW_CONF_DEBUG_ENABLED == 1
                                SEGGER_RTT_WriteString(0, "SEND: fail need join\r\n");
#endif
                                number_of_failures_send = 0;
                                vTaskResume(task_handle_radio_maintenance);
                            }
                        }
                    }
                    else
                    {
#if APP_CONF_DEBUG_ENABLED == 1
                        SEGGER_RTT_WriteString(0, "SEND: not joined need join\r\n");
#endif
                        vTaskResume(task_handle_radio_maintenance);
                    }
                }
                else
                {
#if APP_CONF_DEBUG_ENABLED == 1
                    SEGGER_RTT_WriteString(0, "SEND: check join status failed\r\n");
#endif
                }
            }
            else
            {
#if APP_CONF_DEBUG_ENABLED == 1
                SEGGER_RTT_WriteString(0, "SEND: restore session failed\r\n");
#endif
            }
        }
    }
    // Should never reach here.
    vTaskDelete(NULL);
}

//////////////////////////////////////
// Task radio maintenance
void task_radio_maintenance(void *p)
{
    // The result is set to ERROR, so when the first time the task runs, it won't suspend.

    bool first_time_maintenance = true;
    uint8_t failed_maintenance_count = 0;
#if APP_CONF_DEBUG_ENABLED == 1
    SEGGER_RTT_WriteString(0, "MTN: ready\r\n");
#endif
    for (;;)
    {
        if (maintenance_result == APP_XDOT_OK)
        {
#if APP_CONF_DEBUG_ENABLED == 2
            SEGGER_RTT_WriteString(0, "MTN: good and suspend\r\n");
#endif
            failed_maintenance_count = 0;
            execute_xdot_operation(xdot_sleep, NULL, block_time_xdot_configure);
            radio_maintaining = false;
            vTaskResume(task_handle_send);
            vTaskSuspend(NULL);
        }
        else
        {
            if (first_time_maintenance)
            { // First time. Don't sleep.
#if APP_CONF_DEBUG_ENABLED == 1
                SEGGER_RTT_WriteString(0, "MTN: first time\r\n");
#endif
                first_time_maintenance = false;
            }
            else
            {
#if APP_CONF_DEBUG_ENABLED == 1
                SEGGER_RTT_WriteString(0, "MTN: failed, sleep\r\n");
#endif
                execute_xdot_operation(xdot_sleep, NULL, block_time_xdot_configure);

                vTaskDelay(block_time_maintenance_delay_base);
                //failed_maintenance_count = (failed_maintenance_count + 1) % 32;
            }
        }

        // take the semaphore so it will follow the right timing
        xSemaphoreTake(LORA_send_sem, (TickType_t)10);

#if APP_CONF_DEBUG_ENABLED == 1
        SEGGER_RTT_WriteString(0, "MTN: maintain\r\n");
#endif
        radio_maintaining = true;
        vTaskSuspend(task_handle_send);
#if APP_CONF_DEBUG_ENABLED == 1
        SEGGER_RTT_WriteString(0, "MTN: sus send\r\n");
#endif
        xdot_wake();
        vTaskDelay(block_time_xdot_wake_delay);

        // Set public network
        execute_xdot_operation(xdot_set_public_network, APP_LORA_PUBLIC_NETWORK, block_time_xdot_configure);
        // Set join delay
        execute_xdot_operation(xdot_set_join_delay, APP_LORA_JOIN_DELAY, block_time_xdot_configure);
        // Set communication class
        execute_xdot_operation(xdot_set_tx_communication_class, APP_XDOT_COMMUNICATION_CLASS, block_time_xdot_configure);
        // Set adaptive data rate
        execute_xdot_operation(xdot_set_adaptive_data_rate, APP_LORA_ADAPTIVE_DATA_RATE, block_time_xdot_configure);
        //  Set network id
        execute_xdot_operation(xdot_set_network_id, APP_NETWORK_ID, block_time_xdot_configure);
        // Set network key
        execute_xdot_operation(xdot_set_network_key, APP_NETWORK_KEY, block_time_xdot_configure);
        // Set network band
        execute_xdot_operation(xdot_set_network_band, APP_NETWORK_BAND, block_time_xdot_configure);
        // Set data rate
        execute_xdot_operation(xdot_set_tx_data_rate, APP_LORA_TX_DATA_RATE, block_time_xdot_configure);
        // Set transimission power
        execute_xdot_operation(xdot_set_tx_power, APP_LORA_TX_POWER, block_time_xdot_configure);
        // Set antenna gain
        execute_xdot_operation(xdot_set_antenna_gain, APP_LORA_ANTENNA_GAIN, block_time_xdot_configure);
        // Set acknowlegement
        execute_xdot_operation(xdot_set_acknowledgement, APP_ACKNOWLEDGE_ATTEMPTS, block_time_xdot_configure);
        // Save configuration
        execute_xdot_operation(xdot_save_configuration, NULL, block_time_save_configuration);

        // Join
#if APP_CONF_DEBUG_ENABLED == 2
        SEGGER_RTT_WriteString(0, "MTN: join\r\n");
#endif
        printf("Blocking BLE\r\n");
        BLE_block = 1;
        sd_ble_gap_adv_stop(m_advertising.adv_handle);
        vTaskDelay(US_SUGAR_BLE_DISABLE_DELAY_MS / portTICK_RATE_MS);

        maintenance_result = execute_xdot_operation(xdot_network_join, NULL, block_time_join);
        printf("Unlocking BLE\r\n");
        BLE_block = 0;
        // TODO Enable join retry, so no need to mannually check.
        if (maintenance_result != APP_XDOT_OK)
        {
#if APP_CONF_DEBUG_ENABLED == 2
            SEGGER_RTT_WriteString(0, "MTN: join failed\r\n");
#endif
            continue;
        }
#if APP_CONF_DEBUG_ENABLED == 1
        SEGGER_RTT_WriteString(0, "MTN: save session\r\n");
#endif
        maintenance_result = execute_xdot_operation(xdot_save_session, NULL, block_time_xdot_save_session);
        if (maintenance_result != APP_XDOT_OK)
        {
#if APP_CONF_DEBUG_ENABLED == 1
            SEGGER_RTT_WriteString(0, "MTN: save session failed\r\n");
#endif
            continue;
        }
        radio_maintaining = false;
#if APP_CONF_DEBUG_ENABLED == 1
        SEGGER_RTT_WriteString(0, "MTN: done\r\n");
#endif
    }
    // Should never reach here.
    vTaskDelete(NULL);
}


//////////////////////////////////////////
// helper function

////////////////////////
// run xdot fnc
static app_code_t execute_xdot_operation(void (*xdot_op)(), char *xdot_param, TickType_t block_time)
{
    xdot_init();
    uint8_t retries = APP_RADIO_MAINTENANCE_RETRY;
    uint8_t retry_counter = 1;
    app_code_t maintenance_result;
    do
    {
        if (xdot_param == NULL)
        {
            (*xdot_op)();
        }
        else
        {
            (*xdot_op)(xdot_param);
        }
        vTaskDelay(block_time * retry_counter);
        maintenance_result = xdot_check_command_result();
        retries--;
        retry_counter++;
    } while ((maintenance_result != APP_XDOT_OK) && (retries > 0));
    xdot_seed_read();
    xdot_uninit();
    return maintenance_result;
}