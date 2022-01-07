/*
 * Either COMPILE_RIGHT or COMPILE_LEFT has to be defined from the make call to allow proper functionality
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "redox-w.h"
#include "sdk_config.h"
#include "app_error.h"
#include "app_uart.h"
#include "nrf_gzll.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_uart.h"
#include "nrf51_bitfields.h"
#include "nrf51.h"


/*****************************************************************************/
/** Configuration */
/*****************************************************************************/

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC1. */


// Data and acknowledgement payloads
static uint8_t ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; ///< Placeholder for received ACK payloads from Host.

// Debounce time (dependent on tick frequency)
#define DEBOUNCE 5
// Mark as inactive after a number of ticks:
#define INACTIVITY_THRESHOLD 500 // 0.5sec
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

#ifdef COMPILE_LEFT
static uint8_t channel_table[3]={4, 42, 77};
#endif
#ifdef COMPILE_RIGHT
static uint8_t channel_table[3]={25, 63, 33};
#endif
#ifdef COMPILE_DEBUG
static uint8_t channel_table[3]={25, 63, 33};
#endif


/*****************************************************************************/
/** Code */
/*****************************************************************************/

// Forward declarations
static void uart_error_handle(app_uart_evt_t * p_event);


// Setup switch pins with pullups
static void gpio_config(void)
{
    nrf_gpio_cfg_sense_input(R01, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R02, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R03, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R04, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R05, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);

    nrf_gpio_cfg_output(C01);
    nrf_gpio_cfg_output(C02);
    nrf_gpio_cfg_output(C03);
    nrf_gpio_cfg_output(C04);
    nrf_gpio_cfg_output(C05);
    nrf_gpio_cfg_output(C06);
    nrf_gpio_cfg_output(C07);

    nrf_gpio_pin_clear(C01);
    nrf_gpio_pin_clear(C02);
    nrf_gpio_pin_clear(C03);
    nrf_gpio_pin_clear(C04);
    nrf_gpio_pin_clear(C05);
    nrf_gpio_pin_clear(C06);
    nrf_gpio_pin_clear(C07);
}

// Return the key states
static void read_keys(uint8_t *row_stat)
{
    unsigned short c;
    uint32_t input = 0;
    static const uint32_t COL_PINS[] = { C01, C02, C03, C04, C05, C06, C07 };

    // scan matrix by columns
    for (c = 0; c < COLUMNS; ++c) {
        // Force the compiler to add one cycle gap between activating
        // the column pin and reading the input to allow some time for
        // it to be come stable. Note that compile optimizations are
        // not allowed for next three statements. Setting the pin
        // write a location which is marked as volatile
        // (NRF_GPIO->OUTSET) and reading the input is also a memory
        // access to a location which is marked as volatile too
        // (NRF_GPIO->IN).
        nrf_gpio_pin_set(COL_PINS[c]);
        asm volatile("nop");
        input = NRF_GPIO->IN;
        row_stat[0] = (row_stat[0] << 1) | ((input >> R01) & 1);
        row_stat[1] = (row_stat[1] << 1) | ((input >> R02) & 1);
        row_stat[2] = (row_stat[2] << 1) | ((input >> R03) & 1);
        row_stat[3] = (row_stat[3] << 1) | ((input >> R04) & 1);
        row_stat[4] = (row_stat[4] << 1) | ((input >> R05) & 1);
        nrf_gpio_pin_clear(COL_PINS[c]);
    }

}

static bool empty_keys(const uint8_t* keys_buffer)
{
    for(int i=0; i < ROWS; i++)
    {
        if (keys_buffer[i])
        {
          return false;
        }
    }
    return true;
}

static void handle_inactivity(const uint8_t *keys_buffer)
{
    static uint32_t inactivity_ticks = 0;

    // looking for 500 ticks of no keys pressed, to go back to deep sleep
    if (empty_keys(keys_buffer)) {
        inactivity_ticks++;
        if (inactivity_ticks > INACTIVITY_THRESHOLD) {
            nrf_drv_rtc_disable(&rtc);
            nrf_gpio_port_out_set(NRF_GPIO,
                (1UL << C01) |
                (1UL << C02) |
                (1UL << C03) |
                (1UL << C04) |
                (1UL << C05) |
                (1UL << C06) |
                (1UL << C07));

            inactivity_ticks = 0;

            NRF_POWER->SYSTEMOFF = 1;
        }
    } else {
        inactivity_ticks = 0;
    }
}

static void handle_send(const uint8_t* keys_buffer)
{
    static uint8_t keys_snapshot[ROWS] = {0};
    static uint32_t debounce_ticks = 0;

    const bool no_change = !memcmp(keys_buffer, keys_snapshot, ROWS);
    if (no_change) {
        debounce_ticks++;
        // debouncing - send only if the keys state has been stable
        // for DEBOUNCE ticks
        if (debounce_ticks == DEBOUNCE) {
            // Assemble packet and send to receiver
            nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, keys_snapshot, ROWS);
            debounce_ticks = 0;
        }
    } else {
        // change detected, start over
        debounce_ticks = 0;
        memcpy(keys_snapshot, keys_buffer, ROWS);
    }
}

// 1000Hz debounce sampling
static void tick(nrf_drv_rtc_int_type_t int_type)
{
    uint8_t keys_buffer[ROWS] = {0, 0, 0, 0, 0};
    read_keys(keys_buffer);

    handle_inactivity(keys_buffer);

    handle_send(keys_buffer);
}

// Low frequency clock configuration
static void lfclk_config(void)
{
    nrf_drv_clock_init();

    nrf_drv_clock_lfclk_request(NULL);
}

// RTC peripheral configuration
static void rtc_config(void)
{
    enum {
        RTC_FREQUENCY          = 1000,
        RTC_MAXIMUM_LATENCY_US = 2000,
    };
    static nrf_drv_rtc_config_t config = {
        .prescaler          = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY),
        .interrupt_priority = APP_IRQ_PRIORITY_LOW,
        .reliable           = false,
        .tick_latency       = RTC_US_TO_TICKS(RTC_MAXIMUM_LATENCY_US, RTC_FREQUENCY),
    };

    //Initialize RTC instance
    nrf_drv_rtc_init(&rtc, &config, tick);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc, true);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

// UART debug logging configuration
static void uart_config(void)
{
    uint32_t err_code;
    static const app_uart_comm_params_t comm_params =
    {
        .rx_pin_no    = 11,
        .tx_pin_no    =  9,
        .rts_pin_no   =  8,
        .cts_pin_no   = 10,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

int main(void)
{
#ifdef COMPILE_DEBUG
    uart_config();
    printf("\n\rHello, kbd fans!\n\r");
#endif

    // Initialize Gazell
#ifdef COMPILE_DEBUG
    printf("Configure Gazell...\n\r");
#endif
    nrf_gzll_init(NRF_GZLL_MODE_DEVICE);

    // Attempt sending every packet up to 100 times
    nrf_gzll_set_max_tx_attempts(100);
    nrf_gzll_set_timeslots_per_channel(4);
    nrf_gzll_set_channel_table(channel_table,3);
    nrf_gzll_set_datarate(NRF_GZLL_DATARATE_1MBIT);
    nrf_gzll_set_timeslot_period(900);

    // Addressing
//#ifndef COMPILE_DEBUG
//    nrf_gzll_set_base_address_0(0x01020304);
//    nrf_gzll_set_base_address_1(0x05060708);
//#else
    nrf_gzll_set_base_address_0(0x04030201);
    nrf_gzll_set_base_address_1(0x08070605);
//#endif

    // Enable Gazell to start sending over the air
    nrf_gzll_enable();

    // Configure 32kHz xtal oscillator
#ifdef COMPILE_DEBUG
    printf("Configure 32kHz xtal oscillator...\n\r");
#endif
    lfclk_config();

    // Configure all keys as inputs with pullups
#ifdef COMPILE_DEBUG
    printf("Configure GPIO...\n\r");
#endif
    gpio_config();

    // Configure RTC peripherals with ticks
#ifdef COMPILE_DEBUG
    printf("Configure RTC...\n\r");
#endif
    rtc_config();

    // Main loop, constantly sleep, waiting for RTC and gpio IRQs
#ifdef COMPILE_DEBUG
    printf("Well done!\n\r");
#endif
    while(1)
    {
        __SEV();
        __WFE();
        __WFE();
    }
#ifdef COMPILE_DEBUG
    printf("Bye!\n\r");
#endif
}


/*****************************************************************************/
/** UART callback function definitions  */
/*****************************************************************************/

static void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/*****************************************************************************/
/** Gazell callback function definitions  */
/*****************************************************************************/

void  nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    uint32_t ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    if (tx_info.payload_received_in_ack)
    {
        // Pop packet and write first byte of the payload to the GPIO port.
        nrf_gzll_fetch_packet_from_rx_fifo(pipe, ack_payload, &ack_payload_length);
    }
}

// no action is taken when a packet fails to send, this might need to change
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{

}

// Callbacks not needed
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{}
void nrf_gzll_disabled()
{}
