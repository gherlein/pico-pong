/*!
 * \file      main.c
 *
 * \brief     RPi Pico Ping-Pong implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 * Derived from code in this zip file:  https://files.waveshare.com/wiki/RP2040-LoRa/Rp2040-lora-code.zip
 * Found the code in this wiki: https://www.waveshare.com/wiki/RP2040-LoRa
 * \endcode
 *
 * \author    Greg Herlein - gherlein@herlein.com - https://www.linkedin.com/in/gherlein/
 *
 */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "radio-config.h"

#include "radio.h"
#include "sx126x.h"
#include "sx126x-board.h"

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

#define TX_OUTPUT_POWER 22 // dBm
#define USE_MODEM_LORA

#if defined(USE_MODEM_LORA)

#define LORA_BANDWIDTH 0        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 5   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#elif defined(USE_MODEM_FSK)

#define FSK_FDEV 25000          // Hz
#define FSK_DATARATE 50000      // bps
#define FSK_BANDWIDTH 50000     // Hz
#define FSK_AFC_BANDWIDTH 83333 // Hz
#define FSK_PREAMBLE_LENGTH 5   // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON false

#else
#error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
} States_t;

#define RX_TIMEOUT_VALUE 2000
#define BUFFER_SIZE 64 // Define the payload size here

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

// start status,send a packet
States_t State = RX_ERROR;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * Radio events function pointers
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

uint32_t wk_time;
uint16_t irq_status;

RadioError_t device_error;
RadioStatus_t device_status;

/**
 * Main application entry point.
 */
int main(void)
{
    bool isMaster = true;
    uint8_t i;

    // workaround for a hardware debug problem
    // https://forums.raspberrypi.com/viewtopic.php?t=363914
    // also in openocd you need to us -c "set USE_CORE 0"  before rp2040.cfg is loaded per https://github.com/raspberrypi/debugprobe/issues/45
    timer_hw->dbgpause = 0;
    sleep_ms(150);

    stdio_init_all();

    printf("RADIO_RESET: %d\n", RADIO_RESET);
    printf("RADIO_MOSI : %d %d\n", RADIO_MOSI, PICO_DEFAULT_SPI_TX_PIN);
    printf("RADIO_MISO : %d %d \n", RADIO_MISO, PICO_DEFAULT_SPI_RX_PIN);
    printf("RADIO_SCLK:  %d %d \n", RADIO_SCLK, PICO_DEFAULT_SPI_SCK_PIN);
    printf("RADIO_NSS :  %d %d\n", RADIO_NSS, PICO_DEFAULT_SPI_CSN_PIN);
    printf("RADIO_BUSY : %d \n", RADIO_BUSY);
    printf("RADIO_DIO_1: %d\n", RADIO_DIO_1);
    printf("default spi: %d\n", spi_default);
    printf("spi0       : %d\n", spi0);

    fflush(stdout);
    puts("");

    // initialize SPI
    spi_init(spi0, 1000 * 1000);
    gpio_set_function(RADIO_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(RADIO_MISO, GPIO_FUNC_SPI);
    gpio_set_function(RADIO_SCLK, GPIO_FUNC_SPI);

    // initialize the LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents, spi0);

    while (1)
    {
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
    }

    // RP2040-LoRa-LF
    //    Radio.SetChannel(433000000);
    // RP2040-LoRa-HF
    Radio.SetChannel(915000000);

#if defined(USE_MODEM_LORA)

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);

#elif defined(USE_MODEM_FSK)

    Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, 0, 3000);

    Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                      0, 0, false, true);

    Radio.SetMaxPayloadLength(MODEM_FSK, BUFFER_SIZE);

#else
#error "Please define a frequency band in the compiler options."
#endif

#define TEST_RADIO
#ifdef TEST_RADIO
    Buffer[0] = 'P';
    Buffer[1] = 'I';
    Buffer[2] = 'N';
    Buffer[3] = 'G';
    BufferSize = 4;

    Radio.Rx(RX_TIMEOUT_VALUE);
    // SX126xAntSwOn();
    while (1)
    {
        // Send the next PING frame
        gpio_put(LED_PIN, 0);
        Radio.Send(Buffer, BufferSize);
        sleep_ms(250);
        printf(".");
        fflush(stdout);
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
    }
#endif

// #define RADIO_ENABLE
#ifdef RADIO_ENABLE

    c while (1)
    {
        // printf(".");
        switch (State)
        {
        case RX:
            if (isMaster == true)
            {
                if (BufferSize > 0)
                {
                    if (strncmp((const char *)Buffer, (const char *)PongMsg, 4) == 0)
                    {
                        // Indicates on a LED that the received frame is a PONG
                        gpio_put(LED_PIN, 1);
                        sleep_ms(25);
                        gpio_put(LED_PIN, 0);

                        // Send the next PING frame
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for (i = 4; i < BufferSize; i++)
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs(1);
                        SX126xAntSwOff();
                        Radio.Send(Buffer, BufferSize);
                    }
                    else if (strncmp((const char *)Buffer, (const char *)PingMsg, 4) == 0)
                    { // A master already exists then become a slave
                        isMaster = false;
                        gpio_put(LED_PIN, 1);
                        sleep_ms(25);
                        gpio_put(LED_PIN, 0);
                        SX126xAntSwOn();
                        Radio.Rx(RX_TIMEOUT_VALUE);
                    }
                    else // valid reception but neither a PING or a PONG message
                    {    // Set device as master ans start again
                        isMaster = true;
                        SX126xAntSwOn();
                        Radio.Rx(RX_TIMEOUT_VALUE);
                    }

                    printf("%s\r\n", Buffer);
                }
            }
            else
            {
                if (BufferSize > 0)
                {
                    if (strncmp((const char *)Buffer, (const char *)PingMsg, 4) == 0)
                    {
                        // Indicates on a LED that the received frame is a PING
                        gpio_put(LED_PIN, 1);
                        sleep_ms(25);
                        gpio_put(LED_PIN, 0);

                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for (i = 4; i < BufferSize; i++)
                        {
                            Buffer[i] = i - 4;
                        }
                        SX126xAntSwOff();
                        DelayMs(1);
                        Radio.Send(Buffer, BufferSize);
                    }
                    else // valid reception but not a PING as expected
                    {    // Set device as master and start again
                        isMaster = true;
                        SX126xAntSwOn();
                        Radio.Rx(RX_TIMEOUT_VALUE);
                    }

                    printf("%s\r\n", Buffer);
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            SX126xAntSwOn();
            DelayMs(1);
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
            if (isMaster == true)
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for (i = 4; i < BufferSize; i++)
                {
                    Buffer[i] = i - 4;
                }
                // DelayMs( 1 );
                SX126xAntSwOff();
                DelayMs(1);
                Radio.Send(Buffer, BufferSize);
            }
            else
            {
                // Send the next PONG frame
                Buffer[0] = 'P';
                Buffer[1] = 'O';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for (i = 4; i < BufferSize; i++)
                {
                    Buffer[i] = i - 4;
                }
                // DelayMs( 1 );
                SX126xAntSwOff();
                DelayMs(1);
                Radio.Send(Buffer, BufferSize);

                // SX126xAntSwOn();
                // Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;

        case RX_ERROR:
            if (isMaster == true)
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for (i = 4; i < BufferSize; i++)
                {
                    Buffer[i] = i - 4;
                }
                // DelayMs( 1 );
                SX126xAntSwOff();
                DelayMs(1);
                Radio.Send(Buffer, BufferSize);
            }
            else
            {
                SX126xAntSwOn();
                DelayMs(1);
                Radio.Rx(RX_TIMEOUT_VALUE);
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            SX126xAntSwOn();
            DelayMs(1);
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case LOWPOWER:
            // printf("low power test\r\n");
            break;
        default:
            // Set low power
            break;
        }

        // device_error = SX126xGetDeviceErrors();

        // device_status = SX126xGetStatus();

        BoardLowPowerHandler();
        // Process Radio IRQ
        if (Radio.IrqProcess != NULL)
        {
            Radio.IrqProcess();
        }
    }

#endif // RADIO_ENABLE
}

void OnTxDone(void)
{
    Radio.Sleep();
    State = TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep();
    BufferSize = size;
    memcpy(Buffer, payload, BufferSize);
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
    printf("Rssi is -%ddBm\r\n", -RssiValue / 2);
    printf("Snr is %ddB\r\n", SnrValue / 4);
}

void OnTxTimeout(void)
{
    Radio.Sleep();
    State = TX_TIMEOUT;
    printf("OnTxTimeout\r\n");
    SX126xClearDeviceErrors();
}

void OnRxTimeout(void)
{
    Radio.Sleep();
    State = RX_TIMEOUT;
    printf("OnRxTimeout\r\n");
    SX126xClearDeviceErrors();
}

void OnRxError(void)
{
    Radio.Sleep();
    State = RX_ERROR;
    printf("OnRxError\r\n");
}
