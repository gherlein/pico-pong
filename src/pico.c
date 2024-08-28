#include "pico.h"

void SPI_Init(spi_inst_t *spi)
{

    spi_init(spi, 1000 * 1000);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);

    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Set SPI format
    spi_set_format(spi, // SPI instance
                   8,   // Number of bits per transfer
                   0,   // Polarity (CPOL)
                   0,   // Phase (CPHA)
                   SPI_MSB_FIRST);

    return;
}

void GPIO_Init()
{
    // RADIO_BUSY is an input
    gpio_init(RADIO_BUSY);
    gpio_set_dir(RADIO_NSS, GPIO_IN);

    // Reset is active-low
    gpio_init(RADIO_RESET);
    gpio_set_dir(RADIO_RESET, GPIO_OUT);
    gpio_put(RADIO_RESET, 1);

    // Chip select NSS is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    // Make the CS pin available to picotool
    // bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));
}

static inline void cs_select(void)
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0); // Active low
    asm volatile("nop \n nop \n nop");
    // sleep_ms(1);
}

static inline void cs_deselect(void)
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
    // sleep_ms(1);
}
