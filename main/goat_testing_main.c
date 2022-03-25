#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "components\sht31.h"
#include "components\lis3dh\lis3dh.h"

#include "components\spi_lr1110_esp.h"
#include "components\configuration.h"

#include "components\lr1110\lr1110_gnss.h"
#include "components\lr1110\lr1110_regmem.h"
#include "components\lr1110\lr1110_hal.h"

static const char *TAG = "goat_testing_main";

// I2C config parameters
#define I2C_MASTER_SCL_IO           5                           /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           4                           /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// SPI config parameters
#define LR1110_HOST    SPI2_HOST
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 7
#define PIN_NUM_CLK  6
#define PIN_NUM_CS   10

// LR1110 SPI config parameters
#define SNIFF_LED 3

static lis3dh_sensor_t* accelerometer;

/*
radio_t radio = {
    SPI1,
    { LR1110_NSS_PORT, LR1110_NSS_PIN },
    { LR1110_RESET_PORT, LR1110_RESET_PIN },
    { LR1110_IRQ_PORT, LR1110_IRQ_PIN },
    { LR1110_BUSY_PORT, LR1110_BUSY_PIN },
};
*/

lr1110_gnss_version_t* version;

// Initialize I2C bus
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*
// Init SPI bus
static void spi_master_init(void)
{

    esp_err_t ret;

    spi_device_handle_t spi;

    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    //Initialize the SPI bus
    ret = ESP_ERROR_CHECK(spi_bus_initialize(LR1110_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 4000000,
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    };

    //Attach LR1110 to the SPI bus
    ret=spi_bus_add_device(LR1110_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}
*/

// Initialize and configure lis3dh accelerometer 
void init_accel()
{
    // Initialize lis3dh accelerometer 
    accelerometer = lis3dh_init_sensor (0, LIS3DH_I2C_ADDRESS_1, 0);
    ESP_LOGI(TAG, "Sensor initialized");

    // configure HPF and reset the reference by dummy read
    lis3dh_config_hpf (accelerometer, lis3dh_hpf_normal, 0, true, true, true, true);
    lis3dh_get_hpf_ref (accelerometer);
    
    // enable ADC inputs and temperature sensor for ADC input 3
    lis3dh_enable_adc (accelerometer, true, true);
    
    // LAST STEP: Finally set scale and mode to start measurements
    lis3dh_set_scale(accelerometer, lis3dh_scale_2_g);
    lis3dh_set_mode (accelerometer, lis3dh_odr_10, lis3dh_high_res, true, true, true);
}

// Read acclerometer data and output over serial port
void read_accel()
{
    lis3dh_float_data_t  data;
    lis3dh_get_float_data(accelerometer, &data);
    ESP_LOGI(TAG, "%.3f LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
               (double)esp_timer_get_time()*1e-3, 
                data.ax, data.ay, data.az);
}

//Read temp and humidity and output over serial port
void read_temp_humidty()
{
    float temp, humi;
    ESP_ERROR_CHECK(sht31_read_temp_humi(&temp, &humi));
    ESP_LOGI(TAG, "temp = %.2f, humi = %.2f\n", temp, humi);
}

/*
lr1110_status_t lr1110_gnss_read_firmware_version( const void* context, lr1110_gnss_version_t* version )
{
    const uint8_t cbuffer[LR1110_GNSS_READ_FW_VERSION_CMD_LENGTH] = {
        ( uint8_t )( LR1110_GNSS_READ_FW_VERSION_OC >> 8 ),
        ( uint8_t )( LR1110_GNSS_READ_FW_VERSION_OC >> 0 ),
    };
    uint8_t rbuffer[LR1110_GNSS_READ_FIRMWARE_VERSION_RBUFFER_LENGTH] = { 0 };

    const lr1110_hal_status_t hal_status = lr1110_hal_read( context, cbuffer, LR1110_GNSS_READ_FW_VERSION_CMD_LENGTH,
                                                            rbuffer, LR1110_GNSS_READ_FIRMWARE_VERSION_RBUFFER_LENGTH );

    version->gnss_firmware = rbuffer[0];
    version->gnss_almanac  = rbuffer[1];
    return ( lr1110_status_t ) hal_status;
}
*/

void app_main(void)
{
    // Init I2C bus
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Init SPI bus
    //spi_master_init();

    // Init lis3dh accelerometer
    init_accel();

    // Init Lr1110 LED
    gpio_reset_pin(SNIFF_LED);
    gpio_set_direction(SNIFF_LED, GPIO_MODE_OUTPUT);

    while(true) {

        read_accel();
        
        read_temp_humidty();

        gpio_set_level(SNIFF_LED, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
        gpio_set_level(SNIFF_LED, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
