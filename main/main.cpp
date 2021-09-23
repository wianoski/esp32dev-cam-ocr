#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

// SD-Card ////////////////////
#include "nvs_flash.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
// #include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
///////////////////////////////

#include "ClassLogFile.h"

#include "connect_wlan.h"
#include "read_wlanini.h"

#include "server_main.h"
#include "server_tflite.h"
#include "server_file.h"
#include "server_ota.h"
#include "time_sntp.h"
#include "ClassControllCamera.h"
#include "server_main.h"
#include "server_camera.h"

#include "../st7789/main/decode_jpeg.h"
#include "../st7789/main/st7789.h"
#include "../st7789/main/bmpfile.h"
#include "../st7789/main/pngle.h"
#define CONFIG_WIDTH 240
#define CONFIG_HEIGHT 240
static const int SPI_Frequency = SPI_MASTER_FREQ_20M;
#ifdef CONFIG_IDF_TARGET_ESP32
#define LCD_HOST HSPI_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define LCD_HOST SPI2_HOST
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define LCD_HOST SPI2_HOST
#endif

#define __SD_USE_ONE_LINE_MODE__

#include "server_GPIO.h"

#define BLINK_GPIO GPIO_NUM_33

static const char *TAGMAIN = "main";

//#define FLASH_GPIO GPIO_NUM_4

bool Init_NVS_SDCard()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ////////////////////////////////////////////////

    ESP_LOGI(TAGMAIN, "Using SDSPI peripheral");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    // sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode, uncomment the following line:

#ifdef __SD_USE_ONE_LINE_MODE__
    slot_config.gpio_cs = GPIO_NUM_0;
    slot_config.gpio_miso = GPIO_NUM_22;
    slot_config.gpio_mosi = GPIO_NUM_19;
    slot_config.gpio_sck = GPIO_NUM_21;

#endif

    //pin for ttgo-camera plus
    /*
    CMD  ---> CMD-MOSI-19
    CLK  ---> CLK-SCK-21
    DAT0 ---> SDO-MISO-22
    */

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode(GPIO_NUM_19, GPIO_PULLUP_ONLY); // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(GPIO_NUM_22, GPIO_PULLUP_ONLY); // D0, needed in 4- and 1-line modes
#ifndef __SD_USE_ONE_LINE_MODE__
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);  // D1, needed in 4-line mode only
    gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY); // D2, needed in 4-line mode only
#endif
    gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY); // D3, needed in 4- and 1-line modes

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
    // Please check its source code and implement error recovery when developing
    // production applications.
    sdmmc_card_t *card;
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAGMAIN, "Failed to mount filesystem. "
                              "If you want the card to be formatted, set format_if_mount_failed = true.");
        }
        else
        {
            ESP_LOGE(TAGMAIN, "Failed to initialize the card (%s). "
                              "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return false;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Init the GPIO
    // Flash ausschalten

    // gpio_pad_select_gpio(FLASH_GPIO);
    // gpio_set_direction(FLASH_GPIO, GPIO_MODE_OUTPUT);
    // gpio_set_level(FLASH_GPIO, 0);

    return true;
}

void task_NoSDBlink(void *pvParameter)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    TickType_t xDelay;
    xDelay = 100 / portTICK_PERIOD_MS;
    printf("SD-Card could not be inialized - STOP THE PROGRAMM HERE\n");

    while (1)
    {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(xDelay);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(xDelay);
    }
    vTaskDelete(NULL); //Delete this task if it exits from the loop above
}

void spi_master_init(TFT_t *dev)
{
    esp_err_t ret;
    int16_t GPIO_MOSI = 19;
    int16_t GPIO_SCLK = 21;
    int16_t GPIO_CS = 12;
    int16_t GPIO_DC = 15;
    int16_t GPIO_RESET = -1;
    int16_t GPIO_BL = 2;
    printf("GPIO_CS=%d", GPIO_CS);
    if (GPIO_CS >= 0)
    {
        //gpio_pad_select_gpio( GPIO_CS );
        gpio_reset_pin((gpio_num_t)GPIO_CS);
        gpio_set_direction((gpio_num_t)GPIO_CS, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)GPIO_CS, 0);
    }

    printf("GPIO_DC=%d", GPIO_DC);
    //gpio_pad_select_gpio( GPIO_DC );
    gpio_reset_pin((gpio_num_t)GPIO_DC);
    gpio_set_direction((gpio_num_t)GPIO_DC, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)GPIO_DC, 0);

    printf("GPIO_RESET=%d", GPIO_RESET);
    if (GPIO_RESET >= 0)
    {
        //gpio_pad_select_gpio( GPIO_RESET );
        gpio_reset_pin((gpio_num_t)GPIO_RESET);
        gpio_set_direction((gpio_num_t)GPIO_RESET, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)GPIO_RESET, 1);
        delayMS(50);
        gpio_set_level((gpio_num_t)GPIO_RESET, 0);
        delayMS(50);
        gpio_set_level((gpio_num_t)GPIO_RESET, 1);
        delayMS(50);
    }

    printf("GPIO_BL=%d", GPIO_BL);
    if (GPIO_BL >= 0)
    {
        //gpio_pad_select_gpio(GPIO_BL);
        gpio_reset_pin((gpio_num_t)GPIO_BL);
        gpio_set_direction((gpio_num_t)GPIO_BL, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)GPIO_BL, 0);
    }

    printf("GPIO_MOSI=%d", GPIO_MOSI);
    printf("GPIO_SCLK=%d", GPIO_SCLK);
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = 22,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = 0};

    ret = spi_bus_initialize(LCD_HOST, &buscfg, 1);
    printf("spi_bus_initialize=%d\n", ret);
    assert(ret == ESP_OK);

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = SPI_Frequency;
    devcfg.queue_size = 7;
    devcfg.mode = 2;
    devcfg.flags = SPI_DEVICE_NO_DUMMY;

    if (GPIO_CS >= 0)
    {
        devcfg.spics_io_num = GPIO_CS;
    }
    else
    {
        devcfg.spics_io_num = -1;
    }

    spi_device_handle_t handle;
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &handle);
    // printf("spi_bus_add_device=%d",ret);
    assert(ret == ESP_OK);
    dev->_dc = GPIO_DC;
    dev->_bl = GPIO_BL;
    dev->_SPIHandle = handle;
}

extern "C" void app_main(void)
{
    printf("Do Reset Camera\n");
    PowerResetCamera();
    Camera.InitCam();
    Camera.LightOnOff(false);

    if (!Init_NVS_SDCard())
    {
        xTaskCreate(&task_NoSDBlink, "task_NoSDBlink", configMINIMAL_STACK_SIZE * 64, NULL, tskIDLE_PRIORITY + 1, NULL);
        return;
    };

    CheckOTAUpdate();

    char *ssid = NULL, *passwd = NULL, *hostname = NULL, *ip = NULL, *gateway = NULL, *netmask = NULL, *dns = NULL;
    LoadWlanFromFile("/sdcard/wlan.ini", ssid, passwd, hostname, ip, gateway, netmask, dns);

    if (ssid != NULL && passwd != NULL)
        printf("\nWLan: %s, %s\n", ssid, passwd);
    else
        printf("No SSID and PASSWORD set!!!");

    if (hostname != NULL)
        printf("Hostename: %s\n", hostname);
    else
        printf("Hostname not set.\n");

    if (ip != NULL && gateway != NULL && netmask != NULL)
        printf("Fixed IP: %s, Gateway %s, Netmask %s\n", ip, gateway, netmask);
    if (dns != NULL)
        printf("DNS IP: %s\n", dns);

    wifi_init_sta(ssid, passwd, hostname, ip, gateway, netmask, dns);

    TickType_t xDelay;
    xDelay = 2000 / portTICK_PERIOD_MS;
    printf("main: sleep for : %ldms\n", (long)xDelay);
    //    LogFile.WriteToFile("Startsequence 06");
    vTaskDelay(xDelay);
    //    LogFile.WriteToFile("Startsequence 07");
    setup_time();
    setBootTime();
    LogFile.WriteToFile("=============================================================================================");
    LogFile.WriteToFile("=================================== Main Started ============================================");
    LogFile.WriteToFile("=============================================================================================");
    LogFile.SwitchOnOff(false);

    std::string zw = gettimestring("%Y%m%d-%H%M%S");
    printf("time %s\n", zw.c_str());

    //    Camera.InitCam();
    //    Camera.LightOnOff(false);
    xDelay = 2000 / portTICK_PERIOD_MS;
    printf("main: sleep for : %ldms\n", (long)xDelay);
    vTaskDelay(xDelay);

    server = start_webserver();
    register_server_camera_uri(server);
    register_server_tflite_uri(server);
    register_server_file_uri(server, "/sdcard");
    register_server_ota_sdcard_uri(server);

    // TFT_t dev;
    // printf("Start SPI for TFT \n");
    // spi_master_init(&dev);

    gpio_handler_create(server);
    printf("Start server main !!!\n");
    register_server_main_uri(server, "/sdcard");

    printf("Start TFLite !!!\n");
    TFliteDoAutoStart();
}