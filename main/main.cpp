#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "sdkconfig.h"

// SD-Card ////////////////////
#include <sys/unistd.h>
#include <sys/stat.h>
#include "nvs_flash.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
// #include "driver/sdspi_host.h"
// #include "driver/sdmmc_host.h"
// #include "driver/sdmmc_defs.h"

#define PIN_NUM_MISO 22
#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK 21
#define PIN_NUM_CS 0
#define SPI_DMA_CHAN 1

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
#include "Helper.h"

// #include "jomjol_WS2812Slow.h"
#include "SmartLeds.h"

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
    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_19,
        .miso_io_num = GPIO_NUM_22,
        .sclk_io_num = GPIO_NUM_21,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 10000,
    };
    ret = spi_bus_initialize(HSPI_HOST, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAGMAIN, "Failed to initialize bus.");
        return false;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    // sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_0;
    slot_config.host_id = SPI2_HOST;


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
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
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
    vTaskDelete(NULL); // Delete this task if it exits from the loop above
}

extern "C" void app_main(void)
{
    TickType_t xDelay;

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

    gpio_handler_create(server);

    printf("vor reg server main\n");
    register_server_main_uri(server, "/sdcard");

    printf("vor dotautostart\n");

    // init camera module
    printf("Do Reset Camera\n");
    PowerResetCamera();

    size_t _hsize = getESPHeapSize();
    if (_hsize < 4000000)
    {
        std::string _zws = "Not enought PSRAM available. Expected 4.194.304 MByte - available: " + std::to_string(_hsize);
        _zws = _zws + "\nEither not initialzed or too small (2MByte only) or not present at all. Firmware cannot start!!";
        printf(_zws.c_str());
        LogFile.SwitchOnOff(true);
        LogFile.WriteToFile(_zws);
        LogFile.SwitchOnOff(false);
    }
    else
    {
        esp_err_t cam = Camera.InitCam();
        if (cam != ESP_OK)
        {
            ESP_LOGE(TAGMAIN, "Failed to initialize camera module. "
                              "Check that your camera module is working and connected properly.");

            LogFile.SwitchOnOff(true);
            LogFile.WriteToFile("Failed to initialize camera module. "
                                "Check that your camera module is working and connected properly.");
            LogFile.SwitchOnOff(false);
        }
        else
        {
            // Test Camera
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb)
            {
                ESP_LOGE(TAGMAIN, "esp_camera_fb_get: Camera Capture Failed");
                LogFile.SwitchOnOff(true);
                LogFile.WriteToFile("Camera cannot be initialzed. "
                                    "System will reboot.");
                doReboot();
            }
            esp_camera_fb_return(fb);

            Camera.LightOnOff(false);
            TFliteDoAutoStart();
        }
    }
}
