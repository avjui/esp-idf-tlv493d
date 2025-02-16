// Copyright (C) 2025 Juen Rene´
//
// This file is part of esp-idf-tlv493d.
//
// esp-idf-tlv493d is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// esp-idf-tlv493d is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with esp-idf-tlv493d.  If not, see <https://www.gnu.org/licenses/>.
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "rom/ets_sys.h"

#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(5, 2, 0)
#include "driver/i2c_master.h"
#else
#include "driver/i2c.h"
#endif

#include "tlv493d_defs.h"

#define MODUL_TLV "TLV493D"
#define MODUL_THREAD_TLV "TLV493D-THREAD"
#define DEFAULT_TLV493D_CONFIG()                       \
    {                                                  \
        .dataX = 0.0,                                  \
        .dataY = 0.0,                                  \
        .dataZ = 0.0,                                  \
        .temperature = 0.0,                            \
        .amount = 0.0,                                 \
        .azimuth = 0.0,                                \
        .polar = 0.0,                                  \
        .tlv493d_conf = {                              \
            .tlv_address = CONFIG_TLV493D_I2C_ADDRESS, \
            .pin_sda = CONFIG_TLV493D_PIN_SDA,         \
            .pin_scl = CONFIG_TLV493D_PIN_SCL,         \
            .mode = ULTRALOWPOWERMODE,                          \
            .use_temp = CONFIG_TLV493D_TEMP_ENABLE,    \
            .int_enable = CONFIG_TLV493D_INT_ENABLE,   \
        },                                             \
    }

    /**
     * @enum tlv493d_mode_t
     * @brief  Enum for powermodes
     *
     * @note MASTERCONTROLLERMODE and MASTERCONTROLLERMODE_ULP are not supported for the moment
     *
     * @var tlv493d_mode_t::POWERDOWNMODE
     * Used for disabling tlv493D. It will use 3nA in this mode(see datasheet).
     *
     * @var tlv493d_mode_t::FASTMODE
     * In this mode the value is only 8 bit accuracy but the highest read speed.
     *
     * @var tlv493d_mode_t::LOWPOWERMODE
     * Read out data ever 12ms
     *
     * @var tlv493d_mode_t::ULTRALOWPOWERMODE
     * Read out data ever 100ms
     *
     * @var tlv493d_mode_t::MASTERCONTROLLERMODE
     * Read out data controlled be cpu. This mode has the same speed as fastmode.
     * This mode can be used interrupt driven or polled.a64l
     * @note At the moment only polled mode can be enabled until we find a solution
     *       to connect sda pin to interrupt routine. Maybe this can be done by 'gpio matrix'.
     *
     * @var tlv493d_mode_t:: MASTERCONTROLLERMODE_ULP
     * @warning Is not supported yet!
     */
    typedef enum
    {
        POWERDOWNMODE,           /* Settings: fastmode = 0, lp_mode = 0, int_out = 0 */
        FASTMODE,                /* Settings: fastmode = 1, lp_mode = 0, int_out = 1 */
        LOWPOWERMODE,            /* Settings: fastmode = 0, lp_mode = 1, int_out = 1 */
        ULTRALOWPOWERMODE,       /* Settings: fastmode = 0, lp_mode = 0, int_out = 1 */
        MASTERCONTROLLERMODE,    /* Settings: fastmode = 1, lp_mode = 1, int_out = 0/1 */
        MASTERCONTROLLERMODE_ULP /* not supported yet */
    } tlv493d_mode_t;

    /**
     * @struct tlv493d_io_conf_t
     * @brief Struct to hold bus information data for the tlv493d
     *
     * @var tlv493d_io_conf_t::tlv_address
     * assign I2C address (default 0x5E)
     *
     * @var tlv493d_io_conf_t::pin_sda
     * assign i2c sda pin
     *
     * @var tlv493d_io_conf_t::pin_scl
     * assign i2c scl pin
     *
     * @var tlv493d_io_conf_t::mode
     * set the used powermode
     *
     * @var tlv493d_io_conf_t::use_temp
     * enable/disable temperature read out
     *
     * @var tlv493d_io_conf_t::int_enable
     * enable/disable interrupt pin output
     *
     */
    typedef struct
    {
        uint8_t tlv_address;
        uint8_t pin_sda;
        uint8_t pin_scl;
        tlv493d_mode_t mode;
        bool use_temp;
        bool int_enable;
    } tlv493d_io_conf_t;

    /**
     * @struct tlv493d_conf_t
     * @brief Struct for data and tlv943d configuration
     *
     * @var tlv493d_conf_t::dataX
     * data for x axis
     *
     * @var tlv493d_conf_t::dataY
     * data for y axis
     *
     * @var tlv493d_conf_t::dataZ
     * data for z axis
     *
     * @var tlv493d_conf_t::temperature
     * data for temperature
     *
     * @var tlv493d_conf_t::amount
     * amount data
     *
     * @var tlv493d_conf_t::azimuth
     * azimuth data
     *
     * @var tlv493d_conf_t::polar
     * polar data
     *
     * @var tlv493d_conf_t::tlv493d_conf
     * hold base config (see 'tlv493d_io_conf_t')
     */
    typedef struct
    {
        float dataX;
        float dataY;
        float dataZ;
        float temperature;
        float amount;
        float azimuth;
        float polar;
        tlv493d_io_conf_t tlv493d_conf;
    } tlv493d_conf_t;

    class TLV493D
    {
    public:
        TLV493D();
        /**
         * @brief  Init function with custom config
         *
         * @param config config for tlv493d
         *
         * @return ESP_OK when success.
         */
        esp_err_t init(tlv493d_conf_t *config);

        /**
         * @brief Function to start background task for read out data
         *
         * @return ESP_OK when success.
         */
        esp_err_t startBackgroundRead(void);

        /**
         * @brief Function to stop the background task
         *
         * @return ESP_OK when success.
         */
        esp_err_t stop(void);

        /**
         * @brief Function to enable or disable interrupts
         *
         * @note It will also update the variable in 'tlv493d_conf_t' config.
         *       It will also stop the background thread and inital the tlc493d new.
         *
         * @param enable  true for enable
         *
         * @return ESP_OK when success.
         *
         */
        esp_err_t setGetInterrupt(bool enable);

        /**
         * @brief Function to enable or disable temperature readout
         *
         * @note It will also update the variable in 'tlv493d_conf_t' config
         *       It will also stop the background thread and inital the tlc493d new.
         *
         * @param enable  true for enable
         *
         * @return ESP_OK when success.
         *
         */
        esp_err_t setGetTemperature(bool enable);

        /**
         * @brief Function to set the tlv493d to powerdown mode
         *
         * @param deep sleep for future use
         * @return ESP_OK when success.
         */
        esp_err_t sleep(bool deepsleep);

        /**
         * @brief Set the mode of tlv943d. Available modes can be found in 'tlv943d_mode_t' enum.
         * @note TODO: Save current mode in rtc memory to keep it for deep sleep.
         *
         * @param mode
         * @return ESP_OK when success.
         */
        esp_err_t setMode(tlv493d_mode_t mode);

        /**
         * @brief Function to read out values from tlv493d
         *
         * @note This is a blocking function. It try to read out data. If the tlv493d is not ready or new
         *       new data will present, it will immediately retry to read out. If it will not get new data
         *       after 'READOUT_MAX_TIMES' it will return as ESP_FAIL.
         *
         * @return ESP_OK when success.
         */
        esp_err_t update();

        /**
         * @brief Function to wake up tlv493d after sleep.
         * @note The tlv493d will be new initialed.
         *
         * @return ESP_OK when success.
         */
        esp_err_t wakeup(void);

        esp_err_t task_err;

    private:
        /*************
         * Functions *
         *************/
        esp_err_t init_bus();
        esp_err_t init_tlv();
        static void backgroundTask(void *pvParameter);
        static void IRAM_ATTR interruptHandler(void *pvParameter);
        esp_err_t reset();
        esp_err_t writeConfig();
        esp_err_t writeRegistry(uint8_t *data);
        esp_err_t readRegistry();
        int16_t calculateValue(uint8_t msb, uint8_t lsb, bool use_first_four);
        int16_t calculateTemperatur(uint8_t msb, uint8_t lsb);
        static void periodic_timer_callback(void *arg);

        /*************
         * Variables *
         *********** */
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(5, 2, 0)
        i2c_master_bus_handle_t bus_handle;
        i2c_master_dev_handle_t reset_dev_handle;
        i2c_master_dev_handle_t dev_handle;
#else
        i2c_config_t i2c_conf;
#endif
        esp_err_t err;
        tlv493d_conf_t *_config;
        tlv493d_io_conf_t *_io_conf;
        esp_timer_handle_t tlv493d_timer;
        uint8_t r_buffer[10] = {0};
        uint8_t w_buffer[10] = {0};
        uint64_t readoutPeriod;
        uint8_t readout_count;
        uint8_t failed_count;
    };

#ifdef __cplusplus
}
#endif