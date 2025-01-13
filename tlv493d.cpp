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

#include "tlv493d.h"
#include "math.h"

static i2c_port_t _i2c_master_port;

/* keep status for background task*/
static bool thread_status;

/* variable to store config for deep sleep */
static RTC_DATA_ATTR tlv493d_io_conf_t ds_conf;

/* queue handler*/
static QueueHandle_t isr_evt_queue = NULL;

TLV493D::TLV493D()
{
    _i2c_master_port = (i2c_port_t)I2C_MASTER_NUM;
    err = ESP_OK;
    readoutPeriod = READOUT_LOW_PERIOD;
    io_conf = {};
}

esp_err_t TLV493D::init(tlv493d_conf_t *config)
{

    tlv493d_io_conf_t _conf = config->tlv493d_conf;
    _config = config;

    /* Install interrupt and setup type.
     * We don´t activate the interrupt yet.
     */
/*
    gpio_install_isr_service(0);
    gpio_set_intr_type((gpio_num_t)config->tlv493d_conf.pin_scl, GPIO_INTR_ANYEDGE);
    gpio_intr_disable((gpio_num_t)config->tlv493d_conf.pin_scl);
*/
    err = this->init_bus(&_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Error inital TLV439D");
        return err;
    }

    ESP_LOGI(MODUL_TLV, "Interrupt enabled on pin scl: %s", esp_err_to_name(err));
    isr_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    /* First we try to reset tlv493d */
    err = this->reset();
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Error reset TLV439D with err: %s", esp_err_to_name(err));
        return err;
    }
    /* Second initial tlv493d */
    err = this->init_tlv(&_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Error initialization of TLV439D failed with err: %s", esp_err_to_name(err));
        return err;
    }

    /* generate timer config*/
    esp_timer_create_args_t periodic_timer_args;
    periodic_timer_args.callback = &periodic_timer_callback;
    periodic_timer_args.arg = this;
    periodic_timer_args.name = "tlv493d timer";

    err = esp_timer_create(&periodic_timer_args, &tlv493d_timer);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Can´t generate timer handler with error: %s. Abort!", esp_err_to_name(err));
    }
    ESP_LOGI(MODUL_TLV, "TLV493D ready for read!");
    return ESP_OK;
}

esp_err_t TLV493D::startBackgroundRead(void)
{
    ESP_LOGI(MODUL_TLV, "Starting background task");

    /* start background thread for interrupt handling
     * note: this function is not implement at the moment until i find
     * a solution that the interrupt will coexistend with i2c
     */
    if (_config->tlv493d_conf.int_enable && _config->tlv493d_conf.mode == MASTERCONTROLLERMODE)
    {
        /* set thread status*/
        thread_status = true;
        xTaskCreate(backgroundTask, "TLV493D - Thread", 2096, this, 5, NULL);
    }
    /* start timer */
    else
    {
        ESP_LOGI(MODUL_TLV, "Starting timer for readout");
        esp_timer_start_periodic(tlv493d_timer, (uint64_t)readoutPeriod);
    }
    return ESP_OK;
}

esp_err_t TLV493D::stop(void)
{
    thread_status = false;
    if (_config->tlv493d_conf.int_enable && _config->tlv493d_conf.mode == MASTERCONTROLLERMODE)
    {
        gpio_isr_handler_remove((gpio_num_t)_config->tlv493d_conf.pin_scl);
        ESP_LOGD(MODUL_TLV, "Remove interrupt handler");
    }
    else
    {
        esp_timer_stop(tlv493d_timer);
    }
    return ESP_OK;
}

esp_err_t TLV493D::setGetInterrupt(bool enable)
{
    ESP_LOGI(MODUL_TLV, "%s interrupt pin", enable ? "Enable" : "Disable");
    tlv493d_io_conf_t _c = _config->tlv493d_conf;
    _c.int_enable = enable;
    thread_status = false;
    vTaskDelay(150);
    err = this->writeConfig(&_c);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Error setting interrupt output!");
        return err;
    }
    thread_status = true;
    err = startBackgroundRead();
    return err;
}

esp_err_t TLV493D::setGetTemperature(bool enable)
{
    ESP_LOGI(MODUL_TLV, "%s temperature measuring", enable ? "Enable" : "Disable");
    tlv493d_io_conf_t _c = _config->tlv493d_conf;
    _c.use_temp = enable;
    thread_status = false;
    vTaskDelay(150);
    err = this->writeConfig(&_c);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Error setting temperature read out!");
        return err;
    }
    thread_status = true;
    err = startBackgroundRead();
    return err;
}

esp_err_t TLV493D::sleep(bool deepsleep)
{
    /* TODO: Check what we have to do for isr handler */
    /* save config to slow rtc memory to keep it in case of deep sleep*/
    tlv493d_io_conf_t _c = _config->tlv493d_conf;
    ds_conf = _c;

    err = this->setMode(POWERDOWNMODE);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Failed to go into sleep mode!");
    }
    /* write it back to old mode to be */
    _c.mode = ds_conf.mode;
    return err;
}

esp_err_t TLV493D::setMode(tlv493d_mode_t mode)
{
    /* Check for unsupported modes */
    if (mode == MASTERCONTROLLERMODE || mode == MASTERCONTROLLERMODE_ULP)
    {
        ESP_LOGW(MODUL_TLV, "Selected mode is not supported!");
        return ESP_ERR_NOT_SUPPORTED;
    }
    tlv493d_io_conf_t _c = _config->tlv493d_conf;
    _c.mode = mode;
    thread_status = false;
    vTaskDelay(150);
    err = this->writeConfig(&_c);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Error setting mode!");
        return err;
    }
    /* If powerdownmode we don´t need to restart background task
     */
    if (_c.mode != POWERDOWNMODE)
    {
        thread_status = true;
        err = startBackgroundRead();
    }
    return err;
}

esp_err_t TLV493D::wakeup(void)
{
    /* Write config back */
    _config->tlv493d_conf = ds_conf;
    err = this->init(_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Can not wakeup tlv493d! Something went wrong.");
        return err;
    }
    err = this->startBackgroundRead();
    return err;
}

esp_err_t TLV493D::init_bus(tlv493d_io_conf_t *config)
{
    ESP_LOGI(MODUL_TLV, "Init I2C bus!");
    ESP_LOGI(MODUL_TLV, "Config -  SDA Pin Number: %d | SLC Pin Number: %d | I2C Address: 0x%02x", config->pin_sda, config->pin_scl, config->tlv_address);
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(5, 2, 9)

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = config->pin_sda,
        .scl_io_num = config->pin_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    err = i2c_new_master_bus(&bus_config, bus_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Error initial i2c bus with err: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_TLV493D_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle);
#else

#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = config->pin_sda;
    i2c_conf.scl_io_num = config->pin_scl;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_conf.clk_flags = 0;
#pragma GCC diagnostic pop

    i2c_param_config(_i2c_master_port, &i2c_conf);

    err = i2c_driver_install(_i2c_master_port, i2c_conf.mode, 0, 0, 0);

#endif
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Error initial i2c bus with err: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t TLV493D::init_tlv(tlv493d_io_conf_t *config)
{

    /* after this we read out the registry*/
    err = this->readRegistry(r_buffer);
    if (err == ESP_OK)
    {
        /* send config to tlv493d */
        err = this->writeConfig(config);
        if (err == ESP_OK)
        {
            ESP_LOGI(MODUL_TLV, "Inital done successfully!");
            return err;
        }
    }
    return err;
}

void TLV493D::backgroundTask(void *pvParameter)
{
    esp_err_t _err;
    uint32_t io_num;

    TLV493D *ptrTLV493D = static_cast<TLV493D *>(pvParameter);
    bool _int_enabled = true;

    /* mark interrupt as enabled */
    _int_enabled = true;

    /* install isr handler */
    gpio_set_intr_type((gpio_num_t)ptrTLV493D->_config->tlv493d_conf.pin_scl, GPIO_INTR_ANYEDGE);
    gpio_intr_enable((gpio_num_t)ptrTLV493D->_config->tlv493d_conf.pin_scl);
    gpio_install_isr_service(0);
    _err = gpio_isr_handler_add((gpio_num_t)ptrTLV493D->_config->tlv493d_conf.pin_scl, ptrTLV493D->interruptHandler, (void *)ptrTLV493D->_config->tlv493d_conf.pin_scl);

    if (_err != ESP_OK)
    {
        ESP_LOGE(MODUL_THREAD_TLV, "Can not add interrupt handler with error: %s. Aborting!", esp_err_to_name(_err));
        thread_status = false;
        _int_enabled = false;
    }

    while (thread_status)
    {
        /* TODO: Implement interrupt driven mode. See notes in document.
         *       Check for interrupt driven MASTERCONTROLLMODE otherwise we have to wait
         */
        /* We received interrupt */
        xQueueReceive(isr_evt_queue, &io_num, 10);
        if (io_num == ptrTLV493D->_config->tlv493d_conf.pin_scl)
        {
            /* remove interrupt handler */
            gpio_isr_handler_remove((gpio_num_t)ptrTLV493D->_config->tlv493d_conf.pin_scl);

            /* read out data */
            _err = ptrTLV493D->update();

            /* mark interrupt as disabled */
            _int_enabled = false;
        }
        if (!_int_enabled)
        {

            /* install isr handler */
            gpio_install_isr_service(0);
            gpio_isr_handler_add((gpio_num_t)ptrTLV493D->_config->tlv493d_conf.pin_scl, ptrTLV493D->interruptHandler, (void *)ptrTLV493D->_config->tlv493d_conf.pin_scl);

            /* mark interrupt as enabled */
            _int_enabled = true;
        }
    }

    ESP_LOGI(MODUL_THREAD_TLV, "Stopping background task");
    vTaskSuspend(NULL);
}

esp_err_t TLV493D::update()
{
    tlv493d_io_conf_t _conf = this->_config->tlv493d_conf;
    int _count = 0;

    err = this->readRegistry(this->r_buffer);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_THREAD_TLV, "Failed to read data with err: %s", esp_err_to_name(err));
        return err;
    }

    /* if count or power down flag not ready we will retry. Max retries defines 'READOUT_MAX_TIMES' */
    for (int r = 0; r < READOUT_MAX_TIMES; r++)
    {
        /* first we read out registry */
        err = this->readRegistry(this->r_buffer);
        if (err != ESP_OK)
        {
            ESP_LOGE(MODUL_THREAD_TLV, "Failed to read data with err: %s", esp_err_to_name(err));
            return err;
        }
        /* optional we check the count if it the same as before tlv493d has not
         * update values so we can skeep store it.
         * TODO: It seems to hang after a while so i disable this for the
         *       moment. Future investigation on this behavior.
         */
        if (CHECK_READOUT_COUNT)
        {
            if (((this->r_buffer[BANK_TEMP1] & 0x0C) >> 2) == _count)
            {
                ESP_LOGD(MODUL_THREAD_TLV, "No new data. Skip storing");
                ets_delay_us(20);
                err = ESP_ERR_INVALID_RESPONSE;
                continue;
            }
        }
        /* Let us check power down flag. According to datasheet it must
         * be 1 if it reading is complete, otherwise the tlv493d is running.
         */
        if ((this->r_buffer[BANK_Z2] & 0x08) == 0)
        {
            ESP_LOGD(MODUL_THREAD_TLV, "TLV493D is reading out. Skip storing");
            /* read out again as fast as possible */
            ets_delay_us(20);
            err = ESP_ERR_NOT_FINISHED;
            continue;
        }
        else
        {
            float _tempValue = 0.0;

            /* save new count number */
            _count = ((this->r_buffer[BANK_TEMP1] & 0x0C) >> 2);

            /* save data for x */
            _tempValue = this->calculateValue(this->r_buffer[BANK_X1], this->r_buffer[BANK_XY2], true) * TLV493D_AXE_MULT;
            this->_config->dataX = _tempValue;

            /* save data for y */
            _tempValue = this->calculateValue(this->r_buffer[BANK_Y1], this->r_buffer[BANK_XY2], false) * TLV493D_AXE_MULT;
            this->_config->dataY = _tempValue;

            /* save data for z */
            _tempValue = this->calculateValue(this->r_buffer[BANK_Z1], this->r_buffer[BANK_Z2], false) * TLV493D_AXE_MULT;
            this->_config->dataZ = _tempValue;

            /* save temp if it is enabled*/
            if (_conf.use_temp)
            {
                _tempValue = (this->calculateTemperatur(this->r_buffer[BANK_TEMP1], this->r_buffer[BANK_TEMP2]) - TLV493D_TEMP_OFFSET) * TLV493D_TEMP_MULT;
                this->_config->temperature = _tempValue;
            }

            /* save amount
             * TODO: check we need the multiplier or not
             */
            _tempValue = sqrt(pow(this->_config->dataX, 2) + pow(this->_config->dataY, 2) + pow(this->_config->dataZ, 2)) * TLV493D_AXE_MULT;
            this->_config->amount = _tempValue;

            /* save azimuth */
            _tempValue = atan2(this->_config->dataY, this->_config->dataX);
            this->_config->azimuth = _tempValue;

            /* save polar */
            _tempValue = atan2(this->_config->dataZ, sqrt(pow(this->_config->dataX, 2) + pow(this->_config->dataY, 2)));
            this->_config->polar = _tempValue;

            // vTaskDelay(100 / portTICK_PERIOD_MS);
            return err;
        }
    }
    /* if interrupt disabled we start the timer again */

    if (!_config->tlv493d_conf.int_enable && _config->tlv493d_conf.mode == MASTERCONTROLLERMODE)
    {
        esp_timer_start_once(tlv493d_timer, (uint64_t)readoutPeriod * 3);
        ESP_LOGD(MODUL_THREAD_TLV, "Start timer again!");
    }
    return err;
}

void IRAM_ATTR TLV493D::interruptHandler(void *pvParameter)
{
    /* we send the gpio pin to get shure that the interrupt come from the selected tlv493d */
    uint32_t gpio_num = (uint32_t)pvParameter;
    xQueueSendFromISR(isr_evt_queue, &gpio_num, NULL);
}

esp_err_t TLV493D::reset()
{

    /* we get no ack after write the reset address so we make the call by hand */

    uint8_t data = RESET_VALUE;

    err = i2c_master_write_to_device(_i2c_master_port, CONFIG_TLV493D_I2C_ADDRESS, &data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    /* reset */
    /*
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, TLV493D_RESET_ADDRESS | I2C_MASTER_WRITE, ACK);
    i2c_master_write(cmd, &data, 1, N_ACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Sending reset address failed with error: %s", esp_err_to_name(err));
    }
    i2c_cmd_link_delete(cmd);

    err = this->readRegistry(r_buffer);
    return err;
}

esp_err_t TLV493D::writeConfig(tlv493d_io_conf_t *config)
{

    /* first we set parity bit to 1 it will be calculated before we write*/
    w_buffer[REG_MOD1] = (1 << 7) | w_buffer[REG_MOD1];

    w_buffer[REG_MOD3] = (1 << 5) | w_buffer[REG_MOD3];
    ESP_LOGI(MODUL_TLV, "Set parity check bit as [enabled]");

    /* set interrupt flag (0 disabled 1 enabled) */
    w_buffer[REG_MOD1] = ((config->int_enable) << 2) | w_buffer[REG_MOD1];
    ESP_LOGI(MODUL_TLV, "Interrupt of tlv493d is [%s]", config->int_enable ? "enabled" : "disabled");

    /* set temperature flag (0 enabled 1 disabled) */
    w_buffer[REG_MOD3] = (!config->use_temp << 7) | w_buffer[REG_MOD3];
    ESP_LOGI(MODUL_TLV, "Readout temperature is [%s]", config->use_temp ? "enabled" : "disabled");

    /* set power mode flag */
    switch (config->mode)
    {
        /* TODO: Implement MASTERCONTROLLERMODE_ULP.
         *        At the moment we fall back to ULTRALOWPOWERMODE
         */
    case MASTERCONTROLLERMODE_ULP:
        ESP_LOGW(MODUL_TLV, "This mode is not supported yet. Switching to [ULTRALOWPOWERMODE]!");
        config->mode = LOWPOWERMODE;
        /* enabale low power mode */
        w_buffer[REG_MOD3] = w_buffer[REG_MOD3] & 0xBF;
        readoutPeriod = READOUT_LOW_PERIOD;
        ESP_LOGI(MODUL_TLV, "Powermode is set to [ultra low power mode]!");
        break;
    case POWERDOWNMODE:
        /* set fast mode, low power mode and interrupt out to 0 */
        w_buffer[REG_MOD1] = w_buffer[REG_MOD1] & 0xF8;
        /* set low power period to 100ms*/
        w_buffer[REG_MOD3] = w_buffer[REG_MOD3] & 0xBF;
        break;
    case FASTMODE:
        /* set fast mode bit in register 1
         * we also will set low power period to 12ms
         */
        w_buffer[REG_MOD1] = w_buffer[REG_MOD1] & 0xFE;
        w_buffer[REG_MOD1] = (1 << 1) | w_buffer[REG_MOD1];
        /* set low power period to 100ms*/
        w_buffer[REG_MOD3] = w_buffer[REG_MOD3] & 0xBF;
        
        /* in fastmode we have to disable temp readout and enable interrupt */
        config->use_temp = false;
        w_buffer[REG_MOD3] = ((!config->use_temp) << 7) | w_buffer[REG_MOD3];
        ESP_LOGI(MODUL_TLV, "Readout temperature is [%s]", config->use_temp ? "enabled" : "disabled");

        /* we also have to enable interrupt in fastmode*/
        config->int_enable = true;
        w_buffer[REG_MOD1] = ((config->int_enable) << 2) | w_buffer[REG_MOD1];
        ESP_LOGI(MODUL_TLV, "Readout temperature is [%s]", config->use_temp ? "enabled" : "disabled");

        /* set readout time*/
        readoutPeriod = READOUT_HIGH_PERIOD;
        ESP_LOGI(MODUL_TLV, "Powermode is set to [fastmode]!");
        break;
    case LOWPOWERMODE:
        /* set low power mode bit in register 1
         * we also will set low power period to 12ms
         */
        w_buffer[REG_MOD1] = 1 | w_buffer[REG_MOD1];
        w_buffer[REG_MOD3] = (1 << 6) | w_buffer[REG_MOD3];
        readoutPeriod = READOUT_NORMAL_PERIOD;
        ESP_LOGI(MODUL_TLV, "Powermode is set to [low power mode]!");
        break;
    case ULTRALOWPOWERMODE:
        /* set low power period to 100ms*/
        w_buffer[REG_MOD3] = w_buffer[REG_MOD3] & 0xBF;
        readoutPeriod = READOUT_LOW_PERIOD;
        ESP_LOGI(MODUL_TLV, "Powermode is set to [ultra low power mode]!");
        break;
    case MASTERCONTROLLERMODE:
        /* set master controller mode. fastmode = 1, lp_mode = 1, int_out = 0/1 */
        w_buffer[REG_MOD1] = (1 << 1) | w_buffer[REG_MOD1];
        w_buffer[REG_MOD1] = 1 | w_buffer[REG_MOD1];
        
        /* TODO: Implement this. Find a way to connect interrupt to sda pin
         *       Maybe this can be done with the gpio matrix. At the moment
         *       we disable interrupt for this mode
         */
        config->int_enable = true;
        w_buffer[REG_MOD1] = ((config->int_enable) << 2) | w_buffer[REG_MOD1];
        ESP_LOGI(MODUL_TLV, "Interrupt of tlv493d is [enabled]!");
        
        /* if we disable interrupt we must set low power period to 1
         *  otherwise parity check will fail and no data can read out
         */
        if (!config->int_enable)
        {
            w_buffer[REG_MOD3] = (1 << 6) | w_buffer[REG_MOD3];
        }
        readoutPeriod = READOUT_NORMAL_PERIOD;
        ESP_LOGI(MODUL_TLV, "Powermode is set to [master controller mode]!");
        break;
    default:
        break;
    }

    ESP_LOGI(MODUL_TLV, "Write register");
    ESP_LOGI(MODUL_TLV, "BANK_REG0\tBANK_REG1\tBANK_REG2\tBANK_REG3");
    ESP_LOGI(MODUL_TLV, "0x%02x\t\t0x%02x\t\t0x%02x\t\t0x%02x", w_buffer[REG_MOD0], w_buffer[REG_MOD1], w_buffer[REG_MOD2], w_buffer[REG_MOD3]);

    err = this->writeRegistry(w_buffer);

    /* to check if settings where accepted we will read back the parity check bit */
    ESP_LOGD(MODUL_TLV, "Read register");
    this->readRegistry(r_buffer);
    ESP_LOGD(MODUL_TLV, "BANK_X1\tBANK_Y1\t\tBANK_Z1\t\tBANK_T1\t\tBANK_XY2\tBANK_Z2\t\tBANK_T2\t\tBANK_RES0\tBANK_RES1\tBANK_RES2\t\n");
    ESP_LOGD(MODUL_TLV, "0x%02x\t\t0x%02x\t\t0x%02x\t\t0x%02x\t\t0x%02x\t\t0x%02x\t\t0x%02x\t\t0x%02x\t\t0x%02x\t\t0x%02x\t\t", r_buffer[BANK_X1], r_buffer[BANK_Y1], r_buffer[BANK_Z1], r_buffer[BANK_TEMP1], r_buffer[BANK_XY2], r_buffer[BANK_Z2], r_buffer[BANK_TEMP2], r_buffer[BANK_RES1], r_buffer[BANK_RES2], r_buffer[BANK_RES3]);

    if ((r_buffer[BANK_Z2] & 0x20) == 0)
    {
        ESP_LOGE(MODUL_TLV, "Writing settings to TLV493D failed. Parity check failed.");
        return ESP_FAIL;
    }
    ESP_LOGI(MODUL_TLV, "Parity check success!");

    return err;
}

esp_err_t TLV493D::writeRegistry(uint8_t *data)
{
    ESP_LOGD(MODUL_THREAD_TLV, "Try to write data to sensor");

    uint8_t i;
    uint8_t y = 0x00;
    // combine array to one byte first
    for (i = 0; i < sizeof(data); i++)
    {
        y ^= data[i];
    }
    // combine all bits of this byte
    y = y ^ (y >> 1);
    y = y ^ (y >> 2);
    y = y ^ (y >> 4);

    // parity is in the LSB of y
    data[REG_MOD1] = ((y & 0x01) << 7) | data[REG_MOD1];
    return i2c_master_write_to_device(_i2c_master_port, CONFIG_TLV493D_I2C_ADDRESS, data, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t TLV493D::readRegistry(uint8_t *data)
{

    ESP_LOGD(MODUL_THREAD_TLV, "Try to read data from sensor");
    // err = i2c_master_read_from_device(_i2c_master_port, CONFIG_TLV493D_I2C_ADDRESS, this->_config, 10, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    err = i2c_master_read_from_device(_i2c_master_port, CONFIG_TLV493D_I2C_ADDRESS, r_buffer, 10, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(MODUL_TLV, "Error reading data from slave. Error: %s", esp_err_to_name(err));
        return err;
    }

    w_buffer[REG_MOD0] = REG_MOD0;
    w_buffer[REG_MOD1] = (r_buffer[BANK_RES1] & 0x78); /* extract bit 6 to 3 */
    w_buffer[REG_MOD2] = r_buffer[BANK_RES2];
    w_buffer[REG_MOD3] = (r_buffer[BANK_RES3] & 0x1F); /* extract bit 4 to 0 */

    return err;
}

int16_t TLV493D::calculateValue(uint8_t msb, uint8_t lsb, bool use_first_four)
{
    int16_t value = 0x0000; // 16-bit signed integer we used for 12 bit
    value = msb << 8;

    /* what we need is bit 7 to 4 */
    if (use_first_four)
    {
        value |= (lsb & 0xF0);
    }
    /* else we need bit 4 to 0*/
    else
    {
        value |= (lsb & 0x0F) << 4;
    }
    value = value >> 4; // shift left so that value is a signed 12 bit integer
    return value;
}

int16_t TLV493D::calculateTemperatur(uint8_t msb, uint8_t lsb)
{
    int16_t value = 0x0000; // 16-bit signed integer we used for 12 bit

    value = (msb & 0xF0) << 8;
    value |= (lsb << 4);

    value = value >> 4; // shift left so that value is a signed 12 bit integer
    return value;
}

void TLV493D::periodic_timer_callback(void *arg)
{
    TLV493D *ptrTLV493D = static_cast<TLV493D *>(arg);
    ptrTLV493D->update();
}
