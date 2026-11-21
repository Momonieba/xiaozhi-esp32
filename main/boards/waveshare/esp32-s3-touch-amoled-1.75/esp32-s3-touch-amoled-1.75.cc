// grokbot-175: GrokbotDisplay integrated (assets packed, local OTA)
#include "wifi_board.h"
#include "grokbot_display.h"
#include "display/lcd_display.h"
#include "esp_lcd_co5300.h"

#include "codecs/box_audio_codec.h"
#include "application.h"
#include "button.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "config.h"
#include "power_save_timer.h"
#include "axp2101.h"
#include "i2c_device.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include "esp_io_expander_tca9554.h"
#include "settings.h"

#include <esp_lcd_touch_cst9217.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>

#define TAG "WaveshareEsp32s3TouchAMOLED1inch75"

#include <esp_http_server.h>
#include <esp_wifi.h>


namespace Imu {
// QMI8658 六轴：I2C 地址 0x6A/0x6B，平放时 z 约 ±1000mg
static i2c_master_dev_handle_t device = nullptr;

static esp_err_t Read(uint8_t reg, uint8_t* data, size_t length) {
    if (device == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(device, &reg, 1, data, length, 50);
}

static esp_err_t Write(uint8_t reg, uint8_t value) {
    if (device == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(device, data, sizeof(data), 50);
}

static int16_t Decode(const uint8_t* data) {
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                                (static_cast<uint16_t>(data[1]) << 8));
}

esp_err_t Init(i2c_master_bus_handle_t bus) {
    if (device != nullptr) {
        return ESP_OK;
    }
    esp_err_t added = ESP_ERR_NOT_FOUND;
    for (uint8_t address : {uint8_t{0x6A}, uint8_t{0x6B}}) {
        if (i2c_master_probe(bus, address, 50) != ESP_OK) {
            continue;
        }
        i2c_device_config_t config = {};
        config.device_address = address;
        config.scl_speed_hz = 400000;
        added = i2c_master_bus_add_device(bus, &config, &device);
        break;
    }
    if (added != ESP_OK) {
        return added;
    }
    uint8_t id = 0;
    esp_err_t ret = Read(0x00, &id, 1);
    if (ret == ESP_OK && id != 0x05) {
        ret = ESP_ERR_NOT_FOUND;
    }
    if (ret == ESP_OK) {
        ret = Write(0x02, 0x60);  // CTRL1: 寄存器地址自增
    }
    if (ret == ESP_OK) {
        ret = Write(0x03, 0x25);  // CTRL2: ±8g, 250Hz
    }
    if (ret == ESP_OK) {
        ret = Write(0x08, 0x01);  // CTRL8: 使能加速度计
    }
    if (ret != ESP_OK && device != nullptr) {
        i2c_master_bus_rm_device(device);
        device = nullptr;
    }
    return ret;
}

bool ReadAccel(int& x, int& y, int& z) {
    if (device == nullptr) {
        return false;
    }
    uint8_t data[6] = {};
    if (Read(0x35, data, sizeof(data)) != ESP_OK) {
        return false;
    }
    x = static_cast<int>(Decode(data)) * 1000 / 4096;
    y = static_cast<int>(Decode(data + 2)) * 1000 / 4096;
    z = static_cast<int>(Decode(data + 4)) * 1000 / 4096;
    return true;
}
}  // namespace Imu



class Pmic : public Axp2101 {
public:
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        WriteReg(0x22, 0b110); // PWRON > OFFLEVEL as POWEROFF Source enable
        WriteReg(0x27, 0x10);  // hold 4s to power off

        // Disable All DCs but DC1
        WriteReg(0x80, 0x01);
        // Disable All LDOs
        WriteReg(0x90, 0x00);
        WriteReg(0x91, 0x00);

        // Set DC1 to 3.3V
        WriteReg(0x82, (3300 - 1500) / 100);

        // Set ALDO1 to 3.3V
        WriteReg(0x92, (3300 - 500) / 100);

        // Enable ALDO1(MIC)
        WriteReg(0x90, 0x01);

        WriteReg(0x64, 0x02); // CV charger voltage setting to 4.1V

        WriteReg(0x61, 0x02); // set Main battery precharge current to 50mA
        WriteReg(0x62, 0x08); // set Main battery charger current to 400mA ( 0x08-200mA, 0x09-300mA, 0x0A-400mA )
        WriteReg(0x63, 0x01); // set Main battery term charge current to 25mA
    }
};

#define LCD_OPCODE_WRITE_CMD (0x02ULL)
#define LCD_OPCODE_READ_CMD (0x03ULL)
#define LCD_OPCODE_WRITE_COLOR (0x32ULL)

static const co5300_lcd_init_cmd_t vendor_specific_init[] = {
    // set display to qspi mode
    {0xFE, (uint8_t[]){0x20}, 1, 0},
    {0x19, (uint8_t[]){0x10}, 1, 0},
    {0x1C, (uint8_t[]){0xA0}, 1, 0},

    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 0},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
    {0x63, (uint8_t[]){0xFF}, 1, 0},
    {0x2A, (uint8_t[]){0x00, 0x06, 0x01, 0xD7}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xD1}, 4, 600},
    {0x11, NULL, 0, 600},
    {0x29, NULL, 0, 0},
};

// 在waveshare_amoled_1_75类之前添加新的显示类
class CustomLcdDisplay : public GrokbotDisplay {
public:
    static void rounder_event_cb(lv_event_t* e) {
        lv_area_t* area = (lv_area_t* )lv_event_get_param(e);
        uint16_t x1 = area->x1;
        uint16_t x2 = area->x2;

        uint16_t y1 = area->y1;
        uint16_t y2 = area->y2;

        // round the start of coordinate down to the nearest 2M number
        area->x1 = (x1 >> 1) << 1;
        area->y1 = (y1 >> 1) << 1;
        // round the end of coordinate up to the nearest 2N+1 number
        area->x2 = ((x2 >> 1) << 1) + 1;
        area->y2 = ((y2 >> 1) << 1) + 1;
    }

    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle,
                     esp_lcd_panel_handle_t panel_handle,
                     int width,
                     int height,
                     int offset_x,
                     int offset_y,
                     bool mirror_x,
                     bool mirror_y,
                     bool swap_xy)
        : GrokbotDisplay(io_handle, panel_handle,
                        width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy,
                        GPIO_NUM_NC) {
        // Note: UI customization should be done in SetupUI(), not in constructor
        // to ensure lvgl objects are created before accessing them
    }

    virtual void SetupUI() override {
        // Call parent SetupUI() first to create all lvgl objects
        GrokbotDisplay::SetupUI();

        DisplayLockGuard lock(this);
        lv_obj_set_style_pad_left(status_bar_, LV_HOR_RES*  0.1, 0);
        lv_obj_set_style_pad_right(status_bar_, LV_HOR_RES*  0.1, 0);
        lv_display_add_event_cb(display_, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(esp_lcd_panel_io_handle_t panel_io) : Backlight(), panel_io_(panel_io) {}

protected:
    esp_lcd_panel_io_handle_t panel_io_;

    virtual void SetBrightnessImpl(uint8_t brightness) override {
        auto display = Board::GetInstance().GetDisplay();
        DisplayLockGuard lock(display);
        uint8_t data[1] = {((uint8_t)((255*  brightness) / 100))};
        int lcd_cmd = 0x51;
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_WRITE_CMD << 24;
        esp_lcd_panel_io_tx_param(panel_io_, lcd_cmd, &data, sizeof(data));
    }
};

class WaveshareEsp32s3TouchAMOLED1inch75 : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Pmic* pmic_ = nullptr;
    Button boot_button_;
    CustomLcdDisplay* display_;
    CustomBacklight* backlight_;
    esp_io_expander_handle_t io_expander = NULL;
    PowerSaveTimer* power_save_timer_;

    // 触发 1.75 单板构建（包含情绪优先显示修复），勿删
    void InitializePowerSaveTimer() {
        // 桌面常供电使用：禁用闲置 300 秒后的 pmic_->PowerOff()，只保留息屏。

        power_save_timer_ = new PowerSaveTimer(-1, 60, -1);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(0); });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness(); });
        power_save_timer_->OnShutdownRequest([this](){ 
            pmic_->PowerOff(); });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeTca9554(void) {
        esp_err_t ret = esp_io_expander_new_i2c_tca9554(i2c_bus_, I2C_ADDRESS, &io_expander);
        if (ret != ESP_OK)
            ESP_LOGE(TAG, "TCA9554 create returned error");
        ret = esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_4, IO_EXPANDER_INPUT);
        ESP_ERROR_CHECK(ret);
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.sclk_io_num = EXAMPLE_PIN_NUM_LCD_PCLK;
        buscfg.data0_io_num = EXAMPLE_PIN_NUM_LCD_DATA0;
        buscfg.data1_io_num = EXAMPLE_PIN_NUM_LCD_DATA1;
        buscfg.data2_io_num = EXAMPLE_PIN_NUM_LCD_DATA2;
        buscfg.data3_io_num = EXAMPLE_PIN_NUM_LCD_DATA3;
        buscfg.max_transfer_sz = DISPLAY_WIDTH*  DISPLAY_HEIGHT*  sizeof(uint16_t);
        buscfg.flags = SPICOMMON_BUSFLAG_QUAD;
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS;
        io_config.dc_gpio_num = GPIO_NUM_NC;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 32;
        io_config.lcd_param_bits = 8;
        io_config.flags.quad_mode = true;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        const co5300_vendor_config_t vendor_config = {
            .init_cmds = &vendor_specific_init[0],
            .init_cmds_size = sizeof(vendor_specific_init) / sizeof(co5300_lcd_init_cmd_t),
            .flags = {
                .use_qspi_interface = 1,
            }};

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = (void* )&vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_co5300(panel_io, &panel_config, &panel));
        esp_lcd_panel_set_gap(panel, 0x06, 0);
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, false);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);
        display_ = new CustomLcdDisplay(panel_io, panel,
                                        DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        backlight_ = new CustomBacklight(panel_io);
        backlight_->RestoreBrightness();
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH - 1,
            .y_max = DISPLAY_HEIGHT - 1,
            .rst_gpio_num = PIN_NUM_TOUCH_RST,
            .int_gpio_num = PIN_NUM_TOUCH_INT,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 0,
                .mirror_x = 1,
                .mirror_y = 1,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
        tp_io_config.scl_speed_hz = 400*  1000;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle));
        ESP_LOGI(TAG, "Initialize touch controller");
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst9217(tp_io_handle, &tp_cfg, &tp));
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };
        lvgl_port_add_touch(&touch_cfg);
        ESP_LOGI(TAG, "Touch panel initialized successfully");
    }

    // 初始化工具
    void InitializeTools() {
        auto &mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.system.reconfigure_wifi",
            "End this conversation and enter WiFi configuration mode.\n"
            "**CAUTION** You must ask the user to confirm this action.",
            PropertyList(), [this](const PropertyList& properties) {
                EnterWifiConfigMode();
                return true;
            });
    }

    static void ImuTaskEntry(void* arg) {
        static_cast<WaveshareEsp32s3TouchAMOLED1inch75*>(arg)->ImuTask();
    }

    // 手势 -> 表情：摇晃=困惑、倒扣=睡觉、拿起=醒来
    void ImuTask() {
        int px = 0, py = 0, pz = 0;
        Imu::ReadAccel(px, py, pz);
        int shake = 0;
        int64_t still_since = esp_timer_get_time();
        int64_t face_down_since = 0;
        int64_t last_emotion_us = 0;
        int64_t last_axis_log_us = 0;

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
            int x = 0, y = 0, z = 0;
            if (!Imu::ReadAccel(x, y, z)) {
                continue;
            }

            const int64_t now = esp_timer_get_time();
            const int delta = abs(x - px) + abs(y - py) + abs(z - pz);
            px = x;
            py = y;
            pz = z;

            // 每 60 秒打印一次当前姿态，便于校准倒扣方向
            if (now - last_axis_log_us > 60000000) {
                last_axis_log_us = now;
                ESP_LOGI(TAG, "IMU: x=%d y=%d z=%d mg", x, y, z);
            }

            // 对话中不插手显示
            if (Application::GetInstance().GetDeviceState() != kDeviceStateIdle) {
                shake = 0;
                face_down_since = 0;
                still_since = now;
                continue;
            }

            // 摇晃：连续多次加速度突变
            if (delta > 1300) {
                if (shake < 10) {
                    ++shake;
                }
            } else if (shake > 0) {
                --shake;
            }
            if (shake >= 3 && now - last_emotion_us > 2500000) {
                shake = 0;
                last_emotion_us = now;
                GetDisplay()->SetEmotion("confused");
                ESP_LOGI(TAG, "Gesture: shake -> confused");
            }

            // 倒扣/平放：z 持续反向超过 1.2 秒
            if (z < -650) {
                if (face_down_since == 0) {
                    face_down_since = now;
                } else if (now - face_down_since > 1200000 &&
                           now - last_emotion_us > 2500000) {
                    last_emotion_us = now;
                    GetDisplay()->SetEmotion("sleeping");
                    ESP_LOGI(TAG, "Gesture: face down -> sleeping");
                }
            } else {
                face_down_since = 0;
            }

            // 拿起：静止超过 3 秒后出现明显运动
            if (delta > 900) {
                if (now - still_since > 3000000 &&
                    now - last_emotion_us > 2500000) {
                    last_emotion_us = now;
                    GetDisplay()->SetEmotion("waking");
                    ESP_LOGI(TAG, "Gesture: picked up -> waking");
                }
                still_since = now;
            }
        }
    }

    void InitializeImu() {
        esp_err_t ret = Imu::Init(i2c_bus_);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "IMU init failed (%s), gesture disabled",
                     esp_err_to_name(ret));
            return;
        }
        ESP_LOGI(TAG, "IMU QMI8658 ready (gesture interact enabled)");
        xTaskCreate(&ImuTaskEntry, "imu_gesture", 4096, this, 3, nullptr);
    }

    httpd_handle_t control_httpd_ = nullptr;

    static esp_err_t EmotionHandler(httpd_req_t* req) {
        auto* self = static_cast<WaveshareEsp32s3TouchAMOLED1inch75*>(req->user_ctx);
        char query[96] = {};
        char name[40] = "happy";
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
            httpd_query_key_value(query, "name", name, sizeof(name));
        }
        const bool ok = self->GetDisplay()->SetEmotion(name);
        char resp[128];
        snprintf(resp, sizeof(resp), "{\"ok\":%s,\"emotion\":\"%s\"}",
                 ok ? "true" : "false", name);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        ESP_LOGI(TAG, "HTTP /api/emotion name=%s ok=%d", name, ok ? 1 : 0);
        return ESP_OK;
    }

    static esp_err_t StatusHandler(httpd_req_t* req) {
        auto* self = static_cast<WaveshareEsp32s3TouchAMOLED1inch75*>(req->user_ctx);
        int level = 0;
        bool charging = false;
        bool discharging = false;
        self->GetBatteryLevel(level, charging, discharging);
        wifi_ap_record_t ap = {};
        int rssi = 0;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            rssi = ap.rssi;
        }
        const int state = static_cast<int>(Application::GetInstance().GetDeviceState());
        char resp[256];
        snprintf(resp, sizeof(resp),
                 "{\"ok\":true,\"battery\":%d,\"charging\":%s,"
                 "\"rssi\":%d,\"state\":%d}",
                 level, charging ? "true" : "false", rssi, state);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    void InitializeControlHttp() {
        if (control_httpd_ != nullptr) {
            return;
        }
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = 8080;
        config.ctrl_port = 32768;
        config.stack_size = 5120;
        config.lru_purge_enable = true;
        if (httpd_start(&control_httpd_, &config) != ESP_OK) {
            ESP_LOGW(TAG, "control httpd failed to start on :8080");
            return;
        }

        static const httpd_uri_t emotion_uri = {
            .uri = "/api/emotion",
            .method = HTTP_GET,
            .handler = EmotionHandler,
            .user_ctx = this,
        };
        static const httpd_uri_t status_uri = {
            .uri = "/api/status",
            .method = HTTP_GET,
            .handler = StatusHandler,
            .user_ctx = this,
        };
        httpd_register_uri_handler(control_httpd_, &emotion_uri);
        httpd_register_uri_handler(control_httpd_, &status_uri);
        ESP_LOGI(TAG, "Control HTTP listening on :8080");
    }

public:
    WaveshareEsp32s3TouchAMOLED1inch75() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializePowerSaveTimer();
        InitializeCodecI2c();
        InitializeImu();
#if CONFIG_BOARD_TYPE_WAVESHARE_ESP32_S3_TOUCH_AMOLED_1_75
        InitializeTca9554();
#endif
        InitializeAxp2101();
        InitializeSpi();
        InitializeDisplay();
        InitializeTouch();
        InitializeButtons();
        InitializeTools();
        // LWIP 未就绪时 httpd_start 会触发 assert(tcpip Invalid mbox)，
        // 因此等网络连接建立后再启动控制接口。
        SetNetworkEventCallback([this](NetworkEvent event, const std::string& data) {
            (void)data;
            if (event == NetworkEvent::Connected) {
                InitializeControlHttp();
            }
        });
    }

    // WifiBoard::StartNetwork() 内部完成 esp_netif/LWIP 初始化，
    // 之后 httpd_start 才安全（构造函数里启动会 assert tcpip Invalid mbox）。
    virtual void StartNetwork() override {
        WifiBoard::StartNetwork();
        ESP_LOGI(TAG, "StartNetwork done, starting control HTTP");
        InitializeControlHttp();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7210_ADDR, 
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        return backlight_;
    }

    virtual bool GetBatteryLevel(int &level, bool &charging, bool &discharging) override {
        static bool last_discharging = false;
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        if (discharging != last_discharging)
        {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }

        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(WaveshareEsp32s3TouchAMOLED1inch75);
