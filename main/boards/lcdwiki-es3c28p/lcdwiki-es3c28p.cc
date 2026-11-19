#include "application.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "display/lcd_display.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "wifi_board.h"

#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_ili9341.h>
#include <esp_log.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>

#define TAG "LCDWikiES3C28P"

// ---- FT6336G touch controller (I2C addr 0x55) ----
#define TOUCH_I2C_ADDR 0x55
#define TOUCH_REG_INFO 0x0010
#define TOUCH_REG_MAX_TOUCHES 0x0009
#define TOUCH_REG_POINT0 0x0014

static i2c_master_dev_handle_t touch_dev_handle = NULL;
static uint8_t touch_max_points = 1;

static esp_err_t touch_i2c_read(uint16_t reg, uint8_t* data, size_t len) {
    if (touch_dev_handle == NULL)
        return ESP_FAIL;
    uint8_t wbuf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    return i2c_master_transmit_receive(touch_dev_handle, wbuf, 2, data, len, 100);
}

static void touchpad_read(lv_indev_t*, lv_indev_data_t* data) {
    static int16_t last_x = 0, last_y = 0;
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = LV_INDEV_STATE_RELEASED;

    if (touch_dev_handle == NULL)
        return;
    uint8_t info;
    if (touch_i2c_read(TOUCH_REG_INFO, &info, 1) != ESP_OK)
        return;
    if (info & 0x08) {
        uint8_t buf[7 * 5];
        if (touch_i2c_read(TOUCH_REG_POINT0, buf, 7 * touch_max_points) == ESP_OK) {
            for (int i = 0; i < touch_max_points; i++) {
                if (buf[i * 7] & 0x80) {
                    int16_t x = ((buf[i * 7] & 0x3F) << 8) | buf[i * 7 + 1];
                    int16_t y = ((buf[i * 7 + 2] & 0x3F) << 8) | buf[i * 7 + 3];
                    int16_t dx = x - last_x, dy = y - last_y;
                    if (dx > 3 || dx < -3 || dy > 3 || dy < -3) {
                        last_x = x;
                        last_y = y;
                    }
                    data->state = LV_INDEV_STATE_PRESSED;
                    break;
                }
            }
        }
    }
}

// ---- Display class (ILI9341 SPI, no QSPI rounder needed) ----
class ILI9341_Display : public LcdDisplay {
private:
    void LvglInit(bool mirror_x, bool mirror_y, bool swap_xy) {
        lv_init();
        lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
        port_cfg.task_priority = 1;
#if CONFIG_SOC_CPU_CORES_NUM > 1
        port_cfg.task_affinity = 1;
#endif
        lvgl_port_init(&port_cfg);

        const lvgl_port_display_cfg_t display_cfg = {
            .io_handle = panel_io_,
            .panel_handle = panel_,
            .control_handle = nullptr,
            .buffer_size = static_cast<uint32_t>(DISPLAY_BUFFER_SIZE),
            .double_buffer = false,
            .trans_size = 0,
            .hres = static_cast<uint32_t>(DISPLAY_WIDTH),
            .vres = static_cast<uint32_t>(DISPLAY_HEIGHT),
            .monochrome = false,
            .rotation =
                {
                    .swap_xy = swap_xy,
                    .mirror_x = mirror_x,
                    .mirror_y = mirror_y,
                },
            .color_format = LV_COLOR_FORMAT_RGB565,
            .flags =
                {
                    .buff_dma = 1,
                    .buff_spiram = 0,
                    .sw_rotate = 0,
                    .swap_bytes = 1,
                    .full_refresh = 0,
                    .direct_mode = 0,
                },
        };
        display_ = lvgl_port_add_disp(&display_cfg);
    }

public:
    ILI9341_Display(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width,
                    int height, bool mirror_x, bool mirror_y, bool swap_xy)
        : LcdDisplay(panel_io, panel, width, height) {
        LvglInit(mirror_x, mirror_y, swap_xy);
    }
};

class LCDWikiES3C28P_Board : public WifiBoard {
private:
    Button boot_button_;
    i2c_master_bus_handle_t i2c_bus_;
    ILI9341_Display* display_;

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = 1},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeSpiBus() {
        ESP_LOGI(TAG, "Initialize SPI bus");
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = DISPLAY_MISO_PIN;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_BUFFER_SIZE * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        ESP_LOGI(TAG, "Install ILI9341 LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
        panel_config.bits_per_pixel = 16;

        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

        display_ = new ILI9341_Display(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                       DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeTouch() {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = TOUCH_I2C_ADDR,
            .scl_speed_hz = 400000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &touch_dev_handle));

        gpio_config_t rst_cfg = {.pin_bit_mask = (1ULL << TOUCH_RST_PIN), .mode = GPIO_MODE_OUTPUT};
        gpio_config(&rst_cfg);
        gpio_set_level(TOUCH_RST_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(TOUCH_RST_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));

        uint8_t max_pts;
        if (touch_i2c_read(TOUCH_REG_MAX_TOUCHES, &max_pts, 1) == ESP_OK && max_pts > 0 &&
            max_pts <= 5) {
            touch_max_points = max_pts;
        }
        ESP_LOGI(TAG, "Touch max points: %d", touch_max_points);

        lv_indev_t* indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(indev, touchpad_read);
        lv_indev_set_disp(indev, lv_display_get_default());
        ESP_LOGI(TAG, "Touch initialized successfully");
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
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.system.reconfigure_wifi",
                           "End this conversation and enter WiFi configuration mode.\n"
                           "**CAUTION** You must ask the user to confirm this action.",
                           PropertyList(), [this](const PropertyList& properties) {
                               EnterWifiConfigMode();
                               return true;
                           });
    }

public:
    LCDWikiES3C28P_Board() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpiBus();
        InitializeLcdDisplay();
        InitializeTouch();
        InitializeButtons();
        InitializeTools();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR, true, true);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }
};

DECLARE_BOARD(LCDWikiES3C28P_Board);
