#include "guiTask.h"

static const char *TAG = "guiTask.c";

#if CONFIG_EXAMPLE_LCD_I80_COLOR_IN_PSRAM
// PCLK frequency can't go too high as the limitation of PSRAM bandwidth
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (2 * 1000 * 1000)
#else
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (10 * 1000 * 1000)
#endif // CONFIG_EXAMPLE_LCD_I80_COLOR_IN_PSRAM

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_DATA0          9
#define EXAMPLE_PIN_NUM_DATA1          46
#define EXAMPLE_PIN_NUM_DATA2          3
#define EXAMPLE_PIN_NUM_DATA3          8
#define EXAMPLE_PIN_NUM_DATA4          18
#define EXAMPLE_PIN_NUM_DATA5          17
#define EXAMPLE_PIN_NUM_DATA6          16
#define EXAMPLE_PIN_NUM_DATA7          15
#if CONFIG_EXAMPLE_LCD_I80_BUS_WIDTH > 8
#define EXAMPLE_PIN_NUM_DATA8          14
#define EXAMPLE_PIN_NUM_DATA9          15
#define EXAMPLE_PIN_NUM_DATA10         16
#define EXAMPLE_PIN_NUM_DATA11         17
#define EXAMPLE_PIN_NUM_DATA12         18
#define EXAMPLE_PIN_NUM_DATA13         19
#define EXAMPLE_PIN_NUM_DATA14         20
#define EXAMPLE_PIN_NUM_DATA15         21
#endif
#define EXAMPLE_PIN_NUM_PCLK           47
#define EXAMPLE_PIN_NUM_CS             -1
#define EXAMPLE_PIN_NUM_DC             0
#define EXAMPLE_PIN_NUM_RST            4
#define EXAMPLE_PIN_NUM_BK_LIGHT       45

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              480
#define EXAMPLE_LCD_V_RES              320
// Bit number used to represent command and parameter
#if CONFIG_EXAMPLE_LCD_I80_CONTROLLER_ST7789
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8
#elif CONFIG_EXAMPLE_LCD_I80_CONTROLLER_NT35510
#define EXAMPLE_LCD_CMD_BITS           16
#define EXAMPLE_LCD_PARAM_BITS         16
#elif CONFIG_EXAMPLE_LCD_I80_CONTROLLER_ILI9341
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8
#endif

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
#define EXAMPLE_I2C_NUM                 0   // I2C number
#define EXAMPLE_I2C_SCL                 5
#define EXAMPLE_I2C_SDA                 6
#endif

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2

// Supported alignment: 16, 32, 64. A higher alignment can enables higher burst transfer size, thus a higher i80 bus throughput.
#define EXAMPLE_PSRAM_DATA_ALIGNMENT   64

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
static void example_lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}



// Task to be created.
void vGuiTask( void * pvParameters )
{
	vSemaphoreCreateBinary( xGuiSemaphore );
	static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Initialize Intel 8080 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = EXAMPLE_PIN_NUM_DC,
        .wr_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
#if CONFIG_EXAMPLE_LCD_I80_BUS_WIDTH > 8
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
#endif
        },
        .bus_width = CONFIG_EXAMPLE_LCD_I80_BUS_WIDTH,
        .max_transfer_bytes = EXAMPLE_LCD_H_RES * 100 * sizeof(uint16_t),
        .psram_trans_align = EXAMPLE_PSRAM_DATA_ALIGNMENT,
        .sram_trans_align = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = EXAMPLE_PIN_NUM_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .swap_color_bytes = !LV_COLOR_16_SWAP, // Swap can be done in LvGL (default) or DMA
        },
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
#if CONFIG_EXAMPLE_LCD_I80_CONTROLLER_ST7789
    ESP_LOGI(TAG, "Install LCD driver of st7789");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    // Set inversion, x/y coordinate order, x/y mirror according to your LCD module spec
    // the gap is LCD panel specific, even panels with the same driver IC, can have different gap value
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_set_gap(panel_handle, 0, 0);
    esp_lcd_panel_swap_xy(panel_handle, true);
#endif

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    ESP_LOGI(TAG, "Initialize I2C");

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_I2C_SDA,
        .scl_io_num = EXAMPLE_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    /* Initialize I2C */
    ESP_ERROR_CHECK(i2c_param_config(EXAMPLE_I2C_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(EXAMPLE_I2C_NUM, i2c_conf.mode, 0, 0, 0));


    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();

    ESP_LOGI(TAG, "Initialize touch IO (I2C)");

    /* Touch IO handle */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)EXAMPLE_I2C_NUM, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_V_RES,
        .y_max = EXAMPLE_LCD_H_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 1,
            .mirror_x = 1,
            .mirror_y = 0,
        },
    };


    /* Initialize touch */
    ESP_LOGI(TAG, "Initialize touch controller FT5X06");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp));

#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = NULL;
    lv_color_t *buf2 = NULL;
#if CONFIG_EXAMPLE_LCD_I80_COLOR_IN_PSRAM
    buf1 = heap_caps_aligned_alloc(EXAMPLE_PSRAM_DATA_ALIGNMENT, EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 70 * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
#endif
    assert(buf1);
#if CONFIG_EXAMPLE_LCD_I80_COLOR_IN_PSRAM
    buf2 = heap_caps_aligned_alloc(EXAMPLE_PSRAM_DATA_ALIGNMENT, EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 70 * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
#endif
    assert(buf2);
    ESP_LOGI(TAG, "buf1@%p, buf2@%p", buf1, buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 70);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
    static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);
#endif

    ESP_LOGI(TAG, "Start UI");
    ui_init();	
	
  for( ;; )
  {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        
       if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_timer_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
     
  }
}

// Function that creates a task.
void vStartGuiTask( void )
{
static uint8_t ucParameterToPass;
TaskHandle_t xGuiTaskHandle = NULL;

  // Create the task pinned to core 1, storing the handle.  Note that the passed parameter ucParameterToPass
  // must exist for the lifetime of the task, so in this case is declared static.  If it was just an
  // an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
  // the new task attempts to access it.
  xTaskCreatePinnedToCore( vGuiTask, "GUITASK", 4096, &ucParameterToPass, 10, &xGuiTaskHandle, 1 );
  configASSERT( xGuiTaskHandle );

  // Use the handle to delete the task.
  /*
  if( xGuiTaskHandle != NULL )
  {
     vTaskDelete( xGuiTaskHandle );
  }
  * */
}

