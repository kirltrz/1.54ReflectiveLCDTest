#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_st7305.h"

#define LCD_HOST SPI2_HOST

#define LCD_PIN_SCLK 6
#define LCD_PIN_SDI  7
#define LCD_PIN_RST  5
#define LCD_PIN_DC   4
#define LCD_PIN_CS   3
#define LCD_PIN_TE   2

#define ABS(X) (X >= 0 ? X : -X)
// 二维数组：coord[0]对应原【1】，coord[1]对应原【2】...coord[285]对应原【286】
int coord[138][2] = {{12, 84}, {12, 84}, {13, 84}, {17, 84}, {19, 84}, {22, 84}, {26, 84}, {32, 84}, {37, 84}, {42, 84}, {45, 84}, {47, 83}, {49, 83}, {49, 83}, {49, 83}, {29, 91}, {29, 91}, {29, 91}, {29, 93}, {29, 97}, {30, 101}, {30, 105}, {30, 111}, {30, 117}, {30, 123}, {30, 129}, {30, 132}, {30, 134}, {30, 135}, {30, 135}, {30, 135}, {30, 135}, {30, 135}, {99, 102}, {98, 101}, {97, 101}, {95, 100}, {95, 100}, {95, 100}, {94, 100}, {92, 100}, {90, 100}, {88, 101}, {85, 104}, {83, 107}, {81, 109}, {79, 114}, {79, 117}, {78, 121}, {78, 123}, {78, 127}, {78, 128}, {79, 129}, {80, 129}, {85, 128}, {89, 127}, {94, 125}, {101, 121}, {104, 118}, {107, 115}, {108, 113}, {109, 110}, {109, 107}, {109, 103}, {109, 101}, {109, 101}, {109, 101}, {109, 101}, {109, 103}, {109, 105}, {108, 109}, {108, 113}, {108, 119}, {108, 121}, {109, 125}, {109, 127}, {111, 129}, {113, 131}, {114, 131}, {115, 131}, {117, 131}, {118, 131}, {119, 131}, {141, 97}, {141, 97}, {142, 99}, {142, 105}, {142, 111}, {141, 116}, {140, 123}, {139, 127}, {139, 129}, {139, 131}, {139, 132}, {139, 132}, {139, 132}, {138, 132}, {138, 129}, {139, 124}, {142, 117}, {144, 109}, {145, 105}, {147, 101}, {147, 99}, {148, 97}, {149, 97}, {151, 97}, {153, 97}, {155, 97}, {156, 96}, {158, 96}, {159, 97}, {160, 97}, {161, 100}, {162, 103}, {163, 106}, {163, 110}, {163, 113}, {163, 117}, {163, 119}, {163, 122}, {163, 123}, {164, 125}, {164, 127}, {164, 128}, {164, 129}, {164, 131}, {164, 131}, {164, 133}, {164, 133}, {165, 134}, {165, 135}, {165, 135}, {191, 136}, {191, 136}, {191, 136}, {191, 136}, {191, 136}
};
// 绘制测试图案
// 画两条边缘线和一个中心点
bool is_rotated = 0; // 跟踪旋转状态
uint8_t *buffer = NULL;
static SemaphoreHandle_t buffer_mutex = NULL;
void lcd_init(void){
    buffer = heap_caps_malloc(ST7305_RESOLUTION_HOR * ST7305_RESOLUTION_VER / 8 +100, MALLOC_CAP_DMA);
    assert(buffer);
    memset(buffer,0x00,5100);
}
void lcd_draw_bit(int x, int y, bool enabled){
    if(x >= 200 || y >= 200 || x < 0 || y < 0) return;
    //x += 4;
    x = 199 - x;
    uint8_t real_x = x / 4;
    uint8_t real_y = y / 2;

    uint16_t byte_index = real_y * 51 + real_x;

    uint8_t y_group_index = y % 2 != 0;
    uint8_t x_group_index = x % 4;

    uint8_t bit_index = 7 - (x_group_index *2 + y_group_index);

    if(enabled)
        buffer[byte_index] |= (1 << bit_index);
    else
        buffer[byte_index] &= ~(1 << bit_index);
}
void lcd_display(esp_lcd_panel_handle_t panel_handle){
    if (buffer_mutex) xSemaphoreTakeRecursive(buffer_mutex, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, ST7305_RESOLUTION_HOR, ST7305_RESOLUTION_VER, buffer));
    if (buffer_mutex) xSemaphoreGiveRecursive(buffer_mutex);
}

// Background task: refresh display at fixed 50Hz (every 20ms)
void lcd_task(void* arg)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)arg;
    const TickType_t period = pdMS_TO_TICKS(20); // 50Hz
    TickType_t last_wake = xTaskGetTickCount();
    for (;;)
    {
        if (buffer_mutex) xSemaphoreTakeRecursive(buffer_mutex, portMAX_DELAY);
        esp_err_t err = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, ST7305_RESOLUTION_HOR, ST7305_RESOLUTION_VER, buffer);
        if (buffer_mutex) xSemaphoreGiveRecursive(buffer_mutex);

        (void)err; // suppress unused-warning if not used
        vTaskDelayUntil(&last_wake, period);
    }
}

// 绘制测试图案函数
void draw_test_pattern(uint8_t *buffer, bool rotated)
{
    if (buffer_mutex) xSemaphoreTakeRecursive(buffer_mutex, portMAX_DELAY);
    memset(buffer, 0, ST7305_RESOLUTION_HOR * ST7305_RESOLUTION_VER / 8 +100);

    int width = rotated ? ST7305_RESOLUTION_VER : ST7305_RESOLUTION_HOR;
    int height = rotated ? ST7305_RESOLUTION_HOR : ST7305_RESOLUTION_VER;

    // 画左边竖线
    for (int y = 0; y < height; y++)
    {
        int x = 10; // 距离左边10像素
        uint16_t byte_idx = (y >> 3) * width + x;
        uint8_t bit_pos = y & 0x07;
        if (byte_idx < width * ((height + 7) >> 3))
        {
            buffer[byte_idx] |= (1 << bit_pos);
        }
    }

    // 画右边竖线
    for (int y = 0; y < height; y++)
    {
        int x = width - 10; // 距离右边10像素
        uint16_t byte_idx = (y >> 3) * width + x;
        uint8_t bit_pos = y & 0x07;
        if (byte_idx < width * ((height + 7) >> 3))
        {
            buffer[byte_idx] |= (1 << bit_pos);
        }
    }

    // 在中心画一个5x5的方块
    int center_x = width / 2;
    int center_y = height / 2;
    for (int y = center_y - 2; y <= center_y + 2; y++)
    {
        for (int x = center_x - 2; x <= center_x + 2; x++)
        {
            uint16_t byte_idx = (y >> 3) * width + x;
            uint8_t bit_pos = y & 0x07;
            if (byte_idx < width * ((height + 7) >> 3))
            {
                buffer[byte_idx] |= (1 << bit_pos);
            }
        }
    }
}
// 无返回值，参数为LCD屏句柄，XY移动+边缘反弹，极简测试写法
void lcd_anim_test(esp_lcd_panel_handle_t panel_handle)
{
    lcd_init();                // LCD初始化，与原代码一致
    int block_x = 50;          // 方块左上角X坐标（初始居中）
    int block_y = 95;          // 方块左上角Y坐标（初始居中）
    int dir_x = 1;             // X方向：1=右移，-1=左移
    int dir_y = 1;             // Y方向：1=下移，-1=上移

    // 无限循环实现反弹动画
    while(1)
    {
        if (buffer_mutex) xSemaphoreTakeRecursive(buffer_mutex, portMAX_DELAY);
        /******** 1. 清屏：200*200全屏置0，避免拖尾 ********/
        for(int i=0; i<200; i++)
            for(int j=0; j<200; j++)
                lcd_draw_bit(j, i, 0);
        
        /******** 2. 绘制10*10方块（XY均为变量，随方向移动） ********/
        for(int i=0; i<10; i++)
            for(int j=0; j<10; j++)
                lcd_draw_bit(block_x+j, block_y+i, 1);

        if (buffer_mutex) xSemaphoreGiveRecursive(buffer_mutex);
        
        /******** 3. 等待下一帧（动画更新节律） ********/
        vTaskDelay(pdMS_TO_TICKS(10));
        
        /******** 4. 更新坐标：按方向移动（步长1像素） ********/
        block_x += dir_x;
        block_y += dir_y;
        
        /******** 5. 边界判断+反弹：碰到四周边缘反转对应方向 ********/
        // X轴反弹：左边界（x<=0）或右边界（x+方块宽>200），反转X方向
        if(block_x <= 0 || block_x + 10 > 200) dir_x = -dir_x;
        // Y轴反弹：上边界（y<=0）或下边界（y+方块高>200），反转Y方向
        if(block_y <= 0 || block_y + 10 > 200) dir_y = -dir_y;
    }
}

void lcd_paint_brush(uint8_t x, uint8_t y, uint8_t radius, bool enabled)
{
    // 半径为0时，仅绘制中心单个点，直接返回（避免无效循环）
    if (radius == 0)
    {
        if (buffer_mutex) xSemaphoreTakeRecursive(buffer_mutex, portMAX_DELAY);
        lcd_draw_bit(x, y, enabled);
        if (buffer_mutex) xSemaphoreGiveRecursive(buffer_mutex);
        return;
    }

    if (buffer_mutex) xSemaphoreTakeRecursive(buffer_mutex, portMAX_DELAY);

    // 关键：将无符号的x/y转成int，避免减法溢出（核心修复点1）
    int center_x = (int)x;
    int center_y = (int)y;
    int r = (int)radius;
    // 预计算半径平方，避免循环内重复计算，提升效率（优化点）
    int r_sq = r * r;

    // 修正循环边界为<=，补全所有像素（核心修复点2）
    // 循环变量用int，遍历范围[中心-r, 中心+r]
    for (int i = center_y - r; i <= center_y + r; i++)
    {
        for (int j = center_x - r; j <= center_x + r; j++)
        {
            // LCD屏幕边界校验，过滤无效坐标（核心修复点3）
            if (j < 0 || j > 199 || i < 0 || i > 199)
            {
                continue;
            }

            // 整数平方替代sqrt，无浮点运算（核心修复点4）
            // 圆方程：(j-中心x)² + (i-中心y)² < 半径² → 圆内点
            int dx = j - center_x;
            int dy = i - center_y;
            int dist_sq = dx * dx + dy * dy;

            // 距离平方小于半径平方，绘制该点（转uint8_t传给lcd_draw_bit）
            if (dist_sq < r_sq)
            {
                lcd_draw_bit((uint8_t)j, (uint8_t)i, enabled);
            }
        }
    }

    if (buffer_mutex) xSemaphoreGiveRecursive(buffer_mutex);
}

// Circle animation task: move a brush in a circular orbit using existing lcd_paint_brush
void lcd_circle_anim_task(void* arg)
{
    const int center_x = 100;
    const int center_y = 100;
    const int orbit_r = 60;
    const uint8_t brush_r = 10;
    float angle = 0.0f;
    float dtheta = 0.08f; // step per frame (signed for direction)
    int steps_per_rev = (int)(2 * M_PI / fabsf(dtheta) + 0.5f);
    int step_count = 0;
    bool draw_mode = true; // true = draw trail; false = erase trail

    // Clear screen before starting animation
    if (buffer_mutex) xSemaphoreTakeRecursive(buffer_mutex, portMAX_DELAY);
    for (int yy = 0; yy < 200; yy++)
        for (int xx = 0; xx < 200; xx++)
            lcd_draw_bit(xx, yy, 0);
    if (buffer_mutex) xSemaphoreGiveRecursive(buffer_mutex);

    while (1) {
        int x = center_x + (int)(cosf(angle) * orbit_r);
        int y = center_y + (int)(sinf(angle) * orbit_r);

        // Draw or erase the brush at current position (trail persists until erased in erase mode)
        if (draw_mode) {
            lcd_paint_brush((uint8_t)x, (uint8_t)y, brush_r, 1);
        } else {
            lcd_paint_brush((uint8_t)x, (uint8_t)y, brush_r, 0);
        }

        step_count++;
        if (step_count >= steps_per_rev) {
            step_count = 0;
            draw_mode = !draw_mode; // toggle draw/erase
            dtheta = -dtheta;      // reverse direction for '往复' effect
        }

        angle += dtheta;
        if (angle >= 2 * M_PI) angle -= 2 * M_PI;
        else if (angle < 0) angle += 2 * M_PI;

        // animation speed: ~25 FPS
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
// Replay recorded coordinates as a brush stroke: use coord[] points (20ms interval), radius 10px
void lcd_coord_anim_task(void* arg)
{
    const uint8_t brush_r = 10;
    const int total = sizeof(coord) / sizeof(coord[0]);
    int idx = 0;

    // Clear screen at start
    if (buffer_mutex) xSemaphoreTakeRecursive(buffer_mutex, portMAX_DELAY);
    for (int yy = 0; yy < 200; yy++)
        for (int xx = 0; xx < 200; xx++)
            lcd_draw_bit(xx, yy, 0);
    if (buffer_mutex) xSemaphoreGiveRecursive(buffer_mutex);

    while (1) {
        int x = coord[idx][0];
        int y = coord[idx][1];

        // Draw brush at recorded position (trail persists)
        lcd_paint_brush((uint8_t)x, (uint8_t)y, brush_r, 1);

        idx++;
        if (idx >= total) idx = 0; // loop replay

        vTaskDelay(pdMS_TO_TICKS(20)); // recorded at 20ms intervals
    }
}
void app_main(void)
{
    /*创建SPI总线*/
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_SCLK,
        .mosi_io_num = LCD_PIN_SDI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 200 * 200 / 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO)); // 启用 DMA

    /*从 SPI 总线分配一个 LCD IO 设备句柄*/
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num    = LCD_PIN_DC,
        .cs_gpio_num    = LCD_PIN_CS,
        .pclk_hz        = 40 * 1000 * 1000,
        .lcd_cmd_bits   = 8,
        .lcd_param_bits = 8,
        .spi_mode       = 0,
        .trans_queue_depth = 10,
    };
    // 将 LCD 连接到 SPI 总线
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 1,
    };
    // 为 ST7305 创建 LCD 面板句柄，并指定 SPI IO 设备句柄
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7305(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, is_rotated));
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    //printf("Display simple pattern");
    
    lcd_init();
    buffer_mutex = xSemaphoreCreateRecursiveMutex();
    assert(buffer_mutex);

    lcd_paint_brush(100,100,10,1);
    lcd_display(panel_handle);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Start background LCD refresh task (50Hz)
    xTaskCreate(lcd_task, "lcd_task", 4096, panel_handle, 5, NULL);

    // Start coordinate replay animation task (uses lcd_paint_brush, 20ms per point)
    xTaskCreate(lcd_coord_anim_task, "lcd_coord", 4096, NULL, 5, NULL);

    // Immediate initial refresh
    
    //lcd_anim_test(panel_handle);
    /*
    lcd_init();
    for(int i=0; i<200; i++){
        for(int j=0; j<200; j++){
            lcd_draw_bit(j,i,1);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        lcd_display(panel_handle);
    }
    */

    /*
    // 准备测试数据
    uint8_t *test_pattern = heap_caps_malloc(ST7305_RESOLUTION_HOR * ST7305_RESOLUTION_VER / 8 +100, MALLOC_CAP_DMA);
    assert(test_pattern);
    memset(test_pattern, 0x00, ST7305_RESOLUTION_HOR * ST7305_RESOLUTION_VER / 8 +100);
    // 初始填充并显示：累积字节扫描，完成一次后翻转配色并继续扫描
    size_t buf_size = ST7305_RESOLUTION_HOR * ST7305_RESOLUTION_VER / 8 +100;
    size_t idx = 0;
    bool scanned_is_ff = true; // true：已扫描字节为 0xFF，false：已扫描字节为 0x00
    while (1)
    {
        if (scanned_is_ff) {
            // 已扫描区域为 0xFF，未扫描为 0x00
            memset(test_pattern, 0x00, buf_size);
            memset(test_pattern, 0xFF, idx + 1);
        } else {
            // 已扫描区域为 0x00，未扫描为 0xFF（配色相反）
            memset(test_pattern, 0xFF, buf_size);
            memset(test_pattern, 0x00, idx + 1);
        }
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, ST7305_RESOLUTION_HOR, ST7305_RESOLUTION_VER, test_pattern));
        idx++;
        if (idx >= buf_size) {
            // 完成一次扫描，重置索引并翻转配色
            idx = 0;
            scanned_is_ff = !scanned_is_ff;
            vTaskDelay(pdMS_TO_TICKS(200)); // 稍作停顿以便观察配色翻转
        } else {
            vTaskDelay(pdMS_TO_TICKS(50)); // 平常的扫描速度
        }
    }

    // 释放资源
    free(test_pattern);*/
}
