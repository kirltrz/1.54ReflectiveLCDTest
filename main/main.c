#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

#define DC_HIGH gpio_set_level(LCD_PIN_DC, 1)
#define DC_LOW  gpio_set_level(LCD_PIN_DC, 0)
#define CS_HIGH gpio_set_level(LCD_PIN_CS, 1)
#define CS_LOW  gpio_set_level(LCD_PIN_CS, 0)

// 绘制测试图案
// 画两条边缘线和一个中心点
bool is_rotated = 0; // 跟踪旋转状态
uint8_t *buffer = NULL;
void lcd_init(void){
    buffer = heap_caps_malloc(ST7305_RESOLUTION_HOR * ST7305_RESOLUTION_VER / 8 +100, MALLOC_CAP_DMA);
    assert(buffer);
    memset(buffer,0x00,5100);
}
void lcd_draw_bit(int x, int y, bool enabled){
    if(x>=200||y>=200) return;
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
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, ST7305_RESOLUTION_HOR, ST7305_RESOLUTION_VER, buffer));
}
// 绘制测试图案函数
void draw_test_pattern(uint8_t *buffer, bool rotated)
{
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
        /******** 1. 清屏：200*200全屏置0，避免拖尾 ********/
        for(int i=0; i<200; i++)
            for(int j=0; j<200; j++)
                lcd_draw_bit(j, i, 0);
        
        /******** 2. 绘制10*10方块（XY均为变量，随方向移动） ********/
        for(int i=0; i<10; i++)
            for(int j=0; j<10; j++)
                lcd_draw_bit(block_x+j, block_y+i, 1);
        
        /******** 3. 刷新显示+50ms延时（控制动画速度） ********/
        lcd_display(panel_handle);
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

    printf("Display simple pattern");
    
    lcd_anim_test(panel_handle);
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
