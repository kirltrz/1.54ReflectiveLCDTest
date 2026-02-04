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
int coord[286][2] = {
    {69, 11},   // 0 → 【1】
    {68, 11},   // 1 → 【2】
    {68, 11},   // 2 → 【3】
    {66, 11},   // 3 → 【4】
    {64, 11},   // 4 → 【5】
    {62, 11},   // 5 → 【6】
    {59, 11},   // 6 → 【7】
    {57, 11},   // 7 → 【8】
    {55, 11},   // 8 → 【9】
    {53, 12},   // 9 → 【10】
    {51, 14},   // 10 → 【11】
    {49, 15},   // 11 → 【12】
    {48, 16},   // 12 → 【13】
    {47, 18},   // 13 → 【14】
    {46, 21},   // 14 → 【15】
    {45, 23},   // 15 → 【16】
    {45, 24},   // 16 → 【17】
    {44, 27},   // 17 → 【18】
    {44, 29},   // 18 → 【19】
    {44, 32},   // 19 → 【20】
    {44, 33},   // 20 → 【21】
    {44, 35},   // 21 → 【22】
    {46, 37},   // 22 → 【23】
    {48, 38},   // 23 → 【24】
    {51, 39},   // 24 → 【25】
    {53, 41},   // 25 → 【26】
    {57, 43},   // 26 → 【27】
    {59, 44},   // 27 → 【28】
    {62, 45},   // 28 → 【29】
    {64, 48},   // 29 → 【30】
    {67, 49},   // 30 → 【31】
    {68, 52},   // 31 → 【32】
    {70, 54},   // 32 → 【33】
    {71, 58},   // 33 → 【34】
    {72, 61},   // 34 → 【35】
    {72, 64},   // 35 → 【36】
    {72, 69},   // 36 → 【37】
    {72, 72},   // 37 → 【38】
    {71, 75},   // 38 → 【39】
    {69, 78},   // 39 → 【40】
    {68, 79},   // 40 → 【41】
    {66, 81},   // 41 → 【42】
    {65, 83},   // 42 → 【43】
    {62, 84},   // 43 → 【44】
    {60, 84},   // 44 → 【45】
    {58, 84},   // 45 → 【46】
    {55, 84},   // 46 → 【47】
    {52, 84},   // 47 → 【48】
    {50, 84},   // 48 → 【49】
    {48, 84},   // 49 → 【50】
    {47, 84},   // 50 → 【51】
    {45, 84},   // 51 → 【52】
    {42, 81},   // 52 → 【53】
    {39, 80},   // 53 → 【54】
    {117, 11},  // 54 → 【55】
    {118, 13},  // 55 → 【56】
    {119, 18},  // 56 → 【57】
    {119, 22},  // 57 → 【58】
    {119, 26},  // 58 → 【59】
    {119, 32},  // 59 → 【60】
    {119, 36},  // 60 → 【61】
    {118, 41},  // 61 → 【62】
    {118, 48},  // 62 → 【63】
    {117, 54},  // 63 → 【64】
    {116, 59},  // 64 → 【65】
    {115, 66},  // 65 → 【66】
    {115, 69},  // 66 → 【67】
    {114, 72},  // 67 → 【68】
    {113, 76},  // 68 → 【69】
    {113, 80},  // 69 → 【70】
    {113, 83},  // 70 → 【71】
    {113, 85},  // 71 → 【72】
    {113, 86},  // 72 → 【73】
    {113, 87},  // 73 → 【74】
    {113, 87},  // 74 → 【75】
    {126, 15},  // 75 → 【76】
    {126, 15},  // 76 → 【77】
    {126, 15},  // 77 → 【78】
    {126, 15},  // 78 → 【79】
    {126, 15},  // 79 → 【80】
    {131, 15},  // 80 → 【81】
    {136, 15},  // 81 → 【82】
    {139, 15},  // 82 → 【83】
    {142, 16},  // 83 → 【84】
    {144, 18},  // 84 → 【85】
    {146, 18},  // 85 → 【86】
    {148, 21},  // 86 → 【87】
    {151, 24},  // 87 → 【88】
    {152, 27},  // 88 → 【89】
    {152, 29},  // 89 → 【90】
    {153, 31},  // 90 → 【91】
    {153, 34},  // 91 → 【92】
    {153, 36},  // 92 → 【93】
    {153, 40},  // 93 → 【94】
    {151, 41},  // 94 → 【95】
    {148, 44},  // 95 → 【96】
    {147, 46},  // 96 → 【97】
    {144, 48},  // 97 → 【98】
    {141, 49},  // 98 → 【99】
    {139, 50},  // 99 → 【100】
    {136, 52},  // 100 → 【101】
    {136, 52},  // 101 → 【102】
    {135, 52},  // 102 → 【103】
    {134, 52},  // 103 → 【104】
    {133, 52},  // 104 → 【105】
    {133, 52},  // 105 → 【106】
    {133, 52},  // 106 → 【107】
    {133, 52},  // 107 → 【108】
    {135, 52},  // 108 → 【109】
    {136, 52},  // 109 → 【110】
    {139, 52},  // 110 → 【111】
    {142, 52},  // 111 → 【112】
    {145, 52},  // 112 → 【113】
    {147, 53},  // 113 → 【114】
    {148, 55},  // 114 → 【115】
    {151, 58},  // 115 → 【116】
    {152, 61},  // 116 → 【117】
    {152, 65},  // 117 → 【118】
    {152, 67},  // 118 → 【119】
    {152, 70},  // 119 → 【120】
    {153, 74},  // 120 → 【121】
    {153, 75},  // 121 → 【122】
    {153, 78},  // 122 → 【123】
    {151, 80},  // 123 → 【124】
    {148, 83},  // 124 → 【125】
    {145, 84},  // 125 → 【126】
    {142, 87},  // 126 → 【127】
    {140, 87},  // 127 → 【128】
    {137, 88},  // 128 → 【129】
    {135, 89},  // 129 → 【130】
    {132, 89},  // 130 → 【131】
    {130, 89},  // 131 → 【132】
    {127, 89},  // 132 → 【133】
    {125, 89},  // 133 → 【134】
    {123, 89},  // 134 → 【135】
    {122, 89},  // 135 → 【136】
    {122, 89},  // 136 → 【137】
    {27, 122},  // 137 → 【138】
    {27, 122},  // 138 → 【139】
    {27, 126},  // 139 → 【140】
    {27, 134},  // 140 → 【141】
    {27, 140},  // 141 → 【142】
    {26, 145},  // 142 → 【143】
    {26, 150},  // 143 → 【144】
    {25, 155},  // 144 → 【145】
    {25, 160},  // 145 → 【146】
    {25, 165},  // 146 → 【147】
    {25, 171},  // 147 → 【148】
    {24, 175},  // 148 → 【149】
    {24, 178},  // 149 → 【150】
    {23, 182},  // 150 → 【151】
    {23, 184},  // 151 → 【152】
    {23, 185},  // 152 → 【153】
    {23, 185},  // 153 → 【154】
    {23, 185},  // 154 → 【155】
    {23, 185},  // 155 → 【156】
    {62, 133},  // 156 → 【157】
    {62, 133},  // 157 → 【158】
    {62, 133},  // 158 → 【159】
    {62, 133},  // 159 → 【160】
    {56, 139},  // 160 → 【161】
    {49, 143},  // 161 → 【162】
    {43, 147},  // 162 → 【163】
    {39, 150},  // 163 → 【164】
    {36, 152},  // 164 → 【165】
    {35, 152},  // 165 → 【166】
    {33, 154},  // 166 → 【167】
    {32, 155},  // 167 → 【168】
    {32, 155},  // 168 → 【169】
    {32, 155},  // 169 → 【170】
    {32, 155},  // 170 → 【171】
    {32, 155},  // 171 → 【172】
    {32, 155},  // 172 → 【173】
    {34, 155},  // 173 → 【174】
    {39, 160},  // 174 → 【175】
    {45, 164},  // 175 → 【176】
    {52, 170},  // 176 → 【177】
    {58, 174},  // 177 → 【178】
    {61, 176},  // 178 → 【179】
    {65, 178},  // 179 → 【180】
    {67, 179},  // 180 → 【181】
    {68, 180},  // 181 → 【182】
    {68, 180},  // 182 → 【183】
    {69, 181},  // 183 → 【184】
    {70, 181},  // 184 → 【185】
    {70, 181},  // 185 → 【186】
    {72, 182},  // 186 → 【187】
    {87, 135},  // 187 → 【188】
    {87, 135},  // 188 → 【189】
    {87, 135},  // 189 → 【190】
    {90, 133},  // 190 → 【191】
    {95, 132},  // 191 → 【192】
    {99, 132},  // 192 → 【193】
    {103, 131}, // 193 → 【194】
    {107, 131}, // 194 → 【195】
    {109, 131}, // 195 → 【196】
    {111, 131}, // 196 → 【197】
    {114, 131}, // 197 → 【198】
    {116, 131}, // 198 → 【199】
    {118, 131}, // 199 → 【200】
    {119, 131}, // 200 → 【201】
    {120, 131}, // 201 → 【202】
    {120, 131}, // 202 → 【203】
    {120, 131}, // 203 → 【204】
    {120, 131}, // 204 → 【205】
    {120, 131}, // 205 → 【206】
    {118, 137}, // 206 → 【207】
    {116, 141}, // 207 → 【208】
    {112, 147}, // 208 → 【209】
    {108, 152}, // 209 → 【210】
    {105, 155}, // 210 → 【211】
    {101, 159}, // 211 → 【212】
    {99, 162},  // 212 → 【213】
    {94, 167},  // 213 → 【214】
    {91, 171},  // 214 → 【215】
    {87, 175},  // 215 → 【216】
    {85, 178},  // 216 → 【217】
    {84, 178},  // 217 → 【218】
    {83, 180},  // 218 → 【219】
    {83, 180},  // 219 → 【220】
    {83, 180},  // 220 → 【221】
    {83, 180},  // 221 → 【222】
    {85, 180},  // 222 → 【223】
    {88, 181},  // 223 → 【224】
    {96, 181},  // 224 → 【225】
    {102, 181}, // 225 → 【226】
    {106, 181}, // 226 → 【227】
    {111, 181}, // 227 → 【228】
    {114, 181}, // 228 → 【229】
    {116, 181}, // 229 → 【230】
    {117, 181}, // 230 → 【231】
    {119, 181}, // 231 → 【232】
    {120, 181}, // 232 → 【233】
    {122, 181}, // 233 → 【234】
    {124, 181}, // 234 → 【235】
    {156, 124}, // 235 → 【236】
    {156, 124}, // 236 → 【237】
    {156, 124}, // 237 → 【238】
    {155, 129}, // 238 → 【239】
    {153, 138}, // 239 → 【240】
    {151, 143}, // 240 → 【241】
    {150, 148}, // 241 → 【242】
    {148, 152}, // 242 → 【243】
    {148, 155}, // 243 → 【244】
    {147, 158}, // 244 → 【245】
    {146, 162}, // 245 → 【246】
    {145, 165}, // 246 → 【247】
    {145, 169}, // 247 → 【248】
    {145, 171}, // 248 → 【249】
    {145, 173}, // 249 → 【250】
    {145, 174}, // 250 → 【251】
    {145, 174}, // 251 → 【252】
    {145, 174}, // 252 → 【253】
    {145, 174}, // 253 → 【254】
    {165, 129}, // 254 → 【255】
    {166, 134}, // 255 → 【256】
    {168, 140}, // 256 → 【257】
    {169, 144}, // 257 → 【258】
    {171, 150}, // 258 → 【259】
    {171, 153}, // 259 → 【260】
    {173, 156}, // 260 → 【261】
    {173, 160}, // 261 → 【262】
    {176, 166}, // 262 → 【263】
    {176, 169}, // 263 → 【264】
    {177, 171}, // 264 → 【265】
    {178, 174}, // 265 → 【266】
    {179, 177}, // 266 → 【267】
    {180, 179}, // 267 → 【268】
    {180, 179}, // 268 → 【269】
    {180, 179}, // 269 → 【270】
    {180, 179}, // 270 → 【271】
    {180, 179}, // 271 → 【272】
    {180, 179}, // 272 → 【273】
    {182, 176}, // 273 → 【274】
    {184, 171}, // 274 → 【275】
    {188, 161}, // 275 → 【276】
    {191, 152}, // 276 → 【277】
    {193, 145}, // 277 → 【278】
    {194, 138}, // 278 → 【279】
    {196, 132}, // 279 → 【280】
    {197, 127}, // 280 → 【281】
    {199, 124}, // 281 → 【282】
    {199, 121}, // 282 → 【283】
    {200, 118}, // 283 → 【284】
    {200, 118}, // 284 → 【285】
    {201, 118}  // 285 → 【286】
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

    printf("Display simple pattern");
    
    lcd_init();
    buffer_mutex = xSemaphoreCreateRecursiveMutex();
    assert(buffer_mutex);

    lcd_paint_brush(100,100,10,1);

    // Start background LCD refresh task (50Hz)
    xTaskCreate(lcd_task, "lcd_task", 4096, panel_handle, 5, NULL);

    // Start coordinate replay animation task (uses lcd_paint_brush, 20ms per point)
    xTaskCreate(lcd_coord_anim_task, "lcd_coord", 4096, NULL, 5, NULL);

    // Immediate initial refresh
    lcd_display(panel_handle);
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
