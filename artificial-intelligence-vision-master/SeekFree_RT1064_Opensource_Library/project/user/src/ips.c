#include "ips.h"

#define IPS200_TYPE (IPS200_TYPE_SPI)

#define MAP_REAL_WIDTH 3.2f  // 场地实际宽度 3.2m (对应屏幕 X 轴)
#define MAP_REAL_HEIGHT 2.4f // 场地实际高度 2.4m (对应屏幕 Y 轴)
#define GRID_REAL_SIZE 0.2f  // 每个网格实际尺寸 0.2m

#define MAP_COLUMNS (MAP_REAL_WIDTH / GRID_REAL_SIZE)      // 地图列
#define MAP_ROWS (MAP_REAL_HEIGHT / GRID_REAL_SIZE)         // 地图行
#define CELL_SIZE 12                        // 每个网格 12x12 像素
#define MAP_WIDTH (MAP_COLUMNS * CELL_SIZE) // 地图宽度 192 像素
#define MAP_HEIGHT (MAP_ROWS * CELL_SIZE)   // 地图高度 144 像素

// 地图起始坐标 使地图居中显示
#define MAP_START_X ((240 - MAP_WIDTH) / 2)  // 24
#define MAP_START_Y ((320 - MAP_HEIGHT) / 2) // 88

// 比例尺：1米 = 60像素
#define PIXELS_PER_METER (CELL_SIZE / GRID_REAL_SIZE) // 像素每米

int last_pixel_x = 0;
int last_pixel_y = 0;

extern float global_x;
extern float global_y;

void ips_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_init(IPS200_TYPE);
}

/**
 * @brief 在屏幕上绘制一个矩形
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param color
 */
void ips200_draw_rectangle(int x1, int y1, int x2, int y2, uint16 color)
{
    ips200_draw_line(x1, y1, x2, y1, color);
    ips200_draw_line(x1, y2, x2, y2, color);
    ips200_draw_line(x1, y1, x1, y2, color);
    ips200_draw_line(x2, y1, x2, y2, color);
}
/**
 * @brief 在屏幕上填充一个矩形
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param color
 */
void ips200_fill_rectangle(int x1, int y1, int x2, int y2, uint16 color)
{
    for (int i = y1; i <= y2; i++)
        ips200_draw_line(x1, i, x2, i, color);
}

/**
 * @brief 初始化地图
 *
 */
void map_init(void)
{
    // 画场地边界墙
    ips200_draw_rectangle(MAP_START_X - 4, MAP_START_Y - 4, MAP_START_X + MAP_WIDTH + 4, MAP_START_Y + MAP_HEIGHT + 4, RGB565_BLACK);
    ips200_draw_rectangle(MAP_START_X - 5, MAP_START_Y - 5, MAP_START_X + MAP_WIDTH + 5, MAP_START_Y + MAP_HEIGHT + 5, RGB565_BLACK);

    // 绘制网格线
    for (int i = 0; i <= MAP_COLUMNS; i++)
    {
        int x = MAP_START_X + i * CELL_SIZE;
        ips200_draw_line(x, MAP_START_Y, x, MAP_START_Y + MAP_HEIGHT, RGB565_GRAY);
    }
    for (int j = 0; j <= MAP_ROWS; j++)
    {
        int y = MAP_START_Y + j * CELL_SIZE;
        ips200_draw_line(MAP_START_X, y, MAP_START_X + MAP_WIDTH, y, RGB565_GRAY);
    }
}
/**
 * @brief 更新小车位置
 *
 */

void update_car_position(void)
{
    int current_pixel_x = MAP_START_X + MAP_WIDTH - (int)(global_x * PIXELS_PER_METER);
    int current_pixel_y = MAP_START_Y + (int)(global_y * PIXELS_PER_METER);

    // 边界软限位
    if (current_pixel_x < MAP_START_X + 4)
        current_pixel_x = MAP_START_X + 4;
    if (current_pixel_x > MAP_START_X + MAP_WIDTH - 4)
        current_pixel_x = MAP_START_X + MAP_WIDTH - 4;
    if (current_pixel_y < MAP_START_Y + 4)
        current_pixel_y = MAP_START_Y + 4;
    if (current_pixel_y > MAP_START_Y + MAP_HEIGHT - 4)
        current_pixel_y = MAP_START_Y + MAP_HEIGHT - 4;

    if (current_pixel_x != last_pixel_x || current_pixel_y != last_pixel_y)
    {
        int old_x1 = last_pixel_x - 3;
        int old_y1 = last_pixel_y - 3;
        int old_x2 = last_pixel_x + 3;
        int old_y2 = last_pixel_y + 3;

        if (last_pixel_x != 0)
        {
            // 擦除旧蓝块
            ips200_fill_rectangle(old_x1, old_y1, old_x2, old_y2, RGB565_BLACK);

            // 修复灰线
            for (int i = 0; i <= MAP_COLUMNS; i++)
            {
                int line_x = MAP_START_X + i * CELL_SIZE;
                if (line_x >= old_x1 && line_x <= old_x2)
                    ips200_draw_line(line_x, old_y1, line_x, old_y2, RGB565_GRAY);
            }
            for (int i = 0; i <= MAP_ROWS; i++)
            {
                int line_y = MAP_START_Y + i * CELL_SIZE;
                if (line_y >= old_y1 && line_y <= old_y2)
                    ips200_draw_line(old_x1, line_y, old_x2, line_y, RGB565_GRAY);
            }
        }

        // 画 7x7 蓝块
        ips200_fill_rectangle(current_pixel_x - 3, current_pixel_y - 3, current_pixel_x + 3, current_pixel_y + 3, RGB565_BLUE);

        last_pixel_x = current_pixel_x;
        last_pixel_y = current_pixel_y;
    }
}
