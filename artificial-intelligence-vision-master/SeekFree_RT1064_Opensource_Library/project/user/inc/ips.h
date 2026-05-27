#ifndef __IPS_H
#define __IPS_H

#include "zf_common_headfile.h"

void ips_init(void);
void ips200_draw_rectangle(int x1, int y1, int x2, int y2, uint16 color);
void ips200_fill_rectangle(int x1, int y1, int x2, int y2, uint16 color);
void map_init(void);
void update_car_position(void);

#endif
