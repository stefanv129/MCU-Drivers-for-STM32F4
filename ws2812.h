#ifndef WS2812_H
#define WS2812_H

#ifndef GREEN
#define GREEN 0xFF0000
#endif

#ifndef RED
#define RED 0x00FF00
#endif

#ifndef BLUE 
#define BLUE 0x0000FF
#endif

#ifndef YELLOW 
#define YELLOW 0xFFFF00
#endif

#ifndef PURPLE 
#define PURPLE 0x00FFFF 
#endif


#ifndef LED_NR 
#define LED_NR 64 
#endif

#ifndef INFO_SIZE 
#define INFO_SIZE 24 
#endif

#include <mega164a.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

void ALL_LEDS(uint32_t color);

#endif
